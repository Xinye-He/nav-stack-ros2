#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

try:
    import can
    HAVE_CAN = True
except Exception:
    HAVE_CAN = False

def parse_can_id(val):
    if isinstance(val, (int, float)):
        return int(val)
    s = str(val).strip()
    try:
        return int(s, 0)  # 支持“0x...”
    except Exception:
        return 0x18FED188

class CanSender(Node):
    def __init__(self):
        super().__init__('vcu_can_sender')

        self.declare_parameter('topic', '/vcu_remote_bytes')
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_id_status', '0x18FED188')
        self.declare_parameter('can_extended', True)

        self.topic = self.get_parameter('topic').value
        self.can_interface = self.get_parameter('can_interface').value
        self.can_id = parse_can_id(self.get_parameter('can_id_status').value)
        self.can_extended = bool(self.get_parameter('can_extended').value)

        self.bus = None
        if not HAVE_CAN:
            self.get_logger().error("python-can 未安装（pip install python-can）")
        else:
            try:
                self.bus = can.Bus(channel=self.can_interface, bustype='socketcan', fd=False)
                self.get_logger().info(f"CAN sender on {self.can_interface}, id=0x{self.can_id:X}, ext={self.can_extended}")
            except Exception as e:
                self.get_logger().error(f"Open CAN failed: {e}")

        self.create_subscription(UInt8MultiArray, self.topic, self.on_payload, 10)

    def on_payload(self, msg: UInt8MultiArray):
        if not self.bus:
            return
        data = msg.data
        if data is None or len(data) != 8:
            self.get_logger().warn("payload 长度必须为 8 字节")
            return
        try:
            frame = can.Message(arbitration_id=self.can_id,
                                data=bytes(bytearray([int(b) & 0xFF for b in data])),
                                is_extended_id=self.can_extended)
            self.bus.send(frame, timeout=0.001)
        except Exception as e:
            self.get_logger().warn(f"CAN send failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CanSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
