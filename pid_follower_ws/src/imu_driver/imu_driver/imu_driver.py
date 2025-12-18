#!/usr/bin/env python3

import time
import math
import serial
import struct
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


# ================== 全局解析函数 ==================
def hex_to_short(raw_data):
    """将字节流转换为四个有符号短整型"""
    return list(struct.unpack("hhhh", bytearray(raw_data)))


def check_sum(data_buf, check_byte):
    """校验和：前10字节和 % 256 == 第10字节"""
    return sum(data_buf) & 0xff == check_byte


# ================== IMU 驱动节点 ==================
class IMUDriverNode(Node):
    def __init__(self, port_name, baud_rate=9600):
        super().__init__('imu_driver_node')

        # 初始化 IMU 消息
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'imu_link'

        # 创建发布器
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        # 创建 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 传感器数据存储
        self.acceleration = [0.0, 0.0, 0.0]
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.angle_degree = [0.0, 0.0, 0.0]
        self.has_orientation = False

        # 串口参数
        self.port_name = port_name
        self.baud_rate = baud_rate

        # 启动驱动线程
        self.driver_thread = threading.Thread(target=self.driver_loop, daemon=True)
        self.driver_thread.start()

        # 广播 odom → base_link (1Hz 足够)
        self.create_timer(10.0, self.publish_odom_to_base_link)

    def publish_odom_to_base_link(self):
        """广播 odom → base_link 的静态变换"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def driver_loop(self):
        """主循环：读串口并解析"""
        try:
            self.wt_imu = serial.Serial(port=self.port_name, baudrate=self.baud_rate, timeout=0.5)
            if self.wt_imu.is_open:
                self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port_name}: {e}")
            return

        buff = {}
        key = 0

        while rclpy.ok():
            try:
                count = self.wt_imu.inWaiting()
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

            if count > 0:
                data = self.wt_imu.read(count)
                for b in data:
                    buff[key] = b
                    key += 1

                    # 检查帧头
                    if buff[0] != 0x55:
                        key = 0
                        buff = {}
                        continue

                    # 至少 11 字节才能校验
                    if key < 11:
                        continue

                    data_buf = list(buff.values())
                    packet_type = buff[1]

                    # 处理不同包类型
                    if packet_type == 0x51:  # 加速度
                        if check_sum(data_buf[0:10], data_buf[10]):
                            raw = hex_to_short(data_buf[2:10])
                            self.acceleration = [raw[i] / 32768.0 * 16 * 9.8 for i in range(3)]
                            self.publish_imu()  # 收到数据就发布
                        else:
                            self.get_logger().warn("0x51 Checksum failed")

                    elif packet_type == 0x52:  # 角速度
                        if check_sum(data_buf[0:10], data_buf[10]):
                            raw = hex_to_short(data_buf[2:10])
                            self.angular_velocity = [raw[i] / 32768.0 * 2000 * math.pi / 180 for i in range(3)]
                            self.publish_imu()  # 收到数据就发布
                        else:
                            self.get_logger().warn("0x52 Checksum failed")

                    elif packet_type == 0x53:  # 欧拉角
                        if check_sum(data_buf[0:10], data_buf[10]):
                            raw = hex_to_short(data_buf[2:10])
                            self.angle_degree = [raw[i] / 32768.0 * 180 for i in range(3)]
                            self.has_orientation = True
                            self.publish_imu()  # 收到姿态就发布
                        else:
                            self.get_logger().warn("0x53 Checksum failed")

                    elif packet_type == 0x54:  # 磁力计（可选）
                        if check_sum(data_buf[0:10], data_buf[10]):
                            raw = hex_to_short(data_buf[2:10])
                            # 可扩展 magnetometer 发布
                            pass
                        else:
                            self.get_logger().warn("0x54 Checksum failed")

                    else:
                        self.get_logger().debug(f"Unknown packet type: 0x{packet_type:02x}")

                    # 清空缓冲区
                    buff = {}
                    key = 0

    def publish_imu(self):
        """发布 IMU 消息，并同步广播 base_link → imu_link 的 TF"""
        now = self.get_clock().now()

        # 设置 IMU 消息头
        self.imu_msg.header.stamp = now.to_msg()

        # === 线性加速度 ===
        a = self.acceleration
        self.imu_msg.linear_acceleration.x = a[0]
        self.imu_msg.linear_acceleration.y = a[1]
        self.imu_msg.linear_acceleration.z = a[2]

        # === 角速度 ===
        w = self.angular_velocity
        self.imu_msg.angular_velocity.x = w[0]
        self.imu_msg.angular_velocity.y = w[1]
        self.imu_msg.angular_velocity.z = w[2]

        # === 四元数 ===
        if self.has_orientation:
            roll = math.radians(self.angle_degree[0])
            pitch = math.radians(self.angle_degree[1])
            yaw = math.radians(self.angle_degree[2])
            qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
            self.imu_msg.orientation.x = qx
            self.imu_msg.orientation.y = qy
            self.imu_msg.orientation.z = qz
            self.imu_msg.orientation.w = qw
        else:
            self.imu_msg.orientation.x = 0.0
            self.imu_msg.orientation.y = 0.0
            self.imu_msg.orientation.z = 0.0
            self.imu_msg.orientation.w = 1.0

        # 发布 IMU 消息
        self.imu_pub.publish(self.imu_msg)

        # === 关键：同步广播 base_link → imu_link 变换 ===
        # 使用与 IMU 消息相同的时间戳
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """欧拉角转四元数"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return [qx, qy, qz, qw]


def main():
    rclpy.init()
    node = IMUDriverNode(port_name='/dev/imu_usb', baud_rate=9600)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
