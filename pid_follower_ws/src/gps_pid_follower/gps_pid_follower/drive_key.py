#!/usr/bin/env python3
import sys, os, termios, tty, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool

HELP = """
drive_key: press keys to control
  s -> RUN (drive=1, estop=0, remote=0)
  p -> PAUSE (drive=0)
  x -> ESTOP (drive=0, estop=1, remote=1)
  d -> TASK DONE (/task_done True)
  q -> quit
"""

class DriveKey(Node):
    def __init__(self):
        super().__init__('drive_key')
        self.pub_drive = self.create_publisher(UInt8, '/drive_cmd', 1)
        self.pub_done  = self.create_publisher(Bool,  '/task_done', 1)
        self.get_logger().info(HELP)
        self.fd = None
        self.raw_mode = False
        try:
            self.fd = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
        except Exception:
            self.fd = sys.stdin.fileno()
        self.has_tty = os.isatty(self.fd)

    def enable_raw(self):
        if not self.has_tty:
            return False
        try:
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.raw_mode = True
            return True
        except Exception as e:
            self.get_logger().warn(f"no TTY raw mode: {e}")
            return False

    def disable_raw(self):
        if self.raw_mode:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
            self.raw_mode = False

    def send_drive(self, v:int):
        self.pub_drive.publish(UInt8(data=int(v)))

    def send_done(self):
        self.pub_done.publish(Bool(data=True))

def getch_nonblock(fd, timeout=0.05):
    dr, _, _ = select.select([fd], [], [], timeout)
    if dr:
        try:
            return os.read(fd, 1).decode(errors='ignore')
        except Exception:
            return None
    return None

def main(args=None):
    rclpy.init(args=args)
    node = DriveKey()
    raw_ok = node.enable_raw()
    try:
        node.get_logger().info("drive_key started (raw_tty=%s)" % raw_ok)
        while rclpy.ok():
            if raw_ok:
                c = getch_nonblock(node.fd, 0.05)
                if c:
                    c = c.lower()
                    if c == 's':
                        node.get_logger().info("RUN")
                        node.send_drive(1)
                    elif c == 'p':
                        node.get_logger().info("PAUSE")
                        node.send_drive(0)
                    elif c == 'x':
                        node.get_logger().warn("ESTOP")
                        node.send_drive(2)
                    elif c == 'd':
                        node.get_logger().info("TASK DONE")
                        node.send_done()
                    elif c == 'q':
                        node.get_logger().info("quit")
                        break
            else:
                line = sys.stdin.readline()
                if line:
                    cmd = line.strip().lower()
                    if cmd == 's':
                        node.get_logger().info("RUN");   node.send_drive(1)
                    elif cmd == 'p':
                        node.get_logger().info("PAUSE"); node.send_drive(0)
                    elif cmd == 'x':
                        node.get_logger().warn("ESTOP"); node.send_drive(2)
                    elif cmd == 'd':
                        node.get_logger().info("TASK DONE"); node.send_done()
                    elif cmd == 'q':
                        node.get_logger().info("quit"); break
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.disable_raw()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
