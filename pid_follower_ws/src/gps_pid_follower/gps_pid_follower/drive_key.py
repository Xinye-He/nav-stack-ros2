#!/usr/bin/env python3
import sys, os, termios, tty, select, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8, Bool

HELP = """
drive_key: press keys to control
  s -> RUN (drive=1, estop=0, remote=0)
  p -> PAUSE (drive=0)
  x -> ESTOP (drive=0, estop=1, remote=1)
  d -> TASK DONE (/task_done True)
  j -> TOGGLE DUMP (/dump_cmd Bool)
  k -> TOGGLE PICK  (/pick_cmd 0<->1)
  l -> TOGGLE UNLOAD (/unload_cmd 0<->1)
  q -> quit
"""

DEBOUNCE_SEC = 0.15  # 按键防抖，避免连发导致多次切换

class DriveKey(Node):
    def __init__(self):
        super().__init__('drive_key')

        # 原有发布
        self.pub_drive = self.create_publisher(UInt8, '/drive_cmd', 1)
        self.pub_done  = self.create_publisher(Bool,  '/task_done', 1)

        # 新增发布
        self.pub_dump   = self.create_publisher(Bool,  '/dump_cmd',   1)
        self.pub_pick   = self.create_publisher(UInt8, '/pick_cmd',   1)
        self.pub_unload = self.create_publisher(UInt8, '/unload_cmd', 1)

        # 状态（两档：0/1）
        self.dump_state = False
        self.pick_state = 0  # 0/1
        self.unload_state = 0  # 0/1

        # 防抖时间戳
        self.last_ts = {'j': 0.0, 'k': 0.0, 'l': 0.0}

        self.get_logger().info(HELP)

        # 打开 TTY，进入原始模式读取单字节
        self.fd = None
        self.raw_mode = False
        try:
            self.fd = os.open('/dev/tty', os.O_RDWR | os.O_NOCTTY)
        except Exception:
            self.fd = sys.stdin.fileno()
        self.has_tty = os.isatty(self.fd)

        # 初始广播一次当前状态（可选）
        self._publish_dump()
        self._publish_pick()
        self._publish_unload()

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

    # --- 原有指令 ---
    def send_drive(self, v:int):
        self.pub_drive.publish(UInt8(data=int(v)))

    def send_done(self):
        self.pub_done.publish(Bool(data=True))

    # --- 新增三键的发布 ---
    def _publish_dump(self):
        self.pub_dump.publish(Bool(data=bool(self.dump_state)))

    def _publish_pick(self):
        self.pub_pick.publish(UInt8(data=int(self.pick_state)))

    def _publish_unload(self):
        self.pub_unload.publish(UInt8(data=int(self.unload_state)))

    # --- 统一按键处理（带防抖）---
    def handle_key(self, c: str):
        if not c:
            return
        c = c.lower()
        now = time.time()

        if c == 's':
            self.get_logger().info("RUN")
            self.send_drive(1)
        elif c == 'p':
            self.get_logger().info("PAUSE")
            self.send_drive(0)
        elif c == 'x':
            self.get_logger().warn("ESTOP")
            self.send_drive(2)
        elif c == 'd':
            self.get_logger().info("TASK DONE")
            self.send_done()
        elif c == 'q':
            self.get_logger().info("quit")
            raise KeyboardInterrupt

        elif c in ('j', 'k', 'l'):
            # 防抖
            last = self.last_ts.get(c, 0.0)
            if (now - last) < DEBOUNCE_SEC:
                return
            self.last_ts[c] = now

            if c == 'j':
                # TOGGLE dump
                self.dump_state = not self.dump_state
                self._publish_dump()
                self.get_logger().info(f"[J] dump_state -> {self.dump_state}")
            elif c == 'k':
                # TOGGLE pick 0<->1
                self.pick_state = 1 - self.pick_state
                self._publish_pick()
                self.get_logger().info(f"[K] pick_state -> {self.pick_state}")
            elif c == 'l':
                # TOGGLE unload 0<->1
                self.unload_state = 1 - self.unload_state
                self._publish_unload()
                self.get_logger().info(f"[L] unload_state -> {self.unload_state}")

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
                    try:
                        node.handle_key(c)
                    except KeyboardInterrupt:
                        break
            else:
                line = sys.stdin.readline()
                if line:
                    cmd = line.strip().lower()
                    try:
                        node.handle_key(cmd[:1] if cmd else '')
                    except KeyboardInterrupt:
                        break
            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.disable_raw()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
