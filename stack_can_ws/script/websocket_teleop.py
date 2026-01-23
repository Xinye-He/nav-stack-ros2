#!/usr/bin/env python3

import asyncio
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from stack_msgs.msg import StackCommand
import websockets

DEBOUNCE_SEC = 0.15


class WebSocketTeleopBridge(Node):
    def __init__(self):
        super().__init__('websocket_teleop_bridge')

        # 发布和 teleop_key 一样的两个话题
        self.pub_cmd = self.create_publisher(StackCommand, '/stack_cmd/teleop', 1)
        self.pub_active = self.create_publisher(Bool, '/teleop_active', 1)

        # teleop 内部状态，与 teleop_key 一致
        self.teleop_active = False
        self.pre_speed_kmh = 0.0
        self.angle_deg = 0.0
        self.pick = False
        self.unload = False
        self.dump = False
        self.pick_action = False

        self.last_ts = {}

        self.get_logger().info(
            "WebSocket teleop bridge started. "
            "Listening on ws://0.0.0.0:9010\n"
            "Use JSON like {\"key\": \"t\"} / {\"key\": \"4\"} / {\"type\": \"cmd\", ...}"
        )

        # 初始发布一次
        self.publish_active()
        self.publish_cmd()

    # ------------------- 和 teleop_key 一致的工具函数 -------------------

    def debounce(self, key):
        now = time.time()
        last = self.last_ts.get(key, 0.0)
        if (now - last) < DEBOUNCE_SEC:
            return False
        self.last_ts[key] = now
        return True

    def publish_active(self):
        msg = Bool()
        msg.data = bool(self.teleop_active)
        self.pub_active.publish(msg)
        self.get_logger().info(f"teleop_active -> {self.teleop_active}")

    def publish_cmd(self):
        msg = StackCommand()
        msg.pre_speed_kmh = float(self.pre_speed_kmh)
        msg.angle_deg = float(self.angle_deg)
        msg.dist_to_target_m = 0.0  # teleop 不关心这个
        msg.pick = bool(self.pick)
        msg.unload = bool(self.unload)
        msg.dump = bool(self.dump)
        msg.pick_action = bool(self.pick_action)
        msg.valid = True
        self.pub_cmd.publish(msg)
        self.get_logger().info(
            f"CMD: speed={self.pre_speed_kmh} km/h, angle={self.angle_deg} deg, "
            f"pick={self.pick}, unload={self.unload}, dump={self.dump}, pick_action={self.pick_action}"
        )

    def handle_key(self, c: str):
        """复用 teleop_key 的按键语义，通过 WebSocket 传入 'key' 字段"""

        if not c:
            return
        c = c.lower()

        # 原 teleop_key 的 q 是退出节点，这里改为忽略（避免远程把节点杀掉）
        if c == 'q':
            self.get_logger().info("received 'q' key over websocket (ignored)")
            return

        # 切换 teleop_active
        if c == 't':
            if not self.debounce(c):
                return
            self.teleop_active = not self.teleop_active
            self.publish_active()
            self.publish_cmd()
            return

        # 速度档：0 / 4 / 8 km/h
        if c in ('0', '4', '8'):
            if not self.debounce(c):
                return
            if c == '0':
                self.pre_speed_kmh = 0.0
            elif c == '4':
                self.pre_speed_kmh = 4.0
            elif c == '8':
                self.pre_speed_kmh = 8.0
            self.publish_cmd()
            return

        # 角度档：a/d/q/e/c
        if c in ('a', 'd', 'q', 'e', 'c'):
            if not self.debounce(c):
                return
            if c == 'a':
                self.angle_deg = -10.0
            elif c == 'd':
                self.angle_deg = 10.0
            elif c == 'q':
                self.angle_deg = -20.0
            elif c == 'e':
                self.angle_deg = 20.0
            elif c == 'c':
                self.angle_deg = 0.0
            self.publish_cmd()
            return

        # 作业位：j/k/l/h
        if c in ('j', 'k', 'l', 'h'):
            if not self.debounce(c):
                return
            if c == 'j':
                self.dump = not self.dump
            elif c == 'k':
                self.pick = not self.pick
            elif c == 'l':
                self.unload = not self.unload
            elif c == 'h':
                self.pick_action = not self.pick_action
            self.publish_cmd()
            return

        self.get_logger().warn(f"Unknown key: {c}")

    # ------------------- WebSocket 处理 -------------------

    async def handle_client(self, websocket, path):
        client_addr = websocket.remote_address
        self.get_logger().info(f"Client connected from {client_addr}")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    self.get_logger().error("Received invalid JSON")
                    continue

                # 1) 推荐：用 key 模式，复用 teleop_key 按键逻辑
                #    例如：{"key": "t"}, {"key": "4"}, {"key": "a"}, {"key": "j"} ...
                if 'key' in data:
                    key = str(data['key'])[:1]  # 只取第一个字符
                    self.handle_key(key)
                    continue

                # 2) 可选：显式设置 teleop_active
                #    {"type": "active", "active": true}
                #    或 {"type": "active", "toggle": true}
                msg_type = data.get("type")
                if msg_type == "active":
                    if data.get("toggle", False):
                        self.teleop_active = not self.teleop_active
                    elif "active" in data:
                        self.teleop_active = bool(data["active"])
                    elif "value" in data:
                        self.teleop_active = bool(data["value"])
                    else:
                        self.get_logger().warn(f"No 'active' field in {data}")
                        continue
                    self.publish_active()
                    self.publish_cmd()
                    continue

                # 3) 可选：直接下发一条 StackCommand（部分字段也可以）
                #    {"type": "cmd", "pre_speed_kmh": 4, "angle_deg": 10, "pick": true, ...}
                if msg_type == "cmd":
                    updated = False
                    if "pre_speed_kmh" in data:
                        self.pre_speed_kmh = float(data["pre_speed_kmh"])
                        updated = True
                    if "angle_deg" in data:
                        self.angle_deg = float(data["angle_deg"])
                        updated = True
                    if "pick" in data:
                        self.pick = bool(data["pick"])
                        updated = True
                    if "unload" in data:
                        self.unload = bool(data["unload"])
                        updated = True
                    if "dump" in data:
                        self.dump = bool(data["dump"])
                        updated = True
                    if "pick_action" in data:
                        self.pick_action = bool(data["pick_action"])
                        updated = True

                    if updated:
                        self.publish_cmd()
                    else:
                        self.get_logger().warn(f"No recognized fields in cmd: {data}")
                    continue

                self.get_logger().warn(f"Unknown message format: {data}")

        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f"Client {client_addr} disconnected")
        except Exception as e:
            self.get_logger().error(f"Unexpected error: {e}")

    async def start_server(self):
        server = await websockets.serve(
            self.handle_client,
            "0.0.0.0",
            9010,
            ping_interval=20,
            ping_timeout=10
        )
        self.get_logger().info("WebSocket server listening on ws://0.0.0.0:9010")
        await server.wait_closed()


def main(args=None):
    rclpy.init(args=args)
    node = WebSocketTeleopBridge()

    # 只发布，不需要 spin executor
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.start_server())
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down WebSocket teleop bridge...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
