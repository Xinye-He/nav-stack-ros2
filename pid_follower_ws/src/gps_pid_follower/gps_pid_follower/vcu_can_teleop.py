#!/usr/bin/env python3
import sys, time, math, os, select
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

# 优先用 keyboard 库（支持按键释放），TTY 仅作退化（不推荐）
try:
    import keyboard  # pip install keyboard（需要root）
    HAVE_KBD = True
except Exception:
    HAVE_KBD = False

try:
    import termios, tty
    HAVE_TTY = sys.stdin.isatty()
except Exception:
    HAVE_TTY = False

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

class VcuCanTeleop(Node):
    def __init__(self):
        super().__init__('vcu_can_teleop')

        # 参数
        self.declare_parameter('publish_topic', '/vcu_remote_bytes')
        self.declare_parameter('period_ms', 200)
        self.declare_parameter('speed_offset_kmh', -50.0)       # 预速度编码：km/h = raw*res + offset
        self.declare_parameter('speed_res_kmh_per_lsb', 0.5)

        self.topic = self.get_parameter('publish_topic').value
        self.period_ms = int(self.get_parameter('period_ms').value)
        self.speed_offset = float(self.get_parameter('speed_offset_kmh').value)
        self.speed_res = float(self.get_parameter('speed_res_kmh_per_lsb').value)

        self.pub = self.create_publisher(UInt8MultiArray, self.topic, 10)

        # 状态（初始全 0）
        self.speed_kmh = 0.0
        self.angle_deg = 0.0     # 正=右转，负=左转
        self.pick_val = 0        # 0/1（编码到 bit0-1 两位）
        self.unload_val = 0      # 0/1（编码到 bit2-3 两位）
        self.remote_val = 0      # bit4
        self.dump_val = 0        # bit5（切换 0/1）
        self.fixed_val = 0       # bit6

        # 按键按下状态（用于 speed/angle 松开回 0）
        self.pressed_speed = {'i': False, 'k': False}
        self.pressed_angle = {'u': False, 'o': False, 'j': False, 'l': False}
        self.press_time = {k: 0.0 for k in ['i','k','u','o','j','l']}
        self.debounce_sec = 0.12

        # 键盘接入
        if HAVE_KBD:
            self.get_logger().info("Keyboard mode: using 'keyboard' (supports key up/down).")
            keyboard.on_press(self._on_key_down)
            keyboard.on_release(self._on_key_up)
            self.timer = self.create_timer(self.period_ms/1000.0, self._on_timer)
        elif HAVE_TTY:
            self.get_logger().warn("Fallback TTY mode (no key-up). 不建议用于“松开回0”的需求。")
            try:
                self.fd = sys.stdin.fileno()
                self.old = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
                self.raw = True
            except Exception as e:
                self.get_logger().warn(f"TTY raw mode failed: {e}")
                self.raw = False
            self.timer = self.create_timer(self.period_ms/1000.0, self._on_timer_tty)
        else:
            self.get_logger().error("无 keyboard 库也无 TTY，无法捕获按键。")
            self.timer = self.create_timer(self.period_ms/1000.0, self._on_timer)

        self.get_logger().info("Keys: i(4km/h) k(8km/h) u(-20) o(20) j(-10) l(10) m(fixed toggle) r(remote toggle) y(unload toggle) n(pick toggle) h(dump toggle) q(quit)")

    # ---------- keyboard 模式 ----------
    def _on_key_down(self, ev):
        key = (ev.name or '').lower()
        if not key: return
        now = time.time()

        if key in ('i','k','u','o','j','l'):
            # 防抖
            if now - self.press_time.get(key, 0.0) < self.debounce_sec:
                return
            self.press_time[key] = now

        if key == 'i':
            self.pressed_speed['i'] = True
        elif key == 'k':
            self.pressed_speed['k'] = True
        elif key == 'u':
            self.pressed_angle['u'] = True
        elif key == 'o':
            self.pressed_angle['o'] = True
        elif key == 'j':
            self.pressed_angle['j'] = True
        elif key == 'l':
            self.pressed_angle['l'] = True

        elif key == 'm':  # 固定动作 toggle
            self.fixed_val ^= 1
            self.get_logger().info(f"fixed: {self.fixed_val}")
        elif key == 'r':  # 请求遥控 toggle
            self.remote_val ^= 1
            self.get_logger().info(f"remote: {self.remote_val}")
        elif key == 'y':  # 卸货 toggle（两位字段，这里只用 0/1）
            self.unload_val = 0 if self.unload_val else 1
            self.get_logger().info(f"unload: {self.unload_val}")
        elif key == 'n':  # 拾取 toggle（两位字段，这里只用 0/1）
            self.pick_val = 0 if self.pick_val else 1
            self.get_logger().info(f"pick: {self.pick_val}")
        elif key == 'h':  # 下斗 toggle（非 hold）
            self.dump_val = 0 if self.dump_val else 1
            self.get_logger().info(f"dump: {self.dump_val}")
        elif key == 'q':
            self.get_logger().info("quit")
            rclpy.shutdown()

        # 立刻更新速度/角（按“优先级/最新按下”计算）
        self._recompute_discrete_cmds()

    def _on_key_up(self, ev):
        key = (ev.name or '').lower()
        if not key: return

        if key in ('i','k'):
            self.pressed_speed[key] = False
            self._recompute_discrete_cmds()

        elif key in ('u','o','j','l'):
            self.pressed_angle[key] = False
            self._recompute_discrete_cmds()

        # m/r/y/n/h 为 toggle，不在 key up 动作

    def _recompute_discrete_cmds(self):
        # 速度：K 优先（8km/h），否则 I（4km/h），否则 0
        if self.pressed_speed['k']:
            self.speed_kmh = 8.0
        elif self.pressed_speed['i']:
            self.speed_kmh = 4.0
        else:
            self.speed_kmh = 0.0

        # 角度：优先选 20°按键（最近按下），否则 10°（最近按下），否则 0
        # 20°：u=-20, o=+20；10°：j=-10, l=+10
        candidates = []
        if self.pressed_angle['u']: candidates.append(('u',  20.0))
        if self.pressed_angle['o']: candidates.append(('o', -20.0))
        if self.pressed_angle['j']: candidates.append(('j',  10.0))
        if self.pressed_angle['l']: candidates.append(('l', -10.0))

        if not candidates:
            self.angle_deg = 0.0
        else:
            # 先按绝对值排序(20优先)，同绝对值按最近按下时间
            candidates.sort(key=lambda kv: (abs(kv[1])*-1, self.press_time.get(kv[0], 0.0)), reverse=False)
            # 上面的排序稍别扭，简单起见直接手挑：有20就挑最近的20，否则挑最近的10
            best_key = None; best_ts = -1.0; best_val = 0.0
            for k,v in candidates:
                if abs(v) == 20.0:
                    ts = self.press_time.get(k, 0.0)
                    if ts > best_ts:
                        best_ts = ts; best_key = k; best_val = v
            if best_key is None:
                for k,v in candidates:
                    if abs(v) == 10.0:
                        ts = self.press_time.get(k, 0.0)
                        if ts > best_ts:
                            best_ts = ts; best_key = k; best_val = v
            self.angle_deg = best_val

    def _on_timer(self):
        self._publish_payload()

    # ---------- TTY 退化 ----------
    def _on_timer_tty(self):
        # 简单的无释放：按下设置值，2*period 后复位
        try:
            dr, _, _ = select.select([sys.stdin], [], [], 0)
            if dr:
                ch = sys.stdin.read(1).lower()
                if ch == 'i': self.speed_kmh = 4.0; self._schedule_reset('speed')
                elif ch == 'k': self.speed_kmh = 8.0; self._schedule_reset('speed')
                elif ch == 'u': self.angle_deg =  20.0; self._schedule_reset('angle')
                elif ch == 'o': self.angle_deg = -20.0; self._schedule_reset('angle')
                elif ch == 'j': self.angle_deg =  10.0; self._schedule_reset('angle')
                elif ch == 'l': self.angle_deg = -10.0; self._schedule_reset('angle')
                elif ch == 'm': self.fixed_val ^= 1
                elif ch == 'r': self.remote_val ^= 1
                elif ch == 'y': self.unload_val = 0 if self.unload_val else 1
                elif ch == 'n': self.pick_val   = 0 if self.pick_val   else 1
                elif ch == 'h': self.dump_val   = 0 if self.dump_val   else 1
                elif ch == 'q': rclpy.shutdown()
        except Exception:
            pass

        # 自动复位
        now = time.time()
        if hasattr(self, '_reset_deadline_speed') and now >= self._reset_deadline_speed:
            self.speed_kmh = 0.0; self._reset_deadline_speed = now + 1e9
        if hasattr(self, '_reset_deadline_angle') and now >= self._reset_deadline_angle:
            self.angle_deg = 0.0; self._reset_deadline_angle = now + 1e9

        self._publish_payload()

    def _schedule_reset(self, which):
        dl = time.time() + (2 * self.period_ms / 1000.0)
        if which == 'speed':
            self._reset_deadline_speed = dl
        elif which == 'angle':
            self._reset_deadline_angle = dl

    # ---------- 打包并发布 ----------
    def _publish_payload(self):
        # dist 固定 0
        dist_raw = 0

        # angle_raw（0..359）：steer_deg = -angle_deg；angle_raw = round(steer_deg + 180)
        steer_deg = -float(self.angle_deg)
        while steer_deg >= 180.0: steer_deg -= 360.0
        while steer_deg <  -180.0: steer_deg += 360.0
        angle_raw = int(round(steer_deg + 180.0))
        angle_raw = clamp(angle_raw, 0, 359)

        # speed_raw（1字节）
        raw = (float(self.speed_kmh) - self.speed_offset) / self.speed_res
        speed_raw = int(round(raw)); speed_raw = clamp(speed_raw, 0, 255)

        # Byte5：bit0-1=pick(2bit)、bit2-3=unload(2bit)、bit4=remote、bit5=dump、bit6=fixed
        pick2   = int(self.pick_val)   & 0x3
        unload2 = int(self.unload_val) & 0x3
        remote  = int(self.remote_val) & 0x1
        dump    = int(self.dump_val)   & 0x1
        fixed   = int(self.fixed_val)  & 0x1
        byte5 = (pick2) | (unload2 << 2) | (remote << 4) | (dump << 5) | (fixed << 6)

        # Byte6：bit0=ESTOP, bit1=DRIVE（本遥控节点固定0）
        byte6 = 0

        payload = [0]*8
        payload[0] = dist_raw & 0xFF
        payload[1] = (dist_raw >> 8) & 0xFF
        payload[2] = angle_raw & 0xFF
        payload[3] = (angle_raw >> 8) & 0xFF
        payload[4] = speed_raw
        payload[5] = byte5
        payload[6] = byte6
        payload[7] = 0x00

        msg = UInt8MultiArray()
        msg.data = payload
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VcuCanTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 恢复 TTY
    try:
        if HAVE_TTY and hasattr(node, 'raw') and node.raw:
            termios.tcsetattr(node.fd, termios.TCSADRAIN, node.old)
    except Exception:
        pass
    node.destroy_node()
    rclpy.shutdown()
