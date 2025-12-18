#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
local_follower.py
- 读取全局路径（/global_path）+ 融合位姿（/odometry/filtered_map）
- 纯跟踪（指向前视点）+ 转弯降速 + 大误差原地转
- 闭环路径：按弧长累计“跑一圈就停”；终点对准仅在圈末触发
- 只接一次路径：防止 path_server 周期发布导致的 near/prog 跳变
- 角落预停支持“硬停锁存”：在拐角前硬停并原地对准拐角后的切向，达角阈值再恢复前进
- 静止/硬停时冻结最近点与圈进度，避免“原地不动但进度增长”
- 靠近拐角自适应缩短前视距离与提前减速（距离坡度 + 曲率限速）
- 速度硬上限 2 m/s；误差小则强制给最小速度，避免长时间原地转
- 支持单步调试（/sim/step）与调试打印
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32, Bool, UInt32, Empty
from geometry_msgs.msg import Quaternion
from gps_msgs.msg import AngleSpeed


def yaw_from_quat(q: Quaternion) -> float:
    """从四元数提取 Z 轴偏航角（ENU：x东/y北，逆时针为正）"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_pi(a: float) -> float:
    """将角度归一化到 (-pi, pi]"""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_bearing_deg(yaw_rad: float) -> float:
    """
    yaw 到 bearing 的转换
    - yaw_rad：相对 x东 逆时针为正
    - bearing：相对地理北顺时针为正，范围 [-180, 180)
    关系：bearing = 90° - yaw
    """
    deg = 90.0 - math.degrees(yaw_rad)
    return (deg + 180.0) % 360.0 - 180.0


class LocalFollower(Node):
    def __init__(self):
        super().__init__('gps_local_follower')

        # 速度/到点
        self.declare_parameter('base_speed', 1.0)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('reach_dist', 0.8)
        self.declare_parameter('reach_yaw_deg', 5.0)
        self.declare_parameter('allow_reverse', False)

        # 转弯/降速/原地转
        self.declare_parameter('yaw_stop_deg', 50.0)        # 大误差直接原地转
        self.declare_parameter('yaw_slow_deg', 15.0)        # 中等误差开始降速
        self.declare_parameter('min_speed_mps', 0.2)        # 小误差时的最小推进速度
        self.declare_parameter('shrink_lookahead_on_turn', True)

        # 拐角预停（带锁存）
        self.declare_parameter('corner_stop_enable', True)  # 开启拐角预停
        self.declare_parameter('corner_hard_stop', True)    # 硬停锁存：原地对准拐角后切向
        self.declare_parameter('corner_stop_angle_deg', 35.0)  # 触发角阈值
        self.declare_parameter('corner_check_ahead_m', 2.0)    # 从前视点再往前检查的弧长
        self.declare_parameter('corner_min_ahead_m', 2.0)      # 与拐角距离≤该值才硬停
        self.declare_parameter('corner_release_deg', 8.0)      # 对准到≤该角度阈值解除硬停

        # 靠近拐角提前减速（基于距离与曲率上限）
        self.declare_parameter('corner_slow_enable', True)
        self.declare_parameter('corner_slow_start_m', 12.0)     # 开始减速的前方距离（可调大些）
        self.declare_parameter('corner_slow_end_m', 2.0)        # 减速到最小的距离（≥ corner_min_ahead）
        self.declare_parameter('corner_slow_min_speed', 0.3)    # 减速下限速度
        self.declare_parameter('a_lat_max', 1.0)                # 最大横向加速度（m/s^2），用于曲率限速

        # 跑一圈就停 / 单步调试 / 打印
        self.declare_parameter('stop_after_one_lap', True)
        self.declare_parameter('step_mode', False)
        self.declare_parameter('debug_print', True)

        # 静止判据阈值（米），用于冻结最近点与圈进度
        self.declare_parameter('freeze_move_thresh_m', 0.10)

        # 读取参数
        self.v_base = float(self.get_parameter('base_speed').value)
        self.v_max = float(self.get_parameter('max_speed').value)
        self.reach_dist = float(self.get_parameter('reach_dist').value)
        self.reach_yaw = math.radians(float(self.get_parameter('reach_yaw_deg').value))
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)

        self.yaw_stop = math.radians(float(self.get_parameter('yaw_stop_deg').value))
        self.yaw_slow = math.radians(float(self.get_parameter('yaw_slow_deg').value))
        self.v_min = float(self.get_parameter('min_speed_mps').value)
        self.shrink_ld = bool(self.get_parameter('shrink_lookahead_on_turn').value)

        self.corner_enable = bool(self.get_parameter('corner_stop_enable').value)
        self.corner_hard = bool(self.get_parameter('corner_hard_stop').value)
        self.corner_ang = math.radians(float(self.get_parameter('corner_stop_angle_deg').value))
        self.corner_ahead = float(self.get_parameter('corner_check_ahead_m').value)
        self.corner_min_ahead = float(self.get_parameter('corner_min_ahead_m').value)
        self.corner_release = math.radians(float(self.get_parameter('corner_release_deg').value))

        self.corner_slow_enable = bool(self.get_parameter('corner_slow_enable').value)
        self.slow_start = float(self.get_parameter('corner_slow_start_m').value)
        self.slow_end = float(self.get_parameter('corner_slow_end_m').value)
        self.slow_min = float(self.get_parameter('corner_slow_min_speed').value)
        self.a_lat_max = float(self.get_parameter('a_lat_max').value)

        self.stop_after_one_lap = bool(self.get_parameter('stop_after_one_lap').value)
        self.step_mode = bool(self.get_parameter('step_mode').value)
        self.debug_print = bool(self.get_parameter('debug_print').value)
        self.freeze_thresh = float(self.get_parameter('freeze_move_thresh_m').value)

        # 订阅/发布
        self.create_subscription(Path, '/global_path', self.on_path, 1)
        self.create_subscription(Odometry, '/odometry/filtered_map', self.on_odom, 20)
        self.create_subscription(Float32, '/obstacle/velocity_scale', self.on_scale, 1)
        self.create_subscription(Bool, '/obstacle/hard_stop', self.on_stop, 1)
        self.create_subscription(Empty, '/sim/step', self.on_step, 1)

        self.pub_cmd = self.create_publisher(AngleSpeed, '/drive_cmd', 10)
        self.pub_task_evt = self.create_publisher(UInt32, '/events/reached_task_point', 10)

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz

        # 运行态
        self.path = []               # [(x,y,yaw_tan), ...]
        self.idx_hint = 0
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.scale = 1.0
        self.hard_stop = False

        # 弧长与圈进度
        self.s_path = []             # 弧长表 s[i]
        self.total_len = 0.0
        self.start_s = None
        self.prev_s = None
        self.progress_s = 0.0
        self.finished = False

        # 拐角硬停锁存状态
        self.corner_stopping = False
        self.corner_target_yaw = 0.0

        # 单步与“上一拍坐标/速度”
        self._step_pending = False
        self._last_x = None
        self._last_y = None
        self._last_cmd_speed = 0.0

        # 仅接一次路径的锁
        self._path_locked = False

    # ===== 工具函数 =====
    def path_heading(self, i: int) -> float:
        """取路径在索引 i 的切向角（若 i 为倒数第二个点，采用 i->i+1）"""
        n = len(self.path)
        if n < 2:
            return 0.0
        i0 = max(0, min(n - 2, i))
        x0, y0, _ = self.path[i0]
        x1, y1, _ = self.path[i0 + 1]
        return math.atan2(y1 - y0, x1 - x0)

    def find_index_by_arclength(self, start_i: int, ds: float) -> int:
        """从 start_i 沿着路径前进弧长 ds，返回新索引"""
        n = len(self.path)
        if n < 2:
            return start_i
        s = 0.0
        i = start_i
        x0, y0, _ = self.path[i]
        while s < ds and i + 1 < n:
            x1, y1, _ = self.path[i + 1]
            s += math.hypot(x1 - x0, y1 - y0)
            x0, y0 = x1, y1
            i += 1
        return i

    def angle_diff(self, a: float, b: float) -> float:
        """角差 a-b 归一化"""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    # ===== 回调 =====
    def on_scale(self, msg: Float32):
        """外部避障速度倍率 0~1"""
        self.scale = max(0.0, min(1.0, msg.data))

    def on_stop(self, msg: Bool):
        """外部紧急停"""
        self.hard_stop = bool(msg.data)

    def on_step(self, _msg: Empty):
        """单步模式触发一次推进"""
        self._step_pending = True

    def on_path(self, msg: Path):
        """
        只接一次路径：首次加载/构建弧长表后即锁定
        防止 path_server 周期发布导致圈进度/near 跳变
        """
        if self._path_locked:
            return

        self.path = [(p.pose.position.x, p.pose.position.y, yaw_from_quat(p.pose.orientation)) for p in msg.poses]
        self.idx_hint = 0

        # 构建弧长表
        self.s_path = [0.0]
        for i in range(1, len(self.path)):
            x0, y0, _ = self.path[i - 1]
            x1, y1, _ = self.path[i]
            self.s_path.append(self.s_path[-1] + math.hypot(x1 - x0, y1 - y0))
        self.total_len = self.s_path[-1] if self.s_path else 0.0

        # 重置圈进度
        self.start_s = None
        self.prev_s = None
        self.progress_s = 0.0
        self.finished = False

        # 锁定路径
        self._path_locked = True
        self.get_logger().info(f"global_path locked: points={len(self.path)}, total_len={self.total_len:.2f} m")

    def on_odom(self, msg: Odometry):
        """融合位姿（map 下），用于最近点与前视点几何计算"""
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    # ===== 基本索引查找 =====
    def nearest_index(self, start=0, window=400) -> int:
        """在 [start, start+window) 范围内找到与当前位置最近的路径索引"""
        if not self.path:
            return 0
        n = len(self.path)
        i0 = min(max(0, start), n - 1)
        i1 = min(n, i0 + window)
        d2_min = 1e18
        k = i0
        for i in range(i0, i1):
            x, y, _ = self.path[i]
            d2 = (x - self.x) ** 2 + (y - self.y) ** 2
            if d2 < d2_min:
                d2_min = d2
                k = i
        return k

    def lookahead_index(self, k_near: int, Ld: float) -> int:
        """从最近点开始沿弧长 Ld 找到前视索引"""
        if not self.path:
            return 0
        n = len(self.path)
        s = 0.0
        i = k_near
        x0, y0, _ = self.path[i]
        while s < Ld and i + 1 < n:
            x1, y1, _ = self.path[i + 1]
            s += math.hypot(x1 - x0, y1 - y0)
            x0, y0 = x1, y1
            i += 1
        return i

    # ===== 控制主循环 =====
    def on_timer(self):
        if not self.path:
            return

        # 单步 gating
        if self.step_mode and not self._step_pending:
            return
        if self.step_mode:
            self._step_pending = False

        # 判断这拍是否实际移动（冻结最近点/圈进度的依据）
        moved_small = False
        if self._last_x is not None:
            moved_small = math.hypot(self.x - self._last_x, self.y - self._last_y) < self.freeze_thresh

        # 1) 最近点（停/硬停/上一拍速度为0 时冻结，否则查找）
        if self.corner_stopping or moved_small or self._last_cmd_speed < 1e-3:
            k_near = self.idx_hint
        else:
            k_near = self.nearest_index(self.idx_hint, window=400)
        self.idx_hint = max(self.idx_hint, k_near)

        # 2) 圈进度（停/硬停/静止时不累计；闭环跑满一圈即停）
        if self.stop_after_one_lap and self.total_len > 0.0 and not self.finished:
            if not (self.corner_stopping or moved_small or self._last_cmd_speed < 1e-3):
                s_cur = self.s_path[k_near]
                if self.start_s is None:
                    self.start_s = s_cur
                    self.prev_s = s_cur
                else:
                    ds = s_cur - self.prev_s
                    if ds < -0.5 * self.total_len:
                        ds += self.total_len
                    if ds > 0.0:
                        self.progress_s += ds
                    self.prev_s = s_cur
                    if self.progress_s >= 0.98 * self.total_len:
                        self.finished = True

        # 3) 基础速度与前视距离（速度硬上限 2 m/s）
        v_cmd = min(self.v_max, max(self.v_min, self.v_base)) * self.scale
        v_cmd = min(v_cmd, 2.0)
        Ld = min(5.0, max(1.0, 2.0 + 1.5 * v_cmd))
        k_ld = self.lookahead_index(k_near, Ld)
        xL, yL, _ = self.path[k_ld]

        # 4) 指令朝向 = 指向前视点的方位（先初始化 bearing_cmd）
        yaw_to_look = math.atan2(yL - self.y, xL - self.x)
        yaw_err = normalize_pi(yaw_to_look - self.yaw)
        bearing_cmd = yaw_to_bearing_deg(yaw_to_look)

        # 大误差时缩短前视距离（更容易咬住弯）
        if self.shrink_ld and abs(yaw_err) > self.yaw_slow:
            shrink = max(0.3, 1.0 - abs(yaw_err) / self.yaw_stop)  # [0.3, 1.0]
            Ld2 = max(0.8, Ld * shrink)
            if Ld2 < Ld:
                k_ld = self.lookahead_index(k_near, Ld2)
                xL, yL, _ = self.path[k_ld]
                yaw_to_look = math.atan2(yL - self.y, xL - self.x)
                yaw_err = normalize_pi(yaw_to_look - self.yaw)
                bearing_cmd = yaw_to_bearing_deg(yaw_to_look)

        # 4.5) 靠近拐角：预估拐角信息（临时）
        k_corner_tmp = self.find_index_by_arclength(k_ld, self.corner_ahead)
        hd_now_tmp = self.path_heading(k_near)
        hd_next_tmp = self.path_heading(k_corner_tmp)
        corner_angle_tmp = abs(self.angle_diff(hd_next_tmp, hd_now_tmp))
        ds_to_corner_tmp = self.s_path[k_corner_tmp] - self.s_path[k_near]
        if ds_to_corner_tmp < 0.0 and self.total_len > 0.0:
            ds_to_corner_tmp += self.total_len

        # 4.6) 靠近拐角提前“缩 Ld + 减速”（即便 yaw_err≈0 也会减速）
        if corner_angle_tmp >= self.corner_ang:
            # 4.6.1 缩 Ld（保守）
            if ds_to_corner_tmp <= 2.0 * Ld and not self.corner_stopping:
                Ld2 = max(0.8, min(Ld, ds_to_corner_tmp * 0.6))
                k_ld = self.lookahead_index(k_near, Ld2)
                xL, yL, _ = self.path[k_ld]
                yaw_to_look = math.atan2(yL - self.y, xL - self.x)
                yaw_err = normalize_pi(yaw_to_look - self.yaw)
                bearing_cmd = yaw_to_bearing_deg(yaw_to_look)

            # 4.6.2 距离坡度减速（slow_start→slow_end线性降速到 slow_min）
            if self.corner_slow_enable and ds_to_corner_tmp <= self.slow_start:
                slow_end = max(self.slow_end, self.corner_min_ahead)
                if ds_to_corner_tmp <= slow_end:
                    v_ramp = self.slow_min
                else:
                    alpha = (ds_to_corner_tmp - slow_end) / max(1e-3, (self.slow_start - slow_end))  # 1→0
                    v_ramp = max(self.slow_min, self.v_base * alpha)
                v_cmd = min(v_cmd, v_ramp)

            # 4.6.3 曲率限速（a_lat = v^2 * kappa ⇒ v_max = sqrt(a_lat_max / kappa)）
            ds_for_kappa = max(0.5, min(4.0, ds_to_corner_tmp))  # 取有限弧长估算曲率
            kappa = abs(self.angle_diff(hd_next_tmp, hd_now_tmp)) / max(0.5, ds_for_kappa)
            if kappa > 1e-6:
                v_curv = math.sqrt(max(0.1, self.a_lat_max) / kappa)
                v_cmd = min(v_cmd, v_curv)

        # 5) 若已处于拐角硬停：原地对准到阈值后解除；解除时跳过拐角后小段，避免抖动
        if self.corner_stopping:
            yaw_err_corner = normalize_pi(self.corner_target_yaw - self.yaw)
            bearing_cmd = yaw_to_bearing_deg(self.corner_target_yaw)
            v_cmd = 0.0
            if abs(yaw_err_corner) <= self.corner_release:
                self.corner_stopping = False
                # 解除硬停时，将 idx_hint 跳到拐角后小段，避免起步仍盯着拐角抖动
                k_after = self.find_index_by_arclength(k_near, max(0.5, 0.3 * Ld))
                self.idx_hint = max(self.idx_hint, k_after)
            msg = AngleSpeed()
            msg.heading_deg = float(bearing_cmd)
            msg.speed_mps = float(v_cmd)
            self.pub_cmd.publish(msg)
            # 记录上一拍位置/速度
            self._last_x = self.x
            self._last_y = self.y
            self._last_cmd_speed = float(v_cmd)
            return

        # 6) 拐角预停：满足条件则进入“硬停锁存”或“温和预停”
        if self.corner_enable:
            k_corner = self.find_index_by_arclength(k_ld, self.corner_ahead)
            hd_now = self.path_heading(k_near)
            hd_next = self.path_heading(k_corner)
            corner_angle = abs(self.angle_diff(hd_next, hd_now))
            ds_to_corner = self.s_path[k_corner] - self.s_path[k_near]
            if ds_to_corner < 0.0 and self.total_len > 0.0:
                ds_to_corner += self.total_len
            # 更早触发硬停：ds_to_corner <= max(corner_min_ahead, 0.8*Ld)
            if (corner_angle >= self.corner_ang) and (ds_to_corner <= max(self.corner_min_ahead, 0.8 * Ld)):
                if self.corner_hard:
                    self.corner_stopping = True
                    self.corner_target_yaw = hd_next
                    bearing_cmd = yaw_to_bearing_deg(self.corner_target_yaw)
                    v_cmd = 0.0
                    msg = AngleSpeed()
                    msg.heading_deg = float(bearing_cmd)
                    msg.speed_mps = float(v_cmd)
                    self.pub_cmd.publish(msg)
                    self._last_x = self.x
                    self._last_y = self.y
                    self._last_cmd_speed = float(v_cmd)
                    return
                else:
                    v_cmd = 0.0  # 温和预停：停一拍，后续仍走常规策略

        # 7) 转弯策略：大误差原地转 / 中误差降速 / 小误差保底推进
        if abs(yaw_err) > self.yaw_stop:
            v_cmd = 0.0
        elif abs(yaw_err) > self.yaw_slow:
            scale_turn = max(0.2, 1.0 - (abs(yaw_err) - self.yaw_slow) / (self.yaw_stop - self.yaw_slow))
            v_cmd = max(self.v_min, v_cmd * scale_turn)
        else:
            v_cmd = max(v_cmd, self.v_min)

        # 8) 跑一圈就停
        if self.finished:
            v_cmd = 0.0

        # 9) 终点对准（闭环仅在圈末触发；非闭环可随时触发）
        gx, gy, gyaw = self.path[-1]
        dist_goal = math.hypot(self.x - gx, self.y - gy)
        goal_align_ok = (not self.stop_after_one_lap) or (self.total_len > 0.0 and self.progress_s > 0.9 * self.total_len)
        if goal_align_ok and dist_goal < self.reach_dist and not self.finished:
            yaw_err_goal = normalize_pi(gyaw - self.yaw)
            if abs(yaw_err_goal) > self.reach_yaw:
                v_cmd = 0.0
                bearing_cmd = yaw_to_bearing_deg(self.yaw + yaw_err_goal)
            else:
                evt = UInt32()
                evt.data = 0
                self.pub_task_evt.publish(evt)

        # 10) 外部急停
        if self.hard_stop:
            v_cmd = 0.0

        # 11) 调试打印
        if self.debug_print:
            prog = 0.0 if self.total_len == 0 else 100.0 * self.progress_s / self.total_len
            self.get_logger().info(
                f"near={k_near} ld={k_ld} Ld={Ld:.2f} yaw_err={math.degrees(yaw_err):.1f}deg "
                f"speed={v_cmd:.2f} bearing={bearing_cmd:.1f} prog={prog:.1f}% finished={self.finished}"
            )

        # 12) 发布指令
        msg = AngleSpeed()
        msg.heading_deg = float(bearing_cmd)
        msg.speed_mps = float(v_cmd)
        self.pub_cmd.publish(msg)

        # 记录上一拍位置/速度（用于冻结逻辑）
        self._last_x = self.x
        self._last_y = self.y
        self._last_cmd_speed = float(v_cmd)


def main():
    rclpy.init()
    node = LocalFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
