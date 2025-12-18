#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pid_follower.py
- 订阅 CSV 航点（/global_path_geo），先缓存
- 在 on_timer 中检测到 /fromLL ready 后，懒加载将航点一次性转换为 map 坐标并构建分段（直线/圆弧）
- 在线计算 CTE 与路径切向 yaw_path，横向 PID：cte -> yaw_corr，yaw_cmd = yaw_path_ff - yaw_corr
- 纵向：常速 + 基于 yaw_err 的降速
- 输出 /drive_cmd (AngleSpeed)：heading_deg(地理北顺时针) + speed_mps
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool, Empty, UInt32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from gps_msgs.msg import GlobalPath, Waypoint, AngleSpeed
from robot_localization.srv import FromLL

# ===== 工具函数 =====
def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quat(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def bearing_deg_to_yaw_rad(bearing_deg: float) -> float:
    # bearing: 北顺时针；yaw: 东逆时针
    return math.radians(90.0 - bearing_deg)

def yaw_to_bearing_deg(yaw_rad: float) -> float:
    deg = 90.0 - math.degrees(yaw_rad)
    return (deg + 180.0) % 360.0 - 180.0

def intersect_lines(p, n, q, m):
    # 直线 p + t*n 与 q + s*m 的交点
    denom = n[0]*m[1] - n[1]*m[0]
    if abs(denom) < 1e-9:
        return None
    t = ((q[0]-p[0])*m[1] - (q[1]-p[1])*m[0]) / denom
    return (p[0] + t*n[0], p[1] + t*n[1])

def ang_wrap(a):
    while a <= -math.pi: a += 2.0*math.pi
    while a >  math.pi: a -= 2.0*math.pi
    return a

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

# ===== 节点 =====
class PIDFollowerGeo(Node):
    def __init__(self):
        super().__init__('gps_pid_follower')

        # 速度与门限
        self.declare_parameter('base_speed', 0.6)
        self.declare_parameter('min_speed', 0.2)
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('yaw_slow_deg', 20.0)
        self.declare_parameter('yaw_stop_deg', 75.0)

        # 横向 PID
        self.declare_parameter('kp', 0.40)
        self.declare_parameter('ki', 0.00)
        self.declare_parameter('kd', 0.20)
        self.declare_parameter('i_max', 0.5)
        self.declare_parameter('yaw_corr_deg_max', 35.0)

        # 前瞻距离（用于 yaw_path 前馈）
        self.declare_parameter('lookahead_m', 1.5)

        # 运行选项
        self.declare_parameter('stop_after_one_lap', False)
        self.declare_parameter('freeze_move_thresh_m', 0.10)
        self.declare_parameter('step_mode', False)
        self.declare_parameter('debug_print', True)

        # 读参
        self.v_base = float(self.get_parameter('base_speed').value)
        self.v_min  = float(self.get_parameter('min_speed').value)
        self.v_max  = float(self.get_parameter('max_speed').value)
        self.yaw_slow = math.radians(float(self.get_parameter('yaw_slow_deg').value))
        self.yaw_stop = math.radians(float(self.get_parameter('yaw_stop_deg').value))

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)
        self.i_max = float(self.get_parameter('i_max').value)
        self.yaw_corr_max = math.radians(float(self.get_parameter('yaw_corr_deg_max').value))

        self.Ld = float(self.get_parameter('lookahead_m').value)
        self.stop_after_one_lap = bool(self.get_parameter('stop_after_one_lap').value)
        self.freeze_thresh = float(self.get_parameter('freeze_move_thresh_m').value)
        self.step_mode = bool(self.get_parameter('step_mode').value)
        self.debug_print = bool(self.get_parameter('debug_print').value)

        # I/O
        self.create_subscription(GlobalPath, '/global_path_geo', self.on_geo, 1)
        self.create_subscription(Odometry, '/odometry/filtered_map', self.on_odom, 30)
        self.create_subscription(Float32, '/obstacle/velocity_scale', self.on_scale, 1)
        self.create_subscription(Bool, '/obstacle/hard_stop', self.on_stop, 1)
        self.create_subscription(Empty, '/sim/step', self.on_step, 1)
        self.pub_cmd = self.create_publisher(AngleSpeed, '/drive_cmd', 10)
        self.pub_task_evt = self.create_publisher(UInt32, '/events/reached_task_point', 10)

        # FromLL client
        self.fromll = self.create_client(FromLL, '/fromLL')

        # 状态
        self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.scale = 1.0; self.hard_stop = False
        self._step_pending = False
        self._last_x = None; self._last_y = None
        self._last_cmd_v = 0.0

        # PID 积分/微分
        self.cte_i = 0.0
        self.cte_prev = None
        self.last_t = self.get_clock().now()

        # 航点缓存（懒加载转换）
        self._geo_wps = None

        # 段模型
        self.segments = []   # 列表，每段: dict(type='line'/'arc', ... 含几何参数与长度)
        self.cum_len = []    # 段累计长度 s 累计表
        self.total_len = 0.0
        self.closed = True   # CSV 默认为闭环
        self.path_ready = False

        # 单圈统计
        self.start_s = None
        self.prev_s = None
        self.progress_s = 0.0
        self.finished = False

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz
        self.get_logger().info('pid_follower_geo started.')

    # ===== 话题回调 =====
    def on_scale(self, msg: Float32): self.scale = clamp(msg.data, 0.0, 1.0)
    def on_stop(self, msg: Bool): self.hard_stop = bool(msg.data)
    def on_step(self, _msg: Empty): self._step_pending = True

    def on_odom(self, msg: Odometry):
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)
        self.yaw = yaw_from_quat(msg.pose.pose.orientation)

    def on_geo(self, msg: GlobalPath):
        # 仅缓存航点，不阻塞转换；等 /fromLL 就绪后在 on_timer 中一次性转换
        if not msg.points:
            self.get_logger().warn('GlobalPath empty')
            return
        self._geo_wps = sorted(msg.points, key=lambda w: int(w.index))
        self.path_ready = False
        # 重置圈统计
        self.start_s = None; self.prev_s = None
        self.progress_s = 0.0; self.finished = False
        self.get_logger().info(f'GlobalPath received: N={len(self._geo_wps)} (will convert when /fromLL ready)')

    # ===== 段构建与几何 =====
    def build_segments(self, pts):
        # pts: [(x,y,yaw_rad,next_seg_type), ...] 按 index 排序
        self.segments.clear()
        self.cum_len = [0.0]
        N = len(pts)
        for i in range(N):
            x0,y0,yaw0,seg_type = pts[i]
            x1,y1,yaw1,_ = pts[(i+1)%N]  # 闭环
            seg = None
            if seg_type == 1:
                # 圆弧段：用切向法线求圆心
                n0 = (-math.sin(yaw0), math.cos(yaw0))
                n1 = (-math.sin(yaw1), math.cos(yaw1))
                c = intersect_lines((x0,y0), n0, (x1,y1), n1)
                if c is not None:
                    cx, cy = c
                    v0x, v0y = x0 - cx, y0 - cy
                    v1x, v1y = x1 - cx, y1 - cy
                    r0 = math.hypot(v0x, v0y); r1 = math.hypot(v1x, v1y)
                    if r0 > 1e-3 and r1 > 1e-3:
                        r = 0.5*(r0+r1)
                        th0 = math.atan2(v0y, v0x)
                        th1 = math.atan2(v1y, v1x)
                        # 方向：使圆弧切向尽量贴合 yaw0
                        tan_ccw = th0 + math.pi/2.0
                        tan_cw  = th0 - math.pi/2.0
                        err_ccw = abs(ang_wrap(tan_ccw - yaw0))
                        err_cw  = abs(ang_wrap(tan_cw  - yaw0))
                        ccw = err_ccw <= err_cw
                        dth = ang_wrap(th1 - th0)
                        if ccw and dth < 0: dth += 2*math.pi
                        if (not ccw) and dth > 0: dth -= 2*math.pi
                        arc_len = abs(dth) * r
                        seg = {
                            'type':'arc', 'A':(x0,y0), 'B':(x1,y1),
                            'C':(cx,cy), 'r':r, 'th0':th0, 'dth':dth, 'ccw':ccw,
                            'len': arc_len
                        }
            if seg is None:
                # 退化为直线段
                dx, dy = x1-x0, y1-y0
                L = math.hypot(dx, dy)
                if L < 1e-6:  # 重合点，跳过
                    continue
                yaw_line = math.atan2(dy, dx)
                seg = {'type':'line', 'A':(x0,y0), 'B':(x1,y1), 'yaw':yaw_line, 'len':L}
            self.segments.append(seg)
            self.cum_len.append(self.cum_len[-1] + seg['len'])
        self.total_len = self.cum_len[-1] if self.cum_len else 0.0

    def foot_on_line(self, P, seg):
        Ax,Ay = seg['A']; Bx,By = seg['B']
        vx, vy = Bx-Ax, By-Ay
        L2 = vx*vx + vy*vy
        if L2 < 1e-9:
            F = (Ax,Ay); t = 0.0
        else:
            t = clamp(((P[0]-Ax)*vx + (P[1]-Ay)*vy)/L2, 0.0, 1.0)
            F = (Ax + t*vx, Ay + t*vy)
        yaw = seg['yaw']
        s_on = t * seg['len']
        return F, yaw, s_on

    def clamp_on_arc(self, th0, dth, phi):
        # 将任意角 phi 投影到“从 th0 沿 dth 方向扫过的弧区间”
        delta = ang_wrap(phi - th0)
        if dth >= 0:  # CCW
            if delta < 0: delta += 2*math.pi
            delta = clamp(delta, 0.0, dth)
            return th0 + delta, delta
        else:         # CW
            if delta > 0: delta -= 2*math.pi
            delta = clamp(delta, dth, 0.0)
            return th0 + delta, delta

    def foot_on_arc(self, P, seg):
        cx,cy = seg['C']; r = seg['r']; th0 = seg['th0']; dth = seg['dth']
        phi = math.atan2(P[1]-cy, P[0]-cx)
        phi_c, delta = self.clamp_on_arc(th0, dth, phi)
        Fx = cx + r*math.cos(phi_c); Fy = cy + r*math.sin(phi_c)
        yaw = phi_c + (math.pi/2.0 if seg['ccw'] else -math.pi/2.0)
        s_on = abs(delta) * r
        return (Fx,Fy), yaw, s_on

    def nearest_segment(self, P):
        if not self.segments: return None
        best = None; best_d2 = 1e18
        for idx, seg in enumerate(self.segments):
            if seg['type'] == 'line':
                F,yaw,s_on = self.foot_on_line(P, seg)
            else:
                F,yaw,s_on = self.foot_on_arc(P, seg)
            dx = P[0]-F[0]; dy = P[1]-F[1]
            d2 = dx*dx + dy*dy
            if d2 < best_d2:
                best_d2 = d2
                best = (idx, F, yaw, s_on)
        return best  # (idx, foot(x,y), yaw_at_foot, s_on_seg)

    def pose_at_s(self, s_global):
        # s_global ∈ [0, total_len)，返回 (x,y,yaw_path)
        if self.total_len <= 0.0: return (0.0,0.0,0.0)
        s = s_global % self.total_len
        # 找到所在段
        idx = 0
        while idx < len(self.segments) and not (self.cum_len[idx] <= s <= self.cum_len[idx+1]):
            idx += 1
        if idx >= len(self.segments):
            idx = len(self.segments) - 1
        seg = self.segments[idx]
        s_on = s - self.cum_len[idx]
        if seg['type'] == 'line':
            t = 0.0 if seg['len']<=1e-9 else clamp(s_on / seg['len'], 0.0, 1.0)
            Ax,Ay = seg['A']; Bx,By = seg['B']
            x = Ax + t*(Bx-Ax); y = Ay + t*(By-Ay)
            yaw = seg['yaw']
            return (x,y,yaw)
        else:
            r = seg['r']; th0 = seg['th0']; dth = seg['dth']
            sign = 1.0 if dth >= 0 else -1.0
            phi = th0 + sign * (s_on / max(1e-6, r))
            cx,cy = seg['C']
            x = cx + r*math.cos(phi); y = cy + r*math.sin(phi)
            yaw = phi + (math.pi/2.0 if seg['ccw'] else -math.pi/2.0)
            return (x,y,yaw)

    # ===== 控制主循环 =====
    def on_timer(self):
        # 单步门控
        if self.step_mode and not self._step_pending:
            return
        if self.step_mode:
            self._step_pending = False

        # 懒加载：若尚未构建路径且已收到航点，且 /fromLL 已就绪，则执行一次转换
        if (not self.path_ready) and (self._geo_wps is not None):
            if not self.fromll.service_is_ready():
                # 等下一拍再试，避免阻塞
                return
            try:
                xy_list = []
                for wp in self._geo_wps:
                    req = FromLL.Request()
                    req.ll_point.latitude = float(wp.latitude)
                    req.ll_point.longitude = float(wp.longitude)
                    req.ll_point.altitude = 0.0
                    resp = self.fromll.call(req)  # 同步，航点数量少
                    xy_list.append((resp.map_point.x, resp.map_point.y,
                                    bearing_deg_to_yaw_rad(float(wp.heading_deg)),
                                    int(wp.next_seg_type)))
                self.build_segments(xy_list)
                self.path_ready = True
                self.get_logger().info(f'CSV waypoints converted: N={len(self._geo_wps)}, segments={len(self.segments)}, total_len={self.total_len:.2f} m')
            except Exception as e:
                self.get_logger().warn(f'fromLL convert failed, will retry: {e}')
                return

        # dt
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 0.5: dt = 0.05
        self.last_t = now

        if not self.path_ready or not self.segments:
            return

        # 静止检测（冻结进度的依据）
        moved_small = False
        if self._last_x is not None:
            moved_small = math.hypot(self.x - self._last_x, self.y - self._last_y) < self.freeze_thresh

        # 最近段与脚点
        P = (self.x, self.y)
        near = self.nearest_segment(P)
        if near is None:
            return
        idx, F, yaw_at_foot, s_on_seg = near
        # 全局弧长（用于前瞻与圈进度）
        s_here = self.cum_len[idx] + s_on_seg

        # 前瞻切向作为 yaw_path（前馈）
        _, _, yaw_path = self.pose_at_s(s_here + self.Ld)

        # CTE（以“左法向”为正）
        nx, ny = -math.sin(yaw_at_foot), math.cos(yaw_at_foot)
        cte = (self.x - F[0]) * nx + (self.y - F[1]) * ny

        # PID（cte -> yaw_corr）
        cte_dot = 0.0 if self.cte_prev is None else (cte - self.cte_prev) / max(1e-3, dt)
        self.cte_prev = cte
        # 积分限幅（按 i_max 限定等效角度贡献）
        self.cte_i += cte * dt
        if self.ki > 1e-9:
            i_lim = self.i_max / self.ki
            self.cte_i = clamp(self.cte_i, -i_lim, i_lim)
        yaw_corr = self.kp * cte + self.ki * self.cte_i + self.kd * cte_dot
        yaw_corr = clamp(yaw_corr, -self.yaw_corr_max, self.yaw_corr_max)

        yaw_cmd = wrap_pi(yaw_path - yaw_corr)
        yaw_err = wrap_pi(yaw_cmd - self.yaw)

        # 纵向速度
        v_cmd = clamp(self.v_base, self.v_min, self.v_max) * self.scale
        if abs(yaw_err) > self.yaw_stop:
            v_cmd = 0.0
        elif abs(yaw_err) > self.yaw_slow:
            alpha = (abs(yaw_err) - self.yaw_slow) / max(1e-6, (self.yaw_stop - self.yaw_slow))
            v_cmd = max(self.v_min, v_cmd * max(0.2, 1.0 - alpha))

        # 圈进度（闭环时统计）
        if self.stop_after_one_lap and self.total_len > 0.0 and not self.finished:
            if not (moved_small or self._last_cmd_v < 1e-3):
                if self.start_s is None:
                    self.start_s = s_here; self.prev_s = s_here
                else:
                    ds = s_here - self.prev_s
                    if ds < -0.5 * self.total_len: ds += self.total_len
                    if ds > 0.0: self.progress_s += ds
                    self.prev_s = s_here
                    if self.progress_s >= 0.98 * self.total_len:
                        self.finished = True
        if self.finished:
            v_cmd = 0.0

        if self.hard_stop:
            v_cmd = 0.0

        # 发布命令
        cmd = AngleSpeed()
        cmd.heading_deg = float(yaw_to_bearing_deg(yaw_cmd))
        cmd.speed_mps = float(v_cmd)
        self.pub_cmd.publish(cmd)

        if self.debug_print:
            prog = 0.0 if self.total_len == 0 else 100.0 * self.progress_s / self.total_len
            self.get_logger().info(
                f"seg={idx} cte={cte:.2f} yaw_path={math.degrees(yaw_path):.1f} "
                f"yaw_corr={math.degrees(yaw_corr):.1f} yaw_cmd={math.degrees(yaw_cmd):.1f} "
                f"yaw_err={math.degrees(yaw_err):.1f} v={v_cmd:.2f} prog={prog:.1f}% finished={self.finished}"
            )

        self._last_x = self.x; self._last_y = self.y; self._last_cmd_v = float(v_cmd)

def main():
    rclpy.init()
    node = PIDFollowerGeo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
