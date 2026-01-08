#!/usr/bin/env python3
import math
import csv
import struct
from typing import List, Tuple, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import Bool, Float32MultiArray, UInt8, Float32
from nav_msgs.msg import Path

import tf2_ros

try:
    import can
    HAVE_CAN = True
except Exception:
    HAVE_CAN = False


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def heading_csv_deg_to_enu_rad(hdg_deg: float) -> float:
    # 输入：0°=北，90°=东，顺时针为正 -> 输出 ENU: 0=东，逆时针为正
    return wrap_pi(math.radians(90.0 - hdg_deg))


class LLA2ENU:
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2 - f)

    def __init__(self, lat0_deg: float, lon0_deg: float, alt0: float = 0.0):
        self.lat0 = math.radians(lat0_deg)
        self.lon0 = math.radians(lon0_deg)
        self.alt0 = alt0
        self.x0, self.y0, self.z0 = self.lla_to_ecef(lat0_deg, lon0_deg, alt0)

        sphi = math.sin(self.lat0)
        cphi = math.cos(self.lat0)
        slam = math.sin(self.lon0)
        clam = math.cos(self.lon0)
        self.Re = [
            [-slam,            clam,           0.0],
            [-sphi*clam, -sphi*slam,  cphi],
            [ cphi*clam,  cphi*slam,  sphi]
        ]

    @classmethod
    def lla_to_ecef(cls, lat_deg, lon_deg, alt):
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        sphi = math.sin(lat)
        cphi = math.cos(lat)
        slam = math.sin(lon)
        clam = math.cos(lon)
        N = cls.a / math.sqrt(1.0 - cls.e2 * sphi * sphi)
        x = (N + alt) * cphi * clam
        y = (N + alt) * cphi * slam
        z = (N * (1.0 - cls.e2) + alt) * sphi
        return x, y, z

    def lla_to_enu(self, lat_deg, lon_deg, alt=0.0):
        x, y, z = self.lla_to_ecef(lat_deg, lon_deg, alt)
        dx = x - self.x0
        dy = y - self.y0
        dz = z - self.z0
        e = self.Re[0][0]*dx + self.Re[0][1]*dy + self.Re[0][2]*dz
        n = self.Re[1][0]*dx + self.Re[1][1]*dy + self.Re[1][2]*dz
        u = self.Re[2][0]*dx + self.Re[2][1]*dy + self.Re[2][2]*dz
        return e, n, u


class WaypointPIDFollower(Node):
    DS_PAUSED = 0
    DS_RUNNING = 1
    DS_ESTOP  = 2

    def __init__(self):
        super().__init__('waypoint_pid_follower')

        # I/O topics
        self.declare_parameter('path_csv', '')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('imu_topic', '/imu/data')

        # 路段/切换
        self.declare_parameter('advance_when_close', 2.5)
        self.declare_parameter('t_advance_min', 0.9)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('auto_align_yaw', False)

        # Heading policy
        self.declare_parameter('use_csv_heading', True)
        self.declare_parameter('align_heading_only_at_task_points', True)  # 当前逻辑未使用，仅保留接口
        self.declare_parameter('heading_align_dist', 1.0)

        # Task point handling
        self.declare_parameter('wp_reached_dist', 0.8)
        self.declare_parameter('wp_heading_tol_deg', 10.0)
        self.declare_parameter('stop_turn_tol_deg', 5.0)
        self.declare_parameter('wait_for_task_done', True)
        self.declare_parameter('task_done_topic', '/task_done')

        # Lookahead
        self.declare_parameter('lookahead_dist', 1.5)

        # Frames & viz
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('traj_path_len', 2000)

        # CAN parameters
        self.declare_parameter('enable_can', False)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_extended', True)
        self.declare_parameter('can_id_status', 0x18FED188)
        self.declare_parameter('link_remote_to_abort', True)

        # DR parameters
        self.declare_parameter('dead_reckon', True)
        self.declare_parameter('dr_alpha', 0.6)
        self.declare_parameter('dr_use_gps_speed', True)

        # Heading source (RTK)
        self.declare_parameter('use_rtk_heading', True)
        self.declare_parameter('rtk_heading_topic', '/gps/heading_deg')

        # VCU angle/speed（按你车的定义：0/4/8 km/h + 0/±10/±20°）
        self.declare_parameter('vcu_enhanced_mode', True)
        self.declare_parameter('vcu_angle_move_limit_deg', 20.0)
        self.declare_parameter('vcu_angle_spin_enter_deg', 22.0)
        self.declare_parameter('vcu_angle_spin_exit_deg', 18.0)
        self.declare_parameter('vcu_angle_spin_cmd_deg', 20.0)  # 原地旋转用 20°

        # 拐角识别
        self.declare_parameter('strict_corner_mode', True)
        self.declare_parameter('corner_start_dist', 1.0)
        self.declare_parameter('corner_sharp_deg_strict', 45.0)
        self.declare_parameter('strict_hold_deg', 2.0)  # 保留参数

        # 误差阈值
        self.declare_parameter('emergency_spin_hdg_deg', 35.0)
        self.declare_parameter('hdg_deadband_deg', 1.0)
        self.declare_parameter('cte_deadband_m', 0.1)
        self.declare_parameter('sign_hysteresis_deg', 1.0)

        # VCU 速度档（km/h）
        self.declare_parameter('vcu_speed_fast_kmh', 8.0)
        self.declare_parameter('vcu_speed_slow_kmh', 4.0)
        self.declare_parameter('vcu_speed_stop_kmh', 0.0)

        # 角度阈值 & 档位（0/±10/±20°）
        self.declare_parameter('vcu_turn_small_thresh_deg', 10.0)   # 小误差阈值
        self.declare_parameter('vcu_turn_large_thresh_deg', 25.0)   # 大误差阈值（>25° 算“大”）
        self.declare_parameter('vcu_turn_small_cmd_deg', 10.0)      # 实际下发档：±10°
        self.declare_parameter('vcu_turn_large_cmd_deg', 20.0)      # 实际下发档：±20°

        # Pre-speed raw encoding: km/h = raw*res + offset
        self.declare_parameter('vcu_speed_raw_offset_kmh', -50.0)
        self.declare_parameter('vcu_speed_raw_res_kmh_per_lsb', 0.5)

        # 读参数
        path_csv = self.get_parameter('path_csv').get_parameter_value().string_value
        if not path_csv:
            self.get_logger().error("path_csv is required.")
            raise SystemExit

        self.gps_topic = self.get_parameter('gps_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value

        self.advance_when_close = float(self.get_parameter('advance_when_close').value)
        self.t_advance_min = float(self.get_parameter('t_advance_min').value)
        self.yaw_offset = math.radians(float(self.get_parameter('yaw_offset_deg').value))
        self.auto_align_yaw = bool(self.get_parameter('auto_align_yaw').value)

        self.use_csv_heading = bool(self.get_parameter('use_csv_heading').value)
        self.align_heading_only_at_task_points = bool(self.get_parameter('align_heading_only_at_task_points').value)
        self.heading_align_dist = float(self.get_parameter('heading_align_dist').value)

        self.wp_reached_dist = float(self.get_parameter('wp_reached_dist').value)
        self.wp_heading_tol = math.radians(float(self.get_parameter('wp_heading_tol_deg').value))
        self.stop_turn_tol = math.radians(float(self.get_parameter('stop_turn_tol_deg').value))
        self.wait_for_task_done = bool(self.get_parameter('wait_for_task_done').value)
        self.task_done_topic = self.get_parameter('task_done_topic').value

        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.traj_path_len = int(self.get_parameter('traj_path_len').value)

        # CAN parse
        self.enable_can = bool(self.get_parameter('enable_can').value)
        self.can_interface = self.get_parameter('can_interface').value
        self.can_extended = bool(self.get_parameter('can_extended').value)
        _id_val: Union[int, str] = self.get_parameter('can_id_status').value
        try:
            if isinstance(_id_val, (int, float)):
                self.can_id_status = int(_id_val)
            else:
                self.can_id_status = int(str(_id_val), 0)
        except Exception:
            self.can_id_status = 0x18FED188
        self.link_remote_to_abort = bool(self.get_parameter('link_remote_to_abort').value)

        # DR
        self.dead_reckon = bool(self.get_parameter('dead_reckon').value)
        self.dr_alpha = float(self.get_parameter('dr_alpha').value)
        self.dr_use_gps_speed = bool(self.get_parameter('dr_use_gps_speed').value)
        self.dr_x = None
        self.dr_y = None
        self.last_gps_speed = None

        # RTK heading source
        self.use_rtk_heading = bool(self.get_parameter('use_rtk_heading').value)
        self.rtk_heading_topic = self.get_parameter('rtk_heading_topic').value

        # cache VCU params
        self.vcu_enhanced_mode = bool(self.get_parameter('vcu_enhanced_mode').value)
        self.vcu_angle_move_limit_deg = float(self.get_parameter('vcu_angle_move_limit_deg').value)
        self.vcu_angle_spin_enter_deg = float(self.get_parameter('vcu_angle_spin_enter_deg').value)
        self.vcu_angle_spin_exit_deg  = float(self.get_parameter('vcu_angle_spin_exit_deg').value)
        self.vcu_angle_spin_cmd_deg   = float(self.get_parameter('vcu_angle_spin_cmd_deg').value)

        self.strict_corner_mode = bool(self.get_parameter('strict_corner_mode').value)
        self.corner_start_dist = float(self.get_parameter('corner_start_dist').value)
        self.corner_sharp_rad_strict = math.radians(float(self.get_parameter('corner_sharp_deg_strict').value))
        self.strict_hold_deg = float(self.get_parameter('strict_hold_deg').value)

        self.emergency_spin_hdg_deg = float(self.get_parameter('emergency_spin_hdg_deg').value)
        self.hdg_deadband_deg = float(self.get_parameter('hdg_deadband_deg').value)
        self.cte_deadband_m = float(self.get_parameter('cte_deadband_m').value)
        self.sign_hysteresis_deg = float(self.get_parameter('sign_hysteresis_deg').value)
        self.angle_sign = 0  # 用于左右方向滞回

        self.vcu_speed_fast_kmh = float(self.get_parameter('vcu_speed_fast_kmh').value)
        self.vcu_speed_slow_kmh = float(self.get_parameter('vcu_speed_slow_kmh').value)
        self.vcu_speed_stop_kmh = float(self.get_parameter('vcu_speed_stop_kmh').value)

        self.vcu_turn_small_thresh_deg = float(self.get_parameter('vcu_turn_small_thresh_deg').value)
        self.vcu_turn_large_thresh_deg = float(self.get_parameter('vcu_turn_large_thresh_deg').value)
        self.vcu_turn_small_cmd_deg = float(self.get_parameter('vcu_turn_small_cmd_deg').value)
        self.vcu_turn_large_cmd_deg = float(self.get_parameter('vcu_turn_large_cmd_deg').value)

        self.vcu_speed_raw_offset_kmh = float(self.get_parameter('vcu_speed_raw_offset_kmh').value)
        self.vcu_speed_raw_res_kmh_per_lsb = float(self.get_parameter('vcu_speed_raw_res_kmh_per_lsb').value)

        # 转向模式状态
        self.turn_mode = 'MOVE'

        # SPIN 状态机：IDLE / PRE_STOP / SPIN
        self.spin_state = 'IDLE'
        self.spin_stop_frames_left = 0
        self.spin_use_next_seg = False  # True: 对准下一段；False: 对准当前路径
        self.spin_sign = 0.0           # +1 / -1

        # waypoints
        llh = self.load_csv(path_csv)
        if len(llh) < 1:
            self.get_logger().error("No valid waypoints in CSV")
            raise SystemExit
        llh.sort(key=lambda x: x[0])

        lat0, lon0 = llh[0][1], llh[0][2]
        self.geo = LLA2ENU(lat0, lon0, 0.0)
        self.waypoints_xy: List[Tuple[float, float, float, int]] = []
        for _, lat, lon, hdg_deg_csv, pt_type in llh:
            x, y, _ = self.geo.lla_to_enu(lat, lon, 0.0)
            yaw_enu = heading_csv_deg_to_enu_rad(hdg_deg_csv)
            self.waypoints_xy.append((x, y, yaw_enu, int(pt_type)))

        # State
        self.cur_x = None
        self.cur_y = None
        self.cur_yaw = None
        self.last_time = self.get_clock().now()

        self.seg_idx = 0
        self.aligned = False

        self.waiting_for_task = False
        self.task_done_latch = False

        # Drive state
        self.drive_state = self.DS_PAUSED
        self.remote_req_state = False
        self.pick_state = 0
        self.unload_state = 0
        self.dump_state = False

        # Subscriptions
        self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, qos_profile_sensor_data)
        if self.use_rtk_heading:
            self.create_subscription(Float32, self.rtk_heading_topic, self.on_rtk_heading, qos_profile_sensor_data)
        else:
            self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)

        self.create_subscription(Float32, '/gps/ground_speed_mps',
                                 lambda m: setattr(self, 'last_gps_speed', float(m.data)),
                                 qos_profile_sensor_data)

        # 动作覆盖输入（任务点等待期间，允许上位指定角度/预速）
        self.action_override = False
        self.action_pre_kmh = 0.0
        self.action_angle_deg = 0.0
        self.create_subscription(Bool,    '/action/use_override',  lambda m: setattr(self, 'action_override', bool(m.data)), 1)
        self.create_subscription(Float32, '/action/pre_speed_kmh', lambda m: setattr(self, 'action_pre_kmh', float(m.data)), 1)
        self.create_subscription(Float32, '/action/angle_deg',     lambda m: setattr(self, 'action_angle_deg', float(m.data)), 1)
        self.pub_task_wait = self.create_publisher(Bool, '/at_task_waiting', 1)

        # Viz & TF
        qos_tl = QoSProfile(depth=1)
        qos_tl.history = QoSHistoryPolicy.KEEP_LAST
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub_global_path = self.create_publisher(Path, 'global_path', qos_tl)
        self.pub_traj_path   = self.create_publisher(Path, 'traj_path', 10)
        self.tfb = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.global_path = Path()
        self.global_path.header.frame_id = self.map_frame
        for (x, y, _, _) in self.waypoints_xy:
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            self.global_path.poses.append(ps)

        self.traj_path = Path()
        self.traj_path.header.frame_id = self.map_frame
        self.max_traj_len = self.traj_path_len

        # timers & pubs
        self.create_timer(1.0, self._pub_global_path)
        self.pub_dbg = self.create_publisher(Float32MultiArray, 'ctrl_debug', 10)

        # Commands & task topics
        self.create_subscription(Bool,  self.task_done_topic, self.on_task_done, 1)
        self.create_subscription(UInt8, '/drive_cmd',        self.on_drive_cmd, 1)
        self.create_subscription(Bool,  '/abort',            self.on_abort,     1)
        self.create_subscription(UInt8, '/pick_cmd',   lambda m: setattr(self, 'pick_state',   int(max(0, min(1, m.data)))), 1)
        self.create_subscription(UInt8, '/unload_cmd', lambda m: setattr(self, 'unload_state', int(max(0, min(1, m.data)))), 1)
        self.create_subscription(Bool,  '/remote_req', lambda m: setattr(self, 'remote_req_state', bool(m.data)), 1)
        self.create_subscription(Bool,  '/dump_cmd',   lambda m: setattr(self, 'dump_state', bool(m.data)), 1)

        # CAN init
        self.can_bus = None
        if self.enable_can:
            if not HAVE_CAN:
                self.get_logger().error("python-can not installed")
            else:
                try:
                    self.can_bus = can.Bus(channel=self.can_interface, bustype='socketcan', fd=False)
                    self.get_logger().info(f"CAN enabled on {self.can_interface}, status_id=0x{self.can_id_status:X}")
                except Exception as e:
                    self.get_logger().error(f"Open CAN failed: {e}")

        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.last_pre_kmh_sent = 0.0  # 记录上一周期发给 VCU 的预速度
        self.get_logger().info(
            f"Loaded {len(self.waypoints_xy)} waypoints. GPS:{self.gps_topic}, RTK:{self.rtk_heading_topic}"
        )

    # ---- helpers ----
    def _pub_global_path(self):
        t = self.get_clock().now().to_msg()
        self.global_path.header.stamp = t
        self.pub_global_path.publish(self.global_path)
        # 若尚未定位，则给 VCU 发一个“静止”状态
        if self.enable_can and self.can_bus and (self.cur_x is None or self.cur_y is None or self.cur_yaw is None):
            try:
                self.send_can_status(0.0, 0.0, self.vcu_speed_stop_kmh)
            except Exception:
                pass

    def load_csv(self, path) -> List[Tuple[int, float, float, float, int]]:
        wps = []
        with open(path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                if not row or row[0].startswith('#'):
                    continue
                try:
                    idx = int(row[0]); lat = float(row[1]); lon = float(row[2])
                    hdg = float(row[3]); pt = int(row[4]) if len(row) > 4 else 0
                    wps.append((idx, lat, lon, hdg, pt))
                except Exception:
                    continue
        return wps

    # ---- Inputs ----
    def on_drive_cmd(self, msg: UInt8):
        v = int(msg.data)
        if v == 1:
            self.drive_state = self.DS_RUNNING
            self.remote_req_state = False
            self.get_logger().info("Drive: RUNNING")
        elif v == 2:
            self.drive_state = self.DS_ESTOP
            self.remote_req_state = True
            self.get_logger().warn("Drive: ESTOP")
        else:
            self.drive_state = self.DS_PAUSED
            self.get_logger().info("Drive: PAUSED")

    def on_abort(self, msg: Bool):
        if bool(msg.data):
            self.drive_state = self.DS_ESTOP
            if self.link_remote_to_abort:
                self.remote_req_state = True
            self.get_logger().warn("Drive: ESTOP (via /abort True)")

    def on_task_done(self, msg: Bool):
        if bool(msg.data):
            if self.waiting_for_task:
                self.waiting_for_task = False
                if self.seg_idx < len(self.waypoints_xy) - 2:
                    self.seg_idx += 1
                self.drive_state = self.DS_RUNNING
                self.task_done_latch = False
                self._pub_waiting(False)
                self.action_override = False
                self.action_pre_kmh = 0.0
                self.action_angle_deg = 0.0
                self.get_logger().info("Task done: resume following")
            else:
                self.task_done_latch = True

    def _pub_waiting(self, v: bool):
        self.pub_task_wait.publish(Bool(data=bool(v)))

    # ---- Sensors ----
    def on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        alt = msg.altitude if msg.altitude == msg.altitude else 0.0
        x, y, _ = self.geo.lla_to_enu(msg.latitude, msg.longitude, alt)
        self.cur_x = x
        self.cur_y = y
        if self.dead_reckon:
            if self.dr_x is None:
                self.dr_x, self.dr_y = x, y
            else:
                a = max(0.0, min(1.0, self.dr_alpha))
                self.dr_x = a * self.dr_x + (1.0 - a) * x
                self.dr_y = a * self.dr_y + (1.0 - a) * y

    def on_imu(self, msg: Imu):
        if self.use_rtk_heading:
            return
        q = msg.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.cur_yaw = wrap_pi(yaw + self.yaw_offset)

    def on_rtk_heading(self, msg: Float32):
        hdg = float(msg.data)
        if not math.isfinite(hdg):
            return
        yaw_enu = heading_csv_deg_to_enu_rad(hdg)
        self.cur_yaw = wrap_pi(yaw_enu + self.yaw_offset)

    # ---- 主控制循环（离散 VCU 控制）----
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        now_sec = now.nanoseconds * 1e-9
        self.last_time = now

        if self.cur_x is None or self.cur_y is None or self.cur_yaw is None:
            return

        # DR integrate
        if self.dead_reckon:
            if self.dr_x is None:
                self.dr_x, self.dr_y = self.cur_x, self.cur_y
            else:
                v = 0.0
                if self.drive_state == self.DS_RUNNING:
                    if self.dr_use_gps_speed and (self.last_gps_speed is not None):
                        v = self.last_gps_speed
                self.dr_x += v * math.cos(self.cur_yaw) * dt
                self.dr_y += v * math.sin(self.cur_yaw) * dt

        px = self.dr_x if (self.dead_reckon and self.dr_x is not None) else self.cur_x
        py = self.dr_y if (self.dead_reckon and self.dr_y is not None) else self.cur_y

        # viz path（仅用于 RViz 中的轨迹显示）
        t = self.get_clock().now().to_msg()
        ps = PoseStamped()
        ps.header.stamp = t
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = float(px)
        ps.pose.position.y = float(py)
        ps.pose.orientation = yaw_to_quat(self.cur_yaw)
        self.traj_path.poses.append(ps)
        if len(self.traj_path.poses) > self.max_traj_len:
            self.traj_path.poses.pop(0)
        self.traj_path.header.stamp = t
        self.pub_traj_path.publish(self.traj_path)
        self.global_path.header.stamp = t
        self.pub_global_path.publish(self.global_path)

        n = len(self.waypoints_xy)
        if n == 0:
            return

        # latch task_done
        if self.waiting_for_task and self.task_done_latch:
            self.task_done_latch = False
            self.waiting_for_task = False
            if self.seg_idx < n - 2:
                self.seg_idx += 1
            self.drive_state = self.DS_RUNNING
            self._pub_waiting(False)
            self.action_override = False
            self.action_pre_kmh = 0.0
            self.action_angle_deg = 0.0
            self.get_logger().info("Task done latch: resume following")

        # 下一个路点的简单距离（用于 waiting/非运行态）
        next_idx = min(self.seg_idx + 1, n - 1) if n >= 2 else 0
        dist_to_next_simple = math.hypot(px - self.waypoints_xy[next_idx][0],
                                         py - self.waypoints_xy[next_idx][1])

        # 等待任务阶段：不做自动循迹，只发 override 或停止到 VCU
        if self.waiting_for_task:
            # 等待期间不允许继续 spin
            self.spin_state = 'IDLE'
            self.angle_sign = 0
            if self.action_override:
                hdg_cmd = self.cur_yaw + math.radians(self.action_angle_deg)
                pre_kmh = self.action_pre_kmh
            else:
                hdg_cmd = self.cur_yaw
                pre_kmh = self.vcu_speed_stop_kmh
            self.send_can_status(hdg_cmd, dist_to_next_simple, pre_kmh)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        # 非 RUNNING：始终停车 + 退出 SPIN 状态
        if self.drive_state != self.DS_RUNNING:
            self.spin_state = 'IDLE'
            self.angle_sign = 0
            self.send_can_status(self.cur_yaw, dist_to_next_simple, self.vcu_speed_stop_kmh)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        # 只有一个路点的情况：简化为“朝向该点/其朝向”的控制
        if n == 1:
            xj, yj, yaw_wp, pt_type = self.waypoints_xy[0]
            self.control_single_point(px, py, xj, yj, yaw_wp, pt_type, now_sec)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        # 自动对齐初始 yaw 到第一段方向（可选）
        if self.auto_align_yaw and not self.aligned and n >= 2:
            seg_hdg0 = math.atan2(self.waypoints_xy[1][1] - self.waypoints_xy[0][1],
                                  self.waypoints_xy[1][0] - self.waypoints_xy[0][0])
            self.yaw_offset += wrap_pi(seg_hdg0 - self.cur_yaw)
            self.aligned = True

        self.seg_idx = max(0, min(self.seg_idx, n - 2))
        i = self.seg_idx
        j = i + 1
        xi, yi, _, _ = self.waypoints_xy[i]
        xj, yj, yaw_wp, pt_type_j = self.waypoints_xy[j]

        sx = xj - xi
        sy = yj - yi
        seg_len = math.hypot(sx, sy)
        if seg_len < 1e-6:
            # 当前段几乎为点，跳到下一段
            if j < n - 1:
                self.seg_idx += 1
            else:
                # 最后一个点，停车
                self.send_can_status(self.cur_yaw, 0.0, self.vcu_speed_stop_kmh)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        seg_len2 = seg_len * seg_len
        rx = px - xi
        ry = py - yi
        tproj = max(0.0, min(1.0, (rx * sx + ry * sy) / seg_len2))
        proj_x = xi + tproj * sx
        proj_y = yi + tproj * sy
        cross = sx * ry - sy * rx
        cte = math.copysign(math.hypot(px - proj_x, py - proj_y), cross)
        dist_to_next = math.hypot(px - xj, py - yj)

        # 期望航向：任务点/终点附近用 CSV 指定方向，否则用前瞻点
        use_wp_heading = (
            self.use_csv_heading and
            dist_to_next < self.heading_align_dist and
            (pt_type_j == 1 or j == n - 1)
        )
        if use_wp_heading:
            hdg_des = yaw_wp
        else:
            la_t = min(1.0, tproj + (self.lookahead_dist / max(0.01, seg_len)))
            la_x = xi + la_t * sx
            la_y = yi + la_t * sy
            hdg_des = math.atan2(la_y - py, la_x - px)
        heading_error = wrap_pi(hdg_des - self.cur_yaw)
        heading_err_deg = math.degrees(heading_error)

        # 拐角几何（用于角点 SPIN 判断）
        has_next_seg = (j < n - 1)
        psi1 = math.atan2(sy, sx)
        if has_next_seg:
            sx2 = self.waypoints_xy[j + 1][0] - xj
            sy2 = self.waypoints_xy[j + 1][1] - yj
            psi2 = math.atan2(sy2, sx2)
            dpsi = wrap_pi(psi2 - psi1)
            hdg_err_to_next_deg = math.degrees(wrap_pi(psi2 - self.cur_yaw))
        else:
            psi2 = psi1
            dpsi = 0.0
            hdg_err_to_next_deg = heading_err_deg
        corner_angle_deg = math.degrees(dpsi)

        # 任务点：靠近时进入“到点+对正+等待”逻辑
        if pt_type_j == 1:
            hdg_err_to_wp = wrap_pi(yaw_wp - self.cur_yaw)
            hdg_err_to_wp_deg = math.degrees(hdg_err_to_wp)
            if dist_to_next <= self.wp_reached_dist:
                # 未对正：用原地大旋转对正（同样遵守 SPIN 规则）
                if abs(hdg_err_to_wp_deg) > self.stop_turn_tol_deg:
                    # 这里简单用 emergency spin 判定：把 heading_err_deg 换成对 wp 的误差传入
                    pre_kmh, angle_send_deg = self.decide_vcu_action(
                        heading_err_deg=hdg_err_to_wp_deg,
                        hdg_err_to_next_deg=hdg_err_to_wp_deg,
                        cte=cte,
                        dist_to_next=dist_to_next,
                        corner_angle_deg=0.0,
                        has_next_seg=False,
                        now_sec=now_sec
                    )
                    angle_send_deg = max(-self.vcu_angle_move_limit_deg,
                                         min(self.vcu_angle_move_limit_deg, angle_send_deg))
                    hdg_cmd_for_can = self.cur_yaw + math.radians(angle_send_deg)
                    self.send_can_status(hdg_cmd_for_can, dist_to_next, pre_kmh)
                    self.publish_map_to_odom(px, py, self.cur_yaw)
                    return
                else:
                    # 位置和姿态都满足，到点等待任务
                    self.waiting_for_task = True
                    self.drive_state = self.DS_PAUSED
                    self.spin_state = 'IDLE'
                    self.angle_sign = 0
                    self._pub_waiting(True)
                    self.send_can_status(self.cur_yaw, dist_to_next, self.vcu_speed_stop_kmh)
                    self.publish_map_to_odom(px, py, self.cur_yaw)
                    return
            # 未到点，仍按常规离散控制继续靠近

        # 普通点：段切换
        if pt_type_j != 1:
            if (self.seg_idx < n - 2) and (tproj > self.t_advance_min and dist_to_next < self.advance_when_close):
                # 足够接近下一路点，切到下一段
                self.seg_idx += 1
                self.publish_map_to_odom(px, py, self.cur_yaw)
                return

        # 最后一段末端减速 + 停车（仍然是离散档速度）
        is_last_segment = (self.seg_idx == n - 2)
        if is_last_segment:
            # 很靠近终点：停车
            if dist_to_next < 0.5:
                self.spin_state = 'IDLE'
                self.angle_sign = 0
                self.send_can_status(self.cur_yaw, dist_to_next, self.vcu_speed_stop_kmh)
                self.publish_map_to_odom(px, py, self.cur_yaw)
                return
            # 进入 1 m 左右区域：强制慢速直行靠近
            if dist_to_next < max(0.8, 0.5 * self.advance_when_close):
                self.spin_state = 'IDLE'
                self.angle_sign = 0
                self.send_can_status(self.cur_yaw, dist_to_next, self.vcu_speed_slow_kmh)
                self.publish_map_to_odom(px, py, self.cur_yaw)
                return

        # ---- 核心：基于误差的 VCU 离散动作决策 ----
        pre_kmh, angle_send_deg = self.decide_vcu_action(
            heading_err_deg=heading_err_deg,
            hdg_err_to_next_deg=hdg_err_to_next_deg,
            cte=cte,
            dist_to_next=dist_to_next,
            corner_angle_deg=corner_angle_deg,
            has_next_seg=has_next_seg,
            now_sec=now_sec
        )

        # 限制角度在 VCU 允许范围内
        angle_send_deg = max(-self.vcu_angle_move_limit_deg,
                             min(self.vcu_angle_move_limit_deg, angle_send_deg))

        # 发送到 VCU：目标航向 = 当前 + 方向档
        hdg_cmd_for_can = self.cur_yaw + math.radians(angle_send_deg)
        self.send_can_status(hdg_cmd_for_can, dist_to_next, pre_kmh)

        # debug 输出：cte, heading_err_deg, seg_idx, dist_to_next, pre_kmh, angle_send_deg, spin_state
        dbg = Float32MultiArray()
        dbg.data = [
            float(cte),
            float(heading_err_deg),
            float(self.seg_idx),
            float(dist_to_next),
            float(pre_kmh),
            float(angle_send_deg),
            float({'IDLE': 0, 'PRE_STOP': 1, 'SPIN': 2}[self.spin_state])
        ]
        self.pub_dbg.publish(dbg)

        self.publish_map_to_odom(px, py, self.cur_yaw)

    # ---- 离散动作决策函数 + SPIN 状态机 ----
    def decide_vcu_action(self,
                           heading_err_deg: float,
                           hdg_err_to_next_deg: float,
                           cte: float,
                           dist_to_next: float,
                           corner_angle_deg: float,
                           has_next_seg: bool,
                           now_sec: float):
        """
        根据航向误差、横向误差、拐角几何，输出 (pre_kmh, angle_send_deg)
        angle_send_deg > 0 右转，<0 左转。
        """

        abs_hdg = abs(heading_err_deg)
        abs_hdg_next = abs(hdg_err_to_next_deg)

        # --- 0. SPIN 状态机优先级最高 ---

        # 0.1 PRE_STOP：发送两帧 0 速 0 角
        if self.spin_state == 'PRE_STOP':
            self.turn_mode = 'SPIN'
            self.spin_stop_frames_left -= 1
            if self.spin_stop_frames_left <= 0:
                # 进入真正 SPIN
                self.spin_state = 'SPIN'
            return self.vcu_speed_stop_kmh, 0.0  # 两帧都 0/0

        # 0.2 SPIN：速度 0，固定 ±spin_cmd 角
        if self.spin_state == 'SPIN':
            # 根据启动时记录的目标类型选择误差
            cur_err = abs_hdg_next if self.spin_use_next_seg else abs_hdg
            # 误差小于退出阈值，结束 SPIN
            if cur_err <= self.vcu_angle_spin_exit_deg:
                self.spin_state = 'IDLE'
                self.angle_sign = 0
                # 结束后转入 MOVE 决策
            else:
                self.turn_mode = 'SPIN'
                angle_deg = self.spin_sign * self.vcu_angle_spin_cmd_deg
                return self.vcu_speed_stop_kmh, angle_deg

        # --- 1. 只有在 IDLE 才会考虑启动新的 SPIN ---
        self.turn_mode = 'MOVE'

        # 1.1 拐角区域判定（仅角度和距离，不直接触发 SPIN）
        is_corner_area = False
        if self.strict_corner_mode and has_next_seg:
            if abs(math.radians(corner_angle_deg)) >= self.corner_sharp_rad_strict \
               and dist_to_next <= self.corner_start_dist:
                is_corner_area = True

        # 1.2 只有当误差也足够大时，角点才触发 SPIN
        want_spin_corner = is_corner_area and (abs_hdg_next >= self.vcu_angle_spin_enter_deg)

        # 1.3 紧急 SPIN：当前路径方向误差过大
        want_spin_emergency = abs_hdg >= self.emergency_spin_hdg_deg

        # 1.4 由 MOVE 进入 SPIN 状态机
        if self.spin_state == 'IDLE' and (want_spin_corner or want_spin_emergency):
            # 根据前一帧发送速度决定是否需要 PRE_STOP
            already_stop = abs(self.last_pre_kmh_sent) < 0.1
            if want_spin_corner:
                self.spin_use_next_seg = True
                self.spin_sign = 1.0 if hdg_err_to_next_deg > 0.0 else -1.0
            else:
                self.spin_use_next_seg = False
                self.spin_sign = 1.0 if heading_err_deg > 0.0 else -1.0

            if already_stop:
                # 已经是 0 速，直接进入 SPIN
                self.spin_state = 'SPIN'
                angle_deg = self.spin_sign * self.vcu_angle_spin_cmd_deg
                self.turn_mode = 'SPIN'
                return self.vcu_speed_stop_kmh, angle_deg
            else:
                # 从低速/高速切换到原地 SPIN：先发两帧 0 速 0 角
                self.spin_state = 'PRE_STOP'
                self.spin_stop_frames_left = 2
                self.turn_mode = 'SPIN'
                return self.vcu_speed_stop_kmh, 0.0

        # --- 2. MOVE 模式：根据误差大小选择直行/小弯/大弯 ---

        # 基本对中：认为无需转向，直行快
        if abs_hdg <= self.hdg_deadband_deg and abs(cte) <= self.cte_deadband_m:
            self.angle_sign = 0
            return self.vcu_speed_fast_kmh, 0.0  # 8km/h, 0°

        # 更新方向符号，带滞回，避免左右抖动
        sign = self.update_angle_sign(heading_err_deg)

        # 误差分段：
        #  1) < small_thresh: 用 8km/h + ±10° 做轻微修正
        #  2) [small_thresh, large_thresh): 用 4km/h + ±10° 慢速修正
        #  3) ≥ large_thresh: 用 4km/h + ±20° 慢速大弯
        if abs_hdg < self.vcu_turn_small_thresh_deg:
            angle_deg = sign * self.vcu_turn_small_cmd_deg
            pre_kmh = self.vcu_speed_fast_kmh
        elif abs_hdg < self.vcu_turn_large_thresh_deg:
            angle_deg = sign * self.vcu_turn_small_cmd_deg
            pre_kmh = self.vcu_speed_slow_kmh
        else:
            angle_deg = sign * self.vcu_turn_large_cmd_deg
            pre_kmh = self.vcu_speed_slow_kmh

        # 若横向误差较大，可强制降速
        if abs(cte) > 0.5:
            pre_kmh = min(pre_kmh, self.vcu_speed_slow_kmh)

        return pre_kmh, angle_deg

    def update_angle_sign(self, heading_err_deg: float) -> int:
        """带滞回的左右方向判定，避免在 0° 附近抖动"""
        if self.angle_sign >= 0 and heading_err_deg > self.sign_hysteresis_deg:
            self.angle_sign = +1
        elif self.angle_sign <= 0 and heading_err_deg < -self.sign_hysteresis_deg:
            self.angle_sign = -1
        if self.angle_sign == 0:
            self.angle_sign = 1 if heading_err_deg >= 0.0 else -1
        return self.angle_sign

    def control_single_point(self, px, py, xj, yj, yaw_wp, pt_type, now_sec: float):
        """只有一个路点的情况的离散控制"""
        dist = math.hypot(px - xj, py - yj)
        seg_hdg = math.atan2(yj - py, xj - px)
        use_wp_heading = (
            self.use_csv_heading and
            dist < self.heading_align_dist and
            (pt_type == 1)
        )
        hdg_des = yaw_wp if use_wp_heading else seg_hdg
        heading_error = wrap_pi(hdg_des - self.cur_yaw)
        heading_err_deg = math.degrees(heading_error)

        # 任务点：到点后对正+等待
        if pt_type == 1 and dist <= self.wp_reached_dist:
            hdg_err_to_wp = wrap_pi(yaw_wp - self.cur_yaw)
            hdg_err_to_wp_deg = math.degrees(hdg_err_to_wp)
            if abs(hdg_err_to_wp_deg) > self.stop_turn_tol_deg:
                # 利用 SPIN 状态机对正
                pre_kmh, angle_deg = self.decide_vcu_action(
                    heading_err_deg=hdg_err_to_wp_deg,
                    hdg_err_to_next_deg=hdg_err_to_wp_deg,
                    cte=0.0,
                    dist_to_next=dist,
                    corner_angle_deg=0.0,
                    has_next_seg=False,
                    now_sec=now_sec
                )
                angle_deg = max(-self.vcu_angle_move_limit_deg,
                                min(self.vcu_angle_move_limit_deg, angle_deg))
                self.send_can_status(self.cur_yaw + math.radians(angle_deg),
                                     dist,
                                     pre_kmh)
            else:
                self.waiting_for_task = True
                self.drive_state = self.DS_PAUSED
                self.spin_state = 'IDLE'
                self.angle_sign = 0
                self._pub_waiting(True)
                self.send_can_status(self.cur_yaw, dist, self.vcu_speed_stop_kmh)
            return

        # 非任务点，或者未到点：复用多段逻辑中的决策
        pre_kmh, angle_send_deg = self.decide_vcu_action(
            heading_err_deg=heading_err_deg,
            hdg_err_to_next_deg=heading_err_deg,
            cte=0.0,
            dist_to_next=dist,
            corner_angle_deg=0.0,
            has_next_seg=False,
            now_sec=now_sec
        )
        angle_send_deg = max(-self.vcu_angle_move_limit_deg,
                             min(self.vcu_angle_move_limit_deg, angle_send_deg))
        hdg_cmd_for_can = self.cur_yaw + math.radians(angle_send_deg)
        self.send_can_status(hdg_cmd_for_can, dist, pre_kmh)

    def publish_map_to_odom(self, px: float, py: float, yaw_map_base: float):
        try:
            tf_ob = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
            tx_ob = tf_ob.transform.translation.x
            ty_ob = tf_ob.transform.translation.y
            rz = tf_ob.transform.rotation
            yaw_ob = math.atan2(
                2.0 * (rz.w * rz.z + rz.x * rz.y),
                1.0 - 2.0 * (rz.y * rz.y + rz.z * rz.z)
            )

            yaw_mo = wrap_pi(yaw_map_base - yaw_ob)
            cos_mo = math.cos(yaw_mo)
            sin_mo = math.sin(yaw_mo)
            tmo_x = px - (cos_mo * tx_ob - sin_mo * ty_ob)
            tmo_y = py - (sin_mo * tx_ob + cos_mo * ty_ob)

            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.map_frame
            tf.child_frame_id = self.odom_frame
            tf.transform.translation.x = float(tmo_x)
            tf.transform.translation.y = float(tmo_y)
            tf.transform.translation.z = 0.0
            tf.transform.rotation = yaw_to_quat(yaw_mo)
            self.tfb.sendTransform(tf)
        except Exception:
            pass

    # --- CAN helpers ---
    def send_can_status(self, hdg_des_rad, dist_to_next_m, pre_kmh):
        if not (self.enable_can and self.can_bus):
            return

        # 若处于 ESTOP：强制速度 0、方向 0
        if self.drive_state == self.DS_ESTOP:
            pre_kmh = self.vcu_speed_stop_kmh
            dist_to_next_m = 0.0
            if self.cur_yaw is not None:
                hdg_des_rad = self.cur_yaw

        # 记录上一周期预速度（用于逻辑判断）
        try:
            self.last_pre_kmh_sent = float(pre_kmh)
        except Exception:
            self.last_pre_kmh_sent = 0.0

        # 距离量化：分辨率 0.2 m, 最大 150 * 0.2 = 30 m
        dist_raw = int(round(max(0.0, dist_to_next_m) / 0.2))
        dist_raw = max(0, min(150, dist_raw))

        # 方向角编码：根据目标航向与当前航向的差值
        if self.cur_yaw is None:
            angle_raw = 180
        else:
            heading_error = wrap_pi(hdg_des_rad - self.cur_yaw)
            steer_deg = -math.degrees(heading_error)  # 正为右转
            while steer_deg >= 180.0:
                steer_deg -= 360.0
            while steer_deg < -180.0:
                steer_deg += 360.0
            angle_raw = int(round(steer_deg + 180.0))
            angle_raw = max(0, min(359, angle_raw))

        # 预速度编码
        try:
            raw = (float(pre_kmh) - self.vcu_speed_raw_offset_kmh) / self.vcu_speed_raw_res_kmh_per_lsb
            speed_raw = int(round(raw))
        except Exception:
            speed_raw = 0
        speed_raw = max(0, min(255, speed_raw))

        pick_val = int(max(0, min(1, self.pick_state)))
        unload_val = int(max(0, min(1, self.unload_state)))
        remote_bit = 1 if self.remote_req_state else 0
        dump_bit = 1 if self.dump_state else 0
        byte5 = (pick_val & 0x1) | ((unload_val & 0x1) << 1) | (remote_bit << 4) | (dump_bit << 5)
        estop_bit = 1 if self.drive_state == self.DS_ESTOP else 0
        drive_bit = 1 if self.drive_state == self.DS_RUNNING else 0
        byte6 = (estop_bit & 0x1) | ((drive_bit & 0x1) << 1)

        payload = bytearray(8)
        struct.pack_into('<H', payload, 0, dist_raw)
        struct.pack_into('<H', payload, 2, angle_raw)
        payload[4] = speed_raw
        payload[5] = byte5
        payload[6] = byte6
        payload[7] = 0x00

        try:
            msg = can.Message(arbitration_id=self.can_id_status,
                              data=payload,
                              is_extended_id=self.can_extended)
            self.can_bus.send(msg, timeout=0.001)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPIDFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
