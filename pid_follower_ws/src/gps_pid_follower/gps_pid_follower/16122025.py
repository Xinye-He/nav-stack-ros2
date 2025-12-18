import math
import csv
import struct
from typing import List, Tuple, Union

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Quaternion
from std_msgs.msg import Bool, Float32MultiArray, UInt8, Float32
from nav_msgs.msg import Path

import tf2_ros

# optional: python-can for SocketCAN
try:
    import can
    HAVE_CAN = True
except Exception:
    HAVE_CAN = False


def quat_to_yaw(qx, qy, qz, qw):
    # ENU: yaw around +Z (CCW positive)
    s = 2.0 * (qw * qz + qx * qy)
    c = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(s, c)


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    half = 0.5 * yaw
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def heading_csv_deg_to_enu_rad(hdg_deg: float) -> float:
    # 输入：0°=北，90°=东，顺时针为正 -> 输出 ENU: 0=东，90°=北，逆时针为正
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


class PID:
    def __init__(self, kp=0.8, ki=0.0, kd=0.3, i_limit=3.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0.0
        self.prev = None
        self.i_limit = i_limit

    def reset(self):
        self.i = 0.0
        self.prev = None

    def step(self, error, dt):
        if dt <= 0.0:
            dt = 1e-3
        de = 0.0 if self.prev is None else (error - self.prev) / dt
        self.prev = error
        self.i += error * dt
        self.i = max(-self.i_limit, min(self.i, self.i_limit))
        return self.kp * error + self.ki * self.i + self.kd * de


class WaypointPIDFollower(Node):
    DS_PAUSED = 0
    DS_RUNNING = 1
    DS_ESTOP = 2

    def __init__(self):
        super().__init__('waypoint_pid_follower')

        # I/O topics
        self.declare_parameter('path_csv', '/root/pid_follower.ws/points.csv')
        self.declare_parameter('gps_topic', '/gps/fix')
        self.declare_parameter('imu_topic', '/imu/data')  # 仅在未启用RTK航向时使用
        self.declare_parameter('cmd_topic', '/cmd_vel')

        # Motion/limits
        self.declare_parameter('target_speed', 2.0)
        self.declare_parameter('max_yaw_rate', 2.0)
        self.declare_parameter('advance_when_close', 2.5)
        self.declare_parameter('t_advance_min', 0.9)
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('auto_align_yaw', False)

        # Heading policy
        self.declare_parameter('use_csv_heading', True)
        self.declare_parameter('align_heading_only_at_task_points', True)
        self.declare_parameter('heading_align_dist', 1.0)

        # Task point handling
        self.declare_parameter('wp_reached_dist', 1.0)
        self.declare_parameter('wp_heading_tol_deg', 10.0)
        self.declare_parameter('stop_turn_tol_deg', 5.0)
        self.declare_parameter('wait_for_task_done', True)
        self.declare_parameter('task_done_topic', '/task_done')

        # Controllers
        self.declare_parameter('k_heading', 1.3)
        self.declare_parameter('kp_cte', 0.8)
        self.declare_parameter('ki_cte', 0.0)
        self.declare_parameter('kd_cte', 0.6)
        self.declare_parameter('i_limit', 3.0)

        # Tracked-base helpers
        self.declare_parameter('turn_in_place_deg', 55.0)
        self.declare_parameter('min_speed', 0.5)
        self.declare_parameter('cte_slow_k', 0.1)
        self.declare_parameter('yaw_lpf_alpha', 0.4)

        # Lookahead
        self.declare_parameter('lookahead_dist', 2.0)

        # Frames & visualization
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')       # 本版发布 map->odom
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('traj_path_len', 2000)

        # CAN parameters（仅状态帧）
        self.declare_parameter('enable_can', True)
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_extended', True)  # 29-bit EID
        self.declare_parameter('can_id_status', 0x18FED188)
        self.declare_parameter('link_remote_to_abort', True)

        # DR parameters
        self.declare_parameter('dead_reckon', True)
        self.declare_parameter('dr_alpha', 0.2)            # GPS权重大一些（见 on_gps）
        self.declare_parameter('dr_use_cmd_vel', True)
        self.declare_parameter('dr_use_gps_speed', True)   # 推荐用GPS地速
        self.declare_parameter('dr_v_scale', 1.0)          # 指令速度标定系数

        # --- Heading source (RTK heading instead of IMU) ---
        self.declare_parameter('use_rtk_heading', True)
        self.declare_parameter('rtk_heading_topic', '/gps/heading_deg')

        # --- VCU angle/distance control (MOVE/SPIN unified) ---
        self.declare_parameter('vcu_enhanced_mode', True)
        self.declare_parameter('vcu_angle_move_limit_deg', 18.0)
        self.declare_parameter('vcu_angle_spin_enter_deg', 22.0)
        self.declare_parameter('vcu_angle_spin_exit_deg', 18.0)
        self.declare_parameter('vcu_angle_spin_cmd_deg', 30.0)
        self.declare_parameter('small_keep_fast_deg', 5.0)   # ≤5° 保持 2 m/s
        self.declare_parameter('turn_slow_deg', 20.0)        # >5° 触发 1 m/s
        self.declare_parameter('corner_spin_deg', 45.0)      # 大拐角预判阈值（deg）
        self.declare_parameter('preturn_trigger_dist', 3.0)  # 3~5 m
        self.declare_parameter('angle_lpf_alpha_cmd', 0.5)
        self.declare_parameter('enable_virtual_distance', False)
        self.declare_parameter('virt_spin_dist', 8.0)

        # --- Strict corner mode (turn only near the corner) ---
        self.declare_parameter('strict_corner_mode', True)
        self.declare_parameter('corner_start_dist', 1.0)             # 拐点内1米开始转向
        self.declare_parameter('corner_sharp_deg_strict', 25.0)      # 多少度算拐点
        self.declare_parameter('strict_hold_deg',1.0)               # 直线段纠偏角上限（≤5°）

        # --- Emergency spin when heading error is large ---
        self.declare_parameter('emergency_spin_hdg_deg', 45.0)       # 航向误差≥该值立即原地转

        # Read params
        path_csv = self.get_parameter('path_csv').get_parameter_value().string_value
        self.gps_topic = self.get_parameter('gps_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.target_speed = float(self.get_parameter('target_speed').value)
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
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

        self.k_heading = float(self.get_parameter('k_heading').value)
        kp = float(self.get_parameter('kp_cte').value)
        ki = float(self.get_parameter('ki_cte').value)
        kd = float(self.get_parameter('kd_cte').value)
        i_limit = float(self.get_parameter('i_limit').value)

        self.turn_in_place_th = math.radians(float(self.get_parameter('turn_in_place_deg').value))
        self.min_speed = float(self.get_parameter('min_speed').value)
        self.cte_slow_k = float(self.get_parameter('cte_slow_k').value)
        self.yaw_lpf_alpha = float(self.get_parameter('yaw_lpf_alpha').value)
        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.traj_path_len = int(self.get_parameter('traj_path_len').value)

        # CAN params parse
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

        # DR state
        self.dead_reckon = bool(self.get_parameter('dead_reckon').value)
        self.dr_alpha = float(self.get_parameter('dr_alpha').value)
        self.dr_use_cmd_vel = bool(self.get_parameter('dr_use_cmd_vel').value)
        self.dr_use_gps_speed = bool(self.get_parameter('dr_use_gps_speed').value)
        self.dr_v_scale = float(self.get_parameter('dr_v_scale').value)
        self.dr_x = None
        self.dr_y = None
        self.last_cmd_v = 0.0
        self.last_gps_speed = None

        # RTK heading source
        self.use_rtk_heading = bool(self.get_parameter('use_rtk_heading').value)
        self.rtk_heading_topic = self.get_parameter('rtk_heading_topic').value

        # VCU enhanced mode parameters
        self.vcu_enhanced_mode = bool(self.get_parameter('vcu_enhanced_mode').value)
        self.vcu_angle_move_limit_deg = float(self.get_parameter('vcu_angle_move_limit_deg').value)
        self.vcu_angle_spin_enter_deg = float(self.get_parameter('vcu_angle_spin_enter_deg').value)
        self.vcu_angle_spin_exit_deg  = float(self.get_parameter('vcu_angle_spin_exit_deg').value)
        self.vcu_angle_spin_cmd_deg   = float(self.get_parameter('vcu_angle_spin_cmd_deg').value)
        self.small_keep_fast_deg = float(self.get_parameter('small_keep_fast_deg').value)
        self.turn_slow_deg       = float(self.get_parameter('turn_slow_deg').value)
        self.corner_spin_deg     = float(self.get_parameter('corner_spin_deg').value)
        self.preturn_trigger_dist = float(self.get_parameter('preturn_trigger_dist').value)
        self.angle_lpf_alpha_cmd = float(self.get_parameter('angle_lpf_alpha_cmd').value)
        self.enable_virtual_distance = bool(self.get_parameter('enable_virtual_distance').value)
        self.virt_spin_dist      = float(self.get_parameter('virt_spin_dist').value)

        # Strict corner mode
        self.strict_corner_mode = bool(self.get_parameter('strict_corner_mode').value)
        self.corner_start_dist = float(self.get_parameter('corner_start_dist').value)
        self.corner_sharp_rad_strict = math.radians(float(self.get_parameter('corner_sharp_deg_strict').value))
        self.strict_hold_deg = float(self.get_parameter('strict_hold_deg').value)

        # Emergency spin
        self.emergency_spin_hdg_deg = float(self.get_parameter('emergency_spin_hdg_deg').value)

        # MOVE 能力（由 VCU 固定差速决定：ω_move=2.5°/s）
        self.omega_move = math.radians(2.5)  # rad/s
        self.kappa_move = self.omega_move / 1.0  # 慢速 1 m/s
        self.kappa_fast = self.omega_move / 2.0  # 快速 2 m/s
        self.turn_mode = 'MOVE'
        self.angle_cmd_deg_prev = 0.0

        if not path_csv:
            self.get_logger().error("path_csv is required.")
            raise SystemExit

        # Waypoints
        llh = self.load_csv(path_csv)
        if len(llh) < 1:
            self.get_logger().error("No valid waypoints in CSV")
            raise SystemExit
        llh.sort(key=lambda x: x[0])

        lat0, lon0 = llh[0][1], llh[0][2]
        self.geo = LLA2ENU(lat0, lon0, 0.0)

        self.waypoints_xy = []  # (x, y, yaw_wp_enu, pt_type)
        for _, lat, lon, hdg_deg_csv, pt_type in llh:
            x, y, _ = self.geo.lla_to_enu(lat, lon, 0.0)
            yaw_enu = heading_csv_deg_to_enu_rad(hdg_deg_csv)
            self.waypoints_xy.append((x, y, yaw_enu, int(pt_type)))

        # State
        self.cur_x = None
        # State
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

        # Controllers
        self.pid_cte = PID(kp=kp, ki=ki, kd=kd, i_limit=i_limit)
        self.yaw_rate_prev = 0.0

        # Subscriptions
        self.sub_gps = self.create_subscription(NavSatFix, self.gps_topic, self.on_gps, qos_profile_sensor_data)
        # Heading source: RTK heading (Float32 deg) or IMU quaternion
        if self.use_rtk_heading:
            self.sub_heading = self.create_subscription(Float32, self.rtk_heading_topic, self.on_rtk_heading, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribed GPS: {self.gps_topic} (sensor QoS), Heading(RTK): {self.rtk_heading_topic} (sensor QoS)")
        else:
            self.sub_imu = self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
            self.get_logger().info(f"Subscribed GPS: {self.gps_topic} (sensor QoS), IMU: {self.imu_topic} (sensor QoS)")

        # GPS ground speed for DR
        self.sub_gps_speed = self.create_subscription(Float32, '/gps/ground_speed_mps',
                                                      lambda m: setattr(self, 'last_gps_speed', float(m.data)),
                                                      qos_profile_sensor_data)

        # 监听 /cmd_vel（仅记录线速度用于 DR）
        self.sub_cmd_mon = self.create_subscription(Twist, self.cmd_topic, self._on_cmd, 10)

        # Viz & TF
        qos_tl = QoSProfile(depth=1)
        qos_tl.history = QoSHistoryPolicy.KEEP_LAST
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.pub_global_path = self.create_publisher(Path, 'global_path', qos_tl)
        self.pub_traj_path   = self.create_publisher(Path, 'traj_path', 10)

        self.tfb = tf2_ros.TransformBroadcaster(self)      # 我们发布 map->odom
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

        # 每秒重发 global_path
        self.timer_global_path = self.create_timer(1.0, self._pub_global_path)

        # Debug
        self.pub_dbg = self.create_publisher(Float32MultiArray, 'pid_debug', 10)

        # cmd publisher
        self.pub_cmd = self.create_publisher(Twist, self.cmd_topic, 10)

        # Commands & task topics
        self.sub_task  = self.create_subscription(Bool,  self.task_done_topic, self.on_task_done, 1)
        self.sub_drive = self.create_subscription(UInt8, '/drive_cmd',         self.on_drive_cmd, 1)
        self.sub_abort = self.create_subscription(Bool,  '/abort',             self.on_abort,     1)
        self.create_subscription(UInt8, '/pick_cmd',   lambda m: setattr(self, 'pick_state',   int(max(0, min(3, m.data)))), 1)
        self.create_subscription(UInt8, '/unload_cmd', lambda m: setattr(self, 'unload_state', int(max(0, min(3, m.data)))), 1)
        self.create_subscription(Bool,  '/remote_req', lambda m: setattr(self, 'remote_req_state', bool(m.data)), 1)
        self.create_subscription(Bool,  '/dump_cmd',   lambda m: setattr(self, 'dump_state', bool(m.data)), 1)

        # CAN init
        self.can_bus = None
        if self.enable_can:
            if not HAVE_CAN:
                self.get_logger().error("python-can not installed. apt-get install -y python3-can")
            else:
                try:
                    self.can_bus = can.Bus(channel=self.can_interface, bustype='socketcan', fd=False)
                    self.get_logger().info(f"CAN enabled on {self.can_interface}, status_id=0x{self.can_id_status:X}")
                except Exception as e:
                    self.get_logger().error(f"Open CAN failed: {e}")

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(f"Loaded {len(self.waypoints_xy)} waypoints (sorted by index).")
        self.get_logger().info(f"GPS: {self.gps_topic}, CMD: {self.cmd_topic}, target_speed={self.target_speed} m/s")
        self.get_logger().info("Drive state = PAUSED. Keys: 's'(RUN), 'p'(PAUSE), 'x'(ESTOP), 'f'(/task_done)")

    # ---- helpers ----
    def _on_cmd(self, msg: Twist):
        if self.dr_use_cmd_vel:
            self.last_cmd_v = float(msg.linear.x)

    def _pub_global_path(self):
        t = self.get_clock().now().to_msg()
        self.global_path.header.stamp = t
        self.pub_global_path.publish(self.global_path)
        if self.enable_can and self.can_bus and (self.cur_x is None or self.cur_y is None or self.cur_yaw is None):
            try:
                self.send_can_status(0.0, 0.0, 0.0)
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
                    idx = int(row[0])
                    lat = float(row[1]); lon = float(row[2])
                    hdg = float(row[3])
                    pt = int(row[4]) if len(row) > 4 else 0
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
                self.pid_cte.reset()
                self.get_logger().info("Task done received: resume following NOW")
            else:
                self.task_done_latch = True

    # ---- Sensors ----
    def on_gps(self, msg: NavSatFix):
        if msg.status.status < 0:
            return
        alt = msg.altitude
        if alt != alt:
            alt = 0.0
        x, y, _ = self.geo.lla_to_enu(msg.latitude, msg.longitude, alt)
        self.cur_x = x
        self.cur_y = y
        # DR 融合：dr_alpha 表示对历史DR的权重，(1-a)为GPS权重
        if self.dead_reckon:
            if self.dr_x is None:
                self.dr_x, self.dr_y = x, y
            else:
                a = max(0.0, min(1.0, self.dr_alpha))
                self.dr_x = a * self.dr_x + (1.0 - a) * x
                self.dr_y = a * self.dr_y + (1.0 - a) * y

    def on_imu(self, msg: Imu):
        # 若启用 RTK 航向，则忽略 IMU 航向
        if self.use_rtk_heading:
            return
        yaw = quat_to_yaw(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.cur_yaw = wrap_pi(yaw + self.yaw_offset)

    def on_rtk_heading(self, msg: Float32):
        # /gps/heading_deg: 0°=北，顺时针为正
        hdg = float(msg.data)
        if not math.isfinite(hdg):
            return
        yaw_enu = heading_csv_deg_to_enu_rad(hdg)
        self.cur_yaw = wrap_pi(yaw_enu + self.yaw_offset)

    # ---- Main control loop ----
    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3
        self.last_time = now

        if self.cur_x is None or self.cur_y is None or self.cur_yaw is None:
            return

        # DR 积分（仅 RUNNING 时积分，防漂）
        if self.dead_reckon:
            if self.dr_x is None:
                self.dr_x, self.dr_y = self.cur_x, self.cur_y
            else:
                v = 0.0
                if self.drive_state == self.DS_RUNNING:
                    if self.dr_use_gps_speed and (self.last_gps_speed is not None):
                        v = self.last_gps_speed
                    elif self.dr_use_cmd_vel:
                        v = self.last_cmd_v * self.dr_v_scale
                self.dr_x += v * math.cos(self.cur_yaw) * dt
                self.dr_y += v * math.sin(self.cur_yaw) * dt

        # 统一位置（优先 DR）
        px = self.dr_x if (self.dead_reckon and self.dr_x is not None) else self.cur_x
        py = self.dr_y if (self.dead_reckon and self.dr_y is not None) else self.cur_y

        # Viz paths (map frame)
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

        # latch task_done
        if self.waiting_for_task and self.task_done_latch:
            self.task_done_latch = False
            self.waiting_for_task = False
            if self.seg_idx < n - 2:
                self.seg_idx += 1
            self.drive_state = self.DS_RUNNING
            self.pid_cte.reset()
            self.get_logger().info("Task done latch: resume following")

        # 距离到下一个点（用于CAN）
        next_idx = min(self.seg_idx + 1, n - 1) if n >= 2 else 0
        dist_to_next = math.hypot(px - self.waypoints_xy[next_idx][0],
                                  py - self.waypoints_xy[next_idx][1])

        # 等待/暂停时：真停（drive_bit=0），角度=0，距离=0
        if self.waiting_for_task:
            self.publish_cmd(0.0, 0.0)
            self.send_can_status(self.cur_yaw, 0.0, 0.0)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        if self.drive_state != self.DS_RUNNING:
            self.publish_cmd(0.0, 0.0)
            self.send_can_status(self.cur_yaw, 0.0, 0.0)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        # ---- RUNNING ----
        if n == 1:
            xj, yj, yaw_wp, pt_type = self.waypoints_xy[0]
            self.track_single(xj, yj, yaw_wp, pt_type)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        if self.auto_align_yaw and not self.aligned and n >= 2:
            seg_hdg0 = math.atan2(self.waypoints_xy[1][1] - self.waypoints_xy[0][1],
                                  self.waypoints_xy[1][0] - self.waypoints_xy[0][0])
            self.yaw_offset += wrap_pi(seg_hdg0 - self.cur_yaw)
            self.aligned = True
            self.get_logger().info(f"Auto-aligned yaw_offset: {math.degrees(self.yaw_offset):.2f} deg")

        self.seg_idx = max(0, min(self.seg_idx, n - 2))
        i = self.seg_idx
        j = i + 1
        xi, yi, _, _ = self.waypoints_xy[i]
        xj, yj, yaw_wp, pt_type_j = self.waypoints_xy[j]

        sx = xj - xi
        sy = yj - yi
        seg_len = math.hypot(sx, sy)
        seg_len2 = seg_len * seg_len
        if seg_len2 < 1e-6:
            if j < n - 1:
                self.seg_idx += 1
            else:
                self.publish_cmd(0.0, 0.0)
                self.send_can_status(self.cur_yaw, 0.0, 0.0)
            self.publish_map_to_odom(px, py, self.cur_yaw)
            return

        rx = px - xi
        ry = py - yi
        tproj = (rx * sx + ry * sy) / seg_len2
        tproj = max(0.0, min(1.0, tproj))
        proj_x = xi + tproj * sx
        proj_y = yi + tproj * sy

        cross = sx * ry - sy * rx
        cte = math.copysign(math.hypot(px - proj_x, py - proj_y), cross)
        dist_to_next = math.hypot(px - xj, py - yj)

        # 期望航向：任务点附近可用CSV航向对齐，否则用前瞻点方向
        use_wp_heading = (self.use_csv_heading and dist_to_next < self.heading_align_dist
                          and (pt_type_j == 1 or j == n - 1))
        if use_wp_heading:
            hdg_des = yaw_wp
            la_x = xj; la_y = yj  # 用于投影
        else:
            t_la = min(1.0, tproj + (self.lookahead_dist / max(0.01, seg_len)))
            la_x = xi + t_la * sx
            la_y = yi + t_la * sy
            hdg_des = math.atan2(la_y - py, la_x - px)

        heading_error = wrap_pi(hdg_des - self.cur_yaw)

        # /cmd_vel（兼容 DR/仿真）
        yaw_rate_cmd = self.k_heading * heading_error + self.pid_cte.step(cte, dt)
        yaw_rate_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, yaw_rate_cmd))

        if abs(heading_error) > self.turn_in_place_th:
            self.pid_cte.reset()
            speed_cmd = 0.0
            yaw_rate_cmd = math.copysign(self.max_yaw_rate, heading_error)
        else:
            speed_cmd = self.target_speed
            scale = 1.0 - self.cte_slow_k * abs(cte)
            min_ratio = self.min_speed / max(0.05, self.target_speed)
            scale = max(min_ratio, min(1.0, scale))
            speed_cmd *= scale

        # 任务点进站
        if pt_type_j == 1:
            hdg_err_to_wp = wrap_pi(yaw_wp - self.cur_yaw)
            if dist_to_next <= self.wp_reached_dist:
                if abs(hdg_err_to_wp) > self.stop_turn_tol:
                    speed_cmd = 0.0
                    yaw_rate_cmd = math.copysign(min(self.max_yaw_rate, abs(hdg_err_to_wp) * self.k_heading), hdg_err_to_wp)
                else:
                    speed_cmd = 0.0
                    yaw_rate_cmd = 0.0
                    self.waiting_for_task = True
                    self.drive_state = self.DS_PAUSED
                    self.pid_cte.reset()
                    self.get_logger().info(f"Arrived task point {j}: paused (drive=0), waiting {self.task_done_topic}=True")
        else:
            # 段推进与末段停车（保留原逻辑）
            if (self.seg_idx < n - 2) and (tproj > self.t_advance_min and dist_to_next < self.advance_when_close):
                self.seg_idx += 1
                self.pid_cte.reset()
            elif self.seg_idx == n - 2 and dist_to_next < max(0.8, 0.5 * self.advance_when_close):
                speed_cmd = max(self.min_speed, min(speed_cmd, self.target_speed * (dist_to_next / 3.0)))
                if dist_to_next < 0.5:
                    speed_cmd = 0.0
                    yaw_rate_cmd = 0.0
                    self.seg_idx += 1
                    self.pid_cte.reset()

        # Yaw rate LPF（保持）
        yaw_rate_cmd = self.yaw_lpf_alpha * self.yaw_rate_prev + (1.0 - self.yaw_lpf_alpha) * yaw_rate_cmd
        self.yaw_rate_prev = yaw_rate_cmd

        # Debug
        dbg = Float32MultiArray()
        dbg.data = [float(cte), float(heading_error), float(self.seg_idx), float(dist_to_next), float(tproj)]
        self.pub_dbg.publish(dbg)

        # 发布 /cmd_vel（兼容 DR 或仿真）
        self.publish_cmd(speed_cmd, yaw_rate_cmd)

        # ---- VCU：严格拐点模式 + 紧急对齐 ----
        if self.vcu_enhanced_mode:
            if self.strict_corner_mode:
                # 当前段与下一段航向
                is_strict_corner = False
                psi1 = math.atan2(sy, sx)
                psi2 = psi1
                if j < n - 1:
                    sx2 = self.waypoints_xy[j+1][0] - xj
                    sy2 = self.waypoints_xy[j+1][1] - yj
                    psi2 = math.atan2(sy2, sx2)
                    dpsi = wrap_pi(psi2 - psi1)
                    if abs(dpsi) >= self.corner_sharp_rad_strict and dist_to_next <= self.corner_start_dist:
                        is_strict_corner = True

                hdg_err_deg = math.degrees(heading_error)
                want_spin_emergency = abs(hdg_err_deg) >= self.emergency_spin_hdg_deg

                if want_spin_emergency:
                    # 紧急对齐：误差大，立即原地转
                    self.turn_mode = 'SPIN'
                    angle_cmd_deg = math.copysign(self.vcu_angle_spin_cmd_deg, hdg_err_deg)
                elif is_strict_corner:
                    # 拐点1m内且夹角大：对齐到下一段
                    self.turn_mode = 'SPIN'
                    hdg_err_to_next = wrap_pi(psi2 - self.cur_yaw)
                    angle_cmd_deg = math.copysign(self.vcu_angle_spin_cmd_deg, math.degrees(hdg_err_to_next))
                else:
                    # 远离拐点：严格沿当前段，小角度纠偏（≤5°）
                    self.turn_mode = 'MOVE'
                    angle_cmd_deg = math.copysign(min(self.strict_hold_deg, 5.0), hdg_err_deg)
                    angle_cmd_deg = max(-self.vcu_angle_move_limit_deg,
                                        min(self.vcu_angle_move_limit_deg, angle_cmd_deg))

                # 角度低通
                a = max(0.0, min(1.0, self.angle_lpf_alpha_cmd))
                angle_cmd_deg = a * self.angle_cmd_deg_prev + (1.0 - a) * angle_cmd_deg
                self.angle_cmd_deg_prev = angle_cmd_deg

                # 距离（一般保留真实距离；如需可在 SPIN 时压到 virt_spin_dist）
                dist_for_vcu = dist_to_next
                if self.enable_virtual_distance and self.turn_mode == 'SPIN':
                    dist_for_vcu = min(dist_for_vcu, self.virt_spin_dist)

                # 发给 VCU：保证正=右转、负=左转
                hdg_cmd_for_can = self.cur_yaw + math.radians(angle_cmd_deg)
                self.send_can_status(hdg_cmd_for_can, dist_for_vcu, speed_cmd)
            else:
                # 回退：直接用 hdg_des
                self.send_can_status(hdg_des, dist_to_next, speed_cmd)
        else:
            self.send_can_status(hdg_des, dist_to_next, speed_cmd)

        # 发布 map->odom
        self.publish_map_to_odom(px, py, self.cur_yaw)

    def publish_map_to_odom(self, px: float, py: float, yaw_map_base: float):
        """
        发布 map->odom，使 map->odom * odom->base_link = map->base_link（我们算出的位姿）
        yaw_mo = yaw_mb - yaw_ob
        tmo = tmb - Rmo * tob
        """
        try:
            tf_ob = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, Time())
            tx_ob = tf_ob.transform.translation.x
            ty_ob = tf_ob.transform.translation.y
            rz = tf_ob.transform.rotation
            yaw_ob = math.atan2(2.0 * (rz.w * rz.z + rz.x * rz.y), 1.0 - 2.0 * (rz.y * rz.y + rz.z * rz.z))

            yaw_mo = wrap_pi(yaw_map_base - yaw_ob)
            cos_mo = math.cos(yaw_mo); sin_mo = math.sin(yaw_mo)
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
            # odom->base_link 暂不可用时先跳过
            pass

    def track_single(self, xj, yj, yaw_wp, pt_type):
        # 单点导航使用 DR 位置
        px = self.dr_x if (self.dead_reckon and self.dr_x is not None) else self.cur_x
        py = self.dr_y if (self.dead_reckon and self.dr_y is not None) else self.cur_y
        dist = math.hypot(px - xj, py - yj)

        seg_hdg = math.atan2(yj - py, xj - px)
        use_wp_heading = (self.use_csv_heading and dist < self.heading_align_dist and (pt_type == 1))
        hdg_des = yaw_wp if use_wp_heading else seg_hdg
        heading_error = wrap_pi(hdg_des - self.cur_yaw)

        # /cmd_vel（兼容 DR/仿真）
        yaw_rate_cmd = max(-self.max_yaw_rate, min(self.max_yaw_rate, self.k_heading * heading_error))
        if abs(heading_error) > self.turn_in_place_th:
            speed_cmd = 0.0
            yaw_rate_cmd = math.copysign(self.max_yaw_rate, heading_error)
            self.pid_cte.reset()
        else:
            speed_cmd = self.target_speed * max(self.min_speed / max(0.05, self.target_speed), min(1.0, dist / 3.0))

        if self.drive_state != self.DS_RUNNING:
            speed_cmd = 0.0; yaw_rate_cmd = 0.0
        self.publish_cmd(speed_cmd, yaw_rate_cmd)

        # ---- VCU：严格拐点 + 紧急对齐（单点简化）----
        if self.vcu_enhanced_mode:
            hdg_err_deg = math.degrees(heading_error)
            want_spin_emergency = abs(hdg_err_deg) >= self.emergency_spin_hdg_deg

            if want_spin_emergency and dist > self.corner_start_dist:
                # 远离目标且误差很大：紧急原地转
                self.turn_mode = 'SPIN'
                angle_cmd_deg = math.copysign(self.vcu_angle_spin_cmd_deg, hdg_err_deg)
            else:
                # 平时严格逼近目标：小角度纠偏；靠近目标且误差大可 SPIN
                angle_cmd_deg = math.copysign(min(self.strict_hold_deg, 5.0), hdg_err_deg)
                if dist <= self.corner_start_dist and abs(hdg_err_deg) >= self.vcu_angle_spin_enter_deg:
                    self.turn_mode = 'SPIN'
                    angle_cmd_deg = math.copysign(self.vcu_angle_spin_cmd_deg, hdg_err_deg)
                else:
                    self.turn_mode = 'MOVE'

            a = max(0.0, min(1.0, self.angle_lpf_alpha_cmd))
            angle_cmd_deg = a * self.angle_cmd_deg_prev + (1.0 - a) * angle_cmd_deg
            self.angle_cmd_deg_prev = angle_cmd_deg

            dist_for_vcu = dist
            if self.enable_virtual_distance and self.turn_mode == 'SPIN':
                dist_for_vcu = min(dist_for_vcu, self.virt_spin_dist)

            hdg_cmd_for_can = self.cur_yaw + math.radians(angle_cmd_deg)  # 正=右转，负=左转
            self.send_can_status(hdg_cmd_for_can, dist_for_vcu, speed_cmd)
        else:
            self.send_can_status(hdg_des, dist, speed_cmd)

        # 任务点进站（等待时 drive_bit=0 真停）
        if pt_type == 1 and dist <= self.wp_reached_dist:
            hdg_err_to_wp = wrap_pi(yaw_wp - self.cur_yaw)
            if abs(hdg_err_to_wp) > self.stop_turn_tol:
                self.publish_cmd(0.0, math.copysign(self.max_yaw_rate, hdg_err_to_wp))
            else:
                self.publish_cmd(0.0, 0.0)
                self.waiting_for_task = True
                self.drive_state = self.DS_PAUSED
                self.pid_cte.reset()

    def publish_cmd(self, speed, yaw_rate):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(yaw_rate)
        self.pub_cmd.publish(msg)

    # --- CAN helpers ---
    def send_can_status(self, hdg_des_rad, dist_to_next_m, speed_cmd_mps):
        if not (self.enable_can and self.can_bus):
            return
        dist_raw = int(round(max(0.0, dist_to_next_m) / 0.2)); dist_raw = max(0, min(150, dist_raw))
        if self.cur_yaw is None:
            angle_raw = 180
        else:
            heading_error = wrap_pi(hdg_des_rad - self.cur_yaw)
            steer_deg = -math.degrees(heading_error)
            while steer_deg >= 180.0: steer_deg -= 360.0
            while steer_deg < -180.0: steer_deg += 360.0
            angle_raw = int(round(steer_deg + 180.0)); angle_raw = max(0, min(359, angle_raw))
        kmh = float(speed_cmd_mps) * 3.6
        speed_raw = int(round((kmh + 10.0) / 0.5)); speed_raw = max(0, min(40, speed_raw))
        pick_val = int(max(0, min(3, self.pick_state)))
        unload_val = int(max(0, min(3, self.unload_state)))
        remote_bit = 1 if self.remote_req_state else 0
        dump_bit = 1 if self.dump_state else 0
        byte5 = (pick_val & 0x3) | ((unload_val & 0x3) << 2) | (remote_bit << 4) | (dump_bit << 5)
        estop_bit = 1 if self.drive_state == self.DS_ESTOP else 0
        drive_bit = 1 if self.drive_state == self.DS_RUNNING else 0
        byte6 = (estop_bit & 0x1) | ((drive_bit & 0x1) << 1) | (0 << 2)
        payload = bytearray(8)
        struct.pack_into('<H', payload, 0, dist_raw)
        struct.pack_into('<H', payload, 2, angle_raw)
        payload[4] = speed_raw; payload[5] = byte5; payload[6] = byte6; payload[7] = 0x00
        try:
            msg = can.Message(arbitration_id=self.can_id_status, data=payload, is_extended_id=self.can_extended)
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
