import math
import csv
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped, Quaternion
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Path
import tf2_ros

EARTH_R = 6378137.0  # meters
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def yaw_to_quat(yaw):
    # roll=pitch=0, yaw about +Z (ENU), CCW positive
    half = 0.5 * yaw
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(half)
    q.w = math.cos(half)
    return q

def heading_csv_deg_to_enu_rad(hdg_deg: float) -> float:
    # CSV: 0°=North, 90°=East, clockwise positive
    # ENU yaw: 0=East, 90°=North, CCW positive
    return wrap_pi(math.radians(90.0 - hdg_deg))

class GPSSim(Node):
    def __init__(self):
        super().__init__('sim_diffdrive_gps_imu')

        # Params
        self.declare_parameter('path_csv', '')
        self.declare_parameter('gps_topic', '/fix')
        self.declare_parameter('imu_topic', '/imu/data_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('sim_hz', 50.0)   # integration rate
        self.declare_parameter('gps_hz', 10.0)
        self.declare_parameter('imu_hz', 50.0)

        # Initial state (meters in ENU from CSV[0])
        self.declare_parameter('init_e', -5.0)   # east offset (m)
        self.declare_parameter('init_n', -5.0)   # north offset (m)
        self.declare_parameter('init_heading_deg_csv', 0.0)  # 0°=North, CW+

        # Noise (optional)
        self.declare_parameter('gps_std_m', 0.0)
        self.declare_parameter('yaw_std_deg', 0.0)

        self.path_csv = self.get_parameter('path_csv').get_parameter_value().string_value
        if not self.path_csv:
            self.get_logger().error("path_csv is required. Use --ros-args -p path_csv:=/path/to/points.csv")
            raise SystemExit

        self.gps_topic = self.get_parameter('gps_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.sim_dt = 1.0 / float(self.get_parameter('sim_hz').value)
        self.gps_dt = 1.0 / float(self.get_parameter('gps_hz').value)
        self.imu_dt = 1.0 / float(self.get_parameter('imu_hz').value)

        self.init_e = float(self.get_parameter('init_e').value)
        self.init_n = float(self.get_parameter('init_n').value)
        self.init_yaw = heading_csv_deg_to_enu_rad(float(self.get_parameter('init_heading_deg_csv').value))

        self.gps_std = float(self.get_parameter('gps_std_m').value)
        self.yaw_std = math.radians(float(self.get_parameter('yaw_std_deg').value))

        # Load CSV, get anchor (first waypoint lat/lon)
        self.wps = self.load_csv(self.path_csv)  # (idx, lat, lon, heading_deg_csv, pt_type)
        if len(self.wps) == 0:
            self.get_logger().error("No waypoints in CSV")
            raise SystemExit
        self.wps.sort(key=lambda x: x[0])
        self.lat0 = self.wps[0][1]
        self.lon0 = self.wps[0][2]
        self.lat0_rad = self.lat0 * DEG2RAD
        self.cos_lat0 = math.cos(self.lat0_rad)

        # Build a global path in ENU (approx local projection)
        self.global_path = Path()
        self.global_path.header.frame_id = self.map_frame
        for _, lat, lon, _, _ in self.wps:
            x = (lon - self.lon0) * DEG2RAD * EARTH_R * self.cos_lat0
            y = (lat - self.lat0) * DEG2RAD * EARTH_R
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            self.global_path.poses.append(ps)

        # State
        self.x = self.init_e
        self.y = self.init_n
        self.yaw = self.init_yaw
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.t_acc_gps = 0.0
        self.t_acc_imu = 0.0

        # ROS IO
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.pub_fix = self.create_publisher(NavSatFix, self.gps_topic, 10)
        self.pub_imu = self.create_publisher(Imu, self.imu_topic, 10)
        self.pub_path = self.create_publisher(Path, 'sim_path', 10)
        self.pub_global_path = self.create_publisher(Path, 'global_path', 1)
        self.tfb = tf2_ros.TransformBroadcaster(self)

        # Sim path buffer
        self.sim_path = Path()
        self.sim_path.header.frame_id = self.map_frame
        self.max_path_len = 2000

        # Timer
        self.timer = self.create_timer(self.sim_dt, self.step)

        self.get_logger().info(f"Sim started. CSV anchor lat0={self.lat0:.8f}, lon0={self.lon0:.8f}")
        self.get_logger().info(f"Init ENU offset: E={self.init_e} m, N={self.init_n} m, yaw={math.degrees(self.init_yaw):.1f} deg")

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
                    # header or bad row
                    continue
        return wps

    def on_cmd(self, msg: Twist):
        self.v_cmd = float(msg.linear.x)
        self.w_cmd = float(msg.angular.z)

    def add_noise(self, val, std):
        if std <= 0.0:
            return val
        # Simple normal approximation via sum of uniforms (12 uniforms - 6) * std
        s = 0.0
        import random
        for _ in range(12):
            s += random.random()
        n = (s - 6.0) * std
        return val + n

    def step(self):
        # Integrate kinematics
        dt = self.sim_dt
        self.yaw = wrap_pi(self.yaw + self.w_cmd * dt)
        self.x += self.v_cmd * math.cos(self.yaw) * dt
        self.y += self.v_cmd * math.sin(self.yaw) * dt

        # TF (map -> base_link)
        t = self.get_clock().now().to_msg()
        tf = TransformStamped()
        tf.header.stamp = t
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation = yaw_to_quat(self.yaw)
        self.tfb.sendTransform(tf)

        # Publish sim path
        ps = PoseStamped()
        ps.header.stamp = t
        ps.header.frame_id = self.map_frame
        ps.pose.position.x = self.x
        ps.pose.position.y = self.y
        ps.pose.orientation = yaw_to_quat(self.yaw)
        self.sim_path.header.stamp = t
        self.sim_path.poses.append(ps)
        if len(self.sim_path.poses) > self.max_path_len:
            self.sim_path.poses.pop(0)
        self.pub_path.publish(self.sim_path)

        # Publish global path (low rate is fine)
        self.global_path.header.stamp = t
        self.pub_global_path.publish(self.global_path)

        # GPS/IMU rates
        self.t_acc_gps += dt
        self.t_acc_imu += dt

        if self.t_acc_gps + 1e-9 >= self.gps_dt:
            self.t_acc_gps = 0.0
            self.publish_gps(t)

        if self.t_acc_imu + 1e-9 >= self.imu_dt:
            self.t_acc_imu = 0.0
            self.publish_imu(t)

    def publish_gps(self, stamp):
        # Convert ENU (x=east,y=north) to lat/lon (small area approx)
        lat_rad = self.lat0_rad + (self.y / EARTH_R)
        lon_rad = (self.lon0 * DEG2RAD) + (self.x / (EARTH_R * self.cos_lat0))
        lat = lat_rad * RAD2DEG
        lon = lon_rad * RAD2DEG

        lat = self.add_noise(lat, self.gps_std * RAD2DEG / EARTH_R)
        lon = self.add_noise(lon, self.gps_std * RAD2DEG / (EARTH_R * self.cos_lat0))

        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = 'gps'
        msg.status.status = NavSatStatus.STATUS_FIX  # 0
        msg.status.service = NavSatStatus.SERVICE_GPS  # 1
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        # Covariance (diagonal known)
        sigma = max(0.5, self.gps_std)  # meters
        msg.position_covariance = [
            sigma*sigma, 0.0, 0.0,
            0.0, sigma*sigma, 0.0,
            0.0, 0.0, 4.0  # altitude variance larger
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(msg)

    def publish_imu(self, stamp):
        yaw = self.add_noise(self.yaw, self.yaw_std)
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'imu_link'
        imu.orientation = yaw_to_quat(yaw)
        # orientation_covariance: -1 unknown is acceptable
        imu.orientation_covariance[0] = -1.0
        # Angular velocity (optional: just echo commanded yaw rate)
        imu.angular_velocity.z = float(self.w_cmd)
        self.pub_imu.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    node = GPSSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
