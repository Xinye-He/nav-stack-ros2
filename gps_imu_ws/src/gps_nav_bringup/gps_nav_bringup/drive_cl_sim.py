#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Empty
from gps_msgs.msg import AngleSpeed, GlobalPath
from geographiclib.geodesic import Geodesic


def quat_from_yaw(yaw_rad: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0,
                      z=math.sin(yaw_rad / 2.0),
                      w=math.cos(yaw_rad / 2.0))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def bearing_deg_to_yaw_rad(bearing_deg: float) -> float:
    # bearing: +CW from North; yaw: +CCW from East
    return math.radians(90.0 - bearing_deg)


def enu_to_ll(x_e: float, y_n: float, lat0: float, lon0: float):
    # ENU -> WGS84 via great-circle from datum
    s = math.hypot(x_e, y_n)
    azi = math.degrees(math.atan2(x_e, y_n))  # North=0, East=+90
    g = Geodesic.WGS84.Direct(lat0, lon0, azi, s)
    return g['lat2'], g['lon2']


class DriveClosedLoopSim(Node):
    """
    Sub /drive_cmd(AngleSpeed): heading_deg (bearing) + speed_mps
    Integrate unicycle model:
      yaw_dot = sat(k_yaw * (yaw_cmd - yaw), max_ang_vel)
      x_dot = v * cos(yaw), y_dot = v * sin(yaw)
    Pub /imu/data (orientation only), /gps/fix (lat/lon).
    """

    def __init__(self):
        super().__init__('drive_cl_sim')

        # Control & sim rates
        self.declare_parameter('yaw_gain', 3.5)
        self.declare_parameter('max_ang_vel', 2.0)
        self.declare_parameter('speed_cap_mps', 3.0)
        self.declare_parameter('sim_rate_hz', 50.0)
        self.declare_parameter('gps_rate_hz', 10.0)
        self.declare_parameter('imu_rate_hz', 50.0)

        # Datum & init pose
        self.declare_parameter('use_manual_datum', False)
        self.declare_parameter('datum_lat', 0.0)
        self.declare_parameter('datum_lon', 0.0)
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw_deg', 0.0)

        # Debug/step
        self.declare_parameter('step_mode', False)
        self.declare_parameter('debug_print', True)

        # Read params
        self.k_yaw = float(self.get_parameter('yaw_gain').value)
        self.w_max = float(self.get_parameter('max_ang_vel').value)
        self.v_cap = float(self.get_parameter('speed_cap_mps').value)
        self.sim_rate = float(self.get_parameter('sim_rate_hz').value)
        self.gps_rate = float(self.get_parameter('gps_rate_hz').value)
        self.imu_rate = float(self.get_parameter('imu_rate_hz').value)

        self.use_manual_datum = bool(self.get_parameter('use_manual_datum').value)
        self.lat0 = float(self.get_parameter('datum_lat').value)
        self.lon0 = float(self.get_parameter('datum_lon').value)

        self.x = float(self.get_parameter('init_x').value)
        self.y = float(self.get_parameter('init_y').value)
        self.yaw = math.radians(float(self.get_parameter('init_yaw_deg').value))

        self.step_mode = bool(self.get_parameter('step_mode').value)
        self.debug_print = bool(self.get_parameter('debug_print').value)

        # State
        self.yaw_cmd = self.yaw
        self.v_cmd = 0.0
        self.have_datum = self.use_manual_datum

        # I/O
        self.create_subscription(AngleSpeed, '/drive_cmd', self.on_cmd, 10)
        self.create_subscription(GlobalPath, '/global_path_geo', self.on_geo, 1)
        self.create_subscription(Empty, '/sim/step', self.on_step, 1)

        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 20)

        # Timers
        self.last_t = self.get_clock().now()
        self.last_fix_pub = 0.0
        self.last_imu_pub = 0.0
        self.last_dbg_pub = 0.0
        self._step_pending = False

        self.timer = self.create_timer(1.0 / max(1e-3, self.sim_rate), self.on_timer)

        self.get_logger().info(
            'drive_cl_sim started (closed-loop). Waiting /drive_cmd and datum (GlobalPath first point) ...'
        )

    def on_geo(self, msg: GlobalPath):
        if self.have_datum:
            return
        if msg.points:
            self.lat0 = float(msg.points[0].latitude)
            self.lon0 = float(msg.points[0].longitude)
            self.have_datum = True
            self.get_logger().info(f"datum from GlobalPath: lat0={self.lat0:.7f}, lon0={self.lon0:.7f}")

    def on_cmd(self, msg: AngleSpeed):
        self.yaw_cmd = bearing_deg_to_yaw_rad(float(msg.heading_deg))
        self.v_cmd = max(-self.v_cap, min(self.v_cap, float(msg.speed_mps)))

    def on_step(self, _msg: Empty):
        self._step_pending = True

    def on_timer(self):
        # Step mode
        if self.step_mode and not self._step_pending:
            return
        if self.step_mode:
            self._step_pending = False

        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        if dt <= 0.0 or dt > 1.0:
            dt = 1.0 / max(1e-3, self.sim_rate)
        self.last_t = now

        # Control law
        yaw_err = wrap_pi(self.yaw_cmd - self.yaw)
        w = max(-self.w_max, min(self.w_max, self.k_yaw * yaw_err))
        v = max(-self.v_cap, min(self.v_cap, self.v_cmd))

        # Integrate unicycle (ENU)
        self.yaw = wrap_pi(self.yaw + w * dt)
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        t_sec = now.nanoseconds * 1e-9

        # IMU (orientation only)
        if (t_sec - self.last_imu_pub) >= 1.0 / max(1e-3, self.imu_rate):
            self.last_imu_pub = t_sec
            imu = Imu()
            imu.header.stamp = now.to_msg()
            imu.header.frame_id = 'base_link'
            imu.orientation = quat_from_yaw(self.yaw)
            imu.orientation_covariance = [0.02, 0.0, 0.0,
                                          0.0, 0.02, 0.0,
                                          0.0, 0.0, 0.02]
            imu.angular_velocity_covariance = [-1.0] * 9
            imu.linear_acceleration_covariance = [-1.0] * 9
            self.pub_imu.publish(imu)

        # GPS (lat/lon)
        if self.have_datum and (t_sec - self.last_fix_pub) >= 1.0 / max(1e-3, self.gps_rate):
            self.last_fix_pub = t_sec
            lat, lon = enu_to_ll(self.x, self.y, self.lat0, self.lon0)
            fix = NavSatFix()
            fix.header.stamp = now.to_msg()
            fix.header.frame_id = 'base_link'
            fix.status.status = NavSatStatus.STATUS_FIX
            fix.status.service = NavSatStatus.SERVICE_GPS
            fix.latitude = float(lat)
            fix.longitude = float(lon)
            fix.altitude = 0.0
            fix.position_covariance = [1.0, 0.0, 0.0,
                                       0.0, 1.0, 0.0,
                                       0.0, 0.0, 4.0]
            fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.pub_fix.publish(fix)

        # Debug
        if self.debug_print:
            if self.step_mode or (t_sec - self.last_dbg_pub) >= 0.5:
                self.last_dbg_pub = t_sec
                bearing_cmd = 90.0 - math.degrees(self.yaw_cmd)
                bearing_cmd = (bearing_cmd + 180.0) % 360.0 - 180.0
                self.get_logger().info(
                    f"x={self.x:.2f} y={self.y:.2f} yaw={math.degrees(self.yaw):.1f}deg | "
                    f"cmd_v={self.v_cmd:.2f} cmd_head={bearing_cmd:.1f}deg(bearing) "
                    f"yaw_err={math.degrees(yaw_err):.1f}deg w={w:.2f}"
                )


def main():
    rclpy.init()
    node = DriveClosedLoopSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
