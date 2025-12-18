#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from geometry_msgs.msg import Quaternion
from gps_msgs.msg import GlobalPath
from geographiclib.geodesic import Geodesic

def quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))

def enu_to_ll(x, y, lat0, lon0):
    s = math.hypot(x, y)
    azi = math.degrees(math.atan2(x, y))  # 北为0，东正（与我们 bearing 定义一致）
    g = Geodesic.WGS84.Direct(lat0, lon0, azi, s)
    return g['lat2'], g['lon2']

class GpsImuSim(Node):
    def __init__(self):
        super().__init__('gps_imu_sim')
        self.declare_parameter('speed_mps', 1.0)
        self.v = float(self.get_parameter('speed_mps').value)

        self.path = []          # [(x,y,yaw), ...] 采样轨迹
        self.lat0 = None; self.lon0 = None
        self.idx = 0; self.offset = 0.0  # 当前段内的弧长偏移

        self.create_subscription(Path, '/global_path', self.on_path, 1)
        self.create_subscription(GlobalPath, '/global_path_geo', self.on_geo, 1)
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', 50)

        self.timer = self.create_timer(0.05, self.on_timer)  # 20Hz

    def on_geo(self, msg: GlobalPath):
        if msg.points:
            self.lat0 = msg.points[0].latitude
            self.lon0 = msg.points[0].longitude

    def on_path(self, msg: Path):
        self.path = []
        for p in msg.poses:
            x = p.pose.position.x; y = p.pose.position.y
            q = p.pose.orientation
            # 只用z-yaw
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            self.path.append((x,y,yaw))
        self.idx = 0; self.offset = 0.0

    def on_timer(self):
        if not self.path or self.lat0 is None:
            return
        dt = 0.05
        # 前进 v*dt 的弧长
        s_remain = self.v * dt
        while s_remain > 1e-6 and self.idx+1 < len(self.path):
            x0,y0,_ = self.path[self.idx]
            x1,y1,_ = self.path[self.idx+1]
            seg = math.hypot(x1-x0, y1-y0)
            if self.offset + s_remain < seg:
                self.offset += s_remain
                s_remain = 0.0
            else:
                s_remain -= (seg - self.offset)
                self.idx += 1
                self.offset = 0.0

        # 当前插值位置与切向朝向
        i = self.idx
        if i+1 < len(self.path):
            x0,y0,yaw0 = self.path[i]
            x1,y1,yaw1 = self.path[i+1]
            seg = max(1e-6, math.hypot(x1-x0, y1-y0))
            t = self.offset / seg
            x = x0 + t*(x1-x0); y = y0 + t*(y1-y0)
            # 插值 yaw（保持连续）
            dyaw = math.atan2(math.sin(yaw1-yaw0), math.cos(yaw1-yaw0))
            yaw = yaw0 + t*dyaw
        else:
            x,y,yaw = self.path[-1]

        # 发布 GPS（llh）
        lat, lon = enu_to_ll(x, y, self.lat0, self.lon0)
        fix = NavSatFix()
        fix.header.frame_id = 'base_link'; fix.header.stamp = self.get_clock().now().to_msg()
        fix.status.status = NavSatStatus.STATUS_FIX; fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = float(lat); fix.longitude = float(lon); fix.altitude = 0.0
        # 简单设置协方差
        fix.position_covariance = [1.0,0.0,0.0, 0.0,1.0,0.0, 0.0,0.0,4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_fix.publish(fix)

        # 发布 IMU（仅姿态）
        imu = Imu()
        imu.header.frame_id = 'base_link'; imu.header.stamp = fix.header.stamp
        imu.orientation = quat_from_yaw(yaw)
        imu.orientation_covariance = [0.02,0.0,0.0, 0.0,0.02,0.0, 0.0,0.0,0.02]
        self.pub_imu.publish(imu)

def main():
    rclpy.init()
    rclpy.spin(GpsImuSim())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
