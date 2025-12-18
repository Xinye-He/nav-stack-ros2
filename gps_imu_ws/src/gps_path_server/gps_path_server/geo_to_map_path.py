#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from gps_msgs.msg import GlobalPath
from robot_localization.srv import FromLL
from geographiclib.geodesic import Geodesic

def enu_to_ll(x_e, y_n, lat0, lon0):
    s = math.hypot(x_e, y_n)
    azi = math.degrees(math.atan2(x_e, y_n))  # North=0, East=+90
    g = Geodesic.WGS84.Direct(lat0, lon0, azi, s)
    return g['lat2'], g['lon2']

class GeoToMapPath(Node):
    def __init__(self):
        super().__init__('geo_to_map_path')
        self.lat0 = None
        self.lon0 = None

        self.raw_path = None          # 最新收到的 /global_path
        self.raw_list = []            # [(x,y,quat), ...]
        self.conv_list = []           # 转换后的 [(mx,my,quat), ...]
        self.idx = 0                  # 转换进度索引
        self.cached_path = None       # 完整 Path(map)
        self.converting = False

        q_latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.fromll = self.create_client(FromLL, '/fromLL')
        self.sub_geo  = self.create_subscription(GlobalPath, '/global_path_geo', self.on_geo, 1)
        self.sub_path = self.create_subscription(Path, '/global_path', self.on_path, 1)
        self.pub_path = self.create_publisher(Path, '/global_path_map', q_latched)

        # 2Hz 定时器：尝试分批转换 + 周期重发
        self.timer = self.create_timer(0.5, self.on_timer)
        self.get_logger().info('geo_to_map_path ready: waiting /global_path_geo + /global_path + /fromLL')

    def on_geo(self, msg: GlobalPath):
        if msg.points:
            self.lat0 = float(msg.points[0].latitude)
            self.lon0 = float(msg.points[0].longitude)
            self.get_logger().info(f"geo origin lat0={self.lat0:.7f}, lon0={self.lon0:.7f}")

    def on_path(self, msg: Path):
        # 收到原始 ENU 路径，缓存下来并重置转换进度
        if not msg.poses:
            return
        self.raw_path = msg
        self.raw_list = [(p.pose.position.x, p.pose.position.y, p.pose.orientation) for p in msg.poses]
        self.conv_list = []
        self.idx = 0
        self.cached_path = None
        self.converting = False
        self.get_logger().info(f"received /global_path: N={len(self.raw_list)} poses (will convert to map)")

    def call_fromll(self, lat, lon):
        # 同步调用 /fromLL，失败返回 None
        if not self.fromll.service_is_ready():
            self.fromll.wait_for_service(timeout_sec=1.0)
            if not self.fromll.service_is_ready():
                return None
        req = FromLL.Request()
        req.ll_point.latitude = float(lat)
        req.ll_point.longitude = float(lon)
        req.ll_point.altitude = 0.0
        try:
            resp = self.fromll.call(req)
            return resp.map_point.x, resp.map_point.y
        except Exception as e:
            self.get_logger().warn(f'fromLL call exception: {e}')
            return None

    def convert_chunk(self, batch=120):
        # 分批把 raw_list 从 idx 开始转换 batch 个点
        if self.lat0 is None or self.lon0 is None or not self.raw_list:
            return False
        if not self.fromll.service_is_ready():
            self.get_logger().warn('/fromLL not ready yet')
            return False

        n = len(self.raw_list)
        end = min(self.idx + batch, n)
        for i in range(self.idx, end):
            x, y, q = self.raw_list[i]
            lat, lon = enu_to_ll(x, y, self.lat0, self.lon0)
            xy = self.call_fromll(lat, lon)
            if xy is None:
                # 本批失败，稍后重试
                return False
            mx, my = xy
            self.conv_list.append((mx, my, q))
        self.idx = end
        self.get_logger().info(f'convert progress: {self.idx}/{n}')
        return self.idx >= n

    def publish_cached(self):
        if not self.conv_list:
            return
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        poses = []
        for mx, my, q in self.conv_list:
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = float(mx)
            ps.pose.position.y = float(my)
            ps.pose.position.z = 0.0
            ps.pose.orientation = q
            poses.append(ps)
        out.poses = poses
        self.cached_path = out
        self.pub_path.publish(out)
        self.get_logger().info(f'published /global_path_map with {len(poses)} poses')

    def on_timer(self):
        # 尚未有原始路径或地理原点
        if self.raw_list and (self.lat0 is not None) and (self.cached_path is None):
            # 尚未完成则继续分批转换
            done = self.convert_chunk(batch=120)
            if done:
                self.publish_cached()
        # 周期重发（latched也会缓存，但这里再发一遍更保险）
        if self.cached_path is not None:
            self.cached_path.header.stamp = self.get_clock().now().to_msg()
            for ps in self.cached_path.poses:
                ps.header = self.cached_path.header
            self.pub_path.publish(self.cached_path)

def main():
    rclpy.init()
    node = GeoToMapPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
