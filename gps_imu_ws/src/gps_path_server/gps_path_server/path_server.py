#!/usr/bin/env python3
import rclpy, math, csv, re
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger
from geographiclib.geodesic import Geodesic
from gps_msgs.msg import Waypoint, GlobalPath

def bearing_deg_to_yaw_rad(bearing_deg):
    return math.radians(90.0 - bearing_deg)

def yaw_rad_to_bearing_deg(yaw_rad):
    deg = 90.0 - math.degrees(yaw_rad)
    return (deg + 180.0) % 360.0 - 180.0

def quat_from_yaw(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw/2.0), w=math.cos(yaw/2.0))

def ll_to_enu(lat, lon, lat0, lon0):
    g = Geodesic.WGS84.Inverse(lat0, lon0, lat, lon)
    x = g['s12'] * math.sin(math.radians(g['azi1']))  # East
    y = g['s12'] * math.cos(math.radians(g['azi1']))  # North
    return x, y

def line_samples(p0, p1, step=0.5):
    x0,y0,_ = p0; x1,y1,_ = p1
    dx, dy = x1-x0, y1-y0
    dist = max(1e-6, math.hypot(dx, dy))
    n = max(2, int(dist/step)+1)
    out=[]
    for i in range(n):
        t = i/(n-1)
        x = x0 + t*dx
        y = y0 + t*dy
        yaw = math.atan2(dy, dx)
        out.append((x,y,yaw))
    return out

def intersect_lines(p, n, q, m):
    denom = n[0]*m[1] - n[1]*m[0]
    if abs(denom) < 1e-6: return None
    t = ((q[0]-p[0])*m[1] - (q[1]-p[1])*m[0]) / denom
    return (p[0] + t*n[0], p[1] + t*n[1])

def arc_samples(p0, p1, step=0.5):
    x0,y0,yaw0 = p0; x1,y1,yaw1 = p1
    n0 = (-math.sin(yaw0), math.cos(yaw0))
    n1 = (-math.sin(yaw1), math.cos(yaw1))
    c = intersect_lines((x0,y0), n0, (x1,y1), n1)
    if c is None:
        return line_samples(p0, p1, step)
    cx, cy = c
    v0x, v0y = x0-cx, y0-cy
    v1x, v1y = x1-cx, y1-cy
    r0 = math.hypot(v0x, v0y); r1 = math.hypot(v1x, v1y)
    if r0 < 1e-3 or r1 < 1e-3:
        return line_samples(p0, p1, step)
    r = 0.5*(r0+r1)
    th0 = math.atan2(v0y, v0x)
    th1 = math.atan2(v1y, v1x)

    def wrap(a):
        while a <= -math.pi: a += 2*math.pi
        while a >  math.pi: a -= 2*math.pi
        return a

    tan_ccw = th0 + math.pi/2.0
    tan_cw  = th0 - math.pi/2.0
    err_ccw = abs(wrap(tan_ccw - yaw0))
    err_cw  = abs(wrap(tan_cw  - yaw0))
    ccw = err_ccw <= err_cw

    dth = wrap(th1 - th0)
    if ccw and dth < 0: dth += 2*math.pi
    if (not ccw) and dth > 0: dth -= 2*math.pi

    arc_len = abs(dth) * r
    n = max(2, int(arc_len/step)+1)
    out=[]
    for i in range(n):
        t = i/(n-1)
        th = th0 + t*dth
        x = cx + r*math.cos(th)
        y = cy + r*math.sin(th)
        yaw = th + (math.pi/2.0 if ccw else -math.pi/2.0)
        out.append((x,y,yaw))
    return out

class PathServer(Node):
    def __init__(self):
        super().__init__('gps_path_server')
        self.declare_parameter('csv_file', '')
        self.declare_parameter('sample_step', 0.5)
        self.csv_file = self.get_parameter('csv_file').value
        self.step = float(self.get_parameter('sample_step').value)

        self.pub_path = self.create_publisher(Path, '/global_path', 1)
        self.pub_geo  = self.create_publisher(GlobalPath, '/global_path_geo', 1)
        self.reload_srv = self.create_service(Trigger, '/path_server/reload', self.on_reload)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.lat0 = None; self.lon0 = None
        self.sampled = []; self.geo_points = []

        if self.csv_file:
            self.load_csv(self.csv_file)

    def validate_7dec(self, s):
        return re.fullmatch(r'-?\d+\.\d{7}', s.strip()) is not None

    def load_csv(self, path):
        pts=[]
        with open(path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row in reader:
                lat_s = row['lat'].strip()
                lon_s = row['lon'].strip()
                if not self.validate_7dec(lat_s) or not self.validate_7dec(lon_s):
                    self.get_logger().warn(f"lat/lon not 7-decimals: {lat_s}, {lon_s} (will round)")
                lat = round(float(lat_s), 7)
                lon = round(float(lon_s), 7)
                wp = Waypoint()
                wp.index = int(row['index'])
                wp.latitude = lat; wp.longitude = lon
                wp.heading_deg = float(row['heading_deg'])
                wp.point_type = int(row['point_type'])
                wp.next_seg_type = int(row['next_seg_type'])
                pts.append(wp)
        pts = sorted(pts, key=lambda w: w.index)
        if len(pts) < 2:
            raise RuntimeError("Need at least 2 waypoints")
        self.lat0 = pts[0].latitude; self.lon0 = pts[0].longitude

        samples=[]
        N = len(pts)
        for i in range(N):
            a = pts[i]; b = pts[(i+1)%N]
            ax, ay = ll_to_enu(a.latitude, a.longitude, self.lat0, self.lon0)
            bx, by = ll_to_enu(b.latitude, b.longitude, self.lat0, self.lon0)
            yaw_a = bearing_deg_to_yaw_rad(a.heading_deg)
            yaw_b = bearing_deg_to_yaw_rad(b.heading_deg)
            seg_a = (ax, ay, yaw_a); seg_b = (bx, by, yaw_b)
            if a.next_seg_type == 1:
                seg_samples = arc_samples(seg_a, seg_b, self.step)
            else:
                seg_samples = line_samples(seg_a, seg_b, self.step)
            if samples and seg_samples:
                seg_samples = seg_samples[1:]
            samples.extend(seg_samples)
        self.sampled = samples
        self.geo_points = pts
        self.get_logger().info(f"Loaded {len(pts)} geo waypoints, sampled {len(samples)} poses")

    def on_reload(self, req, resp):
        try:
            self.load_csv(self.csv_file)
            resp.success = True; resp.message = "reloaded"
        except Exception as e:
            resp.success = False; resp.message = str(e)
        return resp

    def on_timer(self):
        if not self.sampled: return
        header_stamp = self.get_clock().now().to_msg()
        path = Path(); path.header.stamp = header_stamp; path.header.frame_id = 'map'
        poses=[]
        for x,y,yaw in self.sampled:
            p = PoseStamped(); p.header = path.header
            p.pose.position.x = float(x); p.pose.position.y = float(y)
            p.pose.orientation = quat_from_yaw(yaw)
            poses.append(p)
        path.poses = poses
        self.pub_path.publish(path)

        g = GlobalPath(); g.header = path.header; g.geo_frame = "WGS84"; g.points = self.geo_points
        self.pub_geo.publish(g)

def main():
    rclpy.init()
    node = PathServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
