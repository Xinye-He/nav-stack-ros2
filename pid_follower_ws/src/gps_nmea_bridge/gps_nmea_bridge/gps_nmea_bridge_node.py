#!/usr/bin/env python3
# coding: utf-8
import threading
import serial
import time
from math import radians
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float32, String


def verify_nmea_checksum(nmea_line: str) -> bool:
    """Verify NMEA checksum like: $GPGGA,...*HH"""
    if not nmea_line.startswith('$') or '*' not in nmea_line:
        return False
    body, cks = nmea_line[1:].split('*', 1)
    try:
        expected = int(cks[:2], 16)
    except ValueError:
        return False
    c = 0
    for ch in body:
        c ^= ord(ch)
    return c == expected


def nmea_to_degrees(raw: str, hemi: str) -> Optional[float]:
    """Convert ddmm.mmmm (lat) or dddmm.mmmm (lon) to decimal degrees."""
    if not raw:
        return None
    try:
        val = float(raw)
    except ValueError:
        return None
    deg = int(val // 100)
    minutes = val - deg * 100.0
    dec = deg + minutes / 60.0
    if hemi in ('S', 'W'):
        dec = -dec
    return round(dec, 8)


class NmeaGpsNode(Node):
    def __init__(self):
        super().__init__('gps_nmea_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('min_satellites', 0)  # 0 = no limit
        self.declare_parameter('check_crc', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.min_satellites = self.get_parameter('min_satellites').get_parameter_value().integer_value
        self.check_crc = self.get_parameter('check_crc').get_parameter_value().bool_value

        # Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)
        self.speed_pub = self.create_publisher(Float32, '/gps/ground_speed_mps', qos_profile_sensor_data)
        self.track_pub = self.create_publisher(Float32, '/gps/track_true_deg', qos_profile_sensor_data)
        self.nmea_pub = self.create_publisher(String, '/gps/nmea_sentence', 10)
        # 新增 heading 角度发布器
        self.heading_pub = self.create_publisher(Float32, '/gps/heading_deg', 10)

        # State from last sentences
        self.state = {
            'lat': None, 'lon': None, 'alt': None,
            'num_sats': None, 'hdop': None, 'fix_quality': 0,
            'track_true': None, 'speed_knots': None
        }

        # Open serial
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=1)
            self.get_logger().info(f'Opened serial: {self.ser.port} @ {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial: {e}')
            raise

        # Start reader thread
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self.read_loop, daemon=True)
        self._thread.start()

        # Periodic republish
        self.create_timer(0.2, self.publish_fix_if_ready)

    def destroy_node(self):
        self._stop.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        return super().destroy_node()

    def read_loop(self):
        while not self._stop.is_set():
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not raw:
                    continue
                # Broadcast raw NMEA
                msg_raw = String()
                msg_raw.data = raw
                self.nmea_pub.publish(msg_raw)

                if not raw.startswith('$'):
                    continue
                if self.check_crc and not verify_nmea_checksum(raw):
                    continue

                body = raw.split('*', 1)[0]
                parts = body.split(',')
                typ = parts[0][3:]  # e.g., 'GGA', 'THS'

                if typ == 'GGA':
                    self.handle_gga(parts)
                elif typ == 'VTG':
                    self.handle_vtg(parts)
                elif typ == 'RMC':
                    self.handle_rmc(parts)
                elif typ == 'THS':
                    self.handle_ths(parts)
            except Exception as e:
                self.get_logger().debug(f'read_loop error: {e}')
                time.sleep(0.05)

    def handle_gga(self, p):
        try:
            lat = nmea_to_degrees(p[2], p[3]) if len(p) > 4 else None
            lon = nmea_to_degrees(p[4], p[5]) if len(p) > 6 else None
            alt = float(p[9]) if len(p) > 10 and p[9] != '' else None
            fixq = int(p[6]) if len(p) > 7 and p[6].isdigit() else 0
            nums = int(p[7]) if len(p) > 8 and p[7].isdigit() else None
            hdop = float(p[8]) if len(p) > 9 and p[8] != '' else None
        except Exception:
            return

        self.state.update({
            'lat': lat, 'lon': lon, 'alt': alt,
            'fix_quality': fixq, 'num_sats': nums, 'hdop': hdop
        })
        self.publish_fix_if_ready()

    def handle_vtg(self, p):
        def f(i):
            return float(p[i]) if len(p) > i and p[i] not in ('', None) else None
        t_true = f(1) if len(p) > 2 and p[2] == 'T' else f(1)
        speed_knots = f(5)
        self.state.update({'track_true': t_true, 'speed_knots': speed_knots})
        self.publish_speed_track()

    def handle_rmc(self, p):
        status = p[2] if len(p) > 2 else 'V'
        if status != 'A':
            return
        lat = nmea_to_degrees(p[3], p[4]) if len(p) > 5 else None
        lon = nmea_to_degrees(p[5], p[6]) if len(p) > 7 else None
        speed_knots = float(p[7]) if len(p) > 8 and p[7] != '' else None
        track_true = float(p[8]) if len(p) > 9 and p[8] != '' else None

        if lat is not None and lon is not None:
            self.state.update({'lat': lat, 'lon': lon})
        self.state.update({'speed_knots': speed_knots, 'track_true': track_true})
        self.publish_speed_track()

    def handle_ths(self, p):
        # $GNTHS,heading,mode*xx
        if len(p) < 2:
            return
        try:
            heading_deg = float(p[1])
            # 可选：验证范围 [0, 360]
            if not (0 <= heading_deg <= 360):
                self.get_logger().warn(f"Invalid heading value: {heading_deg}")
                return
        except Exception as e:
            self.get_logger().warn(f"Failed to parse THS: {e}")
            return

        # 直接发布角度（单位：度）
        msg = Float32()
        msg.data = float(heading_deg)
        self.heading_pub.publish(msg)

        # 可选：打印日志（调试用）
        # self.get_logger().info(f"Heading: {heading_deg:.2f}°")

    def publish_speed_track(self):
        if self.state.get('speed_knots') is not None:
            mps = self.state['speed_knots'] * 0.514444
            smsg = Float32()
            smsg.data = float(mps)
            self.speed_pub.publish(smsg)
        if self.state.get('track_true') is not None:
            tmsg = Float32()
            tmsg.data = float(self.state['track_true'])
            self.track_pub.publish(tmsg)

    def publish_fix_if_ready(self):
        lat = self.state.get('lat')
        lon = self.state.get('lon')
        fixq = self.state.get('fix_quality', 0)
        nums = self.state.get('num_sats')

        if lat is None or lon is None:
            return
        if self.min_satellites and (nums is None or nums < self.min_satellites):
            return

        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        msg.altitude = float(self.state['alt']) if self.state.get('alt') is not None else float('nan')

        status = NavSatStatus.STATUS_NO_FIX if fixq == 0 else NavSatStatus.STATUS_FIX
        if fixq == 2:
            status = NavSatStatus.STATUS_SBAS_FIX
        msg.status.status = status
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.fix_pub.publish(msg)


def main():
    rclpy.init()
    node = NmeaGpsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
