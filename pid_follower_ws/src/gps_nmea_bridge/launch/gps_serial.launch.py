from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_nmea_bridge',
            executable='gps_nmea_bridge_node',
            name='gps_nmea_bridge',
            parameters=[{
                'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                'baudrate': 115200,
                'frame_id': 'gps',
                'check_crc': True,
                'min_satellites': 0
            }],
            output='screen'
        )
    ])
