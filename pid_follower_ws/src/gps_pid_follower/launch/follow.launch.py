from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('gps_pid_follower')
    # 支持环境变量 GPS_PID_PARAMS_FILE 覆盖；最后还可以通过命令行 params_file:=xxx 覆盖
    default_yaml = os.path.join(pkg_share, 'config', 'params.yaml')
    default_yaml_env = EnvironmentVariable(name='GPS_PID_PARAMS_FILE', default_value=default_yaml)

    return LaunchDescription([
        # 参数文件（优先级：命令行 > 环境变量 > 默认路径）
        DeclareLaunchArgument('params_file', default_value=default_yaml_env,
                              description='YAML parameters file for waypoint_pid_follower'),

        # IMU（发布 odom->base_link）
        Node(
            package='imu_driver',
            executable='imu_driver',
            name='imu',
            remappings=[('/imu/data_raw', '/imu/data')],
            parameters=[{'port': '/dev/imu_usb'}, {"baud": 9600}],
            output='screen'
        ),

        # GPS/NMEA 桥（示例）
        Node(
            package='nmea_bridge',
            executable='nmea_bridge_node',
            name='nmea_bridge',
            parameters=[{
                'port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                'baudrate': 115200,
                'frame_id': 'gps',
                'check_crc': True,
                'min_satellites': 0
            }],
            output='screen'
        ),

        Node(
            package='gps_pid_follower',
            executable='waypoint_pid_follower',
            name='waypoint_pid_follower',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file')
            ]
        ),

        # 如需同时启动动作节点，取消下方注释：
        # Node(
        #     package='gps_pid_follower',
        #     executable='bale_pick_action',
        #     name='bale_pick_action',
        #     output='screen',
        #     parameters=[LaunchConfiguration('params_file')]
        # ),
    ])
