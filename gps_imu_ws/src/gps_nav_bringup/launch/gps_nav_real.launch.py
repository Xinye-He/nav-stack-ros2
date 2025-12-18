# gps_nav_bringup/launch/gps_nav_real.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('gps_nav_bringup')
    rl_yaml   = os.path.join(pkg_share, 'config', 'robot_localization.yaml')
    csv_path  = os.environ.get('GPS_PATH_CSV', os.path.join(pkg_share, 'data', 'route.csv'))
    urdf_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf')

    gps_topic = LaunchConfiguration('gps_topic')
    imu_topic = LaunchConfiguration('imu_topic')

    robot_desc_param = []
    if os.path.exists(urdf_path):
        robot_desc_param = [{'robot_description': open(urdf_path, 'r').read()}]

    return LaunchDescription([
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix',
                              description='Real GPS NavSatFix topic'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data',
                              description='Real IMU Imu topic'),

        Node(package='robot_localization', executable='ekf_node', name='ekf_odom',
             output='screen', parameters=[rl_yaml],
             remappings=[('/odometry/filtered','/odometry/filtered_odom')]),

        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform', output='screen', parameters=[rl_yaml],
             remappings=[
                 ('/gps/fix', gps_topic),
                 ('/imu', imu_topic),
                 ('/odometry/filtered','/odometry/filtered_odom'),
             ]),

        Node(package='robot_localization', executable='ekf_node', name='ekf_map',
             output='screen', parameters=[rl_yaml],
             remappings=[('/odometry/filtered','/odometry/filtered_map')]),

        Node(package='gps_path_server', executable='path_server', name='path_server',
             output='screen', parameters=[{'csv_file': csv_path, 'sample_step': 0.5}]),

        Node(package='gps_local_follower', executable='local_follower', name='local_follower',
             output='screen', parameters=[{'base_speed': 1.2, 'max_speed': 2.0}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=robot_desc_param),
    ])
