# gps_nav_bringup/launch/gps_nav_pid_sim.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('gps_nav_bringup')
    rl_yaml   = os.path.join(pkg_share, 'config', 'robot_localization.yaml')
    csv_path  = os.environ.get('GPS_PATH_CSV', os.path.join(pkg_share, 'data', 'route.csv'))
    urdf_path = os.path.join(pkg_share, 'urdf', 'diffbot.urdf')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'gps_nav.rviz')

    use_rviz  = LaunchConfiguration('use_rviz')
    robot_desc_param = [{'robot_description': open(urdf_path, 'r').read()}] if os.path.exists(urdf_path) else []

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz2'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '330'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),

        Node(package='robot_localization', executable='ekf_node', name='ekf_odom',
             output='screen', parameters=[rl_yaml],
             remappings=[('/odometry/filtered','/odometry/filtered_odom')]),

        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform', output='screen', parameters=[rl_yaml],
             remappings=[('/gps/fix','/gps/fix'),
                         ('/imu','/imu/data'),
                         ('/odometry/filtered','/odometry/filtered_odom')]),

        Node(package='robot_localization', executable='ekf_node', name='ekf_map',
             output='screen', parameters=[rl_yaml],
             remappings=[('/odometry/filtered','/odometry/filtered_map')]),

        # 只发 CSV 航点（/global_path_geo），不需要生成密集 Path
        Node(package='gps_path_server', executable='path_server', name='path_server',
             output='screen', parameters=[{'csv_file': csv_path, 'sample_step': 0.5}]),

        # 直接用 CSV 航点的 PID（内部调用 /fromLL）
        Node(package='gps_local_follower', executable='pid_follower', name='pid_follower',
             output='screen',
             parameters=[{
                 'base_speed': 0.6, 'min_speed': 0.2, 'max_speed': 1.5,
                 'yaw_slow_deg': 20.0, 'yaw_stop_deg': 75.0,
                 'kp': 0.40, 'ki': 0.00, 'kd': 0.20,
                 'i_max': 0.5, 'yaw_corr_deg_max': 35.0,
                 'lookahead_m': 1.5,
                 'stop_after_one_lap': False,
                 'freeze_move_thresh_m': 0.10,
                 'step_mode': False, 'debug_print': True
             }]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=robot_desc_param),

        Node(package='gps_nav_bringup', executable='drive_cl_sim', name='drive_cl_sim',
             output='screen',
             parameters=[{'yaw_gain': 3.5, 'max_ang_vel': 2.2,
                          'speed_cap_mps': 3.0,
                          'sim_rate_hz': 50.0, 'gps_rate_hz': 10.0, 'imu_rate_hz': 50.0,
                          'step_mode': False,
                          'use_manual_datum': False,
                          'datum_lat': 0.0, 'datum_lon': 0.0,
                          'init_x': 0.0, 'init_y': 0.0, 'init_yaw_deg': 0.0}]),
    ])
