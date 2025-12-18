# gps_nav_bringup/launch/gps_nav_sim.launch.py
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
        # 开关
        DeclareLaunchArgument('use_rviz', default_value='true', description='Start RViz2'),

        # RViz 在容器中的稳定环境变量（软件渲染 + 关闭 Qt SHM）
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('MESA_GL_VERSION_OVERRIDE', '3.3'),
        SetEnvironmentVariable('MESA_GLSL_VERSION_OVERRIDE', '330'),
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),

        # robot_localization
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

        # 路径/跟随/模型/闭环模拟器
        Node(package='gps_path_server', executable='path_server', name='path_server',
             output='screen', parameters=[{'csv_file': csv_path, 'sample_step': 0.5}]),

        Node(package='gps_local_follower', executable='local_follower', name='local_follower',
             output='screen',
             parameters=[{'base_speed': 0.8, 'max_speed': 2.0,
                          'yaw_stop_deg': 50.0, 'yaw_slow_deg': 15.0,
                          'min_speed_mps': 0.2, 'shrink_lookahead_on_turn': True,
                          'corner_stop_angle_deg': 35.0, 'corner_check_ahead_m': 2.0,
                          'stop_after_one_lap': True, 'corner_stop_enable': True, 'corner_hard_stop': True,
                          'corner_min_ahead_m': 2.0, 'corner_release_deg': 5.0,
                          'corner_slow_start_m': 8.0, 'corner_slow_end_m': 2.0,
                          'corner_slow_min_speed': 0.3, 'a_lat_max': 1.0, 
                          'freeze_move_thresh_m': 0.10,
                          'step_mode': False, 'debug_print': True}]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=robot_desc_param),

        Node(package='gps_nav_bringup', executable='drive_cl_sim', name='drive_cl_sim',
             output='screen',
             parameters=[{'yaw_gain': 3.0, 'max_ang_vel': 1.8,
                          'speed_cap_mps': 3.0,
                          'sim_rate_hz': 50.0, 'gps_rate_hz': 10.0, 'imu_rate_hz': 50.0,
                          'step_mode': False,
                          'use_manual_datum': False,
                          'datum_lat': 0.0, 'datum_lon': 0.0,
                          'init_x': 0.0, 'init_y': 0.0, 'init_yaw_deg': 0.0}]),

        # RViz2（带 URDF 参数与默认配置）
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen',
             arguments=['-d', rviz_cfg],
             parameters=robot_desc_param,
             condition=IfCondition(use_rviz)),
    ])
