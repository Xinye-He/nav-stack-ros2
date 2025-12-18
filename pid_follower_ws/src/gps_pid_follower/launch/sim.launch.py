from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # 共享关键参数
    path_csv       = LaunchConfiguration('path_csv')
    target_speed   = LaunchConfiguration('target_speed')
    lookahead_dist = LaunchConfiguration('lookahead_dist')

    # 仿真参数
    sim_hz   = LaunchConfiguration('sim_hz')
    gps_hz   = LaunchConfiguration('gps_hz')
    imu_hz   = LaunchConfiguration('imu_hz')
    init_e   = LaunchConfiguration('init_e')
    init_n   = LaunchConfiguration('init_n')
    init_yaw = LaunchConfiguration('init_heading_deg_csv')
    gps_std  = LaunchConfiguration('gps_std_m')
    yaw_std  = LaunchConfiguration('yaw_std_deg')

    # CAN 参数（仅状态帧）
    enable_can           = LaunchConfiguration('enable_can')
    can_interface        = LaunchConfiguration('can_interface')
    can_extended         = LaunchConfiguration('can_extended')
    can_id_status        = LaunchConfiguration('can_id_status')        # 支持十进制或"0x..."字符串
    link_remote_to_abort = LaunchConfiguration('link_remote_to_abort')  # /abort联动请求遥控

    # 键盘开关
    with_keyboard        = LaunchConfiguration('with_keyboard')

    return LaunchDescription([
        # CSV/控制
        DeclareLaunchArgument('path_csv', default_value='/jetson-inference/pid_follower_ws/points.csv'),
        DeclareLaunchArgument('target_speed', default_value='2.0'),
        DeclareLaunchArgument('lookahead_dist', default_value='0.5'),

        # 仿真器参数
        DeclareLaunchArgument('sim_hz', default_value='50.0'),
        DeclareLaunchArgument('gps_hz', default_value='10.0'),
        DeclareLaunchArgument('imu_hz', default_value='50.0'),
        DeclareLaunchArgument('init_e', default_value='-8.0'),
        DeclareLaunchArgument('init_n', default_value='-5.0'),
        DeclareLaunchArgument('init_heading_deg_csv', default_value='45.0'),
        DeclareLaunchArgument('gps_std_m', default_value='0.0'),
        DeclareLaunchArgument('yaw_std_deg', default_value='0.0'),

        # CAN（默认开启，虚拟总线 vcan0；真车用 can0）
        DeclareLaunchArgument('enable_can', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_extended', default_value='true'),
        DeclareLaunchArgument('can_id_status', default_value='0x18FED188'),
        DeclareLaunchArgument('link_remote_to_abort', default_value='true'),

        # 键盘：默认开
        DeclareLaunchArgument('with_keyboard', default_value='true'),

        # 仿真器节点
        Node(
            package='gps_pid_sim',
            executable='sim_diffdrive_gps_imu',
            name='sim_diffdrive_gps_imu',
            output='screen',
            parameters=[{
                'path_csv': path_csv,
                'gps_topic': '/fix',
                'imu_topic': '/imu/data_raw',
                'cmd_topic': '/cmd_vel',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'sim_hz': sim_hz,
                'gps_hz': gps_hz,
                'imu_hz': imu_hz,
                'init_e': init_e,
                'init_n': init_n,
                'init_heading_deg_csv': init_yaw,
                'gps_std_m': gps_std,
                'yaw_std_deg': yaw_std,
            }]
        ),

        # 循迹+CAN 节点
        Node(
            package='gps_pid_follower',
            executable='waypoint_pid_follower',
            name='waypoint_pid_follower',
            output='screen',
            parameters=[{
                'path_csv': path_csv,
                'gps_topic': '/fix',
                'imu_topic': '/imu/data_raw',
                'cmd_topic': '/cmd_vel',

                'target_speed': target_speed,
                'max_yaw_rate': 2.0,
                'advance_when_close': 2.5,
                't_advance_min': 0.9,
                'lookahead_dist': lookahead_dist,

                'use_csv_heading': True,
                'align_heading_only_at_task_points': True,
                'heading_align_dist': 1.0,

                'wp_reached_dist': 0.8,
                'wp_heading_tol_deg': 10.0,
                'stop_turn_tol_deg': 5.0,
                'wait_for_task_done': False,

                'k_heading': 1.3,
                'kp_cte': 0.8,
                'ki_cte': 0.0,
                'kd_cte': 0.3,
                'i_limit': 3.0,

                'turn_in_place_deg': 55.0,
                'min_speed': 0.5,
                'cte_slow_k': 0.1,
                'yaw_lpf_alpha': 0.4,

                'enable_can': enable_can,
                'can_interface': can_interface,
                'can_extended': can_extended,
                'can_id_status': can_id_status,
                'link_remote_to_abort': link_remote_to_abort,
            }]
        ),

        # 键盘节点（只有 with_keyboard:=true 才启动）
        Node(
            package='gps_pid_follower',
            executable='drive_key',
            name='drive_key',
            output='screen',
            condition=IfCondition(with_keyboard)
        ),
    ])
