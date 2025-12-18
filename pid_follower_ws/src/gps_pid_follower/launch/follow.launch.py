from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    path_csv = LaunchConfiguration('path_csv')

    return LaunchDescription([
        # 基础参数
        DeclareLaunchArgument('path_csv', default_value='/root/pid_follower_ws/points.csv'),
        DeclareLaunchArgument('gps_topic', default_value='/gps/fix'),            # 建议与GPS驱动保持一致
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel'),

        # 控制关键参数
        DeclareLaunchArgument('target_speed', default_value='2.0'),
        DeclareLaunchArgument('lookahead_dist', default_value='1.5'),
        DeclareLaunchArgument('yaw_offset_deg', default_value='0.0'),

        # 可调控制/行为
        DeclareLaunchArgument('max_yaw_rate', default_value='2.0'),
        DeclareLaunchArgument('advance_when_close', default_value='2.5'),
        DeclareLaunchArgument('t_advance_min', default_value='0.9'),
        DeclareLaunchArgument('turn_in_place_deg', default_value='55.0'),
        DeclareLaunchArgument('min_speed', default_value='0.5'),
        DeclareLaunchArgument('cte_slow_k', default_value='0.1'),
        DeclareLaunchArgument('yaw_lpf_alpha', default_value='0.4'),
        DeclareLaunchArgument('k_heading', default_value='1.3'),
        DeclareLaunchArgument('kp_cte', default_value='0.8'),
        DeclareLaunchArgument('ki_cte', default_value='0.0'),
        DeclareLaunchArgument('kd_cte', default_value='0.3'),
        DeclareLaunchArgument('i_limit', default_value='3.0'),
        DeclareLaunchArgument('use_csv_heading', default_value='true'),
        DeclareLaunchArgument('align_heading_only_at_task_points', default_value='true'),
        DeclareLaunchArgument('heading_align_dist', default_value='1.0'),
        DeclareLaunchArgument('wp_reached_dist', default_value='1.0'),
        DeclareLaunchArgument('wp_heading_tol_deg', default_value='10.0'),
        DeclareLaunchArgument('stop_turn_tol_deg', default_value='5.0'),
        DeclareLaunchArgument('wait_for_task_done', default_value='true'),
        DeclareLaunchArgument('task_done_topic', default_value='/task_done'),
        DeclareLaunchArgument('auto_align_yaw', default_value='false'),

        # 可视化
        DeclareLaunchArgument('with_rviz', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('traj_path_len', default_value='2000'),

        # CAN 参数（仅状态帧）
        DeclareLaunchArgument('enable_can', default_value='true'),
        DeclareLaunchArgument('can_interface', default_value='can0'),
        DeclareLaunchArgument('can_extended', default_value='true'),
        DeclareLaunchArgument('can_id_status', default_value='0x18FED188'),
        DeclareLaunchArgument('link_remote_to_abort', default_value='true'),

        # 选择航向来源（RTK vs IMU）
        DeclareLaunchArgument('use_rtk_heading', default_value='true'),
        DeclareLaunchArgument('rtk_heading_topic', default_value='/gps/heading_deg'),

        # VCU（MOVE/SPIN）增强控制参数
        DeclareLaunchArgument('vcu_enhanced_mode', default_value='true'),
        DeclareLaunchArgument('vcu_angle_move_limit_deg', default_value='18.0'),
        DeclareLaunchArgument('vcu_angle_spin_enter_deg', default_value='22.0'),
        DeclareLaunchArgument('vcu_angle_spin_exit_deg',  default_value='18.0'),
        DeclareLaunchArgument('vcu_angle_spin_cmd_deg',   default_value='30.0'),
        DeclareLaunchArgument('small_keep_fast_deg', default_value='3.0'),
        DeclareLaunchArgument('turn_slow_deg',       default_value='10.0'),
        DeclareLaunchArgument('corner_spin_deg',     default_value='45.0'),
        DeclareLaunchArgument('preturn_trigger_dist', default_value='4.0'),
        DeclareLaunchArgument('angle_lpf_alpha_cmd', default_value='0.5'),
        DeclareLaunchArgument('enable_virtual_distance', default_value='false'),
        DeclareLaunchArgument('virt_spin_dist', default_value='8.0'),

        # 键盘节点开关（未使用，仅保留接口）
        DeclareLaunchArgument('with_keyboard', default_value='false'),

        # IMU驱动（仅在未使用RTK航向时启动）
        Node(
            package='imu_driver',
            executable='imu_driver',
            name='imu',
            remappings=[('/imu/data_raw', '/imu/data')],
            parameters=[{'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'}, {"baud": 9600}],
            output='screen',
            #condition=UnlessCondition(LaunchConfiguration('use_rtk_heading'))
        ),

        # GPS驱动（示例：NMEA 桥接，话题通常是 /fix）
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
        ),

        # 循迹节点
        Node(
            package='gps_pid_follower',
            executable='waypoint_pid_follower',
            name='waypoint_pid_follower',
            output='screen',
            parameters=[{
                'path_csv': path_csv,
                'gps_topic': LaunchConfiguration('gps_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),

                'target_speed': LaunchConfiguration('target_speed'),
                'max_yaw_rate': LaunchConfiguration('max_yaw_rate'),
                'advance_when_close': LaunchConfiguration('advance_when_close'),
                't_advance_min': LaunchConfiguration('t_advance_min'),
                'lookahead_dist': LaunchConfiguration('lookahead_dist'),

                'use_csv_heading': LaunchConfiguration('use_csv_heading'),
                'align_heading_only_at_task_points': LaunchConfiguration('align_heading_only_at_task_points'),
                'heading_align_dist': LaunchConfiguration('heading_align_dist'),

                'wp_reached_dist': LaunchConfiguration('wp_reached_dist'),
                'wp_heading_tol_deg': LaunchConfiguration('wp_heading_tol_deg'),
                'stop_turn_tol_deg': LaunchConfiguration('stop_turn_tol_deg'),
                'wait_for_task_done': LaunchConfiguration('wait_for_task_done'),
                'task_done_topic': LaunchConfiguration('task_done_topic'),

                'k_heading': LaunchConfiguration('k_heading'),
                'kp_cte': LaunchConfiguration('kp_cte'),
                'ki_cte': LaunchConfiguration('ki_cte'),
                'kd_cte': LaunchConfiguration('kd_cte'),
                'i_limit': LaunchConfiguration('i_limit'),

                'turn_in_place_deg': LaunchConfiguration('turn_in_place_deg'),
                'min_speed': LaunchConfiguration('min_speed'),
                'cte_slow_k': LaunchConfiguration('cte_slow_k'),
                'yaw_lpf_alpha': LaunchConfiguration('yaw_lpf_alpha'),

                'yaw_offset_deg': LaunchConfiguration('yaw_offset_deg'),
                'auto_align_yaw': LaunchConfiguration('auto_align_yaw'),

                # 可视化/TF
                'map_frame': LaunchConfiguration('map_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'traj_path_len': LaunchConfiguration('traj_path_len'),

                # CAN
                'enable_can': LaunchConfiguration('enable_can'),
                'can_interface': LaunchConfiguration('can_interface'),
                'can_extended': LaunchConfiguration('can_extended'),
                'can_id_status': LaunchConfiguration('can_id_status'),
                'link_remote_to_abort': LaunchConfiguration('link_remote_to_abort'),

                # 航向来源（RTK）
                'use_rtk_heading': LaunchConfiguration('use_rtk_heading'),
                'rtk_heading_topic': LaunchConfiguration('rtk_heading_topic'),

                # VCU MOVE/SPIN 增强控制
                'vcu_enhanced_mode': LaunchConfiguration('vcu_enhanced_mode'),
                'vcu_angle_move_limit_deg': LaunchConfiguration('vcu_angle_move_limit_deg'),
                'vcu_angle_spin_enter_deg': LaunchConfiguration('vcu_angle_spin_enter_deg'),
                'vcu_angle_spin_exit_deg':  LaunchConfiguration('vcu_angle_spin_exit_deg'),
                'vcu_angle_spin_cmd_deg':   LaunchConfiguration('vcu_angle_spin_cmd_deg'),
                'small_keep_fast_deg': LaunchConfiguration('small_keep_fast_deg'),
                'turn_slow_deg':       LaunchConfiguration('turn_slow_deg'),
                'corner_spin_deg':     LaunchConfiguration('corner_spin_deg'),
                'preturn_trigger_dist': LaunchConfiguration('preturn_trigger_dist'),
                'angle_lpf_alpha_cmd': LaunchConfiguration('angle_lpf_alpha_cmd'),
                'enable_virtual_distance': LaunchConfiguration('enable_virtual_distance'),
                'virt_spin_dist': LaunchConfiguration('virt_spin_dist'),
            }]
        ),

        # 如需 RViz，可在 with_rviz==true 时添加 RViz 节点（此处省略）
    ])
