from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 公共话题
    publish_topic = LaunchConfiguration('publish_topic')
    period_ms     = LaunchConfiguration('period_ms')

    # 发送节点参数
    can_interface = LaunchConfiguration('can_interface')
    can_id        = LaunchConfiguration('can_id_status')
    can_extended  = LaunchConfiguration('can_extended')

    return LaunchDescription([
        # 话题/周期
        DeclareLaunchArgument('publish_topic', default_value='/vcu_remote_bytes',
                              description='Teleop payload topic (UInt8MultiArray, len=8)'),
        DeclareLaunchArgument('period_ms',     default_value='200',
                              description='Teleop publish period in ms'),

        # CAN 发送节点参数（不做任何 ip link 配置）
        DeclareLaunchArgument('can_interface',  default_value='can0',
                              description='SocketCAN interface name'),
        DeclareLaunchArgument('can_id_status',  default_value='0x18FED188',
                              description='Status arbitration ID (hex or int)'),
        DeclareLaunchArgument('can_extended',   default_value='true',
                              description='Use extended (29-bit) ID'),

        # 键盘遥控发布节点
        Node(
            package='gps_pid_follower',
            executable='vcu_can_teleop',
            name='vcu_can_teleop',
            output='screen',
            parameters=[{
                'publish_topic': publish_topic,
                'period_ms': period_ms,
                # 预速度字段编码：km/h = raw*0.5 + (-50)
                'speed_offset_kmh': -50.0,
                'speed_res_kmh_per_lsb': 0.5,
            }]
        ),

        # python-can 发送节点
        Node(
            package='gps_pid_follower',
            executable='vcu_can_sender',
            name='vcu_can_sender',
            output='screen',
            parameters=[{
                'topic': publish_topic,
                'can_interface': can_interface,
                'can_id_status': can_id,
                'can_extended': can_extended,
            }]
        ),
    ])
