from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数文件（默认加载包内标准参数）
    params_file = LaunchConfiguration('params_file')

    # 是否在回放时使用仿真时钟（传给 RViz；bale_estimator 建议在 YAML 里设置 use_sim_time，避免重复）
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 是否发布静态TF（bag 不含 /tf_static 时设 true）
    use_tf       = LaunchConfiguration('use_tf')
    parent_frame = LaunchConfiguration('parent_frame')  # 基座，一般 base_link
    child_frame  = LaunchConfiguration('child_frame')   # 点云 frame_id，需与 bag/驱动一致，如 rslidar

    # 静态TF的平移与四元数（向下俯仰45°：qy=+0.38268343, qw=0.92387953）
    lidar_x = LaunchConfiguration('lidar_x')
    lidar_y = LaunchConfiguration('lidar_y')
    lidar_z = LaunchConfiguration('lidar_z')
    qx      = LaunchConfiguration('qx')
    qy      = LaunchConfiguration('qy')
    qz      = LaunchConfiguration('qz')
    qw      = LaunchConfiguration('qw')

    # 是否启动 RViz
    start_rviz = LaunchConfiguration('start_rviz')

    declares = [
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robosense_driver'),
                'config',
                'bale_60x100.yaml'  # 也可改成 bale_loose.yaml
            ]),
            description='bale_estimator 参数文件'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='回放bag设为true'),
        DeclareLaunchArgument('use_tf',       default_value='true', description='是否发布静态TF base_link->rslidar'),
        DeclareLaunchArgument('parent_frame', default_value='base_link', description='静态TF父坐标'),
        DeclareLaunchArgument('child_frame',  default_value='rslidar',    description='静态TF子坐标(=点云frame_id)'),
        DeclareLaunchArgument('lidar_x',      default_value='0.0'),
        DeclareLaunchArgument('lidar_y',      default_value='0.0'),
        DeclareLaunchArgument('lidar_z',      default_value='1.80'),
        DeclareLaunchArgument('qx',           default_value='0.0'),
        DeclareLaunchArgument('qy',           default_value='0.38268343'),  # +45 deg about Y
        DeclareLaunchArgument('qz',           default_value='0.0'),
        DeclareLaunchArgument('qw',           default_value='0.92387953'),
        DeclareLaunchArgument('start_rviz',   default_value='false'),
    ]

    # 估计器：只加载 params_file，避免在此处重复设置 use_sim_time 导致已声明冲突
    bale_node = Node(
        package='robosense_driver',
        executable='bale_estimator',
        name='bale_pose_estimator',
        output='screen',
        parameters=['/root/lidar_ws/config/bale_60x100.yaml']
    )

    # 静态TF（仅当 use_tf 为 true 时）
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_rslidar',
        output='screen',
        arguments=[
            lidar_x, lidar_y, lidar_z,   # translation
            qx, qy, qz, qw,              # quaterniono

            parent_frame, child_frame    # parent child
        ],
        condition=IfCondition(use_tf)
    )

    # RViz（使用仿真时钟与否由 use_sim_time 控制）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz)
    )

    return LaunchDescription(declares + [static_tf, bale_node, rviz_node])
