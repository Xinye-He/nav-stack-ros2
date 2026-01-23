# /jetson-inference/ucar/src/robosense_driver/launch/robosense.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robosense_driver',
            executable='robosense_node',
            name='robosense_driver',
            output='screen',
            parameters=[{
                "input_type": "1",  # ONLINE_LIDAR
                "msop_port": 6699,
                "difop_port": 7788,
                "lidar_type": "RSE1",  # 根据实际型号修改
            }]
        )
    ])
