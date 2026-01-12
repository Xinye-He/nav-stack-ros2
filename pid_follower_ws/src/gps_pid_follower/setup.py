from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'gps_pid_follower'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
]

setup(
    name=package_name,
    version='0.3.1',
    packages=find_packages(),          # 扁平布局：直接查找当前包
    package_dir={'': '.'},             # 根就是包根
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Waypoint PID follower with VCU CAN control (pre-speed discrete & 3-level angle), RTK heading, action override.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_pid_follower = gps_pid_follower.waypoint_pid_follower:main',
            'drive_key = gps_pid_follower.drive_key:main',
            # 如未使用动作节点，注释下一行
            'bale_pick_action = gps_pid_follower.bale_pick_action:main',
            'vcu_can_teleop = gps_pid_follower.vcu_can_teleop:main',
            'vcu_can_sender = gps_pid_follower.can_sender:main',
            'rtk_center_from_nmea = gps_pid_follower.rtk_center_from_nmea:main',
            'dr_odometry_node = gps_pid_follower.dr_odometry_node:main',
        ],
    },
)
