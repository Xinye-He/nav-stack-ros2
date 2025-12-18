from setuptools import setup

package_name = 'gps_pid_follower'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/follow.launch.py',
            'launch/sim.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='you@example.com',
    description='GPS+IMU PID waypoint follower for differential tracks (/cmd_vel)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_pid_follower = gps_pid_follower.waypoint_pid_follower:main',
            'drive_key = gps_pid_follower.drive_key:main',
        ],
    },
)
