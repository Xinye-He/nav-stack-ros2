import os
from setuptools import find_packages, setup

package_name = 'gps_nmea_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml']),
       ('share/' + package_name + '/launch', ['launch/gps_serial.launch.py']),
       (os.path.join('lib', package_name), ['scripts/gps_nmea_bridge_node']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='NMEA GPS to ROS2 bridge (NavSatFix + speed/track)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_nmea_bridge_node = gps_nmea_bridge.gps_nmea_bridge_node:main',
        ],
    },
)
