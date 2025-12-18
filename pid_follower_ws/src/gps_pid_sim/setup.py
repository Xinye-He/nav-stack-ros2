from setuptools import find_packages, setup

package_name = 'gps_pid_sim'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Simple closed-loop simulator: /cmd_vel -> fake GPS(/fix) + IMU(/imu/data_raw)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_diffdrive_gps_imu = gps_pid_sim.sim_diffdrive_gps_imu:main',
        ],
    },
)
