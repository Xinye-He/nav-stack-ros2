from setuptools import setup
from glob import glob
import os

package_name = 'gps_nav_bringup'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (os.path.join(share_dir, 'launch'), glob('launch/*.py')),
        (os.path.join(share_dir, 'config'), glob('config/*.yaml')),
        (os.path.join(share_dir, 'data'), glob('data/*.csv')),
        (os.path.join(share_dir, 'urdf'), glob('urdf/*')),
        # 如有 scripts 也要装到 lib/<pkg>，可保留这一行；没有就可以去掉
        (os.path.join('lib', package_name), glob('scripts/*')),
        (os.path.join(share_dir, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'geographiclib'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Bringup for GPS-only navigation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_imu_sim = gps_nav_bringup.gps_imu_sim:main',    # 开环模拟
            'drive_cl_sim = gps_nav_bringup.drive_cl_sim:main',  # 闭环模拟
            'pid_follower = gps_local_follower.pid_follower:main',
        ],
    },
)
