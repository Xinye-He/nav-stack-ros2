from setuptools import setup
import os
package_name = 'gps_local_follower'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), ['scripts/local_follower','scripts/pid_follower']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Local follower (Pure Pursuit) output AngleSpeed',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_follower = gps_local_follower.local_follower:main',
            'pid_follower = gps_local_follower.pid_follower:main',
        ],
    },
)
