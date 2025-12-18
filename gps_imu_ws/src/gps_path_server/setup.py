from setuptools import setup
import os
package_name = 'gps_path_server'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), ['scripts/path_server','scripts/geo_to_map_path']),
    ],
    install_requires=['setuptools', 'geographiclib'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='CSV to ENU path server (line/arc sampler)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_server = gps_path_server.path_server:main',
            'geo_to_map_path = gps_path_server.geo_to_map_path:main',
        ],
    },
)
