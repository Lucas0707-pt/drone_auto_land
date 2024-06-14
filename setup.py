from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_auto_land'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lucas',
    maintainer_email='luislucas0707@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'processes = drone_auto_land.processes:main',
            'marker_detector = drone_auto_land.marker_detector:main',
            'frame_converter = drone_auto_land.frame_converter:main',
            'control_node = drone_auto_land.control_node:main',
            'camera_bridge = drone_auto_land.camera_bridge:main',
            'data_logger = drone_auto_land.data_logger:main',
        ],
    },
)
