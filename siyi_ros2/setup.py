from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'siyi_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohamed Abdelkader',
    maintainer_email='mohamedashraf123@gmail.com',
    description='ROS2 Jazzy interface for SIYI gimbal-camera systems',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'siyi_node = siyi_ros2.siyi_node:main',
            'siyi_camera_node = siyi_ros2.camera_node:main',
        ],
    },
)
