from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cloudgripper_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhamamd Zahid',
    maintainer_email='mzmi@kth.se',
    description='The cloudgripper_ros package facilitates seamless integration and control of CloudGripper within the ROS ecosystem',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_service = cloudgripper_ros.state_service:main',
            'robot_subscriber = cloudgripper_ros.robot_subscriber:main',
            'image_service = cloudgripper_ros.image_service:main',
            'image_re_publish = cloudgripper_ros.image_re_publish:main',
            'keyboard_teleop_control = cloudgripper_ros.keyboard_teleop_control:main',
        ],
    },
)
