from setuptools import setup
import os
from glob import glob

package_name = 'rctr_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
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
    maintainer='Team 11',
    maintainer_email='team11@example.com',
    description='RCTR Robot Control Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = rctr_robot.perception_node:main',
            'control_node = rctr_robot.control_node:main',
            'hardware_interface = rctr_robot.hardware_interface:main',
            'challenge_manager = rctr_robot.challenge_manager:main',
        ],
    },
)
