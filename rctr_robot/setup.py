from setuptools import setup
import os
from glob import glob

package_name = 'rctr_robot'

setup(
    name=package_name,
    version='0.2.0',
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
    install_requires=[
        'setuptools',
        'opencv-python',        # or opencv-python-headless
        'ultralytics',          # YOLOv8 (pulls tqdm, pyyaml, etc.)
        # torch & torchvision wheels are installed separately on the Pi
    ],
    zip_safe=True,
    maintainer='Team 11',
    maintainer_email='team11@example.com',
    description='RCTR Robot Teleoperation & Perception Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Robot‑side
            'robot_hardware_interface = rctr_robot.robot_hardware_interface:main',
            'robot_camera_streamer     = rctr_robot.robot_camera_streamer:main',
            'robot_perception          = rctr_robot.robot_perception:main',
            'robot_executor            = rctr_robot.robot_executor:main',

            # Operator‑side
            'operator_control = rctr_robot.operator_control:main',
            'operator_display = rctr_robot.operator_display:main',
        ],
    },
)
