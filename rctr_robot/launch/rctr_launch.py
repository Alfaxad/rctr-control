"""
Launch file for RCTR robot
Starts all necessary nodes with proper parameters
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('rctr_robot')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # Hardware interface node
        Node(
            package='rctr_robot',
            executable='hardware_interface',
            name='hardware_interface',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 115200
            }]
        ),
        
        # Perception node
        Node(
            package='rctr_robot',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'use_compressed': True,
                'camera_topic': '/camera/image_raw',
                'yolo_model_path': os.path.join(pkg_dir, 'models', 'yolov8n_rctr.pt'),
                'confidence_threshold': 0.5,
                'visualize': True
            }]
        ),
        
        # Control node
        Node(
            package='rctr_robot',
            executable='control_node',
            name='control_node',
            output='screen',
            parameters=[{
                'control_rate': 50.0,
                'base_speed': 0.3,
                'max_speed': 0.5,
                'max_angular': 2.0,
                'line_kp': 2.0,
                'line_ki': 0.1,
                'line_kd': 0.5,
                'angle_kp': 1.0,
                'angle_ki': 0.05,
                'angle_kd': 0.2
            }]
        ),
        
        # Challenge manager
        Node(
            package='rctr_robot',
            executable='challenge_manager',
            name='challenge_manager',
            output='screen'
        ),
        
        # Camera node (using v4l2_camera package)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV',
                'camera_frame_id': 'camera_link',
                'io_method': 'mmap',
                'camera_info_url': '',
                'brightness': 128,
                'contrast': 128,
                'saturation': 128,
                'sharpness': 128,
                'autogain': True
            }]
        ),
    ])
