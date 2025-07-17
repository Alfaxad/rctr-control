from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rctr_robot",
            executable="robot_hardware_interface",
            name="hardware",
            output="screen",
            parameters=[{"serial_port": "/dev/ttyACM0"}],
        ),
        Node(
            package="rctr_robot",
            executable="robot_camera_streamer",
            name="camera",
            output="screen",
            parameters=[{"width": 640, "height": 480, "fps": 30}],
        ),
        Node(
            package="rctr_robot",
            executable="robot_perception",
            name="perception",
            output="screen",
            parameters=[{"process_line_detection": True, "process_qr_codes": True}],
        ),
    ])
