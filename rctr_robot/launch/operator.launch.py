from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rctr_robot",
            executable="operator_control",
            name="control",
            output="screen",
            parameters=[{"use_joystick": False}],
        ),
        Node(
            package="rctr_robot",
            executable="operator_display",
            name="display",
            output="screen",
            parameters=[{
                "display_telemetry": True,
                "record_video": False,
                "show_fps": True,
            }],
        ),
    ])
