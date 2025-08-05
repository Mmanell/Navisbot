import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Keyboard teleop node (publishes Twist)
    keyboard_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="keyboard_teleop",
        prefix="xterm -e",
        output="screen",
        remappings=[('/cmd_vel', '/keyboard_raw_cmd')],  # Intermediate topic
        parameters=[{
            'scale_linear': 1.0,    
            'scale_angular': 8.0,   }]
    )

    # Converter node (Twist -> TwistStamped)
    twist_to_twist_stamped = Node(
        package="navisbot_controller",
        executable="twist_to_twist_stamped",
        name="twist_converter",
        remappings=[
            ('/input/cmd_vel', '/keyboard_raw_cmd'),
            ('/output/cmd_vel', '/navisbot_controller/cmd_vel'),  # Matches joystick's final topic
        ]
    )

    return LaunchDescription([
        keyboard_teleop,
        twist_to_twist_stamped,
    ])