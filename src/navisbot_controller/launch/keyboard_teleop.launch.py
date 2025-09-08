import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare namespace argument


    # Keyboard teleop node (publishes Twist)
    keyboard_teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="keyboard_teleop",
        prefix="xterm -e",
        output="screen",
        remappings=[('/cmd_vel', '/key_vel')],
    )

    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("twist_mux"),
                "launch",
                "twist_mux_launch.py"
            )
        ),
        launch_arguments={
            "cmd_vel_out": "navisbot_controller/cmd_vel_unstamped",
            "config_topics": os.path.join(get_package_share_directory("navisbot_controller"), "config", "twist_mux.yaml"),
            "config_locks": "",
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    # Converter node (Twist -> TwistStamped)
    twist_to_twist_stamped = Node(
        package="navisbot_controller",
        executable="twist_to_twist_stamped",
        name="twist_converter",
    )

    return LaunchDescription([
        keyboard_teleop,
        twist_mux_launch,
        twist_to_twist_stamped,
    ])
