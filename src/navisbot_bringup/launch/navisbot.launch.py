import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():


    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("navisbot_description"),
            "launch",
            "navisBot_gazebo.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("navisbot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )
    
    teleop = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("navisbot_controller"),
            "launch",
            "keyboard_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "True",
        }.items()
    )
    
    return LaunchDescription([
        gazebo,
        controller,
        teleop,
    ])