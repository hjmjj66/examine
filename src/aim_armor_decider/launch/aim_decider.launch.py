import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("aim_armor_decider")
    default_params = os.path.join(pkg_share, "config", "aim_armor_decider.yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the ROS2 parameters file.",
        ),
        Node(
            package="aim_armor_decider",
            executable="aim_armor_decider_node",
            name="aim_armor_decider_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
