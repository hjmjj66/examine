import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("aim_armor_controller")
    default_params = os.path.join(pkg_share, "config", "armor_controller.yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the ROS2 parameters file.",
        ),
        Node(
            package="aim_armor_controller",
            executable="armor_controller_node",
            name="armor_controller_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
