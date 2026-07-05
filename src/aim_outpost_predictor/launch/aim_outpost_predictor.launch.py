import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("aim_outpost_predictor"),
        "config",
        "aim_outpost_predictor.yaml",
    )
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the ROS2 parameters file.",
        ),
        Node(
            package="aim_outpost_predictor",
            executable="aim_outpost_predictor_node",
            name="aim_outpost_predictor_node",
            output="screen",
            parameters=[params_file],
        )
    ])
