from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_config_path = (
        get_package_share_directory("aim_armor_detector") + "/config/aim_camera_detector.yaml"
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config_path),
        Node(
            package="aim_armor_detector",
            executable="aim_armor_detector_node",
            name="aim_armor_detector_node",
            output="screen",
            parameters=[config_file],
        ),
    ])
