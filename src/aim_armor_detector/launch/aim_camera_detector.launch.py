from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_config_path = (
        get_package_share_directory("aim_armor_detector") + "/config/aim_camera_detector.yaml"
    )
    config_file = LaunchConfiguration("config_file")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=default_config_path),
        ComposableNodeContainer(
            name="aim_camera_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="aim_armor_detector",
                    plugin="aim_armor_detector::AimArmorDetectorNode",
                    name="aim_armor_detector_node",
                    parameters=[config_file],
                ),
            ],
        ),
    ])
