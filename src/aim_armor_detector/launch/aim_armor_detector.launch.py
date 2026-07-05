from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = (
        get_package_share_directory("aim_armor_detector")
        + "/config/aim_camera_detector.yaml"
    )

    return LaunchDescription([
        Node(
            package="aim_armor_detector",
            executable="aim_armor_detector_node",
            name="aim_armor_detector_node",
            output="screen",
            parameters=[config_file],
        )
    ])
