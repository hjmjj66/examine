from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = (
        get_package_share_directory("aim_handeye_calibrator")
        + "/config/aim_camera_intrinsic_calibrator.yaml"
    )

    return LaunchDescription([
        Node(
            package="aim_handeye_calibrator",
            executable="aim_camera_intrinsic_calibrator_node",
            name="aim_camera_intrinsic_calibrator_node",
            output="screen",
            parameters=[config_file],
        )
    ])
