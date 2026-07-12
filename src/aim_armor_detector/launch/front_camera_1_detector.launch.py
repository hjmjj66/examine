from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    detector_config_path = (
        get_package_share_directory("aim_armor_detector")
        + "/config/front_1_detector.yaml"
    )

    gx_camera_config_path = (
        get_package_share_directory("aim_camera_driver") + "/config/gx_camera_1.yaml"
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name="front_camera_1_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="aim_camera_driver",
                    plugin="aim_camera_driver::GxCameraComponent",
                    name="gx_camera_node",
                    parameters=[gx_camera_config_path],
                    extra_arguments=[{"use_intra_process_comms": False}],
                ),
                ComposableNode(
                    package="aim_armor_detector",
                    plugin="aim_armor_detector::AimArmorDetectorNode",
                    name="aim_armor_detector_node",
                    parameters=[detector_config_path],
                    extra_arguments=[{"use_intra_process_comms": False}],
                ),
            ],
        ),
    ])
