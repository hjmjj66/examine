from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # =========================================================================
    # 探测器配置文件路径（后相机专用配置）
    # =========================================================================
    detector_config_path = (
        get_package_share_directory("aim_armor_detector")
        + "/config/back_camera_detector.yaml"
    )

    # =========================================================================
    # USB 相机驱动配置文件路径
    # =========================================================================
    usb_camera_config_path = (
        get_package_share_directory("aim_camera_driver") + "/config/usb_camera.yaml"
    )

    return LaunchDescription([
        # =====================================================================
        # 后相机检测组件容器
        #    容器内加载 UsbCameraComponent + AimArmorDetectorNode，二者在同一进程，
        #    通过 use_intra_process_comms 实现进程内零拷贝（指针传递）。
        #    Camera → Detector 的 Image 消息不再经过 DDS 序列化/反序列化。
        # =====================================================================
        ComposableNodeContainer(
            name="back_camera_detector_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="aim_camera_driver",
                    plugin="aim_camera_driver::UsbCameraComponent",
                    name="usb_camera_node",
                    parameters=[usb_camera_config_path],
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
