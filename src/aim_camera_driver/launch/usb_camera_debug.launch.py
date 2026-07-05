# 独立调试 USB 相机：仅启动相机组件，不加载 detector
# 用法: ros2 launch aim_camera_driver usb_camera_debug.launch.py
# 然后 ros2 topic hz /usb_camera/image_raw 验证出图

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    usb_camera_config_path = (
        get_package_share_directory("aim_camera_driver") + "/config/usb_camera.yaml"
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name="usb_camera_debug_container",
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
                ),
            ],
        ),
    ])
