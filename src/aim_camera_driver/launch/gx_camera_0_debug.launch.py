# 独立调试大恒 GX 相机 0：仅启动 gx_camera_0，不加载 detector
# 用法: ros2 launch aim_camera_driver gx_camera_0_debug.launch.py
# 然后 ros2 topic hz /gx_camera_0/image_raw 验证出图

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gx_camera_0_config_path = (
        get_package_share_directory("aim_camera_driver") + "/config/gx_camera_0.yaml"
    )

    return LaunchDescription([
        ComposableNodeContainer(
            name="gx_camera_0_debug_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="aim_camera_driver",
                    plugin="aim_camera_driver::GxCameraComponent",
                    name="gx_camera_node",
                    parameters=[gx_camera_0_config_path],
                    extra_arguments=[{"use_intra_process_comms": False}],
                ),
            ],
        ),
    ])
