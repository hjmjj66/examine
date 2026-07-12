import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    del args, kwargs
    params_file = LaunchConfiguration("params_file").perform(context)

    with open(params_file, "r", encoding="utf-8") as file:
        cfg = yaml.safe_load(file)

    tf_params = cfg.get("sentry_tf_node", {}).get("ros__parameters", {})
    cam = cfg.get("barrel_to_camera_0", {})
    cam1 = cfg.get("barrel_to_camera_1", {})
    big_cam = cfg.get("small_yaw_to_usb_camera", {})

    def static_transform_node(name, transform):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=name,
            output="screen",
            arguments=[
                "--x", str(transform["x"]),
                "--y", str(transform["y"]),
                "--z", str(transform["z"]),
                "--yaw", str(transform["yaw"]),
                "--pitch", str(transform["pitch"]),
                "--roll", str(transform["roll"]),
                "--frame-id", transform["parent_frame"],
                "--child-frame-id", transform["child_frame"],
            ],
        )

    def optical_transform_node(parent_frame, child_frame):
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"{parent_frame}_to_optical_tf",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--qx", "0.5", "--qy", "-0.5", "--qz", "0.5", "--qw", "-0.5",
                "--frame-id", parent_frame,
                "--child-frame-id", child_frame,
            ],
        )

    nodes = [
        Node(
            package="sentry_tf",
            executable="tf_node",
            name="sentry_tf_node",
            output="screen",
            parameters=[tf_params],
        ),
        static_transform_node("barrel_to_camera_0_tf", cam),
        optical_transform_node(cam["child_frame"], f"{cam['child_frame']}_optical_frame"),
    ]

    if big_cam:
        nodes.append(static_transform_node("small_yaw_to_usb_camera_tf", big_cam))
        nodes.append(
            optical_transform_node(
                big_cam["child_frame"], f"{big_cam['child_frame']}_optical_frame")
        )

    if cam1:
        nodes.append(static_transform_node("barrel_to_camera_1_tf", cam1))
        nodes.append(
            optical_transform_node(cam1["child_frame"], f"{cam1['child_frame']}_optical_frame")
        )

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("sentry_tf")
    default_params = os.path.join(pkg_share, "config", "sentry_tf.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the ROS2 parameters file.",
        ),
        OpaqueFunction(function=launch_setup),
    ])
