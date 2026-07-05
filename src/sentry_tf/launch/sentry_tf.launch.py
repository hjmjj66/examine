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

    nodes = [
        Node(
            package="sentry_tf",
            executable="tf_node",
            name="sentry_tf_node",
            output="screen",
            parameters=[tf_params],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="barrel_to_camera_0_tf",
            output="screen",
            arguments=[
                str(cam["x"]),
                str(cam["y"]),
                str(cam["z"]),
                str(cam["yaw"]),
                str(cam["pitch"]),
                str(cam["roll"]),
                cam["parent_frame"],
                cam["child_frame"],
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="baselink_to_gimbal_small_yaw_tf",
            output="screen",
            arguments=[
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "0.0",
                "base_link",
                tf_params["yaw_frame"],
            ],
        ),
    ]

    if big_cam:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="small_yaw_to_usb_camera_tf",
                output="screen",
                arguments=[
                    str(big_cam["x"]),
                    str(big_cam["y"]),
                    str(big_cam["z"]),
                    str(big_cam["yaw"]),
                    str(big_cam["pitch"]),
                    str(big_cam["roll"]),
                    big_cam["parent_frame"],
                    big_cam["child_frame"],
                ],
            )
        )

    if cam1:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="barrel_to_camera_1_tf",
                output="screen",
                arguments=[
                    str(cam1["x"]),
                    str(cam1["y"]),
                    str(cam1["z"]),
                    str(cam1["yaw"]),
                    str(cam1["pitch"]),
                    str(cam1["roll"]),
                    cam1["parent_frame"],
                    cam1["child_frame"],
                ],
            )
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
