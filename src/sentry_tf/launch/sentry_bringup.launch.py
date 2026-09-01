import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def include_launch(package_name: str, launch_file: str, launch_arguments=None):
    launch_arguments = launch_arguments or {}
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), "launch", launch_file)
        ),
        launch_arguments=launch_arguments.items(),
    )


def generate_launch_description():
    sentry_tf_params = LaunchConfiguration("sentry_tf_params")
    solver_params = LaunchConfiguration("solver_params")
    tracker_params = LaunchConfiguration("tracker_params")
    decider_params = LaunchConfiguration("decider_params")
    controller_params = LaunchConfiguration("controller_params")

    sentry_tf_default = os.path.join(
        get_package_share_directory("sentry_tf"),
        "config",
        "sentry_tf.yaml",
    )
    solver_default = os.path.join(
        get_package_share_directory("aim_solver"),
        "config",
        "aim_solver.yaml",
    )
    tracker_default = os.path.join(
        get_package_share_directory("tracker"),
        "config",
        "tracker.yaml",
    )
    decider_default = os.path.join(
        get_package_share_directory("aim_armor_decider"),
        "config",
        "aim_armor_decider.yaml",
    )
    controller_default = os.path.join(
        get_package_share_directory("aim_armor_controller"),
        "config",
        "armor_controller.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "sentry_tf_params",
            default_value=sentry_tf_default,
            description="Path to sentry_tf parameters.",
        ),
        DeclareLaunchArgument(
            "solver_params",
            default_value=solver_default,
            description="Path to aim_solver parameters.",
        ),
        DeclareLaunchArgument(
            "tracker_params",
            default_value=tracker_default,
            description="Path to tracker parameters.",
        ),
        DeclareLaunchArgument(
            "decider_params",
            default_value=decider_default,
            description="Path to aim_armor_decider parameters.",
        ),
        DeclareLaunchArgument(
            "controller_params",
            default_value=controller_default,
            description="Path to aim_armor_controller parameters.",
        ),
        include_launch(
            "sentry_tf",
            "sentry_tf.launch.py",
            {"params_file": sentry_tf_params},
        ),
        include_launch("aim_armor_detector", "front_camera_0_detector.launch.py"),
        include_launch("aim_armor_detector", "front_camera_1_detector.launch.py"),
        include_launch("aim_armor_detector", "back_camera_detector.launch.py"),
        include_launch(
            "aim_solver",
            "aim_solver.launch.py",
            {"config_file": solver_params},
        ),
        include_launch(
            "tracker",
            "tracker.launch.py",
            {"params_file": tracker_params},
        ),
        include_launch(
            "aim_outpost_predictor",
            "aim_outpost_predictor.launch.py",
        ),
        include_launch(
            "aim_armor_decider",
            "aim_decider.launch.py",
            {"params_file": decider_params},
        ),
        include_launch(
            "aim_armor_controller",
            "armor_controller.launch.py",
            {"params_file": controller_params},
        ),
    ])
