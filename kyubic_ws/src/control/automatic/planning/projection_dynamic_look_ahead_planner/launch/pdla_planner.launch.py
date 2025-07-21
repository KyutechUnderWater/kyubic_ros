import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg_pdla_planner = os.path.join(
        get_package_share_directory("projection_dynamic_look_ahead_planner"),
        "config",
        "pdla_planner.param.yaml",
    )

    path_planner_path = os.path.join(
        get_package_share_directory("path_planner"),
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    return LaunchDescription(
        [
            log_level_arg,
            Node(
                package="projection_dynamic_look_ahead_planner",
                namespace="planner",
                executable="pdla_planner_component_node",
                remappings=[
                    ("/planner/odom", "/localization/odom"),
                ],
                parameters=[
                    {"path_planner_path": path_planner_path},
                    cfg_pdla_planner,
                ],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
