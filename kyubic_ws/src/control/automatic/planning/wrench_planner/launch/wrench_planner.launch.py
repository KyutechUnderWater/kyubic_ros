import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg_wrench_planner = os.path.join(
        get_package_share_directory("wrench_planner"),
        "config",
        "wrench_planner.param.yaml",
    )

    p_pid_controller_path = get_package_share_directory("p_pid_controller")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    return LaunchDescription(
        [
            log_level_arg,
            Node(
                package="wrench_planner",
                namespace="planner/wrench_planner",
                executable="wrench_planner_component_node",
                remappings=[
                    ("odom", "/localization/odom"),
                    ("robot_force", "/driver/actuator_rp2040_driver/robot_force"),
                    ("targets", "/rt_pose_plotter/targets"),
                ],
                parameters=[
                    {"p_pid_controller_path": p_pid_controller_path},
                    cfg_wrench_planner,
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
