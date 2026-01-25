from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("qr_planner"), "config", "qr_planner.param.yaml"]
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )

    return LaunchDescription(
        [
            log_level_arg,
            Node(
                package="qr_planner",
                namespace="planner/qr_planner",
                executable="qr_planner_node",
                name="qr_planner",
                remappings=[
                    ("odom", "/localization/odom"),
                    ("zed_power", "/driver/sensors_esp32_driver/zed_power"),
                    ("goal_current_odom", "/planner/wrench_planner/goal_current_odom"),
                ],
                parameters=[config],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )
