from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["warn"],
        description="Logging level",
    )

    web_controller_dir = PathJoinSubstitution([FindPackageShare("web_controller"), "launch"])
    dashboard_dir = PathJoinSubstitution([FindPackageShare("dashboard"), "launch"])

    return LaunchDescription(
        [
            log_level_arg,
            IncludeLaunchDescription(
                PathJoinSubstitution([web_controller_dir, "web_controller.launch.py"]),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([dashboard_dir, "dashboard.launch.py"]),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
        ]
    )
