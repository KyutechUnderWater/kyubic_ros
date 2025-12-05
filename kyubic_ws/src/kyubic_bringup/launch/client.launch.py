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

    joy_common_dir = PathJoinSubstitution([FindPackageShare("joy_common"), "launch"])
    kyubic_bringup_dir = PathJoinSubstitution([FindPackageShare("kyubic_bringup_dir"), "launch"])

    return LaunchDescription(
        [
            log_level_arg,
            IncludeLaunchDescription(
                PathJoinSubstitution[joy_common_dir, "joy_common.launch.py"],
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([kyubic_bringup_dir, "client_visualizer.launch.py"]),
                launch_arguments={
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
        ]
    )
