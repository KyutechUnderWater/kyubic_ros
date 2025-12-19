from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy2wrench_dir = PathJoinSubstitution([FindPackageShare("joy2wrench"), "launch"])

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution([joy2wrench_dir, "joy2wrench.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
        ]
    )
