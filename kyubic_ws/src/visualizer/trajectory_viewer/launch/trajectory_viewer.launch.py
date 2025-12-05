from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    trajectory_viewer_launcher_dir = PathJoinSubstitution(
        [FindPackageShare("trajectory_viewer"), "launch"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution([trajectory_viewer_launcher_dir, "odom_viewer.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                    # "odom_topic_name": "/localization/odom",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [trajectory_viewer_launcher_dir, "global_pose_viewer.launch.py"]
                ),
                launch_arguments={
                    "log_level": "warn",
                    "global_pose_topic_name": "/localization/global_pose",
                }.items(),
            ),
        ]
    )
