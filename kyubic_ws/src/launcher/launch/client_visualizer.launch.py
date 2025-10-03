from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rt_pose_plotter_dir = PathJoinSubstitution(
        [FindPackageShare("rt_pose_plotter"), "launch"]
    )
    trajectory_viewer_dir = PathJoinSubstitution(
        [FindPackageShare("trajectory_viewer"), "launch"]
    )
    web_controller_dir = PathJoinSubstitution(
        [FindPackageShare("web_controller"), "launch"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [rt_pose_plotter_dir, "rt_pose_plotter.launch.py"]
                ),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution(
                    [trajectory_viewer_dir, "trajectory_viewer.launch.py"]
                ),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
            IncludeLaunchDescription(
                PathJoinSubstitution([web_controller_dir, "web_controller.launch.py"]),
                launch_arguments={
                    "log_level": "warn",
                }.items(),
            ),
        ]
    )
