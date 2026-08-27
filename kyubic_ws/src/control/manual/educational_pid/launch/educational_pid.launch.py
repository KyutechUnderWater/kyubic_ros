from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pc_launch = PathJoinSubstitution(
        [FindPackageShare("educational_pid"), "launch", "educational_pid_pc.launch.py"]
    )
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(pc_launch))])
