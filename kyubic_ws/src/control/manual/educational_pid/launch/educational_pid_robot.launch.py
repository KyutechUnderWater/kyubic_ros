import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("educational_pid"), "config", "pid_robot.yaml"
    )

    joy_launch = PathJoinSubstitution(
        [FindPackageShare("joy_common"), "launch", "joy_common.launch.py"]
    )

    return LaunchDescription(
        [
            # joy2wrenchは起動しない。このlaunchがJoy入力を直接使用する。
            IncludeLaunchDescription(PythonLaunchDescriptionSource(joy_launch)),
            Node(
                package="educational_pid",
                executable="pid_node",
                name="educational_pid_node",
                output="screen",
                parameters=[config],
                remappings=[
                    ("depth_odom", "/localization/depth/odom"),
                    ("imu_odom", "/localization/imu/transformed"),
                    ("joy_common", "/joy_common/joy_common"),
                    (
                        "robot_force",
                        "/driver/actuator_rp2040_driver/robot_force",
                    ),
                    ("targets", "/rt_pose_plotter/targets"),
                    (
                        "integral_depth",
                        "/educational_pid/debug/integral_depth",
                    ),
                    ("integral_yaw", "/educational_pid/debug/integral_yaw"),
                    ("error_depth", "/educational_pid/debug/error_depth"),
                    ("error_yaw", "/educational_pid/debug/error_yaw"),
                ],
            ),
        ]
    )
