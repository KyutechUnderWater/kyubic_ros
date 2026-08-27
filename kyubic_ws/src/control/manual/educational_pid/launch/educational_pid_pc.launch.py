import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("educational_pid"), "config", "pid_gains.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="educational_pid",
                executable="pid_node",
                name="educational_pid_node",
                output="screen",
                parameters=[config],
                remappings=[
                    ("depth_odom", "/educational_pid/mock/depth_odom"),
                    ("imu_odom", "/educational_pid/mock/imu_odom"),
                    ("joy_common", "/educational_pid/mock/joy_common"),
                    ("robot_force", "/educational_pid/test_robot_force"),
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
            Node(
                package="educational_pid",
                executable="mock_plant_node",
                name="educational_pid_mock_plant",
                output="screen",
                remappings=[
                    ("depth_odom", "/educational_pid/mock/depth_odom"),
                    ("imu_odom", "/educational_pid/mock/imu_odom"),
                    ("robot_force", "/educational_pid/test_robot_force"),
                    ("localization_odom", "/localization/odom"),
                ],
            ),
        ]
    )
