import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("educational_pid_robot_prep")
    controller_config = os.path.join(
        package_share,
        "config",
        "controller_day4.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="educational_pid",
                executable="pid_node",
                name="educational_pid_node",
                output="screen",
                parameters=[controller_config],
                remappings=[
                    ("depth_odom", "/localization/depth/odom"),
                    ("imu_odom", "/localization/imu/transformed"),
                    ("joy_common", "/joy_common/joy_common"),
                    ("robot_force", "/educational_pid_day4/raw_robot_force"),
                    ("targets", "/rt_pose_plotter/targets"),
                ],
            ),
            Node(
                package="educational_pid_robot_prep",
                executable="safety_gate_student",
                name="educational_pid_student_safety_gate",
                output="screen",
                remappings=[
                    ("raw_robot_force", "/educational_pid_day4/raw_robot_force"),
                    (
                        "safe_robot_force",
                        "/educational_pid_day4/student_preview_robot_force",
                    ),
                ],
            ),
            Node(
                package="educational_pid_robot_prep",
                executable="sensor_preview_node",
                name="educational_pid_sensor_preview",
                output="screen",
                remappings=[
                    ("depth_odom", "/localization/depth/odom"),
                    ("imu_odom", "/localization/imu/transformed"),
                    ("preview_odom", "/educational_pid_day4/preview_odom"),
                ],
            ),
        ]
    )
