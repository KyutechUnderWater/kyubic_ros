import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("educational_pid_robot_prep")
    controller_config = os.path.join(
        package_share,
        "config",
        "controller_day4.yaml",
    )
    gate_config = os.path.join(
        package_share,
        "config",
        "safety_gate_robot.yaml",
    )

    allow_depth = LaunchConfiguration("allow_depth")
    allow_yaw = LaunchConfiguration("allow_yaw")
    max_force_z = LaunchConfiguration("max_force_z")
    max_torque_z = LaunchConfiguration("max_torque_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "allow_depth",
                default_value="false",
                description="Pass the PID depth force to the actuator",
            ),
            DeclareLaunchArgument(
                "allow_yaw",
                default_value="false",
                description="Pass the PID yaw torque to the actuator",
            ),
            DeclareLaunchArgument(
                "max_force_z",
                default_value="2.0",
                description="Absolute Day 4 depth-force limit [N]",
            ),
            DeclareLaunchArgument(
                "max_torque_z",
                default_value="0.5",
                description="Absolute Day 4 yaw-torque limit [N m]",
            ),
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
                    (
                        "integral_depth",
                        "/educational_pid/debug/integral_depth",
                    ),
                    ("integral_yaw", "/educational_pid/debug/integral_yaw"),
                ],
            ),
            Node(
                package="educational_pid_robot_prep",
                executable="safety_gate_node",
                name="educational_pid_safety_gate",
                output="screen",
                parameters=[
                    gate_config,
                    {
                        "allow_depth": ParameterValue(allow_depth, value_type=bool),
                        "allow_yaw": ParameterValue(allow_yaw, value_type=bool),
                        "max_force_z": ParameterValue(max_force_z, value_type=float),
                        "max_torque_z": ParameterValue(
                            max_torque_z,
                            value_type=float,
                        ),
                    },
                ],
                remappings=[
                    ("raw_robot_force", "/educational_pid_day4/raw_robot_force"),
                    (
                        "safe_robot_force",
                        "/driver/actuator_rp2040_driver/robot_force",
                    ),
                    (
                        "heartbeat",
                        "/driver/actuator_rp2040_driver/heartbeat",
                    ),
                    ("armed", "/educational_pid_day4/armed"),
                    ("fault", "/educational_pid_day4/fault"),
                ],
            ),
        ]
    )
