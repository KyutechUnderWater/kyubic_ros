import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 🌟 修正ポイント2: BlueROV用のゲインYAMLファイルを直接指定
    bluerov_config_path = os.path.join(
        get_package_share_directory("bluerov_control_bt_nodes"),
        "config",
        "p_pid_controller_gain_bluerov.yaml",
    )

    p_pid_controller_path = get_package_share_directory("p_pid_controller")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    return LaunchDescription(
        [
            log_level_arg,
            Node(
                package="wrench_planner",
                namespace="planner/wrench_planner",
                executable="wrench_planner_component_node",
                remappings=[
                    ("odom", "/localization/odom"),
                    # 🌟 修正ポイント1: 出力先をBlueROVのMavlinkに変更
                    ("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
                    ("targets", "/rt_pose_plotter/targets"),
                ],
                parameters=[
                    {"p_pid_controller_path": p_pid_controller_path},
                    {"pid_gain_yaml": bluerov_config_path}, # 確実なパスを渡す
                ],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
            ),
        ]
    )