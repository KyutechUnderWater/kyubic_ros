import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("p_pid_controller"),
        "config",
        "p_pid_controller_gain.yaml",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    action_component_container = ComposableNodeContainer(
        name="measure_parameter_component_container",
        namespace="test",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                name="measure_paramter_component",
                namespace="controller/p_pid_controller",
                package="p_pid_controller",
                plugin="controller::MeasureParam",
                remappings=[
                    ("/controller/p_pid_controller/odom", "/localization/odom"),
                    ("/controller/p_pid_controller/robot_force", "/driver/robot_force"),
                ],
                parameters=[{"pid_gain_yaml": config_path}],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
        output="screen",
        arguments=[
            "--ros-args",
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
    )

    return LaunchDescription(
        [
            log_level_arg,
            action_component_container,
        ]
    )
