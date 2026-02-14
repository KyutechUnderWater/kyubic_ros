from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    action_component_container = ComposableNodeContainer(
        name="system_alert_component_container",
        namespace="tools",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        composable_node_descriptions=[
            ComposableNode(
                name="localization_error_component",
                namespace="system_alert",
                package="system_alert",
                plugin="tools::system_alert::LocalizationError",
                remappings=[("odom", "/localization/odom")],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
    )

    return LaunchDescription([log_level_arg, action_component_container])
