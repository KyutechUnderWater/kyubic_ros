import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("joy_common"), "config", "joy_common.param.yaml"
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    action_component_container = ComposableNodeContainer(
        name="joy_common_component_container",
        namespace="joy_common",
        package="rclcpp_components",
        executable="component_container_isolated",
        output="screen",
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        arguments=[
            "--use_multi_threaded_executor",
        ],
        composable_node_descriptions=[
            ComposableNode(
                name="joy_common_component",
                namespace="joy_common",
                package="joy_common",
                plugin="joy_common::JoyCommon",
                remappings=[("/joy", "/joy_common/joy")],
                parameters=[config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="joy",
                namespace="joy_common",
                package="joy",
                plugin="joy::Joy",
                parameters=[config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
    )

    return LaunchDescription([log_level_arg, action_component_container])
