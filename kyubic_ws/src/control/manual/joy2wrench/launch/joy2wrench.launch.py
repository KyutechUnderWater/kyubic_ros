import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    joy_common_config = os.path.join(
        get_package_share_directory("joy_common"), "config", "joy_common.param.yaml"
    )
    config = os.path.join(
        get_package_share_directory("joy2wrench"), "config", "joy2wrench.param.yaml"
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    action_component_container = ComposableNodeContainer(
        name="joy2wrench_component_container",
        namespace="joy2wrench",
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
                name="joy",
                namespace="joy2wrench",
                package="joy",
                plugin="joy::Joy",
                parameters=[joy_common_config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="joy_common_component",
                namespace="joy2wrench",
                package="joy_common",
                plugin="joy_common::JoyCommon",
                parameters=[joy_common_config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="joy2wrench_component",
                namespace="joy2wrench",
                package="joy2wrench",
                plugin="joy2wrench::Joy2WrenchStamped",
                remappings=[("robot_force", "/driver/robot_force")],
                parameters=[config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_joy2wrench",
        namespace="joy2wrench",
        output="screen",
        parameters=[
            # configure と activate を両方実行
            {"autostart": True},
            {"node_names": ["joy2wrench_component"]},
            {"bond_timeout": 0.0},
        ],
    )

    return LaunchDescription(
        [log_level_arg, action_component_container, lifecycle_manager]
    )
