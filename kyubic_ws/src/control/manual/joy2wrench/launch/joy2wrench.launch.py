import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnExecutionComplete
from launch_ros.actions import Node, LoadComposableNodes
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

    # コンポーネントコンテナ自体の定義
    container_node = Node(
        name="joy2wrench_component_container",
        namespace="joy2wrench",
        package="rclcpp_components",
        executable="component_container_isolated",
        output="screen",
        ros_arguments=[
            "--log-level",
            LaunchConfiguration("log_level"),
        ],
        parameters=[{"use_multi_threaded_executor": True}],
    )

    # コンポーネントのロード定義
    load_composable_nodes = LoadComposableNodes(
        target_container="joy2wrench/joy2wrench_component_container",
        composable_node_descriptions=[
            ComposableNode(
                name="joy",
                namespace="joy2wrench",
                package="joy",
                plugin="joy::Joy",
                parameters=[joy_common_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                name="joy_common_component",
                namespace="joy2wrench",
                package="joy_common",
                plugin="joy_common::JoyCommon",
                parameters=[joy_common_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                name="joy2wrench_component",
                namespace="joy2wrench",
                package="joy2wrench",
                plugin="joy2wrench::Joy2WrenchStamped",
                remappings=[("robot_force", "/driver/actuator_rp2040_driver/robot_force")],
                parameters=[config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    # ライフサイクルマネージャの定義
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
        [
            log_level_arg,
            container_node,
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=container_node,
                    on_start=[load_composable_nodes],
                )
            ),
            RegisterEventHandler(
                event_handler=OnExecutionComplete(
                    target_action=load_composable_nodes,
                    on_completion=[lifecycle_manager],
                )
            ),
        ]
    )
