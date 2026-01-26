from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [
            FindPackageShare("wrench_planner"),
            "config",
            "wrench_planner.param.yaml",
        ]
    )

    p_pid_controller_path = FindPackageShare("p_pid_controller")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    composable_nodes = [
        ComposableNode(
            package="wrench_planner",
            plugin="planner::wrench_planner::ZeroOrderHold",
            namespace="planner/wrench_planner",
            name="zoh_wrench_planner",
            remappings=[("odom", "/localization/odom")],
            parameters=[config],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="wrench_planner",
            plugin="planner::wrench_planner::WrenchPlanner",
            namespace="planner/wrench_planner",
            name="wrench_planner",
            remappings=[
                ("odom", "/localization/odom"),
                ("robot_force", "/driver/actuator_rp2040_driver/robot_force"),
                ("targets", "/rt_pose_plotter/targets"),
            ],
            parameters=[
                {"p_pid_controller_path": p_pid_controller_path},
                config,
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]

    container = ComposableNodeContainer(
        name="wrench_planner_container",
        namespace="wrench_planner",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        output="screen",
    )

    return LaunchDescription(
        [
            log_level_arg,
            container,
        ]
    )
