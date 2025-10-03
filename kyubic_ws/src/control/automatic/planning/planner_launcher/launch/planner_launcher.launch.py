import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    cfg_pdla_planner = os.path.join(
        get_package_share_directory("projection_dynamic_look_ahead_planner"),
        "config",
        "pdla_planner.param.yaml",
    )
    path_planner_path = os.path.join(get_package_share_directory("path_planner"))

    cfg_wrench_planner = os.path.join(
        get_package_share_directory("wrench_planner"),
        "config",
        "wrench_planner.param.yaml",
    )
    p_pid_controller_path = get_package_share_directory("p_pid_controller")

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    action_component_container = ComposableNodeContainer(
        name="planner_launcher_component_container",
        namespace="planner",
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
                name="pdla_planner_component",
                namespace="planner/pdla_planner",
                package="projection_dynamic_look_ahead_planner",
                plugin="planner::PDLAPlanner",
                remappings=[
                    ("/planner/pdla_planner/odom", "/localization/odom"),
                ],
                parameters=[
                    {"path_planner_path": path_planner_path},
                    cfg_pdla_planner,
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="wrench_planner_component",
                namespace="planner/wrench_planner",
                package="wrench_planner",
                plugin="planner::WrenchPlanner",
                remappings=[
                    (
                        "/planner/wrench_planner/goal_current_odom",
                        "/planner/pdla_planner/goal_current_odom",
                    ),
                    ("/planner/wrench_planner/robot_force", "/driver/robot_force"),
                    ("/planner/wrench_planner/targets", "/rt_pose_plotter/targets"),
                ],
                parameters=[
                    {"p_pid_controller_path": p_pid_controller_path},
                    cfg_wrench_planner,
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
    )

    return LaunchDescription([log_level_arg, action_component_container])
