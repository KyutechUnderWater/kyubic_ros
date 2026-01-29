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
            FindPackageShare("rf_underwater_driver"),
            "config",
            "rf_underwater_driver.param.yaml",
        ]
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    driver_component = ComposableNode(
        package="rf_underwater_driver",
        plugin="rf_underwater_driver::RFUnderwaterDriver",
        namespace="rf_underwater_driver",
        name="rf_underwater_driver",
        parameters=[config],
        remappings=[("pdla_feedback", "/planner/pdla_planner/pdla_plan/_action/feedback")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name="rf_underwater_driver_container",
        namespace="rf_underwater_driver",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[driver_component],
        output="screen",
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    return LaunchDescription(
        [
            log_level_arg,
            container,
        ]
    )
