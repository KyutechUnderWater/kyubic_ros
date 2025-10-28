import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("localization"), "config", "localization.param.yaml"
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value=["info"],
        description="Logging level",
    )

    action_component_container = ComposableNodeContainer(
        name="localization_component_container",
        namespace="localization",
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
                name="depth_odometry_component",
                namespace="localization/depth",
                package="localization",
                plugin="localization::DepthOdometry",
                remappings=[("/localization/depth/depth", "/driver/depth")],
                parameters=[config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="imu_transform_component",
                namespace="localization/imu",
                package="localization",
                plugin="localization::IMUTransform",
                remappings=[("/localization/imu/imu", "/driver/imu")],
                parameters=[config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="dvl_odometry_component",
                namespace="localization/dvl",
                package="localization",
                plugin="localization::DVLOdometry",
                remappings=[
                    ("/localization/dvl/dvl", "/driver/dvl"),
                    (
                        "/localization/dvl/transformed_imu",
                        "/localization/imu/transformed",
                    ),
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
            ComposableNode(
                name="localization_component",
                namespace="localization",
                package="localization",
                plugin="localization::Localization",
                remappings=[("/localization/gnss", "/driver/gnss")],
                parameters=[config],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
    )

    return LaunchDescription([log_level_arg, action_component_container])
