from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription


def generate_launch_description():
    action_component_container = ComposableNodeContainer(
        name="localization_component_container",
        namespace="localization",
        package="rclcpp_components",
        executable="component_container_isolated",
        composable_node_descriptions=[
            ComposableNode(
                name="depth_odometry_component",
                namespace="localization/depth",
                package="localization",
                plugin="localization::DepthOdometry",
                remappings=[("/localization/depth/depth", "/driver/depth")],
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
                remappings=[],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],  # enable intra-process communication
            ),
        ],
        output="screen",
        arguments=["--use_multi_threaded_executor"],
    )

    return LaunchDescription([action_component_container])
