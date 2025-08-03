import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch引数の宣言 ---
    global_pose_topic_arg = DeclareLaunchArgument(
        "global_pose_topic",
        default_value="/localization/global_pose",
        description="Topic name for the odometry message",
    )

    # --- ノードの定義 ---
    plotter_node = Node(
        name="oak_create_mapping",
        namespace="oak_create_mapping",
        package="oak_create_mapping",
        executable="oak_create_mapping",
        output="screen",
        remappings=[
            (
                "/oak_create_mapping/global_pose",
                LaunchConfiguration("global_pose_topic"),
            ),
        ],
    )

    # --- LaunchDescriptionの生成 ---
    # 宣言した引数とノードをLaunchDescriptionに含めて返す
    return LaunchDescription(
        [
            global_pose_topic_arg,
            plotter_node,
        ]
    )
