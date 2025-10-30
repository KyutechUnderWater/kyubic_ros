import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("trajectory_viewer"),
        "config",
        "global_pose_anchor.param.yaml",
    )

    global_pose_topic_arg = DeclareLaunchArgument(
        "global_pose_topic_name",
        default_value="/localization/global_pose",
        description="Topic name for global_pose messages",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (info, warn, error, etc.)",
    )

    # --- Nodeの定義 ---
    viewer_node = Node(
        package="trajectory_viewer",
        executable="global_pose_viewer",
        name="global_pose_viewer",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("global_pose", LaunchConfiguration("global_pose_topic_name")),
        ],
        parameters=[config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # --- LaunchDescriptionの生成 ---
    # 宣言した引数とノードをLaunchDescriptionに渡して返す
    return LaunchDescription([global_pose_topic_arg, log_level_arg, viewer_node])
