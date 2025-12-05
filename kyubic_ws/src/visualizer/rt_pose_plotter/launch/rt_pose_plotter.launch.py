from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/localization/odom",
        description="Topic name for the odometry message",
    )
    targets_topic_arg = DeclareLaunchArgument(
        "targets_topic",
        default_value="/rt_pose_plotter/targets",
        description="Topic name for the odometry message",
    )

    # --- ノードの定義 ---
    plotter_node = Node(
        name="rt_pose_plotter",
        namespace="visualizer",
        package="rt_pose_plotter",
        executable="rt_pose_plotter",
        output="screen",
        remappings=[
            ("odom", LaunchConfiguration("odom_topic")),
            ("targets", LaunchConfiguration("targets_topic")),
        ],
    )

    # --- LaunchDescriptionの生成 ---
    # 宣言した引数とノードをLaunchDescriptionに含めて返す
    return LaunchDescription(
        [
            odom_topic_arg,
            targets_topic_arg,
            plotter_node,
        ]
    )
