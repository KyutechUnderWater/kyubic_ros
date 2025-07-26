import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch引数の宣言 ---
    # Launch実行時にコマンドラインから値を変更できる
    # 例: ros2 launch ... odom_topic:=/robot1/odom

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/localization/odom",
        description="Topic name for the odometry message",
    )

    # --- ノードの定義 ---
    plotter_node = Node(
        name="real_time_plotter",
        namespace="real_time_plotter",
        package="real_time_plotter",
        # setup.pyで設定する実行可能ファイル名
        executable="real_time_plotter",
        output="screen",
        # トピック名のリマッピング設定
        remappings=[
            ("/real_time_plotter/odom", LaunchConfiguration("odom_topic")),
        ],
    )

    # --- LaunchDescriptionの生成 ---
    # 宣言した引数とノードをLaunchDescriptionに含めて返す
    return LaunchDescription(
        [
            odom_topic_arg,
            plotter_node,
        ]
    )
