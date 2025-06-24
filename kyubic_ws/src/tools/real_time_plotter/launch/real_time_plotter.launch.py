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
    target_vx_topic_arg = DeclareLaunchArgument(
        "target_vx_topic",
        default_value="/test_pid/target_x",
        description="Topic name for the target X-velocity",
    )
    target_vy_topic_arg = DeclareLaunchArgument(
        "target_vy_topic",
        default_value="/test_pid/target_y",
        description="Topic name for the target Y-velocity",
    )
    target_vz_topic_arg = DeclareLaunchArgument(
        "target_vz_topic",
        default_value="/test_pid/target_z",
        description="Topic name for the target Z-velocity",
    )
    target_wz_topic_arg = DeclareLaunchArgument(
        "target_wz_topic",
        default_value="/test_pid/target_yaw",
        description="Topic name for the target Z-angular velocity",
    )

    # --- ノードの定義 ---
    plotter_node = Node(
        package="real_time_plotter",
        # setup.pyで設定する実行可能ファイル名
        executable="real_time_plotter",
        name="real_time_plotter",
        output="screen",
        # トピック名のリマッピング設定
        remappings=[
            ("/odom", LaunchConfiguration("odom_topic")),
            ("/target_vel_x", LaunchConfiguration("target_vx_topic")),
            ("/target_vel_y", LaunchConfiguration("target_vy_topic")),
            ("/target_vel_z", LaunchConfiguration("target_vz_topic")),
            ("/target_ang_z", LaunchConfiguration("target_wz_topic")),
        ],
    )

    # --- LaunchDescriptionの生成 ---
    # 宣言した引数とノードをLaunchDescriptionに含めて返す
    return LaunchDescription(
        [
            odom_topic_arg,
            target_vx_topic_arg,
            target_vy_topic_arg,
            target_vz_topic_arg,
            target_wz_topic_arg,
            plotter_node,
        ]
    )
