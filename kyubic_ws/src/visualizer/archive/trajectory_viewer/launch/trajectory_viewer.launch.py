from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # --- Launch Argumentの宣言 ---
    # launch時に渡す引数を定義。デフォルト値を設定できる。
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/localization/odom",
        description="Topic name for odometry messages",
    )
    targets_topic_arg = DeclareLaunchArgument(
        "targets_topic",
        default_value="/real_time_plotter/targets",
        description="Topic name for the target Z position",
    )

    # --- Nodeの定義 ---
    viewer_node = Node(
        package="trajectory_viewer",
        executable="trajectory_viewer",
        name="trajectory_viewer",
        output="screen",
        emulate_tty=True,
        remappings=[
            # (スクリプト内のトピック名, Launch引数で受け取った値)
            ("odom", LaunchConfiguration("odom_topic")),
            ("targets", LaunchConfiguration("targets_topic")),
        ],
    )

    # --- LaunchDescriptionの生成 ---
    # 宣言した引数とノードをLaunchDescriptionに渡して返す
    return LaunchDescription([odom_topic_arg, targets_topic_arg, viewer_node])
