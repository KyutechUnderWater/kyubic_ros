from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic_name",
        default_value="/localization/odom",
        description="Topic name for odometry messages",
    )

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (info, warn, error, etc.)",
    )

    # --- Nodeの定義 ---
    viewer_node = Node(
        package="trajectory_viewer",
        executable="odom_viewer",
        name="odometory_viewer",
        output="screen",
        emulate_tty=True,
        remappings=[
            ("odom", LaunchConfiguration("odom_topic_name")),
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    # --- LaunchDescriptionの生成 ---
    # 宣言した引数とノードをLaunchDescriptionに渡して返す
    return LaunchDescription([odom_topic_arg, log_level_arg, viewer_node])
