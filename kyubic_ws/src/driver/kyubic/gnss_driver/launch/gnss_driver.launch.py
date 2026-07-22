# gnss_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    GNSSドライバーノードを起動するためのLaunchDescriptionを生成
    """

    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (info, warn, error, etc.)",
    )

    log_level_config = LaunchConfiguration("log_level")

    gnss_node = Node(
        package="gnss_driver",
        executable="gnss_driver",
        name="gnss_publisher_node",  # これはノード名
        namespace="driver/gnss_driver",  # これが名前空間
        output="screen",
        arguments=["--ros-args", "--log-level", log_level_config],
    )

    return LaunchDescription([log_level_arg, gnss_node])
