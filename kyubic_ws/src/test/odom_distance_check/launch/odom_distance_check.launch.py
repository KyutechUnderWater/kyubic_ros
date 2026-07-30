from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    odom_topic = LaunchConfiguration("odom_topic")
    output_dir = LaunchConfiguration("output_dir")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/localization/odom",
                description="記録対象のodomトピック",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="~/odom_distance_check_logs",
                description="CSVログの保存先ディレクトリ",
            ),
            Node(
                package="odom_distance_check",
                executable="odom_distance_check",
                name="odom_distance_check",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "odom_topic": odom_topic,
                        "output_dir": output_dir,
                    }
                ],
            ),
        ]
    )
