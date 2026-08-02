from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    config_path = PathJoinSubstitution(
        [FindPackageShare("hydrophone_driver"), "config", "hydrophone_driver.param.yaml"]
    )
    serial_port = LaunchConfiguration("serial_port")
    topic_name = LaunchConfiguration("topic_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/ttyACM0",
                description="Hydrophone acquisition board serial port",
            ),
            DeclareLaunchArgument(
                "topic_name",
                default_value="/pinger_direction",
                description="PingerDirection publish topic (iwakuni2026.xml が購読)",
            ),
            Node(
                package="hydrophone_driver",
                executable="hydrophone_driver",
                name="hydrophone_driver",
                output="screen",
                parameters=[
                    config_path,
                    {
                        "serial_port": serial_port,
                        "topic_name": topic_name,
                    },
                ],
            ),
        ]
    )
