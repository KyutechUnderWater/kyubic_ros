from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_topic",
                default_value="/buoy/relative_position",
                description="perception/buoy_detectorが発行するBuoyRelativePositionトピック",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/perception/buoy_detection",
                description="CheckBuoyDetected(bluerov_control_bt_nodes)が購読するBuoyDetectionトピック",
            ),
            Node(
                package="buoy_detector_driver",
                executable="buoy_detector_driver",
                name="buoy_detector_driver",
                output="screen",
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                    }
                ],
            ),
        ]
    )
