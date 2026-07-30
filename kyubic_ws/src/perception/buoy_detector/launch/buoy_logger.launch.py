from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    input_image_topic = LaunchConfiguration("input_image_topic")
    input_detection_topic = LaunchConfiguration("input_detection_topic")
    output_dir = LaunchConfiguration("output_dir")
    save_rate_hz = LaunchConfiguration("save_rate_hz")
    image_format = LaunchConfiguration("image_format")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "input_image_topic",
                default_value="/camera/image_raw",
            ),
            DeclareLaunchArgument(
                "input_detection_topic",
                default_value="/buoy/relative_position",
            ),
            DeclareLaunchArgument(
                "output_dir",
                default_value="~/buoy_logs",
                description="画像とCSVの保存先ディレクトリ",
            ),
            DeclareLaunchArgument(
                "save_rate_hz",
                default_value="1.5",
                description="画像・検出結果を保存する頻度[Hz]",
            ),
            DeclareLaunchArgument(
                "image_format",
                default_value="jpg",
                description="保存する画像形式('jpg'または'png')",
            ),
            Node(
                package="buoy_detector",
                executable="buoy_logger_node",
                name="buoy_logger",
                output="screen",
                parameters=[
                    {
                        "input_image_topic": input_image_topic,
                        "input_detection_topic": input_detection_topic,
                        "output_dir": output_dir,
                        "save_rate_hz": ParameterValue(
                            save_rate_hz,
                            value_type=float,
                        ),
                        "image_format": image_format,
                    }
                ],
            ),
        ]
    )
