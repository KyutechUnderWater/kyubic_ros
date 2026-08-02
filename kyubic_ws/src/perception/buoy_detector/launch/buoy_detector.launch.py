from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration("model_path")
    config_path = LaunchConfiguration("config_path")
    input_image_topic = LaunchConfiguration("input_image_topic")
    output_topic = LaunchConfiguration("output_topic")
    body_frame_id = LaunchConfiguration("body_frame_id")
    imgsz = LaunchConfiguration("imgsz")
    confidence = LaunchConfiguration("confidence")
    device = LaunchConfiguration("device")
    class_id = LaunchConfiguration("class_id")
    position_estimation_enabled = LaunchConfiguration(
        "position_estimation_enabled"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("buoy_detector"), "models", "best_ncnn_model"]
                ),
                description="YOLO学習済みモデルのパス",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("buoy_detector"),
                        "config",
                        "buoy_position.yaml",
                    ]
                ),
                description="距離・カメラ設定YAMLのパス",
            ),
            DeclareLaunchArgument(
                "input_image_topic",
                default_value="/driver/blue_rov/camera/image_raw",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/buoy/relative_position",
            ),
            DeclareLaunchArgument(
                "body_frame_id",
                default_value="base_link",
                description=(
                    "出力座標が属する機体フレーム名。"
                    "この版ではX前・Y右・Z下のbase_linkを使用する"
                ),
            ),
            DeclareLaunchArgument("imgsz", default_value="320"),
            DeclareLaunchArgument("confidence", default_value="0.25"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("class_id", default_value="0"),
            DeclareLaunchArgument(
                "position_estimation_enabled",
                default_value="false",
                description=(
                    "falseの場合もYOLO検出とヨー角推定は行い、"
                    "position_valid=false、位置座標=NaNで配信する"
                ),
            ),
            Node(
                package="buoy_detector",
                executable="buoy_detector_node",
                name="buoy_detector",
                output="screen",
                parameters=[
                    {
                        "model_path": model_path,
                        "config_path": config_path,
                        "input_image_topic": input_image_topic,
                        "output_topic": output_topic,
                        "body_frame_id": body_frame_id,
                        "imgsz": ParameterValue(imgsz, value_type=int),
                        "confidence": ParameterValue(
                            confidence,
                            value_type=float,
                        ),
                        "device": device,
                        "class_id": ParameterValue(
                            class_id,
                            value_type=int,
                        ),
                        "position_estimation_enabled": ParameterValue(
                            position_estimation_enabled,
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
