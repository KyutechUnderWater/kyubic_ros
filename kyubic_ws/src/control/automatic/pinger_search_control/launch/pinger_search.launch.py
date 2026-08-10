"""BlueROVピンガー探査ミッションノードを起動する。

前提として driver_launcher の blue_rov_driver.launch.py (mavlink_driver等)、
reed_switch_driver、ハイドロフォン(/pinger_direction)、YOLO(buoy_detector)が
別途起動済みであること。DVL・localizationは使用しない。

heartbeatはpinger_search_node自身が常時true送信するため、別ノードは不要。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _config_dir() -> str:
    """パラメータyamlのディレクトリを返す。ソース側があればそちらを優先する。

    installにはビルド時のコピーが置かれるため、通常はyamlを直すたびに
    colcon buildが必要になる。ここでワークスペースの src/ 配下のconfigを
    直接参照することで、yaml編集は再ビルドなしで次のlaunchから反映される。
    """
    share = get_package_share_directory("pinger_search_control")
    # share = <ws>/install/pinger_search_control/share/pinger_search_control
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share))))
    src_config = os.path.join(
        ws_root, "src", "control", "automatic", "pinger_search_control", "config"
    )
    if os.path.isdir(src_config):
        return src_config
    return os.path.join(share, "config")


def generate_launch_description() -> LaunchDescription:
    log_level = LaunchConfiguration("log_level")

    # config_file引数でパラメータセットを切替可能:
    #   本番(4m)         : pinger_search.param.yaml (既定)
    #   練習用0.7mプール : pinger_search_pool_0_7m.param.yaml
    config = PathJoinSubstitution([_config_dir(), LaunchConfiguration("config_file")])

    pinger_search_node = Node(
        package="pinger_search_control",
        executable="pinger_search_node",
        name="pinger_search_node",
        namespace="control/automatic/pinger_search",
        output="screen",
        parameters=[config],
        remappings=[
            ("mission_start_trigger", "/driver/blue_rov/mission_start_trigger"),
            ("imu", "/driver/imu"),
            ("depth", "/driver/depth"),
            ("pinger_direction", "/pinger_direction"),
            # buoy_detector_node の output_topic 既定値に合わせている
            ("buoy_position", "/buoy/relative_position"),
            ("vehicle_state", "/driver/blue_rov/mavlink_driver/vehicle_state"),
            ("robot_force", "/driver/blue_rov/mavlink_driver/robot_force"),
            ("heartbeat", "/driver/blue_rov/mavlink_driver/heartbeat"),
            ("set_armed", "/driver/blue_rov/mavlink_driver/set_armed"),
            # 画処理中のライト点灯(mavlink_driverのled購読、右ライト=RC9)
            ("led", "/driver/blue_rov/mavlink_driver/led"),
            # 画処理スタート用bool(このパッケージで新設したトピック)
            ("image_processing_enable", "/perception/image_processing_enable"),
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="pinger_search.param.yaml",
                description=(
                    "Parameter file name in the package config directory "
                    "(e.g. pinger_search_pool_0_7m.param.yaml for the 0.7 m practice pool)"
                ),
            ),
            pinger_search_node,
        ]
    )
