import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # joy2wrenchノード用の設定ファイルを指定
    config = os.path.join(
        get_package_share_directory(
            "joy2wrench"), "config", "joy2wrench.param.yaml"
    )

    # joy_nodeの起動設定
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            'dev': '/dev/input/js0',
            'deadzone': 0.1
        }],
        # joyトピックをjoy2wrenchノードが購読できるようにリマッピング
        remappings=[
            ('/joy', '/joy2wrench/joy')
        ]
    )

    # joy2wrenchノードの起動設定
    joy2wrench_node = Node(
        package="joy2wrench",
        executable="joy2wrench",
        name="joy2wrench",
        namespace="joy2wrench",
        parameters=[config], # YAMLからodom_topicなどを読み込む
        output="screen",
    )

    return LaunchDescription([
        joy_node,
        joy2wrench_node
    ])
