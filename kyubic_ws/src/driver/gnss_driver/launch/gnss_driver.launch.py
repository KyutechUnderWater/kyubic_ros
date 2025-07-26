# gnss_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    GNSSドライバーノードを起動するためのLaunchDescriptionを生成します。
    """

    gnss_node = Node(
        package='gnss_driver',
        executable='gnss_driver',
        name='gnss_publisher_node',  # これはノード名
        namespace='driver',             # これが名前空間
        output='screen',
    )

    return LaunchDescription([
        gnss_node
    ])