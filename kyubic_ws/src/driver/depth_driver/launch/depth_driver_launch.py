import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('depth_driver'),
        'config',
        'depth_driver.param.yaml'
    )

    return LaunchDescription([
        Node(
            package='depth_driver',
            namespace='depth_driver',
            executable='depth_driver',
            parameters=[config],
            output="screen",
        )
    ])
