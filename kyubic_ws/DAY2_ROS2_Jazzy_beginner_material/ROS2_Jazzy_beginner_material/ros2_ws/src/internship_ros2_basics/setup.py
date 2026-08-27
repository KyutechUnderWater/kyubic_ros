from glob import glob
import os

from setuptools import find_packages, setup


package_name = "internship_ros2_basics"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Internship TA",
    maintainer_email="ta@example.com",
    description="Beginner ROS 2 Jazzy communication examples",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "string_publisher = internship_ros2_basics.string_publisher:main",
            "string_subscriber = internship_ros2_basics.string_subscriber:main",
            "turtle_circle_publisher = internship_ros2_basics.turtle_circle_publisher:main",
            "turtle_pose_subscriber = internship_ros2_basics.turtle_pose_subscriber:main",
            "add_two_ints_server = internship_ros2_basics.add_two_ints_server:main",
            "add_two_ints_client = internship_ros2_basics.add_two_ints_client:main",
            "spawn_turtle_client = internship_ros2_basics.spawn_turtle_client:main",
            "turtle_rotate_action_client = internship_ros2_basics.turtle_rotate_action_client:main",
            "fibonacci_action_server = internship_ros2_basics.fibonacci_action_server:main",
            "fibonacci_action_client = internship_ros2_basics.fibonacci_action_client:main",
        ],
    },
)
