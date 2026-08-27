import os
from glob import glob

from setuptools import find_packages, setup


package_name = "educational_pid_robot_prep"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kyutech Underwater Robotics Team",
    maintainer_email="kyutech.robosub@gmail.com",
    description="Day 4 safety gate for educational PID robot preparation.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_gate_node = educational_pid_robot_prep.safety_gate_node:main",
            "safety_gate_student = educational_pid_robot_prep.safety_gate_student:main",
            "sensor_preview_node = educational_pid_robot_prep.sensor_preview_node:main",
        ],
    },
)
