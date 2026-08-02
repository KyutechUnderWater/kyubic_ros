from glob import glob
import os
from setuptools import find_packages, setup


package_name = "buoy_detector"

setup(
    name=package_name,
    version="0.6.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "models"),
            glob(os.path.join("models", ".pt")),
            
        ),
        (
            os.path.join("share", package_name, "models", "best_ncnn_model"),
            glob(os.path.join("models", "best_ncnn_model", "*.*"))
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description=(
        "ROS 2 node for YOLO buoy detection, base_link FRD position, "
        "yaw estimation, normalized horizontal/vertical image-center error, "
        "and optional relative position."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "buoy_detector_node = "
            "buoy_detector.buoy_detector_node:main",
            "buoy_logger_node = "
            "buoy_detector.buoy_logger_node:main",
        ],
    },
)
