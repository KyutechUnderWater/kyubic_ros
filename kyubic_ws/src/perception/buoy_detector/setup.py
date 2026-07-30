from glob import glob
import os
from setuptools import find_packages, setup


package_name = "buoy_detector"


def model_data_files(models_dir: str, package_name: str) -> list[tuple[str, list[str]]]:
    """NCNNエクスポート等のサブフォルダ構造を保ったままmodels/を一括登録する。"""
    entries = []
    for dirpath, _dirnames, filenames in os.walk(models_dir):
        if not filenames:
            continue
        install_dir = os.path.join("share", package_name, dirpath)
        source_files = [os.path.join(dirpath, filename) for filename in filenames]
        entries.append((install_dir, source_files))
    return entries


setup(
    name=package_name,
    version="0.3.0",
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
        *model_data_files("models", package_name),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="maintainer",
    maintainer_email="maintainer@example.com",
    description=(
        "ROS 2 node for YOLO buoy detection and base_link FRD position "
        "estimation."
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
