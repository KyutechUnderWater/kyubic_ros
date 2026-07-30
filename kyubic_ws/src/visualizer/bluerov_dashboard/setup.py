from glob import glob
import os

from setuptools import find_packages, setup

package_name = "bluerov_dashboard"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    # opencv-python(pip)は同梱するとcv_bridgeが期待するapt版python3-opencvと
    # 衝突するため入れない。cv2はpython3-opencv(apt)から提供される前提。
    install_requires=["setuptools", "nicegui"],
    zip_safe=True,
    maintainer="Kyutech Underwater",
    maintainer_email="kyutech.robosub@gmail.com",
    description="Web-based operational dashboard for BlueROV.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"bluerov_dashboard = {package_name}.main:main",
        ],
    },
)
