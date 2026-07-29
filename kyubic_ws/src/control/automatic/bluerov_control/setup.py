import os
from glob import glob
from setuptools import find_packages, setup

package_name = "bluerov_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kyutech Underwater",
    maintainer_email="kyutech.robosub@gmail.com",
    description="BlueROV mission state-machine control node.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bluerov_control_node = bluerov_control.node:main",
            "bluerov_heartbeat_publisher = bluerov_control.heartbeat_publisher:main",
        ],
    },
)
