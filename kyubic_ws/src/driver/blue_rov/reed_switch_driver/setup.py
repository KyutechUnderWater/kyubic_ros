"""Setuptools configuration for the reed switch mission-start trigger driver."""

import os
from glob import glob

from setuptools import find_packages, setup

package_name = "reed_switch_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*.yaml"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kyutech Underwater",
    maintainer_email="kyutech.robosub@gmail.com",
    description="Competition mission-start reed switch driver.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "reed_switch_driver = reed_switch_driver.reed_switch_driver_node:main",
        ],
    },
)
