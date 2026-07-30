from setuptools import find_packages, setup

package_name = "buoy_detector_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/buoy_detector_driver.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kyutech Underwater",
    maintainer_email="kyutech.robosub@gmail.com",
    description=(
        "Adapter node that converts BuoyRelativePosition into BuoyDetection for "
        "bluerov_control_bt_nodes's CheckBuoyDetected."
    ),
    license="Apache License 2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [f"buoy_detector_driver = {package_name}.main:main"],
    },
)
