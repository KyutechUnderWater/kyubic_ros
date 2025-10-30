from setuptools import find_packages, setup

package_name = "trajectory_viewer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/trajectory_viewer.launch.py",
                "launch/odom_viewer.launch.py",
                "launch/global_pose_viewer.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/global_pose_anchor.param.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="kyutech.robosub@gmail.com",
    description="plot of odometry and latitude-longitude",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"odom_viewer = {package_name}.odom_viewer:main",
            f"global_pose_viewer = {package_name}.global_pose_viewer:main",
        ],
    },
)
