from setuptools import find_packages, setup

package_name = "trajectory_viewer"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/trajectory_viewer.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryohei Ohnishi",
    maintainer_email="kyutech.robosub@gmail.com",
    description="",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            f"trajectory_viewer = {package_name}.main:main",
            f"publish_odom_from_csv = {package_name}.publish_odom_from_csv:main",
        ],
    },
)
