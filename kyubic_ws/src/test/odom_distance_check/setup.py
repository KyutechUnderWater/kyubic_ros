from setuptools import find_packages, setup

package_name = "odom_distance_check"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/odom_distance_check.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kyutech Underwater",
    maintainer_email="kyutech.robosub@gmail.com",
    description=(
        "Read-only tool that records odom position deltas between operator-marked points, "
        "for manual comparison against a known physical distance."
    ),
    license="Apache License 2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [f"odom_distance_check = {package_name}.main:main"],
    },
)
