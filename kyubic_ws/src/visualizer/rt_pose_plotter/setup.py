from setuptools import find_packages, setup

package_name = "rt_pose_plotter"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/rt_pose_plotter.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryohei Ohnishi",
    maintainer_email="kyutech.robosub@gmail.com",
    description="",
    license="Apache License 2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [f"rt_pose_plotter = {package_name}.main:main"],
    },
)
