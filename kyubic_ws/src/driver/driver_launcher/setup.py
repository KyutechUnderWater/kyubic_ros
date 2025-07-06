from setuptools import find_packages, setup

package_name = "driver_launcher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["driver.launch.py"]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ros",
    maintainer_email="kyutech.robosub@gmail.com",
    description="launcher to launch driver packages",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
