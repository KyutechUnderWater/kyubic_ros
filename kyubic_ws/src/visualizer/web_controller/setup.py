from setuptools import find_packages, setup

package_name = "web_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["./launch/web_controller.launch.py"]),
        ("assets/", ["./assets/KYUBIC_transformed.stl"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryohei Ohnishi",
    maintainer_email="kyutech.robosub@gmail.com",
    description="controller, trajectory view, robot status graph",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [f"{package_name} = {package_name}.main:main"],
    },
)
