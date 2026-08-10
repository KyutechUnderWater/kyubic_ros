from setuptools import find_packages, setup

package_name = "p_pid_gain_tuning_commander"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ryohei Ohnishi",
    maintainer_email="kyutech.robosub@gmail.com",
    description="One-shot body-frame step target commander for tuning p_pid_controller gains.",
    license="Apache License 2.0",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            f"step_commander = {package_name}.step_commander:main",
        ],
    },
)
