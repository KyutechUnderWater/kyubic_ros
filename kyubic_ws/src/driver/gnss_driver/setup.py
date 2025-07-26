import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gnss_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takeshi Yamagami',
    maintainer_email='kyutech.robosub@gmail.com',
    description='Python driver msg package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gnss_driver = gnss_driver.gnss_driver:main',
            'gnss_driver_2 = gnss_driver.gnss_driver_2:main',
        ],
    },
)
