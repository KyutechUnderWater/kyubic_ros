# kyubic_ros

# Features
- ROS 2 Jazzy
- Container independent environment
- Automatic environment building

## Requirement
- Docker >= 28.2.2
- Docker Compose >= 2.36.2

## Installation
Create docker container and Setup continer (user password, enviroment variable, alias, and udev rule)
~~~bash
git clone git@github.com:KyutechUnderWater/kyubic_ros.git
cd $_ && . install.sh {your_password}
~~~

## How to Use
Start container and Run bash
~~~bash
ros2_start
~~~

Exit container
~~~bash
exit # or <ctrl + d>
~~~

## Uninstall
~~~bash
. uninstall.sh
~~~

## Tips
### Using ROS 2 Humble
~~~bash
git clone git@github.com:KyutechUnderWater/kyubic_ros.git -b humble kyubic_ros_humble
cd $_ && . install.sh {your_password}
ros2_start_humble  # Start container and Run bash
~~~
