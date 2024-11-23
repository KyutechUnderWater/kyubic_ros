#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

cd /home/kyubic_ros/kyubic_ws
colcon build --symlink-install

/bin/bash
