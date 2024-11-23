#!/bin/bash

source /opt/ros/$ROS_DISTRO/setup.bash

cd /home/kyubic_ros/kyubic_ws
git config --global --add safe.directory /home/kyubic_ros
colcon build --symlink-install

/bin/bash
