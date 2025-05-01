#!/bin/bash

## Resolve hostname
## sudo: unable to resolve host kyubic: Temporary failure in name resolution
sh -c 'echo 172.30.51.207 kyubic >> /etc/hosts'

usermod -u $USER_ID -g $GROUP_ID -G sudo ros
chown ros:ros -R /home/ros

gosu ros bash -l -c 'cd ~/kyubic_ros && 
	uv venv --system-site-packages &&
	uv sync &&
	echo source /home/ros/kyubic_ros/.venv/bin/activate >> /home/ros/.bashrc'

gosu ros bash -l -c 'source /opt/ros/$ROS_DISTRO/setup.bash &&
	git config --global --add safe.directory /home/ros/kyubic_ros &&
	cd ~/kyubic_ros/kyubic_ws && colcon build --symlink-install'

/bin/bash
