#!/bin/bash

		
############################################################################
## Update host
## sudo: unable to resolve host kyubic: Temporary failure in name resolution
############################################################################
sudo sh -c 'echo 172.30.51.207 kyubic >> /etc/hosts'

echo $USER_ID
usermod -u $USER_ID -G sudo -o ros
groupmod -g $GROUP_ID ros
chown ros:ros -R /home/ros


gosu ros bash -l -c 'source /opt/ros/$ROS_DISTRO/setup.bash &&
	git config --global --add safe.directory /home/ros/kyubic_ros &&
	cd ~/kyubic_ros/kyubic_ws && colcon build --symlink-install'
	

/bin/bash
