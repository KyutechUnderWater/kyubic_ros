#!/bin/bash

# Set user and group
groupmod -g $GROUP_ID ros
usermod -u $USER_ID -g $GROUP_ID -G sudo ros

# Set passward
sh -c "echo "ros:$1" | chpasswd"

# Resolve hostname
## sudo: unable to resolve host kyubic: Temporary failure in name resolution
sh -c 'echo 192.168.9.100 kyubic >> /etc/hosts'

# Set python-env with uv
gosu ros bash -l -c 'cd ~/kyubic_ros && uv venv --system-site-packages'

# Add command to .bashrc
gosu ros bash -c 'cat << EOT >> ~/.bashrc
export ROS_DOMAIN_ID=1
source /home/ros/kyubic_ros/.venv/bin/activate
EOT'

# Set config for git
gosu ros bash -l -c 'git config --global --add safe.directory /home/ros/kyubic_ros'
