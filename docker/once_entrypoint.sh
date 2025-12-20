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
gosu ros bash -l -c 'cd ~/kyubic_ros && uv venv --system-site-packages && uv sync'

# Add command to .bashrc
gosu ros bash -c 'cat << EOT >> ~/.bashrc
export BYOBU_CONFIG_DIR=/home/ros/kyubic_ros/docker/byobu
export ROS_DOMAIN_ID=1
source /home/ros/kyubic_ros/.venv/bin/activate
EOT'

# Add aliases to .bash_aliases
gosu ros bash -c 'cat << EOT >> ~/.bash_aliases
alias build="colcon build --symlink-install --cmake-args -GNinja"
alias kyubic_byobu="$HOME/kyubic_ros/docker/script/kyubic_byobu.sh"
EOT'

# Set config for git
gosu ros bash -c 'git config --global --add safe.directory /home/ros/kyubic_ros'

# Build ROS packages
gosu ros bash -i -c 'cd ~/kyubic_ros/kyubic_ws &&
	colcon build --symlink-install --cmake-args -GNinja'

gosu ros bash -i -c 'cd ~/kyubic_ros/kyubic_ws && pre-commit install'

# Give execute permission
gosu ros bash -i -c 'chmod +x $HOME/kyubic_ros/docker/script/*'
gosu ros bash -i -c 'chmod +x $HOME/kyubic_ros/docker/byobu/bin'
