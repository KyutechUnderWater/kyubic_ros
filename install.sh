#!/bin/bash

# Check arguments
if [ $# != 1 ]; then
	echo "Requires a passward to be set in the argument"
	return 1
fi

# Hard link for udev rules
sudo ln -i ./99-usb-kyubic.rules /etc/udev/rules.d/99-usb-kyubic.rules

# Set alias
path=$(pwd)
echo "alias ros2_start=$path/ros2_start.sh" >>~/.bash_aliases
sudo chmod +x ./ros2_start.sh

# Set Environment Variables (UID and GID)
echo "export USER_ID=$(id -u)" >>~/.bashrc
echo "export GROUP_ID=$(id -g)" >>~/.bashrc

# Reload bashrc
eval "$(cat ~/.bashrc | tail -n +10)"

#############################################################################
# Create and Start container
docker compose build $2
docker compose create
docker compose start

## Set passward
docker compose exec kyubic-ros sh -c "echo "ros:$1" | chpasswd"

## Install nvim
docker compose exec kyubic-ros ../nvim_setup.sh

## Stop Container
docker compose stop
#############################################################################
