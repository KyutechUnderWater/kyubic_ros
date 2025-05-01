#!/bin/bash

# Check arguments
if [ $# != 1 ]; then
	echo "Requires a passward to be set in the argument"
	return 1
fi

# Hard link for udev rules
if [ -f ./99-usb-kyubic.rules ] && [ ! -L /etc/udev/rules.d/99-usb-kyubic.rules ]; then
	sudo ln -i ./99-usb-kyubic.rules /etc/udev/rules.d/99-usb-kyubic.rules
fi

# Set alias
path=$(pwd)
echo "alias ros2_start=$path/ros2_start.sh" >>~/.bash_aliases
sudo chmod +x ./ros2_start.sh

# Set Environment Variables (UID and GID)
echo "export USER_ID=$(id -u)" >>~/.bashrc
echo "export GROUP_ID=$(id -g)" >>~/.bashrc
echo "export KYUBIC_ROS=$path" >>~/.bashrc

# Reload bashrc
eval "$(cat ~/.bashrc | tail -n +10)"

#############################################################################
# Create and Start container
docker compose build $2
docker compose create
docker compose start

## Set passward
docker compose exec kyubic-ros sh -c "echo "ros:$1" | chpasswd"

## Set ROS_DOMAIN_ID
docker compose exec kyubic-ros gosu ros bash -l -c "echo 'export ROS_DOMAIN_ID=1' >> ~/.bashrc"

## Install nvim
docker compose exec kyubic-ros gosu ros bash -c "../nvim_setup.sh"

## Stop Container
docker compose stop
#############################################################################
