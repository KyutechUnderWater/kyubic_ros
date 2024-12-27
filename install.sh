#!/bin/bash


# Hard link for udev rules
sudo ln -i ./99-usb-kyubic.rules /etc/udev/rules.d/99-usb-kyubic.rules


# Set alias
path=$(pwd)
echo "alias ros2_start=$path/ros2_start.sh" >>~/.bash_aliases
sudo chmod +x ./ros2_start.sh


# Set Environment Variables (UID and GID)
echo "USER_ID=$(id -u)" > ./.env
echo "GROUP_ID=$(id -g)" > ./.env


# Reload bashrc
eval "$(cat ~/.bashrc | tail -n +10)"


# Create container
docker compose build $2
docker compose create


# Set passward
docker compose start
docker compose exec kyubic-ros sh -c "echo "ros:$1" | chpasswd"
docker compose stop
