#!/bin/bash

# Delete container
docker compose down

# Delete Enviroment Variable
sed -i '/export USER_ID=/d' ~/.bashrc
sed -i '/export GROUP_ID=/d' ~/.bashrc
sed -i '/export KYUBIC_ROS=/d' ~/.bashrc
export -n USER_ID
export -n GROUP_ID
export -n KYUBIC_ROS

# Delete alias
sed -i '/alias ros2_start=/d' ~/.bash_aliases
eval "$(cat ~/.bashrc | tail -n +10)"

# Delete hardlink for udev rules
if [ -f /etc/udev/rules.d/99-usb-kyubic.rules ]; then
	sudo unlink /etc/udev/rules.d/99-usb-kyubic.rules
fi
