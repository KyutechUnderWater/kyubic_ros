#!/bin/bash

# Move directory
cd ./docker || exit

# Delete container
docker compose down

# Delete Enviroment Variable
sed -i '/export USER_ID=/d' ~/.bashrc
sed -i '/export GROUP_ID=/d' ~/.bashrc
sed -i '/export KYUBIC_ROS_HUMBLE=/d' ~/.bashrc
export -n USER_ID
export -n GROUP_ID
export -n KYUBIC_ROS_HUMBLE

# Delete alias
sed -i '/alias ros2_start_humble=/d' ~/.bash_aliases
eval "$(cat ~/.bashrc | tail -n +10)"

# Delete hardlink for udev rules
if [ -f /etc/udev/rules.d/99-usb-kyubic.rules ]; then
	sudo unlink /etc/udev/rules.d/99-usb-kyubic.rules
fi

cd ../
