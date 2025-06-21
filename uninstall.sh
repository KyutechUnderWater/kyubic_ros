#!/bin/bash

# Get project name
if ! [ -f .install.project_name ]; then
	printf '\033[33m[ERROR] Not found environment\nPlease run `. install.sh`\n' && return 1
fi
project_name=$(cat .install.project_name)
declare -n kyubic_ros_compose=KYUBIC_ROS_COMPOSE${project_name}

# Move directory
cd ./docker || exit

# Delete container
docker compose $kyubic_ros_compose down

# Delete Enviroment Variable
sed -i '/export USER_ID=/d' ~/.bashrc
sed -i '/export GROUP_ID=/d' ~/.bashrc
sed -i "/export KYUBIC_ROS${project_name}=/d" ~/.bashrc
sed -i "/export KYUBIC_ROS_COMPOSE${project_name}=/d" ~/.bashrc
export -n USER_ID
export -n GROUP_ID
export -n KYUBIC_ROS${project_name}
export -n KYUBIC_ROS_COMPOSE${project_name}
unset KYUBIC_ROS${project_name}
unset KYUBIC_ROS_COMPOSE${project_name}

# Delete alias
sed -i "/alias ros2_start${project_name}=/d" ~/.bash_aliases
unalias "ros2_start${project_name}"
eval "$(cat ~/.bashrc | tail -n +10)"

# Delete hardlink for udev rules
if [ -f /etc/udev/rules.d/99-usb-kyubic.rules ]; then
	sudo unlink /etc/udev/rules.d/99-usb-kyubic.rules
fi

# Delete install log
cd ../
rm .install.log .install.project_name
