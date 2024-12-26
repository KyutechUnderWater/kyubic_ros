#!/bin/bash

# Delete container
docker compose down


# Delete Enviroment Variable
rm .env


# Delete alias
sed -i '/alias ros2_start=/d' ~/.bash_aliases
eval "$(cat ~/.bashrc | tail -n +10)"


# Delete symbolic link for udev rules
sudo unlink /etc/udev/rules.d/99-usb-kyubic.rules
