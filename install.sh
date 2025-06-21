#!/bin/bash -i

read -r -d '' _help_option <<EOF
Usage:  install.sh [OPTIONS]

Options:
	-h, --help                   show command help
	-p, --project-name string    Name of patckage to install
	    --nvidia                 if nvidia drivers are available, they are used\n
EOF

# Argument Parser
ArgumentParser() {
	ARGS=$1
	shift

	local project_name=""
	local nvidia=false

	while (($# > 0)); do

		case "$1" in

		-h | --help)
			printf "$_help_option"
			return 10
			;;

		-p | --project-name)
			project_name=$2
			if ! [[ $project_name =~ ^[a-zA-Z0-9_]+$ ]]; then
				printf '\033[33m[ERROR] Only a-z, A-Z, and _ can be used for project name\n' && return 128
			fi
			shift
			shift
			;;

		--nvidia)
			nvidia=true
			shift
			;;

		*)
			printf "\033[33m[ERROR] Usage: install.sh [-h] [-p] [--no-nvidia]\n"
			return 1
			;;
		esac
		[[ $# == 0 ]] && break
	done

	eval "$ARGS=(
		["project_name"]=\"${project_name}\"
		["nvidia"]=$nvidia
	)"
}

#####################################################
# Main Script
#####################################################
declare -A args
if ! ArgumentParser args $@; then
	return 1
fi

project_name=${args["project_name"]}
nvidia=${args["nvidia"]}

path=$(pwd)
sudo -v

# Check if the environment has been created
if [ -f .install.project_name ]; then
	printf '\033[33m[ERROR] This repository has already created an environment\nPlease run `. uninstall.sh`\n' && return 1
fi

# Hard link for udev rules
if [ -f ./99-usb-kyubic.rules ] && [ ! -L /etc/udev/rules.d/99-usb-kyubic.rules ]; then
	sudo ln -i ./99-usb-kyubic.rules /etc/udev/rules.d/99-usb-kyubic.rules
fi

# Check nvidia-docker
compose_path='-f compose.yaml'
if [[ $nvidia == true ]] && nvidia-smi 2>/dev/null; then
	compose_path="${compose_path} -f compose-nvidia.yaml"
	if [[ $project_name == "" ]]; then
		project_name="nvidia"
	else
		project_name="${project_name}_nvidia"
	fi
fi

# project_name for unique command
if [[ $project_name == "" ]]; then
	if alias ros2_start >/dev/null 2>&1; then
		printf '\033[33m[ERROR] Already have one or more environments\nRequired to set the project name with the -p option \n' && return 1
	fi
	compose_env_var="$compose_path"
else
	if alias "ros2_start_$project_name" >/dev/null 2>&1; then
		printf '\033[33m[ERROR] The project name "%s" exists\n' "$project_name" && return 1
	fi
	compose_env_var="-p $project_name $compose_path"
	project_name="_$project_name"
fi
echo $project_name >.install.project_name

# Set alias
echo "alias ros2_start${project_name}='$path/ros2_start.sh $project_name'" >>~/.bash_aliases
sudo chmod +x ./ros2_start.sh

# Set Environment Variables (UID and GID)
echo "export USER_ID=$(id -u)" >>~/.bashrc
echo "export GROUP_ID=$(id -g)" >>~/.bashrc
echo "export KYUBIC_ROS${project_name}=$path" >>~/.bashrc
echo "export KYUBIC_ROS_COMPOSE${project_name}='$compose_env_var'" >>~/.bashrc

# Reload bashrc
eval "$(cat ~/.bashrc | tail -n +10)"

#############################################################################
# Enter passward
read -sp "Enter the passward used for the container user : " passwd
echo "" # newine

# Move directory
cd ./docker || exit

# Show compose config
docker compose $compose_env_var config

# Create and Start container
docker compose $compose_env_var build >>${path}/.install.log
docker compose $compose_env_var create
docker compose $compose_env_var start

## Run setup
echo '⚡Initialize and Build'
docker compose $compose_env_var exec kyubic-ros bash -c "/usr/bin/once_entrypoint.sh $passwd" >>${path}/.install.log 2>&1

## Install nvim
docker compose $compose_env_var exec kyubic-ros gosu ros bash -c "../docker/nvim_setup.sh $passwd" >>${path}/.install.log 2>&1

## Stop Container
docker compose $compose_env_var stop
#############################################################################

cd $path || exit
