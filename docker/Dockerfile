#######################################################################################################################################################################################
###  Install ROS2 on ubuntu ###
#
## build
# $ docker build -t my/ros:jazzy .
#
## docker build option: ubuntu_ver, ros2_ver
# $ docker build -t my/ros:dashing  --build-arg ubuntu_ver=18.04 --build-arg ros2_ver=dashing .
#
## run
# $ docker run -it --name ROS_Jazzy  -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 my/ros:jazzy /bin/bash
#
#######################################################################################################################################################################################


ARG ubuntu_ver=24.04
ARG base_image=ubuntu:$ubuntu_ver

FROM $base_image


ENV TURTLEBOT3_MODEL=waffle
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

#########################################
## Bad Proxy measures
## Set locale [UTF-8]
## Ubuntu Universe repository is enabled
#########################################
# RUN touch /etc/apt/apt.conf.d/99fixbadproxy && \
#     echo "Acquire::http::Pipeline-Depth 0; Acquire::http::No-Cache true; Acquire::BrokenProxy true;" >> /etc/apt/apt.conf.d/99fixbadproxy && \
RUN apt update && apt install -y locales && \
	locale-gen en_US en_US.UTF-8  && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	export LANG=en_US.UTF-8
# RUN locale  ## verify settings (check for UTF-8)


##################
## Install variety
##################
RUN apt update && apt install -y sudo gosu iproute2 iputils-ping curl gnupg2 lsb-release ninja-build \
	screen tmux vim git wget locate python3-pip python3-venv python3-flake8-docstrings doxygen pre-commit && \
	updatedb

####################################################
## Enable completion
## ref: https://penguin-coffeebreak.com/archives/242
####################################################
RUN apt install bash-completion && source /etc/bash_completion && \
	rm /etc/apt/apt.conf.d/docker-clean && apt update

##################
## Create new user
##################
RUN userdel -r ubuntu && useradd -ms /bin/bash ros
WORKDIR /home/ros
# RUN userdel ubuntu  # for ubuntu24.04
# RUN echo 'user:pass' | chpasswd


######################################
## Install vertual env for python 3.12
######################################
RUN gosu ros bash -l -c "curl -LsSf https://astral.sh/uv/install.sh | sh && \
	source /home/ros/.local/bin/env && \
	uv python install 3.12 && \
	echo source /home/ros/.local/bin/env >> /home/ros/.bashrc"


#########################################
## Download the ROS2 GPG key
## Add the repository to your sources list
#########################################
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
	apt update && apt -y upgrade


#######################################################################################
## Install ROS2(Humble etc..) and turtlesim and teleop-twist-keyboard and Tool for ROS2
#######################################################################################
ARG ros2_ver=jazzy
ENV ROS_DISTRO=$ros2_ver

RUN apt install -y ros-$ROS_DISTRO-desktop ros-$ROS_DISTRO-turtlesim ros-$ROS_DISTRO-teleop-twist-keyboard ros-dev-tools && \
	echo "ros installed" && \
	rosdep init && \
	rosdep update && \
	echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/ros/.bashrc && \
	echo "source /home/ros/kyubic_ros/kyubic_ws/install/setup.bash" >> /home/ros/.bashrc && \
	echo "source /home/ros/kyubic_ros/kyubic_ws/install/local_setup.bash" >> /home/ros/.bashrc && \
	apt update && apt clean && \
	rm -rf /var/lib/apt/lists/* && \
	echo "succesfully"

######################################
## Install zellij
######################################
RUN gosu ros bash -c "mkdir -p /home/ros/Apps/Zellij && cd /home/ros/Apps/Zellij && \
	curl -OL https://github.com/zellij-org/zellij/releases/latest/download/zellij-x86_64-unknown-linux-musl.tar.gz && \
	tar -xvf zellij*.tar.gz" && \
	chmod +x /home/ros/Apps/Zellij/zellij && \
	ln -s /home/ros/Apps/Zellij/zellij /usr/bin/zellij

COPY ./entrypoint.sh /usr/bin
COPY ./once_entrypoint.sh /usr/bin
RUN chmod +x /usr/bin/entrypoint.sh /usr/bin/once_entrypoint.sh
ENTRYPOINT [ "/bin/bash", "-c", "/usr/bin/entrypoint.sh" ]

