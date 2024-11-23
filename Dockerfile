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
RUN apt update && apt install locales && \
	locale-gen en_US en_US.UTF-8  && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	export LANG=en_US.UTF-8
# RUN locale  ## verify settings (check for UTF-8)


##################
## Install variety
##################
RUN apt update && apt install -y curl gnupg2 lsb-release tmux vim git wget locate python3-pip python3-flake8-docstrings doxygen && \
	updatedb


######################################
## Install vertual env for python 3.12
######################################
ENV PYENV_ROOT=/root/.pyenv
ENV PATH=/root/.pyenv/shims:$PYENV_ROOT/bin:$PATH

RUN apt update && \
    apt install -y git curl build-essential libffi-dev libssl-dev zlib1g-dev liblzma-dev libbz2-dev \
    libreadline-dev libsqlite3-dev libncursesw5-dev libxml2-dev libxmlsec1-dev tk-dev xz-utils && \
    git clone https://github.com/pyenv/pyenv.git ~/.pyenv && \
    cd ~/.pyenv && src/configure && make -C src && \
    echo 'eval "$(pyenv init -)"' >> ~/.bashrc && \
    eval "$(pyenv init -)" && \
    pyenv install 3.12 && pyenv global system


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
	python3 -m pip install -U argcomplete colcon-common-extensions vcstools && \
	echo "ros installed" && \
	rosdep init && \
	rosdep update && \
	echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
	echo "source /home/kyubic_ros/kyubic_ws/install/setup.bash" >> ~/.bashrc && \
	echo "source /home/kyubic_ros/kyubic_ws/install/local_setup.bash" >> ~/.bashrc && \
	apt update && apt clean && \
	rm -rf /var/lib/apt/lists/* && \
	echo "succesfully"

WORKDIR /home/ros2_ws

COPY ./entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/bin/bash", "-c", "chmod 555 /entrypoint.sh && $_ && bash"]



#apt install ros-melodic-joint-state-publisher-gui
#vcs import src < turtlebot3.repos
#git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git && \
#git clone -b foxy https://github.com/ros2/examples && \


#docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 --gpus all nvidia/opengl:1.2-glvnd-runtime-ubuntu20.04 /bin/bash

#RUN apt install -y python3 python3-pip vim
#RUN pip3 install torch torchvision jupyterlab matplotlib
#WORKDIR /work

#COPY train.py /work/

#ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs