# kyubic_ros

## 🔔 Features

- ROS 2 Jazzy
- Container independent environment
- Automatic environment building
- CUDA(NVIDIA GPU) support

<br>

## 📢 Requirement

- Docker >= 28.2.2
- Docker Compose >= 2.36.2

If using a CUDA

- NVIDIA GPU (3080, etc.)
- NVIDIA Container Toolkit >=1.17.8
- NVIDIA Driver >= 570 (CUDA Version >= 12.8) 👉 Make sure with the nvidia-smi command.

<br>

## ✏️ Installation

Create docker container and Setup continer (user password, enviroment variable, alias, and udev rule)

### Step 1: Clone Repository and Update Submodule
```bash
git clone git@github.com:KyutechUnderWater/kyubic_ros.git
cd kyubic_ros && git submodule update --init --recursive
```

### Step 2: Install kyubic_ros
#### For Robot Computers 🤖
```bash
. install.sh
```

#### For Client 💻
add `-c` option
```bash
. install.sh -c
```

(Optional) If you want to use CUDA

```bash
. install.sh --nvidia
```

> [!NOTE]
> If you want to create multiple environments
> See [#Tips](https://github.com/KyutechUnderWater/kyubic_ros/main/README.md#tips)

<br>

## 🎓 How to Use

Start container and Run bash

```bash
ros2_start
```

If `--nvidia` option is enabled, `_nvidia` is added at the end.

```bash
ros2_start_nvidia
```

Exit container

```bash
exit # or <ctrl + d>
```

<br>

## 💧 Uninstall

```bash
. uninstall.sh
```

<br>

## ⚡ Tips

### Using multiple environment

💡 Use the -p (--project-name) option to set a unique project name

> [!NOTE]
> If you set the project name with the -p option, the startup command (ros2_start) will be suffixed with the project name.

Example of creating 4 environments (2 normal and 2 CUDA environments)  
First environment:

```bash
git clone git@github.com:KyutechUnderWater/kyubic_ros.git kyubic_ros_first
cd $_ && git submodule update --init --recursive
. install.sh && \
ros2_start  # Start container and Run bash
```

Second environment:

```bash
git clone git@github.com:KyutechUnderWater/kyubic_ros.git kyubic_ros_second
cd $_ && git submodule update --init --recursive
. install.sh -p second && \  # Set project name
ros2_start_second  # Start container and Run bash
```

First CUDA environment:

```bash
git clone git@github.com:KyutechUnderWater/kyubic_ros.git kyubic_ros_first_cuda
cd $_ && git submodule update --init --recursive
. install.sh --nvidia && \
ros2_start_nvidia  # Start container and Run bash
```

Second CUDA environment:

```bash
git clone git@github.com:KyutechUnderWater/kyubic_ros.git kyubic_ros_second_cuda
cd $_ && git submodule update --init --recursive
. install.sh --nvidia -p second && \  # Set project name
ros2_start_second_nvidia  # Start container and Run bash
```
