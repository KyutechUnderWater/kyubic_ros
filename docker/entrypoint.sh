#!/bin/bash

chown ros:ros -R /home/ros

# Pi 5 の BCM GPIO (/dev/gpiochip4) は dialout グループ所有。
# compose の group_add は root プロセスにしか効かないため、ros へ毎回付与する。
if getent group dialout >/dev/null; then
  usermod -aG dialout ros 2>/dev/null || true
fi

gosu ros bash -l -c 'cd ~/kyubic_ros && uv sync'
gosu ros bash -l -c '~/kyubic_ros/docker/sync_ros_python_deps.sh'

/bin/bash