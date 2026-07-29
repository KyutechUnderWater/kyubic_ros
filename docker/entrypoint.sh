#!/bin/bash

chown ros:ros -R /home/ros

gosu ros bash -l -c 'cd ~/kyubic_ros && uv sync'
gosu ros bash -l -c '~/kyubic_ros/docker/sync_ros_python_deps.sh'

/bin/bash