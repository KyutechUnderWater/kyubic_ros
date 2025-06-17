#!/bin/bash

chown ros:ros -R /home/ros

gosu ros bash -l -c 'cd ~/kyubic_ros && uv sync'

gosu ros bash -l -c 'cd ~/kyubic_ros/kyubic_ws && 
	colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1'

/bin/bash
