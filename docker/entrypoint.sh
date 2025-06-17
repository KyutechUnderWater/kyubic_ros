#!/bin/bash

chown ros:ros -R /home/ros

gosu ros bash -l -c 'cd ~/kyubic_ros && uv sync'

/bin/bash
