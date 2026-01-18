#!/bin/bash

xhost +local:

# --- Argument Parsing Logic ---
ENV=""

# Case 1: First argument is "--" (e.g., ros2_start -- byobu)
if [ "$1" == "--" ]; then
    shift # Remove "--"

# Case 2: Second argument is "--" (e.g., ros2_start test -- byobu)
elif [ "$2" == "--" ]; then
    ENV="$1"
    shift 2 # Remove both ENV ID and "--"

# Case 3: No "--" present (e.g., ros2_start or ros2_start test)
else
    ENV="$1"
    # If an argument exists, consume it so it isn't treated as a command later
    if [ -n "$1" ]; then shift; fi
fi
# ------------------------------

# Load environment variables
declare -n kyubic_ros=KYUBIC_ROS${ENV}
declare -n kyubic_ros_compose=KYUBIC_ROS_COMPOSE${ENV}

# Validate configuration
if [ -z "$kyubic_ros" ]; then
    echo "Error: Variable KYUBIC_ROS${ENV} is not set."
    exit 1
fi

# Start container
cd "${kyubic_ros}/docker" || exit
docker compose $kyubic_ros_compose start

# Execute command inside the container
if [ $# -gt 0 ]; then
  # Execute provided command (e.g., byobu)
  docker compose $kyubic_ros_compose exec kyubic-ros gosu ros "$@"
else
  # Default to bash if no command provided
  docker compose $kyubic_ros_compose exec kyubic-ros gosu ros bash
fi
