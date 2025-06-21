xhost +local:

# Get environment variable
declare -n kyubic_ros=KYUBIC_ROS$1
declare -n kyubic_ros_compose=KYUBIC_ROS_COMPOSE$1

# Start container and Run bash
cd "${kyubic_ros}/docker" || exit
docker compose $kyubic_ros_compose start
docker compose $kyubic_ros_compose exec kyubic-ros gosu ros bash
