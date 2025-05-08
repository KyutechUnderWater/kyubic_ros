xhost +local:

cd "${KYUBIC_ROS}/docker" && docker compose start && docker compose exec kyubic-ros gosu ros bash
