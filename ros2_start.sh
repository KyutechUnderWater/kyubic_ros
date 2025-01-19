xhost +local:

cd $KYUBIC_ROS && docker compose start && docker compose exec kyubic-ros gosu ros bash
