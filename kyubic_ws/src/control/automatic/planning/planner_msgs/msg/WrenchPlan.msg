# enum
uint8 Z_MODE_DEPTH = 0
uint8 Z_MODE_ALTITUDE = 1

# message
std_msgs/Header header
uint8 z_mode
localization_msgs/Pose target
localization_msgs/Odometry odom
