# enum
uint8 Z_MODE_DEPTH = 0
uint8 Z_MODE_ALTITUDE = 1

# message
std_msgs/Header header
uint8 z_mode
p_pid_controller_msgs/BaseAxes twist
p_pid_controller_msgs/BaseAxes pose
