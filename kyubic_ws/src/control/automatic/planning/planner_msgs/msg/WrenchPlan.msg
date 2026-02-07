# enum
uint8 Z_MODE_DEPTH = 0
uint8 Z_MODE_ALTITUDE = 1

# message
std_msgs/Header header
common_msgs/Priority priority
uint8 z_mode
uint8 reset  # specified in bits 0b(null, null, null, x, y, z, roll, yaw)
bool has_master
bool has_slave
p_pid_controller_msgs/BaseAxes targets
p_pid_controller_msgs/BaseAxes master
p_pid_controller_msgs/BaseAxes slave
