# Feedback for PDLA action
uint8 RUNNING=0
uint8 FINE_TUNING=1
uint8 REACHED=2
uint8 WAITING=3

std_msgs/Header header
string csv_file_path
uint32 step_idx
uint8 step_state
localization_msgs/Odometry odom
