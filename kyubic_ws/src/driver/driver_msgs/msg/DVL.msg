# message
std_msgs/Header header
common_msgs/Status status

uint8 system_config
bool leak_a
bool leak_b
bool[4] btm_status_echo_amplitude
bool[4] btm_status_correlation
geometry_msgs/Vector3 velocity  # [m/s]
bool velocity_valid
int16 velocity_error
float64 altitude                # [m]
