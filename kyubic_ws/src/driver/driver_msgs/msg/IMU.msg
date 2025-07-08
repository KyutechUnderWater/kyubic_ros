# enum
uint8 STATUS_NORMAL=1  # Successfull data acquisition
uint8 STATUS_ERROR=2   # Failed to acquire data


# message
std_msgs/Header header
uint8 status
float64 temperature           # [degree celsius]
geometry_msgs/Vector3 accel   # [m/s^2]
geometry_msgs/Vector3 gyro    # [rad/s]
geometry_msgs/Vector3 orient  # [rad]
geometry_msgs/Quaternion qtn  # [-]
