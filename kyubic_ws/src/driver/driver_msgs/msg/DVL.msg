# enum
uint8 STATUS_NORMAL=1   # Successfull data acquisition
uint8 STATUS_WARNING=2  # Detected error-velocity
uint8 STATUS_ERROR=3    # Failed to acquire data


# message
std_msgs/Header header
uint8 status
geometry_msgs/Vector3 velocity  # [m/s]
int16 velocity_error
float64 altitude                # [m]
