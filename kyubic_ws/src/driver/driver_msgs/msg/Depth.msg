# enum
uint8 STATUS_NORMAL=1  # Successfull data acquisition
uint8 STATUS_ERROR=2   # Failed to acquire data


# message
std_msgs/Header header
uint8 status
float64 depth # [m]
