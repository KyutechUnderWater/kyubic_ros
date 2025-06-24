# enum
uint8 STATUS_NORMAL = 0
uint8 STATUS_WARNING = 1
uint8 STATUS_ERROR = 2


# message
std_msgs/Header header
uint8 status
localization_msgs/Pose pose
localization_msgs/Twist twist
