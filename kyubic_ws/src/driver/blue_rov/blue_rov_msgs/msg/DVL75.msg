# Cerulean Sonar DVL-75 measurements.
std_msgs/Header header

# $DVEXT fields
bool dvext_valid
bool bottom_lock
uint8 data_skips
float64 velocity_up              # [m/s]
float64 altitude                 # [m], along the sensor pointing axis
float64 velocity_north           # [m/s]
float64 velocity_east            # [m/s]
bool[4] beam_lock
float64[4] beam_velocity         # [m/s], along each beam axis
float64[4] beam_range            # [m], along each beam axis

# $DVPDL fields
bool dvpdl_valid
uint64 device_time_us
uint32 delta_time_us
geometry_msgs/Vector3 angle_delta       # [rad], angle_axis_sign applied
geometry_msgs/Vector3 position_delta    # [m], position_axis_sign applied
uint8 confidence                         # 0..100
