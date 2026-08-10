# enum
uint8 Z_MODE_DEPTH = 0
uint8 Z_MODE_ALTITUDE = 1

# velocity_axis_mask bits: when set for an axis, that axis's inner velocity loop is driven
# directly from `twist` (bypassing the outer position P loop, which normally derives its
# target velocity from `pose`). Useful for tuning/verifying the velocity loop in isolation.
uint8 VELOCITY_AXIS_X = 1
uint8 VELOCITY_AXIS_Y = 2
uint8 VELOCITY_AXIS_Z = 4
uint8 VELOCITY_AXIS_ROLL = 8
uint8 VELOCITY_AXIS_YAW = 16

# reset bits: when set for an axis, that axis's PID internal state (integral term,
# incremental-form previous output/error, etc.) is cleared before this target is
# applied. Without this, switching an axis between position mode and velocity mode
# (or just sending a very different target) leaves stale internal state (e.g. the
# velocity-form PID's accumulated output) that keeps influencing the output until
# enough new error accumulates to override it.
uint8 RESET_X = 1
uint8 RESET_Y = 2
uint8 RESET_Z = 4
uint8 RESET_ROLL = 8
uint8 RESET_YAW = 16

# message
std_msgs/Header header
uint8 z_mode
uint8 velocity_axis_mask
uint8 reset
p_pid_controller_msgs/BaseAxes twist
p_pid_controller_msgs/BaseAxes pose
