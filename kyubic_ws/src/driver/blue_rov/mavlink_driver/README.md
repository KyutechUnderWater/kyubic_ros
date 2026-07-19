# mavlink_driver

`mavlink_driver` is the sole MAVLink client for the BlueROV ROS deployment.
Do not run `KyutechUnderWaterBlue/blue.py` against the same vehicle at the same
time: both programs send RC override messages.

The driver subscribes to `robot_force`, `led`, `camera_tilt`, and `heartbeat`;
it publishes `depth`, `imu`, `power_state`, `gnss`, and `vehicle_state` below
`/driver/blue_rov/mavlink_driver`.

`set_armed` is a `std_srvs/SetBool` service. `true` requests ARM and `false`
requests DISARM. A request succeeds only after a MAVLink command ACK and a
vehicle heartbeat report the requested state. The driver never arms the
vehicle automatically.

Install `pymavlink` in the Python environment used by ROS before launching.
Start with the conservative PWM range in the supplied configuration, secure
the vehicle, and calibrate each axis sign and range before water operations.
