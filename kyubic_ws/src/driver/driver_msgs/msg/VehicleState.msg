# message
std_msgs/Header header
common_msgs/Status status

# Connection and safety state reported by the vehicle transport.
bool connected
bool armed

# MAVLink vehicle identity and state.  These fields are transport-neutral for
# consumers; a platform without equivalent data publishes its documented
# default value and an appropriate status.
uint8 target_system
uint8 target_component
uint32 custom_mode
uint8 system_status
