std_msgs/Header header

string camera_name
# True only when the driver is actually using a valid device-to-ROS mapping.
bool enabled
bool mapping_valid
uint64 device_tick
uint64 latch_tick
float64 tick_frequency_hz
bool tick_frequency_estimated
bool tick_frequency_override
float64 mapped_stamp_sec
float64 host_receive_stamp_sec
float64 latch_correction_sec
float64 fixed_offset_sec
# Zero means the current mapping path has no recorded error.
int32 mapping_error_code
string mapping_error_stage
