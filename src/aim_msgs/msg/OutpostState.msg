std_msgs/Header header

uint8 id
bool tracking
bool converged
bool jumped

geometry_msgs/Point center
geometry_msgs/Vector3 velocity

float64 yaw
float64 angular_velocity
float64 radius
float64 low_height_offset
float64 high_height_offset

bool has_primary_armor
int8 primary_slot
geometry_msgs/Pose primary_armor
geometry_msgs/Pose[] predicted_armors
