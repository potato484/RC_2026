# Corridor contract from rc26_xhu_nav topology module to xhu_motion_runtime
std_msgs/Header header
string corridor_id
string edge_id
string from_node_id
string to_node_id
string motion_type
string required_mode
nav_msgs/Path path
float32 preferred_linear_speed
float32 max_linear_speed
float32 max_angular_speed
bool stop_at_end
bool allow_reverse
bool allow_in_place_rotate
string speed_limit_reason
string[] active_risk_sources
