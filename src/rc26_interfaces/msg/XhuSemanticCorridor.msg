# Corridor contract from rc26_topo_nav to xhu_motion_follower
std_msgs/Header header
string corridor_id
string edge_id
string from_node_id
string to_node_id
string motion_type
string required_mode
nav_msgs/Path path
float32 max_linear_speed
float32 max_angular_speed
bool stop_at_end
bool allow_reverse
