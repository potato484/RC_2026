# SmartWaypoint.msg - 智能导航点数据契约
geometry_msgs/Pose pose
string strategy_tag
NavTolerance tolerance

# Navigation safety mode (0=NORMAL, 1=MF_SAFE, 2=MF_TRAVERSE, 3=MF_EXIT)
uint8 nav_safety_mode
string speed_profile
float32 timeout_sec

# Extension payload as key-value pairs
string[] payload_keys
float64[] payload_values
