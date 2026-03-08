std_msgs/Header header

uint8 GREEN=0
uint8 YELLOW=1
uint8 ORANGE=2
uint8 RED=3

uint8 overall_level
string overall_reason

uint8 localization_level
string localization_reason
string localization_state

uint8 controller_level
string controller_mode
bool control_degraded
float64 control_degenerate_score
float64 compute_time_ms
float64 pose_age_ms
float64 collision_d_min

uint8 keepout_level
bool keepout_ready
string keepout_reason

uint8 terrain_level
bool terrain_online
bool terrain_obstacle_active
bool terrain_drop_active

uint8 nav_safety_level
string nav_profile
bool nav_stop_required
bool nav_timed_out

uint8 mechanism_level
uint8 mechanism_comm_health

uint32 topic_timeout_count
string[] active_event_codes
