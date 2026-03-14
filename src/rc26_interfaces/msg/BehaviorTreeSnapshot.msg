std_msgs/Header header
uint64 tick_seq
uint8 tree_status
float32 tick_duration_ms
string active_subtree_id
uint16[] running_path_uids
BehaviorTreeNodeState[] nodes

uint8 IDLE=0
uint8 RUNNING=1
uint8 SUCCESS=2
uint8 FAILURE=3
uint8 SKIPPED=4
