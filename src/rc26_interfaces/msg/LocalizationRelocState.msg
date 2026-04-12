# Relocalization progress and state
std_msgs/Header header
uint8 STATE_IDLE=0
uint8 STATE_SEARCHING=1
uint8 STATE_CONVERGING=2
uint8 STATE_CONVERGED=3
uint8 STATE_FAILED=4
uint8 state
float64 progress          # 0.0 ~ 1.0
float64 best_score
string description
