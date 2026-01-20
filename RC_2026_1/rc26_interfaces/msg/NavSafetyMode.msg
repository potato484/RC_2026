# Navigation safety mode state
uint8 NORMAL=0
uint8 MF_SAFE=1
uint8 MF_TRAVERSE=2
uint8 MF_EXIT=3

uint8 mode
builtin_interfaces/Time stamp
string reason
bool stop_required
bool timed_out
