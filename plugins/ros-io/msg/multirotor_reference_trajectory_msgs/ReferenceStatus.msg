uint8 STATE_SELF_CHECK = 1
uint8 STATE_READY = 2
uint8 STATE_PLANNING = 3
uint8 STATE_ACTIVE = 4
uint8 STATE_FAULT = 9

uint8 TYPE_NONE = 0
uint8 TYPE_ANALYTIC = 1
uint8 TYPE_POLYNOMIAL = 2
uint8 TYPE_SAMPLED = 3

std_msgs/Header header
uint8 state
uint32 flags
uint32 active_trajectory_id
uint32 active_revision
uint8 active_type
