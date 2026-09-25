# Synchronization trigger sent by a coordinator to active participants.
uint64 sequence_id
# Scheduled ROS-clock instant shared by every participant (not callback arrival).
time trigger_time
# Actual emission time, retained separately to measure dispatch lateness.
time published_time
uint32[] active_participant_ids
