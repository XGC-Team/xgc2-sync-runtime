uint8 OBJECTIVE_MINCO = 1

uint8 CONSTRAINT_POINT = 0
uint8 CONSTRAINT_SPHERE = 1
uint8 CONSTRAINT_BOX = 2
uint8 CONSTRAINT_GATE = 3

std_msgs/Header header
uint32 request_id
uint32 trajectory_id
uint32 revision
uint32 flags
geometry_msgs/Pose[] waypoints
uint8[] constraint_types
geometry_msgs/Vector3[] region_size
float64[] segment_times
geometry_msgs/Vector3 start_velocity
geometry_msgs/Vector3 start_acceleration
geometry_msgs/Vector3 end_velocity
geometry_msgs/Vector3 end_acceleration
float64 desired_speed
float64 time_weight
float64 max_body_rate
float64 max_tilt
float64 min_thrust
float64 max_thrust
uint32 max_iterations
float64 rel_cost_tol
float64 max_velocity
float64 max_acceleration
float64 max_jerk
float64 max_snap
uint8 objective
