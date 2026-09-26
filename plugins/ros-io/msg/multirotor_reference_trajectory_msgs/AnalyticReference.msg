uint16 ANALYTIC_HOLD = 0
uint16 ANALYTIC_CIRCLE = 1
uint16 ANALYTIC_HEIGHT_CIRCLE = 2
uint16 ANALYTIC_CIRCLE_ENTRY = 3
uint16 ANALYTIC_FIGURE_EIGHT = 4
uint16 ANALYTIC_LINE = 5
uint16 ANALYTIC_LEMNISCATE = 6
uint16 ANALYTIC_HELIX_YZ = 7
uint16 ANALYTIC_HELIX_XY = 8
uint16 ANALYTIC_TORUS_KNOT = 9

std_msgs/Header header
uint32 request_id
uint32 trajectory_id
uint32 revision
uint16 analytic_type
uint32 flags
time start_time
float64 duration
geometry_msgs/Pose origin
# Existing circle params:
#   [radius, line_speed, height, z_amplitude, z_frequency, entry_duration, center_x, center_y]
# Additional analytic curve params:
#   LINE: [target_x, target_y, target_z, target_vx, target_vy, target_vz,
#          start_vx, start_vy, start_vz]
#   LEMNISCATE: [radius, omega, height]
#   HELIX_YZ / HELIX_XY: [radius, omega, linear_scale]
#   TORUS_KNOT: [omega, scale] or
#               [omega, scale, entry_duration, curve_origin_x, curve_origin_y, curve_origin_z]
float64[] params
