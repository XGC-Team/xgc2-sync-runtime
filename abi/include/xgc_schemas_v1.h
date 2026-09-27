/*
 * xgc_schemas_v1.h: first-party port payload schemas, version 1.
 *
 * Every payload is one of these structs, little-endian, copied byte for
 * byte (no padding surprises: every field is naturally aligned and sizes
 * are asserted). `stamp` is the source measurement time in Session seconds,
 * the same role as a ROS header stamp. The envelope's t_produce/t_tx are
 * transport stamps and are separate.
 *
 * The schema id string in a port declaration names the struct and version,
 * e.g. "xgc.imu/1". Changing a struct means a new version number.
 */
#ifndef XGC_SCHEMAS_V1_H
#define XGC_SCHEMAS_V1_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* "xgc.imu/1": body-frame specific force and angular rate. */
typedef struct xgc_imu_v1 {
  double stamp;
  double accel[3];  /* m/s^2, includes gravity reaction (+g on z at rest) */
  double gyro[3];   /* rad/s */
} xgc_imu_v1;

/* "xgc.attitude_target/1": commanded attitude and normalized thrust. */
typedef struct xgc_attitude_target_v1 {
  double stamp;
  double q_wxyz[4];
  double thrust;          /* normalized [0, 1] */
  uint32_t ignore_thrust; /* nonzero when the thrust field is not commanded */
  uint32_t reserved;
} xgc_attitude_target_v1;

/* "xgc.pose/1": position and orientation in the Session world frame. */
typedef struct xgc_pose_v1 {
  double stamp;
  double position[3];
  double q_wxyz[4];
} xgc_pose_v1;

/* "xgc.hover_thrust/1": hover-thrust estimator output. */
typedef struct xgc_hover_thrust_v1 {
  double stamp;                 /* publish event time (ROS header.stamp equivalent) */
  double hover_thrust;
  double raw_hover_thrust;
  double initial_hover_thrust;
  double thrust_to_acceleration;
  double last_estimate_stamp;
  uint32_t state;               /* domain state id (10 SelfCheck, 11 Ground, 12 Airborne) */
  uint32_t flags;               /* HoverThrustRuntimeFlag bits */
  uint32_t sample_used;
  uint32_t reserved;
} xgc_hover_thrust_v1;

/* "xgc.rigid_state/1": estimated rigid-body state, Session world frame. */
typedef struct xgc_rigid_state_v1 {
  double stamp;
  double position[3];
  double velocity[3];
  double q_wxyz[4];
  double body_rate[3];
} xgc_rigid_state_v1;

/* "xgc.flat_ref/1": differentially flat reference (position derivatives + yaw). */
typedef struct xgc_flat_ref_v1 {
  double stamp;
  double position[3];
  double velocity[3];
  double acceleration[3];
  double jerk[3];
  double snap[3];
  double yaw;
  double yaw_rate;
  double yaw_accel;
  uint32_t flags;
  uint32_t reserved;
} xgc_flat_ref_v1;

/* "xgc.attitude_rate_cmd/1": geometric controller output (before any
 * vehicle-specific thrust mapping). */
typedef struct xgc_attitude_rate_cmd_v1 {
  double stamp;                 /* stamp of the state it was computed from */
  double specific_thrust;       /* m/s^2 along body z */
  double q_wxyz[4];             /* desired attitude */
  double body_rate[3];          /* rad/s */
  double position_error[3];
  uint32_t success;
  uint32_t flags;
} xgc_attitude_rate_cmd_v1;

/* "xgc.rigid_state_estimate/1": the full rigid_state_estimator_msgs/
 * RigidStateEstimate (field for field; constants as in that message). */
typedef struct xgc_rigid_state_estimate_v1 {
  double stamp;
  double position[3];
  double velocity[3];
  double q_wxyz[4];
  double angular_velocity[3];
  double linear_acceleration[3];
  double gravity[3];
  double accel_bias[3];
  double last_fused_pose_stamp_sec;
  double vrpn_innovation_window_chi_square;
  double last_pose_position_innovation_norm_m;
  double last_pose_orientation_innovation_norm_rad;
  double last_pose_mahalanobis_distance;
  double innovation_position_gate_m;
  double innovation_orientation_gate_rad;
  double pose_nis_gate;
  double last_imu_sample_stamp_sec;
  double last_vrpn_pose_stamp_sec;
  double filter_inertial_stamp_sec;
  double filter_pose_stamp_sec;
  uint32_t flags;
  uint32_t vrpn_consecutive_rejects;
  uint32_t vrpn_consecutive_accepts;
  uint8_t estimator_state;
  uint8_t vrpn_observation_state;
  uint8_t filter_health;
  uint8_t last_pose_reject_reason;
  uint8_t last_pose_accepted;
  uint8_t reserved[7];
} xgc_rigid_state_estimate_v1;

/* "xgc.dmpc.assumed_trajectory/1": formation_generator/AssumedTrajectory
 * (TRO DMPC). This header is followed by num_states * num_timesteps doubles
 * (`states`, in the message's column-major order: states[i*num_states + j]
 * is state j at timestep i) and then rest_len doubles (`rest_position`). */
typedef struct xgc_dmpc_assumed_trajectory_v1 {
  double stamp;           /* header.stamp */
  uint32_t uav_id;
  uint32_t num_states;
  uint32_t num_timesteps;
  uint32_t rest_len;
  uint32_t valid;
  uint32_t reserved;
} xgc_dmpc_assumed_trajectory_v1;

/* "xgc.dmpc.sync_trigger/1": periodic_sync/SyncTrigger, from local rounds.
 * This header is followed by `count` uint32 active_participant_ids. */
typedef struct xgc_dmpc_sync_trigger_v1 {
  uint64_t sequence_id;   /* the round k */
  double trigger_time;    /* scheduled round start, Session seconds */
  double published_time;  /* when it was written, Session seconds */
  uint32_t count;
  uint32_t reserved;
} xgc_dmpc_sync_trigger_v1;

/* "xgc.dmpc.mission_state/1": a robot's controller state and its locally
 * derived mission phase, sent to its peers every round (data, never a tick). */
typedef struct xgc_dmpc_mission_state_v1 {
  double stamp;           /* sender's Session time when sent */
  uint64_t round;         /* sender's round k */
  uint32_t uav_id;
  uint32_t rolling;       /* the sender's phase at round k */
  double mission_time;
  char state[48];         /* controller CONTROL state, NUL-terminated (e.g. "Custom1") */
} xgc_dmpc_mission_state_v1;

/* "xgc.dmpc.formation_tick/1": formation_generator/FormationTick from local
 * rounds: this head, then an xgc.dmpc.sync_trigger/1 payload. */
typedef struct xgc_dmpc_formation_tick_v1 {
  double mission_time;
  uint32_t rolling;
  uint32_t reserved;
} xgc_dmpc_formation_tick_v1;

/* The shared scene (xgc2_geometry_msgs/SceneSnapshot and SceneState, as the
 * harness scene runtime publishes them), field for field except
 * ScenePart.color. Variable length, little-endian: a head below, then
 * records. A string is a uint32 byte count and its UTF-8 bytes (no NUL); a
 * pose is 7 doubles (position x y z, orientation x y z w); a twist is 6
 * doubles (linear, angular). A payload has no trailing bytes.
 *
 * "xgc.scene.snapshot/1": the scene definition (epoch, revision, obstacles).
 * This head, then frame_id, scene_id, epoch, then obstacle_count obstacles:
 *   id, name, pose, uint32 dynamic, motion_type, uint32 part_count, then
 *   part_count parts: id, pose, geometry type ("box" | "sphere" | "cylinder" |
 *   "capsule" | "convex"), size (3 doubles), double radius, double height,
 *   uint32 vertex_count, vertex_count vertices (3 doubles each), uint32
 *   index_count, index_count uint32 triangle indices. */
typedef struct xgc_scene_snapshot_v1 {
  double stamp;           /* header.stamp */
  uint64_t revision;
  uint32_t obstacle_count;
  uint32_t reserved;
} xgc_scene_snapshot_v1;

/* "xgc.scene.state/1": every obstacle's current pose and twist for one
 * (epoch, revision). This head, then frame_id, epoch, then obstacle_count
 * records: id, pose, twist. */
typedef struct xgc_scene_state_v1 {
  double stamp;           /* header.stamp */
  double scene_time;
  uint64_t revision;
  uint32_t playing;
  uint32_t obstacle_count;
} xgc_scene_state_v1;

/* "xgc.planar_pva/1": unicycle_reference_trajectory_msgs/PlanarPvaReference,
 * a planar position-velocity-acceleration setpoint for a ground robot (the
 * DMPC planner's output with planar_reference_output; the Scout controller's
 * alg/reference/pva input). */
typedef struct xgc_planar_pva_v1 {
  double stamp;           /* header.stamp */
  double x;
  double y;
  double yaw;
  double vx;
  double vy;
  double ax;
  double ay;
} xgc_planar_pva_v1;

/* --- PX4 flight-controller interface (ctl-px4 <-> ros_io) ----------------
 * Field for field the MAVROS messages the PX4 controller reads or writes.
 * On every input, the sample's envelope t_produce is its receive time. */

/* "xgc.fcu_state/1": mavros_msgs/State. */
typedef struct xgc_fcu_state_v1 {
  double stamp;
  uint8_t connected;
  uint8_t armed;
  uint8_t guided;
  uint8_t manual_input;
  uint8_t system_status;
  uint8_t reserved[3];
  char mode[32];          /* NUL-terminated, e.g. "OFFBOARD" */
} xgc_fcu_state_v1;

/* "xgc.twist/1": geometry_msgs/TwistStamped. */
typedef struct xgc_twist_v1 {
  double stamp;
  double linear[3];
  double angular[3];
} xgc_twist_v1;

/* "xgc.battery/1": the sensor_msgs/BatteryState fields the controller reads. */
typedef struct xgc_battery_v1 {
  double stamp;
  double voltage;
  double percentage;      /* 0..1 */
} xgc_battery_v1;

/* "xgc.command/1": an operator command string (std_msgs/String on /command). */
typedef struct xgc_command_v1 {
  char text[64];          /* NUL-terminated */
} xgc_command_v1;

/* "xgc.clock/1": replay only; advances a replaying module's clock with no input. */
typedef struct xgc_clock_v1 {
  double seconds;
} xgc_clock_v1;

/* "xgc.position_target/1": mavros_msgs/PositionTarget (local frame): the
 * controller's setpoint out, and a planner's setpoint in. */
typedef struct xgc_position_target_v1 {
  double stamp;
  double position[3];
  double velocity[3];
  double acceleration[3];
  double yaw;             /* rad; as PositionTarget.yaw */
  double yaw_rate;
  uint16_t type_mask;     /* PositionTarget IGNORE_* bits */
  uint8_t coordinate_frame;
  uint8_t reserved[5];
} xgc_position_target_v1;

/* "xgc.body_rate_thrust/1": body rates + normalized thrust
 * (mavros_msgs/AttitudeTarget with the attitude ignored). */
typedef struct xgc_body_rate_thrust_v1 {
  double stamp;
  double body_rate[3];
  double thrust;
} xgc_body_rate_thrust_v1;

/* "xgc.fcu_request/1": a MAVROS service request the edge must make. */
typedef struct xgc_fcu_request_v1 {
  double stamp;
  uint32_t kind;          /* 1 arm/disarm (cmd/command 400), 2 set_mode */
  uint32_t arm;           /* kind 1: 1 arm, 0 disarm */
  char mode[32];          /* kind 2: custom mode, NUL-terminated */
} xgc_fcu_request_v1;

/* "xgc.controller_status/1": the controller's control-region state name. */
typedef struct xgc_controller_status_v1 {
  double stamp;
  char state[48];         /* NUL-terminated */
} xgc_controller_status_v1;

/* --- Reference trajectories (ref-trajectory <-> ros_io, ctl) -------------
 * Field for field the multirotor_reference_trajectory_msgs messages, times
 * kept as their exact sec/nsec. Each head is followed by its variable parts,
 * in the order the comment lists them. plugins/common/reference_wire.hpp
 * encodes and decodes them. */

/* std_msgs/Header, exact. */
typedef struct xgc_ref_header_v1 {
  uint32_t seq;
  uint32_t stamp_sec;
  uint32_t stamp_nsec;
  uint32_t reserved;
  char frame_id[32];      /* NUL-terminated */
} xgc_ref_header_v1;

/* "xgc.ref.analytic/1": AnalyticReference; then params_len doubles. */
typedef struct xgc_ref_analytic_v1 {
  xgc_ref_header_v1 header;
  uint32_t request_id;
  uint32_t trajectory_id;
  uint32_t revision;
  uint32_t flags;
  uint32_t start_sec;
  uint32_t start_nsec;
  uint16_t analytic_type;
  uint16_t reserved;
  uint32_t params_len;
  double duration;
  double origin_position[3];
  double origin_q_xyzw[4];
} xgc_ref_analytic_v1;

/* One FlatReferencePoint. */
typedef struct xgc_ref_flat_point_v1 {
  double t_from_start;
  double position[3];
  double velocity[3];
  double acceleration[3];
  double jerk[3];
  double snap[3];
  double yaw;
  double yaw_rate;
  double yaw_accel;
} xgc_ref_flat_point_v1;

/* "xgc.ref.sampled/1": SampledReference; then points_len xgc_ref_flat_point_v1. */
typedef struct xgc_ref_sampled_v1 {
  xgc_ref_header_v1 header;
  uint32_t trajectory_id;
  uint32_t revision;
  uint32_t flags;
  uint32_t points_len;
  uint32_t start_sec;
  uint32_t start_nsec;
  double sample_dt;
} xgc_ref_sampled_v1;

/* "xgc.ref.waypoint_request/1": WaypointReferenceRequest; then
 * waypoints_len poses (7 doubles: position xyz, orientation xyzw),
 * constraint_types_len uint8 (zero-padded to a multiple of 8 bytes),
 * region_size_len vectors (3 doubles) and segment_times_len doubles. */
typedef struct xgc_ref_waypoint_request_v1 {
  xgc_ref_header_v1 header;
  uint32_t request_id;
  uint32_t trajectory_id;
  uint32_t revision;
  uint32_t flags;
  uint32_t waypoints_len;
  uint32_t constraint_types_len;
  uint32_t region_size_len;
  uint32_t segment_times_len;
  double start_velocity[3];
  double start_acceleration[3];
  double end_velocity[3];
  double end_acceleration[3];
  double desired_speed;
  double time_weight;
  double max_body_rate;
  double max_tilt;
  double min_thrust;
  double max_thrust;
  uint32_t max_iterations;
  uint8_t objective;
  uint8_t reserved[3];
  double rel_cost_tol;
  double max_velocity;
  double max_acceleration;
  double max_jerk;
  double max_snap;
} xgc_ref_waypoint_request_v1;

/* "xgc.ref.polynomial/1": ActivePolynomialReference; then
 * segment_durations_len, coeff_x_len, coeff_y_len, coeff_z_len and
 * coeff_yaw_len doubles, in that order. */
typedef struct xgc_ref_polynomial_v1 {
  xgc_ref_header_v1 header;
  uint32_t trajectory_id;
  uint32_t revision;
  uint32_t flags;
  uint8_t order;
  uint8_t reserved[3];
  uint32_t start_sec;
  uint32_t start_nsec;
  double duration;
  uint32_t segment_durations_len;
  uint32_t coeff_x_len;
  uint32_t coeff_y_len;
  uint32_t coeff_z_len;
  uint32_t coeff_yaw_len;
  uint32_t reserved2;
} xgc_ref_polynomial_v1;

/* "xgc.ref.status/1": ReferenceStatus. */
typedef struct xgc_ref_status_v1 {
  xgc_ref_header_v1 header;
  uint8_t state;
  uint8_t active_type;
  uint8_t reserved[2];
  uint32_t flags;
  uint32_t active_trajectory_id;
  uint32_t active_revision;
} xgc_ref_status_v1;

/* "xgc.ref.reset/1": a reset request (std_msgs/Empty); the payload is unused. */
typedef struct xgc_ref_reset_v1 {
  uint64_t reserved;
} xgc_ref_reset_v1;

#ifdef __cplusplus
#define XGC_SCHEMA_ASSERT static_assert
#else
#define XGC_SCHEMA_ASSERT _Static_assert
#endif
XGC_SCHEMA_ASSERT(sizeof(xgc_imu_v1) == 56, "xgc_imu_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_attitude_target_v1) == 56, "xgc_attitude_target_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_pose_v1) == 64, "xgc_pose_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_hover_thrust_v1) == 64, "xgc_hover_thrust_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_rigid_state_v1) == 112, "xgc_rigid_state_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_flat_ref_v1) == 160, "xgc_flat_ref_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_attitude_rate_cmd_v1) == 104, "xgc_attitude_rate_cmd_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_rigid_state_estimate_v1) == 304, "xgc_rigid_state_estimate_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_dmpc_assumed_trajectory_v1) == 32, "xgc_dmpc_assumed_trajectory_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_dmpc_sync_trigger_v1) == 32, "xgc_dmpc_sync_trigger_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_dmpc_mission_state_v1) == 80, "xgc_dmpc_mission_state_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_dmpc_formation_tick_v1) == 16, "xgc_dmpc_formation_tick_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_scene_snapshot_v1) == 24, "xgc_scene_snapshot_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_scene_state_v1) == 32, "xgc_scene_state_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_planar_pva_v1) == 64, "xgc_planar_pva_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_fcu_state_v1) == 48, "xgc_fcu_state_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_twist_v1) == 56, "xgc_twist_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_battery_v1) == 24, "xgc_battery_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_command_v1) == 64, "xgc_command_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_clock_v1) == 8, "xgc_clock_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_position_target_v1) == 104, "xgc_position_target_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_body_rate_thrust_v1) == 40, "xgc_body_rate_thrust_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_fcu_request_v1) == 48, "xgc_fcu_request_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_controller_status_v1) == 56, "xgc_controller_status_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_header_v1) == 48, "xgc_ref_header_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_analytic_v1) == 144, "xgc_ref_analytic_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_flat_point_v1) == 152, "xgc_ref_flat_point_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_sampled_v1) == 80, "xgc_ref_sampled_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_waypoint_request_v1) == 272, "xgc_ref_waypoint_request_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_polynomial_v1) == 104, "xgc_ref_polynomial_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_status_v1) == 64, "xgc_ref_status_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_ref_reset_v1) == 8, "xgc_ref_reset_v1");

#ifdef __cplusplus
}
#endif

#endif /* XGC_SCHEMAS_V1_H */
