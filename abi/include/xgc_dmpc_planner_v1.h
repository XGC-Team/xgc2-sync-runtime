/* plan-dmpc aggregate inputs that are not already in xgc_schemas_v1.h.
 * PositionTarget and AssumedTrajectory stay on those existing schemas.
 * Do not add these fields to xgc_schemas_v1.h from this header. */
#ifndef XGC_DMPC_PLANNER_V1_H
#define XGC_DMPC_PLANNER_V1_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Comprehensive legacy only: n=3, q=1, N=40, T=0.1, eight agents. */
typedef struct xgc_dmpc_planner_config_v1 {
  char algorithm[16];     /* "legacy" */
  int32_t chain_n;        /* 3 */
  int32_t state_dim;      /* 9: position, velocity, previous acceleration */
  int32_t horizon;        /* 40 */
  double sampling_time;   /* 0.1 */
  int32_t self_id;        /* 1..8 */
  int32_t fleet_count;    /* 8 */
  char scene_id[64];      /* "dmpc-uav8_comprehensive" */
  uint16_t timeline_authority; /* roster origin; not request.origin */
  uint16_t reserved_authority;
} xgc_dmpc_planner_config_v1;

/* xgc.dmpc.timeline_ack/1. 56 bytes, the record dmpc-rounds writes.
 * revision, digest, command_id, accepted. Not a commit and not applied phase. */
typedef struct xgc_dmpc_timeline_ack_v1 {
  uint64_t revision;
  uint8_t digest[32];
  uint64_t command_id;
  uint32_t accepted;
  uint32_t reserved0;
} xgc_dmpc_timeline_ack_v1;
typedef xgc_dmpc_timeline_ack_v1 xgc_dmpc_mission_ack_v1;

/* One paired measurement. pose_stamp_sec must equal twist_stamp_sec, be finite,
 * and not be in the future relative to the caller's now. Age is not clamped. */
typedef struct xgc_dmpc_paired_state_v1 {
  double pose_stamp_sec;
  double twist_stamp_sec;
  double position[3];
  double orientation_xyzw[4];
  double linear_velocity[3];
} xgc_dmpc_paired_state_v1;

/* xgc.dmpc.measured_position/1. Measured peer position for goal bootstrap,
 * published at the planner cadence, independently of the assumed trajectory. */
typedef struct xgc_dmpc_measured_position_v1 {
  uint32_t uav_id;
  uint32_t reserved;
  double stamp_sec;
  double position[3];
} xgc_dmpc_measured_position_v1;

/* Controller status is a different message from the pose. Its stamp is not
 * the pose stamp and is not a readiness boolean. */
typedef struct xgc_dmpc_controller_status_v1 {
  double stamp_sec;
  char state[48];
} xgc_dmpc_controller_status_v1;

typedef struct xgc_dmpc_scene_ids_v1 {
  char scene_id[64];
  uint64_t revision;
  uint32_t body_count;
  uint32_t reserved;
} xgc_dmpc_scene_ids_v1;

/* One xgc.dmpc.scene_snapshot/1 blob: header, obstacles, parts, vertices.
 * Geometry conversion is PlainSceneAdapter, not a second model.
 * epoch is the snapshot epoch, not scene_id. motion_type is the source
 * string, not a value inferred from dynamic. Part pose is the part frame
 * in the obstacle frame, xyzw order, including identity. */
typedef struct xgc_dmpc_scene_header_v1 {
  uint32_t schema;
  uint32_t obstacle_count;
  uint32_t part_count;
  uint32_t vertex_count;
  uint64_t revision;
  char scene_id[32];
  char frame[16];
  char epoch[64]; /* offset 72; total 136 */
} xgc_dmpc_scene_header_v1;

typedef struct xgc_dmpc_scene_obstacle_v1 {
  char id[16];
  char name[48];
  uint32_t dynamic;
  uint32_t reserved0;
  double position[3];
  double orientation_xyzw[4];
  double linear[3];
  double angular[3];
  char motion_type[16]; /* offset 176, NUL-terminated; total 192 */
} xgc_dmpc_scene_obstacle_v1;

typedef struct xgc_dmpc_scene_part_v1 {
  uint32_t obstacle_index;
  uint32_t geometry_type; /* 0 cylinder, 1 box, 2 capsule, 3 sphere, 4 convex */
  char part_id[16];
  double param[4];
  uint32_t vertex_begin; /* offset 56 */
  uint32_t vertex_count; /* offset 60 */
  double position[3];    /* offset 64, part frame in the obstacle frame */
  double orientation_xyzw[4]; /* x, y, z, w; total 120 */
} xgc_dmpc_scene_part_v1;

typedef struct xgc_dmpc_scene_vertex_v1 {
  double xyz[3];
} xgc_dmpc_scene_vertex_v1;

/* Wall-clock receipt of the latest scene state, seconds since Unix epoch. */
typedef struct xgc_dmpc_scene_heartbeat_v1 {
  double received_wall_sec;
} xgc_dmpc_scene_heartbeat_v1;

/* xgc.dmpc.mission_timeline/1. p2C fills it, p2E checks order, p2D does not. */
typedef struct xgc_dmpc_mission_timeline_v1 {
  uint32_t schema;                 /* 1 */
  uint32_t kind;                   /* 1 start, 2 resume, 3 hold, 4 reset, 5 goal, 6 pattern */
  uint64_t revision;
  uint64_t command_id;
  uint64_t predecessor_revision;
  uint8_t predecessor_digest[32];
  uint64_t effective_round;
  int64_t anchor_ns;
  uint32_t rolling;                /* 0 held, 1 rolling */
  uint32_t pattern_id;
  double goal_xyz[3];
  char session_id[64];
  char origin[64];
} xgc_dmpc_mission_timeline_v1;

/* One committed view for planner round k. mission_ns is p2E's value. */
typedef struct xgc_dmpc_mission_commit_v1 {
  xgc_dmpc_mission_timeline_v1 request;
  uint64_t round_k;
  int64_t mission_ns;
} xgc_dmpc_mission_commit_v1;

typedef struct xgc_dmpc_timeline_status_v1 {
  uint64_t committed_revision;
  uint8_t committed_digest[32];
  uint64_t effective_round;
  int64_t mission_ns;
  uint64_t applied_revision;
  int64_t applied_mission_ns;
  uint32_t held;
  uint32_t fault;
  char reason[80];
} xgc_dmpc_timeline_status_v1;

typedef struct xgc_dmpc_planner_status_v1 {
  double stamp_sec;
  char lifecycle[32];
  char reject_reason[96];
  uint8_t solver_called;
  uint8_t solver_ok;
  uint8_t tracking;
  uint8_t reserved;
  int32_t qp_status;
  int32_t qp_iterations;
} xgc_dmpc_planner_status_v1;

#ifdef __cplusplus
}
#endif

#endif
