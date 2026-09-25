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

#ifdef __cplusplus
#define XGC_SCHEMA_ASSERT static_assert
#else
#define XGC_SCHEMA_ASSERT _Static_assert
#endif
XGC_SCHEMA_ASSERT(sizeof(xgc_imu_v1) == 56, "xgc_imu_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_attitude_target_v1) == 56, "xgc_attitude_target_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_pose_v1) == 64, "xgc_pose_v1");
XGC_SCHEMA_ASSERT(sizeof(xgc_hover_thrust_v1) == 64, "xgc_hover_thrust_v1");

#ifdef __cplusplus
}
#endif

#endif /* XGC_SCHEMAS_V1_H */
