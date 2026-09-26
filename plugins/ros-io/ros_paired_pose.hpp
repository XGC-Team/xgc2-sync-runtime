#pragma once
// Pair pose selection for ros_io. One raw header stamp, not a second clock.
#include "xgc_dmpc_planner_v1.h"
#include "xgc_schemas_v1.h"

inline bool accept_pair_pose(bool local_pose_enabled, bool sample_is_local) {
  return local_pose_enabled == sample_is_local;
}

inline bool same_raw_header(bool have_pose, bool have_twist, double pose_stamp, double twist_stamp) {
  return have_pose && have_twist && pose_stamp == twist_stamp;
}

// Copies the sample only when it is the selected pose source. The stamp is the
// raw ROS header stamp, including zero. A rejected sample leaves the cache.
inline bool note_pair_pose(bool local_pose_enabled, bool sample_is_local, xgc_pose_v1* cache, double raw_header_stamp,
                           double x, double y, double z, double qw, double qx, double qy, double qz) {
  if (!accept_pair_pose(local_pose_enabled, sample_is_local)) return false;
  cache->stamp = raw_header_stamp;
  cache->position[0] = x;
  cache->position[1] = y;
  cache->position[2] = z;
  cache->q_wxyz[0] = qw;
  cache->q_wxyz[1] = qx;
  cache->q_wxyz[2] = qy;
  cache->q_wxyz[3] = qz;
  return true;
}

inline void fill_paired(const xgc_pose_v1& pose, double twist_raw_stamp, double vx, double vy, double vz,
                        xgc_dmpc_paired_state_v1* out) {
  out->pose_stamp_sec = pose.stamp;
  out->twist_stamp_sec = twist_raw_stamp;
  out->position[0] = pose.position[0];
  out->position[1] = pose.position[1];
  out->position[2] = pose.position[2];
  out->orientation_xyzw[0] = pose.q_wxyz[1];
  out->orientation_xyzw[1] = pose.q_wxyz[2];
  out->orientation_xyzw[2] = pose.q_wxyz[3];
  out->orientation_xyzw[3] = pose.q_wxyz[0];
  out->linear_velocity[0] = vx;
  out->linear_velocity[1] = vy;
  out->linear_velocity[2] = vz;
}
