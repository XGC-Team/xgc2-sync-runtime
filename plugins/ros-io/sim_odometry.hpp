// Measured model pose and velocity -> MAVROS-compatible body-twist Odometry.
#pragma once
#include <cmath>
#include "xgc_schemas_v1.h"

struct SimOdometry {
  xgc_pose_v1 pose{};
  double linear[3]{}, angular[3]{};
};

// Never synthesize velocity from position or pair different integration steps.
inline bool measured_sim_odometry(const xgc_pose_v1& pose, const xgc_twist_v1& velocity,
                                  double last_stamp, SimOdometry* result) {
  if (!std::isfinite(pose.stamp) || pose.stamp <= last_stamp || pose.stamp != velocity.stamp) return false;
  double norm = 0;
  for (double q : pose.q_wxyz) { if (!std::isfinite(q)) return false; norm += q*q; }
  if (std::abs(norm - 1.0) > 1e-6) return false;
  for (int i = 0; i < 3; ++i)
    if (!std::isfinite(pose.position[i]) || !std::isfinite(velocity.linear[i]) || !std::isfinite(velocity.angular[i])) return false;
  result->pose = pose;
  // Inverse rotation: model velocity is world/local-parent; Odometry declares
  // its twist in child_frame_id, as actual MAVROS does.
  const double w = pose.q_wxyz[0], x = pose.q_wxyz[1], y = pose.q_wxyz[2], z = pose.q_wxyz[3];
  const double r[3][3] = {
    {1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)},
    {2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)},
    {2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)}};
  for (int i = 0; i < 3; ++i) {
    result->linear[i] = result->angular[i] = 0;
    for (int j = 0; j < 3; ++j) {
      result->linear[i] += r[i][j] * velocity.linear[j];
      result->angular[i] += r[i][j] * velocity.angular[j];
    }
  }
  return true;
}
