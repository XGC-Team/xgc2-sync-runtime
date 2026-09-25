// Driver glue for xgc2_math::control::DfbcGeometricController, shared by the
// ctl-dfbc plugin and its replay reference, so both drive the unmodified
// upstream controller identically. This glue is ours (the ROS strategy in
// px4_multirotor_controller is not reused); the controller is not.
//
// Rule: the latest reference is held; each state sample triggers one
// compute() with dt = stamp difference to the previous state sample,
// defaulting to the configured nominal period on the first sample or on a
// non-positive/oversized gap.
#pragma once

#include <cmath>

#include <xgc2_math/control/dfbc_geometric_controller.hpp>

#include "xgc_schemas_v1.h"

namespace ctl_dfbc {

namespace ctl = xgc2_math::control;

struct Driver {
  ctl::DfbcGeometricController controller;
  double nominal_dt{0.01};
  double max_dt{0.1};
  bool has_reference{false};
  xgc2_math::trajectory::FlatOutput3 reference{};
  double last_state_stamp{NAN};

  void configure(const ctl::DfbcGeometricConfig& config, double nominal, double maximum) {
    controller.configure(config);
    nominal_dt = nominal;
    max_dt = maximum;
    has_reference = false;
    last_state_stamp = NAN;
  }

  void set_reference(const xgc_flat_ref_v1& r) {
    reference.position = Eigen::Vector3d(r.position[0], r.position[1], r.position[2]);
    reference.velocity = Eigen::Vector3d(r.velocity[0], r.velocity[1], r.velocity[2]);
    reference.acceleration = Eigen::Vector3d(r.acceleration[0], r.acceleration[1], r.acceleration[2]);
    reference.jerk = Eigen::Vector3d(r.jerk[0], r.jerk[1], r.jerk[2]);
    reference.snap = Eigen::Vector3d(r.snap[0], r.snap[1], r.snap[2]);
    reference.yaw = r.yaw;
    reference.yaw_rate = r.yaw_rate;
    reference.yaw_accel = r.yaw_accel;
    reference.flags = r.flags;
    has_reference = true;
  }

  // Returns false (no command) until a reference has arrived.
  bool on_state(const xgc_rigid_state_v1& s, xgc_attitude_rate_cmd_v1* out) {
    double dt = nominal_dt;
    if (std::isfinite(last_state_stamp)) {
      const double gap = s.stamp - last_state_stamp;
      if (gap > 0.0 && gap <= max_dt) dt = gap;
    }
    last_state_stamp = s.stamp;
    if (!has_reference) return false;
    ctl::DfbcGeometricInput in;
    in.current.position = Eigen::Vector3d(s.position[0], s.position[1], s.position[2]);
    in.current.velocity = Eigen::Vector3d(s.velocity[0], s.velocity[1], s.velocity[2]);
    in.current.attitude = Eigen::Quaterniond(s.q_wxyz[0], s.q_wxyz[1], s.q_wxyz[2], s.q_wxyz[3]);
    in.current.body_rate = Eigen::Vector3d(s.body_rate[0], s.body_rate[1], s.body_rate[2]);
    in.reference = reference;
    in.dt = dt;
    const ctl::DfbcGeometricOutput o = controller.compute(in);
    *out = xgc_attitude_rate_cmd_v1{};
    out->stamp = s.stamp;
    out->specific_thrust = o.specific_thrust;
    out->q_wxyz[0] = o.desired_attitude.w();
    out->q_wxyz[1] = o.desired_attitude.x();
    out->q_wxyz[2] = o.desired_attitude.y();
    out->q_wxyz[3] = o.desired_attitude.z();
    for (int i = 0; i < 3; ++i) {
      out->body_rate[i] = o.body_rate_command[i];
      out->position_error[i] = o.position_error[i];
    }
    out->success = o.success ? 1u : 0u;
    out->flags = o.flags;
    return true;
  }
};

}  // namespace ctl_dfbc
