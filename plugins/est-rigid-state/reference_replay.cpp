// Reference for the equality test: feeds a recorded input sequence straight
// into VrpnPx4RotorStateEstimatorRuntime the way the ROS node does
// (RigidStateInputProducer per topic, then the loop's update and
// RigidStateOutputConsumer), with the ROS clock at each sample stamp. The
// state timer fires whenever the stamp crosses a 1 / state_publish_rate_hz
// tick. No ports, no host, no plugin code.
//
// Config: the defaults with extrinsic_verified = true, as the test's plugin
// config sets.
//
// Numbers are IEEE-754 bit patterns in hex (16 digits), so both directions are
// exact.
// stdin:  "0 <stamp> <ax> <ay> <az> <gx> <gy> <gz>"          imu
//         "1 <stamp> <px> <py> <pz> <qw> <qx> <qy> <qz>"     pose
// stdout: "S <stamp> <p xyz> <v xyz> <q wxyz> <w xyz>"       state
//         "V <stamp> <p xyz> <q wxyz>"                       vision pose

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "estimator_vrpn_px4_rotor_state/common/config_utils.h"
#include "estimator_vrpn_px4_rotor_state/common/input_sample_timing.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"

namespace rs = estimator_vrpn_px4_rotor_state;
namespace sm = state_machine;

namespace {

void print(double d) {
  uint64_t bits;
  std::memcpy(&bits, &d, sizeof bits);
  std::printf(" %016" PRIx64, bits);
}
void print_quat(const Eigen::Quaterniond& q) { print(q.w()); print(q.x()); print(q.y()); print(q.z()); }
void print_vec(const Eigen::Vector3d& v) { print(v.x()); print(v.y()); print(v.z()); }

bool read(double* out, int n) {
  for (int i = 0; i < n; ++i) {
    uint64_t bits;
    if (std::scanf("%" SCNx64, &bits) != 1) return false;
    std::memcpy(&out[i], &bits, sizeof bits);
  }
  return true;
}

}  // namespace

int main() {
  rs::VrpnPx4RotorStateEstimatorConfig config;
  config.extrinsic_verified = true;
  rs::config_utils::normalizeConfig(config);
  rs::VrpnPx4RotorStateEstimatorRuntime runtime;
  runtime.setConfig(config);
  rs::VrpnPx4RotorStateEstimatorInput input;
  long long last_tick = -1;

  unsigned port = 0;
  double stamp = 0.0;
  while (std::scanf("%u", &port) == 1 && read(&stamp, 1)) {
    const double now = stamp;  // the receipt clock in replay
    sm::EventId id = 0;
    if (port == 0) {
      double a[3], g[3];
      if (!read(a, 3) || !read(g, 3)) return 2;
      auto& s = input.imu;
      rs::input_timing::updateSampleTiming(s, stamp, now);
      s.angular_velocity = Eigen::Vector3d(g[0], g[1], g[2]);
      s.linear_acceleration = Eigen::Vector3d(a[0], a[1], a[2]);
      s.stamp_sec = stamp;
      s.received = true;
      s.valid = xgc2_math::isFinite(s.angular_velocity) && xgc2_math::isFinite(s.linear_acceleration);
      id = rs::event_type::INPUT_IMU_UPDATED;
    } else {
      double p[3], q[4];
      if (!read(p, 3) || !read(q, 4)) return 2;
      auto& s = input.vrpn_pose;
      rs::input_timing::updateSampleTiming(s, stamp, now);
      const Eigen::Quaterniond raw(q[0], q[1], q[2], q[3]);
      s.pose.position = Eigen::Vector3d(p[0], p[1], p[2]);
      s.pose.orientation = xgc2_math::normalizedQuaternion(raw);
      s.stamp_sec = stamp;
      s.received = true;
      s.valid = xgc2_math::isFinite(s.pose.position) && xgc2_math::isFinite(raw) && raw.norm() > 1.0e-9;
      id = rs::event_type::INPUT_VRPN_POSE_UPDATED;
    }
    sm::Event event(id, sm::EventTimestamp{now});
    event.category = sm::EventCategory::kInput;
    (void)runtime.postInputEvent(std::move(event), input);

    runtime.update(now);
    for (const auto& ev : runtime.getStateMachine().currentOutputEvents()) {
      if (ev.id != rs::output_event_type::PUBLISH_VISION_POSE) continue;
      const auto out = runtime.snapshotOutput();
      constexpr uint32_t kBlocking = rs::kVrpnMissing | rs::kVrpnStale | rs::kInvalidVrpn | rs::kTimeJump |
                                     rs::kPoseTimeAlignmentRejected | rs::kVrpnFault | rs::kFilterImuOnly;
      if (!out.has_corrected_vision_pose || (out.flags & kBlocking) != 0u) continue;
      std::printf("V");
      print(std::isfinite(ev.timestamp) && ev.timestamp > 0.0 ? ev.timestamp : now);
      print_vec(out.corrected_vision_pose.position);
      print_quat(out.corrected_vision_pose.orientation);
      std::printf("\n");
    }
    const long long tick = static_cast<long long>(std::floor(stamp * config.state_publish_rate_hz));
    if (tick != last_tick) {
      last_tick = tick;
      const auto out = runtime.refreshOutputSnapshot();
      std::printf("S");
      print(stamp);
      print_vec(out.state.position);
      print_vec(out.state.velocity);
      print_quat(out.state.orientation);
      print_vec(out.state.angular_velocity);
      std::printf("\n");
    }
  }
  return 0;
}
