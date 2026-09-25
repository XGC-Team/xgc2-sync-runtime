// est-rigid-state: the VRPN/PX4 rotor rigid-state ESKF as an xgc_rt plugin.
//
// A WRAP: it runs the unmodified VrpnPx4RotorStateEstimatorRuntime (ESKF on
// xgc2_math::Pose3InertialEskf, domain FSM SelfCheck -> Initializing ->
// Running <-> Coasting with a HealthMonitor region) from
// ros1/perception/estimator/rigid-state. Only RigidStateInputProducer, the
// node loop and RigidStateOutputConsumer are replaced by module I/O:
//
//   in  imu          xgc.imu/1          (raw IMU: accel, gyro)
//   in  pose         xgc.pose/1         (VRPN or LIO pose)
//   out rigid_state  xgc.rigid_state/1  (the node's state topic, at the state rate)
//   out vision_pose  xgc.pose/1         (corrected vision pose for PX4, on each
//                                        PUBLISH_VISION_POSE that may be published)
//
// Scheduling: trigger `both`, period = 1 / state_publish_rate_hz. A step
// applies the new samples in source-stamp order (ties: imu before pose), each
// posting its input event at its receipt time as the producer does, then
// updates the runtime at Session time and dispatches vision-pose events. On a
// round boundary it publishes the state snapshot, as the node's state timer
// does.
//
// time_source = "input" (replay and tests): the runtime clock is each sample's
// stamp, the runtime is updated after every sample, and the state is
// published whenever the stamp crosses a 1 / state_publish_rate_hz tick.
//
// Config keys are the ROS parameter names of the node (e.g. accel_noise_std,
// field_offset_xyz = [x, y, z], field_offset_rpy = [r, p, y]).

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "../common/flat_config.hpp"
#include "estimator_vrpn_px4_rotor_state/common/config_utils.h"
#include "estimator_vrpn_px4_rotor_state/common/input_sample_timing.h"
#include "estimator_vrpn_px4_rotor_state/vrpn_px4_rotor_state_estimator_runtime.h"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {

namespace rs = estimator_vrpn_px4_rotor_state;
namespace sm = state_machine;
namespace cfg = xgc_rt_config;

enum Port : uint32_t { kImu = 0, kPose = 1, kRigidState = 2, kVisionPose = 3 };

struct Pending {
  double stamp;
  double receipt;
  uint32_t port;
  xgc_imu_v1 imu;
  xgc_pose_v1 pose;
};

bool vision_pose_publishable(const rs::VrpnPx4RotorStateEstimatorOutput& out) {
  constexpr uint32_t kBlocking = rs::kVrpnMissing | rs::kVrpnStale | rs::kInvalidVrpn | rs::kTimeJump |
                                 rs::kPoseTimeAlignmentRejected | rs::kVrpnFault | rs::kFilterImuOnly;
  return out.has_corrected_vision_pose && (out.flags & kBlocking) == 0u;
}

void put_quat(double* q_wxyz, const Eigen::Quaterniond& q) {
  q_wxyz[0] = q.w();
  q_wxyz[1] = q.x();
  q_wxyz[2] = q.y();
  q_wxyz[3] = q.z();
}

void put_vec(double* out, const Eigen::Vector3d& v) {
  out[0] = v.x();
  out[1] = v.y();
  out[2] = v.z();
}

bool read_pose(const std::string& text, const char* xyz_key, const char* rpy_key, xgc2_math::Pose3* pose) {
  double xyz[3] = {pose->position.x(), pose->position.y(), pose->position.z()};
  double rpy[3] = {0.0, 0.0, 0.0};
  if (!cfg::numbers(text, xyz_key, xyz, 3) || !cfg::numbers(text, rpy_key, rpy, 3)) return false;
  pose->position = Eigen::Vector3d(xyz[0], xyz[1], xyz[2]);
  pose->orientation = xgc2_math::rpyToQuaternion(Eigen::Vector3d(rpy[0], rpy[1], rpy[2]));
  return true;
}

struct EstRigidState {
  const xgc_host_api* host;
  rs::VrpnPx4RotorStateEstimatorRuntime runtime;
  rs::VrpnPx4RotorStateEstimatorInput input;
  bool input_time{false};
  double state_rate_hz{100.0};
  long long last_state_tick{-1};
  std::vector<Pending> pending;

  void log(xgc_log_level level, const std::string& message) const { host->log(host->host, level, message.c_str()); }

  // RigidStateInputProducer::imuCallback / applyPose, with `receipt` as the
  // ROS receipt clock.
  void apply(const Pending& s) {
    sm::EventId id = 0;
    const char* source = nullptr;
    if (s.port == kImu) {
      auto& sample = input.imu;
      rs::input_timing::updateSampleTiming(sample, s.stamp, s.receipt);
      sample.angular_velocity = Eigen::Vector3d(s.imu.gyro[0], s.imu.gyro[1], s.imu.gyro[2]);
      sample.linear_acceleration = Eigen::Vector3d(s.imu.accel[0], s.imu.accel[1], s.imu.accel[2]);
      sample.stamp_sec = s.stamp;
      sample.received = true;
      sample.valid = xgc2_math::isFinite(sample.angular_velocity) && xgc2_math::isFinite(sample.linear_acceleration);
      id = rs::event_type::INPUT_IMU_UPDATED;
      source = "raw_imu";
    } else {
      auto& sample = input.vrpn_pose;
      rs::input_timing::updateSampleTiming(sample, s.stamp, s.receipt);
      const double* q = s.pose.q_wxyz;
      const Eigen::Quaterniond raw(q[0], q[1], q[2], q[3]);
      sample.pose.position = Eigen::Vector3d(s.pose.position[0], s.pose.position[1], s.pose.position[2]);
      sample.pose.orientation = xgc2_math::normalizedQuaternion(raw);
      sample.stamp_sec = s.stamp;
      sample.received = true;
      sample.valid = xgc2_math::isFinite(sample.pose.position) && xgc2_math::isFinite(raw) && raw.norm() > 1.0e-9;
      id = rs::event_type::INPUT_VRPN_POSE_UPDATED;
      source = "vrpn_pose";
    }
    sm::Event event(id, sm::EventTimestamp{s.receipt});
    event.source = source;
    event.category = sm::EventCategory::kInput;
    const sm::Status status = runtime.postInputEvent(std::move(event), input);
    if (!status.ok()) log(XGC_LOG_WARN, "input event rejected: " + status.message);
  }

  // The node loop body: update, then RigidStateOutputConsumer for vision pose.
  xgc_status update(double now_sec, uint64_t round) {
    runtime.update(now_sec);
    for (const auto& ev : runtime.getStateMachine().currentOutputEvents()) {
      if (ev.id != rs::output_event_type::PUBLISH_VISION_POSE) continue;
      const double stamp = std::isfinite(ev.timestamp) && ev.timestamp > 0.0 ? ev.timestamp : now_sec;
      const auto out = runtime.snapshotOutput();
      if (!vision_pose_publishable(out)) continue;
      xgc_pose_v1 msg{};
      msg.stamp = stamp;
      put_vec(msg.position, out.corrected_vision_pose.position);
      put_quat(msg.q_wxyz, out.corrected_vision_pose.orientation);
      if (host->publish(host->host, kVisionPose, round, reinterpret_cast<const uint8_t*>(&msg), sizeof msg) != XGC_OK)
        return XGC_ERR;
    }
    return XGC_OK;
  }

  // The state timer: RigidStateOutputConsumer's PUBLISH_STATE.
  xgc_status publish_state(double stamp, uint64_t round) {
    const auto out = runtime.refreshOutputSnapshot();
    xgc_rigid_state_v1 msg{};
    msg.stamp = stamp;
    put_vec(msg.position, out.state.position);
    put_vec(msg.velocity, out.state.velocity);
    put_quat(msg.q_wxyz, out.state.orientation);
    put_vec(msg.body_rate, out.state.angular_velocity);
    return host->publish(host->host, kRigidState, round, reinterpret_cast<const uint8_t*>(&msg), sizeof msg);
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    pending.clear();
    xgc_sample_view view;
    for (uint32_t port : {kImu, kPose}) {
      while (host->next(host->host, port, &view) == XGC_OK) {
        Pending s{};
        s.port = port;
        s.receipt = static_cast<double>(view.t_rx) * 1e-9;
        if (port == kImu && view.len == sizeof(xgc_imu_v1)) {
          std::memcpy(&s.imu, view.data, sizeof s.imu);
          s.stamp = s.imu.stamp;
        } else if (port == kPose && view.len == sizeof(xgc_pose_v1)) {
          std::memcpy(&s.pose, view.data, sizeof s.pose);
          s.stamp = s.pose.stamp;
        } else {
          log(XGC_LOG_WARN, "dropped a sample with the wrong payload size");
          continue;
        }
        pending.push_back(s);
      }
    }
    std::stable_sort(pending.begin(), pending.end(), [](const Pending& a, const Pending& b) { return a.stamp < b.stamp; });
    if (input_time) {
      for (auto s : pending) {
        s.receipt = s.stamp;
        apply(s);
        if (update(s.stamp, ctx->round) != XGC_OK) return XGC_ERR;
        const long long tick = static_cast<long long>(std::floor(s.stamp * state_rate_hz));
        if (tick != last_state_tick) {
          last_state_tick = tick;
          if (publish_state(s.stamp, ctx->round) != XGC_OK) return XGC_ERR;
        }
      }
      return XGC_OK;
    }
    for (const auto& s : pending) apply(s);
    const double now_sec = static_cast<double>(ctx->now) * 1e-9;
    if (update(now_sec, ctx->round) != XGC_OK) return XGC_ERR;
    return ctx->round_advanced ? publish_state(now_sec, ctx->round) : XGC_OK;
  }
};

template <typename F>
xgc_status guarded(const xgc_host_api* host, const char* where, F&& f) {
  try {
    return f();
  } catch (const std::exception& e) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": " + e.what()).c_str());
    return XGC_ERR;
  } catch (...) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": unknown exception").c_str());
    return XGC_ERR;
  }
}

void* create(const xgc_host_api* host) {
  try {
    auto* self = new EstRigidState{};
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<EstRigidState*>(p);
  return guarded(self->host, "configure", [&] {
    const std::string t = config ? config : "";
    rs::VrpnPx4RotorStateEstimatorConfig c;
    int buffer = static_cast<int>(c.inertial_buffer_capacity);
    bool ok = read_pose(t, "field_offset_xyz", "field_offset_rpy", &c.field_to_world) &&
              read_pose(t, "imu_to_vrpn_marker_xyz", "imu_to_vrpn_marker_rpy", &c.imu_to_vrpn_marker);
    for (auto [key, field] : std::initializer_list<std::pair<const char*, double*>>{
             {"loop_rate_hz", &c.loop_rate_hz},
             {"state_publish_rate_hz", &c.state_publish_rate_hz},
             {"vision_publish_rate_hz", &c.vision_publish_rate_hz},
             {"gravity_mps2", &c.gravity_mps2},
             {"imu_timeout_s", &c.imu_timeout_s},
             {"vrpn_timeout_s", &c.vrpn_timeout_s},
             {"coasting_timeout_s", &c.coasting_timeout_s},
             {"min_imu_rate_hz", &c.min_imu_rate_hz},
             {"min_vrpn_rate_hz", &c.min_vrpn_rate_hz},
             {"max_time_jump_s", &c.max_time_jump_s},
             {"accel_noise_std", &c.accel_noise_std},
             {"gyro_noise_std", &c.gyro_noise_std},
             {"vrpn_position_noise_std", &c.vrpn_position_noise_std},
             {"vrpn_orientation_noise_std", &c.vrpn_orientation_noise_std},
             {"vrpn_velocity_noise_std", &c.vrpn_velocity_noise_std},
             {"gyro_bias_random_walk_std", &c.gyro_bias_random_walk_std},
             {"accel_bias_random_walk_std", &c.accel_bias_random_walk_std},
             {"extrinsic_position_random_walk_std", &c.extrinsic_position_random_walk_std},
             {"extrinsic_orientation_random_walk_std", &c.extrinsic_orientation_random_walk_std},
             {"innovation_position_gate_m", &c.innovation_position_gate_m},
             {"innovation_orientation_gate_rad", &c.innovation_orientation_gate_rad},
             {"pose_position_kalman_gain", &c.pose_position_kalman_gain},
             {"pose_orientation_kalman_gain", &c.pose_orientation_kalman_gain},
             {"pose_update_convergence", &c.pose_update_convergence},
             {"velocity_innovation_gate_mps", &c.velocity_innovation_gate_mps},
             {"pose_nis_gate", &c.pose_nis_gate},
             {"covariance_high_threshold", &c.covariance_high_threshold},
             {"max_propagation_dt_s", &c.max_propagation_dt_s},
             {"pose_max_late_s", &c.pose_max_late_s},
             {"pose_max_early_s", &c.pose_max_early_s},
             {"pose_observation_delay_s", &c.pose_observation_delay_s},
             {"initial_position_variance", &c.initial_position_variance},
             {"initial_velocity_variance", &c.initial_velocity_variance},
             {"initial_orientation_variance", &c.initial_orientation_variance},
             {"initial_gyro_bias_variance", &c.initial_gyro_bias_variance},
             {"initial_accel_bias_variance", &c.initial_accel_bias_variance},
         }) {
      ok = ok && cfg::number(t, key, field);
    }
    ok = ok && cfg::integer(t, "inertial_buffer_capacity", &buffer) &&
         cfg::integer(t, "pose_update_iterations", &c.pose_update_iterations) &&
         cfg::boolean(t, "extrinsic_verified", &c.extrinsic_verified) &&
         cfg::boolean(t, "estimate_extrinsic", &c.estimate_extrinsic) &&
         cfg::boolean(t, "imu_noise_std_is_density", &c.imu_noise_std_is_density) &&
         cfg::boolean(t, "apply_pose_covariance_floor", &c.apply_pose_covariance_floor);
    const std::string source = cfg::text_or(t, "time_source", "session");
    if (!ok || (source != "session" && source != "input")) {
      self->log(XGC_LOG_ERROR, "invalid est-rigid-state config");
      return XGC_ERR;
    }
    if (buffer > 0) c.inertial_buffer_capacity = static_cast<std::size_t>(buffer);
    rs::config_utils::normalizeConfig(c);
    self->input_time = source == "input";
    self->state_rate_hz = c.state_publish_rate_hz;
    self->runtime.setConfig(c);
    self->input = {};
    return XGC_OK;
  });
}

xgc_status activate(void*) { return XGC_OK; }

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<EstRigidState*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void*) { return XGC_OK; }

void destroy(void* p) { delete static_cast<EstRigidState*>(p); }

const char* domain_state(void* p) {
  switch (static_cast<EstRigidState*>(p)->runtime.snapshotOutput().estimator_state) {
    case rs::state_type::SelfCheck: return "self_check";
    case rs::state_type::Initializing: return "initializing";
    case rs::state_type::Running: return "running";
    case rs::state_type::Coasting: return "coasting";
    default: return "unknown";
  }
}

const xgc_port_decl kPorts[] = {
    {"imu", XGC_PORT_IN, "xgc.imu/1", XGC_QOS_STATE},
    {"pose", XGC_PORT_IN, "xgc.pose/1", XGC_QOS_STATE},
    {"rigid_state", XGC_PORT_OUT, "xgc.rigid_state/1", XGC_QOS_STATE},
    {"vision_pose", XGC_PORT_OUT, "xgc.pose/1", XGC_QOS_STATE},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {
    XGC_RT_ABI_VERSION, 4u, "est-rigid-state", "0.1.0", kPorts, &kVtbl,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
