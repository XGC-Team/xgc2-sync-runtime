// ros_io: the aggregator's ROS edge. It does ordinary ROS subscribe and
// publish and copies data between topics and module I/O. It is not the
// ros1_bridge package and there is no separate bridge process: domain
// modules never touch ROS; only this plugin does.
//
// Every port is optional and is enabled by `<port>_topic` in the config (an
// enabled port must also be bound in the manifest, and a bound port needs its
// topic):
//
//   ROS -> module outputs
//     imu              sensor_msgs/Imu                      -> xgc.imu/1
//     pose             geometry_msgs/PoseStamped            -> xgc.pose/1
//     attitude_target  mavros_msgs/AttitudeTarget           -> xgc.attitude_target/1
//     own_plan         formation_generator/AssumedTrajectory -> xgc.dmpc.assumed_trajectory/1
//   module inputs -> ROS
//     vision_pose      xgc.pose/1                   -> geometry_msgs/PoseStamped (frame `frame_id`)
//     neighbor_plans   xgc.dmpc.assumed_trajectory/1 -> formation_generator/AssumedTrajectory
//     sync_trigger     xgc.dmpc.sync_trigger/1       -> periodic_sync/SyncTrigger
//     rigid_state_estimate xgc.rigid_state_estimate/1 -> rigid_state_estimator_msgs/RigidStateEstimate
//
// Threading: ROS callbacks run on this plugin's own thread. Each step first
// publishes what the modules wrote since the last step, then services this
// plugin's ROS callback queue until the round's deadline, so a callback
// writes module outputs as soon as its message arrives. ros::init is called
// once per process (node name `node_name`, default xgc_ros_io), with
// ROS_MASTER_URI from the environment. Times: ROS time in seconds is the
// Session time (both are the host clock in wall-clock runs).
//
// Config: `<port>_topic` (string), `node_name`, `frame_id` (default "world"),
// `queue_size` (default 10).

#include <cmath>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

#include <formation_generator/AssumedTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <periodic_sync/SyncTrigger.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "../common/flat_config.hpp"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {

namespace cfg = xgc_rt_config;

enum Port : uint32_t {
  kImu = 0,
  kPose,
  kAttitudeTarget,
  kOwnPlan,
  kVisionPose,
  kNeighborPlans,
  kSyncTrigger,
  kRigidStateEstimate,
  kPortCount
};

const char* const kPortNames[kPortCount] = {"imu", "pose", "attitude_target", "own_plan",
                                            "vision_pose", "neighbor_plans", "sync_trigger", "rigid_state_estimate"};

double stamp_or_now(const ros::Time& t) { return (t.isZero() ? ros::Time::now() : t).toSec(); }

void quat(double* wxyz, const geometry_msgs::Quaternion& q) {
  wxyz[0] = q.w;
  wxyz[1] = q.x;
  wxyz[2] = q.y;
  wxyz[3] = q.z;
}

struct RosIo {
  const xgc_host_api* host;
  std::string topics[kPortCount];
  std::string node_name{"xgc_ros_io"};
  std::string frame_id{"world"};
  int queue_size{10};
  ros::CallbackQueue queue;
  std::unique_ptr<ros::NodeHandle> nh;
  std::vector<ros::Subscriber> subs;
  ros::Publisher pubs[kPortCount];
  uint64_t round{0};
  uint64_t from_ros{0};
  uint64_t to_ros{0};
  bool write_failed{false};

  void log(xgc_log_level level, const std::string& m) const { host->log(host->host, level, m.c_str()); }

  bool enabled(Port p) const { return !topics[p].empty(); }

  template <typename T>
  void write(Port p, const T& payload) {
    write_bytes(p, reinterpret_cast<const uint8_t*>(&payload), sizeof payload);
  }

  void write_bytes(Port p, const uint8_t* data, size_t len) {
    if (host->publish(host->host, p, round, data, static_cast<uint32_t>(len)) != XGC_OK) write_failed = true;
    ++from_ros;
  }

  // --- ROS -> module outputs ------------------------------------------------

  void on_imu(const sensor_msgs::Imu::ConstPtr& m) {
    xgc_imu_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.accel[0] = m->linear_acceleration.x;
    s.accel[1] = m->linear_acceleration.y;
    s.accel[2] = m->linear_acceleration.z;
    s.gyro[0] = m->angular_velocity.x;
    s.gyro[1] = m->angular_velocity.y;
    s.gyro[2] = m->angular_velocity.z;
    write(kImu, s);
  }

  void on_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    xgc_pose_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.position[0] = m->pose.position.x;
    s.position[1] = m->pose.position.y;
    s.position[2] = m->pose.position.z;
    quat(s.q_wxyz, m->pose.orientation);
    write(kPose, s);
  }

  void on_attitude_target(const mavros_msgs::AttitudeTarget::ConstPtr& m) {
    xgc_attitude_target_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    quat(s.q_wxyz, m->orientation);
    s.thrust = m->thrust;
    s.ignore_thrust = (m->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) ? 1u : 0u;
    write(kAttitudeTarget, s);
  }

  void on_plan(const formation_generator::AssumedTrajectory::ConstPtr& m) {
    xgc_dmpc_assumed_trajectory_v1 h{};
    h.stamp = stamp_or_now(m->header.stamp);
    h.uav_id = m->uav_id;
    h.num_states = m->num_states;
    h.num_timesteps = m->num_timesteps;
    h.rest_len = static_cast<uint32_t>(m->rest_position.size());
    h.valid = m->valid ? 1u : 0u;
    if (m->states.size() != static_cast<size_t>(m->num_states) * m->num_timesteps) {
      log(XGC_LOG_WARN, "ros_io: AssumedTrajectory with inconsistent dimensions dropped");
      return;
    }
    std::vector<uint8_t> out(sizeof h + 8 * (m->states.size() + m->rest_position.size()));
    std::memcpy(out.data(), &h, sizeof h);
    std::memcpy(out.data() + sizeof h, m->states.data(), 8 * m->states.size());
    std::memcpy(out.data() + sizeof h + 8 * m->states.size(), m->rest_position.data(), 8 * m->rest_position.size());
    write_bytes(kOwnPlan, out.data(), out.size());
  }

  // --- module inputs -> ROS -------------------------------------------------

  void forward_inputs() {
    xgc_sample_view v;
    while (host->next(host->host, kVisionPose, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_pose_v1)) continue;
      xgc_pose_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      geometry_msgs::PoseStamped m;
      m.header.stamp.fromSec(s.stamp);
      m.header.frame_id = frame_id;
      m.pose.position.x = s.position[0];
      m.pose.position.y = s.position[1];
      m.pose.position.z = s.position[2];
      m.pose.orientation.w = s.q_wxyz[0];
      m.pose.orientation.x = s.q_wxyz[1];
      m.pose.orientation.y = s.q_wxyz[2];
      m.pose.orientation.z = s.q_wxyz[3];
      pubs[kVisionPose].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kNeighborPlans, &v) == XGC_OK) {
      xgc_dmpc_assumed_trajectory_v1 h;
      if (v.len < sizeof h) continue;
      std::memcpy(&h, v.data, sizeof h);
      const size_t states = static_cast<size_t>(h.num_states) * h.num_timesteps;
      if (v.len != sizeof h + 8 * (states + h.rest_len)) continue;
      formation_generator::AssumedTrajectory m;
      m.header.stamp.fromSec(h.stamp);
      m.uav_id = static_cast<uint8_t>(h.uav_id);
      m.num_states = h.num_states;
      m.num_timesteps = h.num_timesteps;
      m.valid = h.valid != 0;
      m.states.resize(states);
      m.rest_position.resize(h.rest_len);
      std::memcpy(m.states.data(), v.data + sizeof h, 8 * states);
      std::memcpy(m.rest_position.data(), v.data + sizeof h + 8 * states, 8 * h.rest_len);
      pubs[kNeighborPlans].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kSyncTrigger, &v) == XGC_OK) {
      xgc_dmpc_sync_trigger_v1 h;
      if (v.len < sizeof h) continue;
      std::memcpy(&h, v.data, sizeof h);
      if (v.len != sizeof h + 4 * static_cast<size_t>(h.count)) continue;
      periodic_sync::SyncTrigger m;
      m.sequence_id = h.sequence_id;
      m.trigger_time.fromSec(h.trigger_time);
      m.published_time = ros::Time::now();
      m.active_participant_ids.resize(h.count);
      std::memcpy(m.active_participant_ids.data(), v.data + sizeof h, 4 * static_cast<size_t>(h.count));
      pubs[kSyncTrigger].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kRigidStateEstimate, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_rigid_state_estimate_v1)) continue;
      xgc_rigid_state_estimate_v1 e;
      std::memcpy(&e, v.data, sizeof e);
      rigid_state_estimator_msgs::RigidStateEstimate m;
      m.header.stamp.fromSec(e.stamp);
      m.estimator_state = e.estimator_state;
      m.flags = e.flags;
      m.position.x = e.position[0];
      m.position.y = e.position[1];
      m.position.z = e.position[2];
      auto vec = [](geometry_msgs::Vector3& out, const double* in) {
        out.x = in[0];
        out.y = in[1];
        out.z = in[2];
      };
      vec(m.velocity, e.velocity);
      m.orientation.w = e.q_wxyz[0];
      m.orientation.x = e.q_wxyz[1];
      m.orientation.y = e.q_wxyz[2];
      m.orientation.z = e.q_wxyz[3];
      vec(m.angular_velocity, e.angular_velocity);
      vec(m.linear_acceleration, e.linear_acceleration);
      vec(m.gravity, e.gravity);
      vec(m.accel_bias, e.accel_bias);
      m.vrpn_observation_state = e.vrpn_observation_state;
      m.filter_health = e.filter_health;
      m.last_pose_reject_reason = e.last_pose_reject_reason;
      m.last_pose_accepted = e.last_pose_accepted != 0;
      m.last_fused_pose_stamp_sec = e.last_fused_pose_stamp_sec;
      m.vrpn_innovation_window_chi_square = e.vrpn_innovation_window_chi_square;
      m.last_pose_position_innovation_norm_m = e.last_pose_position_innovation_norm_m;
      m.last_pose_orientation_innovation_norm_rad = e.last_pose_orientation_innovation_norm_rad;
      m.last_pose_mahalanobis_distance = e.last_pose_mahalanobis_distance;
      m.innovation_position_gate_m = e.innovation_position_gate_m;
      m.innovation_orientation_gate_rad = e.innovation_orientation_gate_rad;
      m.pose_nis_gate = e.pose_nis_gate;
      m.last_imu_sample_stamp_sec = e.last_imu_sample_stamp_sec;
      m.last_vrpn_pose_stamp_sec = e.last_vrpn_pose_stamp_sec;
      m.filter_inertial_stamp_sec = e.filter_inertial_stamp_sec;
      m.filter_pose_stamp_sec = e.filter_pose_stamp_sec;
      m.vrpn_consecutive_rejects = e.vrpn_consecutive_rejects;
      m.vrpn_consecutive_accepts = e.vrpn_consecutive_accepts;
      pubs[kRigidStateEstimate].publish(m);
      ++to_ros;
    }
  }

  xgc_status activate() {
    if (!ros::isInitialized()) {
      ros::M_string remappings;
      ros::init(remappings, node_name, ros::init_options::NoSigintHandler | ros::init_options::NoRosout);
    }
    nh = std::make_unique<ros::NodeHandle>();
    nh->setCallbackQueue(&queue);
    if (enabled(kImu)) subs.push_back(nh->subscribe(topics[kImu], queue_size, &RosIo::on_imu, this));
    if (enabled(kPose)) subs.push_back(nh->subscribe(topics[kPose], queue_size, &RosIo::on_pose, this));
    if (enabled(kAttitudeTarget))
      subs.push_back(nh->subscribe(topics[kAttitudeTarget], queue_size, &RosIo::on_attitude_target, this));
    if (enabled(kOwnPlan)) subs.push_back(nh->subscribe(topics[kOwnPlan], queue_size, &RosIo::on_plan, this));
    if (enabled(kVisionPose)) pubs[kVisionPose] = nh->advertise<geometry_msgs::PoseStamped>(topics[kVisionPose], queue_size);
    if (enabled(kNeighborPlans))
      pubs[kNeighborPlans] = nh->advertise<formation_generator::AssumedTrajectory>(topics[kNeighborPlans], queue_size);
    if (enabled(kSyncTrigger))
      pubs[kSyncTrigger] = nh->advertise<periodic_sync::SyncTrigger>(topics[kSyncTrigger], queue_size);
    if (enabled(kRigidStateEstimate))
      pubs[kRigidStateEstimate] =
          nh->advertise<rigid_state_estimator_msgs::RigidStateEstimate>(topics[kRigidStateEstimate], queue_size);
    return XGC_OK;
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    round = ctx->round;
    forward_inputs();
    // Service ROS callbacks until the round's deadline (minus 1 ms), so a
    // message becomes a module output as soon as it arrives.
    const int64_t until = ctx->deadline - 1000000;
    while (ros::ok()) {
      const int64_t left = until - host->now(host->host);
      if (left <= 0) break;
      queue.callAvailable(ros::WallDuration(std::min<int64_t>(left, 1000000) * 1e-9));
    }
    if (write_failed) {
      log(XGC_LOG_ERROR, "ros_io: a module output write failed");
      return XGC_ERR;
    }
    return XGC_OK;
  }

  void shutdown() {
    subs.clear();
    for (auto& p : pubs) p.shutdown();
    nh.reset();
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
    auto* self = new RosIo{};
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "configure", [&] {
    const std::string t = config ? config : "";
    for (uint32_t i = 0; i < kPortCount; ++i) self->topics[i] = cfg::text_or(t, (std::string(kPortNames[i]) + "_topic").c_str(), "");
    self->node_name = cfg::text_or(t, "node_name", "xgc_ros_io");
    self->frame_id = cfg::text_or(t, "frame_id", "world");
    if (!cfg::integer(t, "queue_size", &self->queue_size) || self->queue_size <= 0) {
      self->log(XGC_LOG_ERROR, "ros_io: invalid queue_size");
      return XGC_ERR;
    }
    return XGC_OK;
  });
}

xgc_status activate(void* p) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "activate", [&] { return self->activate(); });
}

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void* p) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "deactivate", [&] {
    self->shutdown();
    return XGC_OK;
  });
}

void destroy(void* p) { delete static_cast<RosIo*>(p); }

const char* domain_state(void* p) {
  const auto* self = static_cast<RosIo*>(p);
  if (!ros::isInitialized() || !ros::ok()) return "down";
  return self->from_ros || self->to_ros ? "flowing" : "connected";
}

const xgc_port_decl kPorts[kPortCount] = {
    {"imu", XGC_PORT_OUT_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"pose", XGC_PORT_OUT_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"attitude_target", XGC_PORT_OUT_OPTIONAL, "xgc.attitude_target/1", XGC_QOS_STATE},
    {"own_plan", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL},
    {"vision_pose", XGC_PORT_IN_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"neighbor_plans", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL},
    {"sync_trigger", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.sync_trigger/1", XGC_QOS_CONTROL},
    {"rigid_state_estimate", XGC_PORT_IN_OPTIONAL, "xgc.rigid_state_estimate/1", XGC_QOS_STATE},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {
    XGC_RT_ABI_VERSION, kPortCount, "ros-io", "0.1.0", kPorts, &kVtbl,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
