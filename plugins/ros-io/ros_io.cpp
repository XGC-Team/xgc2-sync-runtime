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
//     fcu_state        mavros_msgs/State                    -> xgc.fcu_state/1
//     local_pose       geometry_msgs/PoseStamped            -> xgc.pose/1
//     local_velocity   geometry_msgs/TwistStamped           -> xgc.twist/1
//     fcu_imu          sensor_msgs/Imu                      -> xgc.imu/1
//     battery          sensor_msgs/BatteryState             -> xgc.battery/1
//     command          std_msgs/String                      -> xgc.command/1
//     alg_setpoint     mavros_msgs/PositionTarget           -> xgc.position_target/1 (a planner's setpoint)
//     ref_analytic     multirotor_reference_trajectory_msgs/AnalyticReference       -> xgc.ref.analytic/1
//     ref_waypoint     multirotor_reference_trajectory_msgs/WaypointReferenceRequest -> xgc.ref.waypoint_request/1
//     ref_sampled      multirotor_reference_trajectory_msgs/SampledReference        -> xgc.ref.sampled/1
//     ref_reset        std_msgs/Empty                        -> xgc.ref.reset/1
//     hover_thrust     hover_thrust_estimator_msgs/HoverThrustEstimate -> xgc.hover_thrust/1
//     controller_state std_msgs/String (custom/statustext) -> xgc.controller_status/1 (stamp = receipt)
//   module inputs -> ROS
//     vision_pose      xgc.pose/1                   -> geometry_msgs/PoseStamped (frame `frame_id`)
//     neighbor_plans   xgc.dmpc.assumed_trajectory/1 -> formation_generator/AssumedTrajectory
//     sync_trigger     xgc.dmpc.sync_trigger/1       -> periodic_sync/SyncTrigger
//     rigid_state_estimate xgc.rigid_state_estimate/1 -> rigid_state_estimator_msgs/RigidStateEstimate
//     setpoint         xgc.position_target/1        -> mavros_msgs/PositionTarget (frame "map")
//     attitude_rate    xgc.body_rate_thrust/1       -> mavros_msgs/AttitudeTarget (attitude ignored)
//     status           xgc.controller_status/1      -> std_msgs/String
//     fcu_request      xgc.fcu_request/1            -> MAVROS service calls. Here the "topic" is
//                      the MAVROS namespace (e.g. /uav1/mavros): arm/disarm calls
//                      <ns>/cmd/command (MAV_CMD 400), set_mode calls <ns>/set_mode.
//                      Calls run in order on a worker thread, as the ROS node ran them
//                      off its control loop; nothing is reported back to the module.
//     ref_status       xgc.ref.status/1             -> multirotor_reference_trajectory_msgs/ReferenceStatus (latched)
//     ref_active_analytic   xgc.ref.analytic/1      -> .../AnalyticReference (latched)
//     ref_active_polynomial xgc.ref.polynomial/1    -> .../ActivePolynomialReference (latched)
//     ref_active_sampled    xgc.ref.sampled/1       -> .../SampledReference (latched)
//   (the ref_* outputs are latched, as the reference trajectory node's are)
//     formation_tick   xgc.dmpc.formation_tick/1    -> formation_generator/FormationTick
//   sync_trigger and formation_tick are local facades for the unchanged DMPC
//   planner: dmpc-rounds writes them on this robot's own round boundaries
//   (E0 + k*P on the aligned OS clock). Publish them on the robot's own topic;
//   they are never a timing authority for another robot.
//
// Threading: ROS callbacks run on this plugin's own queue. Each step first
// publishes what the modules wrote since the last step, then services that
// queue for a steady-time budget. The budget is fixed at step entry, so a
// paused Session clock cannot keep the step waiting. ros::init is once per
// process and is shared with the clock-source service (same node name and
// the process ROS master). A clock source that has not been started leaves
// the output gate unclaimed, which is the existing wall path.
//
// Config: `<port>_topic` (string), `node_name`, `frame_id` (default "world"),
// `queue_size` (default 10), `slice_ms` (default 0: service ROS until the
// round's deadline). With long rounds (DMPC, 100 ms) set `slice_ms` (e.g. 2)
// and the plugin's `wake_ms` to the same value: each step then services ROS
// for at most one slice, and module outputs (the planner's local
// FormationTick) reach ROS within about a slice instead of a round.

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <formation_generator/AssumedTrajectory.h>
#include <formation_generator/FormationTick.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <hover_thrust_estimator_msgs/HoverThrustEstimate.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <multirotor_reference_trajectory_msgs/ActivePolynomialReference.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/ReferenceStatus.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <multirotor_reference_trajectory_msgs/WaypointReferenceRequest.h>
#include <periodic_sync/SyncTrigger.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include "../common/flat_config.hpp"
#include "../common/reference_wire.hpp"
#include "ros_edge.hpp"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

#include <chrono>

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
  kFcuState,
  kLocalPose,
  kLocalVelocity,
  kFcuImu,
  kBattery,
  kCommand,
  kAlgSetpoint,
  kSetpoint,
  kAttitudeRate,
  kStatus,
  kFcuRequest,
  kRefAnalytic,
  kRefWaypoint,
  kRefSampled,
  kRefReset,
  kRefStatus,
  kRefActiveAnalytic,
  kRefActivePolynomial,
  kRefActiveSampled,
  kHoverThrust,
  kControllerState,
  kFormationTick,
  kPortCount
};

const char* const kPortNames[kPortCount] = {
    "imu",       "pose",       "attitude_target", "own_plan", "vision_pose", "neighbor_plans", "sync_trigger",
    "rigid_state_estimate", "fcu_state", "local_pose", "local_velocity", "fcu_imu", "battery", "command",
    "alg_setpoint", "setpoint",  "attitude_rate", "status", "fcu_request", "ref_analytic", "ref_waypoint",
    "ref_sampled", "ref_reset", "ref_status", "ref_active_analytic", "ref_active_polynomial", "ref_active_sampled",
    "hover_thrust", "controller_state", "formation_tick"};

double stamp_or_now(const ros::Time& t) { return (t.isZero() ? ros::Time::now() : t).toSec(); }

void quat(double* wxyz, const geometry_msgs::Quaternion& q) {
  wxyz[0] = q.w;
  wxyz[1] = q.x;
  wxyz[2] = q.y;
  wxyz[3] = q.z;
}

void vec3(double* out, const geometry_msgs::Vector3& v) {
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
}

// Copies a std::string into a fixed char field, always NUL-terminated.
template <size_t N>
void text(char (&out)[N], const std::string& in) {
  const size_t n = std::min(in.size(), N - 1);
  std::memcpy(out, in.data(), n);
  out[n] = '\0';
}

template <size_t N>
std::string text(const char (&in)[N]) {
  return std::string(in, strnlen(in, N));
}

struct RosIo {
  const xgc_host_api* host;
  std::string topics[kPortCount];
  std::string node_name{"xgc_ros_io"};
  std::string frame_id{"world"};
  int queue_size{10};
  double slice_ms{0.0};
  ros::CallbackQueue queue;
  std::unique_ptr<ros::NodeHandle> nh;
  std::vector<ros::Subscriber> subs;
  ros::Publisher pubs[kPortCount];
  uint64_t round{0};
  uint64_t from_ros{0};
  uint64_t to_ros{0};
  bool write_failed{false};

  // fcu_request: service calls run in order on `caller`; results come back as
  // log lines, logged from the plugin thread.
  std::thread caller;
  std::mutex calls_mutex;
  std::condition_variable calls_cv;
  std::deque<xgc_fcu_request_v1> calls;
  std::vector<std::string> call_log;
  bool calls_stop{false};

  ~RosIo() { stop_calls(); }

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

  void on_fcu_state(const mavros_msgs::State::ConstPtr& m) {
    xgc_fcu_state_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.connected = m->connected ? 1u : 0u;
    s.armed = m->armed ? 1u : 0u;
    s.guided = m->guided ? 1u : 0u;
    s.manual_input = m->manual_input ? 1u : 0u;
    s.system_status = m->system_status;
    text(s.mode, m->mode);
    write(kFcuState, s);
  }

  void on_local_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    xgc_pose_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.position[0] = m->pose.position.x;
    s.position[1] = m->pose.position.y;
    s.position[2] = m->pose.position.z;
    quat(s.q_wxyz, m->pose.orientation);
    write(kLocalPose, s);
  }

  void on_local_velocity(const geometry_msgs::TwistStamped::ConstPtr& m) {
    xgc_twist_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    vec3(s.linear, m->twist.linear);
    vec3(s.angular, m->twist.angular);
    write(kLocalVelocity, s);
  }

  void on_fcu_imu(const sensor_msgs::Imu::ConstPtr& m) {
    xgc_imu_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    vec3(s.accel, m->linear_acceleration);
    vec3(s.gyro, m->angular_velocity);
    write(kFcuImu, s);
  }

  void on_battery(const sensor_msgs::BatteryState::ConstPtr& m) {
    xgc_battery_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.voltage = m->voltage;
    s.percentage = m->percentage;
    write(kBattery, s);
  }

  void on_command(const std_msgs::String::ConstPtr& m) {
    xgc_command_v1 s{};
    text(s.text, m->data);
    write(kCommand, s);
  }

  void on_alg_setpoint(const mavros_msgs::PositionTarget::ConstPtr& m) {
    xgc_position_target_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.position[0] = m->position.x;
    s.position[1] = m->position.y;
    s.position[2] = m->position.z;
    s.velocity[0] = m->velocity.x;
    s.velocity[1] = m->velocity.y;
    s.velocity[2] = m->velocity.z;
    s.acceleration[0] = m->acceleration_or_force.x;
    s.acceleration[1] = m->acceleration_or_force.y;
    s.acceleration[2] = m->acceleration_or_force.z;
    s.yaw = m->yaw;
    s.yaw_rate = m->yaw_rate;
    s.type_mask = m->type_mask;
    s.coordinate_frame = m->coordinate_frame;
    write(kAlgSetpoint, s);
  }

  void on_ref_analytic(const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& m) {
    const auto bytes = xgc_ref_wire::encode_analytic(*m);
    write_bytes(kRefAnalytic, bytes.data(), bytes.size());
  }

  void on_ref_waypoint(const multirotor_reference_trajectory_msgs::WaypointReferenceRequest::ConstPtr& m) {
    const auto bytes = xgc_ref_wire::encode_waypoint_request(*m);
    write_bytes(kRefWaypoint, bytes.data(), bytes.size());
  }

  void on_ref_sampled(const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& m) {
    const auto bytes = xgc_ref_wire::encode_sampled(*m);
    write_bytes(kRefSampled, bytes.data(), bytes.size());
  }

  void on_ref_reset(const std_msgs::Empty::ConstPtr&) { write(kRefReset, xgc_ref_reset_v1{}); }

  void on_controller_state(const std_msgs::String::ConstPtr& m) {
    xgc_controller_status_v1 s{};
    s.stamp = ros::Time::now().toSec();
    text(s.state, m->data);
    write(kControllerState, s);
  }

  void on_hover_thrust(const hover_thrust_estimator_msgs::HoverThrustEstimate::ConstPtr& m) {
    xgc_hover_thrust_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.hover_thrust = m->hover_thrust;
    s.state = m->state;
    s.flags = m->flags;
    write(kHoverThrust, s);
  }

  // --- fcu_request service calls -------------------------------------------

  void call_loop() {
    const std::string ns = topics[kFcuRequest];
    ros::ServiceClient command = nh->serviceClient<mavros_msgs::CommandLong>(ns + "/cmd/command", true);
    ros::ServiceClient set_mode = nh->serviceClient<mavros_msgs::SetMode>(ns + "/set_mode", true);
    for (;;) {
      xgc_fcu_request_v1 r;
      {
        std::unique_lock<std::mutex> lock(calls_mutex);
        calls_cv.wait(lock, [&] { return calls_stop || !calls.empty(); });
        if (calls_stop) return;
        r = calls.front();
        calls.pop_front();
      }
      std::string result;
      if (!xgc_ros_edge::output_allowed()) {
        result = "fcu_request discarded while the output gate is closed";
      } else if (r.kind == 1) {
        // A persistent client drops its connection on a failed call; reconnect.
        if (!command.isValid()) command = nh->serviceClient<mavros_msgs::CommandLong>(ns + "/cmd/command", true);
        mavros_msgs::CommandLong srv;
        srv.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
        srv.request.param1 = r.arm ? 1.0 : 0.0;
        const bool ok = command.call(srv);
        result = std::string(r.arm ? "arm" : "disarm") + (ok ? (srv.response.success ? ": success" : ": refused") : ": call failed");
      } else if (r.kind == 2) {
        if (!set_mode.isValid()) set_mode = nh->serviceClient<mavros_msgs::SetMode>(ns + "/set_mode", true);
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = text(r.mode);
        const bool ok = set_mode.call(srv);
        result = "set_mode " + srv.request.custom_mode + (ok ? (srv.response.mode_sent ? ": sent" : ": refused") : ": call failed");
      } else {
        result = "unknown fcu_request kind " + std::to_string(r.kind) + " dropped";
      }
      std::lock_guard<std::mutex> lock(calls_mutex);
      call_log.push_back("ros_io: " + result);
    }
  }

  void stop_calls() {
    if (!caller.joinable()) return;
    {
      std::lock_guard<std::mutex> lock(calls_mutex);
      calls_stop = true;
    }
    calls_cv.notify_all();
    caller.join();
  }

  void discard_pending_output() {
    xgc_sample_view v;
    for (uint32_t port = 0; port < kPortCount; ++port) {
      while (host->next(host->host, port, &v) == XGC_OK) {
      }
    }
    std::lock_guard<std::mutex> lock(calls_mutex);
    calls.clear();
  }

  // --- module inputs -> ROS -------------------------------------------------

  void forward_inputs() {
    if (!xgc_ros_edge::output_allowed()) {
      discard_pending_output();
      return;
    }
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
      // Bound allocation by the actual received bytes. A uint32-by-uint32
      // product fits uint64_t; its byte count need not fit size_t.
      if ((v.len - sizeof h) % 8 != 0) continue;
      const uint64_t state_count = static_cast<uint64_t>(h.num_states) * h.num_timesteps;
      const size_t elements = (v.len - sizeof h) / 8;
      if (state_count > elements || h.rest_len != elements - state_count) continue;
      const size_t states = static_cast<size_t>(state_count);
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
    while (host->next(host->host, kFormationTick, &v) == XGC_OK) {
      xgc_dmpc_formation_tick_v1 f;
      xgc_dmpc_sync_trigger_v1 h;
      if (v.len < sizeof f + sizeof h) continue;
      std::memcpy(&f, v.data, sizeof f);
      std::memcpy(&h, v.data + sizeof f, sizeof h);
      if (v.len != sizeof f + sizeof h + 4 * static_cast<size_t>(h.count)) continue;
      formation_generator::FormationTick m;
      m.trigger.sequence_id = h.sequence_id;
      m.trigger.trigger_time.fromSec(h.trigger_time);
      m.trigger.published_time = ros::Time::now();
      m.trigger.active_participant_ids.resize(h.count);
      std::memcpy(m.trigger.active_participant_ids.data(), v.data + sizeof f + sizeof h, 4 * static_cast<size_t>(h.count));
      m.rolling = f.rolling != 0;
      m.mission_time = f.mission_time;
      pubs[kFormationTick].publish(m);
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
    while (host->next(host->host, kSetpoint, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_position_target_v1)) continue;
      xgc_position_target_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      mavros_msgs::PositionTarget m;
      m.header.stamp.fromSec(s.stamp);
      m.header.frame_id = "map";
      m.coordinate_frame = s.coordinate_frame;
      m.type_mask = s.type_mask;
      m.position.x = s.position[0];
      m.position.y = s.position[1];
      m.position.z = s.position[2];
      m.velocity.x = s.velocity[0];
      m.velocity.y = s.velocity[1];
      m.velocity.z = s.velocity[2];
      m.acceleration_or_force.x = s.acceleration[0];
      m.acceleration_or_force.y = s.acceleration[1];
      m.acceleration_or_force.z = s.acceleration[2];
      m.yaw = static_cast<float>(s.yaw);
      m.yaw_rate = static_cast<float>(s.yaw_rate);
      pubs[kSetpoint].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kAttitudeRate, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_body_rate_thrust_v1)) continue;
      xgc_body_rate_thrust_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      mavros_msgs::AttitudeTarget m;
      m.header.stamp.fromSec(s.stamp);
      m.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
      m.orientation.w = 1.0;
      m.body_rate.x = s.body_rate[0];
      m.body_rate.y = s.body_rate[1];
      m.body_rate.z = s.body_rate[2];
      m.thrust = static_cast<float>(std::min(1.0, std::max(0.0, s.thrust)));
      pubs[kAttitudeRate].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kStatus, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_controller_status_v1)) continue;
      xgc_controller_status_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      std_msgs::String m;
      m.data = text(s.state);
      pubs[kStatus].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kFcuRequest, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_fcu_request_v1)) continue;
      xgc_fcu_request_v1 r;
      std::memcpy(&r, v.data, sizeof r);
      {
        std::lock_guard<std::mutex> lock(calls_mutex);
        calls.push_back(r);
      }
      calls_cv.notify_one();
      ++to_ros;
    }
    while (host->next(host->host, kRefStatus, &v) == XGC_OK) {
      multirotor_reference_trajectory_msgs::ReferenceStatus m;
      if (!xgc_ref_wire::decode_status(v.data, v.len, m)) continue;
      pubs[kRefStatus].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kRefActiveAnalytic, &v) == XGC_OK) {
      multirotor_reference_trajectory_msgs::AnalyticReference m;
      if (!xgc_ref_wire::decode_analytic(v.data, v.len, m)) continue;
      pubs[kRefActiveAnalytic].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kRefActivePolynomial, &v) == XGC_OK) {
      multirotor_reference_trajectory_msgs::ActivePolynomialReference m;
      if (!xgc_ref_wire::decode_polynomial(v.data, v.len, m)) continue;
      pubs[kRefActivePolynomial].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kRefActiveSampled, &v) == XGC_OK) {
      multirotor_reference_trajectory_msgs::SampledReference m;
      if (!xgc_ref_wire::decode_sampled(v.data, v.len, m)) continue;
      pubs[kRefActiveSampled].publish(m);
      ++to_ros;
    }
    std::vector<std::string> lines;
    {
      std::lock_guard<std::mutex> lock(calls_mutex);
      lines.swap(call_log);
    }
    for (const auto& l : lines) log(XGC_LOG_INFO, l);
  }

  xgc_status activate() {
    const xgc_ros_edge::RosInit ros_init = xgc_ros_edge::ensure_ros(node_name);
    if (!ros_init.ok) {
      log(XGC_LOG_ERROR, std::string("ros_io: ") + ros_init.error);
      return XGC_ERR;
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
    if (enabled(kFcuState)) subs.push_back(nh->subscribe(topics[kFcuState], queue_size, &RosIo::on_fcu_state, this));
    if (enabled(kLocalPose)) subs.push_back(nh->subscribe(topics[kLocalPose], queue_size, &RosIo::on_local_pose, this));
    if (enabled(kLocalVelocity))
      subs.push_back(nh->subscribe(topics[kLocalVelocity], queue_size, &RosIo::on_local_velocity, this));
    if (enabled(kFcuImu)) subs.push_back(nh->subscribe(topics[kFcuImu], queue_size, &RosIo::on_fcu_imu, this));
    if (enabled(kBattery)) subs.push_back(nh->subscribe(topics[kBattery], queue_size, &RosIo::on_battery, this));
    if (enabled(kCommand)) subs.push_back(nh->subscribe(topics[kCommand], queue_size, &RosIo::on_command, this));
    if (enabled(kAlgSetpoint))
      subs.push_back(nh->subscribe(topics[kAlgSetpoint], queue_size, &RosIo::on_alg_setpoint, this));
    if (enabled(kSetpoint)) pubs[kSetpoint] = nh->advertise<mavros_msgs::PositionTarget>(topics[kSetpoint], queue_size);
    if (enabled(kAttitudeRate))
      pubs[kAttitudeRate] = nh->advertise<mavros_msgs::AttitudeTarget>(topics[kAttitudeRate], queue_size);
    if (enabled(kStatus)) pubs[kStatus] = nh->advertise<std_msgs::String>(topics[kStatus], queue_size);
    if (enabled(kFcuRequest)) caller = std::thread([this] { call_loop(); });
    if (enabled(kRefAnalytic))
      subs.push_back(nh->subscribe(topics[kRefAnalytic], queue_size, &RosIo::on_ref_analytic, this));
    if (enabled(kRefWaypoint))
      subs.push_back(nh->subscribe(topics[kRefWaypoint], queue_size, &RosIo::on_ref_waypoint, this));
    if (enabled(kRefSampled))
      subs.push_back(nh->subscribe(topics[kRefSampled], queue_size, &RosIo::on_ref_sampled, this));
    if (enabled(kControllerState))
      subs.push_back(nh->subscribe(topics[kControllerState], queue_size, &RosIo::on_controller_state, this));
    if (enabled(kFormationTick))
      pubs[kFormationTick] = nh->advertise<formation_generator::FormationTick>(topics[kFormationTick], queue_size);
    if (enabled(kHoverThrust))
      subs.push_back(nh->subscribe(topics[kHoverThrust], queue_size, &RosIo::on_hover_thrust, this));
    if (enabled(kRefReset)) subs.push_back(nh->subscribe(topics[kRefReset], queue_size, &RosIo::on_ref_reset, this));
    namespace rmsg = multirotor_reference_trajectory_msgs;
    if (enabled(kRefStatus)) pubs[kRefStatus] = nh->advertise<rmsg::ReferenceStatus>(topics[kRefStatus], queue_size, true);
    if (enabled(kRefActiveAnalytic))
      pubs[kRefActiveAnalytic] = nh->advertise<rmsg::AnalyticReference>(topics[kRefActiveAnalytic], queue_size, true);
    if (enabled(kRefActivePolynomial))
      pubs[kRefActivePolynomial] =
          nh->advertise<rmsg::ActivePolynomialReference>(topics[kRefActivePolynomial], queue_size, true);
    if (enabled(kRefActiveSampled))
      pubs[kRefActiveSampled] = nh->advertise<rmsg::SampledReference>(topics[kRefActiveSampled], queue_size, true);
    return XGC_OK;
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    round = ctx->round;
    if (!xgc_ros_edge::output_allowed() || xgc_ros_edge::take_suppress_backlog()) {
      discard_pending_output();
    } else {
      forward_inputs();
    }
    // One non-blocking pass, then a budget fixed in steady time. Session time
    // is not read again: a frozen simulation clock must not extend the wait.
    queue.callAvailable(ros::WallDuration(0));
    const int64_t session_now = host->now(host->host);
    int64_t until = ctx->deadline - 1000000;
    if (slice_ms > 0.0) {
      until = std::min<int64_t>(until, session_now + static_cast<int64_t>(slice_ms * 1e6));
    }
    int64_t budget_ns = until - session_now;
    if (budget_ns < 0) budget_ns = 0;
    const auto started = std::chrono::steady_clock::now();
    while (ros::ok()) {
      const int64_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
      if (elapsed >= budget_ns) break;
      const int64_t left = budget_ns - elapsed;
      queue.callAvailable(ros::WallDuration(std::min<int64_t>(left, 1000000) * 1e-9));
    }
    if (write_failed) {
      log(XGC_LOG_ERROR, "ros_io: a module output write failed");
      return XGC_ERR;
    }
    return XGC_OK;
  }

  void shutdown() {
    stop_calls();
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
    if (!cfg::number(t, "slice_ms", &self->slice_ms) || self->slice_ms < 0.0) {
      self->log(XGC_LOG_ERROR, "ros_io: invalid slice_ms");
      return XGC_ERR;
    }
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
    {"fcu_state", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_state/1", XGC_QOS_STATE},
    {"local_pose", XGC_PORT_OUT_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"local_velocity", XGC_PORT_OUT_OPTIONAL, "xgc.twist/1", XGC_QOS_STATE},
    {"fcu_imu", XGC_PORT_OUT_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"battery", XGC_PORT_OUT_OPTIONAL, "xgc.battery/1", XGC_QOS_STATE},
    {"command", XGC_PORT_OUT_OPTIONAL, "xgc.command/1", XGC_QOS_EVENT},
    {"alg_setpoint", XGC_PORT_OUT_OPTIONAL, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"setpoint", XGC_PORT_IN_OPTIONAL, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"attitude_rate", XGC_PORT_IN_OPTIONAL, "xgc.body_rate_thrust/1", XGC_QOS_CONTROL},
    {"status", XGC_PORT_IN_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE},
    {"fcu_request", XGC_PORT_IN_OPTIONAL, "xgc.fcu_request/1", XGC_QOS_EVENT},
    {"ref_analytic", XGC_PORT_OUT_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_EVENT},
    {"ref_waypoint", XGC_PORT_OUT_OPTIONAL, "xgc.ref.waypoint_request/1", XGC_QOS_EVENT},
    {"ref_sampled", XGC_PORT_OUT_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_EVENT},
    {"ref_reset", XGC_PORT_OUT_OPTIONAL, "xgc.ref.reset/1", XGC_QOS_EVENT},
    {"ref_status", XGC_PORT_IN_OPTIONAL, "xgc.ref.status/1", XGC_QOS_STATE},
    {"ref_active_analytic", XGC_PORT_IN_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_STATE},
    {"ref_active_polynomial", XGC_PORT_IN_OPTIONAL, "xgc.ref.polynomial/1", XGC_QOS_STATE},
    {"ref_active_sampled", XGC_PORT_IN_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_STATE},
    {"hover_thrust", XGC_PORT_OUT_OPTIONAL, "xgc.hover_thrust/1", XGC_QOS_STATE},
    {"controller_state", XGC_PORT_OUT_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE},
    {"formation_tick", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.formation_tick/1", XGC_QOS_CONTROL},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {
    XGC_RT_ABI_VERSION, kPortCount, "ros-io", "0.1.0", kPorts, &kVtbl,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
