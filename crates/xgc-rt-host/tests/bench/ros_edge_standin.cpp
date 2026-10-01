// ROS-free stand-in for plugins/ros-io, used only by the lightweight plant
// hosting benchmark (tests/lightweight_bench.rs, shape `platform`).
//
// It declares ros_io's 41 ports (names, directions, schemas, QoS) and repeats
// ros_io's host-facing work in every step:
//   - forward_inputs(): `next` on every module input port in ros_io's order,
//     copying each sample out as ros_io does before it builds a ROS message,
//     and the same-stamp odometry pairing (sim_odometry.hpp);
//   - the ROS service of the step, with ros_io's arithmetic (ros_slice.hpp):
//     one non-blocking pass, then condition-variable waits for what remains
//     of the slice, as ros::CallbackQueue::callAvailable(timeout) waits when
//     its queue is empty (a stand-in queue that never receives a callback).
// Nothing reaches ROS, so roscpp's serialization and socket writes are not
// part of the measured cost.
//
// Config is what the Core manifest renders for ros_io (`<port>_topic`,
// `slice_ms`, `node_name`, `frame_id`) plus bench-only keys:
//   standin_commands = true  a commanded edge stands in for the controller's
//                            ROS input: one setpoint, arm and OFFBOARD, then
//                            a 50 Hz acceleration setpoint stream.
//   standin_slice = "legacy" ros_io's service loop before the timer-slack
//                            rule: it waited out any remainder of the slice.
//
// Build: $CXX -std=c++17 -O2 -fPIC -shared -I abi/include -I plugins/common \
//          -I plugins/ros-io ros_edge_standin.cpp -o libros_edge_standin.so

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <string>

#include "flat_config.hpp"
#include "ros_slice.hpp"
#include "sim_odometry.hpp"
#include "xgc_dmpc_planner_v1.h"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {

namespace cfg = xgc_rt_config;

// ros_io's port enumeration, in ros_io's order.
enum Port : uint32_t {
  kImu = 0, kPose, kAttitudeTarget, kOwnPlan, kVisionPose, kNeighborPlans, kSyncTrigger,
  kRigidStateEstimate, kFcuState, kLocalPose, kLocalVelocity, kFcuImu, kBattery, kCommand,
  kAlgSetpoint, kSetpoint, kAttitudeRate, kStatus, kFcuRequest, kRefAnalytic, kRefSampled,
  kRefReset, kRefStatus, kRefActiveAnalytic, kRefActiveSampled, kHoverThrust,
  kControllerState, kFormationTick, kPairedState, kSceneSnapshot, kSceneHeartbeat,
  kMissionRequest, kTimelineAck, kTimelineStatus, kPlanarPva, kSimPose, kSimVelocity,
  kSimImu, kSimFcuState, kCmdVel, kSimFcuRequest, kPortCount
};

// The module inputs ros_io's forward_inputs() polls, in its order.
constexpr Port kForwarded[] = {
    kVisionPose, kSimPose, kSimVelocity, kSimImu, kSimFcuState, kNeighborPlans, kSyncTrigger,
    kFormationTick, kPlanarPva, kRigidStateEstimate, kSetpoint, kAttitudeRate, kStatus,
    kFcuRequest, kRefStatus, kRefActiveAnalytic, kRefActiveSampled, kTimelineAck,
    kTimelineStatus};

// Stand-in for ros::CallbackQueue: callAvailable(timeout) locks the queue
// and, when it is empty and the timeout is not zero, waits on its condition
// variable for that long. Nothing is ever queued here.
struct CallbackQueue {
  std::mutex mutex;
  std::condition_variable condition;
  void callAvailable(int64_t timeout_ns) {
    std::unique_lock<std::mutex> lock(mutex);
    if (timeout_ns > 0) condition.wait_for(lock, std::chrono::nanoseconds(timeout_ns));
  }
};

struct Edge {
  const xgc_host_api* host{};
  bool enabled[kPortCount]{};
  bool odometry{false};
  bool commands{false};
  bool legacy_slice{false};
  double slice_ms{0.0};
  CallbackQueue queue;
  uint64_t round{0};
  // forward_inputs() state, as in ros_io.
  xgc_pose_v1 sim_pose{};
  xgc_twist_v1 sim_velocity{};
  bool have_sim_pose{false}, have_sim_velocity{false};
  double last_odometry_stamp{0};
  uint64_t converted{0};
  double checksum{0};
  // Commanded edge.
  int64_t next_setpoint{0};
  bool armed_sent{false};

  template <typename T>
  bool take(const xgc_sample_view& v, T* out) {
    if (v.len != sizeof(T)) return false;
    std::memcpy(out, v.data, sizeof(T));
    ++converted;
    return true;
  }

  void forward_inputs() {
    xgc_sample_view v;
    for (Port port : kForwarded) {
      while (host->next(host->host, port, &v) == XGC_OK) {
        switch (port) {
          case kSimPose: {
            if (take(v, &sim_pose)) have_sim_pose = true, checksum += sim_pose.position[0];
            break;
          }
          case kSimVelocity: {
            if (take(v, &sim_velocity)) have_sim_velocity = true, checksum += sim_velocity.linear[0];
            break;
          }
          case kSimImu: {
            xgc_imu_v1 s;
            if (take(v, &s)) checksum += s.accel[2];
            break;
          }
          case kSimFcuState: {
            xgc_fcu_state_v1 s;
            if (take(v, &s)) checksum += s.armed + std::strlen(s.mode);
            break;
          }
          default:
            checksum += v.len;
            ++converted;
        }
      }
      // ros_io pairs odometry once the velocity samples are read.
      if (port == kSimVelocity && odometry && have_sim_pose && have_sim_velocity) {
        SimOdometry measured;
        if (measured_sim_odometry(sim_pose, sim_velocity, last_odometry_stamp, &measured)) {
          last_odometry_stamp = measured.pose.stamp;
          checksum += measured.linear[0];
        }
      }
    }
  }

  void write(Port port, const void* data, size_t len) {
    host->publish(host->host, port, round, static_cast<const uint8_t*>(data), static_cast<uint32_t>(len));
  }

  // What ros_io writes when the controller's ROS messages arrive.
  void command_inputs(const xgc_step_ctx* ctx) {
    if (!commands || ctx->now < next_setpoint) return;
    const double stamp = double(ctx->now) * 1e-9;
    const double phase = double(ctx->now) * 1e-9;
    xgc_position_target_v1 setpoint{};
    setpoint.stamp = stamp;
    setpoint.acceleration[0] = 0.1 * std::sin(phase);
    setpoint.acceleration[1] = 0.1 * std::cos(phase);
    setpoint.acceleration[2] = 0.05;
    setpoint.type_mask = 3135;  // acceleration only
    setpoint.coordinate_frame = 1;
    write(kAlgSetpoint, &setpoint, sizeof setpoint);
    if (!armed_sent) {
      armed_sent = true;
      xgc_fcu_request_v1 arm{};
      arm.stamp = stamp;
      arm.kind = 1;
      arm.arm = 1;
      write(kSimFcuRequest, &arm, sizeof arm);
      xgc_fcu_request_v1 offboard{};
      offboard.stamp = stamp;
      offboard.kind = 2;
      std::strcpy(offboard.mode, "OFFBOARD");
      write(kSimFcuRequest, &offboard, sizeof offboard);
    }
    next_setpoint = ctx->now + 20000000;  // 50 Hz
  }

  // ros_io's ROS service: one non-blocking pass, then the rest of the slice
  // budget in waits of at most 1 ms.
  void service(const xgc_step_ctx* ctx) {
    queue.callAvailable(0);
    const int64_t budget_ns = xgc_ros_slice::budget_ns(host->now(host->host), ctx->deadline, slice_ms);
    const auto started = std::chrono::steady_clock::now();
    for (;;) {
      const int64_t elapsed =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - started).count();
      const int64_t wait_ns = legacy_slice ? (elapsed >= budget_ns ? 0 : std::min<int64_t>(budget_ns - elapsed, 1000000))
                                           : xgc_ros_slice::next_wait_ns(budget_ns, elapsed);
      if (wait_ns == 0) break;
      queue.callAvailable(wait_ns);
    }
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    round = ctx->round;
    forward_inputs();
    command_inputs(ctx);
    service(ctx);
    return XGC_OK;
  }
};

const char* const kPortNames[kPortCount] = {
    "imu", "pose", "attitude_target", "own_plan", "vision_pose", "neighbor_plans", "sync_trigger",
    "rigid_state_estimate", "fcu_state", "local_pose", "local_velocity", "fcu_imu", "battery",
    "command", "alg_setpoint", "setpoint", "attitude_rate", "status", "fcu_request",
    "ref_analytic", "ref_sampled", "ref_reset", "ref_status", "ref_active_analytic",
    "ref_active_sampled", "hover_thrust", "controller_state", "formation_tick", "paired_state",
    "scene_snapshot", "scene_heartbeat", "mission_request", "timeline_ack", "timeline_status",
    "planar_pva", "sim_pose", "sim_velocity", "sim_imu", "sim_fcu_state", "cmd_vel",
    "sim_fcu_request"};

void* create(const xgc_host_api* host) {
  auto* self = new (std::nothrow) Edge{};
  if (self) self->host = host;
  return self;
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<Edge*>(p);
  const std::string text = config ? config : "";
  for (uint32_t i = 0; i < kPortCount; ++i)
    self->enabled[i] = !cfg::text_or(text, (std::string(kPortNames[i]) + "_topic").c_str(), "").empty();
  self->odometry = !cfg::text_or(text, "sim_odometry_topic", "").empty();
  if (!cfg::number(text, "slice_ms", &self->slice_ms) || self->slice_ms < 0.0) return XGC_ERR_INVALID;
  if (!cfg::boolean(text, "standin_commands", &self->commands)) return XGC_ERR_INVALID;
  self->legacy_slice = cfg::text_or(text, "standin_slice", "") == "legacy";
  return XGC_OK;
}

xgc_status activate(void*) { return XGC_OK; }
xgc_status step(void* p, const xgc_step_ctx* ctx) { return static_cast<Edge*>(p)->step(ctx); }
xgc_status deactivate(void*) { return XGC_OK; }
void destroy(void* p) { delete static_cast<Edge*>(p); }
const char* domain_state(void* p) { return static_cast<Edge*>(p)->converted ? "flowing" : "connected"; }

// ros_io's declarations, verbatim.
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
    {"ref_sampled", XGC_PORT_OUT_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_EVENT},
    {"ref_reset", XGC_PORT_OUT_OPTIONAL, "xgc.ref.reset/1", XGC_QOS_EVENT},
    {"ref_status", XGC_PORT_IN_OPTIONAL, "xgc.ref.status/1", XGC_QOS_STATE},
    {"ref_active_analytic", XGC_PORT_IN_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_STATE},
    {"ref_active_sampled", XGC_PORT_IN_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_STATE},
    {"hover_thrust", XGC_PORT_OUT_OPTIONAL, "xgc.hover_thrust/1", XGC_QOS_STATE},
    {"controller_state", XGC_PORT_OUT_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE},
    {"formation_tick", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.formation_tick/1", XGC_QOS_CONTROL},
    {"paired_state", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.paired_state/1", XGC_QOS_STATE},
    {"scene_snapshot", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.scene_snapshot/1", XGC_QOS_STATE},
    {"scene_heartbeat", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.scene_heartbeat/1", XGC_QOS_STATE},
    {"mission_request", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.mission_timeline/1", XGC_QOS_EVENT},
    {"timeline_ack", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.timeline_ack/1", XGC_QOS_EVENT},
    {"timeline_status", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.timeline_status/1", XGC_QOS_STATE},
    {"planar_pva", XGC_PORT_IN_OPTIONAL, "xgc.planar_pva/1", XGC_QOS_CONTROL},
    {"sim_pose", XGC_PORT_IN_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"sim_velocity", XGC_PORT_IN_OPTIONAL, "xgc.twist/1", XGC_QOS_STATE},
    {"sim_imu", XGC_PORT_IN_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"sim_fcu_state", XGC_PORT_IN_OPTIONAL, "xgc.fcu_state/1", XGC_QOS_STATE},
    {"cmd_vel", XGC_PORT_OUT_OPTIONAL, "xgc.twist/1", XGC_QOS_CONTROL},
    {"sim_fcu_request", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_request/1", XGC_QOS_EVENT},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {
    XGC_RT_ABI_VERSION, kPortCount, "ros-io", "0.1.0-standin", kPorts, &kVtbl,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
