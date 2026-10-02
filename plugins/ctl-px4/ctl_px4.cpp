// ctl-px4: the PX4 multirotor controller as an aggregator module.
//
// Runs px4_multirotor_controller_core, the ROS-free core of
// xgc2-multirotor-controller (DroneController, its state machines, tracking
// strategies), behind module I/O. Everything the ROS node's input producers
// and output consumers did becomes ports; ros_io does the MAVROS side.
//
//   in  estimate        xgc.rigid_state_estimate/1  (optional; DFBC/NMPC bind it. SMC does not)
//   in  local_pose      xgc.pose/1                  (mavros local position; PX4_LOCAL and SMC feedback)
//   in  local_velocity  xgc.twist/1                 (mavros velocity_local; PX4_LOCAL and SMC feedback)
//   in  imu             xgc.imu/1                   (mavros imu/data, event only)
//   in  fcu_state       xgc.fcu_state/1             (mavros state)
//   in  battery         xgc.battery/1               (optional; telemetry)
//   in  vrpn_pose       xgc.pose/1                  (canonical pose, consistency check)
//   in  command         xgc.command/1               (optional; takeoff/land/hover/custom1)
//   in  clock           xgc.clock/1                 (optional; replay: advance time)
//   in  alg_setpoint    xgc.position_target/1       (optional; planner setpoint, alg/setpoint_raw/local)
//   in  hover_thrust    xgc.hover_thrust/1          (optional; hover_thrust/estimate_state)
//   in  ref_active_analytic   xgc.ref.analytic/1    (optional; the reference generator's active
//   in  ref_active_sampled    xgc.ref.sampled/1      Custom1 backends)
//   out ref_request     xgc.ref.analytic/1          (optional; the reference Custom1 activates,
//                                                    to the reference generator's analytic input)
//   out setpoint        xgc.position_target/1       (mavros setpoint_raw/local)
//   out attitude_rate   xgc.body_rate_thrust/1      (optional; setpoint_raw/attitude)
//   out fcu_request     xgc.fcu_request/1           (optional; arming, set_mode)
//   out status          xgc.controller_status/1     (optional; control state name)
//   out trace           xgc.text/1                  (optional; replay trace lines)
//   out tick_done       xgc.clock/1                 (optional; time_source "input": each
//                                                    controller tick's time, after its outputs,
//                                                    so a simulator can run in lockstep)
//
// Every input sample's receive time is its envelope t_produce. The module
// fills SensorData and posts the input events exactly as the node's input
// producers did (same fields, same event ids and sources, stamped with the
// receive time), keeps the topic stats the way ros1_utils::TopicStatsManager
// did (is_active/is_new on arrival; a 0.1 s timer with a 2.5 s timeout), and
// updates the controller the way the node's 1 kHz loop did.
//
// time_source:
//   "session" (default)  one controller update per round at Session time;
//                        run the aggregator at 1 ms rounds, like the node.
//   "input"              replay: ticks every 1 ms of input time from the first
//                        sample, and a tick runs only once every sample up to
//                        it has arrived (a later sample, or the clock port,
//                        proves it). This reproduces the controller's replay
//                        harness (test/replay/replay_harness.cpp in
//                        xgc2-multirotor-controller) exactly; with trace = true
//                        the trace port carries the harness's output lines.
//
// NMPC (tracking_backend "nmpc"): each REQUEST_NMPC_SOLVE is solved inline,
// as the node's NmpcOutputConsumer worker solves it; the result arrives at
// the next update, as if the worker had finished at once. The module's
// thread is its only one, and runs are deterministic.
//
// Config: time_source, trace, takeoff_altitude, tracking_backend
// ("px4_local" | "dfbc" | "nmpc" | "smc"), px4_local_lift
// ("legacy" | "zero_order_hold"), smc_k1, smc_k2, smc_boundary_layer,
// local_type_mask, skip_takeoff_init_disarm, planning_period,
// enable_yaw_control, reference_analytic_type. Defaults are
// config/uav_nmpc.yaml's (the TRO configuration). Absent keys keep those
// defaults: legacy lift, and the manuscript SMC gains.

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <exception>
#include <limits>
#include <map>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "flat_config.hpp"
#include "px4_multirotor_controller/common/time.h"
#include "px4_multirotor_controller/common/types.h"
#include "px4_multirotor_controller/control/trajectory_lifter.h"
#include "px4_multirotor_controller/drone_controller.h"
#include "px4_multirotor_controller/nmpc/nmpc_math_utils.h"
#include "px4_multirotor_controller/nmpc/uav_nmpc_solver.h"
#include "px4_multirotor_controller/uav/nmpc_tracking_backend.h"
#include "px4_multirotor_controller/uav/reference_activation.h"
#include "reference_wire.hpp"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {

namespace pmc = px4_multirotor_controller;
namespace sm = state_machine;

enum Port : uint32_t {
  kEstimate, kLocalPose, kLocalVelocity, kImu, kFcuState, kBattery, kVrpnPose, kCommand, kClock,
  kSetpoint, kAttitudeRate, kFcuRequest, kStatus, kTrace, kAlgSetpoint, kHoverThrust,
  kRefActiveAnalytic, kRefActiveSampled, kRefRequest, kTickDone, kAttitudeCommand, kFcuRequestFull, kPortCount
};
constexpr uint32_t kFirstStatsPort = kEstimate;
constexpr uint32_t kStatsPorts = 7;  // estimate .. vrpn_pose, in port order

struct Input {
  uint64_t t_ns;
  uint32_t port;
  std::vector<uint8_t> data;
};

struct Track {
  pmc::SensorData::TopicStats* stats{nullptr};
  std::deque<double> times;
};

const std::map<std::string, sm::EventId> kCommands = {
    {"takeoff", pmc::event_type::TAKEOFF_REQUESTED}, {"Takeoff", pmc::event_type::TAKEOFF_REQUESTED},
    {"TAKEOFF", pmc::event_type::TAKEOFF_REQUESTED}, {"land", pmc::event_type::LANDING_REQUESTED},
    {"Land", pmc::event_type::LANDING_REQUESTED},    {"LAND", pmc::event_type::LANDING_REQUESTED},
    {"hover", pmc::event_type::HOVER_REQUESTED},      {"Hover", pmc::event_type::HOVER_REQUESTED},
    {"HOVER", pmc::event_type::HOVER_REQUESTED},      {"custom1", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"Custom1", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED}, {"CUSTOM1", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"start", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},   {"Start", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"START", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},   {"track", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
    {"Track", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},   {"TRACK", pmc::event_type::TRAJECTORY_TRACKING_REQUESTED},
};

uint64_t bits(double v) {
  uint64_t b;
  std::memcpy(&b, &v, sizeof b);
  return b;
}

template <typename T>
bool read(const std::vector<uint8_t>& d, T* out) {
  if (d.size() != sizeof(T)) return false;
  std::memcpy(out, d.data(), sizeof(T));
  return true;
}

std::string bounded(const char* s, size_t n) { return std::string(s, strnlen(s, n)); }

struct CtlPx4 {
  uint64_t fcu_request_sequence{0};
  const xgc_host_api* host{nullptr};
  pmc::SensorData sensor;
  pmc::DroneController controller{sensor};
  Track tracks[kStatsPorts];
  bool input_time{false};
  bool trace{false};

  // Clock state (both modes).
  bool started{false};
  double t0{0.0};
  double next_stats{0.0};
  uint64_t k{0};
  // Replay state.
  std::vector<Input> pending;
  uint64_t latest_ns{0};
  double clock_limit{-std::numeric_limits<double>::infinity()};

  std::string last_state;
  std::string domain{"idle"};
  std::string line;

  pmc::ReferenceActivation activation;       // ReferenceActivationOutputConsumer
  pmc::UavNmpcTrackingBackend nmpc_backend;  // NmpcOutputConsumer
  bool nmpc_entered{false};

  CtlPx4() {
    // config/uav_nmpc.yaml: the values that differ from ControllerConfig's
    // defaults (world_boundary_json null).
    pmc::ControllerConfig c = controller.getConfig();
    c.takeoff_altitude = 2.3;
    c.local_type_mask = 3072;
    c.dfbc.acceleration_correction_enabled = true;
    c.nmpc.hover_thrust_enabled = true;
    c.safety.position_jump_threshold = 100.0;
    c.safety.max_velocity_xy = 1000.0;
    c.safety.max_velocity_z = 1000.0;
    c.safety.acc_saturation_xy = 1000.0;
    c.safety.acc_saturation_z = 1000.0;
    controller.setConfig(c);
    pmc::SensorData::TopicStats* stats[kStatsPorts] = {
        &sensor.uav_state_estimate_stats, &sensor.local_pos_stats, &sensor.local_velocity_stats,
        &sensor.imu_stats, &sensor.state_stats, &sensor.battery_stats, &sensor.vrpn_pose_stats};
    for (uint32_t i = 0; i < kStatsPorts; ++i) tracks[i].stats = stats[i];
  }

  void log(xgc_log_level level, const std::string& m) const { host->log(host->host, level, m.c_str()); }

  void post(sm::EventId id, double now, const char* source) {
    sm::Event event(id, sm::EventTimestamp{now});
    event.source = source;
    controller.getStateMachine().postEvent(std::move(event));
  }

  // ros1_utils::TopicStatsManager's 0.1 s timer, reduced to what the core reads.
  void runStatsTimer(double until) {
    for (; next_stats <= until; next_stats += 0.1) {
      for (auto& track : tracks) {
        if (track.times.size() >= 2) track.stats->is_active = (next_stats - track.times.back()) <= 2.5;
      }
    }
  }

  void start(double first_seconds) {
    t0 = std::floor(first_seconds * 1000.0) / 1000.0;
    next_stats = t0 + 0.1;
    started = true;
  }

  // One input, exactly as the node's input producer for that topic.
  void apply(const Input& in) {
    const double now = in.t_ns * 1e-9;
    runStatsTimer(now);
    if (in.port < kFirstStatsPort + kStatsPorts) {
      Track& track = tracks[in.port - kFirstStatsPort];
      track.times.push_back(now);
      if (track.times.size() > 10) track.times.pop_front();
      track.stats->last_message_time = pmc::Time().fromNSec(in.t_ns);
      track.stats->is_active = true;
      track.stats->is_new = true;
    }
    switch (in.port) {
      case kEstimate: {
        xgc_rigid_state_estimate_v1 m;
        if (!read(in.data, &m)) break;
        sensor.x = m.position[0]; sensor.y = m.position[1]; sensor.z = m.position[2];
        sensor.vx = m.velocity[0]; sensor.vy = m.velocity[1]; sensor.vz = m.velocity[2];
        sensor.qw = m.q_wxyz[0]; sensor.qx = m.q_wxyz[1]; sensor.qy = m.q_wxyz[2]; sensor.qz = m.q_wxyz[3];
        sensor.wx = m.angular_velocity[0]; sensor.wy = m.angular_velocity[1]; sensor.wz = m.angular_velocity[2];
        sensor.ax = m.linear_acceleration[0]; sensor.ay = m.linear_acceleration[1]; sensor.az = m.linear_acceleration[2];
        sensor.gx = m.gravity[0]; sensor.gy = m.gravity[1]; sensor.gz = m.gravity[2];
        sensor.accel_bias_x = m.accel_bias[0]; sensor.accel_bias_y = m.accel_bias[1]; sensor.accel_bias_z = m.accel_bias[2];
        sensor.uav_state_estimator_state = m.estimator_state;
        sensor.uav_state_estimator_flags = m.flags;
        sensor.uav_state_estimate_stamp = m.stamp;
        sensor.uav_state_filter_inertial_stamp = m.filter_inertial_stamp_sec;
        sensor.uav_state_filter_pose_stamp = m.filter_pose_stamp_sec;
        sensor.uav_state_last_vrpn_pose_stamp = m.last_vrpn_pose_stamp_sec;
        post(pmc::event_type::INPUT_UAV_STATE_ESTIMATE_UPDATED, now, "alg/state_estimator/state");
        break;
      }
      case kLocalPose: {
        xgc_pose_v1 m;
        if (!read(in.data, &m)) break;
        sensor.local_x = m.position[0]; sensor.local_y = m.position[1]; sensor.local_z = m.position[2];
        sensor.local_qw = m.q_wxyz[0]; sensor.local_qx = m.q_wxyz[1];
        sensor.local_qy = m.q_wxyz[2]; sensor.local_qz = m.q_wxyz[3];
        post(pmc::event_type::INPUT_LOCAL_POSITION_UPDATED, now, "mavros/local_position/pose");
        break;
      }
      case kLocalVelocity: {
        xgc_twist_v1 m;
        if (!read(in.data, &m)) break;
        sensor.local_vx = m.linear[0]; sensor.local_vy = m.linear[1]; sensor.local_vz = m.linear[2];
        post(pmc::event_type::INPUT_LOCAL_VELOCITY_UPDATED, now, "mavros/local_position/velocity_local");
        break;
      }
      case kImu:
        post(pmc::event_type::INPUT_IMU_UPDATED, now, "mavros/imu/data");
        break;
      case kFcuState: {
        xgc_fcu_state_v1 m;
        if (!read(in.data, &m)) break;
        sensor.fcu_connected = m.connected != 0; sensor.fcu_armed = m.armed != 0;
        sensor.fcu_guided = m.guided != 0; sensor.fcu_manual_input = m.manual_input != 0;
        sensor.fcu_mode = bounded(m.mode, sizeof m.mode);
        sensor.fcu_system_status = m.system_status;
        post(pmc::event_type::INPUT_FCU_STATE_UPDATED, now, "mavros/state");
        break;
      }
      case kBattery: {
        xgc_battery_v1 m;
        if (!read(in.data, &m)) break;
        sensor.battery_percentage = m.percentage;
        post(pmc::event_type::INPUT_BATTERY_UPDATED, now, "mavros/battery");
        break;
      }
      case kVrpnPose: {
        xgc_pose_v1 m;
        if (!read(in.data, &m)) break;
        sensor.vrpn_x = m.position[0]; sensor.vrpn_y = m.position[1]; sensor.vrpn_z = m.position[2];
        sensor.vrpn_qw = m.q_wxyz[0]; sensor.vrpn_qx = m.q_wxyz[1];
        sensor.vrpn_qy = m.q_wxyz[2]; sensor.vrpn_qz = m.q_wxyz[3];
        post(pmc::event_type::INPUT_VRPN_POSE_UPDATED, now, "pose");
        break;
      }
      case kCommand: {
        xgc_command_v1 m;
        if (!read(in.data, &m)) break;
        if (auto it = kCommands.find(bounded(m.text, sizeof m.text)); it != kCommands.end()) {
          post(it->second, now, "command");
        } else {
          log(XGC_LOG_WARN, "ctl-px4: unknown command " + bounded(m.text, sizeof m.text));
        }
        break;
      }
      case kAlgSetpoint: {  // TrajectoryInputProducer::algSetpointCallback
        const pmc::ControllerConfig cfg = controller.getConfig();
        if (cfg.tracking_backend != pmc::TrackingBackend::PX4_LOCAL &&
            cfg.tracking_backend != pmc::TrackingBackend::SMC) break;
        xgc_position_target_v1 m;
        if (!read(in.data, &m)) break;
        pmc::PositionTargetIngress ingress;
        ingress.position = Eigen::Vector3d(m.position[0], m.position[1], m.position[2]);
        ingress.velocity = Eigen::Vector3d(m.velocity[0], m.velocity[1], m.velocity[2]);
        ingress.acceleration = Eigen::Vector3d(m.acceleration[0], m.acceleration[1], m.acceleration[2]);
        ingress.yaw = m.yaw;
        ingress.yaw_rate = m.yaw_rate;
        ingress.type_mask = m.type_mask;
        ingress.coordinate_frame = m.coordinate_frame;
        if (std::isfinite(m.stamp) && m.stamp > 0.0 && m.stamp < 4294967296.0) {
          ingress.header_stamp = pmc::Time(m.stamp);
        }
        ingress.receipt_time = pmc::Time().fromNSec(in.t_ns);
        const pmc::MpcTrajectoryState traj =
            pmc::ingestPositionTarget(ingress, cfg.tracking_backend, cfg.px4_local_lift);
        if (pmc::usesStageEffectiveTime(cfg.tracking_backend, cfg.px4_local_lift) && !traj.is_valid) {
          if (cfg.tracking_backend == pmc::TrackingBackend::SMC &&
              (!pmc::smcCoordinateFrameIsWorld(ingress.coordinate_frame) ||
               (ingress.type_mask != 0U && !pmc::smcMaskSuppliesWorldPva(ingress.type_mask)))) {
            log(XGC_LOG_WARN,
                "ctl-px4: SMC rejected PositionTarget; need world frame 1 and every P/V/A axis, FORCE clear");
          } else {
            log(XGC_LOG_WARN, "ctl-px4: effective-time setpoint needs finite PVA and a header stamp");
          }
          break;
        }
        pmc::cacheTrajectorySample(controller.mpcTrajectoryBuffer(), traj,
                                   ingress.receipt_time, cfg);
        post(pmc::event_type::INPUT_MPC_TRAJECTORY_UPDATED, now, "alg/setpoint_raw/local");
        break;
      }
      case kHoverThrust: {  // TrajectoryInputProducer::hoverThrustCallback
        xgc_hover_thrust_v1 m;
        if (!read(in.data, &m)) break;
        if (!std::isfinite(m.hover_thrust) || m.hover_thrust <= 0.0 || m.hover_thrust >= 1.0) {
          sensor.hover_thrust_estimate_available = false;
          sensor.hover_thrust_estimate_flags = m.flags;
          break;
        }
        sensor.hover_thrust_estimate = m.hover_thrust;
        sensor.hover_thrust_estimate_stamp = m.stamp != 0.0 ? m.stamp : pmc::Time().fromNSec(in.t_ns).toSec();
        sensor.hover_thrust_estimate_available = true;
        sensor.hover_thrust_estimate_flags = m.flags;
        post(pmc::event_type::INPUT_HOVER_THRUST_UPDATED, now, "hover_thrust/estimate_state");
        break;
      }
      case kRefActiveAnalytic:
      case kRefActiveSampled: {  // TrajectoryInputProducer::active{Analytic,Sampled}Callback
        auto& cache = controller.activeTrajectoryCache();
        const pmc::Time received = pmc::Time().fromNSec(in.t_ns);
        bool accepted = false;
        const char* source = "";
        if (in.port == kRefActiveAnalytic) {
          pmc::reference::AnalyticReference m;
          accepted = xgc_ref_wire::decode_analytic(in.data.data(), in.data.size(), m) && cache.updateAnalytic(m, received);
          source = "alg/multirotor_reference_trajectory/active/analytic";
        } else {
          pmc::reference::SampledReference m;
          accepted = xgc_ref_wire::decode_sampled(in.data.data(), in.data.size(), m) && cache.updateSampled(m, received);
          source = "alg/multirotor_reference_trajectory/active/sampled";
        }
        if (accepted) post(pmc::event_type::INPUT_REFERENCE_TRAJECTORY_UPDATED, now, source);
        break;
      }
      default:
        break;
    }
  }

  // NmpcOutputConsumer::handle + workerLoop, inline; returns the trace text.
  void solveNmpc(const sm::Event& e, double t) {
    const uint64_t sequence = e.correlation_id;
    const pmc::ControllerConfig cfg = controller.getConfig();
    const double ros_now = pmc::Time(t).toSec();
    const pmc::Time now(e.timestamp > 0.0 ? e.timestamp : ros_now);
    const double stage_dt = cfg.nmpc.prediction_horizon / static_cast<double>(pmc::UavNmpcSolver::horizonSteps());
    std::vector<pmc::Se3Reference> references;
    pmc::NmpcSolveResult result;
    result.sequence = sequence;
    if (!controller.activeTrajectoryCache().sampleHorizon(now, stage_dt, pmc::UavNmpcSolver::horizonSteps(),
                                                           cfg.nmpc.gravity, references)) {
      result.success = false;
      result.solver_status = pmc::nmpc_solver_status::kReferenceSamplingFailed;
      result.stamp = pmc::Time(t);
    } else {
      const pmc::SensorData snapshot = controller.getSensorData();
      result.stamp = now;
      nmpc_backend.configure(cfg);
      if (sequence == 1) {
        nmpc_backend.exit();
        nmpc_entered = false;
      }
      if (!nmpc_entered) nmpc_entered = nmpc_backend.enter(snapshot);
      if (nmpc_entered) {
        result.success = nmpc_backend.compute(snapshot, references, now, result.target);
        result.solver_status = nmpc_backend.status();
      } else {
        result.success = false;
        result.solver_status = pmc::nmpc_solver_status::kBackendUnavailable;
      }
    }
    controller.nmpcResultBuffer().store(result);
    sm::Event done(result.success ? pmc::event_type::INPUT_NMPC_SOLVE_SUCCEEDED : pmc::event_type::INPUT_NMPC_SOLVE_FAILED,
                   sm::EventTimestamp{ros_now});
    done.source = "nmpc_output_consumer";
    done.correlation_id = sequence;
    controller.getStateMachine().postEvent(std::move(done));
    if (trace) {
      char buf[48];
      std::snprintf(buf, sizeof buf, " nmpc %d status %d", result.success ? 1 : 0, result.solver_status);
      line += buf;
      appendHex(line, "t", {result.target.body_rate_x, result.target.body_rate_y, result.target.body_rate_z,
                            result.target.thrust});
    }
  }

  // ReferenceActivationOutputConsumer::handle: the reference request.
  void activateReference(const sm::Event& e, double t, uint64_t round) {
    const double stamp = e.timestamp > 0.0 ? e.timestamp : pmc::Time(t).toSec();
    const auto m = activation.make(stamp, controller.getSensorData(), controller.getConfig());
    auto bytes = xgc_ref_wire::encode_analytic(m);
    auto* head = reinterpret_cast<xgc_ref_analytic_v1*>(bytes.data());
    std::strncpy(head->header.frame_id, "map", sizeof head->header.frame_id - 1);
    publish(kRefRequest, round, bytes.data(), bytes.size());
    if (trace) {
      char buf[160];
      std::snprintf(buf, sizeof buf, " activation %u.%09u req %u id %u rev %u type %u start %u.%09u", m.header.stamp.sec,
                    m.header.stamp.nsec, m.request_id, m.trajectory_id, m.revision, m.analytic_type, m.start_time.sec,
                    m.start_time.nsec);
      line += buf;
      appendHex(line, "o", {m.duration, m.origin.position.x, m.origin.position.y, m.origin.position.z,
                            m.origin.orientation.x, m.origin.orientation.y, m.origin.orientation.z,
                            m.origin.orientation.w});
      line += " params";
      for (double v : m.params) appendHex(line, "", {v});
    }
  }

  bool publish(uint32_t port, uint64_t round, const void* data, size_t n) {
    return host->publish(host->host, port, round, static_cast<const uint8_t*>(data), static_cast<uint32_t>(n)) == XGC_OK;
  }

  void traceLine(uint64_t round) {
    if (trace) publish(kTrace, round, line.data(), line.size());
  }

  static void appendHex(std::string& s, const char* tag, std::initializer_list<double> values) {
    char buf[24];
    s += ' ';
    s += tag;
    for (double v : values) {
      std::snprintf(buf, sizeof buf, " %016" PRIx64, bits(v));
      s += buf;
    }
  }

  // One controller loop iteration at time t, as DroneRosNode::controlLoopCallback.
  bool tick(double t, uint64_t round) {
    runStatsTimer(t);
    controller.update(t);
    for (const auto& e : controller.getStateMachine().currentOutputEvents()) {
      if (trace) {
        char head[160];
        std::snprintf(head, sizeof head, "%" PRIu64 " ev %u ts %016" PRIx64 " seq %" PRIu64 " cat %d src %s", k,
                      static_cast<unsigned>(e.id), bits(e.timestamp), e.sequence, static_cast<int>(e.category),
                      e.source.c_str());
        line = head;
        for (const auto& [key, value] : e.payload) {
          line += ' ';
          line += key;
          line += '=';
          std::visit(
              [&](const auto& v) {
                using V = std::decay_t<decltype(v)>;
                char buf[40];
                if constexpr (std::is_same_v<V, double>) std::snprintf(buf, sizeof buf, "d:%016" PRIx64, bits(v));
                else if constexpr (std::is_same_v<V, int64_t>) std::snprintf(buf, sizeof buf, "i:%" PRId64, v);
                else if constexpr (std::is_same_v<V, bool>) std::snprintf(buf, sizeof buf, "b:%d", v ? 1 : 0);
                if constexpr (std::is_same_v<V, std::string>) line += "s:" + v;
                else line += buf;
              },
              value);
        }
      }
      if (e.id == pmc::output_event_type::PUBLISH_SETPOINT) {
        const auto& s = controller.getSetpoint();
        xgc_position_target_v1 m{};
        m.stamp = t;
        m.position[0] = s.x; m.position[1] = s.y; m.position[2] = s.z;
        m.velocity[0] = s.vx; m.velocity[1] = s.vy; m.velocity[2] = s.vz;
        m.acceleration[0] = s.ax; m.acceleration[1] = s.ay; m.acceleration[2] = s.az;
        m.yaw = pmc::quaternionToYaw(s.qx, s.qy, s.qz, s.qw);  // as ControlOutputConsumer
        m.yaw_rate = s.yaw_rate;
        m.type_mask = s.type_mask;
        m.coordinate_frame = s.coordinate_frame;
        if (!publish(kSetpoint, round, &m, sizeof m)) return false;
        if (trace) {
          appendHex(line, "sp", {s.x, s.y, s.z, s.vx, s.vy, s.vz, s.ax, s.ay, s.az, s.qx, s.qy, s.qz, s.qw, s.yaw_rate});
          line += " mask " + std::to_string(static_cast<unsigned>(s.type_mask));
        }
      } else if (e.id == pmc::output_event_type::PUBLISH_ATTITUDE_RATE_TARGET) {
        const auto& a = controller.getAttitudeRateTarget();
        const xgc_body_rate_thrust_v1 m{t, {a.body_rate_x, a.body_rate_y, a.body_rate_z}, a.thrust};
        publish(kAttitudeRate, round, &m, sizeof m);
        xgc_attitude_target_v2 full{};
        full.stamp = t;
        full.q_wxyz[0] = 1.0; // ignored: this is a body-rate command
        full.body_rate[0] = a.body_rate_x;
        full.body_rate[1] = a.body_rate_y;
        full.body_rate[2] = a.body_rate_z;
        full.thrust = a.thrust;
        full.type_mask = 128; // MAVROS IGNORE_ATTITUDE, rates/thrust active
        if (!publish(kAttitudeCommand, round, &full, sizeof full)) return false;
        if (trace) appendHex(line, "art", {a.body_rate_x, a.body_rate_y, a.body_rate_z, a.thrust});
      } else if (e.id == pmc::output_event_type::REQUEST_ARMING) {
        const auto it = e.payload.find("arm");
        if (it != e.payload.end() && std::holds_alternative<bool>(it->second)) {
          xgc_fcu_request_v1 m{};
          m.stamp = t;
          m.kind = 1;
          m.arm = std::get<bool>(it->second) ? 1u : 0u;
          publish(kFcuRequest, round, &m, sizeof m);
          xgc_fcu_request_v2 full{};
          std::memcpy(&full, &m, sizeof m);
          full.request_id = ++fcu_request_sequence;
          publish(kFcuRequestFull, round, &full, sizeof full);
        }
      } else if (e.id == pmc::output_event_type::REQUEST_MODE) {
        const auto it = e.payload.find("mode");
        if (it != e.payload.end() && std::holds_alternative<std::string>(it->second)) {
          xgc_fcu_request_v1 m{};
          m.stamp = t;
          m.kind = 2;
          std::strncpy(m.mode, std::get<std::string>(it->second).c_str(), sizeof m.mode - 1);
          publish(kFcuRequest, round, &m, sizeof m);
          xgc_fcu_request_v2 full{};
          std::memcpy(&full, &m, sizeof m);
          full.request_id = ++fcu_request_sequence;
          publish(kFcuRequestFull, round, &full, sizeof full);
        }
      } else if (e.id == pmc::output_event_type::PUBLISH_REFERENCE_TRAJECTORY_ACTIVATION) {
        activateReference(e, t, round);
      } else if (e.id == pmc::output_event_type::REQUEST_NMPC_SOLVE) {
        solveNmpc(e, t);
      } else if (e.id == pmc::output_event_type::PUBLISH_CONTROLLER_STATUS) {
        xgc_controller_status_v1 m{};
        m.stamp = t;
        std::strncpy(m.state, controller.getStateMachine().currentStateName(pmc::region_type::CONTROL).c_str(),
                     sizeof m.state - 1);
        publish(kStatus, round, &m, sizeof m);
      }
      if (trace) {
        line += '\n';
        traceLine(round);
      }
    }
    const std::string state = controller.getStateMachine().currentStateName(pmc::region_type::CONTROL);
    if (state != last_state) {
      if (trace) {
        line = std::to_string(k) + " state " + state + "\n";
        traceLine(round);
      }
      last_state = state;
      domain = state;
    }
    for (auto& track : tracks) track.stats->is_new = false;
    ++k;
    if (input_time) {
      const xgc_clock_v1 done{t};
      publish(kTickDone, round, &done, sizeof done);
    }
    return true;
  }

  void drain(std::vector<Input>& into) {
    xgc_sample_view v;
    for (uint32_t port : {kEstimate, kLocalPose, kLocalVelocity, kImu, kFcuState, kBattery, kVrpnPose, kCommand, kClock,
                          kAlgSetpoint, kHoverThrust, kRefActiveAnalytic, kRefActiveSampled}) {
      while (host->next(host->host, port, &v) == XGC_OK) {
        if (port == kClock) {
          xgc_clock_v1 c;
          if (v.len == sizeof c) {
            std::memcpy(&c, v.data, sizeof c);
            clock_limit = std::max(clock_limit, c.seconds);
          }
          continue;
        }
        into.push_back(Input{static_cast<uint64_t>(v.t_produce), port,
                             std::vector<uint8_t>(v.data, v.data + v.len)});
      }
    }
    std::stable_sort(into.begin(), into.end(), [](const Input& a, const Input& b) {
      return a.t_ns < b.t_ns || (a.t_ns == b.t_ns && a.port < b.port);
    });
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    if (input_time) {
      drain(pending);
      if (!pending.empty()) {
        latest_ns = std::max(latest_ns, pending.back().t_ns);
        if (!started) start(pending.front().t_ns * 1e-9);
      }
      if (!started) return XGC_OK;
      size_t next = 0;
      for (;;) {
        const double t = t0 + static_cast<double>(k) * 0.001;
        // Safe once every sample up to t has arrived: a later one has, or
        // the clock says the stream is done up to t.
        if (!(latest_ns * 1e-9 > t) && !(t <= clock_limit)) break;
        while (next < pending.size() && pending[next].t_ns * 1e-9 <= t) apply(pending[next++]);
        if (!tick(t, ctx->round)) return XGC_ERR;
      }
      pending.erase(pending.begin(), pending.begin() + static_cast<std::ptrdiff_t>(next));
      return XGC_OK;
    }
    std::vector<Input> batch;
    drain(batch);
    const double now = static_cast<double>(ctx->now) * 1e-9;
    if (!started) start(batch.empty() ? now : std::min(now, batch.front().t_ns * 1e-9));
    for (const auto& in : batch) apply(in);
    if (ctx->round_advanced == 0) return XGC_OK;
    return tick(now, ctx->round) ? XGC_OK : XGC_ERR;
  }
};

template <typename F>
xgc_status guarded(const xgc_host_api* host, const char* where, F&& f) {
  try {
    return f();
  } catch (const std::exception& e) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": " + e.what()).c_str());
  } catch (...) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": unknown exception").c_str());
  }
  return XGC_ERR;
}

void* create(const xgc_host_api* host) {
  try {
    auto* self = new CtlPx4();
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<CtlPx4*>(p);
  return guarded(self->host, "configure", [&] {
    namespace cfg = xgc_rt_config;
    const std::string t = config ? config : "";
    pmc::ControllerConfig c = self->controller.getConfig();
    int mask = c.local_type_mask;
    const std::string source = cfg::text_or(t, "time_source", "session");
    const std::string backend = cfg::text_or(t, "tracking_backend", "px4_local");
    const std::string lift = cfg::text_or(t, "px4_local_lift", "legacy");
    double smc_k1 = c.smc.k1;
    double smc_k2 = c.smc.k2;
    double smc_rho = c.smc.boundary_layer;
    bool ok = cfg::number(t, "takeoff_altitude", &c.takeoff_altitude) && cfg::integer(t, "local_type_mask", &mask) &&
              cfg::boolean(t, "skip_takeoff_init_disarm", &c.skip_takeoff_init_disarm) &&
              cfg::number(t, "planning_period", &c.planning_period) &&
              cfg::boolean(t, "enable_yaw_control", &c.enable_yaw_control) && cfg::boolean(t, "trace", &self->trace) &&
              cfg::integer(t, "reference_analytic_type", &c.nmpc.reference_analytic_type) &&
              cfg::number(t, "smc_k1", &smc_k1) && cfg::number(t, "smc_k2", &smc_k2) &&
              cfg::number(t, "smc_boundary_layer", &smc_rho) &&
              (source == "session" || source == "input") && mask >= 0 && mask <= 0xFFFF;
    if (backend == "px4_local") c.tracking_backend = pmc::TrackingBackend::PX4_LOCAL;
    else if (backend == "dfbc") c.tracking_backend = pmc::TrackingBackend::DFBC;
    else if (backend == "nmpc") c.tracking_backend = pmc::TrackingBackend::NMPC;
    else if (backend == "smc") c.tracking_backend = pmc::TrackingBackend::SMC;
    else ok = false;
    if (lift == "legacy") c.px4_local_lift = pmc::Px4LocalLiftMode::Legacy;
    else if (lift == "zero_order_hold") c.px4_local_lift = pmc::Px4LocalLiftMode::ZeroOrderHold;
    else ok = false;
    c.smc.k1 = smc_k1;
    c.smc.k2 = smc_k2;
    c.smc.boundary_layer = smc_rho;
    if (!ok) {
      self->log(XGC_LOG_ERROR, "invalid ctl-px4 config");
      return XGC_ERR;
    }
    c.local_type_mask = static_cast<uint16_t>(mask);
    self->input_time = source == "input";
    self->controller.setConfig(c);
    return XGC_OK;
  });
}

xgc_status activate(void*) { return XGC_OK; }

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<CtlPx4*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void*) { return XGC_OK; }

void destroy(void* p) { delete static_cast<CtlPx4*>(p); }

const char* domain_state(void* p) { return static_cast<CtlPx4*>(p)->domain.c_str(); }

const xgc_port_decl kPorts[kPortCount] = {
    {"estimate", XGC_PORT_IN_OPTIONAL, "xgc.rigid_state_estimate/1", XGC_QOS_STATE},
    {"local_pose", XGC_PORT_IN, "xgc.pose/1", XGC_QOS_STATE},
    {"local_velocity", XGC_PORT_IN, "xgc.twist/1", XGC_QOS_STATE},
    {"imu", XGC_PORT_IN, "xgc.imu/1", XGC_QOS_STATE},
    {"fcu_state", XGC_PORT_IN, "xgc.fcu_state/1", XGC_QOS_STATE},
    {"battery", XGC_PORT_IN_OPTIONAL, "xgc.battery/1", XGC_QOS_STATE},
    {"vrpn_pose", XGC_PORT_IN, "xgc.pose/1", XGC_QOS_STATE},
    {"command", XGC_PORT_IN_OPTIONAL, "xgc.command/1", XGC_QOS_EVENT},
    {"clock", XGC_PORT_IN_OPTIONAL, "xgc.clock/1", XGC_QOS_EVENT},
    {"setpoint", XGC_PORT_OUT, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"attitude_rate", XGC_PORT_OUT_OPTIONAL, "xgc.body_rate_thrust/1", XGC_QOS_CONTROL},
    {"fcu_request", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_request/1", XGC_QOS_EVENT},
    {"status", XGC_PORT_OUT_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE},
    {"trace", XGC_PORT_OUT_OPTIONAL, "xgc.text/1", XGC_QOS_BULK},
    {"alg_setpoint", XGC_PORT_IN_OPTIONAL, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"hover_thrust", XGC_PORT_IN_OPTIONAL, "xgc.hover_thrust/1", XGC_QOS_STATE},
    {"ref_active_analytic", XGC_PORT_IN_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_STATE},
    {"ref_active_sampled", XGC_PORT_IN_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_STATE},
    {"ref_request", XGC_PORT_OUT_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_EVENT},
    {"tick_done", XGC_PORT_OUT_OPTIONAL, "xgc.clock/1", XGC_QOS_EVENT},
    {"attitude_command", XGC_PORT_OUT_OPTIONAL, "xgc.attitude_target/2", XGC_QOS_CONTROL},
    {"fcu_request_full", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_request/2", XGC_QOS_EVENT},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {XGC_RT_ABI_VERSION, kPortCount, "ctl-px4", "0.2.0", kPorts, &kVtbl};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
