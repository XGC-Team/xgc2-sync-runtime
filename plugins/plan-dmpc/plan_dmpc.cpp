// plan-dmpc: the TRO DMPC planner as an aggregator module.
//
// Runs one robot's planner without ROS through the academic planner's
// DmpcAgent (formation_generator_dmpc_config: dmpc_scheduler/dmpc_agent.h),
// which configures, seeds and steps the optimizer core the way the ROS node
// does on a FormationTick. This module only moves samples between ports and
// the agent.
//
//   in  formation_tick  xgc.dmpc.formation_tick/1       (dmpc-rounds: the round's mission phase)
//   in  plan_in         xgc.dmpc.assumed_trajectory/1   (neighbor plans over the link)
//   out plan_out        xgc.dmpc.assumed_trajectory/1   (this robot's plan, every round)
//   in  own_state       xgc.rigid_state/1               (measured state; seeds the plan)
//   out setpoint        xgc.position_target/1           (the node's setpoint_raw/local)
//   in  scene_snapshot  xgc.scene.snapshot/1            (optional; the shared scene's definition)
//   in  scene_state     xgc.scene.state/1               (optional; its obstacles' current state)
//
// Rounds: a formation tick of round k runs the agent once, and its plan and
// setpoint are published for round k. Before the tick, the module gives the
// agent every measured state of rounds <= k and every neighbor plan of
// rounds <= k - 1 (the sample's round, never its arrival time), in round
// order: a neighbor's round-k plan is used at round k + 1 even when it
// arrives before the tick of round k. Scene samples of rounds <= k go to the
// agent after the measured states and before the plans, by round and, within
// a round, definitions before states; a state's receive time is its stamp.
// The agent's clock is the tick's trigger time, so a run is a function of
// its inputs.
//
// A configuration with static or moving obstacle constraints needs the scene
// ports bound: the agent does not seed before the scene arrived, and it
// holds position (the node's scene hold) while the scene is invalid, its
// state is older than scene_state_timeout, or a new scene's first round fails.
//
// Config: param_manifest (required): the robot's param manifest, the
// scenario YAML files its launch block loads (rosparam_yaml.h,
// ParamManifest); relative paths resolve against the manifest's directory.
//
// Domain state: the agent's phase (wait_obstacle_info, wait_self_state,
// wait_neighbors, hold, rolling, fault), or "unconfigured".

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "flat_config.hpp"
#include "formation_generator/core/core_log.h"
#include "formation_generator/core/plan_wire.h"
#include "formation_generator/core/position_target_output.h"
#include "formation_generator/core/scene_wire.h"
#include "formation_generator/dmpc_scheduler/dmpc_agent.h"
#include "formation_generator/params/rosparam_yaml.h"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {

namespace fg = formation_generator_dmpc;

enum Port : uint32_t {
  kFormationTick, kPlanIn, kPlanOut, kOwnState, kSetpoint, kSceneSnapshot, kSceneState, kPortCount
};

static_assert(sizeof(fg::PositionTargetPayload) == sizeof(xgc_position_target_v1),
              "PositionTargetPayload is xgc_position_target_v1");

// The core logs through one process-wide sink; each module thread points it
// at its own host while it calls into the agent.
thread_local const xgc_host_api* t_log_host = nullptr;

void logToHost(dmpc_core::LogLevel level, const char* message) {
  if (!t_log_host) return;
  const xgc_log_level l = level == dmpc_core::LogLevel::kInfo   ? XGC_LOG_INFO
                          : level == dmpc_core::LogLevel::kWarn ? XGC_LOG_WARN
                                                                : XGC_LOG_ERROR;
  t_log_host->log(t_log_host->host, l, message);
}

struct LogScope {
  explicit LogScope(const xgc_host_api* host) { t_log_host = host; }
  ~LogScope() { t_log_host = nullptr; }
};

bool readTick(const xgc_sample_view& s, fg::DmpcTick& tick) {
  constexpr size_t kHead = sizeof(xgc_dmpc_formation_tick_v1);
  constexpr size_t kTrigger = sizeof(xgc_dmpc_sync_trigger_v1);
  if (s.len < kHead + kTrigger) return false;
  xgc_dmpc_formation_tick_v1 head;
  xgc_dmpc_sync_trigger_v1 trigger;
  std::memcpy(&head, s.data, kHead);
  std::memcpy(&trigger, s.data + kHead, kTrigger);
  if (s.len != kHead + kTrigger + 4ull * trigger.count) return false;
  tick.round = s.round;
  tick.mission_time = head.mission_time;
  tick.rolling = head.rolling != 0;
  tick.trigger_time = trigger.trigger_time;
  return true;
}

// One scene sample: a definition or a state, with its round.
struct SceneInput {
  uint64_t round;
  bool definition;
  fg::SceneSnapshotData snapshot;
  fg::SceneStateData state;
};

struct PlanDmpc {
  const xgc_host_api* host{nullptr};
  std::unique_ptr<fg::DmpcAgent> agent;
  // Inputs not yet given to the agent, in arrival order.
  std::vector<std::pair<uint64_t, xgc_rigid_state_v1>> states;
  std::vector<std::pair<uint64_t, fg::PlanMessage>> plans;
  std::vector<SceneInput> scene;
  std::map<uint64_t, fg::DmpcTick> ticks;
  bool ticked{false};
  uint64_t last_tick{0};

  void log(xgc_log_level level, const std::string& message) const {
    host->log(host->host, level, message.c_str());
  }

  void drain() {
    xgc_sample_view s;
    while (host->next(host->host, kOwnState, &s) == XGC_OK) {
      xgc_rigid_state_v1 state;
      if (s.len != sizeof state) {
        log(XGC_LOG_WARN, "plan-dmpc: malformed own_state dropped");
        continue;
      }
      std::memcpy(&state, s.data, sizeof state);
      states.emplace_back(s.round, state);
    }
    while (host->next(host->host, kPlanIn, &s) == XGC_OK) {
      fg::PlanMessage plan;
      if (!fg::decodePlanPayload(s.data, s.len, plan)) {
        log(XGC_LOG_WARN, "plan-dmpc: malformed neighbor plan dropped");
        continue;
      }
      plans.emplace_back(s.round, std::move(plan));
    }
    while (host->next(host->host, kSceneSnapshot, &s) == XGC_OK) {
      SceneInput in{s.round, true, {}, {}};
      if (!fg::decodeSceneSnapshot(s.data, s.len, in.snapshot)) {
        log(XGC_LOG_WARN, "plan-dmpc: malformed scene snapshot dropped");
        continue;
      }
      scene.push_back(std::move(in));
    }
    while (host->next(host->host, kSceneState, &s) == XGC_OK) {
      SceneInput in{s.round, false, {}, {}};
      if (!fg::decodeSceneState(s.data, s.len, in.state)) {
        log(XGC_LOG_WARN, "plan-dmpc: malformed scene state dropped");
        continue;
      }
      scene.push_back(std::move(in));
    }
    while (host->next(host->host, kFormationTick, &s) == XGC_OK) {
      fg::DmpcTick tick;
      if (!readTick(s, tick)) {
        log(XGC_LOG_WARN, "plan-dmpc: malformed formation tick dropped");
        continue;
      }
      if (ticked && tick.round <= last_tick) continue;  // a repeated or late round
      ticks[tick.round] = tick;
    }
  }

  // Hand the agent the inputs a tick of round `k` may use, oldest round first.
  void feed(uint64_t k) {
    std::vector<std::pair<uint64_t, xgc_rigid_state_v1>> later_states;
    for (const auto& [round, st] : states) {
      if (round > k) {
        later_states.emplace_back(round, st);
        continue;
      }
      Eigen::Matrix<double, 6, 1> pv;
      pv << st.position[0], st.position[1], st.position[2], st.velocity[0], st.velocity[1],
          st.velocity[2];
      agent->observeState(pv, Eigen::Vector4d(st.q_wxyz[1], st.q_wxyz[2], st.q_wxyz[3], st.q_wxyz[0]),
                          st.stamp);
    }
    states = std::move(later_states);
    std::vector<SceneInput> usable_scene, later_scene;
    for (auto& in : scene) (in.round <= k ? usable_scene : later_scene).push_back(std::move(in));
    std::stable_sort(usable_scene.begin(), usable_scene.end(), [](const SceneInput& a, const SceneInput& b) {
      return a.round != b.round ? a.round < b.round : a.definition && !b.definition;
    });
    for (const auto& in : usable_scene) {
      if (in.definition) agent->receiveSceneSnapshot(in.snapshot);
      else agent->receiveSceneState(in.state, in.state.stamp);
    }
    scene = std::move(later_scene);
    std::vector<std::pair<uint64_t, fg::PlanMessage>> usable, later_plans;
    for (auto& entry : plans) (entry.first + 1 <= k ? usable : later_plans).push_back(std::move(entry));
    std::stable_sort(usable.begin(), usable.end(),
                     [](const auto& a, const auto& b) { return a.first < b.first; });
    for (const auto& entry : usable) agent->receiveNeighborPlan(entry.second);
    plans = std::move(later_plans);
  }

  xgc_status step(const xgc_step_ctx*) {
    drain();
    while (!ticks.empty()) {
      const fg::DmpcTick tick = ticks.begin()->second;
      ticks.erase(ticks.begin());
      feed(tick.round);
      const fg::DmpcTickOutput out = agent->tick(tick);
      ticked = true;
      last_tick = tick.round;
      if (out.has_plan) {
        const std::vector<uint8_t> payload = fg::encodePlanPayload(out.plan);
        if (host->publish(host->host, kPlanOut, tick.round, payload.data(),
                          static_cast<uint32_t>(payload.size())) != XGC_OK) {
          log(XGC_LOG_ERROR, "plan-dmpc: publish plan failed");
          return XGC_ERR;
        }
      }
      if (out.has_setpoint) {
        if (host->publish(host->host, kSetpoint, tick.round,
                          reinterpret_cast<const uint8_t*>(&out.setpoint),
                          static_cast<uint32_t>(sizeof out.setpoint)) != XGC_OK) {
          log(XGC_LOG_ERROR, "plan-dmpc: publish setpoint failed");
          return XGC_ERR;
        }
      }
    }
    return XGC_OK;
  }
};

template <typename F>
xgc_status guarded(const xgc_host_api* host, const char* where, F&& f) {
  LogScope scope(host);
  try {
    return f();
  } catch (const std::exception& e) {
    host->log(host->host, XGC_LOG_ERROR, (std::string("plan-dmpc ") + where + ": " + e.what()).c_str());
  } catch (...) {
    host->log(host->host, XGC_LOG_ERROR, (std::string("plan-dmpc ") + where + ": unknown exception").c_str());
  }
  return XGC_ERR;
}

void* create(const xgc_host_api* host) {
  try {
    dmpc_core::setLogSink(&logToHost);
    auto* self = new PlanDmpc();
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<PlanDmpc*>(p);
  return guarded(self->host, "configure", [&] {
    const std::string manifest = xgc_rt_config::text_or(config ? config : "", "param_manifest", "");
    if (manifest.empty()) {
      self->log(XGC_LOG_ERROR, "plan-dmpc: param_manifest is required");
      return XGC_ERR;
    }
    self->agent = std::make_unique<fg::DmpcAgent>(fg::privateParamsFromManifest(fg::loadParamManifest(manifest)));
    if (self->agent->configuration().planar_reference_output) {
      self->agent.reset();
      self->log(XGC_LOG_ERROR, "plan-dmpc: planar_reference_output has no output port yet (xgc.position_target/1 only)");
      return XGC_ERR;
    }
    return XGC_OK;
  });
}

xgc_status activate(void* p) {
  auto* self = static_cast<PlanDmpc*>(p);
  return self->agent ? XGC_OK : XGC_ERR;
}

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<PlanDmpc*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void*) { return XGC_OK; }

void destroy(void* p) { delete static_cast<PlanDmpc*>(p); }

const char* domain_state(void* p) {
  const auto* self = static_cast<PlanDmpc*>(p);
  return self->agent ? fg::dmpcAgentPhaseName(self->agent->phase()) : "unconfigured";
}

const xgc_port_decl kPorts[kPortCount] = {
    {"formation_tick", XGC_PORT_IN, "xgc.dmpc.formation_tick/1", XGC_QOS_CONTROL},
    {"plan_in", XGC_PORT_IN, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL},
    {"plan_out", XGC_PORT_OUT, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL},
    {"own_state", XGC_PORT_IN, "xgc.rigid_state/1", XGC_QOS_STATE},
    {"setpoint", XGC_PORT_OUT, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"scene_snapshot", XGC_PORT_IN_OPTIONAL, "xgc.scene.snapshot/1", XGC_QOS_EVENT},
    {"scene_state", XGC_PORT_IN_OPTIONAL, "xgc.scene.state/1", XGC_QOS_STATE},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {XGC_RT_ABI_VERSION, kPortCount, "plan-dmpc", "0.2.0", kPorts, &kVtbl};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
