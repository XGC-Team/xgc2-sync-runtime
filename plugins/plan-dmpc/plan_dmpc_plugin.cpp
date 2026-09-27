#include "plan_dmpc.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "common/flat_config.hpp"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {
enum Port : uint32_t {
  kPaired = 0,
  kController,
  kSceneSnapshot,
  kHeartbeat,
  kCommit,
  kNeighbor,
  kSyncTrigger,
  kTimelineStatus,
  kPositionTarget,
  kOwnPlan,
  kPlannerStatus,
  kNeighborPosition,
  kOwnPosition,
  kPortCount
};

const xgc_port_decl kPorts[kPortCount] = {
    {"paired_state", XGC_PORT_IN, "xgc.dmpc.paired_state/1", XGC_QOS_STATE},
    {"controller_state", XGC_PORT_IN, "xgc.controller_status/1", XGC_QOS_STATE},
    {"scene_snapshot", XGC_PORT_IN, "xgc.dmpc.scene_snapshot/1", XGC_QOS_STATE},
    {"scene_heartbeat", XGC_PORT_IN, "xgc.dmpc.scene_heartbeat/1", XGC_QOS_STATE},
    {"timeline_commit", XGC_PORT_IN, "xgc.dmpc.mission_commit/1", XGC_QOS_EVENT},
    {"neighbor_plan", XGC_PORT_IN, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL},
    {"sync_trigger", XGC_PORT_IN, "xgc.dmpc.sync_trigger/1", XGC_QOS_CONTROL},
    {"timeline_status", XGC_PORT_OUT, "xgc.dmpc.timeline_status/1", XGC_QOS_STATE},
    {"position_target", XGC_PORT_OUT, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"own_plan", XGC_PORT_OUT, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL},
    {"planner_status", XGC_PORT_OUT, "xgc.dmpc.planner_status/1", XGC_QOS_STATE},
    {"neighbor_position", XGC_PORT_IN, "xgc.dmpc.measured_position/1", XGC_QOS_STATE},
    {"own_position", XGC_PORT_OUT, "xgc.dmpc.measured_position/1", XGC_QOS_STATE},
};

void sha256(const uint8_t* data, size_t len, uint8_t out[32]) {
  static const uint32_t k[64] = {
      0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
      0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
      0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
      0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
      0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
      0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
      0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
      0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
  uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
  std::vector<uint8_t> msg(data, data + len);
  const uint64_t bits = static_cast<uint64_t>(len) * 8;
  msg.push_back(0x80);
  while ((msg.size() % 64) != 56) msg.push_back(0);
  for (int shift = 56; shift >= 0; shift -= 8) msg.push_back(static_cast<uint8_t>(bits >> shift));
  for (size_t offset = 0; offset < msg.size(); offset += 64) {
    uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
      w[i] = (uint32_t(msg[offset + i * 4]) << 24) | (uint32_t(msg[offset + i * 4 + 1]) << 16) |
             (uint32_t(msg[offset + i * 4 + 2]) << 8) | uint32_t(msg[offset + i * 4 + 3]);
    }
    for (int i = 16; i < 64; ++i) {
      const uint32_t s0 = ((w[i - 15] >> 7) | (w[i - 15] << 25)) ^ ((w[i - 15] >> 18) | (w[i - 15] << 14)) ^ (w[i - 15] >> 3);
      const uint32_t s1 = ((w[i - 2] >> 17) | (w[i - 2] << 15)) ^ ((w[i - 2] >> 19) | (w[i - 2] << 13)) ^ (w[i - 2] >> 10);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; ++i) {
      const uint32_t S1 = ((e >> 6) | (e << 26)) ^ ((e >> 11) | (e << 21)) ^ ((e >> 25) | (e << 7));
      const uint32_t ch = (e & f) ^ (~e & g);
      const uint32_t t1 = hh + S1 + ch + k[i] + w[i];
      const uint32_t S0 = ((a >> 2) | (a << 30)) ^ ((a >> 13) | (a << 19)) ^ ((a >> 22) | (a << 10));
      const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const uint32_t t2 = S0 + maj;
      hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
  }
  for (int i = 0; i < 8; ++i) {
    out[i * 4] = static_cast<uint8_t>(h[i] >> 24);
    out[i * 4 + 1] = static_cast<uint8_t>(h[i] >> 16);
    out[i * 4 + 2] = static_cast<uint8_t>(h[i] >> 8);
    out[i * 4 + 3] = static_cast<uint8_t>(h[i]);
  }
}

struct Plugin {
  const xgc_host_api* host = nullptr;
  std::unique_ptr<xgc_plan_dmpc::PlanDmpc> plan;
  std::string domain = "unconfigured";
  bool have_planner_round = false;
  uint64_t planner_k = 0;
  std::optional<xgc_dmpc_measured_position_v1> own_position;
};

xgc_status guarded(const xgc_host_api* host, const char* what, const std::function<xgc_status()>& body) {
  try {
    return body();
  } catch (const std::exception& error) {
    if (host && host->log) host->log(host->host, XGC_LOG_ERROR, error.what());
    (void)what;
    return XGC_ERR;
  } catch (...) {
    return XGC_ERR;
  }
}

void* create(const xgc_host_api* host) {
  auto* self = new Plugin;
  self->host = host;
  return self;
}

std::string unquote(std::string text) {
  if (text.size() >= 2 && text.front() == '"' && text.back() == '"') {
    return text.substr(1, text.size() - 2);
  }
  return text;
}

bool optional_integer(const std::string& config, const char* key, std::optional<int>* out) {
  std::string present;
  if (!xgc_rt_config::value(config, key, &present)) return true;
  int value = 0;
  if (!xgc_rt_config::integer(config, key, &value)) return false;
  *out = value;
  return true;
}

xgc_status configure(void* p, const char* text) {
  auto* self = static_cast<Plugin*>(p);
  return guarded(self->host, "configure", [&] {
    namespace cfg = xgc_rt_config;
    const std::string config = text ? text : "";
    std::string present;
    if (!cfg::value(config, "self_id", &present) || !cfg::value(config, "timeline_authority", &present) ||
        !cfg::value(config, "manifest", &present) || !cfg::value(config, "scene_id", &present)) {
      return XGC_ERR;
    }
    xgc_plan_dmpc::PlanDmpcOpen request;
    int self_id = 0;
    int authority = 0;
    if (!cfg::integer(config, "self_id", &self_id) || !cfg::integer(config, "timeline_authority", &authority) ||
        authority < 0 || authority > 65535) {
      return XGC_ERR;
    }
    request.self_id = self_id;
    request.timeline_authority = static_cast<uint16_t>(authority);
    request.manifest_path = unquote(cfg::text_or(config, "manifest", ""));
    request.scene_id = unquote(cfg::text_or(config, "scene_id", ""));
    std::string algorithm;
    if (cfg::value(config, "algorithm", &algorithm)) request.algorithm = unquote(algorithm);
    if (!optional_integer(config, "chain_n", &request.chain_n) ||
        !optional_integer(config, "state_dim", &request.state_dim) ||
        !optional_integer(config, "horizon", &request.horizon) ||
        !optional_integer(config, "fleet_count", &request.fleet_count)) {
      return XGC_ERR;
    }
    if (cfg::value(config, "sampling_time", &present)) {
      double sampling = 0.0;
      if (!cfg::number(config, "sampling_time", &sampling)) return XGC_ERR;
      request.sampling_time = sampling;
    }
    std::string error;
    auto plan = xgc_plan_dmpc::PlanDmpc::open(request, &error);
    if (!plan) {
      if (self->host && self->host->log) self->host->log(self->host->host, XGC_LOG_ERROR, error.c_str());
      return XGC_ERR;
    }
    self->plan = std::move(plan);
    self->domain = "configured";
    return XGC_OK;
  });
}

xgc_status activate(void*) { return XGC_OK; }

template <typename T>
bool take_last(const xgc_host_api* host, uint32_t port, T* out) {
  xgc_sample_view view{};
  bool got = false;
  while (host->next(host->host, port, &view) == XGC_OK) {
    if (view.len != sizeof(T) || view.data == nullptr) continue;
    std::memcpy(out, view.data, sizeof(T));
    got = true;
  }
  return got;
}

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<Plugin*>(p);
  return guarded(self->host, "step", [&] {
    if (!self->plan || ctx == nullptr) return XGC_ERR;
    const double now_sec = static_cast<double>(ctx->now) * 1e-9;
    const double wall = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    xgc_dmpc_paired_state_v1 paired{};
    if (take_last(self->host, kPaired, &paired) && self->plan->push_state(paired, now_sec).empty()) {
      xgc_dmpc_measured_position_v1 position{};
      position.uav_id = static_cast<uint32_t>(self->plan->config().self_id);
      position.stamp_sec = paired.pose_stamp_sec;
      std::memcpy(position.position, paired.position, sizeof position.position);
      self->own_position = position;
    }
    xgc_controller_status_v1 controller{};
    if (take_last(self->host, kController, &controller)) {
      xgc_dmpc_controller_status_v1 status{};
      status.stamp_sec = controller.stamp;
      std::memcpy(status.state, controller.state, sizeof status.state);
      (void)self->plan->push_controller(status, now_sec);
    }
    xgc_sample_view view{};
    std::vector<uint8_t> scene;
    while (self->host->next(self->host->host, kSceneSnapshot, &view) == XGC_OK) {
      if (view.data == nullptr) continue;
      scene.assign(view.data, view.data + view.len);
    }
    if (!scene.empty()) (void)self->plan->push_scene_blob(scene.data(), scene.size());
    xgc_dmpc_scene_heartbeat_v1 beat{};
    if (take_last(self->host, kHeartbeat, &beat)) (void)self->plan->push_heartbeat(beat);
    while (self->host->next(self->host->host, kCommit, &view) == XGC_OK) {
      if (view.len != sizeof(xgc_dmpc_mission_commit_v1) || view.data == nullptr) continue;
      xgc_dmpc_mission_commit_v1 commit{};
      std::memcpy(&commit, view.data, sizeof commit);
      uint8_t digest[32];
      sha256(view.data, 240, digest);
      (void)self->plan->push_commit(commit, digest);
    }
    while (self->host->next(self->host->host, kNeighbor, &view) == XGC_OK) {
      if (view.data == nullptr) continue;
      (void)self->plan->push_neighbor_plan(view.data, view.len);
    }
    while (self->host->next(self->host->host, kNeighborPosition, &view) == XGC_OK) {
      if (view.data == nullptr || view.len != sizeof(xgc_dmpc_measured_position_v1)) continue;
      xgc_dmpc_measured_position_v1 position{};
      std::memcpy(&position, view.data, sizeof position);
      (void)self->plan->push_neighbor_position(position);
    }
    bool planner_due = false;
    uint64_t planner_k = 0;
    double trigger_time = 0.0;
    while (self->host->next(self->host->host, kSyncTrigger, &view) == XGC_OK) {
      if (view.data == nullptr || view.len < sizeof(xgc_dmpc_sync_trigger_v1)) continue;
      xgc_dmpc_sync_trigger_v1 trigger{};
      std::memcpy(&trigger, view.data, sizeof trigger);
      if (self->have_planner_round && trigger.sequence_id == self->planner_k) continue;
      planner_due = true;
      planner_k = trigger.sequence_id;
      trigger_time = trigger.trigger_time;
    }
    if (!planner_due) return XGC_OK;
    self->have_planner_round = true;
    self->planner_k = planner_k;
    if (self->own_position &&
        self->host->publish(self->host->host, kOwnPosition, planner_k,
                            reinterpret_cast<const uint8_t*>(&*self->own_position),
                            sizeof(xgc_dmpc_measured_position_v1)) != XGC_OK) {
      return XGC_ERR;
    }
    const auto trace = self->plan->step(trigger_time, trigger_time, wall);
    const char* domain = trace.status.lifecycle[0] ? trace.status.lifecycle : "step";
    if (self->domain != domain) {
      self->domain = domain;
      if (self->host->log) {
        const std::string message = "planner lifecycle: " + self->domain;
        self->host->log(self->host->host, XGC_LOG_INFO, message.c_str());
      }
    }
    if (self->host->publish(self->host->host, kTimelineStatus, planner_k,
                            reinterpret_cast<const uint8_t*>(&trace.timeline), sizeof trace.timeline) != XGC_OK ||
        self->host->publish(self->host->host, kPlannerStatus, planner_k,
                            reinterpret_cast<const uint8_t*>(&trace.status), sizeof trace.status) != XGC_OK) {
      return XGC_ERR;
    }
    if (trace.have_position_target &&
        self->host->publish(self->host->host, kPositionTarget, planner_k,
                            reinterpret_cast<const uint8_t*>(&trace.position_target),
                            sizeof trace.position_target) != XGC_OK) {
      return XGC_ERR;
    }
    if (!trace.own_plan.empty() &&
        self->host->publish(self->host->host, kOwnPlan, planner_k, trace.own_plan.data(),
                            static_cast<uint32_t>(trace.own_plan.size())) != XGC_OK) {
      return XGC_ERR;
    }
    return XGC_OK;
  });
}

xgc_status deactivate(void*) { return XGC_OK; }
void destroy(void* p) { delete static_cast<Plugin*>(p); }
const char* domain_state(void* p) { return static_cast<Plugin*>(p)->domain.c_str(); }

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};
const xgc_plugin_descriptor kDescriptor = {XGC_RT_ABI_VERSION, kPortCount, "plan-dmpc", "0.1.0", kPorts, &kVtbl};
}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
