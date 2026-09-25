// est-hover-thrust: the hover-thrust estimator as an xgc_rt plugin.
//
// This is a WRAP (docs/migration-register in the plan): it runs the unmodified
// HoverThrustEstimatorRuntime from the ROS package hover_thrust_estimator, with
// its own domain state machine (HealthMonitor region + SelfCheck -> Ground <->
// Airborne) on libxgc2-state-machine. Only the ROS input producer and output
// consumer are replaced, by ports:
//
//   in  imu              xgc.imu/1              (accel z)
//   in  attitude_target  xgc.attitude_target/1  (normalized thrust, ignore mask)
//   in  pose             xgc.pose/1             (altitude = position z)
//   out hover_thrust     xgc.hover_thrust/1     (on each PUBLISH_ESTIMATE)
//
// Scheduling: trigger `both`. New samples are applied in source-stamp order
// (ties in port order) and each posts its input event; the runtime is then
// updated. On a round with no input the runtime is still updated so its
// timeouts see time pass. The ROS node polled at 1 kHz to get the same.
//
// Config (TOML, all optional): gravity, initial_hover_thrust, rho2,
// min_hover_thrust, max_hover_thrust, min_altitude, sample_timeout,
// filter_enabled, filter_cutoff_hz, input_rate_low_hz, publish_rate_hz,
// raw_update_rate_hz, time_source = "session" | "input".
// time_source "input" drives the runtime clock from sample stamps only (for
// replay and deterministic tests); "session" (default) uses Session time.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "hover_thrust_estimator/hover_thrust_estimator_runtime.h"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

namespace {

namespace hte = hover_thrust_estimator;
namespace sm = state_machine;

enum Port : uint32_t { kImu = 0, kAttitudeTarget = 1, kPose = 2, kHoverThrust = 3 };

struct Pending {
  double stamp;
  uint32_t port;
  double value;
  bool ignore_thrust;
};

// Minimal reader for flat `key = value` TOML lines, which is all the host
// passes for this plugin's config table.
bool config_value(const std::string& text, const char* key, std::string* out) {
  size_t pos = 0;
  const size_t key_len = std::strlen(key);
  while ((pos = text.find(key, pos)) != std::string::npos) {
    const bool at_line_start = pos == 0 || text[pos - 1] == '\n';
    size_t p = pos + key_len;
    while (p < text.size() && text[p] == ' ') ++p;
    if (at_line_start && p < text.size() && text[p] == '=') {
      ++p;
      while (p < text.size() && text[p] == ' ') ++p;
      size_t end = text.find('\n', p);
      *out = text.substr(p, end == std::string::npos ? std::string::npos : end - p);
      return true;
    }
    pos += key_len;
  }
  return false;
}

bool config_double(const std::string& text, const char* key, double* out) {
  std::string v;
  if (!config_value(text, key, &v)) return true;
  char* end = nullptr;
  const double d = std::strtod(v.c_str(), &end);
  if (end == v.c_str()) return false;
  *out = d;
  return true;
}

bool config_bool(const std::string& text, const char* key, bool* out) {
  std::string v;
  if (!config_value(text, key, &v)) return true;
  if (v == "true") *out = true;
  else if (v == "false") *out = false;
  else return false;
  return true;
}

struct EstHoverThrust {
  const xgc_host_api* host;
  hte::HoverThrustEstimatorRuntime runtime;
  hte::HoverThrustEstimatorRuntime::Input input;
  bool input_time{false};
  std::vector<Pending> pending;

  void log(xgc_log_level level, const std::string& message) const {
    host->log(host->host, level, message.c_str());
  }

  // Apply one sample exactly as HoverThrustInputProducer does for its topic.
  bool apply(const Pending& s) {
    hte::HoverThrustSample* sample = nullptr;
    uint32_t event = 0;
    switch (s.port) {
      case kImu: sample = &input.imu_acc_z; event = hte::event_type::INPUT_IMU_UPDATED; break;
      case kAttitudeTarget:
        input.thrust_ignored = s.ignore_thrust;
        sample = &input.normalized_thrust;
        event = hte::event_type::INPUT_THRUST_UPDATED;
        break;
      default: sample = &input.altitude; event = hte::event_type::INPUT_ALTITUDE_UPDATED; break;
    }
    sample->value = s.value;
    sample->period_sec = sample->received && std::isfinite(sample->stamp_sec) && std::isfinite(s.stamp)
                             ? s.stamp - sample->stamp_sec
                             : 0.0;
    sample->stamp_sec = s.stamp;
    sample->received = true;
    sample->finite = std::isfinite(s.value);
    sm::Event ev(event, sm::EventTimestamp{s.stamp});
    const sm::Status status = runtime.postInputEvent(std::move(ev), input);
    if (!status.ok()) {
      log(XGC_LOG_WARN, "input event rejected: " + status.message);
      return false;
    }
    return true;
  }

  xgc_status update_and_publish(double now_sec, uint64_t round) {
    runtime.update(now_sec);
    for (const auto& ev : runtime.getStateMachine().currentOutputEvents()) {
      if (ev.id != hte::output_event_type::PUBLISH_ESTIMATE) continue;
      // Exactly HoverThrustOutputConsumer::handle: drive the output model to
      // the event time (or now), snapshot, stamp the message with that time.
      const double stamp = std::isfinite(ev.timestamp) && ev.timestamp > 0.0 ? ev.timestamp : now_sec;
      runtime.outputModel().driveTowardTarget(stamp);
      const auto out = runtime.refreshOutputSnapshot();
      xgc_hover_thrust_v1 msg{};
      msg.stamp = stamp;
      msg.hover_thrust = out.hover_thrust;
      msg.raw_hover_thrust = out.raw_hover_thrust;
      msg.initial_hover_thrust = out.initial_hover_thrust;
      msg.thrust_to_acceleration = out.thrust_to_acceleration;
      msg.last_estimate_stamp = out.last_estimate_stamp_sec;
      msg.state = out.state;
      msg.flags = out.flags;
      msg.sample_used = out.sample_used ? 1u : 0u;
      if (host->publish(host->host, kHoverThrust, round, reinterpret_cast<const uint8_t*>(&msg), sizeof msg) != XGC_OK) {
        return XGC_ERR;
      }
    }
    return XGC_OK;
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    pending.clear();
    xgc_sample_view view;
    for (uint32_t port : {kImu, kAttitudeTarget, kPose}) {
      while (host->next(host->host, port, &view) == XGC_OK) {
        if (port == kImu && view.len == sizeof(xgc_imu_v1)) {
          xgc_imu_v1 m;
          std::memcpy(&m, view.data, sizeof m);
          pending.push_back({m.stamp, port, m.accel[2], false});
        } else if (port == kAttitudeTarget && view.len == sizeof(xgc_attitude_target_v1)) {
          xgc_attitude_target_v1 m;
          std::memcpy(&m, view.data, sizeof m);
          pending.push_back({m.stamp, port, m.thrust, m.ignore_thrust != 0});
        } else if (port == kPose && view.len == sizeof(xgc_pose_v1)) {
          xgc_pose_v1 m;
          std::memcpy(&m, view.data, sizeof m);
          pending.push_back({m.stamp, port, m.position[2], false});
        } else {
          log(XGC_LOG_WARN, "dropped a sample with the wrong payload size");
        }
      }
    }
    std::stable_sort(pending.begin(), pending.end(),
                     [](const Pending& a, const Pending& b) { return a.stamp < b.stamp; });
    if (input_time) {
      // Deterministic replay: the runtime clock is the stamp of each sample.
      for (const auto& s : pending) {
        apply(s);
        if (update_and_publish(s.stamp, ctx->round) != XGC_OK) return XGC_ERR;
      }
      return XGC_OK;
    }
    for (const auto& s : pending) apply(s);
    return update_and_publish(static_cast<double>(ctx->now) * 1e-9, ctx->round);
  }
};

// No exception may cross the C ABI. A caught one is logged through the host
// (so health.jsonl says why) and becomes XGC_ERR, which faults the plugin.
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
    auto* self = new EstHoverThrust{host, {}, {}, false, {}};
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<EstHoverThrust*>(p);
  return guarded(self->host, "configure", [&] {
    const std::string text = config ? config : "";
    hte::HoverThrustEstimatorConfig c;
    bool ok = config_double(text, "gravity", &c.gravity) &&
              config_double(text, "initial_hover_thrust", &c.initial_hover_thrust) &&
              config_double(text, "rho2", &c.rho2) &&
              config_double(text, "min_hover_thrust", &c.min_hover_thrust) &&
              config_double(text, "max_hover_thrust", &c.max_hover_thrust) &&
              config_double(text, "min_altitude", &c.min_altitude) &&
              config_double(text, "sample_timeout", &c.sample_timeout) &&
              config_bool(text, "filter_enabled", &c.filter_enabled) &&
              config_double(text, "filter_cutoff_hz", &c.filter_cutoff_hz) &&
              config_double(text, "input_rate_low_hz", &c.input_rate_low_hz) &&
              config_double(text, "publish_rate_hz", &c.publish_rate_hz) &&
              config_double(text, "raw_update_rate_hz", &c.raw_update_rate_hz);
    std::string source = "\"session\"";
    config_value(text, "time_source", &source);
    if (source != "\"session\"" && source != "\"input\"") ok = false;
    if (!ok) {
      self->log(XGC_LOG_ERROR, "invalid est-hover-thrust config");
      return XGC_ERR;
    }
    self->input_time = source == "\"input\"";
    self->runtime.setConfig(c);
    self->input = {};
    return XGC_OK;
  });
}

xgc_status activate(void*) { return XGC_OK; }

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<EstHoverThrust*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void*) { return XGC_OK; }

void destroy(void* p) { delete static_cast<EstHoverThrust*>(p); }

const char* domain_state(void* p) {
  switch (static_cast<EstHoverThrust*>(p)->runtime.snapshotOutput().state) {
    case hte::state_type::SelfCheck: return "self_check";
    case hte::state_type::Ground: return "ground";
    case hte::state_type::Airborne: return "airborne";
    default: return "unknown";
  }
}

const xgc_port_decl kPorts[] = {
    {"imu", XGC_PORT_IN, "xgc.imu/1", XGC_QOS_STATE},
    {"attitude_target", XGC_PORT_IN, "xgc.attitude_target/1", XGC_QOS_STATE},
    {"pose", XGC_PORT_IN, "xgc.pose/1", XGC_QOS_STATE},
    {"hover_thrust", XGC_PORT_OUT, "xgc.hover_thrust/1", XGC_QOS_STATE},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {
    XGC_RT_ABI_VERSION, 4u, "est-hover-thrust", "0.1.0", kPorts, &kVtbl,
};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
