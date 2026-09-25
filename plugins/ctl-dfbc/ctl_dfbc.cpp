// ctl-dfbc: xgc2_math's DFBC geometric controller as an xgc_rt plugin.
//
//   in  state   xgc.rigid_state/1        (each new sample -> one command)
//   in  ref     xgc.flat_ref/1           (latest is held)
//   out cmd     xgc.attitude_rate_cmd/1  (control QoS)
//
// Trigger on_dirty. New samples from both ports are applied in stamp order
// (ties: ref before state, so a reference stamped with its state applies).
// Domain FSM: `waiting_reference` -> `tracking`; `rejecting` while the
// controller reports failure.
//
// Config (TOML, optional): tilt_gain, tilt_rate_damping, yaw_gain,
// yaw_rate_damping, gravity, min_specific_thrust, use_body_rate_feedforward,
// enable_yaw_control, nominal_dt, max_dt.

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "dfbc_driver.hpp"
#include "xgc_rt.h"

namespace {

enum Port : uint32_t { kState = 0, kRef = 1, kCmd = 2 };

bool config_value(const std::string& text, const char* key, std::string* out) {
  size_t pos = 0;
  const size_t n = std::strlen(key);
  while ((pos = text.find(key, pos)) != std::string::npos) {
    size_t p = pos + n;
    while (p < text.size() && text[p] == ' ') ++p;
    if ((pos == 0 || text[pos - 1] == '\n') && p < text.size() && text[p] == '=') {
      ++p;
      while (p < text.size() && text[p] == ' ') ++p;
      const size_t end = text.find('\n', p);
      *out = text.substr(p, end == std::string::npos ? std::string::npos : end - p);
      return true;
    }
    pos += n;
  }
  return false;
}

bool read_double(const std::string& t, const char* k, double* out) {
  std::string v;
  if (!config_value(t, k, &v)) return true;
  char* end = nullptr;
  *out = std::strtod(v.c_str(), &end);
  return end != v.c_str();
}

bool read_bool(const std::string& t, const char* k, bool* out) {
  std::string v;
  if (!config_value(t, k, &v)) return true;
  if (v != "true" && v != "false") return false;
  *out = v == "true";
  return true;
}

struct Item {
  double stamp;
  bool is_ref;
  xgc_rigid_state_v1 state;
  xgc_flat_ref_v1 ref;
};

struct CtlDfbc {
  const xgc_host_api* host{nullptr};
  ctl_dfbc::Driver driver;
  const char* domain{"waiting_reference"};
  std::vector<Item> items;

  xgc_status step(const xgc_step_ctx* ctx) {
    items.clear();
    xgc_sample_view v;
    while (host->next(host->host, kRef, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_flat_ref_v1)) continue;
      Item it{};
      std::memcpy(&it.ref, v.data, sizeof it.ref);
      it.stamp = it.ref.stamp;
      it.is_ref = true;
      items.push_back(it);
    }
    while (host->next(host->host, kState, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_rigid_state_v1)) continue;
      Item it{};
      std::memcpy(&it.state, v.data, sizeof it.state);
      it.stamp = it.state.stamp;
      items.push_back(it);
    }
    std::stable_sort(items.begin(), items.end(), [](const Item& a, const Item& b) {
      return a.stamp < b.stamp || (a.stamp == b.stamp && a.is_ref && !b.is_ref);
    });
    for (const auto& it : items) {
      if (it.is_ref) {
        driver.set_reference(it.ref);
        continue;
      }
      xgc_attitude_rate_cmd_v1 cmd;
      if (!driver.on_state(it.state, &cmd)) continue;
      domain = cmd.success ? "tracking" : "rejecting";
      if (host->publish(host->host, kCmd, ctx->round, reinterpret_cast<const uint8_t*>(&cmd), sizeof cmd) != XGC_OK) {
        return XGC_ERR;
      }
    }
    return XGC_OK;
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
    auto* self = new CtlDfbc();
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<CtlDfbc*>(p);
  return guarded(self->host, "configure", [&] {
    const std::string t = config ? config : "";
    xgc2_math::control::DfbcGeometricConfig c;
    double nominal = 0.01, maximum = 0.1;
    const bool ok = read_double(t, "tilt_gain", &c.tilt_gain) && read_double(t, "tilt_rate_damping", &c.tilt_rate_damping) &&
                    read_double(t, "yaw_gain", &c.yaw_gain) && read_double(t, "yaw_rate_damping", &c.yaw_rate_damping) &&
                    read_double(t, "gravity", &c.gravity) && read_double(t, "min_specific_thrust", &c.min_specific_thrust) &&
                    read_bool(t, "use_body_rate_feedforward", &c.use_body_rate_feedforward) &&
                    read_bool(t, "enable_yaw_control", &c.enable_yaw_control) && read_double(t, "nominal_dt", &nominal) &&
                    read_double(t, "max_dt", &maximum) && nominal > 0.0 && maximum >= nominal;
    if (!ok) {
      self->host->log(self->host->host, XGC_LOG_ERROR, "invalid ctl-dfbc config");
      return XGC_ERR;
    }
    self->driver.configure(c, nominal, maximum);
    self->domain = "waiting_reference";
    return XGC_OK;
  });
}

xgc_status activate(void*) { return XGC_OK; }

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<CtlDfbc*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void*) { return XGC_OK; }

void destroy(void* p) { delete static_cast<CtlDfbc*>(p); }

const char* domain_state(void* p) { return static_cast<CtlDfbc*>(p)->domain; }

const xgc_port_decl kPorts[] = {
    {"state", XGC_PORT_IN, "xgc.rigid_state/1", XGC_QOS_STATE},
    {"ref", XGC_PORT_IN, "xgc.flat_ref/1", XGC_QOS_CONTROL},
    {"cmd", XGC_PORT_OUT, "xgc.attitude_rate_cmd/1", XGC_QOS_CONTROL},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {XGC_RT_ABI_VERSION, 3u, "ctl-dfbc", "0.1.0", kPorts, &kVtbl};

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
