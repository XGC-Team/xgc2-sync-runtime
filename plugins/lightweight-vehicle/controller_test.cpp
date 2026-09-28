// Numerical ABI coupling with the actual, separately built ctl-px4 ELF.
// No real FCU/ROS/network; this verifies the unchanged SMC lifecycle on the
// plant.
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"
#include <cassert>
#include <cmath>
#include <cstring>
#include <deque>
#include <dlfcn.h>
#include <iostream>
#include <map>
#include <string>
#include <vector>

struct Module {
  struct Sample {
    int64_t at;
    std::vector<uint8_t> data;
  };
  struct Link {
    uint32_t out;
    Module *target;
    uint32_t in;
  };
  void *library;
  void *instance;
  const xgc_plugin_descriptor *descriptor;
  xgc_host_api host{};
  std::map<std::string, uint32_t> ports;
  std::map<uint32_t, std::deque<Sample>> inbox;
  std::map<uint32_t, std::vector<uint8_t>> last;
  std::vector<uint8_t> popped;
  std::vector<Link> links;
  int64_t now{1000000000};
  int smc_outputs{0}, attitude_outputs{0};
  Module(const char *path, const char *config) {
    library = dlopen(path, RTLD_NOW | RTLD_LOCAL);
    if (!library) {
      std::cerr << dlerror() << "\n";
      std::abort();
    }
    auto get = reinterpret_cast<const xgc_plugin_descriptor *(*)()>(
        dlsym(library, "xgc_rt_plugin_v1"));
    descriptor = get();
    for (uint32_t i = 0; i < descriptor->port_count; ++i)
      ports[descriptor->ports[i].name] = i;
    host.abi_version = 1;
    host.abi_minor = 2;
    host.host = this;
    host.now = [](void *p) { return static_cast<Module *>(p)->now; };
    host.next = [](void *p, uint32_t port, xgc_sample_view *v) {
      auto &s = *static_cast<Module *>(p);
      auto &q = s.inbox[port];
      if (q.empty())
        return XGC_ERR_AGAIN;
      auto t = q.front().at;
      s.popped = std::move(q.front().data);
      q.pop_front();
      *v = {};
      v->data = s.popped.data();
      v->len = s.popped.size();
      v->t_rx = v->t_produce = v->t_tx = t;
      return XGC_OK;
    };
    host.publish = [](void *p, uint32_t port, uint64_t, const uint8_t *data,
                      uint32_t len) {
      auto &s = *static_cast<Module *>(p);
      s.last[port] = {data, data + len};
      if (std::strcmp(s.descriptor->ports[port].name, "setpoint") == 0 &&
          len == sizeof(xgc_position_target_v1)) {
        xgc_position_target_v1 v;
        std::memcpy(&v, data, len);
        if (v.type_mask == 3135)
          ++s.smc_outputs;
      }
      if (std::strcmp(s.descriptor->ports[port].name, "attitude_rate") == 0)
        ++s.attitude_outputs;
      for (auto &l : s.links)
        if (l.out == port)
          l.target->inbox[l.in].push_back({s.now, {data, data + len}});
      return XGC_OK;
    };
    host.log = [](void *, xgc_log_level level, const char *msg) {
      if (level >= XGC_LOG_WARN)
        std::cerr << msg << "\n";
    };
    host.request_degrade = [](void *, const char *reason) {
      std::cerr << "degrade " << reason << "\n";
    };
    host.request_recover = [](void *) {};
    host.node_id = [](void *) -> uint16_t { return 0; };
    host.port_origins = [](void *, uint32_t, uint16_t *out,
                           uint32_t cap) -> uint32_t {
      if (cap)
        out[0] = 0;
      return 1;
    };
    instance = descriptor->vtbl->create(&host);
    assert(instance);
    assert(descriptor->vtbl->configure(instance, config) == XGC_OK);
    assert(descriptor->vtbl->activate(instance) == XGC_OK);
  }
  ~Module() {
    descriptor->vtbl->deactivate(instance);
    descriptor->vtbl->destroy(instance);
    dlclose(library);
  }
  void link(const char *out, Module &target, const char *in) {
    links.push_back({ports.at(out), &target, target.ports.at(in)});
  }
  template <class T> void input(const char *name, const T &v) {
    auto *b = reinterpret_cast<const uint8_t *>(&v);
    inbox[ports.at(name)].push_back({now, {b, b + sizeof v}});
  }
  template <class T> T output(const char *name) {
    T v{};
    auto &b = last[ports.at(name)];
    if (!b.empty()) {
      assert(b.size() == sizeof v);
      std::memcpy(&v, b.data(), sizeof v);
    }
    return v;
  }
  void tick(int64_t t) {
    now = t;
    xgc_step_ctx c{};
    c.now = c.round_start = t;
    c.round = (t - 1000000000) / 1000000;
    c.round_advanced = 1;
    assert(descriptor->vtbl->step(instance, &c) == XGC_OK);
  }
  void command(const char *text) {
    char b[64]{};
    std::strcpy(b, text);
    input("command", b);
  }
};

int main(int argc, char **argv) {
  assert(argc == 3);
  Module plant(
      argv[1],
      "model=\"fs150\"\nepoch_ns=1000000000\nstep_ms=1\noutput_ms=10\n");
  Module ctl(argv[2], "time_source=\"session\"\ntracking_backend="
                      "\"smc\"\ntakeoff_altitude=1\nplanning_period=0.1\n");
  plant.link("pose", ctl, "local_pose");
  plant.link("pose", ctl, "vrpn_pose");
  plant.link("velocity", ctl, "local_velocity");
  plant.link("imu", ctl, "imu");
  plant.link("fcu_state", ctl, "fcu_state");
  ctl.link("setpoint", plant, "setpoint");
  ctl.link("fcu_request", plant, "fcu_request");
  bool takeoff = false, custom = false, land = false, landed = false;
  int custom_ms = -1;
  std::string last_state;
  double error = -1;
  for (int ms = 0; ms != 60000; ++ms) {
    const int64_t now = 1000000000 + int64_t(ms) * 1000000;
    ctl.now = plant.now = now;
    const auto status = ctl.output<xgc_controller_status_v1>("status");
    const std::string state(status.state);
    if (state != last_state) {
      std::cout << "state " << ms << " " << state << "\n";
      last_state = state;
    }
    if (!takeoff && state == "Ready") {
      ctl.command("takeoff");
      takeoff = true;
    }
    if (takeoff && !custom && state == "Hover") {
      custom = true;
      custom_ms = ms;
      ctl.command("custom1");
    }
    if (custom && !land && ((ms - custom_ms) % 100) == 0) {
      const double elapsed = double(ms - custom_ms) * .001,
                   q = std::min(1.0, elapsed / 6.0);
      xgc_position_target_v1 p{};
      p.stamp = double(now) * 1e-9;
      p.coordinate_frame = 1;
      p.type_mask = 3072;
      p.position[2] = 1;
      p.position[0] =
          10 * q * q * q - 15 * q * q * q * q + 6 * q * q * q * q * q;
      if (q < 1) {
        p.velocity[0] = (30 * q * q - 60 * q * q * q + 30 * q * q * q * q) / 6;
        p.acceleration[0] = (60 * q - 180 * q * q + 120 * q * q * q) / 36;
      }
      ctl.input("alg_setpoint", p);
    }
    if (custom && !land && ms - custom_ms >= 10000) {
      auto p = plant.output<xgc_pose_v1>("pose");
      error = std::hypot(p.position[0] - 1, p.position[2] - 1);
      std::cout << "tracking_error " << error << "\n";
      assert(state == "Custom1");
      assert(error < .03);
      ctl.command("land");
      land = true;
    }
    plant.tick(now);
    ctl.tick(now);
    const auto pose = plant.output<xgc_pose_v1>("pose");
    for (double v : pose.position)
      assert(std::isfinite(v));
    if (land && !plant.output<xgc_fcu_state_v1>("fcu_state").armed &&
        pose.position[2] < .03) {
      landed = true;
      break;
    }
  }
  assert(takeoff && custom && land && landed);
  assert(ctl.smc_outputs > 100);
  assert(ctl.attitude_outputs == 0);
  std::cout << "PASS SMC unchanged controller + lightweight plugin "
               "takeoff/tracking/landing; error="
            << error << " acceleration_outputs=" << ctl.smc_outputs << "\n";
}
