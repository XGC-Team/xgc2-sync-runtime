#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <string>
#include <vector>

extern "C" const xgc_plugin_descriptor *xgc_rt_plugin_v1();

struct Sample {
  int64_t received;
  std::vector<uint8_t> data;
};
struct Boundary {
  const xgc_plugin_descriptor *plugin{xgc_rt_plugin_v1()};
  std::array<std::deque<Sample>, 3> inputs;
  std::map<uint32_t, std::vector<uint8_t>> outputs;
  std::vector<uint8_t> popped;
  std::vector<std::string> errors, warnings;
  xgc_host_api api{};
  void *instance;
  static constexpr int64_t epoch = 1000000000;

  explicit Boundary(const char *model = "fs150", const std::string &extra = "") {
    api.abi_version = XGC_RT_ABI_VERSION;
    api.abi_minor = XGC_RT_ABI_MINOR;
    api.host = this;
    api.next = [](void *p, uint32_t port, xgc_sample_view *out) {
      auto &self = *static_cast<Boundary *>(p);
      auto &queue = self.inputs.at(port);
      if (queue.empty())
        return XGC_ERR_AGAIN;
      const auto received = queue.front().received;
      self.popped = std::move(queue.front().data);
      queue.pop_front();
      *out = {};
      out->data = self.popped.data();
      out->len = self.popped.size();
      out->t_rx = received;
      return XGC_OK;
    };
    api.publish = [](void *p, uint32_t port, uint64_t, const uint8_t *data,
                     uint32_t size) {
      static_cast<Boundary *>(p)->outputs[port] = {data, data + size};
      return XGC_OK;
    };
    api.log = [](void *p, xgc_log_level level, const char *text) {
      auto &self = *static_cast<Boundary *>(p);
      (level >= XGC_LOG_ERROR ? self.errors : self.warnings).emplace_back(text);
    };
    instance = plugin->vtbl->create(&api);
    assert(instance);
    const std::string config = std::string("model = \"") + model +
                               "\"\nepoch_ns = 1000000000\noutput_ms = 1\n" +
                               extra;
    assert(plugin->vtbl->configure(instance, config.c_str()) == XGC_OK);
    assert(plugin->vtbl->activate(instance) == XGC_OK);
  }
  ~Boundary() {
    plugin->vtbl->deactivate(instance);
    plugin->vtbl->destroy(instance);
  }
  template <class T>
  void send(uint32_t port, const T &value, int64_t received) {
    auto *bytes = reinterpret_cast<const uint8_t *>(&value);
    inputs.at(port).push_back({received, {bytes, bytes + sizeof value}});
  }
  void tick(int64_t now) {
    xgc_step_ctx context{};
    context.now = now;
    context.round_start = now;
    context.round_advanced = 1;
    context.round = (now - epoch) / 1000000;
    assert(plugin->vtbl->step(instance, &context) == XGC_OK);
    assert(errors.empty());
  }
  void arm(bool value, int64_t received = epoch) {
    xgc_fcu_request_v1 request{};
    request.stamp = 1;
    request.kind = 1;
    request.arm = value;
    send(2, request, received);
  }
  void mode(const char *name, int64_t received = epoch) {
    xgc_fcu_request_v1 request{};
    request.stamp = 1;
    request.kind = 2;
    std::strcpy(request.mode, name);
    send(2, request, received);
  }
  void enable_flight() {
    arm(true);
    mode("OFFBOARD");
  }
  xgc_fcu_state_v1 fcu_state() const {
    xgc_fcu_state_v1 value;
    const auto &bytes = outputs.at(6);
    assert(bytes.size() == sizeof value);
    std::memcpy(&value, bytes.data(), sizeof value);
    return value;
  }
  xgc_pose_v1 pose() const {
    xgc_pose_v1 value;
    const auto &bytes = outputs.at(3);
    assert(bytes.size() == sizeof value);
    std::memcpy(&value, bytes.data(), sizeof value);
    return value;
  }
  xgc_twist_v1 velocity() const {
    xgc_twist_v1 value;
    const auto &bytes = outputs.at(4);
    assert(bytes.size() == sizeof value);
    std::memcpy(&value, bytes.data(), sizeof value);
    return value;
  }
};

xgc_position_target_v1 acceleration(double at, double value) {
  xgc_position_target_v1 result{};
  result.stamp = at;
  result.coordinate_frame = 1;
  result.type_mask = 3135;
  result.acceleration[0] = value;
  return result;
}

// The controller's Takeoff/Hover position-only mask.
xgc_position_target_v1 position(double at, double z) {
  xgc_position_target_v1 result{};
  result.stamp = at;
  result.coordinate_frame = 1;
  result.type_mask = 0b111111111000;
  result.position[2] = z;
  return result;
}

bool has_warning(const Boundary &boundary, const char *text) {
  for (const auto &warning : boundary.warnings)
    if (warning.find(text) != std::string::npos)
      return true;
  return false;
}

// Stream a position setpoint every 10 ms (received at its stamp) while
// ticking every millisecond from `from` (exclusive) to `to` (inclusive).
void hover(Boundary &boundary, int64_t from, int64_t to, double z) {
  for (int64_t now = from + 1000000; now <= to; now += 1000000) {
    if ((now - Boundary::epoch) % 10000000 == 0)
      boundary.send(0, position(double(now) * 1e-9, z), now);
    boundary.tick(now);
  }
}

void fcu_semantics() {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  // Only modelled modes are accepted; the others leave the mode unchanged.
  Boundary modes;
  modes.tick(epoch);
  assert(std::string(modes.fcu_state().mode) == "POSCTL");
  modes.mode("ALTCTL", epoch + ms);
  modes.tick(epoch + 2 * ms);
  assert(std::string(modes.fcu_state().mode) == "ALTCTL");
  for (const char *unsupported : {"STABILIZED", "AUTO.RTL", "AUTO.MISSION",
                                  "MANUAL", "AUTO.TAKEOFF", ""})
    modes.mode(unsupported, epoch + 3 * ms);
  modes.tick(epoch + 4 * ms);
  assert(std::string(modes.fcu_state().mode) == "ALTCTL");
  assert(has_warning(modes, "STABILIZED is not modelled; ALTCTL kept"));
  // OFFBOARD without a setpoint stream is refused.
  modes.mode("OFFBOARD", epoch + 5 * ms);
  modes.tick(epoch + 6 * ms);
  assert(std::string(modes.fcu_state().mode) == "ALTCTL");
  assert(has_warning(modes, "OFFBOARD refused without a fresh setpoint"));

  // Takeoff to 2 m in OFFBOARD; an in-air disarm is refused; AUTO.LAND
  // descends and disarms on touchdown (the old plant hovered, armed).
  Boundary land;
  land.send(0, position(1.0, 2.0), epoch);
  land.enable_flight();
  land.tick(epoch);
  hover(land, epoch, epoch + 8000 * ms, 2.0);
  assert(std::abs(land.pose().position[2] - 2.0) < 1e-3);
  assert(land.fcu_state().armed && std::string(land.fcu_state().mode) == "OFFBOARD");
  land.arm(false, epoch + 8000 * ms);
  land.tick(epoch + 8001 * ms);
  assert(land.fcu_state().armed);
  assert(has_warning(land, "disarm refused while airborne"));
  land.mode("AUTO.LAND", epoch + 8001 * ms);
  int64_t now = epoch + 8001 * ms;
  while (land.fcu_state().armed && now < epoch + 20000 * ms) {
    now += ms;
    land.tick(now);
  }
  assert(!land.fcu_state().armed);
  assert(std::string(land.fcu_state().mode) == "AUTO.LAND");
  assert(land.pose().position[2] == 0.0 && land.velocity().linear[2] == 0.0);
  // About 2 m at 0.7 m/s after the 0.3 s velocity response.
  assert(now - epoch - 8001 * ms > 2800 * ms && now - epoch - 8001 * ms < 3600 * ms);

  // A stopped setpoint stream falls back to AUTO.LOITER after 500 ms and the
  // plant brakes instead of integrating the last acceleration forever.
  Boundary silent;
  silent.send(0, acceleration(1.0, 0.5), epoch);
  silent.enable_flight();
  for (int i = 0; i <= 499; ++i)
    silent.tick(epoch + i * ms);
  assert(std::string(silent.fcu_state().mode) == "OFFBOARD");
  silent.tick(epoch + 501 * ms);
  assert(std::string(silent.fcu_state().mode) == "AUTO.LOITER");
  assert(has_warning(silent, "holding in AUTO.LOITER"));
  silent.tick(epoch + 60000 * ms);
  assert(std::abs(silent.velocity().linear[0]) < 1e-9);
  assert(silent.pose().position[0] < 0.2);
  // A configured timeout is honoured.
  Boundary patient("fs150", "offboard_timeout_ms = 2000\n");
  patient.send(0, acceleration(1.0, 0.5), epoch);
  patient.enable_flight();
  patient.tick(epoch + 1999 * ms);
  assert(std::string(patient.fcu_state().mode) == "OFFBOARD");
  patient.tick(epoch + 2001 * ms);
  assert(std::string(patient.fcu_state().mode) == "AUTO.LOITER");
}

int main() {
  // Different wake schedules produce the same plant trajectory when the
  // actual arrivals are equal. Future controls cannot affect earlier steps.
  Boundary frequent, delayed;
  for (auto *boundary : {&frequent, &delayed}) {
    boundary->enable_flight();
    boundary->send(0, acceleration(1.0, 2.0), Boundary::epoch);
    boundary->send(0, acceleration(1.015, 4.0), Boundary::epoch + 10000000);
    boundary->tick(Boundary::epoch);
  }
  for (int i = 1; i <= 20; ++i)
    frequent.tick(Boundary::epoch + i * 1000000);
  delayed.tick(Boundary::epoch + 20000000);
  assert(std::abs(frequent.pose().position[0] - delayed.pose().position[0]) <
         1e-14);
  assert(std::abs(delayed.pose().position[0] - 0.000425) < 1e-14);
  assert(std::abs(delayed.velocity().linear[0] - 0.05) < 1e-14);

  // Old header but late reception: only the remaining interval receives it.
  Boundary late;
  late.send(0, acceleration(1.0, 0.0), Boundary::epoch);
  late.enable_flight();
  late.send(0, acceleration(1.0, 2.0), Boundary::epoch + 10000000);
  late.tick(Boundary::epoch + 20000000);
  assert(std::abs(late.pose().position[0] - 0.0001) < 1e-14);
  late.send(0, acceleration(1.0, 0.0), Boundary::epoch + 5000000);
  late.tick(Boundary::epoch + 30000000);
  assert(std::abs(late.pose().position[0] - 0.0003) < 1e-14);

  // Drift-free command/step times over many exact-nanosecond boundaries.
  Boundary scout("scout"), mecanum("mecanum");
  for (int i = 0; i != 10000; ++i) {
    const auto now = Boundary::epoch + int64_t(i) * 1000000;
    xgc_twist_v1 command{};
    command.stamp = double(now) * 1e-9;
    command.linear[0] = 1.0;
    command.linear[1] = 0.5;
    for (auto *boundary : {&scout, &mecanum}) {
      boundary->send(1, command, now);
      boundary->tick(now);
    }
  }
  assert(scout.pose().position[1] == 0.0);
  assert(std::abs(mecanum.pose().position[1] - 4.999) < 0.002);
  assert(scout.outputs.count(6) == 0 && mecanum.outputs.count(6) == 0);
  fcu_semantics();
  std::cout
      << "lightweight ABI arrival timing and independent time grid: passed\n";
}
