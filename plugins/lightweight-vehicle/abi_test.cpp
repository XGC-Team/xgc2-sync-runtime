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
  std::vector<std::string> errors;
  xgc_host_api api{};
  void *instance;
  static constexpr int64_t epoch = 1000000000;

  explicit Boundary(const char *model = "fs150") {
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
    api.log = [](void *p, xgc_log_level, const char *text) {
      static_cast<Boundary *>(p)->errors.emplace_back(text);
    };
    instance = plugin->vtbl->create(&api);
    assert(instance);
    const std::string config = std::string("model = \"") + model +
                               "\"\nepoch_ns = 1000000000\noutput_ms = 1\n";
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
  void enable_flight() {
    xgc_fcu_request_v1 request{};
    request.stamp = 1;
    request.kind = 1;
    request.arm = 1;
    send(2, request, epoch);
    request.kind = 2;
    std::strcpy(request.mode, "OFFBOARD");
    send(2, request, epoch);
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
  std::cout
      << "lightweight ABI arrival timing and independent time grid: passed\n";
}
