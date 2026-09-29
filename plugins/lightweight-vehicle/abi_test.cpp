#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstring>
#include <deque>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

extern "C" const xgc_plugin_descriptor *xgc_rt_plugin_v1();

struct Sample {
  int64_t received;
  std::vector<uint8_t> data;
};
struct Boundary {
  const xgc_plugin_descriptor *plugin{xgc_rt_plugin_v1()};
  std::array<std::deque<Sample>, XGC_RT_MAX_PORTS> inputs;
  std::map<uint32_t, std::vector<uint8_t>> outputs;
  // Every publication in order: (port, bytes).
  std::vector<std::pair<uint32_t, std::vector<uint8_t>>> published;
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
      auto &self = *static_cast<Boundary *>(p);
      self.outputs[port] = {data, data + size};
      self.published.emplace_back(port, std::vector<uint8_t>(data, data + size));
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

void bounded_future_controls() {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  // A stamp further ahead of its receipt than max_future_ms is a clock-domain
  // error: it is dropped and reported, never queued for later.
  Boundary future("fs150", "max_future_ms = 100\n");
  future.send(0, acceleration(1.0, 0.0), epoch);
  future.enable_flight();
  future.send(0, acceleration(1.1005, 3.0), epoch + ms); // 99.5 ms ahead
  future.send(0, acceleration(3.0, 5.0), epoch + ms);    // 2 s ahead
  future.tick(epoch + 150 * ms);
  assert(has_warning(future, "dropped 1 control(s) stamped beyond max_future_ms"));
  // The in-window control applied at 100.5 ms -> the 101 ms grid boundary.
  const double t = 0.049;
  assert(std::abs(future.velocity().linear[0] - 3.0 * t) < 1e-12);
  future.tick(epoch + 400 * ms);
  assert(std::abs(future.velocity().linear[0] - 3.0 * 0.299) < 1e-12);

  // At most max_pending future controls wait; the rest are dropped, counted.
  Boundary full("fs150", "max_pending = 4\n");
  full.send(0, acceleration(1.0, 0.0), epoch);
  full.enable_flight();
  full.tick(epoch);
  full.tick(epoch + ms); // applies the controls due at the epoch
  for (int i = 0; i != 10; ++i)
    full.send(0, acceleration(1.0 + 0.01 * (i + 1), double(i)), epoch + ms);
  full.tick(epoch + 2 * ms);
  assert(has_warning(full, "dropped 1 control(s): max_pending"));
  assert(has_warning(full, "dropped 4 control(s): max_pending"));
  assert(!has_warning(full, "dropped 3 control(s): max_pending"));
  // Only the four queued accelerations (0..3) ever act.
  full.tick(epoch + 60 * ms);
  const double expected = 0.01 * (1.0 + 2.0) + 0.02 * 3.0;
  assert(std::abs(full.velocity().linear[0] - expected) < 1e-12);
}

constexpr uint32_t kBlock = 8; // ports per robot

// Robot r's publications in order, as (port within its block, bytes).
std::vector<std::pair<uint32_t, std::vector<uint8_t>>>
robot_outputs(const Boundary &boundary, uint32_t robot) {
  std::vector<std::pair<uint32_t, std::vector<uint8_t>>> result;
  for (const auto &item : boundary.published)
    if (item.first / kBlock == robot)
      result.emplace_back(item.first % kBlock, item.second);
  return result;
}

double stamp_of(const std::vector<uint8_t> &bytes) {
  double stamp;
  std::memcpy(&stamp, bytes.data(), sizeof stamp);
  return stamp;
}

bool configures(const std::string &config) {
  const auto *plugin = xgc_rt_plugin_v1();
  xgc_host_api api{};
  api.abi_version = XGC_RT_ABI_VERSION;
  api.abi_minor = XGC_RT_ABI_MINOR;
  api.log = [](void *, xgc_log_level, const char *) {};
  void *instance = plugin->vtbl->create(&api);
  assert(instance);
  const bool ok = plugin->vtbl->configure(instance, config.c_str()) == XGC_OK;
  plugin->vtbl->destroy(instance);
  return ok;
}

// Robot r owns block r; block 0 keeps the single-robot names and indices.
void port_blocks() {
  const auto *plugin = xgc_rt_plugin_v1();
  assert(plugin->port_count == XGC_RT_MAX_PORTS);
  const char *names[kBlock] = {"setpoint", "cmd_vel",   "fcu_request",
                               "pose",     "velocity",  "imu",
                               "fcu_state", "paired_state"};
  for (uint32_t kind = 0; kind != kBlock; ++kind) {
    const auto &first = plugin->ports[kind];
    const auto &last = plugin->ports[7 * kBlock + kind];
    assert(std::string(first.name) == names[kind]);
    assert(std::string(last.name) == std::string(names[kind]) + "_7");
    assert(std::string(last.schema_id) == first.schema_id);
    assert(last.qos == first.qos);
  }
  assert(plugin->ports[3].dir == XGC_PORT_OUT);
  assert(plugin->ports[kBlock + 3].dir == XGC_PORT_OUT_OPTIONAL);
  const std::string base = "model = \"fs150\"\nepoch_ns = 1000000000\n";
  assert(configures(base + "robots = 8\n"));
  assert(!configures(base + "robots = 9\n"));
  assert(!configures(base + "robots = 0\n"));
  assert(!configures(base + "robots = 2\ninitial_pose = [0, 0, 0, 0]\n"));
  assert(!configures(base + "robots = 2\ninitial_poses = [0, 0, 0, 0]\n"));
  assert(!configures(base + "initial_pose = [0, 0, 0, 0]\n"
                            "initial_poses = [0, 0, 0, 0]\n"));
  assert(configures(base + "robots = 2\n"
                           "initial_poses = [0, 0, 0, 0, 1, 1, 0, 0]\n"));
}

// A batch fed the controls a set of independent single-robot instances get,
// with the same receive times, publishes byte-identical states and stamps.
// A batch woken on another schedule matches them at every common stamp.
void batch_matches_independent_robots(const char *model) {
  constexpr int64_t ms = 1000000, epoch = Boundary::epoch;
  constexpr uint32_t n = 5;
  const bool flight = std::string(model) == "fs150";
  std::vector<std::string> pose;
  std::string poses;
  for (uint32_t r = 0; r != n; ++r) {
    pose.push_back(std::to_string(0.5 * r) + ", " + std::to_string(-0.25 * r) +
                   ", " + std::to_string(0.1 * r) + ", " +
                   std::to_string(0.3 * r));
    poses += (r ? ", " : "") + pose.back();
  }
  const std::string batch_config =
      "robots = 5\ninitial_poses = [" + poses + "]\n";
  Boundary batch(model, batch_config), sparse(model, batch_config);
  std::vector<std::unique_ptr<Boundary>> singles;
  for (uint32_t r = 0; r != n; ++r)
    singles.push_back(std::make_unique<Boundary>(
        model, "initial_pose = [" + pose[r] + "]\n"));

  struct Event {
    int64_t received;
    uint32_t robot, port;
    std::vector<uint8_t> bytes;
  };
  std::vector<Event> events;
  auto add = [&](int64_t received, uint32_t robot, uint32_t port,
                 const auto &value) {
    auto *bytes = reinterpret_cast<const uint8_t *>(&value);
    events.push_back({received, robot, port, {bytes, bytes + sizeof value}});
  };
  auto request = [&](int64_t received, uint32_t robot, uint32_t kind,
                     const char *mode, uint32_t arm) {
    xgc_fcu_request_v1 value{};
    value.stamp = double(received) * 1e-9;
    value.kind = kind;
    value.arm = arm;
    std::strcpy(value.mode, mode);
    add(received, robot, 2, value);
  };
  for (uint32_t r = 0; r != n; ++r) {
    for (int64_t t = int64_t(r) * ms; t < 1400 * ms; t += 20 * ms) {
      const int64_t received = epoch + t;
      const double phase = 1e-9 * double(t) + r;
      if (flight) {
        if (r == 1 && t > 800 * ms)
          break; // the stream stops: offboard timeout, AUTO.LOITER
        auto value = acceleration(double(received) * 1e-9, 0.3 * std::sin(phase));
        value.acceleration[1] = 0.1 * r;
        value.acceleration[2] = 0.4 * std::cos(phase);
        value.type_mask = 3135 & ~2048; // with a yaw rate
        value.yaw_rate = 0.2 * r;
        if (r == 3)
          value.stamp -= 0.015; // late: effective at receipt
        if (r == 4 && t == 404 * ms)
          value.stamp += 0.030; // future: effective 30 ms after receipt
        add(received, r, 0, value);
        if (t == int64_t(r) * ms) {
          request(received, r, 1, "", 1);
          request(received, r, 2, "OFFBOARD", 0);
        }
      } else {
        xgc_twist_v1 value{};
        value.stamp = double(received) * 1e-9;
        value.linear[0] = 0.5 + 0.1 * r;
        value.linear[1] = 0.2 * std::sin(phase);
        value.angular[2] = 0.3 * std::cos(phase);
        add(received, r, 1, value);
      }
    }
  }
  if (flight) {
    request(epoch + 600 * ms, 2, 2, "AUTO.LAND", 0);
    request(epoch + 1000 * ms, 0, 1, "", 0); // in-air disarm: refused
    request(epoch + 1100 * ms, 3, 2, "STABILIZED", 0);
  }
  std::stable_sort(events.begin(), events.end(),
                   [](const Event &a, const Event &b) {
                     return a.received < b.received;
                   });

  // Irregular reference wakes that include every 50 ms point; sparse wakes
  // every 50 ms only.
  std::vector<int64_t> wakes;
  for (int64_t t = epoch, k = 0; t < epoch + 1500 * ms; ++k)
    wakes.push_back(t += (1 + (k * 7919) % 7) * ms);
  for (int64_t t = epoch; t <= epoch + 1500 * ms; t += 50 * ms)
    wakes.push_back(t);
  std::sort(wakes.begin(), wakes.end());
  wakes.erase(std::unique(wakes.begin(), wakes.end()), wakes.end());

  auto run = [&](const std::vector<int64_t> &schedule, auto &&deliver,
                 auto &&tick) {
    size_t next = 0;
    for (int64_t now : schedule) {
      for (; next != events.size() && events[next].received <= now; ++next)
        deliver(events[next]);
      tick(now);
    }
  };
  run(
      wakes,
      [&](const Event &e) {
        batch.inputs[e.robot * kBlock + e.port].push_back(
            {e.received, e.bytes});
        singles[e.robot]->inputs[e.port].push_back({e.received, e.bytes});
      },
      [&](int64_t now) {
        batch.tick(now);
        for (auto &single : singles)
          single->tick(now);
      });
  std::vector<int64_t> every_50;
  for (int64_t t = epoch; t <= epoch + 1500 * ms; t += 50 * ms)
    every_50.push_back(t);
  run(
      every_50,
      [&](const Event &e) {
        sparse.inputs[e.robot * kBlock + e.port].push_back(
            {e.received, e.bytes});
      },
      [&](int64_t now) { sparse.tick(now); });

  size_t single_warnings = 0;
  for (uint32_t r = 0; r != n; ++r) {
    const auto independent = robot_outputs(*singles[r], 0);
    assert(independent.size() > 1000);
    assert(robot_outputs(batch, r) == independent);
    const auto late = robot_outputs(sparse, r);
    assert(late.size() == 31 * (flight ? 5 : 3));
    for (const auto &[kind, bytes] : late) {
      bool found = false;
      for (const auto &item : independent)
        if (item.first == kind && stamp_of(item.second) == stamp_of(bytes)) {
          assert(item.second == bytes);
          found = true;
        }
      assert(found);
    }
    single_warnings += singles[r]->warnings.size();
  }
  assert(batch.warnings.size() == single_warnings);
  if (flight) {
    // The scenario exercised the FCU paths it names.
    const auto state = [&](uint32_t r) {
      xgc_fcu_state_v1 value;
      std::memcpy(&value, singles[r]->outputs.at(6).data(), sizeof value);
      return value;
    };
    assert(std::string(state(1).mode) == "AUTO.LOITER");
    assert(std::string(state(2).mode) == "AUTO.LAND");
    assert(state(0).armed && std::string(state(3).mode) == "OFFBOARD");
    assert(has_warning(batch, "robot 1: no setpoint for offboard_timeout_ms"));
    assert(has_warning(batch, "robot 0: disarm refused while airborne"));
    assert(has_warning(batch, "robot 3: FCU mode STABILIZED is not modelled"));
  }
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
  bounded_future_controls();
  port_blocks();
  for (const char *model : {"fs150", "scout", "mecanum"})
    batch_matches_independent_robots(model);
  std::cout
      << "lightweight ABI arrival timing and independent time grid: passed\n";
}
