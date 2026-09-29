#include "flat_config.hpp"
#include "vehicle_model.hpp"
#include "xgc_dmpc_planner_v1.h"
#include "xgc_rt.h"
#include "xgc_schemas_v1.h"

#include <array>
#include <cstring>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// One instance advances a batch of same-type robots (one by default). Their
// model state is one contiguous vector; each step reads the host time once,
// advances every robot to that one grid point with the same per-robot function
// a single-robot instance uses, and publishes only at the output period.
// Robot r owns port block r: `setpoint` ... `paired_state` for robot 0 and
// `setpoint_r` ... `paired_state_r` after it. The ABI allows 64 ports, so one
// instance holds at most 8 robots of 8 ports.
namespace {
enum Port {
  Setpoint,
  VelocityCommand,
  FcuRequest,
  Pose,
  Velocity,
  Imu,
  FcuState,
  PairedState,
  kPortsPerRobot
};
constexpr uint32_t kMaxRobots = XGC_RT_MAX_PORTS / kPortsPerRobot;

struct Input {
  int64_t at;
  Port port;
  std::array<uint8_t, sizeof(xgc_position_target_v1)> data{};
};

int64_t nanoseconds(double seconds) {
  if (!std::isfinite(seconds) || seconds < 0 ||
      seconds >= double(INT64_MAX) * 1e-9)
    throw std::invalid_argument("lightweight-vehicle: invalid command time");
  return static_cast<int64_t>(std::llround(seconds * 1e9));
}

// Counts a repeated condition; true on the 1st, 2nd, 4th, 8th... occurrence so
// a persistent fault stays visible without flooding the health log.
struct Occurrences {
  uint64_t count{0};
  bool report() {
    ++count;
    return (count & (count - 1)) == 0;
  }
};

bool flight_mode(const std::string &name, xgc_lightweight::FlightMode *mode) {
  using xgc_lightweight::FlightMode;
  if (name == "OFFBOARD")
    *mode = FlightMode::Offboard;
  else if (name == "POSCTL" || name == "ALTCTL" || name == "AUTO.LOITER")
    *mode = FlightMode::Hold;
  else if (name == "AUTO.LAND")
    *mode = FlightMode::Land;
  else
    return false;
  return true;
}

// The time grid and limits every robot of the instance shares.
struct Grid {
  const xgc_host_api *host;
  uint32_t robots{1};
  int64_t epoch{0}, time{0}, step_ns{1000000}, output_ns{10000000},
      next_output{0}, max_future_ns{1000000000};
  size_t max_pending{1024};

  void log(xgc_log_level level, uint32_t robot,
           const std::string &message) const {
    const auto who =
        robots > 1 ? "robot " + std::to_string(robot) + ": " : std::string();
    host->log(host->host, level,
              ("lightweight-vehicle: " + who + message).c_str());
  }
  template <class T>
  void publish(uint32_t robot, Port port, uint64_t round,
               const T &value) const {
    if (host->publish(host->host, robot * kPortsPerRobot + port, round,
                      reinterpret_cast<const uint8_t *>(&value),
                      sizeof value) != XGC_OK)
      throw std::runtime_error("lightweight-vehicle: state publish failed");
  }
};

// One robot's received controls, ordered by the grid time they take effect.
struct Commands {
  std::deque<Input> pending;
  Occurrences future_drops, full_drops;

  void drain(const Grid &grid, uint32_t robot, Port port, size_t size) {
    xgc_sample_view sample{};
    while (grid.host->next(grid.host->host, robot * kPortsPerRobot + port,
                           &sample) == XGC_OK) {
      if (sample.len != size)
        throw std::invalid_argument("lightweight-vehicle: wrong input size");
      double stamp;
      std::memcpy(&stamp, sample.data, sizeof(stamp));
      const int64_t effective = nanoseconds(stamp);
      int64_t ahead;
      const bool unrepresentable =
          __builtin_sub_overflow(effective, sample.t_rx, &ahead);
      if (unrepresentable || ahead > grid.max_future_ns) {
        if (future_drops.report())
          grid.log(XGC_LOG_WARN, robot,
                   "dropped " + std::to_string(future_drops.count) +
                       " control(s) stamped beyond max_future_ms after "
                       "receipt (last " +
                       (unrepresentable ? std::string("out of range")
                                        : std::to_string(ahead / 1000000) +
                                              " ms ahead") +
                       ")");
        continue;
      }
      if (pending.size() >= grid.max_pending) {
        if (full_drops.report())
          grid.log(XGC_LOG_WARN, robot,
                   "dropped " + std::to_string(full_drops.count) +
                       " control(s): max_pending controls already queued");
        continue;
      }
      // Never use a future control early or rewrite an already integrated past.
      Input input{std::max({grid.time, sample.t_rx, effective}), port, {}};
      std::memcpy(input.data.data(), sample.data, size);
      auto at = std::upper_bound(
          pending.begin(), pending.end(), input.at,
          [](int64_t t, const Input &item) { return t < item.at; });
      pending.insert(at, input);
    }
  }
};

xgc_twist_v1 body_velocity(const Input &input) {
  xgc_twist_v1 value;
  std::memcpy(&value, input.data.data(), sizeof value);
  if (!std::isfinite(value.linear[0]) || !std::isfinite(value.linear[1]) ||
      !std::isfinite(value.angular[2]))
    throw std::invalid_argument("lightweight-vehicle: nonfinite body velocity");
  return value;
}

void planar_output(const Grid &grid, uint32_t robot, uint64_t round,
                   const xgc2_math::Pose2 &state, double z,
                   const Eigen::Vector2d &body, double yaw_rate) {
  const double stamp = double(grid.time) * 1e-9;
  xgc_pose_v1 pose{};
  pose.stamp = stamp;
  pose.position[0] = state.position.x();
  pose.position[1] = state.position.y();
  pose.position[2] = z;
  pose.q_wxyz[0] = std::cos(state.yaw * 0.5);
  pose.q_wxyz[3] = std::sin(state.yaw * 0.5);
  xgc_twist_v1 velocity{};
  velocity.stamp = stamp;
  const auto world = (xgc2_math::rotationMatrix2(state.yaw) * body).eval();
  velocity.linear[0] = world.x();
  velocity.linear[1] = world.y();
  velocity.angular[2] = yaw_rate;
  xgc_dmpc_paired_state_v1 paired{};
  paired.pose_stamp_sec = paired.twist_stamp_sec = stamp;
  std::copy(pose.position, pose.position + 3, paired.position);
  paired.orientation_xyzw[2] = pose.q_wxyz[3];
  paired.orientation_xyzw[3] = pose.q_wxyz[0];
  std::copy(velocity.linear, velocity.linear + 3, paired.linear_velocity);
  grid.publish(robot, Pose, round, pose);
  grid.publish(robot, Velocity, round, velocity);
  grid.publish(robot, PairedState, round, paired);
}

struct FlightRobot {
  xgc_lightweight::FlightModel model;
  Commands commands;
  std::string fcu_mode{"POSCTL"};
  Occurrences refused_disarm, refused_mode, refused_offboard;

  FlightRobot(const double *initial, double offboard_timeout_s)
      : model(Eigen::Vector3d(initial[0], initial[1], initial[2]), initial[3],
              offboard_timeout_s) {}

  void apply(const Grid &grid, uint32_t robot, const Input &input, int64_t) {
    if (input.port == VelocityCommand)
      throw std::invalid_argument(
          "lightweight-vehicle: ground velocity on flight model");
    if (input.port == Setpoint) {
      xgc_position_target_v1 wire;
      std::memcpy(&wire, input.data.data(), sizeof wire);
      // FORCE only changes enabled acceleration axes. The controller's
      // position-only Takeoff mask sets every unused bit, including FORCE.
      const bool force_input =
          (wire.type_mask & 512) && (wire.type_mask & 448) != 448;
      if (wire.coordinate_frame != 1 || force_input)
        throw std::invalid_argument(
            "lightweight-vehicle: requires world ENU acceleration, not force");
      xgc_lightweight::FlightSetpoint value;
      for (int i = 0; i != 3; ++i) {
        value.position_enabled[i] = !(wire.type_mask & (1 << i));
        value.velocity_enabled[i] = !(wire.type_mask & (8 << i));
        value.acceleration_enabled[i] = !(wire.type_mask & (64 << i));
        if ((value.position_enabled[i] && !std::isfinite(wire.position[i])) ||
            (value.velocity_enabled[i] && !std::isfinite(wire.velocity[i])) ||
            (value.acceleration_enabled[i] &&
             !std::isfinite(wire.acceleration[i])))
          throw std::invalid_argument(
              "lightweight-vehicle: nonfinite enabled setpoint axis");
        value.position[i] = wire.position[i];
        value.velocity[i] = wire.velocity[i];
        value.acceleration[i] = wire.acceleration[i];
      }
      value.yaw_enabled = !(wire.type_mask & 1024);
      value.yaw = wire.yaw;
      value.yaw_rate = (wire.type_mask & 2048) ? 0.0 : wire.yaw_rate;
      if ((value.yaw_enabled && !std::isfinite(value.yaw)) ||
          !std::isfinite(value.yaw_rate))
        throw std::invalid_argument(
            "lightweight-vehicle: nonfinite yaw command");
      model.setpoint(value);
      return;
    }
    xgc_fcu_request_v1 request;
    std::memcpy(&request, input.data.data(), sizeof request);
    // A refused request leaves armed/mode unchanged; the next fcu_state
    // shows the refusal, as PX4's does.
    if (request.kind == 1) {
      if (!model.request_arm(request.arm != 0) && refused_disarm.report())
        grid.log(XGC_LOG_WARN, robot,
                 "disarm refused while airborne (" +
                     std::to_string(refused_disarm.count) + " time(s))");
    } else if (request.kind == 2) {
      const auto end = static_cast<const char *>(
          std::memchr(request.mode, 0, sizeof request.mode));
      if (!end)
        throw std::invalid_argument(
            "lightweight-vehicle: unterminated FCU mode");
      const std::string name(request.mode,
                             static_cast<size_t>(end - request.mode));
      xgc_lightweight::FlightMode mode;
      if (!flight_mode(name, &mode)) {
        if (refused_mode.report())
          grid.log(XGC_LOG_WARN, robot,
                   "FCU mode " + name + " is not modelled; " + fcu_mode +
                       " kept (" + std::to_string(refused_mode.count) +
                       " unmodelled mode request(s))");
      } else if (!model.request_mode(mode)) {
        if (refused_offboard.report())
          grid.log(XGC_LOG_WARN, robot,
                   "OFFBOARD refused without a fresh setpoint; " + fcu_mode +
                       " kept (" + std::to_string(refused_offboard.count) +
                       " time(s))");
      } else
        fcu_mode = name;
    } else
      throw std::invalid_argument(
          "lightweight-vehicle: unsupported FCU request");
  }

  void step(const Grid &grid, uint32_t robot, int64_t) {
    const auto event = model.step(double(grid.step_ns) * 1e-9);
    if (event == xgc_lightweight::FlightEvent::OffboardLost) {
      fcu_mode = "AUTO.LOITER";
      grid.log(XGC_LOG_WARN, robot,
               "no setpoint for offboard_timeout_ms; holding in AUTO.LOITER");
    } else if (event == xgc_lightweight::FlightEvent::Landed)
      grid.log(XGC_LOG_INFO, robot, "AUTO.LAND touchdown; disarmed");
  }

  void output(const Grid &grid, uint32_t robot, uint64_t round) const {
    const double stamp = double(grid.time) * 1e-9;
    const double yaw = model.yaw();
    xgc_pose_v1 pose{};
    pose.stamp = stamp;
    xgc_twist_v1 velocity{};
    velocity.stamp = stamp;
    for (int i = 0; i != 3; ++i) {
      pose.position[i] = model.state().position[i];
      velocity.linear[i] = model.state().velocity[i];
    }
    pose.q_wxyz[0] = std::cos(yaw * 0.5);
    pose.q_wxyz[3] = std::sin(yaw * 0.5);
    // The minimal plant assumes level yaw-only attitude. Specific force is
    // expressed in that body frame; it is not an EKF estimate or PX4 physics.
    const auto &a = model.acceleration();
    xgc_imu_v1 imu{};
    imu.stamp = stamp;
    imu.accel[0] = std::cos(yaw) * a.x() + std::sin(yaw) * a.y();
    imu.accel[1] = -std::sin(yaw) * a.x() + std::cos(yaw) * a.y();
    imu.accel[2] = 9.8066 + a.z();
    velocity.angular[2] = imu.gyro[2] = model.yaw_rate();
    xgc_fcu_state_v1 state{};
    state.stamp = stamp;
    state.connected = 1;
    state.armed = model.armed();
    state.guided = 1;
    std::strcpy(state.mode, fcu_mode.c_str());
    xgc_dmpc_paired_state_v1 paired{};
    paired.pose_stamp_sec = paired.twist_stamp_sec = stamp;
    std::copy(pose.position, pose.position + 3, paired.position);
    paired.orientation_xyzw[2] = pose.q_wxyz[3];
    paired.orientation_xyzw[3] = pose.q_wxyz[0];
    std::copy(velocity.linear, velocity.linear + 3, paired.linear_velocity);
    grid.publish(robot, FcuState, round, state);
    grid.publish(robot, Pose, round, pose);
    grid.publish(robot, Velocity, round, velocity);
    grid.publish(robot, Imu, round, imu);
    grid.publish(robot, PairedState, round, paired);
  }
};

struct ScoutRobot {
  xgc_lightweight::ScoutModel model;
  Commands commands;
  double z;

  ScoutRobot(const double *initial, double)
      : model(xgc2_math::Pose2{{initial[0], initial[1]}, initial[3]}),
        z(initial[2]) {}

  void apply(const Grid &grid, uint32_t, const Input &input, int64_t t) {
    if (input.port != VelocityCommand)
      throw std::invalid_argument(
          input.port == Setpoint
              ? "lightweight-vehicle: flight setpoint on ground model"
              : "lightweight-vehicle: FCU request on ground model");
    const auto value = body_velocity(input);
    model.command(double(t - grid.epoch) * 1e-9, value.linear[0],
                  value.angular[2]);
  }
  void step(const Grid &grid, uint32_t, int64_t t) {
    model.advance(double(t + grid.step_ns - grid.epoch) * 1e-9);
  }
  void output(const Grid &grid, uint32_t robot, uint64_t round) const {
    const auto response = model.velocity();
    planar_output(grid, robot, round, model.pose(), z,
                  {response.linear_m_s, 0.0}, response.yaw_rad_s);
  }
};

struct MecanumRobot {
  xgc_lightweight::MecanumModel model;
  Commands commands;
  double z;

  MecanumRobot(const double *initial, double)
      : model(xgc2_math::Pose2{{initial[0], initial[1]}, initial[3]}),
        z(initial[2]) {}

  void apply(const Grid &, uint32_t, const Input &input, int64_t) {
    if (input.port != VelocityCommand)
      throw std::invalid_argument(
          input.port == Setpoint
              ? "lightweight-vehicle: flight setpoint on ground model"
              : "lightweight-vehicle: FCU request on ground model");
    const auto value = body_velocity(input);
    model.command(value.linear[0], value.linear[1], value.angular[2]);
  }
  void step(const Grid &grid, uint32_t, int64_t) {
    model.step(double(grid.step_ns) * 1e-9);
  }
  void output(const Grid &grid, uint32_t robot, uint64_t round) const {
    planar_output(grid, robot, round, model.pose(), z, model.body_velocity(),
                  model.yaw_rate());
  }
};

// The one advance function of single, batched and distributed placement:
// apply the controls due at each grid boundary, then integrate one step.
template <class Robot>
void advance(const Grid &grid, uint32_t index, Robot &robot, int64_t target) {
  auto &pending = robot.commands.pending;
  for (int64_t t = grid.time; t < target; t += grid.step_ns) {
    while (!pending.empty() && pending.front().at <= t) {
      robot.apply(grid, index, pending.front(), t);
      pending.pop_front();
    }
    robot.step(grid, index, t);
  }
}

struct Plant {
  Grid grid;
  std::vector<FlightRobot> flights;
  std::vector<ScoutRobot> scouts;
  std::vector<MecanumRobot> mecanums;

  explicit Plant(const xgc_host_api *host) { grid.host = host; }

  void configure(const char *text) {
    namespace cfg = xgc_rt_config;
    std::string config(text ? text : ""), epoch_text;
    const auto model = cfg::text_or(config, "model", "fs150");
    if (model != "fs150" && model != "scout" && model != "mecanum")
      throw std::invalid_argument("lightweight-vehicle: unknown model");
    if (!cfg::value(config, "epoch_ns", &epoch_text))
      throw std::invalid_argument(
          "lightweight-vehicle: shared epoch_ns is required");
    grid.epoch = grid.time = grid.next_output = std::stoll(epoch_text);
    int robots = 1, step_ms = 1, output_ms = 10, offboard_timeout_ms = 500,
        max_future_ms = 1000, pending_limit = 1024;
    if (!cfg::integer(config, "robots", &robots) || robots < 1 ||
        robots > int(kMaxRobots))
      throw std::invalid_argument(
          "lightweight-vehicle: robots must be 1.." +
          std::to_string(kMaxRobots) + " (8 ports each, 64 per instance)");
    std::vector<double> initial(4 * size_t(robots), 0.0);
    std::string unused;
    const bool poses = cfg::value(config, "initial_poses", &unused);
    if ((poses && cfg::value(config, "initial_pose", &unused)) ||
        (!poses && robots > 1 && cfg::value(config, "initial_pose", &unused)))
      throw std::invalid_argument(
          "lightweight-vehicle: use initial_poses (4 numbers per robot) for a "
          "batch, initial_pose for one robot");
    if (!cfg::integer(config, "step_ms", &step_ms) || step_ms <= 0 ||
        !cfg::integer(config, "output_ms", &output_ms) || output_ms < step_ms ||
        !(poses ? cfg::numbers(config, "initial_poses", initial.data(),
                               initial.size())
                : cfg::numbers(config, "initial_pose", initial.data(), 4)) ||
        !std::all_of(initial.begin(), initial.end(),
                     [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument(
          "lightweight-vehicle: invalid step, output period or initial pose");
    // PX4 COM_OF_LOSS_T role; a future-stamped control beyond max_future_ms
    // is a clock-domain error, not a schedule. Both bound what a silent or
    // mis-stamped controller can do to the plant.
    if (!cfg::integer(config, "offboard_timeout_ms", &offboard_timeout_ms) ||
        offboard_timeout_ms <= 0 ||
        !cfg::integer(config, "max_future_ms", &max_future_ms) ||
        max_future_ms < 0 ||
        !cfg::integer(config, "max_pending", &pending_limit) ||
        pending_limit <= 0)
      throw std::invalid_argument(
          "lightweight-vehicle: invalid offboard timeout or command bounds");
    grid.robots = uint32_t(robots);
    grid.step_ns = int64_t(step_ms) * 1000000;
    grid.output_ns = int64_t(output_ms) * 1000000;
    grid.max_future_ns = int64_t(max_future_ms) * 1000000;
    grid.max_pending = size_t(pending_limit);
    flights.clear();
    scouts.clear();
    mecanums.clear();
    const double timeout_s = offboard_timeout_ms * 1e-3;
    if (model == "fs150")
      create(flights, initial, timeout_s);
    else if (model == "scout")
      create(scouts, initial, timeout_s);
    else
      create(mecanums, initial, timeout_s);
  }

  template <class Robot>
  void create(std::vector<Robot> &robots, const std::vector<double> &initial,
              double offboard_timeout_s) {
    robots.reserve(grid.robots);
    for (uint32_t i = 0; i != grid.robots; ++i)
      robots.emplace_back(&initial[4 * i], offboard_timeout_s);
  }

  void step(const xgc_step_ctx &context) {
    if (!flights.empty())
      run(flights, context);
    else if (!scouts.empty())
      run(scouts, context);
    else
      run(mecanums, context);
  }

  template <class Robot>
  void run(std::vector<Robot> &robots, const xgc_step_ctx &context) {
    for (uint32_t i = 0; i != robots.size(); ++i) {
      auto &commands = robots[i].commands;
      commands.drain(grid, i, Setpoint, sizeof(xgc_position_target_v1));
      commands.drain(grid, i, VelocityCommand, sizeof(xgc_twist_v1));
      commands.drain(grid, i, FcuRequest, sizeof(xgc_fcu_request_v1));
    }
    // One read of the host time sets one target grid point for the batch.
    if (context.now - grid.time >= grid.step_ns) {
      const int64_t target =
          grid.time + (context.now - grid.time) / grid.step_ns * grid.step_ns;
      for (uint32_t i = 0; i != robots.size(); ++i)
        advance(grid, i, robots[i], target);
      grid.time = target;
    }
    if (context.now >= grid.epoch && grid.time >= grid.next_output) {
      for (uint32_t i = 0; i != robots.size(); ++i)
        robots[i].output(grid, i, context.round);
      grid.next_output =
          grid.epoch +
          ((grid.time - grid.epoch) / grid.output_ns + 1) * grid.output_ns;
    }
  }
};

template <class F> xgc_status guarded(Plant *self, F &&action) {
  try {
    action();
    return XGC_OK;
  } catch (const std::exception &e) {
    self->grid.host->log(self->grid.host->host, XGC_LOG_ERROR, e.what());
    return XGC_ERR;
  }
}
void *create(const xgc_host_api *host) {
  try {
    return new Plant{host};
  } catch (...) {
    return nullptr;
  }
}
xgc_status configure(void *p, const char *text) {
  auto *self = static_cast<Plant *>(p);
  return guarded(self, [&] { self->configure(text); });
}
xgc_status activate(void *) { return XGC_OK; }
xgc_status step(void *p, const xgc_step_ctx *ctx) {
  auto *self = static_cast<Plant *>(p);
  return guarded(self, [&] { self->step(*ctx); });
}
xgc_status deactivate(void *) { return XGC_OK; }
void destroy(void *p) { delete static_cast<Plant *>(p); }
const char *state(void *) { return "Simulating"; }
const xgc_plugin_vtbl vtbl{create,     configure, activate, step,
                           deactivate, destroy,   state};

struct PortKind {
  const char *name;
  xgc_port_dir dir;
  const char *schema;
  xgc_qos qos;
};
// Block 0 keeps the single-robot ports, names and indices unchanged.
constexpr PortKind kPortKinds[kPortsPerRobot] = {
    {"setpoint", XGC_PORT_IN_OPTIONAL, "xgc.position_target/1",
     XGC_QOS_CONTROL},
    {"cmd_vel", XGC_PORT_IN_OPTIONAL, "xgc.twist/1", XGC_QOS_CONTROL},
    {"fcu_request", XGC_PORT_IN_OPTIONAL, "xgc.fcu_request/1", XGC_QOS_EVENT},
    {"pose", XGC_PORT_OUT, "xgc.pose/1", XGC_QOS_STATE},
    {"velocity", XGC_PORT_OUT, "xgc.twist/1", XGC_QOS_STATE},
    {"imu", XGC_PORT_OUT_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"fcu_state", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_state/1", XGC_QOS_STATE},
    {"paired_state", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.paired_state/1",
     XGC_QOS_STATE},
};

struct Descriptor {
  std::array<std::string, XGC_RT_MAX_PORTS> names;
  std::array<xgc_port_decl, XGC_RT_MAX_PORTS> ports{};
  xgc_plugin_descriptor descriptor{};

  Descriptor() {
    for (uint32_t robot = 0; robot != kMaxRobots; ++robot)
      for (uint32_t kind = 0; kind != kPortsPerRobot; ++kind) {
        const auto i = robot * kPortsPerRobot + kind;
        const auto &port = kPortKinds[kind];
        names[i] = port.name;
        if (robot != 0)
          names[i] += "_" + std::to_string(robot);
        // Only robot 0 must be bound; later blocks serve a batch.
        const auto dir = robot != 0 && port.dir == XGC_PORT_OUT
                             ? XGC_PORT_OUT_OPTIONAL
                             : port.dir;
        ports[i] = {names[i].c_str(), dir, port.schema, port.qos};
      }
    descriptor = {XGC_RT_ABI_VERSION, XGC_RT_MAX_PORTS, "lightweight-vehicle",
                  "0.2.0",           ports.data(),     &vtbl};
  }
};
} // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor *
xgc_rt_plugin_v1() {
  static const Descriptor descriptor;
  return &descriptor.descriptor;
}
