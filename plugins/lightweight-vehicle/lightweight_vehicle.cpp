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

namespace {
enum Port {
  Setpoint,
  VelocityCommand,
  FcuRequest,
  Pose,
  Velocity,
  Imu,
  FcuState,
  PairedState
};
enum class Kind { Flight, Scout, Mecanum };
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

struct Vehicle {
  const xgc_host_api *host;
  explicit Vehicle(const xgc_host_api *api) : host(api) {}
  Kind kind{Kind::Flight};
  std::unique_ptr<xgc_lightweight::FlightModel> flight;
  std::unique_ptr<xgc_lightweight::ScoutModel> scout;
  std::unique_ptr<xgc_lightweight::MecanumModel> mecanum;
  std::deque<Input> pending;
  int64_t epoch{0}, time{0}, step_ns{1000000}, output_ns{10000000},
      next_output{0};
  double ground_z{0.0}, yaw{0.0}, yaw_rate{0.0};
  std::string fcu_mode{"POSCTL"};
  bool yaw_target_enabled{false};
  double yaw_target{0.0};

  void configure(const char *text) {
    namespace cfg = xgc_rt_config;
    std::string config(text ? text : ""), epoch_text;
    const auto model = cfg::text_or(config, "model", "fs150");
    if (model == "fs150")
      kind = Kind::Flight;
    else if (model == "scout")
      kind = Kind::Scout;
    else if (model == "mecanum")
      kind = Kind::Mecanum;
    else
      throw std::invalid_argument("lightweight-vehicle: unknown model");
    if (!cfg::value(config, "epoch_ns", &epoch_text))
      throw std::invalid_argument(
          "lightweight-vehicle: shared epoch_ns is required");
    epoch = time = next_output = std::stoll(epoch_text);
    int step_ms = 1, output_ms = 10;
    double initial[4]{0, 0, 0, 0};
    if (!cfg::integer(config, "step_ms", &step_ms) || step_ms <= 0 ||
        !cfg::integer(config, "output_ms", &output_ms) || output_ms < step_ms ||
        !cfg::numbers(config, "initial_pose", initial, 4) ||
        !std::all_of(initial, initial + 4,
                     [](double v) { return std::isfinite(v); }))
      throw std::invalid_argument(
          "lightweight-vehicle: invalid step, output period or initial pose");
    step_ns = int64_t(step_ms) * 1000000;
    output_ns = int64_t(output_ms) * 1000000;
    ground_z = initial[2];
    yaw = initial[3];
    flight.reset();
    scout.reset();
    mecanum.reset();
    pending.clear();
    yaw_rate = 0;
    yaw_target_enabled = false;
    fcu_mode = "POSCTL";
    if (kind == Kind::Flight)
      flight = std::make_unique<xgc_lightweight::FlightModel>(
          Eigen::Vector3d(initial[0], initial[1], initial[2]));
    else if (kind == Kind::Scout)
      scout = std::make_unique<xgc_lightweight::ScoutModel>(
          xgc2_math::Pose2{{initial[0], initial[1]}, yaw});
    else
      mecanum = std::make_unique<xgc_lightweight::MecanumModel>(
          xgc2_math::Pose2{{initial[0], initial[1]}, yaw});
  }

  void drain(Port port, size_t size) {
    xgc_sample_view sample{};
    while (host->next(host->host, port, &sample) == XGC_OK) {
      if (sample.len != size)
        throw std::invalid_argument("lightweight-vehicle: wrong input size");
      double stamp;
      std::memcpy(&stamp, sample.data, sizeof(stamp));
      // Never use a future control early or rewrite an already integrated past.
      Input input{std::max({time, sample.t_rx, nanoseconds(stamp)}), port, {}};
      std::memcpy(input.data.data(), sample.data, size);
      auto at = std::upper_bound(
          pending.begin(), pending.end(), input.at,
          [](int64_t t, const Input &item) { return t < item.at; });
      pending.insert(at, input);
    }
  }

  void apply(const Input &input) {
    if (input.port == Setpoint) {
      if (!flight)
        throw std::invalid_argument(
            "lightweight-vehicle: flight setpoint on ground model");
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
      flight->setpoint(value);
      yaw_target_enabled = !(wire.type_mask & 1024);
      yaw_target = wire.yaw;
      yaw_rate = (wire.type_mask & 2048) ? 0.0 : wire.yaw_rate;
      if ((yaw_target_enabled && !std::isfinite(yaw_target)) ||
          !std::isfinite(yaw_rate))
        throw std::invalid_argument(
            "lightweight-vehicle: nonfinite yaw command");
    } else if (input.port == VelocityCommand) {
      xgc_twist_v1 value;
      std::memcpy(&value, input.data.data(), sizeof value);
      const double forward = value.linear[0], left = value.linear[1],
                   turn = value.angular[2];
      if (!std::isfinite(forward) || !std::isfinite(left) ||
          !std::isfinite(turn))
        throw std::invalid_argument(
            "lightweight-vehicle: nonfinite body velocity");
      if (scout)
        scout->command(double(time - epoch) * 1e-9, forward, turn);
      else if (mecanum)
        mecanum->command(forward, left, turn);
      else
        throw std::invalid_argument(
            "lightweight-vehicle: ground velocity on flight model");
    } else {
      if (!flight)
        throw std::invalid_argument(
            "lightweight-vehicle: FCU request on ground model");
      xgc_fcu_request_v1 request;
      std::memcpy(&request, input.data.data(), sizeof request);
      if (request.kind == 1)
        flight->arm(request.arm != 0);
      else if (request.kind == 2) {
        const auto end = static_cast<const char *>(
            std::memchr(request.mode, 0, sizeof request.mode));
        if (!end)
          throw std::invalid_argument(
              "lightweight-vehicle: unterminated FCU mode");
        fcu_mode.assign(request.mode, static_cast<size_t>(end - request.mode));
        flight->offboard(fcu_mode == "OFFBOARD");
      } else
        throw std::invalid_argument(
            "lightweight-vehicle: unsupported FCU request");
    }
  }

  template <class T> void publish(Port port, uint64_t round, const T &value) {
    if (host->publish(host->host, port, round,
                      reinterpret_cast<const uint8_t *>(&value),
                      sizeof value) != XGC_OK)
      throw std::runtime_error("lightweight-vehicle: state publish failed");
  }

  void output(uint64_t round) {
    const double stamp = double(time) * 1e-9;
    xgc_pose_v1 pose{};
    pose.stamp = stamp;
    xgc_twist_v1 velocity{};
    velocity.stamp = stamp;
    xgc_imu_v1 imu{};
    imu.stamp = stamp;
    imu.accel[2] = 9.8066;
    if (flight) {
      for (int i = 0; i != 3; ++i) {
        pose.position[i] = flight->state().position[i];
        velocity.linear[i] = flight->state().velocity[i];
      }
      // The minimal plant assumes level yaw-only attitude. Specific force is
      // expressed in that body frame; it is not an EKF estimate or PX4 physics.
      const auto &a = flight->acceleration();
      imu.accel[0] = std::cos(yaw) * a.x() + std::sin(yaw) * a.y();
      imu.accel[1] = -std::sin(yaw) * a.x() + std::cos(yaw) * a.y();
      imu.accel[2] += a.z();
      xgc_fcu_state_v1 state{};
      state.stamp = stamp;
      state.connected = 1;
      state.armed = flight->armed();
      state.guided = 1;
      std::strcpy(state.mode, fcu_mode.c_str());
      publish(FcuState, round, state);
    } else {
      const auto &state = scout ? scout->pose() : mecanum->pose();
      yaw = state.yaw;
      pose.position[0] = state.position.x();
      pose.position[1] = state.position.y();
      pose.position[2] = ground_z;
      const Eigen::Vector2d body =
          scout ? Eigen::Vector2d(scout->velocity().linear_m_s, 0.0)
                : mecanum->body_velocity();
      const auto world = (xgc2_math::rotationMatrix2(yaw) * body).eval();
      velocity.linear[0] = world.x();
      velocity.linear[1] = world.y();
      yaw_rate = scout ? scout->velocity().yaw_rad_s : mecanum->yaw_rate();
    }
    pose.q_wxyz[0] = std::cos(yaw * 0.5);
    pose.q_wxyz[3] = std::sin(yaw * 0.5);
    velocity.angular[2] = imu.gyro[2] = yaw_rate;
    xgc_dmpc_paired_state_v1 paired{};
    paired.pose_stamp_sec = paired.twist_stamp_sec = stamp;
    std::copy(pose.position, pose.position + 3, paired.position);
    paired.orientation_xyzw[2] = pose.q_wxyz[3];
    paired.orientation_xyzw[3] = pose.q_wxyz[0];
    std::copy(velocity.linear, velocity.linear + 3, paired.linear_velocity);
    publish(Pose, round, pose);
    publish(Velocity, round, velocity);
    if (flight)
      publish(Imu, round, imu);
    publish(PairedState, round, paired);
  }

  void step(const xgc_step_ctx &context) {
    drain(Setpoint, sizeof(xgc_position_target_v1));
    drain(VelocityCommand, sizeof(xgc_twist_v1));
    drain(FcuRequest, sizeof(xgc_fcu_request_v1));
    while (context.now - time >= step_ns) {
      while (!pending.empty() && pending.front().at <= time) {
        apply(pending.front());
        pending.pop_front();
      }
      const double dt = double(step_ns) * 1e-9;
      if (flight) {
        flight->step(dt);
        if (flight->armed() && flight->offboard())
          yaw = yaw_target_enabled
                    ? yaw_target
                    : xgc2_math::normalizeAngle(yaw + yaw_rate * dt);
      } else if (scout)
        scout->advance(double(time + step_ns - epoch) * 1e-9);
      else
        mecanum->step(dt);
      time += step_ns;
    }
    if (context.now >= epoch && time >= next_output) {
      output(context.round);
      next_output = epoch + ((time - epoch) / output_ns + 1) * output_ns;
    }
  }
};

template <class F> xgc_status guarded(Vehicle *self, F &&action) {
  try {
    action();
    return XGC_OK;
  } catch (const std::exception &e) {
    self->host->log(self->host->host, XGC_LOG_ERROR, e.what());
    return XGC_ERR;
  }
}
void *create(const xgc_host_api *host) {
  try {
    return new Vehicle{host};
  } catch (...) {
    return nullptr;
  }
}
xgc_status configure(void *p, const char *text) {
  auto *self = static_cast<Vehicle *>(p);
  return guarded(self, [&] { self->configure(text); });
}
xgc_status activate(void *) { return XGC_OK; }
xgc_status step(void *p, const xgc_step_ctx *ctx) {
  auto *self = static_cast<Vehicle *>(p);
  return guarded(self, [&] { self->step(*ctx); });
}
xgc_status deactivate(void *) { return XGC_OK; }
void destroy(void *p) { delete static_cast<Vehicle *>(p); }
const char *state(void *) { return "Simulating"; }
const xgc_port_decl ports[] = {
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
const xgc_plugin_vtbl vtbl{create,     configure, activate, step,
                           deactivate, destroy,   state};
const xgc_plugin_descriptor descriptor{
    XGC_RT_ABI_VERSION, 8, "lightweight-vehicle", "0.1.0", ports, &vtbl};
} // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor *
xgc_rt_plugin_v1() {
  return &descriptor;
}
