#pragma once

#include <algorithm>
#include <cmath>

#include <xgc2_math/control/delayed_planar_velocity.hpp>
#include <xgc2_math/geometry/kinematics.hpp>

// The numerical plant has no ROS, transport, scheduler or controller. Native
// and ROS boundaries supply actual actuator commands and elapsed model time.
namespace xgc_lightweight {

struct FlightSetpoint {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool position_enabled[3]{false, false, false};
  bool velocity_enabled[3]{false, false, false};
  bool acceleration_enabled[3]{false, false, false};
  // PositionTarget yaw fields. Without a yaw target, yaw_rate is integrated;
  // it is zero when the command ignores yaw rate.
  bool yaw_enabled{false};
  double yaw{0.0};
  double yaw_rate{0.0};
};

// The FCU behaviours this plant implements. A mode request that maps to none
// of them is refused by the caller and leaves the reported mode unchanged.
enum class FlightMode {
  Hold,     // POSCTL / ALTCTL / AUTO.LOITER with centred sticks: brake, hold
  Offboard, // follow the latest setpoint while it stays fresh
  Land,     // AUTO.LAND: descend, then disarm on touchdown
};

// Transitions the plant makes on its own during a step.
enum class FlightEvent { None, OffboardLost, Landed };

// Ideal acceleration inner loop for SMC. The position/velocity branch models
// the FCU modes used by the existing controller's Takeoff/Hover/Landing states;
// its constants match the existing px4_plant numerical test (not PX4 tuning).
// It is bypassed entirely for an acceleration-only command from SMC.
//
// FCU rules follow PX4 where the existing controller depends on them:
// OFFBOARD needs a setpoint younger than the offboard timeout, and a stream
// that stops for that long falls back to Hold (COM_OF_LOSS_T); a disarm
// without force is refused in the air; AUTO.LAND descends at the default
// MPC_LAND_SPEED and disarms on touchdown. Estimator, RC and preflight
// behaviour is not modelled.
class FlightModel {
public:
  static constexpr double kLandSpeed = 0.7;    // PX4 default MPC_LAND_SPEED
  static constexpr double kLandedHeight = 0.01; // above the initial ground

  explicit FlightModel(const Eigen::Vector3d &position, double yaw = 0.0,
                       double offboard_timeout_s = 0.5)
      : state_{position, Eigen::Vector3d::Zero()}, ground_z_(position.z()),
        offboard_timeout_s_(offboard_timeout_s), yaw_(yaw) {}

  void setpoint(const FlightSetpoint &value) {
    setpoint_ = value;
    has_setpoint_ = true;
    setpoint_age_ = 0.0;
  }
  // False when refused: disarming is refused while airborne.
  bool request_arm(bool value) {
    if (!value && armed_ && !landed())
      return false;
    armed_ = value;
    return true;
  }
  // False when refused: OFFBOARD needs a fresh setpoint stream.
  bool request_mode(FlightMode value) {
    if (value == FlightMode::Offboard && !setpoint_fresh())
      return false;
    mode_ = value;
    return true;
  }
  bool armed() const { return armed_; }
  FlightMode mode() const { return mode_; }
  bool landed() const {
    return state_.position.z() <= ground_z_ + kLandedHeight;
  }
  const xgc2_math::TranslationalState &state() const { return state_; }
  const Eigen::Vector3d &acceleration() const { return acceleration_; }
  double yaw() const { return yaw_; }
  // Realized yaw change over the last step, not the last commanded rate.
  double yaw_rate() const { return yaw_rate_; }

  FlightEvent step(double dt) {
    FlightEvent event = FlightEvent::None;
    if (mode_ == FlightMode::Offboard && !setpoint_fresh()) {
      mode_ = FlightMode::Hold;
      event = FlightEvent::OffboardLost;
    }
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    const double yaw_before = yaw_;
    if (armed_ && mode_ == FlightMode::Offboard) {
      acceleration = track(setpoint_);
      yaw_ = setpoint_.yaw_enabled
                 ? setpoint_.yaw
                 : xgc2_math::normalizeAngle(yaw_ + setpoint_.yaw_rate * dt);
    } else if (armed_ && mode_ == FlightMode::Land) {
      FlightSetpoint descend;
      descend.velocity.z() = -kLandSpeed;
      for (auto &enabled : descend.velocity_enabled)
        enabled = true;
      acceleration = track(descend);
    } else if (armed_) {
      acceleration = -state_.velocity / 0.2; // Hold brakes, also on the ground
    } else if (state_.position.z() > ground_z_) {
      acceleration = Eigen::Vector3d(0.0, 0.0, -9.8066);
    }
    state_ = xgc2_math::stepWorldAcceleration(state_, acceleration, dt);
    if (state_.position.z() <= ground_z_) {
      state_.position.z() = ground_z_;
      state_.velocity.z() = std::max(0.0, state_.velocity.z());
      acceleration.z() = std::max(0.0, acceleration.z());
      if (armed_ && mode_ == FlightMode::Land) {
        armed_ = false;
        event = FlightEvent::Landed;
      }
      if (!armed_) {
        state_.velocity.setZero();
        acceleration.setZero();
      }
    }
    acceleration_ = acceleration;
    yaw_rate_ =
        dt > 0.0 ? xgc2_math::normalizeAngle(yaw_ - yaw_before) / dt : 0.0;
    setpoint_age_ += dt;
    return event;
  }

private:
  // A setpoint applied at t stays usable until t + timeout (1 ns tolerance
  // absorbs the summed step lengths).
  bool setpoint_fresh() const {
    return has_setpoint_ && setpoint_age_ < offboard_timeout_s_ - 1e-9;
  }

  Eigen::Vector3d track(const FlightSetpoint &setpoint) const {
    bool acceleration_only = true;
    for (int i = 0; i != 3; ++i) {
      acceleration_only =
          acceleration_only && !setpoint.position_enabled[i] &&
          !setpoint.velocity_enabled[i] && setpoint.acceleration_enabled[i];
    }
    if (acceleration_only)
      return setpoint.acceleration;
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_command = Eigen::Vector3d::Zero();
    for (int i = 0; i != 3; ++i) {
      if (setpoint.position_enabled[i])
        velocity_command[i] +=
            1.5 * (setpoint.position[i] - state_.position[i]);
      if (setpoint.velocity_enabled[i])
        velocity_command[i] += setpoint.velocity[i];
    }
    const double speed = velocity_command.norm();
    if (speed > 1.5)
      velocity_command *= 1.5 / speed;
    for (int i = 0; i != 3; ++i) {
      acceleration[i] = std::clamp(
          (velocity_command[i] - state_.velocity[i]) / 0.3, -3.0, 3.0);
      if (setpoint.acceleration_enabled[i])
        acceleration[i] += setpoint.acceleration[i];
    }
    return acceleration;
  }

  xgc2_math::TranslationalState state_;
  double ground_z_;
  double offboard_timeout_s_;
  FlightSetpoint setpoint_;
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  double setpoint_age_{0.0};
  double yaw_;
  double yaw_rate_{0.0};
  FlightMode mode_{FlightMode::Hold};
  bool has_setpoint_{false};
  bool armed_{false};
};

class ScoutModel {
public:
  explicit ScoutModel(
      const xgc2_math::Pose2 &pose,
      const xgc2_math::DelayedPlanarVelocityParameters &response = {})
      : pose_(pose), response_(response) {}

  void command(double time, double forward, double yaw_rate) {
    // Same defaults as the accepted Gazebo unicycle plant; not wheel physics.
    response_.command(time, {std::clamp(forward, -1.5, 1.5),
                             std::clamp(yaw_rate, -1.0, 1.0)});
  }
  void advance(double end) {
    const double dt = end - response_.time();
    const auto middle = response_.advance(response_.time() + 0.5 * dt);
    pose_ = xgc2_math::stepBodyVelocity(pose_, {middle.linear_m_s, 0.0},
                                        middle.yaw_rad_s, dt);
    response_.advance(end);
  }
  const xgc2_math::Pose2 &pose() const { return pose_; }
  xgc2_math::PlanarVelocity velocity() const { return response_.velocity(); }

private:
  xgc2_math::Pose2 pose_;
  xgc2_math::DelayedPlanarVelocity response_;
};

class MecanumModel {
public:
  explicit MecanumModel(const xgc2_math::Pose2 &pose) : pose_(pose) {}

  void command(double forward, double left, double yaw_rate) {
    // Defaults of ugv_sim_single.launch, not unverified MCU/TEB limits.
    body_velocity_ = {std::clamp(forward, -1.5, 1.5),
                      std::clamp(left, -1.5, 1.5)};
    yaw_rate_ = std::clamp(yaw_rate, -1.5707963267948966, 1.5707963267948966);
  }
  void step(double dt) {
    pose_ = xgc2_math::stepBodyVelocity(pose_, body_velocity_, yaw_rate_, dt);
  }
  const xgc2_math::Pose2 &pose() const { return pose_; }
  const Eigen::Vector2d &body_velocity() const { return body_velocity_; }
  double yaw_rate() const { return yaw_rate_; }

private:
  xgc2_math::Pose2 pose_;
  Eigen::Vector2d body_velocity_{Eigen::Vector2d::Zero()};
  double yaw_rate_{0.0};
};

} // namespace xgc_lightweight
