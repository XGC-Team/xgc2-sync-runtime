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
};

// Ideal acceleration inner loop for SMC. The position/velocity branch models
// the FCU modes used by the existing controller's Takeoff/Hover/Landing states;
// its constants match the existing px4_plant numerical test (not PX4 tuning).
// It is bypassed entirely for an acceleration-only command from SMC.
class FlightModel {
public:
  explicit FlightModel(const Eigen::Vector3d &position)
      : state_{position, Eigen::Vector3d::Zero()}, ground_z_(position.z()) {}

  void setpoint(const FlightSetpoint &value) {
    setpoint_ = value;
    has_setpoint_ = true;
  }
  void arm(bool value) { armed_ = value; }
  void offboard(bool value) { offboard_ = value; }
  bool armed() const { return armed_; }
  bool offboard() const { return offboard_; }
  const xgc2_math::TranslationalState &state() const { return state_; }
  const Eigen::Vector3d &acceleration() const { return acceleration_; }

  void step(double dt) {
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    if (armed_ && offboard_ && has_setpoint_) {
      bool acceleration_only = true;
      for (int i = 0; i != 3; ++i) {
        acceleration_only =
            acceleration_only && !setpoint_.position_enabled[i] &&
            !setpoint_.velocity_enabled[i] && setpoint_.acceleration_enabled[i];
      }
      if (acceleration_only) {
        acceleration = setpoint_.acceleration;
      } else {
        Eigen::Vector3d velocity_command = Eigen::Vector3d::Zero();
        for (int i = 0; i != 3; ++i) {
          if (setpoint_.position_enabled[i])
            velocity_command[i] +=
                1.5 * (setpoint_.position[i] - state_.position[i]);
          if (setpoint_.velocity_enabled[i])
            velocity_command[i] += setpoint_.velocity[i];
        }
        const double speed = velocity_command.norm();
        if (speed > 1.5)
          velocity_command *= 1.5 / speed;
        for (int i = 0; i != 3; ++i) {
          acceleration[i] = std::clamp(
              (velocity_command[i] - state_.velocity[i]) / 0.3, -3.0, 3.0);
          if (setpoint_.acceleration_enabled[i])
            acceleration[i] += setpoint_.acceleration[i];
        }
      }
    } else if (state_.position.z() > ground_z_) {
      acceleration =
          armed_ ? -state_.velocity / 0.2 : Eigen::Vector3d(0.0, 0.0, -9.8066);
    }
    state_ = xgc2_math::stepWorldAcceleration(state_, acceleration, dt);
    if (state_.position.z() <= ground_z_) {
      state_.position.z() = ground_z_;
      state_.velocity.z() = std::max(0.0, state_.velocity.z());
      acceleration.z() = std::max(0.0, acceleration.z());
      if (!armed_) {
        state_.velocity.setZero();
        acceleration.setZero();
      }
    }
    acceleration_ = acceleration;
  }

private:
  xgc2_math::TranslationalState state_;
  double ground_z_;
  FlightSetpoint setpoint_;
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  bool has_setpoint_{false};
  bool armed_{false};
  bool offboard_{false};
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
