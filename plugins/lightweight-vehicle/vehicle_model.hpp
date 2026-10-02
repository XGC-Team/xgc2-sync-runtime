#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "flight_controller.hpp"

#include <xgc2_math/control/delayed_planar_velocity.hpp>
#include <xgc2_math/geometry/kinematics.hpp>

// The numerical plant has no ROS, transport or scheduler. Native and ROS
// boundaries supply commands and elapsed model time; flight embeds an FCU loop.
namespace xgc_lightweight {

struct FlightSetpoint {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  bool position_enabled[3]{false, false, false};
  bool velocity_enabled[3]{false, false, false};
  bool acceleration_enabled[3]{false, false, false};
  // PositionTarget yaw fields. Without a yaw target, yaw_rate is integrated
  // into the desired heading; it is zero when the command ignores yaw rate.
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

// Simplified FCU mode/stream behaviour around a physical four-rotor plant.
// Existing P/V branches keep their numerical outer-loop constants. ACC-only
// commands bypass those branches, then pass through the same finite-bandwidth
// attitude/rate/motor dynamics. Estimator, RC and preflight are not modelled.
class FlightModel {
public:
  static constexpr double kLandSpeed = 0.7;
  static constexpr double kLandedHeight = 0.01;
  static constexpr double kMaxControlStep = 0.001;

  explicit FlightModel(const Eigen::Vector3d &position, double yaw = 0.0,
                       double offboard_timeout_s = 0.5)
      : body_(position, yaw), controller_(body_.parameters()),
        ground_z_(position.z()), offboard_timeout_s_(offboard_timeout_s),
        yaw_command_(yaw) {
    if (!std::isfinite(offboard_timeout_s) || offboard_timeout_s <= 0.0)
      throw std::invalid_argument("flight model: invalid offboard timeout");
    refresh_state();
  }

  void setpoint(const FlightSetpoint &value) {
    for (int i = 0; i != 3; ++i)
      if ((value.position_enabled[i] && !std::isfinite(value.position[i])) ||
          (value.velocity_enabled[i] && !std::isfinite(value.velocity[i])) ||
          (value.acceleration_enabled[i] &&
           !std::isfinite(value.acceleration[i])))
        throw std::invalid_argument("flight model: nonfinite setpoint");
    if ((value.yaw_enabled && !std::isfinite(value.yaw)) ||
        !std::isfinite(value.yaw_rate))
      throw std::invalid_argument("flight model: nonfinite yaw setpoint");
    setpoint_ = value;
    has_setpoint_ = true;
    setpoint_age_ = 0.0;
  }
  bool request_arm(bool value) {
    if (!value && armed_ && !landed())
      return false;
    if (value != armed_)
      yaw_command_ = yaw();
    armed_ = value;
    return true;
  }
  bool request_mode(FlightMode value) {
    if (value == FlightMode::Offboard && !setpoint_fresh())
      return false;
    if (value != mode_)
      yaw_command_ = yaw();
    mode_ = value;
    return true;
  }
  bool armed() const { return armed_; }
  FlightMode mode() const { return mode_; }
  bool landed() const {
    return state_.position.z() <= ground_z_ + kLandedHeight;
  }
  // The original measurement point is the body origin, not the COM.
  const xgc2_math::TranslationalState &state() const { return state_; }
  const Eigen::Vector3d &acceleration() const { return acceleration_; }
  const Eigen::Quaterniond &orientation() const {
    return body_.state().orientation;
  }
  Eigen::Vector3d angular_velocity_body() const {
    return body_.state().angular_velocity;
  }
  Eigen::Vector3d specific_force_body() const {
    return orientation().conjugate() *
           (acceleration_ - Eigen::Vector3d(0.0, 0.0, -body_.parameters().gravity));
  }
  double yaw() const {
    const auto r = orientation().toRotationMatrix();
    return std::atan2(r(1, 0), r(0, 0));
  }
  // Realized world-heading change, distinct from the body gyro's z component
  // when tilted. Neither is the commanded yaw rate.
  double yaw_rate() const { return yaw_rate_; }
  const FlightControlOutput &control_output() const { return control_output_; }

  FlightEvent step(double dt) {
    if (!std::isfinite(dt) || dt < 0.0 ||
        dt / kMaxControlStep > std::numeric_limits<int>::max())
      throw std::invalid_argument("flight model: invalid dt");
    if (dt == 0.0)
      return FlightEvent::None;
    FlightEvent event = FlightEvent::None;
    const double yaw_before = yaw();
    // Resolve the FCU and ground constraint at 1 kHz even if a caller advances
    // in larger chunks. Motor response remains exclusively in RigidBodyModel.
    const int count = static_cast<int>(std::ceil(dt / kMaxControlStep));
    const double h = dt / count;
    for (int step = 0; step != count; ++step) {
      if (mode_ == FlightMode::Offboard && !setpoint_fresh()) {
        mode_ = FlightMode::Hold;
        yaw_command_ = yaw();
        event = FlightEvent::OffboardLost;
      }
      Eigen::Vector3d a_command = Eigen::Vector3d::Zero();
      double desired_yaw_rate = 0.0;
      Eigen::Vector4d motors = Eigen::Vector4d::Zero();
      if (armed_) {
        if (mode_ == FlightMode::Offboard) {
          a_command = track(setpoint_);
          if (setpoint_.yaw_enabled)
            yaw_command_ = setpoint_.yaw;
          else {
            desired_yaw_rate = setpoint_.yaw_rate;
            yaw_command_ = xgc2_math::normalizeAngle(
                yaw_command_ + desired_yaw_rate * h);
          }
        } else if (mode_ == FlightMode::Land) {
          FlightSetpoint descend;
          descend.velocity.z() = -kLandSpeed;
          for (auto &enabled : descend.velocity_enabled)
            enabled = true;
          a_command = track(descend);
        } else {
          a_command = -state_.velocity / 0.2;
        }
        control_output_ = controller_.command(body_.state(), a_command,
                                             yaw_command_, desired_yaw_rate);
        motors = control_output_.allocation.target_rotor_speed;
      } else {
        control_output_ = FlightControlOutput{};
      }
      body_.step(motors, h);
      acceleration_ = body_.base_acceleration();
      // Only the initial ground plane constrains the base measurement point.
      // This is not full contact/friction/collision physics: no wall avoidance,
      // roll/pitch constraint or horizontal friction is introduced here.
      if (body_.base_position().z() <= ground_z_) {
        auto contact = body_.state();
        contact.position.z() += ground_z_ - body_.base_position().z();
        const double downward = std::min(0.0, body_.base_velocity().z());
        contact.velocity.z() -= downward;
        body_.set_state(contact);
        acceleration_ = body_.base_acceleration();
        // Include sustained support and the resolved impact impulse in the
        // base accelerometer sample. Impact is averaged over this substep;
        // the underlying free rigid-body acceleration is an endpoint value.
        acceleration_.z() +=
            std::max({0.0, -acceleration_.z(), -downward / h});
        if (armed_ && mode_ == FlightMode::Land) {
          armed_ = false;
          event = FlightEvent::Landed;
        }
      }
      refresh_state();
      setpoint_age_ += h;
    }
    yaw_rate_ = xgc2_math::normalizeAngle(yaw() - yaw_before) / dt;
    return event;
  }

private:
  void refresh_state() {
    state_.position = body_.base_position();
    state_.velocity = body_.base_velocity();
  }
  bool setpoint_fresh() const {
    return has_setpoint_ && setpoint_age_ < offboard_timeout_s_ - 1e-9;
  }
  Eigen::Vector3d track(const FlightSetpoint &setpoint) const {
    bool acceleration_only = true;
    for (int i = 0; i != 3; ++i)
      acceleration_only = acceleration_only && !setpoint.position_enabled[i] &&
                          !setpoint.velocity_enabled[i] &&
                          setpoint.acceleration_enabled[i];
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

  RigidBodyModel body_;
  FlightController controller_;
  xgc2_math::TranslationalState state_;
  double ground_z_;
  double offboard_timeout_s_;
  FlightSetpoint setpoint_;
  FlightControlOutput control_output_;
  Eigen::Vector3d acceleration_{Eigen::Vector3d::Zero()};
  double setpoint_age_{0.0};
  double yaw_command_;
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
