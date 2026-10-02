#pragma once

#include "rigid_body.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xgc_lightweight {

// Frozen numerical FCU tuning, in s^-1 and rad/s, respectively. These are
// simulation inner-loop parameters, not PX4 gains or an FS150 calibration.
// No integrator: actuator saturation cannot accumulate windup.
struct FlightControllerParameters {
  Eigen::Vector3d attitude_gain{6.0, 6.0, 3.0};
  Eigen::Vector3d rate_gain{18.0, 18.0, 10.0};
  Eigen::Vector3d max_body_rate{4.0, 4.0, 2.0};
};

struct RotorAllocation {
  Eigen::Vector4d target_rotor_speed{Eigen::Vector4d::Zero()};
  // Wrench order is collective N, then body FLU torque xyz in N m.
  Eigen::Vector4d requested_wrench{Eigen::Vector4d::Zero()};
  Eigen::Vector4d allocated_wrench{Eigen::Vector4d::Zero()};
  double differential_scale{1.0};
  bool collective_saturated{false};
  bool saturated() const {
    return collective_saturated || differential_scale < 1.0 - 1e-12;
  }
};

struct FlightControlOutput {
  Eigen::Vector3d acceleration_command{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond desired_orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d desired_body_rate{Eigen::Vector3d::Zero()};
  RotorAllocation allocation;
};

class FlightController {
public:
  explicit FlightController(
      const RigidBodyParameters &plant,
      const FlightControllerParameters &control = {})
      : plant_(plant), control_(control) {
    if (!(plant.mass > 0.0) || !std::isfinite(plant.mass) ||
        !(plant.thrust_coefficient > 0.0) ||
        !std::isfinite(plant.thrust_coefficient) ||
        !(plant.max_rotor_speed > 0.0) ||
        !std::isfinite(plant.max_rotor_speed) ||
        !plant.inertia.allFinite() || !std::isfinite(plant.gravity) ||
        !control.attitude_gain.allFinite() || !control.rate_gain.allFinite() ||
        !control.max_body_rate.allFinite() ||
        (control.attitude_gain.array() <= 0.0).any() ||
        (control.rate_gain.array() <= 0.0).any() ||
        (control.max_body_rate.array() <= 0.0).any())
      throw std::invalid_argument("flight controller: invalid parameters");
    for (int i = 0; i != 4; ++i) {
      const auto &r = plant.rotor_position[i];
      mixer_.col(i) << 1.0, r.y(), -r.x(),
          plant.moment_ratio * plant.rotor_yaw_sign[i];
    }
    const Eigen::FullPivLU<Eigen::Matrix4d> lu(mixer_);
    if (!mixer_.allFinite() || !lu.isInvertible())
      throw std::invalid_argument("flight controller: singular rotor geometry");
    inverse_mixer_ = lu.inverse();
    max_rotor_thrust_ = plant.thrust_coefficient *
                        plant.max_rotor_speed * plant.max_rotor_speed;
    if (!std::isfinite(4.0 * max_rotor_thrust_))
      throw std::invalid_argument("flight controller: nonfinite thrust limit");
  }

  const FlightControllerParameters &parameters() const { return control_; }

  // Force direction sets body +Z; desired world heading fixes the remaining
  // degree of freedom. Choose a deterministic orthogonal axis at the heading
  // singularity; zero desired force retains a level yaw reference.
  static Eigen::Quaterniond desired_orientation(const Eigen::Vector3d &force,
                                                double yaw) {
    if (!force.allFinite() || !std::isfinite(yaw))
      throw std::invalid_argument("flight controller: nonfinite reference");
    Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
    const double magnitude = force.stableNorm();
    if (magnitude > 1e-9)
      z = force / magnitude;
    const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
    const Eigen::Vector3d heading_left(-std::sin(yaw), std::cos(yaw), 0.0);
    Eigen::Vector3d x = heading_left.cross(z);
    if (x.norm() < 1e-8)
      x = heading; // horizontal thrust perpendicular to the desired heading
    if (x.dot(heading) < 0.0)
      x *= -1.0;
    x.normalize();
    Eigen::Matrix3d rotation;
    rotation.col(0) = x;
    rotation.col(1) = z.cross(x);
    rotation.col(2) = z;
    return Eigen::Quaterniond(rotation).normalized();
  }

  RotorAllocation allocate(double collective,
                           const Eigen::Vector3d &torque) const {
    if (!std::isfinite(collective) || !torque.allFinite())
      throw std::invalid_argument("flight controller: nonfinite wrench");
    RotorAllocation result;
    result.requested_wrench << collective, torque;
    const double clipped = std::clamp(collective, 0.0, 4 * max_rotor_thrust_);
    result.collective_saturated = clipped != collective;
    const Eigen::Vector4d equal = Eigen::Vector4d::Constant(clipped / 4.0);
    Eigen::Vector4d desired;
    desired << clipped, torque;
    Eigen::Vector4d differential = inverse_mixer_ * (desired - mixer_ * equal);
    if (!differential.allFinite())
      throw std::overflow_error("flight controller: unrepresentable wrench");
    // Remove roundoff in the null-collective differential. Uniform scaling
    // preserves total thrust, including at asymmetric rotor/COM geometries.
    differential.array() -= differential.mean();
    for (int i = 0; i != 4; ++i) {
      if (differential[i] > 0.0)
        result.differential_scale =
            std::min(result.differential_scale,
                     (max_rotor_thrust_ - equal[i]) / differential[i]);
      else if (differential[i] < 0.0)
        result.differential_scale =
            std::min(result.differential_scale, -equal[i] / differential[i]);
    }
    const Eigen::Vector4d thrust =
        equal + result.differential_scale * differential;
    for (int i = 0; i != 4; ++i)
      result.target_rotor_speed[i] =
          std::sqrt(std::clamp(thrust[i], 0.0, max_rotor_thrust_) /
                    plant_.thrust_coefficient);
    result.allocated_wrench =
        mixer_ * (plant_.thrust_coefficient *
                  result.target_rotor_speed.array().square()).matrix();
    return result;
  }

  FlightControlOutput command(const RigidBodyState &state,
                              const Eigen::Vector3d &acceleration_command,
                              double yaw, double yaw_rate = 0.0) const {
    if (!acceleration_command.allFinite() || !std::isfinite(yaw_rate))
      throw std::invalid_argument("flight controller: nonfinite command");
    FlightControlOutput result;
    result.acceleration_command = acceleration_command;
    const Eigen::Vector3d force =
        plant_.mass * (acceleration_command -
                       Eigen::Vector3d(0.0, 0.0, -plant_.gravity));
    result.desired_orientation = desired_orientation(force, yaw);
    Eigen::Quaterniond error =
        state.orientation.conjugate() * result.desired_orientation;
    // q and -q describe the same attitude; use the shortest rotation, also
    // for 180-degree commands where the SO(3) skew error would vanish.
    if (error.w() < 0.0)
      error.coeffs() *= -1.0;
    result.desired_body_rate =
        control_.attitude_gain.cwiseProduct(2.0 * error.vec()) +
        state.orientation.conjugate() * Eigen::Vector3d(0.0, 0.0, yaw_rate);
    for (int i = 0; i != 3; ++i)
      result.desired_body_rate[i] =
          std::clamp(result.desired_body_rate[i], -control_.max_body_rate[i],
                     control_.max_body_rate[i]);
    const Eigen::Vector3d rate_feedback = control_.rate_gain.cwiseProduct(
        result.desired_body_rate - state.angular_velocity);
    const Eigen::Vector3d torque =
        plant_.inertia * rate_feedback +
        state.angular_velocity.cross(plant_.inertia * state.angular_velocity);
    const double collective =
        force.dot(state.orientation * Eigen::Vector3d::UnitZ());
    result.allocation = allocate(collective, torque);
    return result;
  }

private:
  RigidBodyParameters plant_;
  FlightControllerParameters control_;
  Eigen::Matrix4d mixer_;
  Eigen::Matrix4d inverse_mixer_;
  double max_rotor_thrust_;
};

} // namespace xgc_lightweight
