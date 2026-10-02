#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace xgc_lightweight {

struct RigidBodyParameters {
  double mass{0.310}; // kg, including base, IMU and four rotor links
  Eigen::Matrix3d inertia{
      Eigen::Vector3d(0.0027741165893548385, 0.0026090971893548386,
                      0.00515920977).asDiagonal()}; // kg m^2, about COM
  std::array<Eigen::Vector3d, 4> rotor_position{
      Eigen::Vector3d(0.13, -0.22, 0.023 - 0.0014838709677419356),
      Eigen::Vector3d(-0.13, 0.2, 0.023 - 0.0014838709677419356),
      Eigen::Vector3d(0.13, 0.22, 0.023 - 0.0014838709677419356),
      Eigen::Vector3d(-0.13, -0.2, 0.023 - 0.0014838709677419356)};
  Eigen::Vector4d rotor_yaw_sign{-1.0, -1.0, 1.0, 1.0};
  double thrust_coefficient{5.33969944334e-6}; // N / (rad/s)^2
  double moment_ratio{0.06};                 // yaw torque / thrust, m
  double motor_tau_up{0.006};               // s
  double motor_tau_down{0.012};             // s
  double max_rotor_speed{1100.0};           // rad/s
  double gravity{9.8066};                   // positive magnitude, m/s^2
  Eigen::Vector3d body_origin_to_com{0.0, 0.0, 0.0014838709677419356};
};

// Provenance (paths relative to devops/products/ros1/simulator/gazebo-sim):
// fs150-sitl/scripts/render_fs150_indoor_sdf.py and
// fs150-sitl/models/fs150/iris.sdf.
// Base: .275 kg, J = Iris J*(.275/1.5)*.35; IMU: .015 kg, J=1e-5 I.
// Four .005 kg rotors: J=diag(9.75e-7,.000273104,.000274004)*.35,
// at the stated base-frame offsets. Parallel-axis aggregation about the
// total COM gives the inertia above (off-diagonals cancel), using stored SDF
// base inertia (.00186885417,.00186885417,.00354360417), rounded from scaling.
// GazeboMotorModel uses body yaw torque -turning_direction*force*momentConstant
// with CCW=+1, CW=-1: rotor 0/1 CCW therefore have NEGATIVE reaction torque.
// Verified against PX4 v1.12.0's Tools/sitl_gazebo gitlink
// 822050a7ab6fd87972e59f16312f451bce217a56, include/gazebo_motor_model.h:43-44
// and src/gazebo_motor_model.cpp:243-246 (PX4/PX4-SITL_gazebo on GitHub).
// This is the renderer's Iris-equivalent model, NOT real-airframe calibration.
// Rotor spin gyroscopic effects, air drag, rolling moments, wind and contacts
// are deliberately absent. The FCU wrapper owns ground/contact behaviour.
inline RigidBodyParameters fs150EquivalentParameters() { return {}; }

struct RigidBodyState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()}; // COM, world ENU, m
  Eigen::Vector3d velocity{Eigen::Vector3d::Zero()}; // COM, world ENU, m/s
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; // FLU->ENU
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()}; // body, rad/s
  Eigen::Vector4d rotor_speed{Eigen::Vector4d::Zero()}; // nonnegative rad/s
};

// Pure 6DoF plant. A command holds target rotor speeds for dt; attitude evolves
// ONLY through torque and qdot = .5*q*(0,omega_body). Motors follow their exact
// first-order response, sampled at every RK4 stage. Rigid-body state uses RK4
// with substeps <=2 ms (and <=.1 rad of rotation at the initial angular rate).
class RigidBodyModel {
public:
  explicit RigidBodyModel(
      const Eigen::Vector3d &initial_base_position, double yaw = 0.0,
      const RigidBodyParameters &params = fs150EquivalentParameters())
      : parameters_(params) {
    validate_parameters(parameters_);
    if (!initial_base_position.allFinite() || !std::isfinite(yaw))
      throw std::invalid_argument("nonfinite initial rigid-body pose");
    inverse_inertia_ = parameters_.inertia.llt().solve(Eigen::Matrix3d::Identity());
    if (!inverse_inertia_.allFinite())
      throw std::invalid_argument("inertia inverse is not representable");
    state_.orientation = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    state_.position = initial_base_position +
                      state_.orientation * parameters_.body_origin_to_com;
    if (!state_.position.allFinite())
      throw std::invalid_argument("overflow in initial COM position");
  }

  const RigidBodyState &state() const { return state_; }
  const RigidBodyParameters &parameters() const { return parameters_; }

  // Controlled initialization/reset entry. Validates before mutation and
  // normalizes a finite nonzero quaternion; COM coordinates are not shifted.
  void set_state(const RigidBodyState &state) {
    validate_state(state);
    RigidBodyState next = state;
    next.orientation.normalize();
    state_ = next;
  }

  // dt=0 is a validated no-op. Reject negative/nonfinite dt and pauses >.1 s;
  // callers catching up a longer elapsed time must supply bounded chunks.
  // Finite commands saturate to [0,max_rotor_speed]. Invalid input or numeric
  // overflow leaves the previous state intact (no partial step publication).
  // Limit each call to 10000 substeps so extreme finite rates cannot hang it.
  void step(const Eigen::Vector4d &target_rotor_speed, double dt) {
    if (!target_rotor_speed.allFinite() || !std::isfinite(dt) || dt < 0.0 ||
        dt > 0.1)
      throw std::invalid_argument("invalid rotor command or dt (expected [0,.1])");
    if (dt == 0.0)
      return;
    const Eigen::Vector4d target = target_rotor_speed.cwiseMax(0.0).cwiseMin(
        parameters_.max_rotor_speed);
    RigidBodyState next = state_;
    double remaining = dt;
    unsigned int substeps = 0;
    while (remaining > 0.0) {
      if (++substeps > 10000)
        throw std::overflow_error("rigid-body step exceeds substep budget");
      const double rate = next.angular_velocity.stableNorm();
      // An unresolvable extreme state must fail rather than loop forever.
      const double h = std::min({remaining, 0.002, 0.1 / std::max(1.0, rate)});
      if (!std::isfinite(h) || h <= 0.0 || remaining - h == remaining)
        throw std::overflow_error("rigid-body step cannot resolve angular rate");
      rk4(next, target, h);
      validate_state(next);
      remaining = h == remaining ? 0.0 : remaining - h;
    }
    state_ = next;
  }

  // Instantaneous endpoint accelerations, including gravity in world ENU.
  Eigen::Vector3d acceleration_world() const {
    return linear_acceleration(state_);
  }
  Eigen::Vector3d angular_acceleration_body() const {
    return angular_acceleration(state_);
  }
  Eigen::Vector3d base_position() const {
    return state_.position - state_.orientation * parameters_.body_origin_to_com;
  }
  Eigen::Vector3d base_velocity() const {
    return state_.velocity - state_.orientation *
        state_.angular_velocity.cross(parameters_.body_origin_to_com);
  }
  Eigen::Vector3d base_acceleration() const {
    const Eigen::Vector3d &r = parameters_.body_origin_to_com;
    const Eigen::Vector3d &w = state_.angular_velocity;
    return acceleration_world() - state_.orientation *
        (angular_acceleration_body().cross(r) + w.cross(w.cross(r)));
  }

private:
  static void validate_parameters(const RigidBodyParameters &p) {
    const auto positive = [](double x) { return std::isfinite(x) && x > 0.0; };
    if (!positive(p.mass) || !positive(p.thrust_coefficient) ||
        !positive(p.motor_tau_up) || !positive(p.motor_tau_down) ||
        !positive(p.max_rotor_speed) || !std::isfinite(p.gravity) ||
        p.gravity < 0.0 || !std::isfinite(p.moment_ratio) || p.moment_ratio < 0.0 ||
        !p.inertia.allFinite() || !p.body_origin_to_com.allFinite() ||
        !p.rotor_yaw_sign.allFinite())
      throw std::invalid_argument("invalid rigid-body parameters");
    if (!p.inertia.isApprox(p.inertia.transpose(), 1e-12) ||
        Eigen::LLT<Eigen::Matrix3d>(p.inertia).info() != Eigen::Success)
      throw std::invalid_argument("inertia must be symmetric positive definite");
    for (int i = 0; i < 4; ++i)
      if (!p.rotor_position[i].allFinite() || std::abs(p.rotor_yaw_sign[i]) != 1.0)
        throw std::invalid_argument("invalid rotor position or reaction sign");
    const double max_force = p.thrust_coefficient * p.max_rotor_speed *
                             p.max_rotor_speed;
    if (!std::isfinite(max_force) || !std::isfinite(4.0 * max_force / p.mass))
      throw std::invalid_argument("unrepresentable rotor thrust");
  }

  void validate_state(const RigidBodyState &s) const {
    const double qnorm = s.orientation.norm();
    if (!s.position.allFinite() || !s.velocity.allFinite() ||
        !s.orientation.coeffs().allFinite() || !std::isfinite(qnorm) ||
        qnorm < 1e-12 || !s.angular_velocity.allFinite() ||
        !s.rotor_speed.allFinite() || (s.rotor_speed.array() < 0.0).any() ||
        (s.rotor_speed.array() > parameters_.max_rotor_speed).any())
      throw std::invalid_argument("invalid rigid-body state");
  }

  Eigen::Vector4d motor_at(const Eigen::Vector4d &initial,
                         const Eigen::Vector4d &target, double t) const {
    Eigen::Vector4d speed;
    for (int i = 0; i < 4; ++i) {
      const double tau = target[i] > initial[i] ? parameters_.motor_tau_up
                                               : parameters_.motor_tau_down;
      // expm1 avoids cancellation for tiny dt. This is a convex interpolation.
      speed[i] = initial[i] + (target[i] - initial[i]) * (-std::expm1(-t / tau));
      speed[i] = std::clamp(speed[i], 0.0, parameters_.max_rotor_speed);
    }
    return speed;
  }

  Eigen::Vector4d forces(const RigidBodyState &s) const {
    return parameters_.thrust_coefficient * s.rotor_speed.array().square().matrix();
  }
  Eigen::Vector3d linear_acceleration(const RigidBodyState &s) const {
    return Eigen::Vector3d(0.0, 0.0, -parameters_.gravity) +
           s.orientation.normalized() *
               Eigen::Vector3d(0.0, 0.0, forces(s).sum() / parameters_.mass);
  }
  Eigen::Vector3d angular_acceleration(const RigidBodyState &s) const {
    const Eigen::Vector4d f = forces(s);
    Eigen::Vector3d torque = Eigen::Vector3d::Zero();
    for (int i = 0; i < 4; ++i)
      torque += parameters_.rotor_position[i].cross(Eigen::Vector3d(0, 0, f[i])) +
                Eigen::Vector3d(0, 0, parameters_.rotor_yaw_sign[i] *
                                           parameters_.moment_ratio * f[i]);
    return inverse_inertia_ *
           (torque - s.angular_velocity.cross(parameters_.inertia *
                                             s.angular_velocity));
  }

  struct Derivative {
    Eigen::Vector3d position, velocity, angular_velocity;
    Eigen::Vector4d quaternion; // Eigen coefficient order x,y,z,w
  };
  Derivative derivative(const RigidBodyState &s) const {
    const Eigen::Quaterniond spin(0.0, s.angular_velocity.x(),
                                 s.angular_velocity.y(), s.angular_velocity.z());
    return {s.velocity, linear_acceleration(s), angular_acceleration(s),
            0.5 * (s.orientation * spin).coeffs()};
  }
  static RigidBodyState stage(const RigidBodyState &s, const Derivative &d,
                             double t, const Eigen::Vector4d &motor) {
    RigidBodyState result = s;
    result.position += t * d.position;
    result.velocity += t * d.velocity;
    result.angular_velocity += t * d.angular_velocity;
    result.orientation.coeffs() += t * d.quaternion;
    result.rotor_speed = motor;
    return result;
  }
  void rk4(RigidBodyState &s, const Eigen::Vector4d &target, double h) const {
    const Eigen::Vector4d middle = motor_at(s.rotor_speed, target, 0.5 * h);
    const Eigen::Vector4d end = motor_at(s.rotor_speed, target, h);
    const Derivative k1 = derivative(s);
    const Derivative k2 = derivative(stage(s, k1, 0.5 * h, middle));
    const Derivative k3 = derivative(stage(s, k2, 0.5 * h, middle));
    const Derivative k4 = derivative(stage(s, k3, h, end));
    s.position += (h / 6.0) * (k1.position + 2.0 * k2.position +
                               2.0 * k3.position + k4.position);
    s.velocity += (h / 6.0) * (k1.velocity + 2.0 * k2.velocity +
                               2.0 * k3.velocity + k4.velocity);
    s.angular_velocity += (h / 6.0) *
        (k1.angular_velocity + 2.0 * k2.angular_velocity +
         2.0 * k3.angular_velocity + k4.angular_velocity);
    s.orientation.coeffs() += (h / 6.0) *
        (k1.quaternion + 2.0 * k2.quaternion + 2.0 * k3.quaternion + k4.quaternion);
    const double qnorm = s.orientation.norm();
    if (!std::isfinite(qnorm) || qnorm < 1e-12)
      throw std::overflow_error("rigid-body quaternion overflow");
    s.orientation.normalize();
    s.rotor_speed = end;
  }

  RigidBodyParameters parameters_;
  Eigen::Matrix3d inverse_inertia_;
  RigidBodyState state_;
};

} // namespace xgc_lightweight
