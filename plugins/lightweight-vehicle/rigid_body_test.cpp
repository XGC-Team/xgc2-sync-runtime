#include "rigid_body.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace xgc_lightweight;

namespace {

void require(bool condition, const std::string &message) {
  if (!condition)
    throw std::runtime_error(message);
}
void near(double actual, double expected, double tolerance,
          const std::string &message) {
  require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
          message + " actual=" + std::to_string(actual) +
              " expected=" + std::to_string(expected));
}
template <typename A, typename B>
void near_vector(const A &actual, const B &expected, double tolerance,
                 const std::string &message) {
  require(actual.allFinite() && (actual - expected).norm() <= tolerance, message);
}
template <typename Function> void rejects(Function operation, const char *message) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::invalid_argument &) {
    rejected = true;
  } catch (const std::overflow_error &) {
    rejected = true;
  }
  require(rejected, message);
}

void advance(RigidBodyModel &model, const Eigen::Vector4d &target, double duration,
             double h) {
  const int count = static_cast<int>(std::lround(duration / h));
  near(count * h, duration, 1e-12, "test duration must be an integer step count");
  for (int i = 0; i < count; ++i)
    model.step(target, h);
}

void parameters_from_link_mass_properties() {
  // Independent parallel-axis assembly from renderer constants and iris.sdf.
  // This checks the aggregation, not just the copied final parameter values.
  const double mass[6] = {0.275, 0.015, 0.005, 0.005, 0.005, 0.005};
  const Eigen::Vector3d positions[6] = {
      {0, 0, 0}, {0, 0, 0}, {0.13, -0.22, 0.023},
      {-0.13, 0.2, 0.023}, {0.13, 0.22, 0.023}, {-0.13, -0.2, 0.023}};
  const Eigen::Vector3d renderer_body_j =
      Eigen::Vector3d(0.029125, 0.029125, 0.055225) * (0.275 / 1.5) * 0.35;
  // iris.sdf stores 11-digit rounded values; the frozen aggregate uses these.
  const Eigen::Vector3d body_j(0.00186885417, 0.00186885417, 0.00354360417);
  near_vector(body_j, renderer_body_j, 6e-12, "SDF rounding of renderer inertia");
  const Eigen::Vector3d rotor_j =
      Eigen::Vector3d(9.75e-7, 0.000273104, 0.000274004) * 0.35;
  double total_mass = 0;
  Eigen::Vector3d com = Eigen::Vector3d::Zero();
  for (int i = 0; i < 6; ++i) {
    total_mass += mass[i];
    com += mass[i] * positions[i];
  }
  com /= total_mass;
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
  for (int i = 0; i < 6; ++i) {
    const Eigen::Vector3d d = positions[i] - com;
    const Eigen::Vector3d diagonal =
        i == 0 ? body_j : (i == 1 ? Eigen::Vector3d::Constant(1e-5) : rotor_j);
    inertia += diagonal.asDiagonal().toDenseMatrix() +
               mass[i] * (d.squaredNorm() * Eigen::Matrix3d::Identity() -
                          d * d.transpose());
  }
  const auto p = fs150EquivalentParameters();
  near(p.mass, total_mass, 1e-15, "aggregate mass");
  near_vector(p.body_origin_to_com, com, 1e-15, "aggregate COM");
  near_vector(p.inertia, inertia, 1e-15, "aggregate inertia");
  for (int i = 0; i < 4; ++i)
    near_vector(p.rotor_position[i] + com, positions[i + 2], 1e-15,
                "rotor offset must be COM-relative");
  std::cout << "  aggregate COM=" << com.transpose()
            << " J=" << inertia.diagonal().transpose() << '\n';
}

void free_fall() {
  RigidBodyModel model({1.0, -2.0, 0.0}, 0.7);
  auto s = model.state();
  s.velocity = {0.4, -0.3, 1.2};
  model.set_state(s);
  const double t = 1.0;
  advance(model, Eigen::Vector4d::Zero(), t, 0.01);
  const Eigen::Vector3d g(0, 0, -model.parameters().gravity);
  near_vector(model.state().position, s.position + t * s.velocity + 0.5 * t * t * g,
              1e-12, "ballistic position");
  near_vector(model.state().velocity, s.velocity + t * g, 1e-12,
              "ballistic velocity");
  near_vector(model.acceleration_world(), g, 1e-14, "ballistic acceleration");
  require(model.base_position().z() < -3.0, "kernel must not clamp to ground");
  near(model.state().orientation.angularDistance(s.orientation), 0.0, 1e-14,
       "free fall cannot change orientation");
}

void balanced_hover() {
  RigidBodyModel model({0.2, -0.3, 4.0}, 1.1);
  const auto &p = model.parameters();
  const double hover = std::sqrt(p.mass * p.gravity / (4.0 * p.thrust_coefficient));
  auto s = model.state();
  s.rotor_speed.setConstant(hover);
  model.set_state(s);
  advance(model, s.rotor_speed, 2.0, 0.002);
  const double drift = (model.state().position - s.position).norm();
  near(drift, 0, 2e-12, "balanced hover position");
  near(model.state().velocity.norm(), 0, 2e-12, "balanced hover velocity");
  near(model.state().angular_velocity.norm(), 0, 1e-12, "balanced hover torque");
  near(model.acceleration_world().norm(), 0, 1e-12, "balanced hover acceleration");
  std::cout << "  hover speed=" << hover << " drift_m=" << drift << '\n';
}

void single_rotor_signs() {
  // 1 N upward at each physical lever: FLU roll=left lever, pitch=-front
  // lever. Reaction yaw is opposite rotor spin, per Gazebo motor source.
  const Eigen::Vector3d moments[4] = {
      {-0.22, -0.13, -0.06}, {0.2, 0.13, -0.06},
      {0.22, -0.13, 0.06}, {-0.2, 0.13, 0.06}};
  for (int rotor = 0; rotor < 4; ++rotor) {
    RigidBodyModel model(Eigen::Vector3d::Zero());
    auto s = model.state();
    s.rotor_speed[rotor] = std::sqrt(1.0 / model.parameters().thrust_coefficient);
    model.set_state(s);
    const Eigen::Vector3d expected =
        moments[rotor].array() / model.parameters().inertia.diagonal().array();
    near_vector(model.angular_acceleration_body(), expected, 1e-12,
                "single rotor signed acceleration " + std::to_string(rotor));
    const double h = 1e-5;
    model.step(s.rotor_speed, h);
    near_vector(model.state().angular_velocity / h, expected, 1e-7,
                "single rotor integrates physical torque");
    for (int axis = 0; axis < 3; ++axis)
      require(model.state().angular_velocity[axis] * moments[rotor][axis] > 0,
              "single rotor roll/pitch/yaw sign");
    require(model.state().orientation.angularDistance(s.orientation) > 0,
            "torque must integrate attitude");
    std::cout << "  rotor=" << rotor << " alpha=" << expected.transpose() << '\n';
  }
}

void torque_free_coupling() {
  auto p = fs150EquivalentParameters();
  p.gravity = 0;
  p.inertia = Eigen::Vector3d(2, 3, 4).asDiagonal();
  RigidBodyModel model(Eigen::Vector3d::Zero(), 0.0, p);
  auto s = model.state();
  s.angular_velocity = {1, 2, 3};
  s.orientation = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX());
  model.set_state(s);
  near_vector(model.angular_acceleration_body(), Eigen::Vector3d(-3, 2, -0.5),
              1e-14, "Euler torque-free coupling");
  const double energy = 0.5 * s.angular_velocity.dot(p.inertia * s.angular_velocity);
  const Eigen::Vector3d momentum = s.orientation * (p.inertia * s.angular_velocity);
  advance(model, Eigen::Vector4d::Zero(), 2.0, 0.001);
  const auto &end = model.state();
  const double energy_error = std::abs(
      0.5 * end.angular_velocity.dot(p.inertia * end.angular_velocity) - energy);
  const double momentum_error =
      (end.orientation * (p.inertia * end.angular_velocity) - momentum).norm();
  near(energy_error, 0, 2e-10, "torque-free energy conservation");
  near(momentum_error, 0, 2e-10, "world angular momentum conservation");
  near(end.orientation.norm(), 1, 3e-15, "quaternion norm");
  std::cout << "  energy_error=" << energy_error
            << " world_momentum_error=" << momentum_error << '\n';

  // The API promises a full inertia tensor, not only a diagonal one.
  const Eigen::Matrix3d basis =
      Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  p.inertia = basis * p.inertia * basis.transpose();
  RigidBodyModel rotated(Eigen::Vector3d::Zero(), 0.0, p);
  s.angular_velocity = basis * Eigen::Vector3d(1, 2, 3);
  rotated.set_state(s);
  near_vector(rotated.angular_acceleration_body(), basis * Eigen::Vector3d(-3, 2, -0.5),
              1e-13, "non-diagonal inertia covariance");
}

void quaternion_and_frames() {
  auto p = fs150EquivalentParameters();
  p.gravity = 0;
  p.inertia = Eigen::Matrix3d::Identity();
  RigidBodyModel model({2, 3, 4}, 0.0, p);
  auto s = model.state();
  s.orientation = Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX());
  s.angular_velocity = {0.7, -0.4, 0.9};
  model.set_state(s);
  advance(model, Eigen::Vector4d::Zero(), 3.0, 0.002);
  const Eigen::Quaterniond expected = s.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(3.0 * s.angular_velocity.norm(), s.angular_velocity.normalized()));
  near(model.state().orientation.angularDistance(expected), 0, 2e-13,
       "body angular velocity must right-multiply quaternion");
  near(model.state().orientation.norm(), 1, 3e-15, "long spin quaternion norm");

  // Tilted thrust has a physical world direction; gravity remains world -Z.
  p.gravity = 9.8066;
  RigidBodyModel tilted(Eigen::Vector3d::Zero(), 0.0, p);
  s = tilted.state();
  s.orientation = Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitY());
  s.rotor_speed.setConstant(std::sqrt(p.mass * p.gravity / (4 * p.thrust_coefficient)));
  tilted.set_state(s);
  near_vector(tilted.acceleration_world(), Eigen::Vector3d(p.gravity, 0, -p.gravity),
              1e-13, "FLU +Z thrust rotates into ENU +X at positive pitch");
}

void base_com_kinematics() {
  auto p = fs150EquivalentParameters();
  p.gravity = 0;
  p.inertia = Eigen::Matrix3d::Identity();
  p.body_origin_to_com = {0.2, -0.1, 0.3};
  RigidBodyModel model({2, 3, 4}, 0.7, p);
  near_vector(model.base_position(), Eigen::Vector3d(2, 3, 4), 1e-14,
              "constructor receives base position");
  auto s = model.state();
  s.velocity = {0.4, 0.2, -0.3};
  s.angular_velocity = {0, 0, 2};
  model.set_state(s);
  const Eigen::Vector3d base = model.base_position();
  const Eigen::Vector3d velocity = model.base_velocity();
  const Eigen::Vector3d acceleration = model.base_acceleration();
  const double h = 0.0001;
  model.step(Eigen::Vector4d::Zero(), h);
  near_vector((model.base_position() - base) / h, velocity + 0.5 * h * acceleration,
              1e-8, "base velocity follows derivative of physical offset");
  near_vector((model.base_velocity() - velocity) / h, acceleration, 2e-4,
              "base acceleration includes centripetal term");

  RigidBodyModel torque(Eigen::Vector3d::Zero());
  s = torque.state();
  s.rotor_speed[0] = 400;
  torque.set_state(s);
  const Eigen::Vector3d v0 = torque.base_velocity();
  const Eigen::Vector3d a0 = torque.base_acceleration();
  torque.step(s.rotor_speed, 1e-6);
  near_vector((torque.base_velocity() - v0) / 1e-6, a0, 1e-7,
              "base acceleration includes angular-acceleration offset");
}

void motor_step_and_impulse() {
  RigidBodyModel model(Eigen::Vector3d::Zero());
  const auto &p = model.parameters();
  const double target = 500.0;
  const double t = 0.03;
  advance(model, Eigen::Vector4d::Constant(target), t, 0.001);
  near(model.state().rotor_speed[0], target * (1 - std::exp(-t / p.motor_tau_up)),
       2e-12, "exact motor step up");
  // Analytical integral of squared exponential speed gives thrust impulse.
  const double tau = p.motor_tau_up;
  const double integral_w2 = target * target *
      (t - 2 * tau * (1 - std::exp(-t / tau)) +
       0.5 * tau * (1 - std::exp(-2 * t / tau)));
  const double vz = 4 * p.thrust_coefficient * integral_w2 / p.mass - p.gravity * t;
  near(model.state().velocity.z(), vz, 2e-6, "analytic motor-induced thrust impulse");
  const double impulse_error = std::abs(model.state().velocity.z() - vz);
  const double initial = model.state().rotor_speed[0];
  advance(model, Eigen::Vector4d::Zero(), t, 0.001);
  near(model.state().rotor_speed[0], initial * std::exp(-t / p.motor_tau_down),
       2e-12, "exact motor step down");

  model.step(Eigen::Vector4d::Constant(1e6), 0.1);
  require((model.state().rotor_speed.array() <= p.max_rotor_speed).all(),
          "upper motor saturation");
  model.step(Eigen::Vector4d::Constant(-1e6), 0.1);
  require((model.state().rotor_speed.array() >= 0).all(), "nonnegative motors");
  std::cout << "  impulse_velocity_error=" << impulse_error << '\n';
}

RigidBodyState convergence_run(double h) {
  RigidBodyModel model({0.1, -0.2, 1}, 0.6);
  auto s = model.state();
  s.orientation = s.orientation * Eigen::Quaterniond(
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()));
  s.angular_velocity = {0.4, -0.2, 0.6};
  s.velocity = {0.2, 0.1, -0.1};
  s.rotor_speed = {200, 250, 220, 210};
  model.set_state(s);
  advance(model, Eigen::Vector4d(400, 380, 410, 390), 0.08, h);
  advance(model, Eigen::Vector4d(250, 270, 230, 260), 0.08, h);
  return model.state();
}
double state_distance(const RigidBodyState &a, const RigidBodyState &b) {
  return (a.position - b.position).norm() + (a.velocity - b.velocity).norm() +
         a.orientation.angularDistance(b.orientation) +
         (a.angular_velocity - b.angular_velocity).norm();
}
void step_convergence() {
  // Compare against a separately refined trajectory, not the RHS implementation.
  const auto reference = convergence_run(0.00003125);
  const double e1 = state_distance(convergence_run(0.002), reference);
  const double e2 = state_distance(convergence_run(0.001), reference);
  const double e4 = state_distance(convergence_run(0.0005), reference);
  require(e4 > 1e-12 && e1 > e2 && e2 > e4, "h/h2/h4 must converge");
  require(e1 / e2 > 12 && e2 / e4 > 12, "expected fourth-order convergence");
  std::cout << "  h/h2/h4 errors=" << e1 << ',' << e2 << ',' << e4
            << " ratios=" << e1 / e2 << ',' << e2 / e4 << '\n';
}

void input_validation() {
  RigidBodyModel model({0, 0, 1});
  const auto before = model.state();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (double dt : {-0.01, 0.10001, nan, inf})
    rejects([&] { model.step(Eigen::Vector4d::Zero(), dt); }, "invalid dt accepted");
  rejects([&] { model.step(Eigen::Vector4d::Constant(nan), 0.001); },
          "nonfinite target accepted");
  rejects([&] { model.step(Eigen::Vector4d::Constant(inf), 0.001); },
          "infinite target accepted");
  model.step(Eigen::Vector4d::Constant(1000), 0);
  near(state_distance(before, model.state()), 0, 0, "zero/invalid dt cannot mutate");
  near_vector(model.state().rotor_speed, before.rotor_speed, 0, "no partial motor step");
  auto s = before;
  s.orientation.coeffs().setZero();
  rejects([&] { model.set_state(s); }, "zero quaternion accepted");
  s = before;
  s.rotor_speed[0] = -1;
  rejects([&] { model.set_state(s); }, "negative initial motor accepted");
  s.rotor_speed[0] = model.parameters().max_rotor_speed + 1;
  rejects([&] { model.set_state(s); }, "excess initial motor accepted");
  s = before;
  s.velocity[1] = inf;
  rejects([&] { model.set_state(s); }, "nonfinite state accepted");
  s = before;
  s.orientation.coeffs() *= 2;
  model.set_state(s);
  near(model.state().orientation.norm(), 1, 1e-15, "initial quaternion normalization");
  auto p = fs150EquivalentParameters();
  p.inertia(0, 0) = -1;
  rejects([&] { RigidBodyModel bad({0, 0, 0}, 0, p); }, "negative inertia accepted");
  p = fs150EquivalentParameters();
  p.inertia(0, 1) = 1e-3;
  rejects([&] { RigidBodyModel bad({0, 0, 0}, 0, p); }, "asymmetric inertia accepted");
  p = fs150EquivalentParameters();
  p.motor_tau_up = 0;
  rejects([&] { RigidBodyModel bad({0, 0, 0}, 0, p); }, "zero motor tau accepted");
  p = fs150EquivalentParameters();
  p.mass = nan;
  rejects([&] { RigidBodyModel bad({0, 0, 0}, 0, p); }, "nonfinite mass accepted");
  p = fs150EquivalentParameters();
  p.inertia = Eigen::Matrix3d::Identity() * 1e-320;
  rejects([&] { RigidBodyModel bad({0, 0, 0}, 0, p); },
          "unrepresentable inertia inverse accepted");

  p = fs150EquivalentParameters();
  p.inertia = Eigen::Matrix3d::Identity();
  RigidBodyModel extreme({0, 0, 1}, 0, p);
  s = extreme.state();
  s.angular_velocity = {0, 0, 1e5};
  extreme.set_state(s);
  rejects([&] { extreme.step(Eigen::Vector4d::Zero(), 0.1); },
          "extreme finite spin must have a bounded integration budget");
  near(state_distance(extreme.state(), s), 0, 0,
       "failed substep budget must not publish partial state");
  near_vector(extreme.state().rotor_speed, s.rotor_speed, 0,
              "failed substep budget must not publish partial motors");
  s = extreme.state();
  s.angular_velocity = {1e200, 0, 0};
  extreme.set_state(s);
  rejects([&] { extreme.step(Eigen::Vector4d::Zero(), 0.001); },
          "unresolvable finite spin must fail immediately");
  near_vector(extreme.state().angular_velocity, s.angular_velocity, 0,
              "failed resolution must not mutate angular velocity");
}

} // namespace

int main() {
  const std::pair<const char *, void (*)()> tests[] = {
      {"parameter provenance", parameters_from_link_mass_properties},
      {"free fall", free_fall}, {"balanced hover", balanced_hover},
      {"single rotor signs", single_rotor_signs},
      {"torque-free coupling", torque_free_coupling},
      {"quaternion and frames", quaternion_and_frames},
      {"base/COM kinematics", base_com_kinematics},
      {"motor step and impulse", motor_step_and_impulse},
      {"step convergence", step_convergence}, {"input validation", input_validation}};
  int failed = 0;
  std::cout << std::setprecision(12);
  for (const auto &test : tests) {
    try {
      test.second();
      std::cout << "PASS " << test.first << '\n';
    } catch (const std::exception &e) {
      ++failed;
      std::cerr << "FAIL " << test.first << ": " << e.what() << '\n';
    }
  }
  std::cout << "RigidBody: " << std::size(tests) - failed << '/' << std::size(tests)
            << " passed\n";
  return failed == 0 ? 0 : 1;
}
