#include "vehicle_model.hpp"

#include <cassert>
#include <iostream>

using namespace xgc_lightweight;

namespace {
FlightSetpoint acceleration_setpoint(const Eigen::Vector3d &value) {
  FlightSetpoint result;
  result.acceleration = value;
  for (auto &enabled : result.acceleration_enabled)
    enabled = true;
  return result;
}

FlightSetpoint position_setpoint(const Eigen::Vector3d &value) {
  FlightSetpoint result;
  result.position = value;
  for (auto &enabled : result.position_enabled)
    enabled = true;
  return result;
}

// Stream `setpoint` every step, as the controller does, for `steps` steps.
void fly(FlightModel &model, const FlightSetpoint &setpoint, int steps,
         double dt = 0.001) {
  for (int i = 0; i != steps; ++i) {
    model.setpoint(setpoint);
    assert(model.step(dt) == FlightEvent::None);
  }
}

// Arm on the ground and enter OFFBOARD behind a live setpoint stream.
void take_off(FlightModel &model, const FlightSetpoint &setpoint) {
  model.setpoint(setpoint);
  assert(model.request_arm(true));
  assert(model.request_mode(FlightMode::Offboard));
}
} // namespace

int main() {
  // Unarmed commands cannot drive motors. The stationary accelerometer sees
  // gravity-balanced specific force from the simplified ground support.
  FlightModel flight({0.0, 0.0, 0.0});
  auto acceleration = acceleration_setpoint({8.0, -2.0, 1.0});
  acceleration.position = {100.0, 100.0, 100.0}; // disabled payload is ignored
  flight.setpoint(acceleration);
  flight.step(0.1);
  assert(flight.state().position.norm() == 0.0);
  assert(flight.angular_velocity_body().norm() == 0.0);
  assert((flight.specific_force_body() - Eigen::Vector3d(0.0, 0.0, 9.8066))
             .norm() < 1e-6);
  take_off(flight, acceleration);
  fly(flight, acceleration, 10);
  assert(flight.orientation().angularDistance(Eigen::Quaterniond::Identity()) <
         0.05); // real attitude delay, not an ideal acceleration integrator
  assert(flight.acceleration().x() < 0.5);
  assert(flight.angular_velocity_body().norm() > 0.0);
  assert((flight.control_output().acceleration_command - acceleration.acceleration)
             .norm() == 0.0); // SMC still bypasses the P/V branch and its clamp
  const double initial_acceleration_x = flight.acceleration().x();
  int acceleration_90_ms = -1;
  for (int i = 0; i != 1990; ++i) {
    flight.setpoint(acceleration);
    assert(flight.step(0.001) == FlightEvent::None);
    if (acceleration_90_ms < 0 && flight.acceleration().x() >= 7.2)
      acceleration_90_ms = i + 11;
  }
  assert(acceleration_90_ms > 10 && acceleration_90_ms < 1000);
  const double acceleration_error =
      (flight.acceleration() - acceleration.acceleration).norm();
  assert((flight.acceleration() - acceleration.acceleration).norm() < 0.15);
  assert(flight.state().velocity.x() > 8.0);
  assert(flight.state().position.x() > 4.0);
  const auto tilt = flight.orientation() * Eigen::Vector3d::UnitZ();
  assert(tilt.x() > 0.5 && tilt.y() < -0.1);
  assert(std::abs(flight.orientation().norm() - 1.0) < 1e-12);
  assert((flight.orientation() * flight.specific_force_body() -
          (flight.acceleration() + Eigen::Vector3d(0.0, 0.0, 9.8066)))
             .norm() < 1e-6);

  // OFFBOARD requires a fresh stream, including before the first command.
  FlightModel stream({0.0, 0.0, 0.0});
  assert(stream.request_arm(true));
  assert(!stream.request_mode(FlightMode::Offboard));
  stream.setpoint(position_setpoint({0.0, 0.0, 0.0}));
  for (int i = 0; i != 500; ++i)
    stream.step(0.001);
  assert(!stream.request_mode(FlightMode::Offboard));
  assert(stream.mode() == FlightMode::Hold);
  stream.setpoint(position_setpoint({0.0, 0.0, 0.0}));
  assert(stream.request_mode(FlightMode::Offboard));

  // Closed-loop P/V takeoff and hover, followed by LAND's original velocity
  // branch. Bounds allow the motor/attitude response instead of exact ideal
  // double-integrator trajectories. The base measurement keeps ground_z.
  FlightModel lifecycle({0.0, 0.0, 0.15});
  const auto hover = position_setpoint({0.6, -0.4, 2.15});
  take_off(lifecycle, hover);
  fly(lifecycle, hover, 10000);
  const double hover_error = (lifecycle.state().position - hover.position).norm();
  assert((lifecycle.state().position - hover.position).norm() < 0.01);
  assert(lifecycle.state().velocity.norm() < 0.01);
  assert(lifecycle.acceleration().norm() < 0.02);
  assert(!lifecycle.landed());
  assert(!lifecycle.request_arm(false));
  assert(lifecycle.armed() && lifecycle.mode() == FlightMode::Offboard);
  assert(lifecycle.request_mode(FlightMode::Land));
  for (int i = 0; i != 2000; ++i)
    assert(lifecycle.step(0.001) == FlightEvent::None);
  assert(std::abs(lifecycle.state().velocity.z() + FlightModel::kLandSpeed) <
         0.02);
  assert(lifecycle.armed());
  int landed_at = -1;
  double peak_contact_force = 0.0;
  for (int i = 0; i != 5000 && landed_at < 0; ++i) {
    if (lifecycle.step(0.001) == FlightEvent::Landed) {
      landed_at = i;
      peak_contact_force = lifecycle.specific_force_body().norm();
    }
  }
  assert(landed_at > 0);
  assert(peak_contact_force > 100.0); // resolved impact reaction is observable
  assert(!lifecycle.armed() && lifecycle.landed());
  assert(lifecycle.mode() == FlightMode::Land);
  assert(std::abs(lifecycle.state().position.z() - 0.15) < 1e-12);
  assert(lifecycle.state().velocity.z() >= -1e-12);
  for (int i = 0; i != 1000; ++i)
    assert(lifecycle.step(0.001) == FlightEvent::None);
  assert(std::abs(lifecycle.state().position.z() - 0.15) < 1e-12);
  assert(lifecycle.acceleration().norm() < 0.01);
  assert(lifecycle.request_arm(true) && lifecycle.request_arm(false));

  // A stopped stream triggers exactly one fallback; real dynamics brake the
  // vehicle, instead of integrating the stale acceleration for 60 seconds.
  FlightModel silent({0.0, 0.0, 1.0});
  const auto drift = acceleration_setpoint({0.5, 0.0, 0.0});
  take_off(silent, position_setpoint({0.0, 0.0, 2.0}));
  fly(silent, position_setpoint({0.0, 0.0, 2.0}), 8000);
  silent.setpoint(drift);
  int lost_at = -1;
  for (int i = 0; i != 10000; ++i)
    if (silent.step(0.001) == FlightEvent::OffboardLost) {
      assert(lost_at < 0);
      lost_at = i;
    }
  assert(lost_at == 500);
  assert(silent.mode() == FlightMode::Hold && silent.armed());
  assert(std::abs(silent.state().position.x()) < 0.5);
  assert(silent.state().velocity.norm() < 0.01);
  assert(silent.state().position.z() > 1.9);
  FlightModel patient({0.0, 0.0, 1.0}, 0.0, 2.0);
  take_off(patient, drift);
  lost_at = -1;
  for (int i = 0; i != 3000 && lost_at < 0; ++i)
    if (patient.step(0.001) == FlightEvent::OffboardLost)
      lost_at = i;
  assert(lost_at == 2000);

  // Both rate and absolute-yaw commands are realized through torque/motors.
  FlightModel turning({0.0, 0.0, 0.0}, 0.25);
  FlightSetpoint spin = position_setpoint({0.0, 0.0, 1.0});
  turning.setpoint(spin);
  turning.step(0.001);
  assert(std::abs(turning.yaw() - 0.25) < 1e-12);
  assert(std::abs(turning.yaw_rate()) < 1e-9);
  take_off(turning, spin);
  fly(turning, spin, 8000);
  spin.yaw_rate = 0.5;
  fly(turning, spin, 10);
  assert(turning.yaw() - 0.25 < 0.003);
  assert(turning.yaw_rate() < 0.3);
  fly(turning, spin, 3990);
  assert(std::abs(turning.yaw_rate() - 0.5) < 0.02);
  assert(std::abs(turning.angular_velocity_body().z() - 0.5) < 0.02);
  spin.yaw_enabled = true;
  spin.yaw = -1.0;
  const double before_turn = turning.yaw();
  fly(turning, spin, 1);
  assert(std::abs(xgc2_math::normalizeAngle(turning.yaw() - before_turn)) < 0.002);
  assert(std::abs(xgc2_math::normalizeAngle(turning.yaw() - spin.yaw)) > 1.0);
  fly(turning, spin, 6000);
  assert(std::abs(xgc2_math::normalizeAngle(turning.yaw() - spin.yaw)) < 0.01);
  assert(std::abs(turning.yaw_rate()) < 0.01);
  assert(turning.request_mode(FlightMode::Hold));
  turning.step(0.001);
  assert(std::abs(turning.yaw() - spin.yaw) < 0.01);

  // Infeasible ACC commands remain explicit and do not overwrite state.
  const auto infeasible = acceleration_setpoint({0.0, 0.0, 1e4});
  turning.setpoint(infeasible);
  assert(turning.request_mode(FlightMode::Offboard));
  turning.step(0.001);
  assert(turning.control_output().allocation.collective_saturated);
  assert(turning.acceleration().z() < 100.0);
  assert(turning.state().position.allFinite());
  fly(turning, infeasible, 99);
  assert(turning.control_output().allocation.saturated());
  spin.yaw_rate = 0.0;
  fly(turning, spin, 20000);
  assert((turning.state().position - spin.position).norm() < 0.02);
  assert(turning.state().velocity.norm() < 0.02);
  assert(!turning.control_output().allocation.saturated());

  // The legacy mixed branch keeps its 1.5 m/s velocity-command cap,
  // +/-3 m/s^2 feedback clamp, and enabled feedforward acceleration.
  FlightModel mixed({0.0, 0.0, 0.0});
  auto mixed_command = position_setpoint({100.0, 0.0, 0.0});
  mixed_command.velocity_enabled[0] = true;
  mixed_command.velocity.x() = 10.0;
  mixed_command.acceleration_enabled[0] = true;
  mixed_command.acceleration.x() = 0.7;
  take_off(mixed, mixed_command);
  mixed.step(0.001);
  assert((mixed.control_output().acceleration_command -
          Eigen::Vector3d(3.7, 0.0, 0.0)).norm() < 1e-12);

  const auto unchanged = turning.state();
  const auto unchanged_orientation = turning.orientation();
  assert(turning.step(0.0) == FlightEvent::None);
  assert((turning.state().position - unchanged.position).norm() == 0.0);
  assert(turning.orientation().angularDistance(unchanged_orientation) == 0.0);
  for (const double dt : {-0.001, std::numeric_limits<double>::infinity()}) {
    bool refused = false;
    try {
      turning.step(dt);
    } catch (const std::invalid_argument &) {
      refused = true;
    }
    assert(refused);
    assert((turning.state().position - unchanged.position).norm() == 0.0);
  }

  // Equivalent caller chunking runs the same bounded inner-loop time steps.
  FlightModel fine({0.0, 0.0, 0.0});
  FlightModel chunked({0.0, 0.0, 0.0});
  const auto target = position_setpoint({0.5, -0.3, 1.0});
  take_off(fine, target);
  take_off(chunked, target);
  fly(fine, target, 2000);
  fly(chunked, target, 200, 0.01);
  assert((fine.state().position - chunked.state().position).norm() < 1e-9);
  assert(fine.orientation().angularDistance(chunked.orientation()) < 1e-9);

  std::cout << "FCU response: a_x(10ms)=" << initial_acceleration_x
            << ", a_x90%_ms=" << acceleration_90_ms
            << ", acceleration_error(2s)=" << acceleration_error
            << ", hover_error(10s)=" << hover_error
            << ", impact_specific_force=" << peak_contact_force << "\n";

  ScoutModel scout({{0.0, 0.0}, 0.0});
  scout.command(0.0, 1.0, 0.0);
  scout.advance(0.004);
  assert(scout.pose().position.norm() == 0.0);
  for (int i = 0; i != 996; ++i)
    scout.advance(double(i + 5) * 0.001);
  const double expected = 0.995 - 0.005 * (1.0 - std::exp(-0.995 / 0.005));
  assert(std::abs(scout.pose().position.x() - expected) < 1e-5);
  assert(scout.pose().position.y() == 0.0);
  // Converge after stop with the same delayed velocity response as Gazebo.
  scout.command(1.0, 0.0, 0.0);
  for (int i = 0; i != 100; ++i)
    scout.advance(double(i + 1001) * 0.001);
  assert(scout.velocity().linear_m_s < 1e-7);

  MecanumModel sideways({{0.0, 0.0}, 1.5707963267948966});
  sideways.command(0.0, 1.0, 0.0);
  sideways.step(2.0);
  assert((sideways.pose().position - Eigen::Vector2d(-2.0, 0.0)).norm() <
         1e-12);
  MecanumModel arc({{0.0, 0.0}, 0.0});
  arc.command(1.0, 0.5, 1.0);
  arc.step(6.283185307179586);
  assert(arc.pose().position.norm() < 1e-12);
  std::cout << "lightweight six-DOF FCU and unchanged ground plants: passed\n";
}
