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
  // The SMC acceleration is the plant input: no PVA reset, virtual position
  // feedback, 3 m/s^2 FCU-position clamp or gravity is inserted in this branch.
  FlightModel flight({0.0, 0.0, 0.0});
  auto acceleration = acceleration_setpoint({8.0, -2.0, 1.0});
  acceleration.position = {100.0, 100.0, 100.0}; // disabled payload is ignored
  flight.setpoint(acceleration);
  flight.step(0.1);
  assert(flight.state().position.norm() == 0.0); // unarmed commands cannot move
  take_off(flight, acceleration);
  fly(flight, acceleration, 100, 0.01);
  assert((flight.state().position - Eigen::Vector3d(4.0, -1.0, 0.5)).norm() <
         1e-12);
  assert((flight.state().velocity - Eigen::Vector3d(8.0, -2.0, 1.0)).norm() <
         1e-12);

  // OFFBOARD needs a live stream: no setpoint yet, then one that went stale.
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

  // Takeoff, hover, then AUTO.LAND: descend at the PX4 default land speed and
  // disarm on touchdown (probe a: the old plant hovered forever, armed).
  FlightModel lifecycle({0.0, 0.0, 0.15});
  const auto hover = position_setpoint({0.0, 0.0, 2.15});
  take_off(lifecycle, hover);
  fly(lifecycle, hover, 10000);
  assert(std::abs(lifecycle.state().position.z() - 2.15) < 1e-5);
  assert(lifecycle.state().velocity.norm() < 1e-5);
  assert(!lifecycle.landed());
  // Probe b: a disarm without force is refused in the air, state unchanged.
  assert(!lifecycle.request_arm(false));
  assert(lifecycle.armed() && lifecycle.mode() == FlightMode::Offboard);
  assert(lifecycle.request_mode(FlightMode::Land));
  for (int i = 0; i != 2500; ++i)
    assert(lifecycle.step(0.001) == FlightEvent::None);
  assert(std::abs(lifecycle.state().velocity.z() + FlightModel::kLandSpeed) <
         1e-3);
  assert(lifecycle.armed());
  int landed_at = -1;
  for (int i = 0; i != 5000 && landed_at < 0; ++i)
    if (lifecycle.step(0.001) == FlightEvent::Landed)
      landed_at = i;
  assert(landed_at > 0);
  assert(!lifecycle.armed() && lifecycle.landed());
  assert(lifecycle.mode() == FlightMode::Land);
  assert(lifecycle.state().position.z() == 0.15);
  assert(lifecycle.state().velocity.norm() == 0.0);
  for (int i = 0; i != 1000; ++i)
    assert(lifecycle.step(0.001) == FlightEvent::None);
  assert(lifecycle.state().position.z() == 0.15);
  // On the ground a disarm is accepted.
  assert(lifecycle.request_arm(true) && lifecycle.request_arm(false));

  // Probe c: the last acceleration setpoint is not integrated forever. After
  // the offboard timeout the plant brakes and holds (the old plant reached
  // 900 m and 30 m/s after 60 s).
  FlightModel silent({0.0, 0.0, 1.0});
  const auto drift = acceleration_setpoint({0.5, 0.0, 0.0});
  take_off(silent, drift);
  int lost_at = -1;
  for (int i = 0; i != 60000; ++i)
    if (silent.step(0.001) == FlightEvent::OffboardLost) {
      assert(lost_at < 0);
      lost_at = i;
    }
  assert(lost_at == 500);
  assert(silent.mode() == FlightMode::Hold && silent.armed());
  // 0.5 s at 0.5 m/s^2, then v * tau of braking: 0.0625 + 0.25 * 0.2 m.
  assert(std::abs(silent.state().position.x() - 0.1125) < 1e-3);
  assert(silent.state().velocity.norm() < 1e-9);
  assert(std::abs(silent.state().position.z() - 1.0) < 1e-12);
  // A different timeout is honoured.
  FlightModel patient({0.0, 0.0, 1.0}, 0.0, 2.0);
  take_off(patient, drift);
  lost_at = -1;
  for (int i = 0; i != 3000 && lost_at < 0; ++i)
    if (patient.step(0.001) == FlightEvent::OffboardLost)
      lost_at = i;
  assert(lost_at == 2000);

  // The reported yaw rate is the realized one, not the last command.
  FlightModel turning({0.0, 0.0, 0.0}, 0.25);
  FlightSetpoint spin = acceleration_setpoint({0.0, 0.0, 0.0});
  spin.yaw_rate = 0.5;
  turning.setpoint(spin);
  turning.step(0.001);
  assert(turning.yaw() == 0.25 && turning.yaw_rate() == 0.0); // disarmed
  take_off(turning, spin);
  fly(turning, spin, 1000);
  assert(std::abs(turning.yaw() - 0.75) < 1e-12);
  assert(std::abs(turning.yaw_rate() - 0.5) < 1e-9);
  spin.yaw_enabled = true;
  spin.yaw = 1.0;
  fly(turning, spin, 2);
  assert(turning.yaw() == 1.0 && turning.yaw_rate() == 0.0);
  assert(turning.request_mode(FlightMode::Hold));
  turning.step(0.001);
  assert(turning.yaw() == 1.0 && turning.yaw_rate() == 0.0);

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
  std::cout << "lightweight actual-input plants: passed\n";
}
