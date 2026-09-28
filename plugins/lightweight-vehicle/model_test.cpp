#include "vehicle_model.hpp"

#include <cassert>
#include <iostream>

using namespace xgc_lightweight;

int main() {
  // The SMC acceleration is the plant input: no PVA reset, virtual position
  // feedback, 3 m/s^2 FCU-position clamp or gravity is inserted in this branch.
  FlightModel flight({0.0, 0.0, 0.0});
  FlightSetpoint acceleration;
  acceleration.position = {100.0, 100.0, 100.0}; // disabled payload is ignored
  acceleration.acceleration = {8.0, -2.0, 1.0};
  for (auto &enabled : acceleration.acceleration_enabled)
    enabled = true;
  flight.setpoint(acceleration);
  flight.step(0.1);
  assert(flight.state().position.norm() == 0.0); // unarmed commands cannot move
  flight.arm(true);
  flight.offboard(true);
  for (int i = 0; i != 100; ++i)
    flight.step(0.01);
  assert((flight.state().position - Eigen::Vector3d(4.0, -1.0, 0.5)).norm() <
         1e-12);
  assert((flight.state().velocity - Eigen::Vector3d(8.0, -2.0, 1.0)).norm() <
         1e-12);

  FlightModel lifecycle({0.0, 0.0, 0.15});
  FlightSetpoint position;
  for (auto &enabled : position.position_enabled)
    enabled = true;
  position.position = {0.0, 0.0, 1.15};
  lifecycle.setpoint(position);
  lifecycle.arm(true);
  lifecycle.offboard(true);
  for (int i = 0; i != 10000; ++i)
    lifecycle.step(0.001);
  assert(std::abs(lifecycle.state().position.z() - 1.15) < 1e-5);
  assert(lifecycle.state().velocity.norm() < 1e-5);
  lifecycle.arm(false);
  for (int i = 0; i != 2000; ++i)
    lifecycle.step(0.001);
  assert(lifecycle.state().position.z() == 0.15);
  assert(lifecycle.state().velocity.norm() == 0.0);

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
