#include "flight_controller.hpp"

#include <cassert>
#include <iostream>

using namespace xgc_lightweight;

namespace {
Eigen::Vector4d physical_wrench(const RigidBodyParameters &p,
                                const Eigen::Vector4d &speed) {
  Eigen::Vector4d wrench = Eigen::Vector4d::Zero();
  for (int i = 0; i != 4; ++i) {
    const double thrust = p.thrust_coefficient * speed[i] * speed[i];
    wrench[0] += thrust;
    wrench.segment<3>(1) +=
        p.rotor_position[i].cross(Eigen::Vector3d(0.0, 0.0, thrust));
    wrench[3] += p.rotor_yaw_sign[i] * p.moment_ratio * thrust;
  }
  return wrench;
}
}

int main() {
  const auto p = fs150EquivalentParameters();
  const FlightController controller(p);
  const double hover = p.mass * p.gravity;
  const Eigen::Vector3d torque(0.01, -0.01, 0.005);
  auto result = controller.allocate(hover, torque);
  assert(!result.saturated());
  Eigen::Vector4d expected;
  expected << hover, torque;
  assert((physical_wrench(p, result.target_rotor_speed) - expected).norm() <
         1e-12);
  // Saturation must preserve feasible collective, while uniformly reducing
  // the requested differential. The plant independently supplies the wrench.
  result = controller.allocate(hover, {100.0, -50.0, 20.0});
  assert(result.saturated() && !result.collective_saturated);
  assert(result.differential_scale > 0.0 && result.differential_scale < 0.01);
  assert((result.target_rotor_speed.array() >= 0.0).all());
  assert((result.target_rotor_speed.array() <= p.max_rotor_speed).all());
  assert(std::abs(physical_wrench(p, result.target_rotor_speed)[0] - hover) <
         1e-12);
  result = controller.allocate(1e6, Eigen::Vector3d::Zero());
  assert(result.collective_saturated);
  assert(std::abs(result.allocated_wrench[0] -
                  4 * p.thrust_coefficient * p.max_rotor_speed *
                      p.max_rotor_speed) < 1e-12);
  result = controller.allocate(-hover, {0.1, 0.1, 0.1});
  assert(result.collective_saturated && result.target_rotor_speed.norm() == 0.0);

  for (const Eigen::Vector3d force :
       {Eigen::Vector3d(2.0, -1.0, 3.0), Eigen::Vector3d::UnitX().eval(),
        Eigen::Vector3d::Zero().eval()}) {
    const auto q = FlightController::desired_orientation(force, 0.0);
    assert(q.coeffs().allFinite() && std::abs(q.norm() - 1.0) < 1e-12);
    if (force.norm() > 0.0)
      assert((q * Eigen::Vector3d::UnitZ() - force.normalized()).norm() <
             1e-12);
  }
  RigidBodyModel model({0.0, 0.0, 1.0});
  const auto tilted_heading =
      FlightController::desired_orientation({2.0, -1.0, 3.0}, 0.7)
          .toRotationMatrix();
  assert(std::abs(std::atan2(tilted_heading(1, 0), tilted_heading(0, 0)) - 0.7) <
         1e-12);
  auto state = model.state();
  auto output = controller.command(state, {2.0, 0.0, 0.0}, 0.0);
  assert(output.desired_body_rate.y() > 0.0);
  // The controller produces rotor commands, never an attitude/state update.
  assert(model.state().orientation.angularDistance(state.orientation) == 0.0);
  state.orientation.coeffs() *= -1.0;
  const auto equivalent = controller.command(state, {2.0, 0.0, 0.0}, 0.0);
  assert((output.allocation.target_rotor_speed -
          equivalent.allocation.target_rotor_speed).norm() < 1e-12);
  const auto half_turn = controller.command(model.state(), {0.0, 0.0, 0.0},
                                           std::acos(-1.0));
  assert(half_turn.desired_body_rate.z() > 1.0);
  assert((half_turn.desired_body_rate.array().abs() <=
          controller.parameters().max_body_rate.array()).all());

  // Each positive body torque must produce positive angular acceleration
  // through the actual motor/rigid-body dynamics, with arbitrary motor order.
  for (int axis = 0; axis != 3; ++axis) {
    RigidBodyModel response({0.0, 0.0, 1.0});
    auto equilibrium = response.state();
    equilibrium.rotor_speed =
        controller.allocate(hover, Eigen::Vector3d::Zero()).target_rotor_speed;
    response.set_state(equilibrium);
    Eigen::Vector3d request = Eigen::Vector3d::Zero();
    request[axis] = 0.001;
    const auto motors = controller.allocate(hover, request).target_rotor_speed;
    response.step(motors, 0.001);
    assert(response.state().angular_velocity[axis] > 0.0);
  }
  // Rotor numbering is not assumed by the mixer.
  auto reordered = p;
  std::swap(reordered.rotor_position[0], reordered.rotor_position[2]);
  std::swap(reordered.rotor_yaw_sign[0], reordered.rotor_yaw_sign[2]);
  const FlightController reordered_controller(reordered);
  result = reordered_controller.allocate(hover, torque);
  assert((physical_wrench(reordered, result.target_rotor_speed) - expected)
             .norm() < 1e-12);
  std::cout << "flight controller: allocation, saturation, attitude and torque signs passed\n";
}
