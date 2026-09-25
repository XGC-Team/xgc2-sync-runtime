// Reference for the ctl-dfbc equality test: the same samples straight into
// the shared driver and the upstream controller, no host or ports.
// stdin lines:  "S <14 doubles: stamp p3 v3 q4 w3>" or "R <23 doubles: stamp p3 v3 a3 j3 s3 yaw yaw_rate yaw_accel>"
// (lines must already be in the plugin's apply order)
// stdout lines: "<stamp> <thrust> <q4> <rate3> <perr3> <success> <flags>" (%.17g)

#include <cstdio>
#include <cstring>

#include "dfbc_driver.hpp"

int main() {
  ctl_dfbc::Driver d;
  d.configure(xgc2_math::control::DfbcGeometricConfig{}, 0.01, 0.1);
  char kind[2];
  while (std::scanf("%1s", kind) == 1) {
    if (kind[0] == 'R') {
      xgc_flat_ref_v1 r{};
      double* f[] = {&r.stamp, &r.position[0], &r.position[1], &r.position[2], &r.velocity[0], &r.velocity[1], &r.velocity[2],
                     &r.acceleration[0], &r.acceleration[1], &r.acceleration[2], &r.jerk[0], &r.jerk[1], &r.jerk[2],
                     &r.snap[0], &r.snap[1], &r.snap[2], &r.yaw, &r.yaw_rate, &r.yaw_accel};
      for (double* x : f) if (std::scanf("%lf", x) != 1) return 2;
      d.set_reference(r);
    } else {
      xgc_rigid_state_v1 s{};
      double* f[] = {&s.stamp, &s.position[0], &s.position[1], &s.position[2], &s.velocity[0], &s.velocity[1], &s.velocity[2],
                     &s.q_wxyz[0], &s.q_wxyz[1], &s.q_wxyz[2], &s.q_wxyz[3], &s.body_rate[0], &s.body_rate[1], &s.body_rate[2]};
      for (double* x : f) if (std::scanf("%lf", x) != 1) return 2;
      xgc_attitude_rate_cmd_v1 c;
      if (!d.on_state(s, &c)) continue;
      std::printf("%.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g %u %u\n", c.stamp, c.specific_thrust,
                  c.q_wxyz[0], c.q_wxyz[1], c.q_wxyz[2], c.q_wxyz[3], c.body_rate[0], c.body_rate[1], c.body_rate[2],
                  c.position_error[0], c.position_error[1], c.position_error[2], c.success, c.flags);
    }
  }
  return 0;
}
