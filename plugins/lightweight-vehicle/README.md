# Lightweight vehicle

Numerical plants behind the existing native plugin ABI, with no ROS, Gazebo,
MAVLink, sockets or plugin-owned threads. One instance simulates one robot; a
host can hold many instances, or each onboard host can integrate its own robot.
No controller or planner implementation is changed.

`model` is `fs150`, `scout` or `mecanum`. `initial_pose = [x, y, z, yaw]` is in
world metres/radians. `epoch_ns` is the same shared Session epoch given to the
host. `step_ms` defaults to 1; `output_ms` defaults to 10. Both belong to simulator
configuration, independently of where the host runs. Use the host's existing
wall clock for wireless HIL; this plugin does not need Gazebo `/clock` or a
central tick publisher.

The host wakes the instance. It advances complete `epoch + k * step` intervals,
without waiting for other robots. Controls become active on the first grid
boundary at or after both their effective header time and receive time. A late
control cannot change the integrated past. Delayed wakeups preserve queued
command boundaries; feedback publishes the latest integrated state at the
configured output period rather than serializing every catch-up step. This
does not recover control samples already dropped by transport QoS, nor prove
that the complete host/controller workload meets an onboard deadline.

| Model | Actual control input | Plant |
| --- | --- | --- |
| FS150 | `setpoint`, `xgc.position_target/1`, world frame 1; `fcu_request`, `xgc.fcu_request/1` | SMC acceleration-only input drives the exact ZOH translational double integrator. The position/velocity branch provides the ideal FCU response needed for the unchanged controller's Takeoff/Hover/Landing states. |
| Scout | `cmd_vel`, `xgc.twist/1`, body forward and yaw rate | Existing `DelayedPlanarVelocity` with the Gazebo unicycle defaults: 5 ms delay and two 5 ms lags, 1.5 m/s and 1 rad/s limits. Midpoint SE(2) pose integration; no lateral motion. |
| Mecanum | `cmd_vel`, `xgc.twist/1`, body forward/left and yaw rate | Exact constant-body-velocity SE(2) integration. Existing `ugv_sim_single.launch` limits: 1.5 m/s in each direction and pi/2 rad/s. These are simulation defaults, not measured MCU limits. |

FS150's position-mode constants are those of the existing numerical PX4 test
plant: position gain 1.5, velocity response 0.3 s, speed 1.5 m/s, acceleration
3 m/s², plus any enabled acceleration feedforward. **None of these limits or
gains are applied to SMC's acceleration-only output.** Disarmed flight has
gravity and ground contact at the initial z. The minimal attitude model is
level with ideal commanded yaw; it does not validate thrust, body rates, PX4
inner loops, aerodynamic effects or EKF behavior. FORCE is rejected only when
an acceleration axis is enabled; unused mask bits in the controller's
position-only command have no effect. FCU mode requests are reflected in the
synthetic state, including ALTCTL during takeoff initialization.

Outputs `pose` and `velocity` are mandatory world-frame truth. Optional
`paired_state` carries the same pose/velocity timestamp. FS150 also provides
synthetic `fcu_state` and body-frame `imu` for the unchanged control interface.
Ground models do not publish an IMU or FCU state. No estimator, hover-thrust,
attitude-rate actuator or planner PVA shortcut is added.

Build with `bash scripts/build-lightweight-vehicle.sh OUT.so`. It needs
xgc2-math headers including `geometry/kinematics.hpp` (0.5.10 source) and Eigen.
Run `bash plugins/lightweight-vehicle/test.sh` for analytic trajectories,
response delay and ABI receive/effective-time behavior. Run
`bash plugins/lightweight-vehicle/test-controller.sh /path/to/libctl_px4.so`
with that ELF's libraries available to exercise actual SMC takeoff, 10 Hz PVA
reference tracking and landing through both real plugin vtables. This test
supplies in-memory host callbacks; actual scheduler/transport tests are separate.
`cargo test -p xgc-rt-host --test lightweight_vehicle` loads the built ELF in
two real hosts with independent manual clocks and loopback transport.
