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

One instance can also advance a batch of up to 8 robots of the same model
(`robots = N`, `initial_poses = [x0, y0, z0, yaw0, x1, ...]` with four numbers
per robot). Their state is one contiguous vector; each step reads the host
time once and advances every robot to the same grid point with the same
per-robot function a single-robot instance uses, so a batch and independent
instances given the same controls publish identical states and stamps. Robot 0
keeps the single-robot ports; robot r uses the same names with `_r` appended
(`setpoint_3`, `pose_3`, ...). The ABI's 64-port limit sets the 8-robot bound;
larger fleets use several instances.

A plant catches up from the host time, so waking it every 1 ms only adds
work. In a plant-only host set `period_ms = output_ms` and `trigger =
"on_round"`: each instance wakes once per output period on the shared grid
and input never wakes it. Where the host keeps 1 ms rounds, use `trigger =
"on_dirty"`, `wake_ms = output_ms` and `step_budget_ms = output_ms`; without
input that relative timer skips a few output periods, and without the budget
a step slowed by a busy link past 10 ms is abandoned as hung. Measured cost at
1/32/100 robots: `docs/validation/lightweight-batch-20260928/`.

The host wakes the instance. It advances complete `epoch + k * step` intervals,
without waiting for other robots. Controls become active on the first grid
boundary at or after both their effective header time and receive time. A late
control cannot change the integrated past. Delayed wakeups preserve queued
command boundaries; feedback publishes the latest integrated state at the
configured output period rather than serializing every catch-up step. This
does not recover control samples already dropped by transport QoS, nor prove
that the complete host/controller workload meets an onboard deadline.

A control stamped more than `max_future_ms` (default 1000) after its receipt
is a clock-domain error, not a schedule: it is dropped. At most `max_pending`
(default 1024) controls wait per robot; later ones are dropped. Both drops are
counted and logged as warnings on the 1st, 2nd, 4th, 8th... occurrence.

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
position-only command have no effect. The reported angular rate is the yaw
change actually integrated in the last step, not the last commanded rate.

FCU requests follow PX4 only where the existing controller and Stop flows
depend on it; a refused request leaves armed/mode unchanged and the next
`fcu_state` shows it:

| Request | Plant |
| --- | --- |
| `OFFBOARD` | Follows setpoints. Refused without a setpoint younger than `offboard_timeout_ms` (default 500). When the stream stops for that long the plant reports `AUTO.LOITER` and holds (PX4 `COM_OF_LOSS_T`). |
| `POSCTL`, `ALTCTL`, `AUTO.LOITER` | Hold: brake to rest (0.2 s velocity time constant) and stay there; no RC sticks are modelled. |
| `AUTO.LAND` | Descends at 0.7 m/s (PX4 default `MPC_LAND_SPEED`) with horizontal braking, then disarms on touchdown. |
| Any other mode | Refused and logged; the previous mode stays. |
| Disarm | Refused while more than 1 cm above the initial ground (no force flag exists in `xgc.fcu_request/1`). |

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
