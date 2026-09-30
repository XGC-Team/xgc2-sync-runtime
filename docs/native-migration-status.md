# Native migration status

The September 2026 validation establishes a functional native-module path on
Linux/ROS Noetic with real upstream C++ algorithms and acados. It does not
establish deployment on a vehicle. Exact sources, commands and results are in
[the evidence record](validation/native-20260926/README.md).

| Component | Current native entry | Evidence and boundary |
|---|---|---|
| Rigid-state ESKF | `est-rigid-state` | Upstream runtime and FSM compiled into the plugin; byte-equivalent replay and ROS sensor integration. |
| Hover-thrust estimator | `est-hover-thrust` | Upstream runtime and FSM; byte-equivalent replay and real estimator output in the five-module composition fixture. |
| DFBC kernel | `ctl-dfbc` | Upstream controller functions; byte-equivalent kernel replay. This standalone plugin is distinct from the DFBC tracking backend inside `ctl-px4`. |
| PX4 multirotor control | `ctl-px4` | Links current ROS-free controller core and real acados runtime; byte-equivalent PX4 local, DFBC and NMPC core traces. |
| Reference generation | `ref-trajectory` | Links current ROS-free core; retained analytic/sample payloads and state sequences compared with the ROS node/core; UAV waypoint and external polynomial interfaces retired. |
| ROS boundary | `ros-io` | Real ROS subscriptions, publications and service calls; contains the ROS dependencies. |
| UGV control | No UGV domain wrapper in this tree | Upstream ROS-free core extraction is separate work; it is not an aggregated UGV deployment here. |
| DMPC exchange | `dmpc-rounds` | Owns neighbor admission and round timing. A source round is delivered before the next planner beat; the planner does not apply a second delay. The 27 module tests pass on this integration tree. |
| DMPC planner | `plan-dmpc` | Uses the academic `DmpcAgent` for lifecycle, configuration, optimizer, local sensing crop, planar/pass-through output, and operator-goal bootstrap. Native transport retains the existing `xgc.dmpc.scene_snapshot/1` fixed scene records and mission timeline. No second scene protocol or planner state machine. The C++ configuration/scene/goal receipt tests pass, including planar scene-loss hold and late-trigger rejection. Five strict byte-equivalent replays pass: no scene, Knot with scene loss, mixed UAV, mixed Scout, and Act1 pass-through. Scene UUIDs and source timestamps are preserved. Vehicle acceptance remains open. |
| DMPC closed loop | `plan-dmpc` + `ctl-px4` | The current ports pass the Knot single-robot numerical closed loop with both PX4 local and SMC backends. Each backend reproduces two flights byte-for-byte. SMC uses raw pose/velocity and acceleration-only numerical integration; no estimator or hover-thrust input is bound. Reference arrival at an activation boundary uses the controller fix 4074bb0. This does not establish PX4 attitude dynamics, SITL or vehicle acceptance. |
| DMPC fleet closed loop | One native host per robot | The incoming fleet tests still use obsolete planner ports and must be migrated before the merge stack is accepted. The required gate is the shared-scene fleet over explicit Zenoh endpoints, including delayed and lost peer plans. Private container lifecycle and Agent enrollment are separate from this algorithm gate. Current flight/mission closure must be demonstrated on the deployed bundle and is not inferred from process survival. |

The native-hover integration fixture loads `ros-io`, `est-rigid-state`,
`est-hover-thrust`, `ctl-px4` and `ref-trajectory` into one host. Domain handoff
is in memory. ROS connects that host to the test's software plant and PX4/MAVROS
stand-in. In this fixture the stand-in publishes actuator feedback and zero
hover-thrust estimates; the real estimator owns that channel. The original
three-backend fixture still uses synthetic hover estimates and is labeled as
such in its output.

`examples/z1-pipeline` remains a skeleton demonstration using four Rust stubs
and one C stub. It is not the native graph. Deployment uses the existing
`xgc-rt-render` compositions and the Core/Agent process workflow. See
[native deployment](native-deployment.md) for the selected role and clock
wiring; packaging, remote Run/Stop and the resulting robot behavior each
need their own acceptance evidence.

The host watchdog measures elapsed wall time. A 1 ms session period gives a
default 1 ms step budget and 10 ms abandonment threshold, which failed once
in this non-real-time environment. The functional composition fixture now
sets the controller step budget explicitly to 10 ms (100 ms abandonment).
That test setting makes no 1 kHz deadline guarantee; deployment needs measured
budget selection and a fault-handling policy. Production defaults were not
changed. Distributed clock configuration and its current timeout behavior are
documented in [the time model](time-model.md).
