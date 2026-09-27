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
| Reference generation | `ref-trajectory` | Links current ROS-free core; analytic, sampled and polynomial interfaces compared with the original ROS node and core. |
| ROS boundary | `ros-io` | Real ROS subscriptions, publications and service calls; contains the ROS dependencies. |
| UGV control | No UGV domain wrapper in this tree | Upstream ROS-free core extraction is separate work; it is not an aggregated UGV deployment here. |
| DMPC | Exchange/round protocol plugins | `dmpc-exchange-demo` and `dmpc-rounds` exercise exchange, timing and coordination. Their existence does not establish a migrated full DMPC optimizer or its closed-loop vehicle acceptance. |
| DMPC planner | `plan-dmpc` | Links the academic planner's ROS-free DMPC agent (configuration from the scenario YAML, the optimizer core and acados) and takes the shared scene (`xgc.scene.snapshot/1`, `xgc.scene.state/1`; `ros-io` bridges them from the scene runtime). UAVs publish `xgc.position_target/1`, Scout UGVs (`planar_reference_output`) `xgc.planar_pva/1` (`ros-io` publishes it as `PlanarPvaReference`); the legacy backend's pass-through mode publishes the stage-1 sample and runs the node's zero-order-hold timer. Plans and setpoints are byte-equivalent to the academic fleet replay over 60–80 rounds (`plan_dmpc_replay`): knot_fs150 without the scene; unmodified knot_fs150 with its static posts and a withheld scene state (position holds); unmodified mixed_circle (5 UAVs, 4 Scouts, static obstacles, a constant-velocity mover) for a UAV and for a Scout; act1_mega pass-through (164 rocks, 8 m local sensing crop, uniform-velocity leader started 2 m ahead, ideal plant, hold timer from clock samples). No goal input (a uniform-velocity leader holds its start). |
| DMPC closed loop | `plan-dmpc` + `ctl-px4` | One aggregator, deterministic lockstep on a manual clock (`closed_loop_dmpc_px4`): plan-dmpc's setpoint goes to ctl-px4's `alg_setpoint` in memory (px4_local, Custom1), a software plant with a PX4/MAVROS stand-in (px4_standin.py's position mode; the estimate is the plant truth, no ESKF) closes the loop, and the controller and planner report each tick and round (`tick_done`, `round_done`). knot_fs150 robot 1 with its scene and recorded neighbor plans: takeoff, Hover, Custom1 before the rolling rounds, tracking error p50 about 4 cm, and two flights byte-equal. A software plant, not PX4 SITL or a vehicle; one closed-loop robot, the neighbors open loop. |
| DMPC fleet closed loop | `plan-dmpc` (+ `ctl-px4` per UAV) per robot | Whole fleets fly together (`closed_loop_fleet`), one aggregator per robot as deployed, the plans robot to robot over the link (loopback transport), in deterministic lockstep on one manual clock with a software plant per robot: the unmodified knot_fs150 (5 UAVs, scene on) and the unmodified mixed_circle (5 UAVs, 4 Scouts following their planar setpoints, static obstacles and a mover). Every UAV: Custom1 before the rolling rounds; tracking error p50 2-5 cm; two flights byte-equal; each robot's closed-loop rounds equal an offline run fed its peers' round k - 1 plans strictly in order (a one-round-late feed does not). Software plants (no UGV controller module), no ESKF, no radio impairment. |

The native-hover integration fixture loads `ros-io`, `est-rigid-state`,
`est-hover-thrust`, `ctl-px4` and `ref-trajectory` into one host. Domain handoff
is in memory. ROS connects that host to the test's software plant and PX4/MAVROS
stand-in. In this fixture the stand-in publishes actuator feedback and zero
hover-thrust estimates; the real estimator owns that channel. The original
three-backend fixture still uses synthetic hover estimates and is labeled as
such in its output.

`examples/z1-pipeline` remains a skeleton demonstration using four Rust stubs
and one C stub. It is not this native graph. At the reviewed source baseline,
the native graph lives in integration-test manifests. A targeted review of
XGC's Agent/Core/container/packaging sources found no dedicated sync-runtime
launch wiring. Generic process-management capability is not proof that this
graph is packaged, configured, started or health-checked by Agent/Core. Any
subsequent deployment work needs its own source identity and acceptance.

The host watchdog measures elapsed wall time. A 1 ms session period gives a
default 1 ms step budget and 10 ms abandonment threshold, which failed once
in this non-real-time environment. The functional composition fixture now
sets the controller step budget explicitly to 10 ms (100 ms abandonment).
That test setting makes no 1 kHz deadline guarantee; deployment needs measured
budget selection and a fault-handling policy. Production defaults were not
changed. Distributed clock configuration and its current timeout behavior are
documented in [the time model](time-model.md).
