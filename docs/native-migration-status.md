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
| DMPC planner | `plan-dmpc` | Links the academic planner's ROS-free DMPC agent (configuration from the scenario YAML, the optimizer core and acados); its plans and setpoints are byte-equivalent to the academic fleet replay for one robot of a five-robot knot_fs150 fleet over 60 rounds (`plan_dmpc_replay`). No scene input yet (static and moving obstacles off), no planar or pass-through output, and no closed-loop run with `ctl-px4` or on a vehicle. |

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
