# XGC2 Sync Runtime

One module skeleton for every onboard and station module: perception, estimation, planning, control, DMPC neighbor exchange and simulation adapters. Communications are one plugin family on the same skeleton, with Zenoh as the cross-host transport. Latency, loss, reordering and throughput are audited per link against exact definitions.

**It is not:** a planner, PX4 HIL, a ROS replacement mandate, or anything to do with AI chat. "Agent" in XGC2 means the robot `xgc-agent` ops process, which launches the aggregator.

## Topology

An aggregator is a plain process (`xgc-rt-host`) that loads the `.so` modules one manifest lists. Usually there is one per robot; a computer may run several for independent jobs.

```text
            robot (container or onboard)                         other robots / station
 ┌─────────────────── aggregator (xgc-rt-host) ─────────┐
 │  ros_io ─▶ estimator ─▶ planner ─▶ controller ─▶ ros_io ─▶ MAVROS (ROS pub)
 │   (ROS sub)                    ▲  │                  │
 │  same process: memory only    │  └── dmpc/plan ─────┼──▶ Zenoh over radio ◀──▶ peers
 │  (one thread per module)      └───── neighbor plans ◀┼───  stamped + audited
 └──────────────────────────────────────────────────────┘
```

## Shape

| Piece | What |
|---|---|
| `abi/include/xgc_rt.h` | The C ABI (v1). A plugin is a `.so` exporting `xgc_rt_plugin_v1`, in any language. |
| `crates/xgc-rt-abi` | Rust mirror of the ABI, plus the safe plugin SDK (`Plugin` trait, `export_plugin!`) |
| `crates/xgc-rt-core` | Envelope v2, the lifecycle state machine, `Clock`/`RoundSchedule`, the `Transport` and `AuditSink` interfaces, and the manifest |
| `crates/xgc-rt-audit` | Lossless records, the multi-node merge, and the `audit-def/1` report (`xgc-rt-audit merge`) |
| `crates/xgc-rt-transport-loopback` | In-memory stand-in for a link between nodes, with a seeded ground-truth impairment injector (tests only; never used between modules of one aggregator) |
| `crates/xgc-rt-host` | The aggregator (`xgc-rt-host --manifest`): loader, one thread per module, memory handoff, watchdog, restart policy, health and step logs |
| `plugins/stub-*`, `plugins/c-stub` | Z1 domain stubs (Rust) and the pure-C plugin |

## Host rules (acceptance for first-party modules)

1. A new domain is a plugin plus a manifest entry. It never needs changes to host, clock, transport or audit code.
2. Each plugin has one responsibility, and its declared ports are its entire I/O.
3. Threads: one thread per module (D9), so a slow or blocking module stalls only itself. The main thread routes link frames and runs the clock probe and the watchdog. Transport IO threads only stamp, verify, audit and enqueue; writer threads append audit and step records.
4. Modules step on their round (`on_round`), on new input (`on_dirty`), or both. There is no busy-polling. Each step reads one snapshot of its inputs taken at step start.
4a. **Same process = memory only.** An output write hands one shared sample to each same-process reader's input (a bounded queue, or with `latest = true` only the newest sample) and wakes it. No envelope, transport or pub/sub between modules of one aggregator. The link (Zenoh) is used only when the roster has other nodes. `steps.jsonl` records each step's start, end and the samples it read.
4b. **Watchdog.** A step over `step_budget_ms` (default one period) marks the module Degraded; the next step within budget recovers it. A step over 10× the budget is a hang: the instance is abandoned and, if the restart policy allows, replaced. After `session.max_abandoned` (default 2) abandons the aggregator stops and exits nonzero, so the Agent restarts it.
5. Every module has the lifecycle state machine (`Unconfigured → Inactive → Active ⇄ Degraded`, plus `Error` and `Finalized`), tested exhaustively, and its own domain state is visible in health.
6. A module's declared inputs and outputs are its entire I/O. Between processes, only the link (Zenoh) is used.
6a. **Modules never touch ROS.** No ros::init/rospy, no publish/subscribe, no ROS libraries in a domain plugin. The aggregator's `ros_io` module does ordinary ROS subscribe and publish: topics are copied into module inputs, module outputs are published as topics. It is not the `ros1_bridge` package. VRPN, simulators and third-party ROS stacks stay ROS nodes, reached through `ros_io`.
7. Every link (cross-process) channel is audited, and the audit is calibrated (`docs/audit-definitions.md`).

## Build, test and demo

```bash
cargo test                       # unit tests + Z1 exit tests (needs a C compiler for the C plugin)
examples/z1-pipeline/run.sh      # release build, 5-plugin host for 5 s, merged audit in out/z1-pipeline/merged
```

Rust ≥ 1.75 (matches the pinned zenoh 1.9.0).
