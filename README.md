# XGC2 Sync Runtime

One module skeleton for every onboard and station module: perception, estimation, planning, control, DMPC neighbor exchange and simulation adapters. Communications are one plugin family on the same skeleton, with Zenoh as the cross-host transport. Latency, loss, reordering and throughput are audited per link against exact definitions.

**It is not:** a planner, PX4 HIL, a ROS replacement mandate, or anything to do with AI chat. "Agent" in XGC2 means the robot `xgc-agent` ops process, which launches the host.

## Topology

One `xgc-rt-host` process per robot (and one on the station) loads every module as a `.so` plugin from one manifest. Development and simulation always run this way.

```text
            robot (container or onboard)                         other robots / station
 ┌──────────────────── xgc-rt-host ─────────────────────┐
 │  ros1-bridge ─▶ estimator ─▶ planner ─▶ controller ──┼─▶ (bridge ▶ MAVROS)
 │                               ▲  │                    │
 │  in-host hops: loopback       │  └── dmpc/plan ───────┼──▶ Zenoh over radio ◀──▶ peers
 │  every hop stamped + audited  └───── neighbor plans ◀─┼───
 └──────────────────────────────────────────────────────┘
```

Splitting modules across processes is optional later, only for a concrete need (e.g. ROS thread isolation on a real robot).

## Shape

| Piece | What |
|---|---|
| `abi/include/xgc_rt.h` | The C ABI (v1). A plugin is a `.so` exporting `xgc_rt_plugin_v1`, in any language. |
| `crates/xgc-rt-abi` | Rust mirror of the ABI, plus the safe plugin SDK (`Plugin` trait, `export_plugin!`) |
| `crates/xgc-rt-core` | Envelope v2, the lifecycle state machine, `Clock`/`RoundSchedule`, the `Transport` and `AuditSink` interfaces, and the manifest |
| `crates/xgc-rt-audit` | Lossless records, the multi-node merge, and the `audit-def/1` report (`xgc-rt-audit merge`) |
| `crates/xgc-rt-transport-loopback` | In-process transport with a seeded ground-truth impairment injector |
| `crates/xgc-rt-host` | The host process (`xgc-rt-host --manifest`): loader, round and dirty executor, restart policy, health log |
| `plugins/stub-*`, `plugins/c-stub` | Z1 domain stubs (Rust) and the pure-C plugin |

## Host rules (acceptance for first-party modules)

1. A new domain is a plugin plus a manifest entry. It never needs changes to host, clock, transport or audit code.
2. Each plugin has one responsibility, and its declared ports are its entire I/O.
3. Threads:
   - one executor thread runs all plugin calls;
   - transport IO threads only stamp, verify, audit and enqueue;
   - one audit writer thread writes records.
4. Plugins step on their round (`on_round`), on new input (`on_dirty`), or both. There is no busy-polling: an idle host measured 0.44 % CPU.
5. Every module has the lifecycle state machine (`Unconfigured → Inactive → Active ⇄ Degraded`, plus `Error` and `Finalized`), tested exhaustively, and its own domain state is visible in health.
6. IPC goes only through ports and transports.
7. Every channel is audited, and the audit is calibrated (`docs/audit-definitions.md`).

## Build, test and demo

```bash
cargo test                       # unit tests + Z1 exit tests (needs a C compiler for the C plugin)
examples/z1-pipeline/run.sh      # release build, 5-plugin host for 5 s, merged audit in out/z1-pipeline/merged
```

Rust ≥ 1.75 (matches the pinned zenoh 1.9.0).
