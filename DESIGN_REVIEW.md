# XGC2 Sync Runtime: design pack for author review

**STATUS: DESIGN FREEZE, awaiting author review.** Coding stopped at `e14de41` (local, not pushed). The full pack is plan sections A–D (`/tmp/claude-zenoh-sync/plan.md`; harness copy `CLAUDE_TERMINAL2_ZENOH_SYNC_COMMS.md`).

**Locks, written as requirements:**
- **L1:** one `xgc-rt-host` per robot, many `.so` plugins, one manifest. Multi-process is optional later only.
- **L2:** modules never touch ROS. One host-loaded `ros1-bridge` plugin converts topics to and from port events.
- **L3:** no module can block the process; one Session clock; explicit synchronization; many FSMs alive at once.

**A. Architecture:**
- **ROS / Zenoh:** ROS exists only in the bridge plugin. Zenoh is the transport under the host endpoint, bound to the radio network only; in-host hops use loopback.
- **Audit / clock:** the host stamps and audits every hop (send, receive, first read); a probe bounds each node's clock error, and the bound rides every frame.
- **DMPC round:** bridge → estimator → planner (snapshot neighbors k−1, solve, publish k over Zenoh) → controller → bridge.

**B. Base components.** Proposed new abstractions, required by L3:
- **ModuleRunner:** one thread per plugin; all of its calls run there.
- **Router:** in the transport sink; fills per-port **Mailboxes** and wakes runners.
- **Supervisor:** host FSM, lifecycles, clock probe, **Watchdog** (overrun → Degraded; hang → abandon the runner and restart; repeated → exit so the Agent restarts the host).
- **FSM stack:** host FSM + one lifecycle FSM per plugin + plugin domain FSMs. They couple only through degrade/recover requests, activate/deactivate, and port events.

**The one L3 gap in the built code:** the host runs all plugins on *one* executor thread, so a blocking module stalls the host. B3 is the fix, and it is the first slice after GO.

**C. Communications plugin family:**
- **Transport / audit:** loadable through a proposed `xgc_rt_transport_v1` (loopback and Zenoh built; shm later); audit host-side, `audit-def/1`.
- **Impairment / clock:** netem on the station, the seeded relay in the sandbox; the clock probe is built.
- **NeighborExchange:** Fresh/Stale/Missing snapshot. Rust built; C header proposed.

**D. Migration:**
- **Wrap first:** keep the ROS-free core, replace ROS I/O with ports, route topics through the bridge.
- **Done:** the hover-thrust estimator and DFBC controller, both bit-identical to the originals.
- **Next:** TRO DMPC Phase 1 keeps the node behind the bridge, with `/formation/assumed_trajectories` ↔ `dmpc/plan` and local rounds. Phase 2 wraps `IDmpcOptimizer` once it is ROS-free.

**Evidence so far (31 tests pass):**
- **Zenoh finding:** best-effort drops late frames (reordering shows as loss) and suppresses duplicates. See `docs/transport-findings.md`.
- **Audit / clock:** audit counts equal injected ground truth; the clock probe measured a 5 ms skew exactly.

**Decisions needed (defaults proposed):**
- **D2:** create `XGC-Team/xgc2-sync-runtime`. D1, D3, D5, D6: as in the plan.
- **D7:** control channels are latest-wins best-effort.
- **D8 / D9:** watchdog policy as above; a thread per plugin rather than a pool.
