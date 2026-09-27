# NeighborExchange

The DMPC-facing API over two ports: the planner publishes its plan for round
k, and at round k it solves with each neighbor's plan produced for round
k − 1 (the assumed trajectories of the previous round). It never blocks. The
planner decides at its own deadline with whatever has arrived, and a
snapshot says exactly what that was.

There are two implementations of one contract:

| Language | Where | Used by |
|---|---|---|
| Rust | `crates/xgc-rt-abi/src/neighbor.rs` (`NeighborExchange`) | `dmpc-rounds`, `dmpc-exchange-demo` |
| C / C++ (header-only) | `abi/include/xgc_rt_nx.h` (`xgc_nx`) | `plan-dmpc` |

`tests/neighbor_exchange_c.rs` runs one seeded script of offers and
snapshots through both, with the header compiled as C11 and as C++17. The
script covers neighbors and strangers; past, current and future rounds; and
older, duplicate and newer sequence numbers. Every admission, status, stale
count, round, age, plan byte and encoded record must match.

## Contract

| Call (Rust / C) | Meaning |
|---|---|
| `NeighborExchange::new(host, plan_in, plan_out, s_max)` / `xgc_nx_open(&nx, host, plan_in, plan_out, s_max)` | Neighbors are `plan_in`'s bound origins (the manifest `from`; host API `port_origins`, `abi_minor >= 1`). `with_neighbors` / `xgc_nx_init` take them explicitly. |
| `offer(planner_k, origin, round, seq, t_produce, data)` / `xgc_nx_offer` | Admits one received plan at planner round `planner_k`. It refuses a non-neighbor or a future round (`round > planner_k`), and a refused plan is not cached. The newest admitted `(round, seq)` per neighbor is kept. An older or duplicate plan does not replace it. |
| `absorb(host, planner_k)` / `xgc_nx_absorb` | Drains `plan_in`, offering every sample. |
| `snapshot(k, now)` / `xgc_nx_view_of(&nx, i, k, now, &view)` | Each neighbor's status at round k, from its newest admitted plan. |
| `publish(host, k, bytes)` / `xgc_nx_publish` | Publishes this planner's plan on `plan_out` for round k. |

Status of neighbor j in the snapshot of round k:

- **Fresh**: the newest plan was produced for round k − 1 ≤ round ≤ k.
- **Stale(n)**: it was produced for round k − 1 − n, with 1 ≤ n ≤ `s_max`.
- **Missing**: nothing was admitted, the plan is older than the stale window, or the cached round is in the future relative to this snapshot. A future round is never Fresh.

Each view also carries the plan's round (none if never admitted), its age
`now − t_produce` and its bytes (none when Missing). The host audits each
plan's delivery separately (loss, OWD, age at use; see
[audit-definitions.md](audit-definitions.md)).

## Snapshot record (`xgc.dmpc.neighbor_snapshot/1`)

`xgc_nx_snapshot_encode` writes one record per round. All fields are
little-endian, and it is 16 + 32 · count bytes long:

| Offset | Field | Type |
|---|---|---|
| 0 | round k | u64 |
| 8 | count | u32 |
| 12 | reserved | u32 |
| 16 + 32 i | origin (roster id) | u16 |
| +2 | status: 0 Fresh, 1 Stale, 2 Missing | u8 |
| +3 | reserved | u8 + u32 |
| +8 | stale rounds n (0 unless Stale) | u64 |
| +16 | round of the newest admitted plan (`u64::MAX` if none) | u64 |
| +24 | age ns (0 if none) | i64 |

## plan-dmpc

plan-dmpc keeps the round rule of the ROS node it replaces. At a tick of
round k, it gives the agent every buffered neighbor plan of round ≤ k − 1, in
round order. A plan of round k is kept until round k + 1, even when it
arrived before tick k. Each plan given to the agent at round k is offered to
an `xgc_nx` at planner round k. After the round's outputs, and before
`round_done`, the module publishes the snapshot of round k on its optional
`neighbors` port (event QoS). `stale_rounds` (default 3) sets `s_max`. It
changes only the report, never what the agent receives.

The snapshot therefore reports what the round used, not what had arrived.
A peer's round-k plan that arrived before tick k is not in the snapshot of
round k, because the agent was not given it.

## Evidence on the fleet path

`tests/closed_loop_fleet.rs` runs the knot_fs150 and mixed_circle fleets over
the Zenoh transport plugin, with relays between robots (clean and C4
profiles). The test reconstructs, from each robot host's `steps.jsonl`, the
peer plans the module had read by each tick. It then requires that
plan-dmpc's own snapshot match that record exactly (status, stale count,
plan round) for every robot, round and peer. The same run replays every
robot's rounds byte for byte, with early peer plans held back, and joins the
plan audit with the relay truth.

One run's snapshot counts (Fresh / Stale / Missing):

| Run | Fresh / Stale / Missing |
|---|---|
| knot clean | 1576 / 0 / 4 |
| knot C4 | 1533 / 35 / 12 |
| mixed_circle C4 | 5540 / 116 / 32 |

Missing includes rounds where the peer published no plan for k − 1 and its
previous plan was already outside the stale window.
