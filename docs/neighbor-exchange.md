# NeighborExchange

The DMPC-facing API over two ports: a planner publishes its plan for round
k, and at round k it solves with each neighbor's plan produced for round
k − 1 (the assumed trajectories of the previous round). It never blocks. The
planner decides at its own deadline with whatever has arrived, and a
snapshot says exactly what that was.

There are two implementations of one contract:

| Language | Where | Used by |
|---|---|---|
| Rust | `crates/xgc-rt-abi/src/neighbor.rs` (`NeighborExchange`, `Snapshot::encode`) | `dmpc-rounds`, `dmpc-exchange-demo` |
| C / C++ (header-only) | `abi/include/xgc_rt_nx.h` (`xgc_nx`) | C and C++ planner modules |

`tests/neighbor_exchange_c.rs` runs one seeded script of offers and
snapshots through both, with the header compiled as C11 and as C++17. The
script covers neighbors and strangers; past, current and future rounds; and
older, duplicate and newer sequence numbers. Every admission, status, stale
count, round, age and plan byte must match, and the C encoder must write the
same bytes as `Snapshot::encode`.

## Contract

| Call (Rust / C) | Meaning |
|---|---|
| `NeighborExchange::new(host, plan_in, plan_out, s_max)` / `xgc_nx_open(&nx, host, plan_in, plan_out, s_max)` | Neighbors are `plan_in`'s bound origins (the manifest `from`; host API `port_origins`, `abi_minor >= 1`). `with_neighbors` / `xgc_nx_init` take them explicitly. |
| `offer(planner_k, origin, round, seq, t_produce, data)` / `xgc_nx_offer` | Admits one received plan at planner round `planner_k`. It refuses a non-neighbor or a future round (`round > planner_k`), and a refused plan is not cached. The newest admitted `(round, seq)` per neighbor is kept. An older or duplicate plan does not replace it. |
| `absorb(host, planner_k)` / `xgc_nx_absorb` | Drains `plan_in`, offering every sample. |
| `snapshot(k, now)` / `xgc_nx_view_of(&nx, i, k, now, &view)` | Each neighbor's status at round k, from its newest admitted plan. |
| `Snapshot::encode()` / `xgc_nx_snapshot_encode` | The snapshot as a wire record (below). |
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

One record per round. All fields are little-endian, and it is
16 + 32 · count bytes long:

| Offset | Field | Type |
|---|---|---|
| 0 | round k | u64 |
| 8 | count | u32 |
| 12 | reserved | u32 |
| 16 + 32 i | origin (roster id) | u16 |
| +2 | status: 0 Fresh, 1 Stale, 2 Missing | u8 |
| +3 | reserved | 5 bytes |
| +8 | stale rounds n (0 unless Stale) | u64 |
| +16 | round of the newest admitted plan (`u64::MAX` if none) | u64 |
| +24 | age ns (0 if none) | i64 |

## In the DMPC fleet

dmpc-rounds owns peer admission. plan-dmpc keeps no second neighbor cache: it
gets exactly the plans that dmpc-rounds forwards on `neighbor_plans`.

At planner beat k, dmpc-rounds:

1. forwards each neighbor's plans of source rounds < k, and offers them to its
   `NeighborExchange` at round k;
2. holds rounds ≥ k for a later beat;
3. publishes the snapshot of beat k on its optional `neighbors` out port
   (event QoS). `stale_rounds` (default 2) sets `s_max`.

The snapshot therefore says which plans the planner's round could use. A
peer's round-k plan that arrived before beat k is not in it.

`tests/closed_loop_fleet.rs` runs the knot_fs150 and mixed_circle fleets over
the Zenoh transport plugin, with relays between robots (clean and C4
profiles). For each robot, beat and peer, the test reconstructs from the
steps log the newest source round the planner was given by tick k. It then
requires dmpc-rounds' own snapshot to say exactly that: status, stale count
and plan round.

One sandbox run each (Fresh / Stale / Missing, robot × beat × peer):

| Run | Fresh / Stale / Missing |
|---|---|
| knot clean | 1536 / 0 / 44 |
| knot C4 | 1268 / 115 / 197 |

Missing includes beats where a peer had published no plan for k − 1 and its
previous plan was outside the stale window, such as rounds before a peer's
planner started producing plans.
