# Z3 Session artifact contract (draft)

What a Z3 station run records, and where. The contract has two parts:

1. what the Session fixes before the run: `(E0, P)`, the roster, the link
   profile and the pinned binaries;
2. the `Shared/SyncAudit/<run>/` directory that the robots' hosts and the
   merge fill.

Every per-node file below is written today by `xgc-rt-host` and
`xgc-rt-audit`. They are the same files the Z2 calibrations, the C4 matrix
and the Block I fleet runs produce. The only new files are `session.json`
and `link/`, which the Session writes. Placing and exporting them is the one
Core touch (plan D5).

## 1. What the Session fixes

`session.json`, written once by the Session before any host starts, is
read-only afterwards:

```json
{
  "schema_version": 1,
  "session_id": "tro-z3-...",
  "run_id": "20261015T101500Z-knot-c",
  "clock_domain": "wall",
  "epoch_ns": 1791000000000000000,
  "period_ns": 100000000,
  "start_margin_ns": 5000000000,
  "roster": [
    {"node": "uav1", "node_id": 0, "kind": "fs150", "radio": "udp/172.30.251.101:7447"},
    {"node": "uav2", "node_id": 1, "kind": "fs150", "radio": "udp/172.30.251.102:7447"}
  ],
  "channels": [{"name": "dmpc/plan", "qos": "control"}],
  "link_profile": {"name": "C", "source": "GET /api/link-profiles", "frozen": {"delay_ms": 40, "loss": "gilbert_elliott(...)", "duplicate": 0.02, "reorder": 0.02}},
  "bundle": {"source_pins_sha256": "<SOURCE-PINS.json of the robot image>", "sync_runtime_revision": "<40 hex>", "academic_revision": "<40 hex>"},
  "manifests": {"uav1": "<sha256 of uav1's rendered node.toml>"}
}
```

| Field | Rule | Where it goes on a robot |
|---|---|---|
| `epoch_ns` (E0) | Session time, at least `start_margin_ns` after the slowest host's startup. Never derived per node. | manifest `[session] epoch_ns` |
| `period_ns` (P) | The round period. Round k starts at E0 + k·P on every node. | `[session] period_ms` |
| `clock_domain` | `wall` (chrony/PTP, probe-bounded) or `sim` (one simulator authority) | host clock / `[session.clock_source]` |
| `roster` | The node order defines the envelope's `origin` ids (roster index), identical on every node. | `[session] roster`; Zenoh `listen` = own `radio`, `connect` = peers' `radio` |
| `channels` | Names and QoS. The order defines channel ids. | `[[channel]]` |
| `link_profile` | Frozen when the Session starts. It applies to the radio network only (`network-station.sh` netem). | not in the manifest; recorded here and in `link/` |
| `bundle`, `manifests` | The image's pins and each robot's rendered manifest hash | `[[plugin]] sha256`, `[transport] sha256` |

A host started without a shared `epoch_ns` logs `epoch_local_only` in
`health.jsonl`. A Z3 run with that event on any node is invalid.

## 2. `Shared/SyncAudit/<run>/`

```
Shared/SyncAudit/<run>/
  session.json                 the Session's fix (above)
  link/profile.json            the frozen profile as applied (netem qdisc per radio interface)
  link/netem-<iface>.txt       `tc -s qdisc show` before and after the run (drops the kernel counted)
  <node>/                      one per roster node, written by that node's xgc-rt-host
    meta.json                  NodeMeta: format "xgc-rt-audit-records/1", session, node, node_id,
                               roster, channels, clock_domain, audit_queue_drops,
                               records_written, complete
    records.bin                56-byte records: Subscribe, Tx, Rx, Consume, Reject, Overflow
                               (origin, channel, seq, t_a, t_b, …; crates/xgc-rt-audit/src/record.rs)
    steps.jsonl                per module step: {"m", "k", "t0", "t1", "in": [[port, origin, seq], …]}
    health.jsonl               lifecycle, epoch, startup timings, clock-source and log events
    clock_source.json          present when the manifest has a clock source
    node.toml                  the rendered manifest this host ran (its sha256 = session.json manifests)
  merged/                      `xgc-rt-audit merge <run>` after every host finished
    summary.json               Session totals and per-stream statistics, `audit-def/1`
    streams.jsonl              one line per (channel, origin → receiver)
    summary.md                 the readable matrix
  truth/                       sandbox runs only: relay ground truth per link, one
    <i>-<j>.jsonl              TruthRecord per line (origin, channel, seq, action,
                               t_in, delays_ns, released_ns)
```

- **Layout.** `<node>/` sits directly under the run, because that is the
  layout `merge_run` reads. A host's `[audit] dir` is the run directory. The
  host creates `<node>/` itself (`FileAudit::create(run_dir, meta)`), exactly
  as in every Z2, C4 and Block I test.
- **Completeness.** A node's audit is complete when `meta.json` says
  `complete: true` with `audit_queue_drops: 0` and `records.bin` holds
  `records_written` records. The merge marks the run invalid otherwise, and
  refuses nodes from another session, roster, channel set or clock domain.
  The merge does not yet flag a roster node with no directory at all
  (tests use observer roster entries that do not audit). A Z3 run check
  must therefore also require one `<node>/` per robot in `session.json`.
- **Per-link matrix.** The evidence target (DMPC plans robot ↔ robot under
  profiles B and C) is `merged/streams.jsonl` filtered to `dmpc/plan`: an
  n·(n−1) matrix with each stream's loss, duplicates, reordering, OWD with
  its clock bound, and age. For a run that must show round selection, each
  planner's `neighbors` records (plan-dmpc's NeighborExchange snapshots,
  [neighbor-exchange.md](neighbor-exchange.md)) sit on a `dmpc/neighbors`
  channel in the same audit. `steps.jsonl` records which plan each round
  read.
- **Ground truth.** Netem has no per-sample truth. On the station, a link's
  loss is the audit's own count, set against the frozen profile and the
  qdisc counters in `link/`. The sample-by-sample join with ground truth
  (C4, Block I) exists only for sandbox relays, and those runs write it
  under `truth/`.
- **Managed root.** A host launched through `xgc-rt-render run` today writes
  under `<managed-root>/sync-runtime/<session_id>/<node_id>/generations/<nonce>/`.
  For Z3, the generation's audit directory is either the Session's
  `<run>/<node>/` or copied there when the run closes. That choice belongs
  to the Core slice.

## 3. Alignment with what exists

| Stamp or file | Written by | Already produced in |
|---|---|---|
| envelope `origin`, `channel`, `seq` (from 1), `round`, `t_produce`, `t_tx` | host endpoint | Z1 onward |
| `records.bin` Tx/Rx with `t_tx`, `t_rx` | host endpoint | Z1, Z2b/c, C4, Block I |
| `steps.jsonl` reads per step | host | G, H, Block I (replay and round selection) |
| `health.jsonl` `epoch` / `epoch_local_only` | host | Z2d onward |
| `merged/` | `xgc-rt-audit merge` / `merge_run` + `write_report` | C4, Block I |
| relay `TruthRecord` incl. `released_ns` | xgc-rt-impair | Z2b, C4, Block I (in memory; `truth/` makes it a file) |

## 4. Core touch (blocked on coordination)

- The Session must write `session.json`: `(E0, P)` with margin, the roster
  with radio addresses, the frozen link profile, and the pins.
- It must render each robot's manifest (or deployment JSON) from that file.
- It must provide `Shared/SyncAudit/<run>/` to every robot container and run
  the merge when the run closes.

These are Core Session files (plan D5). They are changed in one Core slice
coordinated with the other terminals, not from this repository. Until that
slice lands, a station run can be driven with hand-rendered manifests that
follow this contract.
