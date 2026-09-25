# Audit definitions: `audit-def/1`

These are the definitions that latency, loss, reordering and throughput claims cite, including TRO DMPC claims.
- They are implemented once, in `crates/xgc-rt-audit/src/merge.rs`.
- Every artifact carries the version string `audit-def/1`.
- Changing a definition means a new version. A definition never changes silently.

## Where the stamps come from

The host takes every stamp, in `crates/xgc-rt-host/src/endpoint.rs`. Modules and transports take none, so every transport is measured identically.

| Stamp | Clock | Taken |
|---|---|---|
| `t_produce` | sender Session clock | when the module calls `publish` |
| `t_tx` | sender Session clock | after the header is filled, just before encode and transport send |
| `t_rx` | receiver Session clock | when the transport hands the frame to the host sink, before verification and queueing |
| `t_consume` | receiver Session clock | when a module first pops the sample from its in-port |

Every frame carries the sender's clock-error bound `u_s`, and each receive record also stores the receiver's bound `u_r` (see `time-model.md`).

## Records

Each node appends fixed 56-byte records to `<run>/<node>/records.bin` (`xgc-rt-audit-records/1`, layout in `record.rs`). The record kinds:

| Kind | Written when |
|---|---|
| subscribe | the node subscribes to a channel from an origin |
| tx | a frame is handed to the transport |
| rx | a frame arrives and passes verification |
| consume | a module first reads a sample |
| reject | a frame fails decode or its CRC |
| overflow | a bounded queue overflows |

- The writer is bounded and never blocks the data path.
- A record it cannot queue is counted in `meta.json` (`audit_queue_drops`), and that makes the run **invalid**.
- A frame dropped because the host receive queue was full gets **no** rx record, so it counts as lost. It also gets an overflow record, which makes the run invalid.

## Stream and expected set

- A **stream** is `(channel, origin → receiver)`. It exists from the receiver's subscribe record at `t_sub` onward.
- Its **expected set** `S` is every seq the origin logged as sent on that channel with `t_tx ≥ t_sub`.

## Per-arrival classification

Arrivals are processed in record (arrival) order, and each rx record falls into exactly one class:

1. **phantom:** the seq was never logged as sent. This is an integrity error, and the run is invalid.
2. **duplicate:** the seq was already seen on this stream.
3. **late beyond grace:** a first arrival with `t_rx − t_tx > grace` (default 1 s). It counts as lost.
4. **received:** otherwise. A received sample is **reordered** when its seq is below the highest seq already received on the stream (RFC 4737 §3.3). Its *reorder extent* is `max_seen − seq`.

## Metrics

| Metric | Definition |
|---|---|
| loss | `lost = |S \ R|` and `loss_ratio = lost / |S|`, where `R` is the set of received (class 4) seqs. The set difference is exact, computed from joined sender and receiver logs. |
| duplicates | count of class-2 arrivals |
| reordering | count of reordered arrivals; `reorder_ratio = reordered / received`; extent distribution |
| one-way delay | `t_rx − t_tx` over received samples, as min, p50, p90, p99, max and mean. Each sample's error bound is `u_s + u_r`, reported as the `owd_bound_ns` distribution. An OWD figure is only as precise as its bound. |
| age at use | `t_consume − t_produce` over received samples a module read. For DMPC, this is how old a neighbor's plan was when the solver used it. |
| duration | `(t_last − t_first) · n / (n − 1)` over the expected set's `t_tx`, so that n samples at period P give exactly 1/P |
| rate | `received / duration` |
| offered throughput | payload bytes of `S` × 8 / duration |
| delivered throughput | payload bytes of `R` × 8 / duration. The envelope variant adds 64 B per sample. Transport and IP overhead are not included; interface counters come from Z2. |

- **Percentiles** are nearest-rank.
- **Windows** are `floor((t_tx − t0) / window)`, where `t0` is the run's first expected `t_tx`. Each window line in `streams.jsonl` carries the same counts computed over its window.

## Validity

A report is `valid: false` when any of these holds, and `invalid_reasons` lists them:
- any node's audit is incomplete;
- any node dropped audit records;
- a record count does not match `meta.json`;
- a node reused a seq;
- phantom samples were received;
- a receive queue overflowed.

Numbers from an invalid run are not claimable.

## Calibration (Z1)

`crates/xgc-rt-host/tests/audit_exact.rs` runs 100 000 samples from 1 sender to 3 receivers on the loopback transport, with a seeded injector set to 5 % drop, 2 % duplicate and 3 % reorder. It requires the merged `lost`, `duplicates` and `reordered` to **equal** the injector's ground truth on every stream. Z2 repeats this over Zenoh through a userspace impairment relay.
