# Lightweight plant hosting cost — 2026-09-28

Wall-clock measurements of lightweight FS150 plants inside one real
`xgc-rt-host`, before and after the batch path (`8121ec3`). They show what the
host costs per robot; they are not a controller, DMPC, Zenoh or 100-robot
experiment result.

## What runs

`crates/xgc-rt-host/tests/lightweight_bench.rs`, only when asked:

```bash
XGC_LIGHTWEIGHT_BENCH=1 cargo test --release -p xgc-rt-host \
    --test lightweight_bench -- --nocapture
# the pre-batch plugin, built from a3fd46f with scripts/build-lightweight-vehicle.sh:
XGC_LIGHTWEIGHT_ELF=/path/to/a3fd46f/liblightweight_vehicle.so \
XGC_LIGHTWEIGHT_BENCH_SHAPES=per-robot-round,per-robot-dirty XGC_LIGHTWEIGHT_BENCH=1 \
    cargo test --release -p xgc-rt-host --test lightweight_bench -- --nocapture
```

Each run is one host with N FS150 robots, 1 ms model steps and 10 ms outputs
(pose, velocity, imu, fcu_state, paired_state), a 3 s startup, 1 s warm-up
and a 5 s measured window. All plant instances use `step_budget_ms = 10`.

| Shape | Plant instances | Wake rule |
| --- | --- | --- |
| `per-robot-round` (today) | one per robot | every 1 ms round |
| `per-robot-dirty` | one per robot | on input, else `wake_ms = 10` (no code change) |
| `batch` | one per 8 robots | on input, else `wake_ms = 10`; host rounds stay 1 ms |
| `batch-round10` | one per 8 robots | every 10 ms round of a plant-only host (`period_ms = 10`); input never wakes it |

Loads: `idle` has no controls and no link, so it isolates hosting cost.
`commanded` adds a stand-in for the robots' controller processes on the
loopback link: it arms each robot, enters OFFBOARD, streams 50 Hz acceleration
setpoints and receives every output. Loopback delivers inline, so the
receiving side of the plant's outputs is charged to the plant threads.

Metrics come from `/proc/<pid>/task/*` for the host's own threads only
(`xgc-*`): CPU is schedstat run time in the window as a percentage of one
core; wakeups/s are voluntary context switches, which also count lock waits.
Steps and step durations come from the host's `steps.jsonl` (`t1 - t0` of
each plant step call); outputs/s are the plants' published samples.

Machine: shared 8-vCPU Intel Xeon VM, Linux 6.12, Rust 1.85 release build,
plugin `-O2`. Unrelated builds kept the load average between 9 and 28 during
all runs, so latency percentiles are noisy; CPU time and wakeup counts
repeated within a few percent (below). Raw records: `results.jsonl`
(`run1` full matrix, `run2`/`run3` repeats at 100 robots).

## Results (run1)

### Idle: hosting cost

| robots | plugin | shape | instances / host threads | CPU % of one core | wakeups/s | plant steps/s | step µs mean / p99 | outputs/s |
|---:|---|---|---:|---:|---:|---:|---:|---:|
| 1 | before | per-robot-round | 1 / 5 | 3.4 | 3313 | 765 | 1.2 / 3.2 | 498 |
| 1 | before | per-robot-dirty | 1 / 5 | 1.3 | 1207 | 92 | 3.1 / 7.2 | 463 |
| 1 | after | batch | 1 / 5 | 1.2 | 1237 | 97 | 3.3 / 6.9 | 485 |
| 1 | after | batch-round10 | 1 / 5 | 0.5 | 384 | 100 | 3.3 / 7.8 | 497 |
| 32 | before | per-robot-round | 32 / 36 | 38.1 | 45114 | 30333 | 0.7 / 2.8 | 15956 |
| 32 | before | per-robot-dirty | 32 / 36 | 7.2 | 6684 | 3118 | 2.0 / 5.2 | 15645 |
| 32 | after | batch | 4 / 8 | 3.0 | 1839 | 369 | 7.2 / 14.9 | 14825 |
| 32 | after | batch-round10 | 4 / 8 | 1.2 | 1118 | 397 | 6.6 / 14.6 | 15871 |
| 100 | before | per-robot-round | 100 / 104 | 90.2 | 119675 | 95975 | 1.0 / 2.7 | 49969 |
| 100 | before | per-robot-dirty | 100 / 104 | 18.6 | 19089 | 9790 | 1.8 / 5.5 | 48965 |
| 100 | after | batch | 13 / 17 | 4.9 | 3604 | 1269 | 7.2 / 16.8 | 48825 |
| 100 | after | batch-round10 | 13 / 17 | 3.2 | 2620 | 1300 | 8.0 / 17.2 | 50009 |

### Commanded: controls in and every output over the link

| robots | plugin | shape | instances / host threads | CPU % of one core | wakeups/s | plant steps/s | step µs mean / p99 | outputs/s |
|---:|---|---|---:|---:|---:|---:|---:|---:|
| 1 | before | per-robot-round | 1 / 5 | 3.5 | 3677 | 921 | 7.3 / 54.3 | 500 |
| 1 | before | per-robot-dirty | 1 / 5 | 1.4 | 1835 | 102 | 38.1 / 87.4 | 499 |
| 1 | after | batch | 1 / 5 | 1.6 | 1921 | 105 | 43.4 / 120.6 | 498 |
| 1 | after | batch-round10 | 1 / 5 | 0.8 | 897 | 99 | 96.8 / 2095.2 | 497 |
| 32 | before | per-robot-round | 32 / 36 | 52.4 | 61418 | 28359 | 59.9 / 1240.0 | 15941 |
| 32 | before | per-robot-dirty | 32 / 36 | 16.6 | 19440 | 3611 | 247.2 / 4777.5 | 15990 |
| 32 | after | batch | 4 / 8 | 8.9 | 8143 | 461 | 181.4 / 1913.9 | 15822 |
| 32 | after | batch-round10 | 4 / 8 | 8.5 | 11070 | 397 | 372.6 / 5209.8 | 15705 |
| 100 | before | per-robot-round | 100 / 104 | 141.3 | 155666 | 78206 | 175.0 / 3989.9 | 49399 |
| 100 | before | per-robot-dirty | 100 / 104 | 42.7 | 50625 | 11132 | 668.1 / 11107.2 | 49538 |
| 100 | after | batch | 13 / 17 | 25.2 | 24660 | 1630 | 494.0 / 4789.2 | 49877 |
| 100 | after | batch-round10 | 13 / 17 | 25.5 | 20308 | 1298 | 866.3 / 5336.9 | 49978 |

The new plugin in the old per-robot shapes costs the same as the old plugin
(`run1-after` records, e.g. 92.6 % / 120097 wakeups/s idle at 100 robots), so
the gain comes from the batch and the wake rule, not from a cheaper model.

### Repeats at 100 robots

Commanded, three interleaved runs (`run3`): per-robot-round 134.2 / 131.8 /
133.6 % CPU and 143691 / 138703 / 143413 wakeups/s; batch 20.2 / 18.7 /
17.0 % and 23165 / 22620 / 22745; batch-round10 21.3 / 25.1 / 21.6 % and
23150 / 18262 / 20428. Idle (`run2`): 84.6 %, 5.4 % and 4.0 %.

Where the host's time goes at 100 robots (`run2`, CPU % / wakeups/s):

| shape, load | plant threads | main thread | step/health logs | audit writer |
| --- | ---: | ---: | ---: | ---: |
| per-robot-round, idle | 71.4 / 103089 | 1.1 / 971 | 12.2 / 11613 | 0 / 0 |
| batch, idle | 3.6 / 1299 | 1.0 / 997 | 0.8 / 1062 | 0 / 0 |
| batch-round10, idle | 3.4 / 1437 | 0.2 / 100 | 0.5 / 611 | 0 / 0 |
| per-robot-round, commanded | 130.1 / 120959 | 3.7 / 1451 | 16.5 / 13932 | 6.4 / 14515 |
| batch, commanded | 33.5 / 21836 | 2.8 / 2413 | 0.9 / 1289 | 6.1 / 18369 |

## Reading

- At 100 robots the batch cuts host CPU from 90 % to 3–5 % of a core idle and
  from about 133 % to 19–22 % commanded (median of three), wakeups from about
  120 k/s to 2.6–3.6 k/s idle and from about 143 k/s to 20–23 k/s commanded,
  and plant steps from 96 k/s to 1.3 k/s. Outputs stay at the configured
  50 k/s: the batch changes scheduling, not what is published.
- Changing only the wake rule (`per-robot-dirty`) already gives 4–5× idle, but
  keeps 100 threads and 19 k wakeups/s.
- With `on_dirty`/`wake_ms`, a plant that gets no input wakes on a timer
  relative to its last step, and 2–7 % of output periods are skipped
  (outputs/s below 500 per robot in the idle `per-robot-dirty` and `batch`
  rows). The plant-only host with
  `period_ms` equal to the output period wakes on the output grid, publishes
  every period and reduces the main thread to 100 wakeups/s.
- Commanded runs are dominated by the link path, not the model: each 8-robot
  step publishes 40 frames over the link, the plant threads' remaining
  wakeups are mostly lock waits in that path, and the audit writer wakes about
  18 k/s. Loopback is not Zenoh; this cost must be measured again on the
  production transport.
- With the default step budget of one 1 ms round, a step that publishes over
  a busy link for more than 10 ms is abandoned as hung: the per-robot shape
  lost plant instances at 32 commanded robots on this machine (an earlier
  run, before the bench set `step_budget_ms = 10`). A plant host needs a
  budget of at least its output period.

## Not shown

No controller, estimator, DMPC or ROS edge ran; the stand-in feeder does not
close a control loop. No Zenoh, container or onboard placement. Pure model
cost is not separated from publishing here. Equivalence of batch and
independent execution is a test, not a benchmark: `plugins/lightweight-vehicle/abi_test.cpp`
(byte-identical outputs, three models) and
`crates/xgc-rt-host/tests/lightweight_vehicle.rs` (real hosts, shared clock).
