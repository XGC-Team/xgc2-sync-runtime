# Native C++ validation — 2026-09-26

This is local functional and replay evidence for sync-runtime base
`e3a3196938f07b38e6da5365d788a4aaa16f32dc`, plus the test/documentation changes
in this commit. No production algorithm, host scheduling policy or default
control configuration changed. This is not PX4 SITL/HIL, physical flight, a
formal experiment or Agent/Core deployment acceptance.

## Dependencies and isolation

The immutable runtime image was
`sha256:e2fbd86c7d3c75c8aadf64eee3e946300ecf7e9a2fc77f61fcac3cdb0e5b2529`
(`xgc2-core-runtime:ubuntu20-noetic-amd64-v1` at the time). Its ROS Noetic,
acados and C++ dependencies were real installed packages. Rust/Cargo were
1.93.0. Source identities are in `source-identities.json`. In particular,
multirotor source was freshly archived from
`40bc1084677901f2f5cbf15eb1f64b227a9cd37a`; the image's older controller
binary was not used as the core under test. UGV `dbcd549894e1e56d9c3942a451b42b1179c66461`
is a separately validated ROS-free extraction and is not a module in this graph.

A private catkin workspace contained that source at `src/multirotor`:

```bash
catkin_make -j4 -l4 -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DPX4_CONTROLLER_REPLAY_HARNESS=ON \
  -DREFERENCE_TRAJECTORY_REPLAY_HARNESS=ON
```

The build regenerated and linked the actual acados solver, both ROS-free
cores, the original ROS nodes and replay harnesses. Native plugins compiled
against the current source trees, not stand-in implementations. `readelf -d`
results in `artifact-identities.json` show no ROS dependency in the five
C++ domain plugins or controller/reference cores. `libros_io.so` correctly
links ROS. Artifact hashes identify this local build, not a reproducible
compiler-output guarantee.

Each Docker invocation used `--network none`, no published ports, and
`--label com.docker.compose.project=pr-validation
--label com.docker.compose.service=validation`. Source mounts were read-only;
only the isolated checkout/build was writable. Tests created private ROS
masters on 11411/11412/11413 in the container and never contacted the live
11311 station. Historical wrapper scripts are included with the evidence.

## Executed gates

All following gates used configured real dependencies and logs were scanned
for both `skipped:` and `skipped ` early-return notices. A generic green
`cargo test` run without this check would not establish native execution.

| Gate | Cases | Observed result |
|---|---:|---|
| `est_rigid_state`, `est_hover_thrust`, `ctl_dfbc` | 3 | Wrapped runtime/kernel outputs exactly match independently executed upstream C++ references. HTE: 3,400 inputs, 820 estimates, final 0.4196 against 0.42; ESKF: 2,370 inputs, 801 states, 224 vision poses, final position error 0.0012 m. |
| `ros_io`, `ref_trajectory_ros_io`, `px4_ros_io` | 6 | Real ROS sensor/estimator I/O and vendored message identity; original reference node parity over seven requests and 90 polynomial coefficients; original controller node completes PX4 local, DFBC and NMPC software-plant flights. |
| `ref_trajectory_replay` | 1 | 27 input records; 1,643,910 output bytes exactly equal the original reference core. |
| `ctl_px4_replay` | 3 | PX4 local: 17,763 input records / 2,809,547 output bytes; DFBC: 29,597 / 4,084,510; NMPC: 29,694 / 4,094,587. Every output byte equals the original controller-core harness. |
| `ctl_px4_ros_io` | 5 | Three existing tracking modes plus two real-hover-estimator compositions. See final composition summary below. |

The controller replay streams were recorded during the original-node
software-plant flights above, converted using that source revision's
`bag_to_stream.py`, then replayed through the current core harness and native
wrapper. They are not hardware recordings. The scripted reference stream
covers analytic, sampled and polynomial requests. End-of-stream states differ
by recording (PX4 local Ready, DFBC SelfCheck, NMPC Landing); the byte-equality
claim is for the captured traces, not an assertion that each replay ends in
Ready. Successful replay output is compared in memory; the archived expected
trace is also the byte-identical actual output, not a separately written
second trace file.

The baseline three-mode fixture uses a synthetic hover estimate. The two new
native-hover cases instead load `ros-io`, `est-rigid-state`,
`est-hover-thrust`, `ctl-px4` and `ref-trajectory` in one host. The stand-in
provides simulated IMU/pose/actuator feedback but publishes **zero** hover
estimates; the real estimator is the channel's sole producer. A passive
observer records its 64-byte outputs. Flight assertions also require sustained
Custom1, more than 500 attitude-rate commands, reference progress, airborne
altitude, landing/disarming, service calls and bounded ESKF error.

## Final composition result

All five cases passed in 317.82 seconds, with no environment skips. Together
with the preceding gates this is **18 executed cases**.

| Case | Attitude-rate commands | ESKF median error (m) | Final altitude / armed | Synthetic hover estimates |
|---|---:|---:|---|---:|
| `dfbc` | 1147 | 0.0002 | 0.0 / False | 2862 |
| `dfbc-native-hover` | 1148 | 0.0002 | 0.0 / False | 0 |
| `nmpc` | 1214 | 0.0001 | 0.0 / False | 2875 |
| `nmpc-native-hover` | 1214 | 0.0001 | 0.0 / False | 0 |
| `px4_local` | 0 | 0.0 | 0.0 / False | 0 |

The real HTE produced 5,575 outputs / 3,006 Airborne outputs / 314 distinct
raw-update stamps with DFBC, and 5,576 / 2,994 / 313 with NMPC. Their final
100 Airborne estimates averaged 0.499967 and 0.499966 respectively. All
module summaries had no final error. These measurements demonstrate this
software-plant composition, with the explicit functional watchdog budget.

## Failures retained and test corrections

1. The original ROS controller test assumed the catkin package was directly
   under `src/`. The actual workspace has `src/multirotor/`. The test now honors
   `PX4_CONTROLLER_ROOT` and retains the original layout as fallback. The
   initial failure preceded actual flight execution.
2. The first composition attempt passed two cases and failed three. The native
   HTE cases completed flight, but an added assertion incorrectly required
   more than 100 published messages with `sample_used=1`. Upstream
   `AirborneState` runs raw-update and publication gates independently:
   `sample_used` describes only the publication tick. It can be false after
   successful updates between publications. The corrected test counts distinct
   `last_estimate_stamp` values (>100), sustained Airborne publications (>1000),
   and convergence of the final 100 Airborne estimates to the known 0.5 plant
   value (tolerance 0.05). The algorithm and its gate rates were not changed.
3. In that first attempt the original NMPC composition was abandoned at
   `busy_ms=10.068607`, with the default 1 ms budget / 10 ms hang threshold.
   This is a real limitation of the tested scheduling environment. The
   functional fixture now explicitly sets the controller budget to 10 ms,
   retaining a 100 ms hang threshold and 1 ms session period. Production
   defaults and watchdog behavior are unchanged. This validation establishes
   functional outputs at that test budget, not a 1 ms deadline guarantee.
4. The source controller emitted a landing-timeout warning after 20 seconds
   on several software-plant cases, then confirmed landing/disarm within the
   fixture's separate wait. The warning is retained. No force-disarm shortcut
   or production landing-policy change was introduced.

`first-attempt.tar.xz` retains the original failed composition log, manifests,
health records and observed outputs (scheduler step-by-step logs omitted to
limit repository size). The larger local first-attempt archive retains those
step records too. These failures are not silently reclassified as passes.

## Reproduction and evidence

Source ROS and the fresh catkin workspace before invoking
`scripts/check-native-gates.sh` from the repository root. Set `ROS_PREFIX`,
`XGC2_WS`, `PX4_CONTROLLER_ROOT`, `REF_ROOT`, `PX4_CORE_LIB_DIR`,
`REF_CORE_LIB_DIR`, `ACADOS_ROOT` and `LD_LIBRARY_PATH` to that build. The C++
helper scripts also document `HTE_ROOT`, `ESKF_ROOT`, `XGC2_MATH_INCLUDE`,
`XGC2_SM_ROOT` and `XGC2_PREFIX` overrides when the standard source layout is
not present. The vendored message test needs original academic message sources
at the layout it documents. Missing originals are rejected by the gate's
skip scan.

```bash
scripts/check-native-gates.sh /absolute/path/to/native-evidence
```

The runner executes tests serially because plugin-build helpers share output
paths. It fails on missing prerequisites, failed cargo commands, any reported
environment skip or an unexpected case count. Its constituent commands were
executed in this review; the consolidated wrapper was syntax-checked without
repeating all flights. `evidence.tar.xz` contains the recorded replay inputs,
expected byte traces, logs, manifests, actual hover outputs and plant summaries;
`SHA256SUMS` and `completed-gate-summary.json` identify the captured evidence.
No build products, dependency binaries or cargo caches are committed.

For deployment boundaries, read [migration status](../../native-migration-status.md)
and [clock configuration](../../time-model.md). The checked CLI does consume
manifest `[clock]`, but absent configuration leaves an initial zero bound;
probe gate timeout marks frames degraded and continues. This one-host run
provides no radio or board clock-bound evidence. Agent/Core managed-process
packaging, startup, health and restart acceptance must be recorded separately.
