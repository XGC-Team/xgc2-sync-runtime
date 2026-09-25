# Time model

There is one Session timebase, rounds are derived locally from it, and the clock error is measured, not assumed.

- **`Clock`:** the only time API (`xgc-rt-core::clock`). It has two domains:
  - **`Wall`** (physical and hybrid runs): `CLOCK_REALTIME`, disciplined by chrony to the station, or by PTP where hardware timestamping exists. It uses the runtime-sync gate: offset ≤ 2 ms, uncertainty ≤ 2 ms, source = station, and it may slew but never step during a run.
  - **`Sim`**: one simulator authority. Stamps from it have a bound of 0.
- **Local loop durations:** use a monotonic clock, never Session time.
- **Rounds:**
  - Round `k` starts at `E0 + k·P` on every node (runtime-sync ADR 0003), with no network tick.
  - Missed rounds are skipped.
  - The publish deadline is `start(k) + publish_deadline`.
  - In Z1, `E0` = host start + `start_delay_ms`, or `session.epoch_ns`. From Z3 the Session sets it.
- **Bound:**
  - Every frame carries the sender's current bound `u` on |its clock − Session reference|, and the receiver stores its own.
  - In Z1 every node shares one process clock, so `u = 0`.
  - From Z2, `u` comes from chrony's estimate and an in-band NTP-style probe over the same radio path. The probe gives offset `((t2−t1)+(t3−t4))/2` and bound `(RTT − (t3−t2))/2`.
