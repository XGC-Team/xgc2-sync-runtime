# Time model

- Session time is `int64` nanoseconds from the `Clock` in `xgc-rt-core::clock`.
- Round k starts at `E0 + k·P` on every node: `E0` is `session.epoch_ns` (or the host start plus `start_delay_ms`), `P` is `session.period_ms`. The host records `epoch_local_only` when a linked session has no `epoch_ns`.
- `[clock]` in the manifest runs the in-host clock probe (`crates/xgc-rt-host/src/clock_service.rs`); it measures the offset bound and never adjusts time.
- A plugin's `wake_ms` steps it again within a round (a local timer).

Decisions, rationale and plans for this runtime live in the academic knowledge base: lxk36/academic, `docs/architecture/xgc2-sync-runtime/time-model.md` (the timing contract).
