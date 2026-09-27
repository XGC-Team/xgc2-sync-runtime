# Time model

- Session time is `int64` nanoseconds from the `Clock` in `xgc-rt-core::clock`.
- Round k starts at `E0 + k·P` on every node: `E0` is `session.epoch_ns` (or the host start plus `start_delay_ms`), `P` is `session.period_ms`. The host records `epoch_local_only` when a linked session has no `epoch_ns`.
- `[clock]` in the manifest runs the in-host clock probe (`crates/xgc-rt-host/src/clock_service.rs`); it measures the offset bound and never adjusts time.
- A plugin's `wake_ms` steps it again within a round on wall time. A production simulator source additionally requires a new accepted simulator timestamp for every step.

Without `[clock_source]`, the executable creates `WallClock::new(0)` for both transports.
An explicit simulator source uses the same host with native ROS clock polling; see
[Simulator clock source](clock-source.md). Its source, world identity and shared
epoch are frozen per experiment, for either colocated or per-robot placement.
`Host::new` installs the probe service when the manifest includes `[clock]`;
this is an executable configuration path, not only a test fixture. For example,
the client configuration is:

```toml
[clock]
role = "client"
server = "station" # must be a name in the session roster
interval_ms = 1000
gate_ms = 2.0
gate_timeout_ms = 5000
window = 8
```

The station uses `role = "server"` and the same server name. The service adds
the reserved request/reply channels. A client waits for at least three samples
whose estimated bound is within `gate_ms`; startup probes run every 100 ms.
Received probe estimates update the bound stamped on outgoing frames. The
server's bound is zero relative to itself.

Deployment must account for these current limits:

- Without `[clock]`, the initial zero bound remains even for Zenoh. It is not
  evidence of synchronization between machines.
- A gate timeout sets `CLOCK_DEGRADED` and records `clock_gate_timeout`, then
  continues running. It does not prevent controller activation or arm a safety
  interlock. A deployment needing that behavior must enforce it separately.
- `chronyc tracking` is recorded in `clock.jsonl`; it does not discipline the
  clock or feed the bound assigned by `ClockService`. Clock synchronization
  remains an external service.
- The probe window has no elapsed-time expiry. No new replies means no new
  bound update, not a demonstrated drift bound during a network outage.
- Cross-host rounds need the same `session.epoch_ns`. Omitting it produces a
  local start epoch and the host records `epoch_local_only` on linked sessions.

The native composition checks in [the September 2026 validation](validation/native-20260926/README.md)
use one host and a software plant. They do not establish an onboard or radio
clock bound.

Decisions, rationale and plans for this runtime live in the academic knowledge base: lxk36/academic, `docs/architecture/xgc2-sync-runtime/time-model.md` (the timing contract).
