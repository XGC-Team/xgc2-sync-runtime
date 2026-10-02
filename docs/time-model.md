# Time model

Session time is `int64` nanoseconds from `xgc-rt-core::clock::Clock`. Each
node derives round k from `session.epoch_ns + k * period_ns`, without a network
tick. A module's `ctx.now` is its own clock sample; `round_start` names its
scheduled round boundary. Equal epochs do not make different module arrival
times or independent batches one simultaneously sampled world frame.

Without `[clock_source]`, the executable uses the host wall clock. Its bound
starts **unknown** (`u32::MAX`), except a single-node loopback manifest whose
clock is entirely local. A probe server has zero error relative to itself;
this alone establishes neither external synchronization nor UTC accuracy.

An explicit simulator source keeps the existing native ROS clock path and
frozen world identity; see [Simulator clock source](clock-source.md). Wall
probes and simulator sources cannot be combined. Simulator pause, freshness
and missed-epoch checks are unchanged.

## Required wall-clock admission

HIL wall deployments freeze an explicit external-clock policy and one shared
future epoch. Their renderer emits the existing `[clock]` service with required
admission, for example:

```toml
[session]
# The Run coordinator supplies exactly this epoch to every participant.
epoch_ns = 1900000000000000000

[clock]
role = "client"                 # "server" exactly on the reference roster node
server = "station"              # roster identity, not a discovered replacement
required = true
interval_ms = 1000
gate_ms = 2.0
gate_timeout_ms = 5000
window = 8
stale_after_ms = 3000
chrony_source = "192.168.0.1"    # exact selected source in `chronyc sources -v`
chrony_max_offset_ms = 2.0
chrony_max_uncertainty_ms = 2.0
```

The external chrony service remains responsible for clock discipline. The host
only reads `chronyc -c tracking` and `chronyc sources -v`: no source selection,
step, slew or new synchronization service. Admission requires the frozen
selected source, Normal leap status, a finite offset within the offset limit,
and root dispersion plus half root delay within the uncertainty limit. Missing,
malformed, failed or timed-out observations are failures, never zero error.
Each command has a 500 ms wall timeout and runs outside module scheduling.

A client also needs at least three fresh four-stamp probe samples whose
estimated bound is within `gate_ms`. Startup probes use `min(interval_ms, 100 ms)`;
the required freshness window must exceed two such intervals to admit three samples.
Replies must match the reference origin, an outstanding nonce and its exact
original transmit stamp; consumed, expired or unmatched requests cannot refresh
the window. Required clients reject unknown/degraded reference replies. The
minimum-delay estimator excludes each expired sample even if other replies
continue arriving. All evidence ages use monotonic elapsed time.

Required admission checks both clients and the reference's external clock.
A missing source, invalid policy or missing shared epoch is rejected. A failed
gate never activates or steps modules. Once startup completes, a passed wall
epoch also fails before activation: the caller must freeze a new common future
epoch and start a new Session. Hosts never shift the epoch locally or catch up
by resetting their own origin.

After admission, at least one unexpired estimate within the same bound and a
fresh valid external observation must remain. Three samples establish initial
admission; they are not a second sliding-window quorum during normal probing.
During a required run, lost validity latches a Session failure. Module dispatch
and Host output callbacks check expiry themselves, including while the main
host loop is busy. They stop domain work and discard outputs; the host then
performs its existing orderly shutdown and returns a non-success result.
Later replies or recovered chrony cannot resume that Session. This host gate
is not an actuator-specific fail-safe and does not change Adapter or physical
protection policy. Plugins must keep external side effects within their normal
Host-dispatched lifecycle; an already-running plugin call is not forcibly
preempted.

`clock.jsonl` records accepted probe estimates and external observations;
`health.jsonl` records gate results and the abort reason. Outgoing frame bounds
become unknown/degraded on lost evidence. Configured timing fields are frozen
limits, not measured radio/board guarantees.

## Diagnostic and single-host use

Existing probe manifests with `required = false` retain their diagnostic
startup behavior: gate timeout records `clock_gate_timeout` and continues with
`CLOCK_DEGRADED`. Their measurements now expire as well. They are not HIL
admission. A manifest without `[clock]` cannot prove inter-machine clock quality.

A local single-Host lightweight launcher may generate one future epoch for that
Host and all its batches. Copying that launcher onto multiple boards would
create different epochs and is not a distributed deployment contract. A
multi-board consumer must accept the Run's frozen epoch instead.

Plugin manifests may additionally freeze `expected_name` and `expected_version`.
The Host compares these with the loaded library's actual descriptor before
calling any plugin's create function. These checks complement SHA256 pins;
they do not create a second artifact registry.

The [September 2026 native validation](validation/native-20260926/README.md)
uses one host and a software plant. CPU tests with local transports and fake
chrony observations likewise do not establish onboard or radio clock bounds.
The external-clock/Session policy follows runtime-sync's existing ClockMonitor
and shared-epoch/local-cycle contract. Decisions live in the academic knowledge
base: lxk36/academic, `docs/architecture/xgc2-sync-runtime/time-model.md`.
