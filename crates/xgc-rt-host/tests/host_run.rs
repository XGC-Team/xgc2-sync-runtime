//! Z1 exit: one host loads four Rust stubs and one C stub from a manifest.
//! It runs them on rounds and dirty triggers, applies the restart policy,
//! and stays idle without input.

mod common;

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{RxSink, Transport, TransportContext, TransportError};
use xgc_rt_core::{ChannelId, OriginId};
use xgc_rt_host::{Host, HostOptions, RunSummary};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

struct PanicOnWaitReady(LoopbackTransport);

impl Transport for PanicOnWaitReady {
    fn kind(&self) -> &str {
        self.0.kind()
    }

    fn open(&mut self, ctx: &TransportContext, sink: RxSink) -> Result<(), TransportError> {
        self.0.open(ctx, sink)
    }

    fn declare_out(&mut self, channel: ChannelId) -> Result<(), TransportError> {
        self.0.declare_out(channel)
    }

    fn declare_in(&mut self, channel: ChannelId, origins: &[OriginId]) -> Result<(), TransportError> {
        self.0.declare_in(channel, origins)
    }

    fn send(&mut self, channel: ChannelId, frame: &[u8]) -> Result<(), TransportError> {
        self.0.send(channel, frame)
    }

    fn wait_ready(&mut self, _timeout: Duration) -> bool {
        panic!("production Host::run must not wait for remote peer readiness")
    }

    fn close(&mut self) {
        self.0.close();
    }
}

fn run(dir: &Path, manifest: &str) -> RunSummary {
    run_with_transport(
        dir,
        manifest,
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
    )
}

fn run_with_transport(dir: &Path, manifest: &str, transport: Box<dyn Transport>) -> RunSummary {
    std::fs::write(dir.join("node.toml"), manifest).unwrap();
    let manifest = Manifest::from_toml_str(manifest).unwrap();
    let host = Host::new(
        manifest,
        dir,
        transport,
        Arc::new(WallClock::new(0)),
        HostOptions::default(),
    )
    .unwrap();
    host.run(&AtomicBool::new(false)).unwrap()
}

fn header(session: &str, period_ms: u32, run_for_ms: u32) -> String {
    format!(
        r#"
[session]
id = "{session}"
node = "uav1"
roster = ["uav1"]
period_ms = {period_ms}
start_delay_ms = 50
run_for_ms = {run_for_ms}

[transport]
kind = "loopback"

[audit]
dir = "audit"
"#
    )
}

fn plugin(summary: &RunSummary, name: &str) -> xgc_rt_host::host::PluginSummary {
    summary.plugins.iter().find(|p| p.name == name).unwrap().clone()
}

/// Perception → estimation → planning → control → sink, all in one
/// process. `sink` is the C stub; `sink_config` and `sink_latest` tune it.
fn pipeline(session: &str, period_ms: u32, run_for_ms: u32, sink_config: &str, sink_latest: bool) -> String {
    format!(
        r#"{}
[[channel]]
name = "detections"
qos = "state"
[[channel]]
name = "state"
qos = "state"
[[channel]]
name = "plan"
qos = "control"
[[channel]]
name = "cmd"
qos = "event"

[[plugin]]
name = "perception"
path = "{}"
trigger = "on_round"
config = {{ payload_bytes = 256 }}
bind = {{ detections = {{ channel = "detections" }} }}

[[plugin]]
name = "estimation"
path = "{}"
trigger = "on_dirty"
bind = {{ detections = {{ channel = "detections", from = ["uav1"] }}, state = {{ channel = "state" }} }}

[[plugin]]
name = "planning"
path = "{}"
trigger = "both"
bind = {{ state = {{ channel = "state", from = ["uav1"] }}, plan = {{ channel = "plan" }} }}

[[plugin]]
name = "control"
path = "{}"
trigger = "on_dirty"
bind = {{ plan = {{ channel = "plan", from = ["uav1"] }}, cmd = {{ channel = "cmd" }} }}

[[plugin]]
name = "sink"
path = "{}"
trigger = "on_dirty"
restart = {{ policy = "on_error", max = 3, backoff_ms = 100 }}
config = {{ {sink_config} }}
bind = {{ cmd = {{ channel = "cmd", from = ["uav1"], latest = {sink_latest} }} }}
"#,
        header(session, period_ms, run_for_ms),
        common::lib("stub_perception"),
        common::lib("stub_estimation"),
        common::lib("stub_planning"),
        common::lib("stub_control"),
        common::lib("c_stub"),
    )
}

#[test]
fn five_plugins_in_rust_and_c_hand_off_in_memory_on_their_own_threads() {
    let dir = common::scratch("pipeline");
    let summary = run(&dir, &pipeline("pipeline", 20, 1000, "", false));
    println!("{}", serde_json::to_string_pretty(&summary).unwrap());

    for (name, domain) in [
        ("perception", "tracking"),
        ("estimation", "converged"),
        ("planning", "planning"),
        ("control", "track"),
        ("sink", "counting"),
    ] {
        let p = plugin(&summary, name);
        assert_eq!(p.state, "inactive", "{name}: {p:?}");
        assert_eq!(p.domain_state, domain, "{name}");
        assert!(p.steps > 0 && p.last_error.is_none() && p.dropped == 0, "{name}: {p:?}");
    }
    assert!(summary.rounds >= 45, "rounds {}", summary.rounds);
    let perception = plugin(&summary, "perception");
    assert_eq!(perception.published, perception.steps, "one detection per step");
    // Each module runs on its own thread: under load it may wake late and
    // step once for two rounds (recorded as rounds_skipped in health).
    let health = std::fs::read_to_string(summary.audit_dir.join("uav1/health.jsonl")).unwrap();
    let skipped: u64 = health
        .lines()
        .map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap())
        .filter(|v| v["event"] == "rounds_skipped" && v["plugin"] == "perception")
        .map(|v| v["to"].as_u64().unwrap() - v["from"].as_u64().unwrap() + 1)
        .sum();
    assert!(perception.steps + skipped + 1 >= summary.rounds && perception.steps <= summary.rounds + 1, "steps {} skipped {skipped} rounds {}", perception.steps, summary.rounds);
    // At stop, at most one sample per hop was written after its reader stopped.
    let estimation = plugin(&summary, "estimation");
    assert!(estimation.consumed <= perception.published && estimation.consumed + 1 >= perception.published, "every detection but the last in flight was read");
    let (cmds, sink) = (plugin(&summary, "control").published, plugin(&summary, "sink").consumed);
    assert!(sink <= cmds && sink + 1 >= cmds, "sink read {sink} of {cmds}");
    let t = &summary.timings;
    assert!(t.manifest_ms <= t.plugins_loaded_ms && t.plugins_loaded_ms <= t.ports_ready_ms && t.ports_ready_ms <= t.first_round_ms);

    // Same-process hops are memory only: nothing went over the link.
    let node = summary.audit_dir.join("uav1");
    assert_eq!(std::fs::metadata(node.join("records.bin")).unwrap().len(), 0, "no link frames for same-process hops");
    // Step records show what each step read: every detection seq exactly once.
    let steps = std::fs::read_to_string(node.join("steps.jsonl")).unwrap();
    let mut seqs: Vec<u64> = steps
        .lines()
        .map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap())
        .filter(|v| v["m"] == "estimation")
        .flat_map(|v| v["in"].as_array().unwrap().iter().map(|r| r[2].as_u64().unwrap()).collect::<Vec<_>>())
        .collect();
    seqs.sort_unstable();
    assert_eq!(seqs, (1..=estimation.consumed).collect::<Vec<_>>(), "in order, each once");
}

#[test]
fn linked_host_runs_local_outputs_without_waiting_for_remote_subscribers() {
    let dir = common::scratch("local-output-without-peers");
    let manifest = pipeline("local-output-without-peers", 20, 500, "", false)
        .replace("roster = [\"uav1\"]", "roster = [\"uav1\", \"uav2\"]");
    assert!(manifest.contains("roster = [\"uav1\", \"uav2\"]"));

    // There is no uav2 host or remote subscriber. The panic wrapper makes any
    // production call to Transport::wait_ready fail this real Host run.
    let transport = Box::new(PanicOnWaitReady(LoopbackTransport::new(LoopbackBus::new())));
    let summary = run_with_transport(&dir, &manifest, transport);
    let perception = plugin(&summary, "perception");
    let estimation = plugin(&summary, "estimation");
    assert!(perception.published > 0, "local producer did not publish: {perception:?}");
    assert!(estimation.consumed > 0, "local consumer did not receive input: {estimation:?}");
}

#[test]
fn a_latest_input_keeps_only_the_newest_sample_and_a_queue_keeps_every_one() {
    // sink takes 30 ms per step while control writes one command per 5 ms
    // round.
    let queued = run(&common::scratch("queue"), &pipeline("queue", 5, 600, "step_sleep_ms = 30", false));
    let latest = run(&common::scratch("latest"), &pipeline("latest", 5, 600, "step_sleep_ms = 30", true));
    let (q, l) = (plugin(&queued, "sink"), plugin(&latest, "sink"));
    println!("queue: {q:?}\nlatest: {l:?}");
    let written = plugin(&queued, "control").published;
    assert!(q.consumed + 8 >= written, "a queue keeps every sample: read {} of {written}", q.consumed);
    assert!(l.consumed <= l.steps, "latest: at most one sample per step ({} reads, {} steps)", l.consumed, l.steps);
    assert!(l.consumed * 3 < plugin(&latest, "control").published, "latest skipped superseded samples");
    assert_eq!((q.dropped, l.dropped), (0, 0));
}

#[test]
fn restart_policy_restarts_a_failing_c_plugin_then_gives_up() {
    let dir = common::scratch("restart");
    let manifest = format!(
        r#"{}
[[channel]]
name = "cmd"
qos = "event"

[[plugin]]
name = "sink"
path = "{}"
trigger = "on_round"
config = {{ fail_after = 3 }}
restart = {{ policy = "on_error", max = 2, backoff_ms = 10 }}
bind = {{ cmd = {{ channel = "cmd", from = ["uav1"] }} }}
"#,
        header("restart", 10, 400),
        common::lib("c_stub"),
    );
    let summary = run(&dir, &manifest);
    let sink = plugin(&summary, "sink");
    assert_eq!(sink.restarts, 2, "{sink:?}");
    assert_eq!(sink.state, "error", "a third failure exhausts max = 2");
    assert_eq!(sink.steps, 9, "three instances, each failing on its third step");
    let health = std::fs::read_to_string(summary.audit_dir.join("uav1/health.jsonl")).unwrap();
    assert_eq!(health.matches("\"cause\":\"Reset\"").count(), 2);
    assert_eq!(health.matches("\"cause\":\"Fault\"").count(), 3);
}

#[test]
fn an_input_driven_plugin_without_input_never_steps_and_the_host_sleeps() {
    let dir = common::scratch("idle");
    let manifest = format!(
        r#"{}
[[channel]]
name = "plan"
qos = "control"
[[channel]]
name = "cmd"
qos = "event"

[[plugin]]
name = "control"
path = "{}"
trigger = "on_dirty"
bind = {{ plan = {{ channel = "plan", from = ["uav1"] }}, cmd = {{ channel = "cmd" }} }}
"#,
        header("idle", 20, 600),
        common::lib("stub_control"),
    );
    let summary = run(&dir, &manifest);
    let control = plugin(&summary, "control");
    assert_eq!(control.steps, 0, "no input, no step");
    assert_eq!(control.domain_state, "hold");
    // One wake per round boundary, plus a couple before E0 and at stop.
    assert!(summary.wakeups <= summary.rounds + 4, "wakeups {} rounds {}", summary.wakeups, summary.rounds);
}

/// perception writes every round; `sink` (C stub, its own thread) is tuned
/// by `sink` fields and config.
fn watched(session: &str, sink: &str) -> String {
    format!(
        r#"{}
[[channel]]
name = "detections"
qos = "state"
[[channel]]
name = "cmd"
qos = "event"

[[plugin]]
name = "perception"
path = "{}"
trigger = "on_round"
bind = {{ detections = {{ channel = "detections" }} }}

[[plugin]]
name = "sink"
path = "{}"
trigger = "on_round"
{sink}
bind = {{ cmd = {{ channel = "cmd", from = ["uav1"] }} }}
"#,
        header(session, 20, 3000),
        common::lib("stub_perception"),
        common::lib("c_stub"),
    )
}

#[test]
fn a_hung_module_is_abandoned_and_replaced_while_the_others_keep_running() {
    // Budget 10 ms, so a step over 100 ms is a hang. Each instance hangs on
    // its 3rd step; the replacement hangs too, and the second abandon stops
    // the aggregator (default max_abandoned = 2).
    let dir = common::scratch("hang");
    let summary = run(
        &dir,
        &watched("hang", "step_budget_ms = 10\nrestart = { policy = \"on_error\", max = 1, backoff_ms = 20 }\nconfig = { hang_after = 3 }"),
    );
    let (sink, perception) = (plugin(&summary, "sink"), plugin(&summary, "perception"));
    println!("{summary:?}");
    assert_eq!((sink.abandons, sink.restarts), (2, 1), "{sink:?}");
    assert_eq!(sink.state, "error");
    assert!(summary.aborted.is_some(), "two abandons reach max_abandoned = 2");
    assert!(summary.rounds < 100, "stopped early: {} rounds of 150", summary.rounds);
    // perception never waited on the hung sink: it stepped every round.
    assert!(perception.steps + 1 >= summary.rounds && perception.state == "inactive", "{perception:?} rounds {}", summary.rounds);
    let health = std::fs::read_to_string(summary.audit_dir.join("uav1/health.jsonl")).unwrap();
    assert_eq!(health.matches("\"event\":\"abandoned\"").count(), 2);
    assert!(health.contains("\"event\":\"aborted\""));
}

#[test]
fn a_step_over_budget_degrades_the_module_without_abandoning_it() {
    let dir = common::scratch("overrun");
    // 30 ms steps against a 10 ms budget: Degraded, but not a hang (100 ms).
    let slow = run(&dir, &watched("overrun", "step_budget_ms = 10\nconfig = { step_sleep_ms = 30 }"));
    let sink = plugin(&slow, "sink");
    assert_eq!((sink.abandons, sink.state.as_str()), (0, "inactive"), "{sink:?}");
    let health = std::fs::read_to_string(slow.audit_dir.join("uav1/health.jsonl")).unwrap();
    assert!(health.contains("\"event\":\"overrun\""));
    assert!(health.contains("\"detail\":\"step over budget\""), "Active → Degraded on overrun");
    assert!(slow.aborted.is_none());

    // Budget 50 ms: the same steps are within it and nothing degrades.
    let dir = common::scratch("within-budget");
    let ok = run(&dir, &watched("within", "step_budget_ms = 50\nconfig = { step_sleep_ms = 5 }"));
    let health = std::fs::read_to_string(ok.audit_dir.join("uav1/health.jsonl")).unwrap();
    assert!(!health.contains("\"event\":\"overrun\""));
}

#[test]
fn a_sampled_step_log_keeps_one_round_in_n_and_every_failed_or_slow_step() {
    // perception steps every 20 ms round; slow overruns its 10 ms budget on
    // every step; failing fails its second step and is not restarted.
    let dir = common::scratch("sampled-steps");
    let manifest = format!(
        r#"{}
[[channel]]
name = "detections"
qos = "state"
[[channel]]
name = "cmd"
qos = "event"

[[plugin]]
name = "perception"
path = "{}"
trigger = "on_round"
bind = {{ detections = {{ channel = "detections" }} }}

[[plugin]]
name = "slow"
path = "{}"
trigger = "on_round"
step_budget_ms = 10
config = {{ step_sleep_ms = 12 }}
bind = {{ cmd = {{ channel = "cmd", from = ["uav1"] }} }}

[[plugin]]
name = "failing"
path = "{}"
trigger = "on_round"
config = {{ fail_after = 2 }}
bind = {{ cmd = {{ channel = "cmd", from = ["uav1"] }} }}
"#,
        header("sampled-steps", 20, 1000).replace("dir = \"audit\"", "dir = \"audit\"\nsteps_every = 5"),
        common::lib("stub_perception"),
        common::lib("c_stub"),
        common::lib("c_stub"),
    );
    let summary = run(&dir, &manifest);
    let steps = std::fs::read_to_string(summary.audit_dir.join("uav1/steps.jsonl")).unwrap();
    let records: Vec<serde_json::Value> = steps.lines().map(|l| serde_json::from_str(l).unwrap()).collect();
    let rounds_of = |name: &str| -> Vec<u64> {
        records.iter().filter(|r| r["m"] == name).map(|r| r["k"].as_u64().unwrap()).collect()
    };

    // A healthy module is recorded for one round in five, and still counted
    // on every step.
    let (perception, recorded) = (plugin(&summary, "perception"), rounds_of("perception"));
    assert!(recorded.iter().all(|k| k % 5 == 0), "unsampled rounds recorded: {recorded:?}");
    assert!(perception.steps >= 40 && recorded.len() as u64 * 5 <= perception.steps + 5, "{} records for {} steps", recorded.len(), perception.steps);
    assert!(recorded.len() >= 6, "sampled rounds missing: {recorded:?}");

    // Every step over budget and the failed step are recorded, whatever the round.
    let slow = plugin(&summary, "slow");
    assert_eq!(rounds_of("slow").len() as u64, slow.steps, "{slow:?}");
    let failing = plugin(&summary, "failing");
    let error = failing.last_error.clone().unwrap_or_default();
    let failed_round: u64 = error.rsplit(' ').next().unwrap().parse().unwrap_or_else(|_| panic!("last error {error:?}"));
    assert_eq!(failing.steps, 2, "{failing:?}");
    assert!(rounds_of("failing").contains(&failed_round), "failed step (round {failed_round}) missing: {:?}", rounds_of("failing"));
}
