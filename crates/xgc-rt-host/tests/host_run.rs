//! Z1 exit: one host loads four Rust stubs and one C stub from a manifest.
//! It runs them on rounds and dirty triggers, applies the restart policy,
//! and stays idle without input.

mod common;

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions, RunSummary};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

fn run(dir: &Path, manifest: &str) -> RunSummary {
    std::fs::write(dir.join("node.toml"), manifest).unwrap();
    let manifest = Manifest::from_toml_str(manifest).unwrap();
    let host = Host::new(
        manifest,
        dir,
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
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
