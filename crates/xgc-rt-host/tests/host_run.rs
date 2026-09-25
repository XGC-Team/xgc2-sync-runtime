//! Z1 exit: one host loads four Rust stubs and one C stub from a manifest.
//! It runs them on rounds and dirty triggers, applies the restart policy,
//! and stays idle without input.

mod common;

use std::path::Path;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use xgc_rt_audit::{merge_run, MergeOptions};
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

#[test]
fn five_plugins_in_rust_and_c_run_a_pipeline_with_a_clean_audit() {
    let dir = common::scratch("pipeline");
    let manifest = format!(
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
bind = {{ cmd = {{ channel = "cmd", from = ["uav1"] }} }}
"#,
        header("pipeline", 20, 1000),
        common::lib("stub_perception"),
        common::lib("stub_estimation"),
        common::lib("stub_planning"),
        common::lib("stub_control"),
        common::lib("c_stub"),
    );
    let summary = run(&dir, &manifest);
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
        assert!(p.steps > 0 && p.last_error.is_none(), "{name}: {p:?}");
    }
    assert!(summary.rounds >= 45, "rounds {}", summary.rounds);
    let perception = plugin(&summary, "perception");
    assert_eq!(perception.published, summary.rounds, "one detection per round");
    assert_eq!(plugin(&summary, "control").published, plugin(&summary, "sink").consumed);
    let t = &summary.timings;
    assert!(t.manifest_ms <= t.plugins_loaded_ms && t.plugins_loaded_ms <= t.ports_ready_ms && t.ports_ready_ms <= t.first_round_ms);

    let report = merge_run(&summary.audit_dir, MergeOptions::default()).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    assert_eq!(report.streams.len(), 4);
    for s in &report.streams {
        assert_eq!(s.counts.lost, 0, "{s:?}");
        assert_eq!((s.counts.duplicates, s.counts.reordered), (0, 0));
        assert!(s.counts.expected > 0);
    }
    let detections = report.streams.iter().find(|s| s.channel == "detections").unwrap();
    assert_eq!(detections.counts.expected, perception.published);
    assert_eq!(detections.age_at_use_ns.count, detections.counts.received, "every detection was consumed");
    xgc_rt_audit::write_report(&report, &summary.audit_dir.join("merged")).unwrap();
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
