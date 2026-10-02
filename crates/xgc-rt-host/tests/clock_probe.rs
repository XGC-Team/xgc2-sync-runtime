//! Z2c: the in-host clock probe measures each node's offset to the station
//! over the data path, gates activation on the bound, and stamps the bound
//! into every frame, so a skewed clock can't produce an unqualified OWD.
//!
//! station: probe server (bound 0), runs the estimation stub fed by both
//! nodes' detections. uav1: clock +5 ms, gate 10 ms, so it passes. uav2:
//! clock −2 ms, gate 1 ms, so it cannot pass and must run flagged degraded.

mod common;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use xgc_rt_audit::{merge_run, MergeOptions};
use xgc_rt_core::clock::{Clock, SkewedClock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

fn manifest(node: &str, role: &str, gate_ms: f64, e0: i64, plugin: &str) -> String {
    format!(
        r#"
[session]
id = "clk"
node = "{node}"
roster = ["station", "uav1", "uav2"]
period_ms = 20
epoch_ns = {e0}
run_for_ms = 1500

[transport]
kind = "loopback"

[audit]
dir = "audit"

[clock]
role = "{role}"
server = "station"
interval_ms = 100
gate_ms = {gate_ms}
gate_timeout_ms = 800

[[channel]]
name = "detections"
qos = "state"
[[channel]]
name = "state"
qos = "state"

{plugin}
"#
    )
}

#[test]
fn probe_bounds_a_skewed_clock_and_gates_activation() {
    let dir = common::scratch("clock-probe");
    let perception = format!(
        "[[plugin]]\nname = \"perception\"\npath = \"{}\"\ntrigger = \"on_round\"\nbind = {{ detections = {{ channel = \"detections\" }} }}",
        common::lib("stub_perception")
    );
    let estimation = format!(
        "[[plugin]]\nname = \"estimation\"\npath = \"{}\"\ntrigger = \"on_dirty\"\nbind = {{ detections = {{ channel = \"detections\", from = [\"uav1\", \"uav2\"] }}, state = {{ channel = \"state\" }} }}",
        common::lib("stub_estimation")
    );
    let bus = LoopbackBus::new();
    let station_clock: Arc<dyn Clock> = Arc::new(WallClock::new(0));
    let e0 = station_clock.now() + 2_500_000_000;
    let specs: Vec<(&str, &str, f64, Arc<dyn Clock>, &String)> = vec![
        ("station", "server", 2.0, station_clock.clone(), &estimation),
        ("uav1", "client", 10.0, Arc::new(SkewedClock::new(5_000_000)), &perception),
        ("uav2", "client", 1.0, Arc::new(SkewedClock::new(-2_000_000)), &perception),
    ];
    let stop = Arc::new(AtomicBool::new(false));
    let mut runners = Vec::new();
    for (node, role, gate, clock, plugin) in specs {
        let m = Manifest::from_toml_str(&manifest(node, role, gate, e0, plugin)).unwrap();
        let host = Host::new(m, &dir, Box::new(LoopbackTransport::new(bus.clone())), clock, HostOptions::default()).unwrap();
        let stop = stop.clone();
        runners.push(std::thread::spawn(move || host.run(&stop).unwrap()));
    }
    for r in runners {
        r.join().unwrap();
    }

    let health = |n: &str| std::fs::read_to_string(dir.join("audit").join(n).join("health.jsonl")).unwrap();
    assert!(health("uav1").contains("clock_gate_passed"), "uav1 (5 ms off, gate 10 ms) must pass");
    assert!(health("uav2").contains("clock_gate_timeout"), "uav2 (2 ms off, gate 1 ms) must not pass");

    // uav1's own estimate: station − local = −5 ms, within its bound.
    let last = std::fs::read_to_string(dir.join("audit/uav1/clock.jsonl")).unwrap();
    let est: serde_json::Value = serde_json::from_str(last.lines().last().unwrap()).unwrap();
    let (off, bound, delay) = (
        est["estimate"]["offset_ns"].as_i64().unwrap(),
        est["estimate"]["bound_ns"].as_i64().unwrap(),
        est["estimate"]["delay_ns"].as_i64().unwrap(),
    );
    println!("uav1 estimate: offset {:.3} ms, delay {:.3} ms, bound {:.3} ms", off as f64 / 1e6, delay as f64 / 1e6, bound as f64 / 1e6);
    assert!((off + 5_000_000).abs() <= delay / 2 + 1, "offset {off} must be -5 ms within delay/2");
    assert!(bound >= 5_000_000 && bound <= 5_000_000 + delay + 1, "bound {bound}");

    // The frames carry the bound. The invariant is |measured − true| ≤ bound.
    // In one process the measured−true error is exactly the sender's skew
    // (the station's clock is the reference), so every frame's bound must
    // cover it.
    let report = merge_run(&dir.join("audit"), MergeOptions::default()).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    for s in report.streams.iter().filter(|s| s.channel == "detections") {
        println!(
            "{} → {}: OWD p50 {:.3} ms, bound p50/max {:.3}/{:.3} ms, received {}",
            s.origin, s.receiver, s.owd_ns.p50 as f64 / 1e6, s.owd_bound_ns.p50 as f64 / 1e6, s.owd_bound_ns.max as f64 / 1e6, s.counts.received
        );
        assert!(s.counts.received > 50);
        let skew: i64 = if s.origin == "uav1" { 5_000_000 } else { 2_000_000 };
        assert!(s.owd_bound_ns.min >= skew, "{}: bound min {} < skew {skew}", s.origin, s.owd_bound_ns.min);
        let true_p50 = s.owd_ns.p50 + if s.origin == "uav1" { 5_000_000 } else { -2_000_000 };
        assert!(true_p50 >= 0, "{}: implied true OWD {true_p50} must be non-negative", s.origin);
    }
    let uav1 = report.streams.iter().find(|s| s.channel == "detections" && s.origin == "uav1").unwrap();
    assert!(uav1.owd_ns.p50 < -4_000_000, "the 5 ms skew shows in the raw OWD: {}", uav1.owd_ns.p50);
}

fn strict_manifest(node: &str, role: &str, e0: i64, run_ms: u64, transport: &str, plugin: &str) -> String {
    format!(r#"
[session]
id = "strict-clock"
node = "{node}"
roster = ["station", "uav1"]
period_ms = 10
epoch_ns = {e0}
run_for_ms = {run_ms}
[transport]
{transport}
[audit]
dir = "audit"
[clock]
role = "{role}"
server = "station"
required = true
chrony_source = "station-clock"
chrony_max_offset_ms = 2.0
chrony_max_uncertainty_ms = 2.0
interval_ms = 100
gate_ms = 2.0
gate_timeout_ms = 1000
stale_after_ms = 800
[[channel]]
name = "detections"
qos = "state"
[[plugin]]
name = "perception"
path = "{plugin}"
expected_name = "stub-perception"
expected_version = "0.1.0"
trigger = "on_round"
bind = {{ detections = {{ channel = "detections" }} }}
"#)
}

fn fake_chrony(dir: &std::path::Path) -> std::path::PathBuf {
    use std::os::unix::fs::PermissionsExt;
    let bin = dir.join("bin"); std::fs::create_dir_all(&bin).unwrap();
    let path = bin.join("chronyc");
    std::fs::write(&path, r#"#!/bin/sh
if [ "$1" = "-c" ]; then
    echo 'x,station-clock,3,0,0.00001,0,0,0,0,0,0.00002,0.00003,1,Normal'
else
    echo '^* station-clock 2 6 377 1'
fi
"#).unwrap();
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o755)).unwrap();
    bin
}

fn host_process(dir: &std::path::Path, file: &str, text: &str, path: &std::path::Path) -> std::process::Child {
    use std::process::{Command, Stdio};
    let manifest = dir.join(file); std::fs::write(&manifest, text).unwrap();
    Command::new(env!("CARGO_BIN_EXE_xgc-rt-host")).args(["--manifest", manifest.to_str().unwrap()])
        .env("PATH", path).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().unwrap()
}

fn output(child: std::process::Child) -> (std::process::ExitStatus, serde_json::Value) {
    let output = child.wait_with_output().unwrap();
    let summary = serde_json::from_slice(&output.stdout).unwrap_or_else(|_| panic!("host output: {} / {}", String::from_utf8_lossy(&output.stdout), String::from_utf8_lossy(&output.stderr)));
    (output.status, summary)
}

#[test]
fn required_clock_refuses_bad_external_clock_missing_probe_and_missed_epoch_before_activation() {
    let plugin = common::lib("stub_perception");
    for (name, role, external, past) in [("no-external", "server", false, false), ("no-probe", "client", true, false), ("missed-epoch", "server", true, true)] {
        let dir = common::scratch(name);
        let bin = if external { fake_chrony(&dir) } else { dir.join("absent-bin") };
        let e0 = WallClock::new(0).now() + if past { -1_000_000_000 } else { 3_000_000_000 };
        let node = if role == "client" { "uav1" } else { "station" };
        let (status, summary) = output(host_process(&dir, "host.toml", &strict_manifest(node, role, e0, 500, "kind = \"loopback\"", &plugin), &bin));
        assert!(!status.success(), "{name}: {summary}");
        let reason = summary["aborted"].as_str().unwrap();
        assert!(reason.contains(if past { "epoch passed" } else { "clock gate" }), "{reason}");
        assert_eq!(summary["plugins"][0]["steps"], 0);
        assert_eq!(summary["plugins"][0]["published"], 0);
        let health = std::fs::read_to_string(dir.join("audit").join(node).join("health.jsonl")).unwrap();
        assert!(!health.contains("\"cause\":\"Activate\""), "{name}: activated before admission");
    }
}

#[test]
fn required_clock_two_hosts_share_epoch_then_probe_loss_aborts_and_stops_outputs() {
    let plugin = common::lib("stub_perception");
    let dir = common::scratch("required-clock-pair");
    let bin = fake_chrony(&dir);
    let port = common::listen_port();
    let epoch = WallClock::new(0).now() + 3_000_000_000;
    // The reference finishes early; its client must not keep old probe evidence
    // for the remaining 2.7 seconds of the requested run.
    let server = strict_manifest("station", "server", epoch, 300, &format!("kind=\"zenoh\"\nlisten=[\"tcp/127.0.0.1:{port}\"]"), &plugin);
    let client = strict_manifest("uav1", "client", epoch, 3000, &format!("kind=\"zenoh\"\nconnect=[\"tcp/127.0.0.1:{port}\"]"), &plugin);
    let a = host_process(&dir, "server.toml", &server, &bin);
    let b = host_process(&dir, "client.toml", &client, &bin);
    let (a_status, a) = output(a);
    let (b_status, b) = output(b);
    assert!(a_status.success(), "reference: {a}");
    assert!(!b_status.success(), "expired client returned success: {b}");
    assert_eq!(a["e0_ns"], b["e0_ns"]);
    assert!(b["aborted"].as_str().unwrap().contains("clock probe"), "{b}");
    let steps = b["plugins"][0]["steps"].as_u64().unwrap();
    assert!(steps > 0 && steps < 150, "client did not start then stop after expiry: {b}");
    let health = std::fs::read_to_string(dir.join("audit/uav1/health.jsonl")).unwrap();
    assert!(health.contains("clock_gate_passed"));
    let records = std::fs::read(dir.join("audit/uav1/records.bin")).unwrap();
    use xgc_rt_audit::record::{Kind, Record, RECORD_LEN};
    let state: Vec<_> = records.chunks_exact(RECORD_LEN).map(|v| Record::decode(v).unwrap())
        .filter(|r| r.kind == Kind::Tx && r.channel == 0).collect();
    assert!(!state.is_empty());
    assert!(state.iter().all(|r| r.bound_a != u32::MAX));
    assert!(state.last().unwrap().t_b < epoch + 1_500_000_000, "state still published after probe expiry");
}

#[test]
fn production_unknown_bound_and_loaded_descriptor_checks_are_consumed() {
    let plugin = common::lib("stub_perception");
    let dir = common::scratch("clock-production-descriptor");
    let epoch = WallClock::new(0).now() + 400_000_000;
    let mut text = strict_manifest("station", "server", epoch, 100, "kind=\"loopback\"", &plugin);
    let start = text.find("[clock]").unwrap(); let end = text.find("[[channel]]").unwrap();
    text.replace_range(start..end, "");
    let (status, summary) = output(host_process(&dir, "unknown.toml", &text, &dir));
    assert!(status.success(), "{summary}");
    use xgc_rt_audit::record::{Kind, Record, RECORD_LEN};
    let records = std::fs::read(dir.join("audit/station/records.bin")).unwrap();
    let tx: Vec<_> = records.chunks_exact(RECORD_LEN).map(|b| Record::decode(b).unwrap()).filter(|r| r.kind == Kind::Tx).collect();
    assert!(!tx.is_empty()); assert!(tx.iter().all(|r| r.bound_a == u32::MAX));
    for (key, wanted, bad) in [("expected_name", "stub-perception", "legacy-plant"), ("expected_version", "0.1.0", "0.2.0")] {
        let wrong = text.replace(&format!("{key} = \"{wanted}\""), &format!("{key} = \"{bad}\""));
        let out = host_process(&dir, "wrong.toml", &wrong, &dir).wait_with_output().unwrap();
        assert!(!out.status.success());
        assert!(String::from_utf8_lossy(&out.stderr).contains("loaded descriptor"));
    }
}

#[test]
fn required_short_probe_interval_passes_with_three_fresh_samples() {
    let plugin = common::lib("stub_perception");
    let dir = common::scratch("required-short-clock-pair");
    let bin = fake_chrony(&dir);
    let port = common::listen_port();
    let epoch = WallClock::new(0).now() + 2_000_000_000;
    let short = |node: &str, role: &str, transport: &str| {
        strict_manifest(node, role, epoch, 200, transport, &plugin)
            .replace("interval_ms = 100", "interval_ms = 10")
            .replace("stale_after_ms = 800", "stale_after_ms = 100")
    };
    let server = short("station", "server", &format!("kind=\"zenoh\"\nlisten=[\"tcp/127.0.0.1:{port}\"]"));
    let client = short("uav1", "client", &format!("kind=\"zenoh\"\nconnect=[\"tcp/127.0.0.1:{port}\"]"));
    let a = host_process(&dir, "server.toml", &server, &bin);
    let b = host_process(&dir, "client.toml", &client, &bin);
    let (a_status, a) = output(a);
    let (b_status, b) = output(b);
    assert!(a_status.success(), "reference: {a}");
    assert!(b_status.success(), "10 ms probes / 100 ms TTL cannot establish or retain admission: {b}");
    assert_eq!(a["e0_ns"], b["e0_ns"]);
    assert!(b["plugins"][0]["steps"].as_u64().unwrap() > 0);
    assert!(b["plugins"][0]["published"].as_u64().unwrap() > 0);
    let log = std::fs::read_to_string(dir.join("audit/uav1/clock.jsonl")).unwrap();
    assert!(log.lines().filter_map(|v| serde_json::from_str::<serde_json::Value>(v).ok())
        .any(|v| v["accepted"] == true && v["estimate"]["samples"].as_u64().is_some_and(|n| n >= 3)));
    let health = std::fs::read_to_string(dir.join("audit/uav1/health.jsonl")).unwrap();
    assert!(health.contains("clock_gate_passed"));
}
