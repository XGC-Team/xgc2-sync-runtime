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
