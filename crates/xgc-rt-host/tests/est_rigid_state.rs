//! Phase 2: the rigid-state ESKF (ros1/perception/estimator/rigid-state)
//! wrapped as a plugin. Its state and vision-pose outputs through the
//! aggregator must equal, bit for bit, the runtime driven directly the way
//! the ROS node drives it.

mod common;

use std::io::Write;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const G: f64 = 9.8066;
const CHANNELS: [&str; 4] = ["imu", "pose", "rigid_state", "vision_pose"];

/// One input sample: port 0 imu [stamp, a xyz, w xyz], port 1 pose
/// [stamp, p xyz, q wxyz].
struct Sample {
    port: u32,
    values: Vec<f64>,
}

/// 8 s on a 0.5 m circle at 0.8 rad/s with a slow altitude wave, level
/// attitude. IMU 200 Hz with the exact specific force (+g on z); VRPN pose
/// 100 Hz, silent for 5.0–5.3 s (beyond the 0.12 s timeout, within the
/// 0.5 s coasting limit).
fn flight() -> Vec<Sample> {
    let (r, w) = (0.5, 0.8);
    let mut out: Vec<(f64, Sample)> = Vec::new();
    for i in 1..=1600 {
        let t = 1.0 + i as f64 * 0.005;
        let (ax, ay, az) = (-r * w * w * (w * t).sin(), -r * w * w * (w * t).cos(), -0.2 * 0.09 * (0.3 * t).sin());
        out.push((t, Sample { port: 0, values: vec![t, ax, ay, az + G, 0.0, 0.0, 0.0] }));
    }
    for i in 1..=800 {
        let t = 1.0 + i as f64 * 0.01 + 0.001;
        if (5.0..5.3).contains(&t) {
            continue;
        }
        let p = [r * (w * t).sin(), r * (w * t).cos() - r, 1.0 + 0.2 * (0.3 * t).sin()];
        out.push((t, Sample { port: 1, values: vec![t, p[0], p[1], p[2], 1.0, 0.0, 0.0, 0.0] }));
    }
    out.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap().then(a.1.port.cmp(&b.1.port)));
    out.into_iter().map(|(_, s)| s).collect()
}

fn payload(s: &Sample) -> Vec<u8> {
    s.values.iter().flat_map(|v| v.to_le_bytes()).collect()
}

/// Bits of every f64 in an output payload.
fn bits(p: &[u8]) -> Vec<u64> {
    p.chunks_exact(8).map(|c| u64::from_le_bytes(c.try_into().unwrap())).collect()
}

/// (state lines, vision-pose lines) from the reference program, as bits.
fn reference(bin: &std::path::Path, samples: &[Sample]) -> (Vec<Vec<u64>>, Vec<Vec<u64>>) {
    let mut child = Command::new(bin).stdin(Stdio::piped()).stdout(Stdio::piped()).spawn().unwrap();
    // Feed stdin from another thread: the reference writes more than a pipe
    // buffer before it has read everything.
    let mut text = String::new();
    for s in samples {
        text += &s.port.to_string();
        for v in &s.values {
            text += &format!(" {:016x}", v.to_bits());
        }
        text.push('\n');
    }
    let mut stdin = child.stdin.take().unwrap();
    let writer = std::thread::spawn(move || stdin.write_all(text.as_bytes()).unwrap());
    let out = child.wait_with_output().unwrap();
    writer.join().unwrap();
    assert!(out.status.success());
    let (mut state, mut vision) = (Vec::new(), Vec::new());
    for line in String::from_utf8(out.stdout).unwrap().lines() {
        let mut f = line.split(' ');
        let kind = f.next().unwrap();
        let v: Vec<u64> = f.map(|x| u64::from_str_radix(x, 16).unwrap()).collect();
        if kind == "S" { state.push(v) } else { vision.push(v) }
    }
    (state, vision)
}

#[test]
fn wrapped_eskf_matches_the_direct_runtime_bit_for_bit() {
    let (lib, reference_bin) = common::est_rigid_state();
    let dir = common::scratch("est-rigid-state");
    let manifest = format!(
        r#"
[session]
id = "eskf"
node = "uav1"
roster = ["uav1", "feeder"]
period_ms = 10
start_delay_ms = 50

[transport]
kind = "loopback"

[audit]
dir = "audit"

[[channel]]
name = "imu"
qos = "state"
[[channel]]
name = "pose"
qos = "state"
[[channel]]
name = "rigid_state"
qos = "state"
[[channel]]
name = "vision_pose"
qos = "state"

[[plugin]]
name = "rigid-state"
path = "{}"
trigger = "both"
step_budget_ms = 1000
config = {{ time_source = "input", extrinsic_verified = true }}
bind = {{ imu = {{ channel = "imu", from = ["feeder"] }}, pose = {{ channel = "pose", from = ["feeder"] }}, rigid_state = {{ channel = "rigid_state" }}, vision_pose = {{ channel = "vision_pose" }} }}
"#,
        lib.display()
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(WallClock::new(0));
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();

    let meta = NodeMeta {
        format: String::new(),
        session: "eskf".into(),
        node: "feeder".into(),
        node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: CHANNELS.iter().map(|s| s.to_string()).collect(),
        clock_domain: "wall".into(),
        audit_queue_drops: 0,
        records_written: 0,
        complete: false,
    };
    let feeder_audit = Arc::new(FileAudit::create(&dir.join("audit"), meta, clock.clone()).unwrap());
    let ctx = TransportContext {
        session: "eskf".into(),
        node: "feeder".into(),
        node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: CHANNELS.iter().enumerate().map(|(i, n)| ChannelSpec { id: i as u32, name: n.to_string(), qos: Qos::State }).collect(),
    };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), feeder_audit.clone(), 1 << 16).unwrap();
    feeder.declare_out(0).unwrap();
    feeder.declare_out(1).unwrap();
    feeder.declare_in(2, &[0]).unwrap();
    feeder.declare_in(3, &[0]).unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_millis(150)); // past E0
    let samples = flight();
    for (i, s) in samples.iter().enumerate() {
        feeder.publish(s.port, 0, clock.now(), &payload(s)).unwrap();
        if i % 20 == 19 {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    std::thread::sleep(Duration::from_millis(500));
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    let (mut state, mut vision) = (Vec::new(), Vec::new());
    for f in feeder.drain() {
        if f.header.channel == 2 { state.push(bits(&f.payload)) } else { vision.push(bits(&f.payload)) }
    }
    feeder.close();
    feeder_audit.finish().unwrap();

    let (want_state, want_vision) = reference(reference_bin, &samples);
    let plugin = &summary.plugins[0];
    println!(
        "plugin {} state={} domain={} steps={} consumed={} published={}; reference {} states, {} vision poses",
        plugin.library, plugin.state, plugin.domain_state, plugin.steps, plugin.consumed, plugin.published, want_state.len(), want_vision.len()
    );
    assert_eq!(plugin.consumed as usize, samples.len(), "every sample reached the runtime");
    assert!(want_state.len() > 700 && want_vision.len() > 100, "the reference ran and fused poses");
    assert_eq!(state.len(), want_state.len(), "state count");
    assert_eq!(vision.len(), want_vision.len(), "vision pose count");
    for (i, (g, w)) in state.iter().zip(&want_state).enumerate() {
        assert_eq!(g, w, "state {i} differs");
    }
    for (i, (g, w)) in vision.iter().zip(&want_vision).enumerate() {
        assert_eq!(g, w, "vision pose {i} differs");
    }
    assert_eq!(plugin.domain_state, "running");

    // The estimate tracks the circle: last state within 5 cm of the truth.
    let last = want_state.last().unwrap();
    let f = |i: usize| f64::from_bits(last[i]);
    let t = f(0);
    let truth = [0.5 * (0.8 * t).sin(), 0.5 * (0.8 * t).cos() - 0.5, 1.0 + 0.2 * (0.3 * t).sin()];
    let err = ((f(1) - truth[0]).powi(2) + (f(2) - truth[1]).powi(2) + (f(3) - truth[2]).powi(2)).sqrt();
    println!("final position error {:.4} m at t = {t:.3}", err);
    assert!(err < 0.05, "position error {err}");
}
