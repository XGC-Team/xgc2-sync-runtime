//! M-wrap-1: the ROS package hover_thrust_estimator's own runtime, wrapped
//! as a plugin. The outputs it publishes through the host must equal,
//! bit for bit, the runtime driven directly (the reference program replays
//! the same samples the way the ROS input producer does).

mod common;

use std::io::Write;
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_audit::{merge_run, FileAudit, MergeOptions, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const G: f64 = 9.8066;
const HOVER: f64 = 0.42;
const CHANNELS: [&str; 4] = ["imu", "attitude_target", "pose", "hover_thrust"];

#[derive(Clone, Copy)]
struct Sample {
    port: u32,
    stamp: f64,
    value: f64,
}

/// 10 s: IMU 200 Hz, thrust 100 Hz, pose 50 Hz. On the ground (z = 0)
/// until t = 2 s, then at 1.5 m. IMU silent for 6.0–6.5 s (beyond the
/// 0.2 s sample timeout).
fn flight() -> Vec<Sample> {
    let mut rng = 0x2545_f491_4f6c_dd1du64;
    let mut noise = || {
        rng ^= rng << 13;
        rng ^= rng >> 7;
        rng ^= rng << 17;
        (rng >> 11) as f64 / (1u64 << 53) as f64 * 0.1 - 0.05
    };
    let thrust = |t: f64| HOVER + 0.03 * (std::f64::consts::TAU * 0.5 * t).sin();
    let mut out = Vec::new();
    for i in 1..=2000 {
        let t = 1.0 + i as f64 * 0.005;
        if !(7.0..7.5).contains(&t) {
            out.push(Sample { port: 0, stamp: t, value: G / HOVER * thrust(t) + noise() });
        }
    }
    for i in 1..=1000 {
        let t = 1.0 + i as f64 * 0.01 + 0.0025;
        out.push(Sample { port: 1, stamp: t, value: thrust(t) });
    }
    for i in 1..=500 {
        let t = 1.0 + i as f64 * 0.02 + 0.001;
        out.push(Sample { port: 2, stamp: t, value: if t < 3.0 { 0.0 } else { 1.5 } });
    }
    out.sort_by(|a, b| a.stamp.partial_cmp(&b.stamp).unwrap().then(a.port.cmp(&b.port)));
    out
}

fn payload(s: &Sample) -> Vec<u8> {
    let mut b = Vec::new();
    let f = |b: &mut Vec<u8>, v: f64| b.extend_from_slice(&v.to_le_bytes());
    match s.port {
        0 => {
            // xgc_imu_v1: stamp, accel[3], gyro[3]
            for v in [s.stamp, 0.0, 0.0, s.value, 0.0, 0.0, 0.0] {
                f(&mut b, v);
            }
        }
        1 => {
            // xgc_attitude_target_v1: stamp, q[4], thrust, ignore u32, reserved u32
            for v in [s.stamp, 1.0, 0.0, 0.0, 0.0, s.value] {
                f(&mut b, v);
            }
            b.extend_from_slice(&[0u8; 8]);
        }
        _ => {
            // xgc_pose_v1: stamp, position[3], q[4]
            for v in [s.stamp, 0.0, 0.0, s.value, 1.0, 0.0, 0.0, 0.0] {
                f(&mut b, v);
            }
        }
    }
    b
}

#[derive(Debug, PartialEq)]
struct Estimate {
    stamp: u64,
    hover: u64,
    raw: u64,
    thr2acc: u64,
    last_estimate_stamp: u64,
    state: u32,
    flags: u32,
    sample_used: u32,
}

fn decode(p: &[u8]) -> Estimate {
    assert_eq!(p.len(), 64, "xgc_hover_thrust_v1 is 64 bytes");
    let f = |o: usize| u64::from_le_bytes(p[o..o + 8].try_into().unwrap());
    let u = |o: usize| u32::from_le_bytes(p[o..o + 4].try_into().unwrap());
    Estimate { stamp: f(0), hover: f(8), raw: f(16), thr2acc: f(32), last_estimate_stamp: f(40), state: u(48), flags: u(52), sample_used: u(56) }
}

fn reference(bin: &std::path::Path, samples: &[Sample]) -> Vec<Estimate> {
    let mut child = Command::new(bin).stdin(Stdio::piped()).stdout(Stdio::piped()).spawn().unwrap();
    {
        let mut stdin = child.stdin.take().unwrap();
        for s in samples {
            writeln!(stdin, "{} {:?} {:?} 0", s.port, s.stamp, s.value).unwrap();
        }
    }
    let out = child.wait_with_output().unwrap();
    assert!(out.status.success());
    String::from_utf8(out.stdout)
        .unwrap()
        .lines()
        .map(|line| {
            let v: Vec<&str> = line.split(' ').collect();
            let d = |i: usize| v[i].parse::<f64>().unwrap().to_bits();
            let n = |i: usize| v[i].parse::<u32>().unwrap();
            Estimate { stamp: d(0), hover: d(1), raw: d(2), thr2acc: d(3), last_estimate_stamp: d(4), state: n(5), flags: n(6), sample_used: n(7) }
        })
        .collect()
}

#[test]
fn wrapped_runtime_matches_the_direct_runtime_bit_for_bit() {
    let (lib, reference_bin) = common::est_hover_thrust();
    let dir = common::scratch("est-hover-thrust");
    let manifest = format!(
        r#"
[session]
id = "hte"
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
name = "attitude_target"
qos = "state"
[[channel]]
name = "pose"
qos = "state"
[[channel]]
name = "hover_thrust"
qos = "state"

[[plugin]]
name = "hover-thrust"
path = "{}"
trigger = "both"
config = {{ time_source = "input" }}
bind = {{ imu = {{ channel = "imu", from = ["feeder"] }}, attitude_target = {{ channel = "attitude_target", from = ["feeder"] }}, pose = {{ channel = "pose", from = ["feeder"] }}, hover_thrust = {{ channel = "hover_thrust" }} }}
"#,
        lib.display()
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(WallClock::new(0));
    let host = Host::new(
        Manifest::from_toml_str(&manifest).unwrap(),
        &dir,
        Box::new(LoopbackTransport::new(bus.clone())),
        clock.clone(),
        HostOptions::default(),
    )
    .unwrap();

    let feeder_audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(),
                session: "hte".into(),
                node: "feeder".into(),
                node_id: 1,
                roster: vec!["uav1".into(), "feeder".into()],
                channels: CHANNELS.iter().map(|s| s.to_string()).collect(),
                clock_domain: "wall".into(),
                audit_queue_drops: 0,
                records_written: 0,
                complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext {
        session: "hte".into(),
        node: "feeder".into(),
        node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: CHANNELS
            .iter()
            .enumerate()
            .map(|(i, n)| ChannelSpec { id: i as u32, name: n.to_string(), qos: Qos::State })
            .collect(),
    };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), feeder_audit.clone(), 1 << 16).unwrap();
    for ch in 0..3 {
        feeder.declare_out(ch).unwrap();
    }
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
    std::thread::sleep(Duration::from_millis(300));
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    let got: Vec<Estimate> = feeder.drain().iter().map(|f| decode(&f.payload)).collect();
    feeder.close();
    feeder_audit.finish().unwrap();

    let want = reference(reference_bin, &samples);
    let plugin = &summary.plugins[0];
    println!(
        "plugin {} state={} domain={} steps={} consumed={} published={}; reference estimates {}",
        plugin.library, plugin.state, plugin.domain_state, plugin.steps, plugin.consumed, plugin.published, want.len()
    );
    assert_eq!(plugin.consumed as usize, samples.len(), "every sample reached the runtime");
    assert!(want.len() > 500, "reference produced {} estimates", want.len());
    assert_eq!(got.len(), want.len(), "estimate count");
    for (i, (g, w)) in got.iter().zip(&want).enumerate() {
        assert_eq!(g, w, "estimate {i} differs");
    }

    // The domain FSM went Ground → Airborne, dropped to SelfCheck in the IMU
    // gap, and came back Airborne; the estimate converged.
    let states: Vec<u32> = want.iter().map(|e| e.state).collect();
    let first_airborne = states.iter().position(|&s| s == 12).expect("reached Airborne");
    assert!(states[..first_airborne].contains(&11), "was Ground before takeoff");
    let dropout = states[first_airborne..].iter().position(|&s| s == 10).expect("IMU gap forced SelfCheck") + first_airborne;
    assert!(states[dropout..].contains(&12), "recovered to Airborne");
    let last = want.last().unwrap();
    let hover = f64::from_bits(last.hover);
    println!("final hover thrust {hover:.4} (true {HOVER}), state {}", last.state);
    assert!((hover - HOVER).abs() < 0.01, "hover estimate {hover}");
    assert_eq!(plugin.domain_state, "airborne");

    let report = merge_run(&dir.join("audit"), MergeOptions::default()).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    assert!(report.streams.iter().all(|s| s.counts.lost == 0));
}
