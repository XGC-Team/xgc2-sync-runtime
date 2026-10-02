//! M-wrap-1: the ROS package hover_thrust_estimator's own runtime, wrapped
//! as a plugin. The outputs it publishes through the host must equal,
//! bit for bit, the runtime driven directly (the reference program replays
//! the same samples the way the ROS input producer does). The /1 and optional /2
//! ports must feed that same runtime, including thrust-mask rejection/recovery.

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
const DEFAULT_INITIAL: f64 = 0.3;
const IGNORE_THRUST: u32 = 64;
const IGNORE_ATTITUDE: u32 = 128;
const IGNORE_RATES: u32 = 7;
const THRUST_INVALID: u32 = 1 << 7; // original runtime's kThrustInvalid
const CHANNELS: [&str; 5] = ["imu", "attitude_target", "pose", "hover_thrust", "attitude_target_full"];

#[derive(Clone, Copy)]
struct Sample {
    port: u32,
    stamp: f64,
    value: f64,
    ignore_thrust: bool,
}

#[derive(Clone, Copy, Debug)]
enum TargetEncoding {
    Legacy,
    Full { mask: u32, ignored_payload: bool },
}

impl TargetEncoding {
    fn channel(self) -> u32 {
        match self {
            Self::Legacy => 1,
            Self::Full { .. } => 4,
        }
    }

    fn binding(self) -> &'static str {
        match self {
            Self::Legacy => r#"attitude_target = { channel = "attitude_target", from = ["feeder"] }"#,
            Self::Full { .. } => r#"attitude_target_full = { channel = "attitude_target_full", from = ["feeder"] }"#,
        }
    }
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
            out.push(Sample { port: 0, stamp: t, value: G / HOVER * thrust(t) + noise(), ignore_thrust: false });
        }
    }
    for i in 1..=1000 {
        let t = 1.0 + i as f64 * 0.01 + 0.0025;
        out.push(Sample { port: 1, stamp: t, value: thrust(t), ignore_thrust: false });
    }
    for i in 1..=500 {
        let t = 1.0 + i as f64 * 0.02 + 0.001;
        out.push(Sample { port: 2, stamp: t, value: if t < 3.0 { 0.0 } else { 1.5 }, ignore_thrust: false });
    }
    out.sort_by(|a, b| a.stamp.partial_cmp(&b.stamp).unwrap().then(a.port.cmp(&b.port)));
    out
}

fn payload(s: &Sample, encoding: TargetEncoding) -> Vec<u8> {
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
            // Both layouts retain normalized thrust verbatim. In /2, ignored
            // q/rate bytes deliberately contain values no estimator should use.
            let q = match encoding {
                TargetEncoding::Full { ignored_payload: true, .. } => [f64::NAN, f64::INFINITY, -2.0, 0.1],
                _ => [1.0, 0.0, 0.0, 0.0],
            };
            f(&mut b, s.stamp);
            for v in q {
                f(&mut b, v);
            }
            let mask = match encoding {
                TargetEncoding::Legacy => u32::from(s.ignore_thrust),
                TargetEncoding::Full { mask, ignored_payload } => {
                    let rates = if ignored_payload && mask & IGNORE_RATES == IGNORE_RATES {
                        [f64::NAN, f64::INFINITY, f64::NEG_INFINITY]
                    } else if ignored_payload {
                        // mask=128 leaves rates commanded; HTE still only
                        // observes thrust, so differing finite rates are inert.
                        [-130.0, 77.0, s.stamp]
                    } else {
                        [0.0; 3]
                    };
                    for v in rates {
                        f(&mut b, v);
                    }
                    mask | if s.ignore_thrust { IGNORE_THRUST } else { 0 }
                }
            };
            f(&mut b, s.value);
            b.extend_from_slice(&mask.to_le_bytes());
            b.extend_from_slice(&0u32.to_le_bytes());
            assert_eq!(b.len(), if encoding.channel() == 1 { 56 } else { 80 });
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
    initial: u64,
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
    Estimate { stamp: f(0), hover: f(8), raw: f(16), initial: f(24), thr2acc: f(32), last_estimate_stamp: f(40), state: u(48), flags: u(52), sample_used: u(56) }
}

fn reference(bin: &std::path::Path, samples: &[Sample]) -> Vec<Estimate> {
    let mut child = Command::new(bin).stdin(Stdio::piped()).stdout(Stdio::piped()).spawn().unwrap();
    {
        let mut stdin = child.stdin.take().unwrap();
        for s in samples {
            writeln!(stdin, "{} {:?} {:?} {}", s.port, s.stamp, s.value, u32::from(s.ignore_thrust)).unwrap();
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
            // The direct runtime's text stream does not serialize config;
            // published initial is independently checked against its .3 default.
            Estimate { stamp: d(0), hover: d(1), raw: d(2), initial: DEFAULT_INITIAL.to_bits(), thr2acc: d(3), last_estimate_stamp: d(4), state: n(5), flags: n(6), sample_used: n(7) }
        })
        .collect()
}

fn run_host(label: &str, samples: &[Sample], encoding: TargetEncoding) -> Vec<Estimate> {
    let (lib, _) = common::est_hover_thrust();
    let dir = common::scratch(label);
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
[[channel]]
name = "attitude_target_full"
qos = "state"

[[plugin]]
name = "hover-thrust"
path = "{}"
trigger = "both"
config = {{ time_source = "input" }}
bind = {{ imu = {{ channel = "imu", from = ["feeder"] }}, {}, pose = {{ channel = "pose", from = ["feeder"] }}, hover_thrust = {{ channel = "hover_thrust" }} }}
"#,
        lib.display(), encoding.binding()
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
    for ch in [0, 2, encoding.channel()] {
        feeder.declare_out(ch).unwrap();
    }
    feeder.declare_in(3, &[0]).unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_millis(150)); // past E0
    for (i, s) in samples.iter().enumerate() {
        let channel = if s.port == 1 { encoding.channel() } else { s.port };
        feeder.publish(channel, 0, clock.now(), &payload(s, encoding)).unwrap();
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

    let plugin = &summary.plugins[0];
    println!(
        "{label}: plugin {} state={} domain={} steps={} consumed={} published={} estimates={}",
        plugin.library, plugin.state, plugin.domain_state, plugin.steps, plugin.consumed, plugin.published, got.len()
    );
    assert_eq!(plugin.consumed as usize, samples.len(), "every sample reached the runtime");
    assert_eq!(plugin.domain_state, "airborne");
    assert!(got.iter().all(|e| e.initial == DEFAULT_INITIAL.to_bits()), "published default initial must remain .3");

    let report = merge_run(&dir.join("audit"), MergeOptions::default()).unwrap();
    assert!(report.valid, "{label}: {:?}", report.invalid_reasons);
    assert!(report.streams.iter().all(|s| s.counts.lost == 0), "{label}: lost samples");
    println!("{label}: audit valid={} no_loss={}", report.valid, report.streams.iter().all(|s| s.counts.lost == 0));
    got
}

fn assert_estimates_equal(label: &str, got: &[Estimate], want: &[Estimate]) {
    assert_eq!(got.len(), want.len(), "{label}: estimate count");
    for (i, (g, w)) in got.iter().zip(want).enumerate() {
        assert_eq!(g, w, "{label}: estimate {i} differs");
    }
}

#[test]
fn wrapped_runtime_matches_the_direct_runtime_bit_for_bit() {
    // Keep this as one gate: check-native-gates counts one HTE equivalence test.
    let (_, reference_bin) = common::est_hover_thrust();
    let samples = flight();
    let want = reference(reference_bin, &samples);
    let legacy = run_host("est-hover-thrust-v1", &samples, TargetEncoding::Legacy);
    assert!(want.len() > 500, "reference produced {} estimates", want.len());
    assert_estimates_equal("/1 vs direct runtime", &legacy, &want);
    let first = legacy.first().unwrap();
    assert_eq!(first.hover, DEFAULT_INITIAL.to_bits(), "actual default initial output");
    assert_eq!(first.raw, DEFAULT_INITIAL.to_bits());
    assert_eq!(first.thr2acc, (G / DEFAULT_INITIAL).to_bits());

    for (label, mask, ignored_payload) in [
        ("est-hover-thrust-v2", 0, false),
        ("est-hover-thrust-v2-mask128", IGNORE_ATTITUDE, true),
        ("est-hover-thrust-v2-mask135", IGNORE_ATTITUDE | IGNORE_RATES, true),
    ] {
        let full = run_host(label, &samples, TargetEncoding::Full { mask, ignored_payload });
        assert_estimates_equal(label, &full, &legacy);
        assert_estimates_equal(label, &full, &want);
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
    println!("final hover thrust {hover:.8} (true {HOVER}), state {}", last.state);
    assert!((hover - HOVER).abs() < 0.01, "hover estimate {hover}");

    // IGNORE_THRUST alone must prevent raw-estimator updates even with fresh,
    // finite, in-range but misleading thrust. Unmasking restores updates.
    // SelfCheck may filter hover back toward .3; raw and last-estimate stamp
    // must stay fixed. This distinguishes rejection from merely slow filtering.
    let mut ignored = samples;
    for s in &mut ignored {
        if s.port == 1 && (4.0..6.0).contains(&s.stamp) {
            s.ignore_thrust = true;
            s.value = 0.81;
        }
    }
    let ignored_want = reference(reference_bin, &ignored);
    let ignored_legacy = run_host("est-hover-thrust-ignore-v1", &ignored, TargetEncoding::Legacy);
    let ignored_full = run_host("est-hover-thrust-ignore-v2-mask64", &ignored,
        TargetEncoding::Full { mask: 0, ignored_payload: false });
    let ignored_full_rates = run_host("est-hover-thrust-ignore-v2-mask192", &ignored,
        TargetEncoding::Full { mask: IGNORE_ATTITUDE, ignored_payload: true });
    assert_estimates_equal("ignored /1 vs direct runtime", &ignored_legacy, &ignored_want);
    assert_estimates_equal("ignored /2 vs /1", &ignored_full, &ignored_legacy);
    assert_estimates_equal("ignored /2 vs direct runtime", &ignored_full, &ignored_want);
    assert_estimates_equal("ignored /2 mask192 vs mask64", &ignored_full_rates, &ignored_full);
    let before_ignore = ignored_full.iter().rev()
        .find(|e| f64::from_bits(e.stamp) < 4.0025).unwrap();
    let rejected: Vec<_> = ignored_full.iter()
        .filter(|e| (4.02..5.98).contains(&f64::from_bits(e.stamp))).collect();
    assert!(rejected.len() > 100, "must observe a sustained ignored-thrust interval");
    for e in &rejected {
        assert_eq!(e.state, 10, "ignored thrust forces original SelfCheck");
        assert_ne!(e.flags & THRUST_INVALID, 0, "original thrust-invalid flag");
        assert_eq!(e.sample_used, 0, "ignored thrust cannot update the estimator");
        assert_eq!(e.raw, before_ignore.raw, "raw estimate changed under IGNORE_THRUST");
        assert_eq!(e.last_estimate_stamp, before_ignore.last_estimate_stamp,
                   "last successful estimate stamp changed under IGNORE_THRUST");
    }
    let recovered: Vec<_> = ignored_full.iter()
        .filter(|e| (6.02..6.95).contains(&f64::from_bits(e.stamp))).collect();
    assert!(recovered.iter().all(|e| e.state == 12 && e.flags & THRUST_INVALID == 0),
            "recovery must remain Airborne with thrust accepted");
    assert!(recovered.iter().any(|e| e.sample_used == 1 && f64::from_bits(e.last_estimate_stamp) > 6.0),
        "unmasking must recover Airborne and successful estimator updates");
    assert!(recovered.iter().any(|e| e.raw != before_ignore.raw),
            "recovery must actually change the raw estimate");
    let mut update_stamps: Vec<_> = recovered.iter().map(|e| e.last_estimate_stamp).collect();
    update_stamps.dedup();
    assert!(update_stamps.len() >= 5, "recovery must sustain raw updates at the unchanged 10 Hz gate");
    let final_hover = f64::from_bits(ignored_full.last().unwrap().hover);
    assert!((final_hover - HOVER).abs() < 0.01, "recovered convergence {final_hover}");
    println!("default initial={DEFAULT_INITIAL}; /1 and /2 estimates bit-identical, mask128/135 payload inert; IGNORE_THRUST mask64/192 rejected={} recovered_update_stamps={} frozen_raw={} frozen_stamp={}; final={final_hover:.8} true={HOVER}",
        rejected.len(), update_stamps.len(),
        f64::from_bits(before_ignore.raw), f64::from_bits(before_ignore.last_estimate_stamp));
}
