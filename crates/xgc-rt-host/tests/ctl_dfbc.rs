//! M-wrap-1b: xgc2_math's DFBC geometric controller as a plugin. Commands
//! published through the host equal the controller driven directly, bit
//! for bit; on-reference hover gives thrust = g and identity attitude.

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

const CHANNELS: [&str; 3] = ["state", "ref", "cmd"];

enum Item {
    State([f64; 14]),
    Ref([f64; 19]),
}

impl Item {
    fn stamp(&self) -> f64 {
        match self {
            Item::State(v) => v[0],
            Item::Ref(v) => v[0],
        }
    }
}

/// 1 s hover exactly on the reference, then a 1 m circle at 0.5 rad/s at
/// 1.5 m with a decaying 0.2 m state offset. Ref 50 Hz, state 100 Hz.
fn script() -> Vec<Item> {
    let w = 0.5;
    let circle = |t: f64| -> [f64; 19] {
        let tau = (t - 2.0).max(0.0);
        let on = if t < 2.0 { 0.0 } else { 1.0 };
        let (s, c) = ((w * tau).sin(), (w * tau).cos());
        [
            t,
            on * s, on * (1.0 - c), 1.5,
            on * w * c, on * w * s, 0.0,
            -on * w * w * s, on * w * w * c, 0.0,
            -on * w.powi(3) * c, -on * w.powi(3) * s, 0.0,
            on * w.powi(4) * s, -on * w.powi(4) * c, 0.0,
            0.0, 0.0, 0.0,
        ]
    };
    let mut items = Vec::new();
    for i in 0..=250 {
        items.push(Item::Ref(circle(1.0 + i as f64 * 0.02)));
    }
    for i in 1..=500 {
        let t = 1.0 + i as f64 * 0.01;
        let r = circle(t);
        let off = if t < 2.0 { 0.0 } else { 0.2 * (-(t - 2.0)).exp() };
        items.push(Item::State([t, r[1] + off, r[2], r[3] - off, r[4], r[5], r[6], 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]));
    }
    // The plugin's apply order: by stamp, a reference before a state with
    // the same stamp.
    items.sort_by(|a, b| {
        a.stamp().partial_cmp(&b.stamp()).unwrap().then((matches!(a, Item::State(_))).cmp(&matches!(b, Item::State(_))))
    });
    items
}

fn bytes(v: &[f64], tail: usize) -> Vec<u8> {
    let mut b: Vec<u8> = v.iter().flat_map(|x| x.to_le_bytes()).collect();
    b.extend(std::iter::repeat(0u8).take(tail));
    b
}

fn cmd_fields(p: &[u8]) -> Vec<u64> {
    assert_eq!(p.len(), 104);
    let mut v: Vec<u64> = (0..12).map(|i| u64::from_le_bytes(p[i * 8..i * 8 + 8].try_into().unwrap())).collect();
    v.push(u32::from_le_bytes(p[96..100].try_into().unwrap()) as u64);
    v.push(u32::from_le_bytes(p[100..104].try_into().unwrap()) as u64);
    v
}

#[test]
fn wrapped_dfbc_matches_the_direct_controller_bit_for_bit() {
    let (lib, reference_bin) = common::ctl_dfbc();
    let dir = common::scratch("ctl-dfbc");
    let manifest = format!(
        r#"
[session]
id = "dfbc"
node = "uav1"
roster = ["uav1", "feeder"]
period_ms = 10
start_delay_ms = 50

[transport]
kind = "loopback"

[audit]
dir = "audit"

[[channel]]
name = "state"
qos = "state"
[[channel]]
name = "ref"
qos = "control"
[[channel]]
name = "cmd"
qos = "control"

[[plugin]]
name = "dfbc"
path = "{}"
trigger = "on_dirty"
bind = {{ state = {{ channel = "state", from = ["feeder"] }}, ref = {{ channel = "ref", from = ["feeder"] }}, cmd = {{ channel = "cmd" }} }}
"#,
        lib.display()
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(WallClock::new(0));
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(),
                session: "dfbc".into(),
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
        session: "dfbc".into(),
        node: "feeder".into(),
        node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: vec![
            ChannelSpec { id: 0, name: "state".into(), qos: Qos::State },
            ChannelSpec { id: 1, name: "ref".into(), qos: Qos::Control },
            ChannelSpec { id: 2, name: "cmd".into(), qos: Qos::Control },
        ],
    };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 16).unwrap();
    feeder.declare_out(0).unwrap();
    feeder.declare_out(1).unwrap();
    feeder.declare_in(2, &[0]).unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_millis(150));
    let items = script();
    for (i, it) in items.iter().enumerate() {
        match it {
            Item::State(v) => feeder.publish(0, 0, clock.now(), &bytes(v, 0)).unwrap(),
            Item::Ref(v) => feeder.publish(1, 0, clock.now(), &bytes(v, 8)).unwrap(),
        };
        if i % 10 == 9 {
            std::thread::sleep(Duration::from_millis(1));
        }
    }
    std::thread::sleep(Duration::from_millis(200));
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    let got: Vec<Vec<u64>> = feeder.drain().iter().map(|f| cmd_fields(&f.payload)).collect();
    feeder.close();
    audit.finish().unwrap();

    let mut child = Command::new(reference_bin).stdin(Stdio::piped()).stdout(Stdio::piped()).spawn().unwrap();
    {
        let mut stdin = child.stdin.take().unwrap();
        for it in &items {
            let (tag, v): (&str, &[f64]) = match it {
                Item::State(v) => ("S", v),
                Item::Ref(v) => ("R", v),
            };
            let line: Vec<String> = v.iter().map(|x| format!("{x:?}")).collect();
            writeln!(stdin, "{tag} {}", line.join(" ")).unwrap();
        }
    }
    let out = child.wait_with_output().unwrap();
    assert!(out.status.success());
    let want: Vec<Vec<u64>> = String::from_utf8(out.stdout)
        .unwrap()
        .lines()
        .map(|l| {
            let f: Vec<&str> = l.split(' ').collect();
            let mut v: Vec<u64> = f[..12].iter().map(|x| x.parse::<f64>().unwrap().to_bits()).collect();
            v.push(f[12].parse().unwrap());
            v.push(f[13].parse().unwrap());
            v
        })
        .collect();

    let p = &summary.plugins[0];
    println!("plugin {} domain={} steps={} consumed={} published={}; reference {}", p.library, p.domain_state, p.steps, p.consumed, p.published, want.len());
    assert_eq!(p.consumed as usize, items.len());
    assert_eq!(want.len(), 500, "one command per state sample");
    assert_eq!(got, want, "commands must be bit-identical");
    assert_eq!(p.domain_state, "tracking");

    // Sanity on the hover second: thrust = g, attitude identity, no error.
    let hover = &want[50];
    let f = |i: usize| f64::from_bits(hover[i]);
    println!("hover: thrust {:.6} q ({:.6},{:.6},{:.6},{:.6}) perr ({:.2e},{:.2e},{:.2e})", f(1), f(2), f(3), f(4), f(5), f(9), f(10), f(11));
    assert!((f(1) - 9.8066).abs() < 1e-9);
    assert!((f(2) - 1.0).abs() < 1e-9 && f(3).abs() < 1e-9 && f(4).abs() < 1e-9);
    assert!(f(9).abs() + f(10).abs() + f(11).abs() < 1e-12);
    assert_eq!(hover[12], 1, "success");
}
