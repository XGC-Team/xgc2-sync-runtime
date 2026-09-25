//! Z2d: four hosts exchange DMPC-sized plans every round through
//! NeighborExchange over a lossy bus. What each planner was told about each
//! neighbor must agree with the audit: every Stale/Missing neighbor at
//! round k is a plan(j, k − 1) the audit shows was never received.

mod common;

use std::collections::{HashMap, HashSet};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_audit::record::{Kind, Record, RECORD_LEN};
use xgc_rt_audit::{merge_run, FileAudit, MergeOptions, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{Impairment, LoopbackBus, LoopbackTransport};

const NODES: [&str; 4] = ["uav1", "uav2", "uav3", "uav4"];

fn manifest(node: &str, lib: &str, e0: i64) -> String {
    let others: Vec<String> = NODES.iter().filter(|n| **n != node).map(|n| format!("\"{n}\"")).collect();
    format!(
        r#"
[session]
id = "nx"
node = "{node}"
roster = ["uav1", "uav2", "uav3", "uav4", "obs"]
period_ms = 50
epoch_ns = {e0}
run_for_ms = 3000

[transport]
kind = "loopback"

[audit]
dir = "audit"

[[channel]]
name = "dmpc/plan"
qos = "control"
[[channel]]
name = "dmpc/completeness"
qos = "state"

[[plugin]]
name = "planner"
path = "{lib}"
trigger = "both"
bind = {{ plan_in = {{ channel = "dmpc/plan", from = [{}] }}, plan_out = {{ channel = "dmpc/plan" }}, completeness = {{ channel = "dmpc/completeness" }} }}
"#,
        others.join(", ")
    )
}

#[test]
fn neighbor_statuses_agree_with_the_audit_under_loss() {
    let dir = common::scratch("neighbor-exchange");
    let lib = common::lib("dmpc_exchange_demo");
    let bus = LoopbackBus::with_impairment(Impairment { drop: 0.10, duplicate: 0.0, reorder: 0.0, seed: 0xD3C });
    let clock = Arc::new(WallClock::new(0));
    // Like a Session: E0 far enough ahead that every host is up before it
    // (Host::new hashes each plugin; unstripped debug builds take ~0.3 s).
    let e0 = clock.now() + 3_000_000_000;

    // Observer collects every node's completeness records.
    let obs_audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(), session: "nx".into(), node: "obs".into(), node_id: 4,
                roster: ["uav1", "uav2", "uav3", "uav4", "obs"].iter().map(|s| s.to_string()).collect(),
                channels: vec!["dmpc/plan".into(), "dmpc/completeness".into()], clock_domain: "wall".into(),
                audit_queue_drops: 0, records_written: 0, complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let obs = Endpoint::open(
        Box::new(LoopbackTransport::new(bus.clone())),
        &TransportContext {
            session: "nx".into(), node: "obs".into(), node_id: 4,
            roster: ["uav1", "uav2", "uav3", "uav4", "obs"].iter().map(|s| s.to_string()).collect(),
            channels: vec![
                ChannelSpec { id: 0, name: "dmpc/plan".into(), qos: Qos::Control },
                ChannelSpec { id: 1, name: "dmpc/completeness".into(), qos: Qos::State },
            ],
        },
        clock.clone(),
        obs_audit.clone(),
        1 << 16,
    )
    .unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let mut runners = Vec::new();
    for node in NODES {
        let host = Host::new(
            Manifest::from_toml_str(&manifest(node, &lib, e0)).unwrap(),
            &dir,
            Box::new(LoopbackTransport::new(bus.clone())),
            clock.clone(),
            HostOptions::default(),
        )
        .unwrap();
        let stop = stop.clone();
        runners.push(std::thread::spawn(move || host.run(&stop).unwrap()));
    }
    // The observer subscribes only to completeness; the loopback injector
    // applies to it too, so tolerate missing completeness records.
    obs.declare_in(1, &[0, 1, 2, 3]).unwrap();
    let summaries: Vec<_> = runners.into_iter().map(|r| r.join().unwrap()).collect();
    std::thread::sleep(Duration::from_millis(50));
    let frames = obs.drain();
    obs.close();
    obs_audit.finish().unwrap();
    let _ = stop.load(Ordering::Relaxed);

    let report = merge_run(&dir.join("audit"), MergeOptions::default()).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);

    // Received plans per (receiver, origin): set of rounds, from the audit.
    let mut seq_round: HashMap<(u16, u64), u64> = HashMap::new();
    let mut received: HashMap<(u16, u16), HashSet<u64>> = HashMap::new();
    for (i, node) in NODES.iter().enumerate() {
        let bytes = std::fs::read(dir.join("audit").join(node).join("records.bin")).unwrap();
        for c in bytes.chunks_exact(RECORD_LEN) {
            let r = Record::decode(c).unwrap();
            if r.channel != 0 {
                continue;
            }
            match r.kind {
                Kind::Tx => {
                    seq_round.insert((r.origin, r.seq), r.round);
                }
                Kind::Rx => {
                    received.entry((i as u16, r.origin)).or_default().insert(r.round);
                }
                _ => {}
            }
        }
    }
    let _ = seq_round;

    let (mut fresh, mut not_fresh, mut checked) = (0u64, 0u64, 0u64);
    for f in &frames {
        let p = &f.payload;
        let receiver = f.header.origin;
        let k = u64::from_le_bytes(p[0..8].try_into().unwrap());
        let n = u32::from_le_bytes(p[8..12].try_into().unwrap()) as usize;
        assert_eq!(n, 3, "each node has three neighbors");
        if k < 2 {
            continue; // before every neighbor could have published round k - 1
        }
        for e in 0..n {
            let o = 16 + 16 * e;
            let origin = u16::from_le_bytes(p[o..o + 2].try_into().unwrap());
            let status = p[o + 2];
            let got = received.get(&(receiver, origin));
            let had_prev = got.is_some_and(|s| s.contains(&(k - 1)));
            checked += 1;
            if status == 0 {
                fresh += 1;
            } else {
                not_fresh += 1;
                assert!(!had_prev, "node {receiver} marked neighbor {origin} not fresh at round {k}, but the audit shows plan(k-1) received");
            }
        }
    }
    println!(
        "checked {checked} neighbor-rounds from {} completeness records: fresh {fresh}, stale/missing {not_fresh}; plan streams: {}",
        frames.len(),
        report.streams.iter().filter(|s| s.channel == "dmpc/plan").map(|s| format!("{}→{} lost {}/{}", s.origin, s.receiver, s.counts.lost, s.counts.expected)).collect::<Vec<_>>().join(", ")
    );
    assert!(checked > 600, "every node ran every round from E0");
    assert!(not_fresh > 0, "10 % loss must produce some non-fresh neighbors");
    for s in &summaries {
        assert!(s.plugins[0].last_error.is_none(), "{:?}", s.plugins[0]);
    }
}
