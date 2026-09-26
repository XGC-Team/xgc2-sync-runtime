//! TRO DMPC Phase 1 without the planner: three robots, each an aggregator
//! running `dmpc-rounds`. A stand-in for each robot's `ros_io` plays the
//! unchanged planner node: it writes the node's own plan (and echoes the
//! neighbor plans it was given, as the shared ROS topic would) and reads the
//! sync triggers and neighbor plans the node would get.

mod common;

use std::sync::atomic::AtomicBool;
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const ROSTER: [&str; 6] = ["uav1", "uav2", "uav3", "rosio1", "rosio2", "rosio3"];
const CHANNELS: [&str; 4] = ["dmpc/plan", "ros/assumed_trajectories", "ros/neighbor_plans", "ros/sync_trigger"];
const PERIOD_MS: i64 = 50;

/// An AssumedTrajectory payload: 9 states × 51 steps, the round in states[0].
fn plan(uav: u32, round: f64) -> Vec<u8> {
    let mut p = Vec::new();
    p.extend_from_slice(&round.to_le_bytes());
    for v in [uav, 9, 51, 0, 1, 0] {
        p.extend_from_slice(&v.to_le_bytes());
    }
    p.extend_from_slice(&round.to_le_bytes());
    p.resize(32 + 9 * 51 * 8, 0);
    p
}

fn uav_of(p: &[u8]) -> u32 {
    u32::from_le_bytes(p[8..12].try_into().unwrap())
}

fn manifest(i: usize, e0: i64) -> String {
    let others: Vec<String> = (1..=3).filter(|j| *j != i).map(|j| format!("\"uav{j}\"")).collect();
    format!(
        r#"
[session]
id = "dmpc"
node = "uav{i}"
roster = {ROSTER:?}
period_ms = {PERIOD_MS}
epoch_ns = {e0}
run_for_ms = 1500

[transport]
kind = "loopback"

[audit]
dir = "audit"

[[channel]]
name = "dmpc/plan"
qos = "control"
[[channel]]
name = "ros/assumed_trajectories"
qos = "control"
[[channel]]
name = "ros/neighbor_plans"
qos = "control"
[[channel]]
name = "ros/sync_trigger"
qos = "control"

[[plugin]]
name = "dmpc"
path = "{lib}"
trigger = "both"
config = {{ uav_id = {i}, participant_ids = [1, 2, 3] }}
bind = {{ own_plan = {{ channel = "ros/assumed_trajectories", from = ["rosio{i}"] }}, plan_in = {{ channel = "dmpc/plan", from = [{others}] }}, plan_out = {{ channel = "dmpc/plan" }}, neighbor_plans = {{ channel = "ros/neighbor_plans" }}, sync_trigger = {{ channel = "ros/sync_trigger" }} }}
"#,
        lib = common::lib("dmpc_rounds"),
        others = others.join(", "),
    )
}

#[test]
fn local_rounds_trigger_every_robot_and_plans_cross_only_between_robots() {
    let dir = common::scratch("dmpc-rounds");
    let bus = LoopbackBus::new();
    let clock: Arc<dyn Clock> = Arc::new(WallClock::new(0));
    common::lib("dmpc_rounds"); // build the plugin before choosing E0
    let e0 = clock.now() + 800_000_000;
    let stop = Arc::new(AtomicBool::new(false));
    let mut hosts = Vec::new();
    for i in 1..=3 {
        let m = Manifest::from_toml_str(&manifest(i, e0)).unwrap();
        let host = Host::new(m, &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();
        let stop = stop.clone();
        hosts.push(std::thread::spawn(move || host.run(&stop).unwrap()));
    }
    // The ros_io stand-ins.
    let channels: Vec<ChannelSpec> = CHANNELS.iter().enumerate().map(|(c, n)| ChannelSpec { id: c as u32, name: n.to_string(), qos: Qos::Control }).collect();
    let mut rosio = Vec::new();
    for i in 1..=3u16 {
        let node = 2 + i; // rosio1 = 3
        let meta = NodeMeta {
            format: String::new(), session: "dmpc".into(), node: ROSTER[node as usize].into(), node_id: node,
            roster: ROSTER.iter().map(|s| s.to_string()).collect(), channels: CHANNELS.iter().map(|s| s.to_string()).collect(),
            clock_domain: "wall".into(), audit_queue_drops: 0, records_written: 0, complete: false,
        };
        let audit = Arc::new(FileAudit::create(&dir.join("audit"), meta, clock.clone()).unwrap());
        let ctx = TransportContext { session: "dmpc".into(), node: ROSTER[node as usize].into(), node_id: node, roster: ROSTER.iter().map(|s| s.to_string()).collect(), channels: channels.clone() };
        let ep = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 16).unwrap();
        ep.declare_out(1).unwrap();
        ep.declare_in(2, &[i - 1]).unwrap();
        ep.declare_in(3, &[i - 1]).unwrap();
        rosio.push((ep, audit));
    }
    // Each node publishes its plan ten times, 100 ms apart, after E0, and
    // echoes whatever neighbor plans it was given.
    std::thread::sleep(Duration::from_nanos((e0 - clock.now()).max(0) as u64) + Duration::from_millis(60));
    let mut got: Vec<(Vec<Vec<u8>>, Vec<Vec<u8>>)> = vec![(Vec::new(), Vec::new()); 3];
    for n in 0..10 {
        for (i, (ep, _)) in rosio.iter().enumerate() {
            ep.publish(1, 0, clock.now(), &plan(i as u32 + 1, n as f64)).unwrap();
            for f in ep.drain() {
                if f.header.channel == 2 {
                    ep.publish(1, 0, clock.now(), &f.payload).unwrap(); // the echo
                    got[i].0.push(f.payload);
                } else {
                    got[i].1.push(f.payload);
                }
            }
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    let summaries: Vec<_> = hosts.into_iter().map(|h| h.join().unwrap()).collect();
    for (i, (ep, audit)) in rosio.iter().enumerate() {
        for f in ep.drain() {
            if f.header.channel == 2 { got[i].0.push(f.payload) } else { got[i].1.push(f.payload) }
        }
        ep.close();
        audit.finish().unwrap();
    }

    for (i, (neighbor_plans, triggers)) in got.iter().enumerate() {
        let me = i as u32 + 1;
        let dmpc = &summaries[i].plugins[0];
        println!("uav{me}: {dmpc:?}; {} neighbor plans, {} triggers", neighbor_plans.len(), triggers.len());
        // Plans stop ~0.5 s before the run ends, so the last rounds see stale
        // neighbors: "partial", not "waiting".
        assert_eq!(dmpc.domain_state, "partial", "uav{me}");
        // The node got every neighbor plan once, and never its own.
        assert!(neighbor_plans.iter().all(|p| uav_of(p) != me));
        for other in (1..=3).filter(|j| *j != me) {
            let rounds: Vec<f64> = neighbor_plans.iter().filter(|p| uav_of(p) == other).map(|p| f64::from_le_bytes(p[32..40].try_into().unwrap())).collect();
            assert_eq!(rounds, (0..10).map(f64::from).collect::<Vec<_>>(), "uav{me} from uav{other}");
        }
        // Echoed neighbor plans were ignored: plan_out carried only the ten
        // own plans, and neighbor_plans the twenty neighbor plans.
        assert_eq!(dmpc.published as usize, 10 + 20 + triggers.len(), "uav{me}: plans, forwards and triggers only");
        // One trigger per local round, scheduled at E0 + k·P.
        let seqs: Vec<u64> = triggers.iter().map(|t| u64::from_le_bytes(t[0..8].try_into().unwrap())).collect();
        assert!(seqs.windows(2).all(|w| w[1] == w[0] + 1) && seqs.len() >= 25, "uav{me} triggers {seqs:?}");
        for t in triggers {
            let k = u64::from_le_bytes(t[0..8].try_into().unwrap());
            let at = f64::from_le_bytes(t[8..16].try_into().unwrap());
            let want = (e0 + k as i64 * PERIOD_MS * 1_000_000) as f64 * 1e-9;
            assert!((at - want).abs() < 1e-6, "uav{me} round {k}: trigger at {at}, scheduled {want}");
            assert_eq!(&t[24..28], &3u32.to_le_bytes());
        }
    }
    // Every robot saw the same round numbers at the same scheduled times.
    let first = |i: usize| u64::from_le_bytes(got[i].1[0][0..8].try_into().unwrap());
    assert!((0..3).map(first).max().unwrap() - (0..3).map(first).min().unwrap() <= 1);
}
