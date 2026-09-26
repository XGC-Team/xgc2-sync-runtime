//! C4 calibration, the full matrix: 8 nodes × 2 channels (control, state)
//! × 20 Hz × 1 KiB over Zenoh UDP through the seeded relay, every node
//! publishing both channels to all 7 others (112 streams), with the
//! transport loaded from the Zenoh transport plugin. Plus bulk: one bulk
//! channel offered at 50 Mbps.
//!
//! Topology: node j listens; for every pair i < j, node i connects to its
//! own relay whose target is j. The relay impairs i → j and passes j → i
//! through untouched, so every run has 56 impaired streams (i < j) and 56
//! clean ones (i > j).
//!
//! - Clean profile (no impairment): every stream loses nothing; OWD p50 and
//!   p99 reported.
//! - Impaired profile (40 ms, Gilbert-Elliott burst loss, 2 % duplicate,
//!   2 % reorder; a seed per relay): per-sample join of each receiver's
//!   audit against its relay's ground truth. Every loss is a relay drop, a
//!   frame Zenoh discarded on receipt (it arrived after a later one: the
//!   held, reordered datagrams), or a sample the sender's best-effort queue
//!   shed; nothing the relay dropped is received; the clean direction
//!   still loses nothing. OWD: the audited OWD minus the delay the relay
//!   applied (release time - arrival time, recorded per copy) is the path
//!   latency, never negative (one clock); it is reported with its two legs
//!   (sender -> relay, relay release -> receiver). Plan C4 asks for p50
//!   within 1 ms of the injected delay; with 8 Zenoh peers and 28 relays in
//!   one process on an 8-core sandbox the path latency alone is about
//!   1 ms at p50 (0.97-1.13 ms over four runs: 0.32 ms to the relay,
//!   0.62 ms from it; 2 nodes in zenoh_impaired.rs: 0.5-0.6 ms), so this
//!   test asserts p50 within 2 ms and reports the figure.
//! - Bulk: one bulk channel (reliable, block) through a clean relay at
//!   50 Mbps offered for 10 s; delivered Mbps and loss reported.
//!
//! Warm-up: `wait_ready` returns once every out-channel has *a* matching
//! subscriber, which in a mesh can be one peer while the other links are
//! still coming up (measured: the first 0.7-1.5 s of samples lost to some
//! peers). So every node first publishes on a separate `warmup` channel
//! until every node has heard every other one; the measured channels start
//! after that, and the warm-up streams are not part of the calibration.

mod common;

use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};

use xgc_rt_audit::record::{Kind, Record, RECORD_LEN};
use xgc_rt_audit::{merge_run, FileAudit, MergeOptions, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_impair::{Action, GilbertElliott, Profile, Relay, TruthRecord};

const NODES: usize = 8;
const RATE_HZ: u64 = 20;
const SECONDS: u64 = 30;
const PAYLOAD: usize = 1024;

fn free_udp() -> u16 {
    std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

fn pct(v: &mut [i64], p: f64) -> i64 {
    if v.is_empty() {
        return 0;
    }
    v.sort_unstable();
    v[((p / 100.0 * v.len() as f64).ceil() as usize).clamp(1, v.len()) - 1]
}

fn endpoints_list(v: &[String]) -> String {
    v.iter().map(|e| format!("\"{e}\"")).collect::<Vec<_>>().join(", ")
}

struct Mesh {
    run: std::path::PathBuf,
    eps: Vec<Arc<Endpoint>>,
    audits: Vec<Arc<FileAudit>>,
    relays: HashMap<(usize, usize), Relay>,
    clock: Arc<WallClock>,
}

/// `nodes` nodes on `channels`, node i connecting to j (i < j) through a
/// relay with `profile(i, j)`.
fn mesh(name: &str, nodes: usize, channels: &[ChannelSpec], profile: &dyn Fn(usize, usize) -> Profile) -> Mesh {
    let run = common::scratch(name);
    let clock = Arc::new(WallClock::new(0));
    let roster: Vec<String> = (1..=nodes).map(|i| format!("uav{i}")).collect();
    let ports: Vec<u16> = (0..nodes).map(|_| free_udp()).collect();
    let mut relays = HashMap::new();
    for i in 0..nodes {
        for j in i + 1..nodes {
            let relay = Relay::start("127.0.0.1:0".parse().unwrap(), format!("127.0.0.1:{}", ports[j]).parse().unwrap(), profile(i, j)).unwrap();
            relays.insert((i, j), relay);
        }
    }
    let mut eps = Vec::new();
    let mut audits = Vec::new();
    for i in 0..nodes {
        let listen = vec![format!("udp/127.0.0.1:{}", ports[i])];
        let connect: Vec<String> = (i + 1..nodes).map(|j| format!("udp/{}", relays[&(i, j)].listen)).collect();
        let options = format!("listen = [{}]\nconnect = [{}]\n", endpoints_list(&listen), endpoints_list(&connect));
        let meta = NodeMeta {
            format: String::new(), session: name.into(), node: roster[i].clone(), node_id: i as u16,
            roster: roster.clone(), channels: channels.iter().map(|c| c.name.clone()).collect(), clock_domain: "wall".into(),
            audit_queue_drops: 0, records_written: 0, complete: false,
        };
        let audit = Arc::new(FileAudit::create_with_capacity(&run, meta, clock.clone(), 1 << 20).unwrap());
        let ctx = TransportContext { session: name.into(), node: roster[i].clone(), node_id: i as u16, roster: roster.clone(), channels: channels.to_vec() };
        eps.push(Endpoint::open(common::so_transport("zenoh", &options), &ctx, clock.clone(), audit.clone(), 1 << 20).unwrap());
        audits.push(audit);
    }
    for (i, ep) in eps.iter().enumerate() {
        let others: Vec<u16> = (0..nodes).filter(|&j| j != i).map(|j| j as u16).collect();
        for c in channels {
            ep.declare_out(c.id).unwrap();
            ep.declare_in(c.id, &others).unwrap();
        }
    }
    for (i, ep) in eps.iter().enumerate() {
        assert!(ep.wait_ready(Duration::from_secs(30)), "uav{} not matched by every peer", i + 1);
    }
    Mesh { run, eps, audits, relays, clock }
}

impl Mesh {
    /// Drain, close, finish; the relays' truth by (sender, receiver).
    fn finish(self) -> (std::path::PathBuf, HashMap<(usize, usize), Vec<TruthRecord>>) {
        std::thread::sleep(Duration::from_millis(1_500));
        for ep in &self.eps {
            ep.drain();
            ep.close();
        }
        for a in &self.audits {
            a.finish().unwrap();
        }
        let truth = self.relays.into_iter().map(|(k, r)| (k, r.stop())).collect();
        (self.run, truth)
    }
}

/// First arrival OWD per (origin, channel, seq) at `receiver`.
fn arrivals(run: &std::path::Path, receiver: usize) -> HashMap<(u16, u32, u64), i64> {
    stamped_arrivals(run, receiver).into_iter().map(|(k, (t_tx, t_rx))| (k, t_rx - t_tx)).collect()
}

/// First arrival (t_tx, t_rx) per (origin, channel, seq) at `receiver`.
fn stamped_arrivals(run: &std::path::Path, receiver: usize) -> HashMap<(u16, u32, u64), (i64, i64)> {
    let bytes = std::fs::read(run.join(format!("uav{}/records.bin", receiver + 1))).unwrap();
    let mut first = HashMap::new();
    for chunk in bytes.chunks_exact(RECORD_LEN) {
        let r = Record::decode(chunk).unwrap();
        if r.kind == Kind::Rx {
            first.entry((r.origin, r.channel, r.seq)).or_insert((r.t_a, r.t_b));
        }
    }
    first
}

const WARMUP: u32 = 2;

fn channels() -> Vec<ChannelSpec> {
    vec![
        ChannelSpec { id: 0, name: "dmpc/plan".into(), qos: Qos::Control },
        ChannelSpec { id: 1, name: "state".into(), qos: Qos::State },
        ChannelSpec { id: WARMUP, name: "warmup".into(), qos: Qos::Control },
    ]
}

/// Publish on `warmup` until every node has heard every other node.
fn warm_up(m: &Mesh) {
    let n = m.eps.len();
    let mut heard = vec![std::collections::HashSet::new(); n];
    let start = Instant::now();
    let mut k = 0;
    while heard.iter().any(|h| h.len() < n - 1) {
        assert!(start.elapsed() < Duration::from_secs(30), "warm-up: not every node heard every other one");
        k += 1;
        for ep in &m.eps {
            ep.publish(WARMUP, k, m.clock.now(), &[0u8; 16]).unwrap();
        }
        std::thread::sleep(Duration::from_millis(50));
        for (r, ep) in m.eps.iter().enumerate() {
            heard[r].extend(ep.drain().into_iter().filter(|f| f.header.channel == WARMUP).map(|f| f.header.origin));
        }
    }
    println!("warm-up: every node heard every other one after {:.2} s", start.elapsed().as_secs_f64());
}

/// Warm up, then 20 Hz, both channels, every node, for SECONDS.
fn drive(m: &Mesh) {
    warm_up(m);
    let payload = vec![0x5Au8; PAYLOAD];
    let start = Instant::now();
    for k in 0..RATE_HZ * SECONDS {
        let due = start + Duration::from_micros(1_000_000 / RATE_HZ * k);
        if let Some(wait) = due.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
        for ep in &m.eps {
            for c in 0..2 {
                ep.publish(c, k, m.clock.now(), &payload).unwrap();
            }
            ep.drain();
        }
    }
}

struct Totals {
    streams: usize,
    expected: u64,
    lost: u64,
    owd: Vec<i64>,
}

#[test]
fn eight_nodes_two_channels_clean_baseline_loses_nothing() {
    let m = mesh("c4-clean", NODES, &channels(), &|i, j| Profile { seed: (i * NODES + j) as u64, ..Profile::default() });
    drive(&m);
    let (run, _truth) = m.finish();
    let report = merge_run(&run, MergeOptions::default()).unwrap();
    xgc_rt_audit::write_report(&report, &run.join("merged")).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    assert_eq!(report.streams.len(), NODES * (NODES - 1) * 3);
    let mut t = Totals { streams: 0, expected: 0, lost: 0, owd: Vec::new() };
    for r in 0..NODES {
        t.owd.extend(arrivals(&run, r).iter().filter(|(k, _)| k.1 != WARMUP).map(|(_, v)| *v));
    }
    for s in report.streams.iter().filter(|s| s.channel != "warmup") {
        t.streams += 1;
        t.expected += s.counts.expected;
        t.lost += s.counts.lost;
        assert_eq!(s.counts.expected, RATE_HZ * SECONDS, "{} {} -> {}", s.channel, s.origin, s.receiver);
        assert_eq!((s.counts.lost, s.counts.duplicates, s.counts.reordered), (0, 0, 0), "{} {} -> {}: {:?}", s.channel, s.origin, s.receiver, s.counts);
    }
    let (p50, p99) = (pct(&mut t.owd.clone(), 50.0), pct(&mut t.owd, 99.0));
    println!(
        "[C4 clean] {} streams, {} samples expected, {} lost; OWD p50 {:.3} ms p99 {:.3} ms",
        t.streams, t.expected, t.lost, p50 as f64 / 1e6, p99 as f64 / 1e6
    );
}

#[test]
fn eight_nodes_two_channels_impaired_audit_equals_relay_truth() {
    let profile = |i: usize, j: usize| Profile {
        delay_ms: 40.0,
        jitter_ms: 0.0,
        loss: 0.0,
        gilbert_elliott: Some(GilbertElliott { p_good_bad: 0.01, p_bad_good: 0.3, loss_good: 0.01, loss_bad: 0.5 }),
        duplicate: 0.02,
        reorder: 0.02,
        seed: 0xC4_0000 + (i * NODES + j) as u64,
    };
    let m = mesh("c4-impaired", NODES, &channels(), &profile);
    drive(&m);
    let (run, truth) = m.finish();
    let report = merge_run(&run, MergeOptions::default()).unwrap();
    xgc_rt_audit::write_report(&report, &run.join("merged")).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    assert_eq!(report.streams.len(), NODES * (NODES - 1) * 3);
    let node = |name: &str| name.trim_start_matches("uav").parse::<usize>().unwrap() - 1;
    let arrivals: Vec<_> = (0..NODES).map(|r| arrivals(&run, r)).collect();
    let stamped: Vec<_> = (0..NODES).map(|r| stamped_arrivals(&run, r)).collect();
    let mut leg_in = Vec::new();
    let mut leg_out = Vec::new();
    let (mut drop, mut dup, mut held, mut discarded, mut shed, mut lost_impaired, mut lost_clean) = (0, 0, 0, 0, 0, 0, 0);
    let mut residual = Vec::new();
    let mut residual_scheduled = Vec::new();
    let mut lateness = Vec::new();
    let mut clean_owd = Vec::new();
    for s in report.streams.iter().filter(|s| s.channel != "warmup") {
        let (i, j) = (node(&s.origin), node(&s.receiver));
        let c: u32 = if s.channel == "dmpc/plan" { 0 } else { 1 };
        let got = &arrivals[j];
        assert_eq!(s.counts.expected, RATE_HZ * SECONDS);
        if i > j {
            // The relay's pass-through direction: clean.
            assert_eq!(s.counts.lost, 0, "clean {} {} -> {}: {:?}", s.channel, s.origin, s.receiver, s.counts);
            clean_owd.extend((1..=s.counts.expected).filter_map(|q| got.get(&(i as u16, c, q))));
            lost_clean += s.counts.lost;
            continue;
        }
        let by_seq: HashMap<u64, &TruthRecord> = truth[&(i, j)].iter().filter(|t| t.origin == i as u16 && t.channel == c).map(|t| (t.seq, t)).collect();
        let (mut s_drop, mut s_discard, mut s_shed) = (0u64, 0u64, 0u64);
        for q in 1..=s.counts.expected {
            let arrived = got.get(&(i as u16, c, q));
            match (by_seq.get(&q), arrived) {
                (None, None) => s_shed += 1,
                (None, Some(_)) => panic!("{} {i}->{j} seq {q} never passed the relay but was received", s.channel),
                (Some(t), None) if t.action == Action::Drop => s_drop += 1,
                (Some(t), Some(_)) if t.action == Action::Drop => panic!("{} {i}->{j} seq {q} dropped by the relay but received", s.channel),
                (Some(t), None) => {
                    s_discard += 1;
                    if t.action == Action::Reorder {
                        held += 1;
                    }
                }
                (Some(t), Some(owd)) => {
                    let scheduled = t.delays_ns.iter().min().copied().unwrap();
                    let applied = t.released_ns.iter().min().copied().unwrap() - t.t_in;
                    residual.push(owd - applied);
                    let (t_tx, t_rx) = stamped[j][&(i as u16, c, q)];
                    leg_in.push(t.t_in - t_tx);
                    leg_out.push(t_rx - t.released_ns.iter().min().copied().unwrap());
                    residual_scheduled.push(owd - scheduled);
                    lateness.push(applied - scheduled);
                }
            }
            if let Some(t) = by_seq.get(&q) {
                dup += u64::from(t.action == Action::Duplicate);
            }
        }
        assert_eq!(s.counts.lost, s_drop + s_discard + s_shed, "{} uav{} -> uav{}: every loss is a relay drop, a receive-side discard or a sender shed", s.channel, i + 1, j + 1);
        assert_eq!((s.counts.duplicates, s.counts.reordered), (0, 0), "best-effort Zenoh suppresses duplicates and discards late frames");
        drop += s_drop;
        discarded += s_discard;
        shed += s_shed;
        lost_impaired += s.counts.lost;
    }
    let relay_held: u64 = truth.values().flatten().filter(|t| t.action == Action::Reorder && t.channel != WARMUP).count() as u64;
    let (r50, r99) = (pct(&mut residual.clone(), 50.0), pct(&mut residual, 99.0));
    let (s50, s99) = (pct(&mut residual_scheduled.clone(), 50.0), pct(&mut residual_scheduled, 99.0));
    let (l50, l99) = (pct(&mut lateness.clone(), 50.0), pct(&mut lateness, 99.0));
    println!(
        "  legs: sender -> relay p50 {:.3} ms p99 {:.3} ms; relay release -> receiver p50 {:.3} ms p99 {:.3} ms",
        pct(&mut leg_in.clone(), 50.0) as f64 / 1e6, pct(&mut leg_in, 99.0) as f64 / 1e6,
        pct(&mut leg_out.clone(), 50.0) as f64 / 1e6, pct(&mut leg_out, 99.0) as f64 / 1e6
    );
    let (c50, c99) = (pct(&mut clean_owd.clone(), 50.0), pct(&mut clean_owd, 99.0));
    println!(
        "[C4 impaired] 56 impaired streams: lost {lost_impaired} = relay drop {drop} + receive-side discard {discarded} (of which held {held}; relay held {relay_held}) + sender shed {shed}; relay duplicates {dup}, none received twice\n  OWD - applied delay p50 {:.3} ms p99 {:.3} ms; OWD - scheduled delay p50 {:.3} ms p99 {:.3} ms; relay lateness (applied - scheduled) p50 {:.3} ms p99 {:.3} ms\n  56 clean streams: lost {lost_clean}; OWD p50 {:.3} ms p99 {:.3} ms",
        r50 as f64 / 1e6, r99 as f64 / 1e6, s50 as f64 / 1e6, s99 as f64 / 1e6, l50 as f64 / 1e6, l99 as f64 / 1e6, c50 as f64 / 1e6, c99 as f64 / 1e6
    );
    assert!(drop > 50 && dup > 50 && relay_held > 50, "the profile exercised every action");
    assert!(residual.iter().all(|&r| r >= 0), "the audited OWD is never below the delay the relay applied (one clock)");
    assert!(r50 < 2_000_000, "median OWD within 2 ms of the applied delay (got {r50} ns)");
}

#[test]
fn bulk_at_50_mbps_offered() {
    const FRAME: usize = 16 * 1024;
    const BULK_SECONDS: u64 = 10;
    let rate = 50_000_000 / (FRAME as u64 * 8); // frames per second
    let channels = vec![ChannelSpec { id: 0, name: "bulk".into(), qos: Qos::Bulk }];
    let m = mesh("c4-bulk", 2, &channels, &|_, _| Profile::default());
    let payload = vec![0xB5u8; FRAME];
    let start = Instant::now();
    let total = rate * BULK_SECONDS;
    for k in 0..total {
        let due = start + Duration::from_nanos(1_000_000_000 / rate * k);
        if let Some(wait) = due.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
        m.eps[0].publish(0, k, m.clock.now(), &payload).unwrap();
        if k % 64 == 0 {
            m.eps[1].drain();
        }
    }
    let offered_s = start.elapsed().as_secs_f64();
    let (run, _) = m.finish();
    let report = merge_run(&run, MergeOptions::default()).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    let s = &report.streams[0];
    let delivered_mbps = s.counts.received as f64 * FRAME as f64 * 8.0 / offered_s / 1e6;
    println!(
        "[C4 bulk] offered {:.1} Mbps ({} x {} B in {:.2} s): received {} lost {} -> delivered {:.1} Mbps; OWD p50 {:.3} ms p99 {:.3} ms",
        total as f64 * FRAME as f64 * 8.0 / offered_s / 1e6,
        total,
        FRAME,
        offered_s,
        s.counts.received,
        s.counts.lost,
        delivered_mbps,
        s.owd_ns.p50 as f64 / 1e6,
        s.owd_ns.p99 as f64 / 1e6
    );
    assert_eq!(s.counts.expected, total);
    assert_eq!(s.counts.received, total, "clean bulk path must deliver every frame");
    assert_eq!(s.counts.lost, 0);
    assert!(delivered_mbps >= 45.0, "50 Mbps offered must sustain at least 90% throughput, got {delivered_mbps:.1} Mbps");
}
