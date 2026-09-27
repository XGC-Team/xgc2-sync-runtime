//! Z2b calibration: the audit stays correct under impairment. uav1 → uav2
//! over Zenoh UDP through the seeded relay; per-sample join of the audit
//! against the relay's ground truth. Profile A runs twice: with the
//! built-in Zenoh transport and with the Zenoh transport plugin
//! (xgc_rt_transport_v1), which must account the same way.

mod common;

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_audit::record::{Kind, Record, RECORD_LEN};
use xgc_rt_audit::{merge_run, FileAudit, MergeOptions, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::transport::{ChannelSpec, Qos, Transport, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_impair::{Action, GilbertElliott, Profile, Relay};
use xgc_rt_transport_zenoh::{ZenohOptions, ZenohTransport};

const SAMPLES: u64 = 2_000;

fn free_udp() -> u16 {
    std::net::UdpSocket::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

fn pct(v: &mut [i64], p: f64) -> i64 {
    v.sort_unstable();
    v[((p / 100.0 * v.len() as f64).ceil() as usize).clamp(1, v.len()) - 1]
}

#[allow(dead_code)]
struct Outcome {
    lost: u64,
    received: u64,
    dup: u64,
    reordered: u64,
    relay_drop: u64,
    relay_dup: u64,
    relay_reorder: u64,
    zenoh_discarded: u64,
    discarded_after_reorder: u64,
}

fn calibrate(name: &str, profile: Profile) -> Outcome {
    calibrate_with(name, profile, &|o| Box::new(ZenohTransport::new(o)))
}

/// The Zenoh transport plugin with the same options.
fn plugin(o: ZenohOptions) -> Box<dyn Transport> {
    let list = |v: &[String]| v.iter().map(|e| format!("\"{e}\"")).collect::<Vec<_>>().join(", ");
    common::so_transport("zenoh", &format!("listen = [{}]\nconnect = [{}]\n", list(&o.listen), list(&o.connect)))
}

fn calibrate_with(name: &str, profile: Profile, transport: &dyn Fn(ZenohOptions) -> Box<dyn Transport>) -> Outcome {
    let run = common::scratch(name);
    let clock = Arc::new(WallClock::new(0));
    let roster = vec!["uav1".to_string(), "uav2".to_string()];
    let channels = vec![ChannelSpec { id: 0, name: "dmpc/plan".into(), qos: Qos::Control }];
    let pb = free_udp();
    let relay = Relay::start("127.0.0.1:0".parse().unwrap(), format!("127.0.0.1:{pb}").parse().unwrap(), profile).unwrap();
    let opts = [
        ZenohOptions { listen: vec![], connect: vec![format!("udp/{}", relay.listen)] },
        ZenohOptions { listen: vec![format!("udp/127.0.0.1:{pb}")], connect: vec![] },
    ];
    let mut audits = Vec::new();
    let mut eps = Vec::new();
    for (i, o) in opts.into_iter().enumerate() {
        let meta = NodeMeta {
            format: String::new(), session: name.into(), node: roster[i].clone(), node_id: i as u16,
            roster: roster.clone(), channels: vec!["dmpc/plan".into()], clock_domain: "wall".into(),
            audit_queue_drops: 0, records_written: 0, complete: false,
        };
        let audit = Arc::new(FileAudit::create(&run, meta, clock.clone()).unwrap());
        let ctx = TransportContext { session: name.into(), node: roster[i].clone(), node_id: i as u16, roster: roster.clone(), channels: channels.clone() };
        eps.push(Endpoint::open(transport(o), &ctx, clock.clone(), audit.clone(), 1 << 16).unwrap());
        audits.push(audit);
    }
    eps[0].declare_out(0).unwrap();
    eps[1].declare_in(0, &[0]).unwrap();
    assert!(eps[0].wait_ready(Duration::from_secs(15)), "no subscriber matched through the relay");

    let payload = vec![0xC3u8; 1024];
    let start = std::time::Instant::now();
    for k in 0..SAMPLES {
        let due = start + Duration::from_micros(5_000 * k);
        if let Some(wait) = due.checked_duration_since(std::time::Instant::now()) {
            std::thread::sleep(wait);
        }
        eps[0].publish(0, k, clock.now(), &payload).unwrap();
    }
    std::thread::sleep(Duration::from_millis(1_500)); // > grace + max delay
    for ep in &eps {
        ep.drain();
        ep.close();
    }
    for a in &audits {
        a.finish().unwrap();
    }
    let truth = relay.stop();

    let report = merge_run(&run, MergeOptions::default()).unwrap();
    xgc_rt_audit::write_report(&report, &run.join("merged")).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    let s = &report.streams[0];

    // Per-sample join: receiver arrivals vs relay truth.
    let bytes = std::fs::read(run.join("uav2/records.bin")).unwrap();
    let mut first_owd: HashMap<u64, i64> = HashMap::new();
    for chunk in bytes.chunks_exact(RECORD_LEN) {
        let r = Record::decode(chunk).unwrap();
        if r.kind == Kind::Rx {
            first_owd.entry(r.seq).or_insert(r.t_b - r.t_a);
        }
    }
    let by_seq: HashMap<u64, &xgc_rt_impair::TruthRecord> = truth.iter().filter(|t| t.origin == 0).map(|t| (t.seq, t)).collect();
    // Samples the relay never saw were shed by the sender's own best-effort
    // queue under congestion (control class = drop). None may be received.
    let sender_shed = (1..=SAMPLES).filter(|seq| !by_seq.contains_key(seq)).count() as u64;
    for seq in (1..=SAMPLES).filter(|seq| !by_seq.contains_key(seq)) {
        assert!(!first_owd.contains_key(&seq), "seq {seq} never passed the relay but was received");
    }
    let count = |a: Action| truth.iter().filter(|t| t.action == a).count() as u64;
    let (relay_drop, relay_dup, relay_reorder) = (count(Action::Drop), count(Action::Duplicate), count(Action::Reorder));
    let mut residual = Vec::new();
    let mut zenoh_discarded = 0;
    let mut discarded_after_reorder = 0;
    for (seq, t) in &by_seq {
        match (t.action, first_owd.get(seq)) {
            (Action::Drop, Some(_)) => panic!("seq {seq} was dropped by the relay but the audit received it"),
            (Action::Drop, None) => {}
            (action, None) => {
                zenoh_discarded += 1;
                if action == Action::Reorder {
                    discarded_after_reorder += 1;
                }
            }
            (_, Some(owd)) => residual.push(owd - t.delays_ns.iter().min().copied().unwrap()),
        }
    }
    let c = &s.counts;
    assert_eq!(c.lost, relay_drop + zenoh_discarded + sender_shed, "every loss is a relay drop, a receive-side discard, or a sender shed");
    let (p50, p99) = (pct(&mut residual.clone(), 50.0), pct(&mut residual, 99.0));
    println!(
        "[{name}] expected {} received {} lost {} (relay dropped {relay_drop}, zenoh discarded {zenoh_discarded}, sender shed {sender_shed}) dup {} (relay {relay_dup}) reordered {} (relay held {relay_reorder})\n  OWD p50/p99 {:.3}/{:.3} ms; OWD - injected delay p50/p99 {:.3}/{:.3} ms; bound {} ns",
        c.expected, c.received, c.lost, c.duplicates, c.reordered,
        s.owd_ns.p50 as f64 / 1e6, s.owd_ns.p99 as f64 / 1e6, p50 as f64 / 1e6, p99 as f64 / 1e6, s.owd_bound_ns.max
    );
    assert!(p50.abs() < 1_000_000, "median OWD must equal injected delay within 1 ms (got {p50} ns)");
    Outcome {
        lost: c.lost,
        received: c.received,
        dup: c.duplicates,
        reordered: c.reordered,
        relay_drop,
        relay_dup,
        relay_reorder,
        zenoh_discarded,
        discarded_after_reorder,
    }
}

fn profile_a() -> Profile {
    Profile {
        delay_ms: 40.0,
        jitter_ms: 0.0,
        loss: 0.0,
        gilbert_elliott: Some(GilbertElliott { p_good_bad: 0.01, p_bad_good: 0.3, loss_good: 0.01, loss_bad: 0.5 }),
        duplicate: 0.02,
        reorder: 0.02,
        seed: 0xA11CE,
    }
}

#[test]
fn audit_matches_relay_truth_under_burst_loss_duplication_and_reordering() {
    check_profile_a(calibrate("z2b-profile-a", profile_a()));
}

#[test]
fn audit_matches_relay_truth_through_the_zenoh_transport_plugin() {
    check_profile_a(calibrate_with("z2e-profile-a-plugin", profile_a(), &plugin));
}

fn check_profile_a(o: Outcome) {
    assert!(o.relay_drop > 10 && o.relay_dup > 10 && o.relay_reorder > 10, "profile exercised every action");
    assert_eq!(o.received + o.lost, SAMPLES);
    // Transport finding (zenoh 1.9 best-effort over UDP): a frame that
    // arrives after a later one is discarded, and duplicates are
    // suppressed. So wire reordering surfaces as loss, never as reorder.
    assert_eq!(o.zenoh_discarded, o.relay_reorder, "exactly the held (reordered) datagrams were discarded");
    assert_eq!(o.discarded_after_reorder, o.relay_reorder);
    assert_eq!((o.dup, o.reordered), (0, 0));
}

#[test]
fn jitter_beyond_the_send_interval_turns_into_loss_on_best_effort() {
    // 40 ± 10 ms on a 5 ms send interval reorders heavily on the wire.
    let o = calibrate(
        "z2b-profile-b",
        Profile { delay_ms: 40.0, jitter_ms: 10.0, loss: 0.0, gilbert_elliott: None, duplicate: 0.0, reorder: 0.0, seed: 0xB0B },
    );
    assert_eq!(o.relay_drop, 0);
    assert_eq!(o.lost, o.zenoh_discarded, "all loss is transport discard of late frames");
    assert!(o.lost > 0, "jitter across the send interval must cost samples on best-effort");
    assert_eq!(o.reordered, 0);
}
