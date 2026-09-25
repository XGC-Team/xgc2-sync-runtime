//! Z2a: two nodes exchange audited frames over Zenoh (peer mode, TCP on
//! localhost, explicit endpoints, no scouting). Evidence for the transport
//! only: no impairment here (that is the Z2b calibration through the relay).
//! Reliable `event` must lose nothing; best-effort `control` may shed under
//! congestion by design and is checked for accounting only.

mod common;

use std::sync::Arc;
use std::time::Duration;

use xgc_rt_audit::{merge_run, FileAudit, MergeOptions, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_transport_zenoh::{ZenohOptions, ZenohTransport};

const SAMPLES: u64 = 2_000;

fn free_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port()
}

/// Returns (lost, expected) per stream.
fn exchange(name: &str, qos: Qos) -> Vec<(u64, u64)> {
    let run = common::scratch(name);
    let clock = Arc::new(WallClock::new(0));
    let roster = vec!["uav1".to_string(), "uav2".to_string()];
    let channels = vec![ChannelSpec { id: 0, name: "dmpc/plan".into(), qos }];
    let (pa, pb) = (free_port(), free_port());
    let endpoints = [
        ZenohOptions { listen: vec![format!("tcp/127.0.0.1:{pa}")], connect: vec![format!("tcp/127.0.0.1:{pb}")] },
        ZenohOptions { listen: vec![format!("tcp/127.0.0.1:{pb}")], connect: vec![format!("tcp/127.0.0.1:{pa}")] },
    ];
    let mut audits = Vec::new();
    let mut eps = Vec::new();
    for (i, opts) in endpoints.into_iter().enumerate() {
        let meta = NodeMeta {
            format: String::new(),
            session: "zpair".into(),
            node: roster[i].clone(),
            node_id: i as u16,
            roster: roster.clone(),
            channels: vec!["dmpc/plan".into()],
            clock_domain: "wall".into(),
            audit_queue_drops: 0,
            records_written: 0,
            complete: false,
        };
        let audit = Arc::new(FileAudit::create(&run, meta, clock.clone()).unwrap());
        let ctx = TransportContext { session: "zpair".into(), node: roster[i].clone(), node_id: i as u16, roster: roster.clone(), channels: channels.clone() };
        let ep = Endpoint::open(Box::new(ZenohTransport::new(opts)), &ctx, clock.clone(), audit.clone(), 1 << 16).unwrap();
        audits.push(audit);
        eps.push(ep);
    }
    // Both publish and both subscribe to the other: a DMPC neighbor pair.
    eps[0].declare_out(0).unwrap();
    eps[1].declare_out(0).unwrap();
    eps[0].declare_in(0, &[1]).unwrap();
    eps[1].declare_in(0, &[0]).unwrap();
    let t = std::time::Instant::now();
    assert!(eps[0].wait_ready(Duration::from_secs(10)) && eps[1].wait_ready(Duration::from_secs(10)), "peers never matched");
    println!("peers matched after {:.1} ms", t.elapsed().as_secs_f64() * 1e3);

    let payload = vec![0x5au8; 3_672]; // one AssumedTrajectory: 9 x 51 f64
    for k in 0..SAMPLES {
        eps[0].publish(0, k, clock.now(), &payload).unwrap();
        eps[1].publish(0, k, clock.now(), &payload).unwrap();
        std::thread::sleep(Duration::from_micros(500));
    }
    std::thread::sleep(Duration::from_millis(500));
    for ep in &eps {
        ep.drain();
        ep.close();
    }
    for a in &audits {
        a.finish().unwrap();
    }
    let report = merge_run(&run, MergeOptions::default()).unwrap();
    xgc_rt_audit::write_report(&report, &run.join("merged")).unwrap();
    print!("{}", xgc_rt_audit::merge::markdown(&report));
    assert!(report.valid, "{:?}", report.invalid_reasons);
    assert_eq!(report.streams.len(), 2);
    report
        .streams
        .iter()
        .map(|s| {
            let c = &s.counts;
            assert_eq!(c.expected, SAMPLES);
            assert_eq!(c.received + c.lost, SAMPLES, "every expected sample is either received or lost");
            assert!(s.owd_ns.p50 > 0, "{:?}", s.owd_ns);
            (c.lost, c.expected)
        })
        .collect()
}

#[test]
fn reliable_event_class_delivers_every_frame() {
    for (lost, _) in exchange("zenoh-pair-event", Qos::Event) {
        assert_eq!(lost, 0, "reliable + block over TCP must not lose");
    }
}

#[test]
fn best_effort_control_class_is_audited_consistently() {
    // Control is best-effort + drop by design: at ~1.4 kHz of 3.7 KB frames
    // each way it may shed a few under congestion. The claim here is only
    // that every sample is accounted for; the loss is reported, not bounded.
    for (lost, expected) in exchange("zenoh-pair-control", Qos::Control) {
        println!("control class: lost {lost}/{expected}");
    }
}
