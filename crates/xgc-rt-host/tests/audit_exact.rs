//! Z1 exit: on loopback with a seeded injector, the merged audit equals
//! the injector's ground truth *exactly* for loss, duplication and
//! reordering: 100 000 samples, 1 sender, 3 receivers, 5 % drop, 2 %
//! duplicate, 3 % reorder.

mod common;

use std::sync::Arc;

use xgc_rt_audit::{merge_run, FileAudit, MergeOptions, NodeMeta};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_transport_loopback::{Impairment, LoopbackBus, LoopbackTransport};

const SAMPLES: u64 = 100_000;
const ROSTER: [&str; 4] = ["uav1", "uav2", "uav3", "uav4"];

#[test]
fn merged_audit_equals_injected_ground_truth() {
    let run = common::scratch("audit-exact");
    let bus = LoopbackBus::with_impairment(Impairment { drop: 0.05, duplicate: 0.02, reorder: 0.03, seed: 0x5eed });
    let clock = Arc::new(WallClock::new(0));
    let channels = vec![ChannelSpec { id: 0, name: "dmpc/plan".into(), qos: Qos::Control }];

    let mut audits = Vec::new();
    let mut endpoints = Vec::new();
    for (i, node) in ROSTER.iter().enumerate() {
        let meta = NodeMeta {
            format: String::new(),
            session: "exact".into(),
            node: (*node).into(),
            node_id: i as u16,
            roster: ROSTER.iter().map(|s| s.to_string()).collect(),
            channels: vec!["dmpc/plan".into()],
            clock_domain: "wall".into(),
            audit_queue_drops: 0,
            records_written: 0,
            complete: false,
        };
        let audit = Arc::new(FileAudit::create_with_capacity(&run, meta, clock.clone(), 1 << 20).unwrap());
        let ctx = TransportContext {
            session: "exact".into(),
            node: (*node).into(),
            node_id: i as u16,
            roster: ROSTER.iter().map(|s| s.to_string()).collect(),
            channels: channels.clone(),
        };
        let ep = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 20).unwrap();
        audits.push(audit);
        endpoints.push(ep);
    }
    endpoints[0].declare_out(0).unwrap();
    for ep in &endpoints[1..] {
        ep.declare_in(0, &[0]).unwrap();
    }

    let payload = [7u8; 256];
    for k in 0..SAMPLES {
        endpoints[0].publish(0, k, clock.now(), &payload).unwrap();
        if k % 4096 == 0 {
            for ep in &endpoints[1..] {
                ep.drain();
            }
        }
    }
    bus.flush();
    for ep in &endpoints {
        ep.drain();
        ep.close();
    }
    for audit in &audits {
        audit.finish().unwrap();
    }

    let report = merge_run(&run, MergeOptions::default()).unwrap();
    assert!(report.valid, "invalid: {:?}", report.invalid_reasons);
    let truth = bus.truth();
    assert_eq!(report.streams.len(), 3);
    for stream in &report.streams {
        let receiver = ROSTER.iter().position(|n| *n == stream.receiver).unwrap() as u16;
        let t = truth[&(0, 0, receiver)];
        let c = &stream.counts;
        println!(
            "{} -> {}: expected {} lost {} (truth {}) dup {} (truth {}) reordered {} (truth {}) extent max {}",
            stream.origin, stream.receiver, c.expected, c.lost, t.dropped, c.duplicates, t.duplicated, c.reordered, t.reordered, c.reorder_extent_max
        );
        assert_eq!(c.expected, SAMPLES);
        assert_eq!(t.offered, SAMPLES);
        assert_eq!(c.lost, t.dropped, "loss must equal injected drops exactly");
        assert_eq!(c.duplicates, t.duplicated, "duplicates must equal injected duplicates exactly");
        assert_eq!(c.reordered, t.reordered, "reorders must equal injected reorders exactly");
        assert_eq!(c.received, SAMPLES - t.dropped);
        assert!(t.dropped > 4_000 && t.duplicated > 1_500 && t.reordered > 2_000, "injector did not exercise every path: {t:?}");
        assert_eq!(c.late_beyond_grace, 0);
    }
}
