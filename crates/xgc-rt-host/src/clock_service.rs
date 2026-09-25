//! The in-host clock probe (docs/time-model.md). It is a host service on
//! two reserved channels, not a plugin, because it sets the bound that
//! stamps every frame.
//!
//! Wire format (little-endian):
//! - req: `nonce u64`
//! - rep (32 bytes): `target u16, 0 u16, 0 u32, nonce u64, t1 i64, t2 i64`
//!
//! t1 and t2 are the request's `t_tx` and the server's `t_rx` for it. The
//! client takes t3 from the reply's `t_tx` and t4 from its `t_rx`, so the
//! probe measures exactly the stamp points the audit uses.

use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

use xgc_rt_clock::{ChronyTracking, Estimate, ProbeEstimator, ProbeSample};
use xgc_rt_core::manifest::{ClockRole, ResolvedClock};
use xgc_rt_core::transport::TransportError;
use xgc_rt_core::OriginId;

use crate::endpoint::{Endpoint, RxFrame};

pub struct ClockService {
    spec: ResolvedClock,
    node: OriginId,
    endpoint: Arc<Endpoint>,
    next_probe: Instant,
    interval: Duration,
    nonce: u64,
    estimator: ProbeEstimator,
    log: Option<File>,
    chrony_present: bool,
    replies_sent: u64,
}

impl ClockService {
    pub fn new(spec: ResolvedClock, node: OriginId, roster_len: usize, endpoint: Arc<Endpoint>, log_path: &Path) -> Result<Self, TransportError> {
        match spec.role {
            ClockRole::Client => {
                endpoint.declare_out(spec.req)?;
                endpoint.declare_in(spec.rep, &[spec.server])?;
            }
            ClockRole::Server => {
                endpoint.declare_out(spec.rep)?;
                let clients: Vec<OriginId> = (0..roster_len as OriginId).filter(|&o| o != node).collect();
                endpoint.declare_in(spec.req, &clients)?;
                endpoint.clock().set_bound_ns(0);
            }
        }
        let log = OpenOptions::new().create(true).append(true).open(log_path).ok();
        Ok(Self {
            interval: Duration::from_nanos(spec.interval_ns as u64),
            estimator: ProbeEstimator::new(spec.window),
            spec,
            node,
            endpoint,
            next_probe: Instant::now(),
            nonce: 0,
            log,
            chrony_present: true,
            replies_sent: 0,
        })
    }

    pub fn is_client(&self) -> bool {
        self.spec.role == ClockRole::Client
    }

    pub fn estimate(&self) -> Option<Estimate> {
        self.estimator.estimate()
    }

    pub fn within_gate(&self) -> bool {
        !self.is_client() || self.estimate().is_some_and(|e| e.samples >= 3 && e.bound_ns <= self.spec.gate_ns)
    }

    pub fn gate_timeout(&self) -> Duration {
        Duration::from_nanos(self.spec.gate_timeout_ns as u64)
    }

    /// When the next probe is due (clients only).
    pub fn next_due(&self) -> Option<Instant> {
        self.is_client().then_some(self.next_probe)
    }

    /// Send a probe if due. `fast` probes every 100 ms, used while gating.
    pub fn tick(&mut self, fast: bool) {
        if !self.is_client() || Instant::now() < self.next_probe {
            return;
        }
        self.nonce += 1;
        let _ = self.endpoint.publish(self.spec.req, 0, self.endpoint.clock().now(), &self.nonce.to_le_bytes());
        self.next_probe = Instant::now() + if fast { Duration::from_millis(100) } else { self.interval };
    }

    /// Handle a frame on a probe channel. It returns false for any other
    /// channel.
    pub fn on_frame(&mut self, f: &RxFrame) -> bool {
        let h = &f.header;
        if h.channel == self.spec.req && self.spec.role == ClockRole::Server {
            if f.payload.len() == 8 {
                let mut rep = Vec::with_capacity(32);
                rep.extend_from_slice(&h.origin.to_le_bytes());
                rep.extend_from_slice(&[0u8; 6]);
                rep.extend_from_slice(&f.payload);
                rep.extend_from_slice(&h.t_tx.to_le_bytes());
                rep.extend_from_slice(&f.t_rx.to_le_bytes());
                if self.endpoint.publish(self.spec.rep, 0, self.endpoint.clock().now(), &rep).is_ok() {
                    self.replies_sent += 1;
                }
            }
            return true;
        }
        if h.channel == self.spec.rep && self.spec.role == ClockRole::Client {
            let p = &f.payload;
            if p.len() == 32 && u16::from_le_bytes([p[0], p[1]]) == self.node {
                let i64_at = |o: usize| i64::from_le_bytes(p[o..o + 8].try_into().unwrap());
                let sample = ProbeSample { t1: i64_at(16), t2: i64_at(24), t3: h.t_tx, t4: f.t_rx };
                let accepted = self.estimator.add(sample);
                let estimate = self.estimator.estimate();
                if let Some(e) = estimate {
                    self.endpoint.clock().set_bound_ns(u32::try_from(e.bound_ns.max(0)).unwrap_or(u32::MAX));
                    self.endpoint.set_clock_degraded(e.bound_ns > self.spec.gate_ns);
                }
                self.record(sample, accepted, estimate);
            }
            return true;
        }
        h.channel == self.spec.req || h.channel == self.spec.rep
    }

    fn record(&mut self, s: ProbeSample, accepted: bool, e: Option<Estimate>) {
        let chrony = if self.chrony_present { ChronyTracking::read() } else { None };
        if chrony.is_none() {
            self.chrony_present = false; // absent once, absent for the run
        }
        let Some(log) = self.log.as_mut() else { return };
        let line = serde_json::json!({
            "t": s.t4,
            "t1": s.t1, "t2": s.t2, "t3": s.t3, "t4": s.t4,
            "offset_ns": s.offset_ns(), "delay_ns": s.delay_ns(), "sample_bound_ns": s.bound_ns(), "accepted": accepted,
            "estimate": e.map(|e| serde_json::json!({"offset_ns": e.offset_ns, "delay_ns": e.delay_ns, "bound_ns": e.bound_ns, "samples": e.samples})),
            "chrony": chrony.map(|c| serde_json::json!({"reference": c.reference, "offset_s": c.system_offset_s, "bound_ns": c.bound_ns(), "leap": c.leap_status})),
        });
        let _ = writeln!(log, "{line}");
    }
}
