//! Read-only wall-clock evidence over the data path and the existing external
//! chrony service. Required admission latches on lost validity; it never sets
//! system time, changes the Session epoch, or broadcasts a per-round tick.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::Path;
use std::sync::{mpsc, Arc, Mutex};
use std::time::{Duration, Instant};

use xgc_rt_clock::{ChronyTracking, Estimate, ProbeEstimator, ProbeSample};
use xgc_rt_core::manifest::{ClockRole, ResolvedClock, CLOCK_STARTUP_PROBE_MAX_INTERVAL_MS};
use xgc_rt_core::transport::TransportError;
use xgc_rt_core::OriginId;

use crate::endpoint::{Endpoint, RxFrame};

struct Evidence {
    estimator: ProbeEstimator,
    external: Option<(Instant, Result<ChronyTracking, String>)>,
    armed: bool,
    failure: Option<String>,
}

/// Shared by the probe service and module dispatch/output callbacks. They check
/// elapsed validity themselves, so a busy host cannot extend an expired lease.
pub(crate) struct ClockGuard {
    spec: ResolvedClock,
    endpoint: Arc<Endpoint>,
    evidence: Mutex<Evidence>,
}

impl ClockGuard {
    fn stale_after(&self) -> Duration { Duration::from_nanos(self.spec.stale_after_ns as u64) }

    fn reason(&self, e: &Evidence, now: Instant) -> Option<String> {
        if let Some(reason) = &e.failure { return Some(reason.clone()); }
        if self.spec.chrony_source.is_some() {
            match &e.external {
                Some((at, Ok(_))) if now.saturating_duration_since(*at) < self.stale_after() => {},
                Some((_, Err(reason))) => return Some(format!("external clock rejected: {reason}")),
                Some(_) => return Some("external clock evidence expired; synchronize externally and start a new Session".into()),
                None => return Some("external clock evidence unavailable".into()),
            }
        }
        if self.spec.role == ClockRole::Client {
            let minimum = if e.armed { 1 } else { 3 };
            match e.estimator.estimate_at(now) {
                Some(estimate) if estimate.samples >= minimum && estimate.bound_ns <= self.spec.gate_ns => {},
                Some(estimate) => return Some(format!("clock probe requires {minimum} fresh samples within {} ns; got {} samples, bound {} ns", self.spec.gate_ns, estimate.samples, estimate.bound_ns)),
                None => return Some("clock probe evidence unavailable or expired; synchronize externally and start a new Session".into()),
            }
        }
        None
    }

    fn refresh(&self, e: &Evidence, now: Instant) {
        let reason = self.reason(e, now);
        let bound = if e.failure.is_some() || (self.spec.required && reason.is_some()) { u32::MAX }
            else if self.spec.role == ClockRole::Server { 0 }
            else { e.estimator.estimate_at(now).map_or(u32::MAX, |v| u32::try_from(v.bound_ns).unwrap_or(u32::MAX)) };
        self.endpoint.clock().set_bound_ns(bound);
        self.endpoint.set_clock_degraded(reason.is_some());
    }

    pub(crate) fn required(&self) -> bool { self.spec.required }

    pub(crate) fn admit(&self) -> Result<(), String> {
        let mut e = self.evidence.lock().unwrap();
        let now = Instant::now();
        if let Some(reason) = self.reason(&e, now) { return Err(reason); }
        e.armed = true;
        self.refresh(&e, now);
        Ok(())
    }

    pub(crate) fn close(&self) {
        if self.spec.required { self.evidence.lock().unwrap().armed = false; }
    }

    pub(crate) fn fail(&self, reason: String) {
        let mut e = self.evidence.lock().unwrap();
        if e.failure.is_none() { e.failure = Some(reason); }
        self.refresh(&e, Instant::now());
    }

    pub(crate) fn failure(&self) -> Option<String> {
        let mut e = self.evidence.lock().unwrap();
        let now = Instant::now();
        if self.spec.required && e.armed && e.failure.is_none() {
            e.failure = self.reason(&e, now);
        }
        self.refresh(&e, now);
        e.failure.clone()
    }

    pub(crate) fn runnable(&self) -> bool {
        if !self.spec.required { return true; }
        let mut e = self.evidence.lock().unwrap();
        let now = Instant::now();
        if e.armed && e.failure.is_none() { e.failure = self.reason(&e, now); }
        self.refresh(&e, now);
        e.armed && e.failure.is_none()
    }
}

pub struct ClockService {
    spec: ResolvedClock,
    node: OriginId,
    endpoint: Arc<Endpoint>,
    next_probe: Instant,
    interval: Duration,
    nonce: u64,
    pending: VecDeque<(u64, Instant, i64)>,
    guard: Arc<ClockGuard>,
    next_external: Instant,
    external_pending: Option<mpsc::Receiver<(Instant, Result<ChronyTracking, String>)>>,
    log: Option<File>,
}

impl ClockService {
    pub fn new(spec: ResolvedClock, node: OriginId, roster_len: usize, endpoint: Arc<Endpoint>, log_path: &Path) -> Result<Self, TransportError> {
        match spec.role {
            ClockRole::Client => {
                endpoint.declare_out(spec.req)?;
                endpoint.declare_in(spec.rep, &[spec.server])?;
                endpoint.clock().set_bound_ns(u32::MAX);
            }
            ClockRole::Server => {
                endpoint.declare_out(spec.rep)?;
                let clients: Vec<OriginId> = (0..roster_len as OriginId).filter(|&o| o != node).collect();
                endpoint.declare_in(spec.req, &clients)?;
                endpoint.clock().set_bound_ns(if spec.required { u32::MAX } else { 0 });
            }
        }
        let guard = Arc::new(ClockGuard {
            evidence: Mutex::new(Evidence {
                estimator: ProbeEstimator::with_expiry(spec.window, Duration::from_nanos(spec.stale_after_ns as u64)),
                external: None, armed: false, failure: None,
            }), spec: spec.clone(), endpoint: endpoint.clone(),
        });
        endpoint.set_clock_degraded(spec.required || spec.role == ClockRole::Client);
        Ok(Self {
            interval: Duration::from_nanos(spec.interval_ns as u64), spec, node, endpoint,
            next_probe: Instant::now(), nonce: 0, pending: VecDeque::new(), guard,
            next_external: Instant::now(), external_pending: None,
            log: OpenOptions::new().create(true).append(true).open(log_path).ok(),
        })
    }

    pub(crate) fn guard(&self) -> Arc<ClockGuard> { self.guard.clone() }
    pub fn is_client(&self) -> bool { self.spec.role == ClockRole::Client }
    pub fn required(&self) -> bool { self.spec.required }
    pub fn estimate(&self) -> Option<Estimate> { self.guard.evidence.lock().unwrap().estimator.estimate() }
    pub fn within_gate(&self) -> bool { self.gate_reason().is_none() }
    pub fn gate_reason(&self) -> Option<String> { self.guard.reason(&self.guard.evidence.lock().unwrap(), Instant::now()) }
    pub fn gate_timeout(&self) -> Duration { Duration::from_nanos(self.spec.gate_timeout_ns as u64) }

    pub fn next_due(&self) -> Option<Instant> {
        let probe = self.is_client().then_some(self.next_probe);
        let external = self.spec.chrony_source.as_ref().map(|_| {
            if self.external_pending.is_some() { Instant::now() + Duration::from_millis(10) } else { self.next_external }
        });
        probe.into_iter().chain(external).min()
    }

    /// External commands run off the scheduler; their own wall timeout is
    /// bounded. One in-flight observation only, and no stale-result fallback.
    fn sample_external(&mut self, now: Instant) {
        if let Some(rx) = self.external_pending.as_ref() {
            match rx.try_recv() {
                Ok((at, result)) => {
                    if let Some(log) = self.log.as_mut() {
                        let line = serde_json::json!({"event":"external_clock_observation", "t":self.endpoint.clock().now(),
                            "age_ms":at.elapsed().as_millis(), "source":self.spec.chrony_source,
                            "observation":result.as_ref().ok().map(|c| serde_json::json!({"reference":c.reference,"offset_s":c.system_offset_s,"bound_ns":c.bound_ns(),"leap":c.leap_status})),
                            "error":result.as_ref().err()});
                        let _ = writeln!(log, "{line}");
                    }
                    self.guard.evidence.lock().unwrap().external = Some((at, result));
                    self.external_pending = None;
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    self.guard.evidence.lock().unwrap().external = Some((now, Err("chrony observation worker failed".into())));
                    self.external_pending = None;
                }
                Err(mpsc::TryRecvError::Empty) => {},
            }
        }
        if self.external_pending.is_none() && now >= self.next_external {
            if let Some(source) = self.spec.chrony_source.clone() {
                let (offset, uncertainty) = (self.spec.chrony_max_offset_ns, self.spec.chrony_max_uncertainty_ns);
                let (tx, rx) = mpsc::channel();
                self.external_pending = Some(rx);
                self.next_external = now + self.interval;
                std::thread::spawn(move || { let _ = tx.send((now, ChronyTracking::read_checked(&source, offset, uncertainty))); });
            }
        }
    }

    /// Startup probes use the faster of the configured interval and 100 ms;
    /// this clock-service cadence does not alter Session round scheduling.
    pub fn tick(&mut self, fast: bool) {
        let now = Instant::now();
        let _ = self.guard.failure();
        self.sample_external(now);
        self.pending.retain(|(_, at, _)| now.saturating_duration_since(*at) < self.guard.stale_after());
        if !self.is_client() || now < self.next_probe { return; }
        self.nonce = self.nonce.checked_add(1).expect("clock nonce exhausted");
        if let Ok(header) = self.endpoint.publish(self.spec.req, 0, self.endpoint.clock().now(), &self.nonce.to_le_bytes()) {
            self.pending.push_back((self.nonce, now, header.t_tx));
        }
        // Bound outstanding requests even with unusually long configured TTLs.
        while self.pending.len() > 4096 { self.pending.pop_front(); }
        self.next_probe = now + if fast {
            self.interval.min(Duration::from_millis(CLOCK_STARTUP_PROBE_MAX_INTERVAL_MS))
        } else { self.interval };
    }

    pub fn on_frame(&mut self, f: &RxFrame) -> bool {
        let _ = self.guard.failure();
        let h = &f.header;
        if h.channel == self.spec.req && self.spec.role == ClockRole::Server {
            if f.payload.len() == 8 {
                let mut rep = Vec::with_capacity(32);
                rep.extend_from_slice(&h.origin.to_le_bytes());
                rep.extend_from_slice(&[0u8; 6]);
                rep.extend_from_slice(&f.payload);
                rep.extend_from_slice(&h.t_tx.to_le_bytes());
                rep.extend_from_slice(&f.t_rx.to_le_bytes());
                let _ = self.endpoint.publish(self.spec.rep, 0, self.endpoint.clock().now(), &rep);
            }
            return true;
        }
        if h.channel == self.spec.rep && self.is_client() {
            let p = &f.payload;
            if h.origin == self.spec.server && p.len() == 32 && u16::from_le_bytes([p[0], p[1]]) == self.node {
                let nonce = u64::from_le_bytes(p[8..16].try_into().unwrap());
                let Some(index) = self.pending.iter().position(|(n, _, _)| *n == nonce) else { return true; };
                let (_, sent, t_tx) = self.pending.remove(index).unwrap();
                let now = Instant::now();
                let i64_at = |o: usize| i64::from_le_bytes(p[o..o + 8].try_into().unwrap());
                let sample = ProbeSample { t1: i64_at(16), t2: i64_at(24), t3: h.t_tx, t4: f.t_rx };
                let (accepted, estimate) = {
                    let mut e = self.guard.evidence.lock().unwrap();
                    let accepted = sample.t1 == t_tx && now.saturating_duration_since(sent) < self.guard.stale_after()
                        && (!self.spec.required || (h.clock_bound_ns != u32::MAX && h.flags & xgc_rt_core::envelope::FLAG_CLOCK_DEGRADED == 0))
                        && e.estimator.add_at(sample, now);
                    self.guard.refresh(&e, now);
                    (accepted, e.estimator.estimate_at(now))
                };
                self.record(sample, accepted, estimate);
            }
            return true;
        }
        h.channel == self.spec.req || h.channel == self.spec.rep
    }

    fn record(&mut self, s: ProbeSample, accepted: bool, estimate: Option<Estimate>) {
        let Some(log) = self.log.as_mut() else { return; };
        let e = self.guard.evidence.lock().unwrap();
        let line = serde_json::json!({
            "t": s.t4, "t1": s.t1, "t2": s.t2, "t3": s.t3, "t4": s.t4, "accepted": accepted,
            "estimate": estimate.map(|v| serde_json::json!({"offset_ns":v.offset_ns,"delay_ns":v.delay_ns,"bound_ns":v.bound_ns,"samples":v.samples})),
            "chrony": e.external.as_ref().map(|(at, result)| serde_json::json!({"age_ms":at.elapsed().as_millis(),"result":format!("{result:?}")})),
        });
        let _ = writeln!(log, "{line}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use xgc_rt_core::audit::NullAudit;
    use xgc_rt_core::clock::WallClock;
    use xgc_rt_core::envelope::Header;
    use xgc_rt_core::manifest::Manifest;
    use xgc_rt_core::transport::TransportContext;
    use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

    fn service() -> ClockService {
        let manifest = Manifest::from_toml_str(r#"
[session]
id="clock-test"
node="client"
roster=["server","client"]
period_ms=10
epoch_ns=1000000000
[transport]
kind="loopback"
[audit]
dir="unused"
[clock]
role="client"
server="server"
required=true
chrony_source="station"
interval_ms=100
stale_after_ms=500
"#).unwrap();
        let resolved = manifest.resolve().unwrap();
        let ctx = TransportContext { session: "clock-test".into(), node: "client".into(), node_id: 1,
            roster: manifest.session.roster, channels: resolved.channels };
        let endpoint = Endpoint::open(Box::new(LoopbackTransport::new(LoopbackBus::new())), &ctx,
            Arc::new(WallClock::new(0)), Arc::new(NullAudit), 32).unwrap();
        ClockService::new(resolved.clock.unwrap(), 1, 2, endpoint, Path::new("/dev/null")).unwrap()
    }

    fn external() -> ChronyTracking {
        ChronyTracking { reference: "station".into(), system_offset_s: 0.0, root_delay_s: 0.0,
            root_dispersion_s: 0.0, leap_status: "Normal".into() }
    }

    fn reply(s: &ClockService, nonce: u64, t1: i64) -> RxFrame {
        let mut payload = Vec::new();
        payload.extend_from_slice(&s.node.to_le_bytes()); payload.extend_from_slice(&[0; 6]);
        payload.extend_from_slice(&nonce.to_le_bytes()); payload.extend_from_slice(&t1.to_le_bytes());
        payload.extend_from_slice(&(t1 + 1000).to_le_bytes());
        RxFrame { header: Header { flags: 0, channel: s.spec.rep, origin: s.spec.server,
            seq: nonce, round: 0, t_produce: t1 + 1000, t_tx: t1 + 1000, clock_bound_ns: 0, payload_len: 32 },
            payload, t_rx: t1 + 2000 }
    }

    #[test]
    fn replies_need_pending_nonce_server_and_original_transmit_stamp() {
        let mut s = service();
        assert_eq!(s.endpoint.clock().bound_ns(), u32::MAX);
        s.pending.push_back((1, Instant::now(), 100));
        let mut wrong_server = reply(&s, 1, 100); wrong_server.header.origin = 7;
        s.on_frame(&wrong_server); s.on_frame(&reply(&s, 2, 100));
        assert!(s.estimate().is_none());
        s.on_frame(&reply(&s, 1, 100));
        assert_eq!(s.estimate().unwrap().samples, 1);
        s.on_frame(&reply(&s, 1, 100));
        assert_eq!(s.estimate().unwrap().samples, 1, "duplicate reply refreshed the window");
        s.pending.push_back((3, Instant::now(), 300));
        s.on_frame(&reply(&s, 3, 301));
        assert_eq!(s.estimate().unwrap().samples, 1, "mismatched t1 accepted");
        s.pending.push_back((4, Instant::now() - Duration::from_secs(1), 400));
        s.on_frame(&reply(&s, 4, 400));
        assert_eq!(s.estimate().unwrap().samples, 1, "stale request accepted");
    }

    #[test]
    fn expired_probe_latches_even_if_fresh_evidence_arrives_before_next_host_tick() {
        let mut s = service();
        s.guard.evidence.lock().unwrap().external = Some((Instant::now(), Ok(external())));
        for nonce in 1..=3 {
            s.pending.push_back((nonce, Instant::now(), nonce as i64 * 10000));
            s.on_frame(&reply(&s, nonce, nonce as i64 * 10000));
        }
        s.guard.admit().unwrap();
        assert!(s.guard.runnable());
        {
            let mut e = s.guard.evidence.lock().unwrap();
            e.estimator = ProbeEstimator::with_expiry(8, Duration::from_millis(500));
            for n in 0..3 { e.estimator.add_at(ProbeSample { t1:n, t2:n+1, t3:n+1, t4:n+2 }, Instant::now() - Duration::from_secs(1)); }
        }
        s.pending.push_back((4, Instant::now(), 40000));
        s.on_frame(&reply(&s, 4, 40000));
        assert!(!s.guard.runnable());
        assert_eq!(s.endpoint.clock().bound_ns(), u32::MAX);
        for nonce in 5..=8 {
            s.pending.push_back((nonce, Instant::now(), nonce as i64 * 10000));
            s.on_frame(&reply(&s, nonce, nonce as i64 * 10000));
        }
        assert!(!s.guard.runnable(), "late recovery must require a new Session");
    }

    #[test]
    fn startup_requires_three_samples_then_the_frozen_expiry_controls_validity() {
        let s = service();
        {
            let mut e = s.guard.evidence.lock().unwrap();
            e.external = Some((Instant::now(), Ok(external())));
            e.estimator.add(ProbeSample { t1:0, t2:1, t3:1, t4:2 });
        }
        assert!(s.guard.admit().is_err());
        {
            let mut e = s.guard.evidence.lock().unwrap();
            for n in 1..3 { e.estimator.add(ProbeSample { t1:n, t2:n+1, t3:n+1, t4:n+2 }); }
        }
        s.guard.admit().unwrap();
        {
            let mut e = s.guard.evidence.lock().unwrap();
            e.estimator = ProbeEstimator::with_expiry(8, Duration::from_millis(500));
            e.estimator.add(ProbeSample { t1:10, t2:11, t3:11, t4:12 });
        }
        assert!(s.guard.runnable(), "periodic replies inside frozen TTL must retain established admission");
    }

    #[test]
    fn expired_external_evidence_is_not_rescued_by_a_new_observation() {
        let s = service();
        {
            let mut e = s.guard.evidence.lock().unwrap();
            e.external = Some((Instant::now(), Ok(external())));
            for n in 0..3 { e.estimator.add(ProbeSample { t1:n, t2:n+1, t3:n+1, t4:n+2 }); }
        }
        s.guard.admit().unwrap();
        s.guard.evidence.lock().unwrap().external = Some((Instant::now()-Duration::from_secs(1), Ok(external())));
        assert!(!s.guard.runnable());
        s.guard.evidence.lock().unwrap().external = Some((Instant::now(), Ok(external())));
        assert!(!s.guard.runnable());
        assert!(s.guard.failure().unwrap().contains("external clock evidence expired"));
    }
}
