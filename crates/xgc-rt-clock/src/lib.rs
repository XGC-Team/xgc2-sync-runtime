//! Clock-error bound for Session time (docs/time-model.md).
//!
//! The host runs a probe to the station over the same transport and path as
//! data. The four stamps are the audit's own stamp points: t1 is the
//! request's `t_tx`, t2 its `t_rx` at the station, t3 the reply's `t_tx`, and
//! t4 its `t_rx`. For each probe:
//!
//! ```text
//! offset θ = ((t2 − t1) + (t3 − t4)) / 2      (station − local)
//! delay  δ = (t4 − t1) − (t3 − t2)            (round trip minus station hold)
//! ```
//!
//! The true offset lies within `θ ± δ/2` for *any* split of δ between the two
//! directions. So `|local − station| ≤ |θ| + δ/2` holds even on an
//! asymmetric path. The estimate uses the minimum-delay sample of a sliding
//! window (the NTP clock filter idea): its interval is the tightest.

use std::collections::VecDeque;
use std::io::Read;
use std::process::{Command, Stdio};
use std::time::{Duration, Instant};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProbeSample {
    pub t1: i64,
    pub t2: i64,
    pub t3: i64,
    pub t4: i64,
}

impl ProbeSample {
    pub fn offset_ns(&self) -> i64 {
        ((self.t2 - self.t1) + (self.t3 - self.t4)) / 2
    }

    pub fn delay_ns(&self) -> i64 {
        (self.t4 - self.t1) - (self.t3 - self.t2)
    }

    /// Upper bound on |local − station| implied by this sample.
    pub fn bound_ns(&self) -> i64 {
        self.offset_ns().abs() + (self.delay_ns().max(0) + 1) / 2
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Estimate {
    pub offset_ns: i64,
    pub delay_ns: i64,
    pub bound_ns: i64,
    pub samples: usize,
}

#[derive(Debug, Clone)]
pub struct ProbeEstimator {
    window: VecDeque<(Instant, ProbeSample)>,
    capacity: usize,
    stale_after: Duration,
}

impl ProbeEstimator {
    pub fn new(capacity: usize) -> Self {
        Self::with_expiry(capacity, Duration::from_secs(3))
    }

    pub fn with_expiry(capacity: usize, stale_after: Duration) -> Self {
        assert!(!stale_after.is_zero());
        Self { window: VecDeque::new(), capacity: capacity.max(1), stale_after }
    }

    /// Invalid/replayed replies must be rejected by the caller before add.
    /// Individual samples expire on steady time, including an old minimum-delay
    /// sample while newer, less accurate replies continue arriving.
    pub fn add(&mut self, s: ProbeSample) -> bool { self.add_at(s, Instant::now()) }

    pub fn add_at(&mut self, s: ProbeSample, received: Instant) -> bool {
        // Validate with wide arithmetic before the public i64 metrics are used.
        let delay = (s.t4 as i128 - s.t1 as i128) - (s.t3 as i128 - s.t2 as i128);
        let offset_sum = (s.t2 as i128 - s.t1 as i128) + (s.t3 as i128 - s.t4 as i128);
        if s.t3 < s.t2 || s.t4 < s.t1 || !(0..i64::MAX as i128).contains(&delay)
            || offset_sum.abs() > i64::MAX as i128
            || (offset_sum / 2).abs() + (delay + 1) / 2 > i64::MAX as i128
            || [s.t2 as i128 - s.t1 as i128, s.t3 as i128 - s.t4 as i128,
                s.t4 as i128 - s.t1 as i128, s.t3 as i128 - s.t2 as i128]
                .iter().any(|v| v.abs() > i64::MAX as i128) {
            return false;
        }
        self.window.retain(|(at, _)| received.saturating_duration_since(*at) < self.stale_after);
        if self.window.len() == self.capacity { self.window.pop_front(); }
        self.window.push_back((received, s));
        true
    }

    pub fn estimate(&self) -> Option<Estimate> { self.estimate_at(Instant::now()) }

    pub fn estimate_at(&self, now: Instant) -> Option<Estimate> {
        let fresh: Vec<_> = self.window.iter()
            .filter(|(at, _)| now.saturating_duration_since(*at) < self.stale_after)
            .map(|(_, sample)| sample).collect();
        let best = fresh.iter().min_by_key(|s| s.delay_ns())?;
        Some(Estimate { offset_ns: best.offset_ns(), delay_ns: best.delay_ns(), bound_ns: best.bound_ns(), samples: fresh.len() })
    }
}

/// `chronyc -c tracking` fields that the bound uses.
#[derive(Debug, Clone, PartialEq)]
pub struct ChronyTracking {
    pub reference: String,
    pub system_offset_s: f64,
    pub root_delay_s: f64,
    pub root_dispersion_s: f64,
    pub leap_status: String,
}

impl ChronyTracking {
    /// Parse one CSV line of `chronyc -c tracking`:
    /// ref-id, ref-name, stratum, ref-time, system-time offset, last offset,
    /// RMS offset, frequency, residual freq, skew, root delay, root
    /// dispersion, update interval, leap status.
    pub fn parse_csv(line: &str) -> Option<Self> {
        let f: Vec<&str> = line.trim().split(',').collect();
        if f.len() < 14 {
            return None;
        }
        let value = Self {
            reference: f[1].to_string(),
            system_offset_s: f[4].parse().ok()?,
            root_delay_s: f[10].parse().ok()?,
            root_dispersion_s: f[11].parse().ok()?,
            leap_status: f[13].to_string(),
        };
        if [value.system_offset_s, value.root_delay_s, value.root_dispersion_s].iter().any(|v| !v.is_finite())
            || value.root_delay_s < 0.0 || value.root_dispersion_s < 0.0 { return None; }
        Some(value)
    }

    /// chrony's maximum error: |offset| + root dispersion + root delay / 2.
    pub fn bound_ns(&self) -> i64 {
        ((self.system_offset_s.abs() + self.root_dispersion_s + self.root_delay_s / 2.0) * 1e9).ceil() as i64
    }

    /// Read-only observation of the external service, bounded in wall time.
    pub fn read() -> Option<Self> {
        Self::parse_csv(&chronyc(&["-c", "tracking"]).ok()?)
    }

    /// Existing external-clock contract: a selected frozen source, Normal leap,
    /// and separate offset/uncertainty bounds. This never adjusts system time.
    pub fn read_checked(source: &str, max_offset_ns: i64, max_uncertainty_ns: i64) -> Result<Self, String> {
        let tracking = Self::parse_csv(&chronyc(&["-c", "tracking"])?).ok_or("invalid chronyc tracking fields")?;
        tracking.check(&chronyc(&["sources", "-v"])?, source, max_offset_ns, max_uncertainty_ns)?;
        Ok(tracking)
    }

    pub fn check(&self, sources: &str, source: &str, max_offset_ns: i64, max_uncertainty_ns: i64) -> Result<(), String> {
        let selected = sources.lines().find_map(|line| {
            let mut fields = line.split_whitespace();
            let state = fields.next()?;
            (matches!(state, "^*" | "=*" | "#*")).then(|| fields.next()).flatten()
        }).ok_or("chrony has no selected external source")?;
        if source.is_empty() || selected != source { return Err(format!("chrony selected source {selected:?} differs from frozen chrony_source {source:?}")); }
        if self.leap_status != "Normal" { return Err(format!("chrony leap status is {:?}, requires Normal", self.leap_status)); }
        if [self.system_offset_s, self.root_delay_s, self.root_dispersion_s].iter().any(|v| !v.is_finite())
            || self.root_delay_s < 0.0 || self.root_dispersion_s < 0.0 { return Err("invalid chrony clock measurements".into()); }
        if self.system_offset_s.abs() * 1e9 > max_offset_ns as f64 { return Err("chrony offset exceeds frozen preflight gate".into()); }
        if (self.root_dispersion_s + self.root_delay_s / 2.0) * 1e9 > max_uncertainty_ns as f64 { return Err("chrony uncertainty exceeds frozen preflight gate".into()); }
        Ok(())
    }
}

/// Run in the service's observation thread, never the module/round scheduler.
/// Missing/hung chrony is failed evidence, not an implicit zero bound.
fn chronyc(args: &[&str]) -> Result<String, String> {
    let mut child = Command::new("chronyc").args(args).stdin(Stdio::null())
        .stdout(Stdio::piped()).stderr(Stdio::null()).spawn().map_err(|e| format!("chronyc unavailable: {e}"))?;
    let deadline = Instant::now() + Duration::from_millis(500);
    loop {
        match child.try_wait() {
            Ok(Some(status)) => {
                if !status.success() { return Err(format!("chronyc failed: {status}")); }
                let mut text = String::new();
                child.stdout.take().ok_or("chronyc stdout unavailable")?.take(65_537)
                    .read_to_string(&mut text).map_err(|e| format!("chronyc output: {e}"))?;
                if text.len() > 65_536 { return Err("chronyc output exceeds limit".into()); }
                return Ok(text);
            }
            Ok(None) if Instant::now() < deadline => std::thread::sleep(Duration::from_millis(5)),
            state => {
                let _ = child.kill();
                let _ = child.wait();
                return Err(match state { Err(e) => format!("chronyc wait: {e}"), _ => "chronyc timed out".into() });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn symmetric_path_recovers_the_offset_exactly() {
        // Station 5 ms ahead; 2 ms each way; station holds 0.1 ms.
        let s = ProbeSample { t1: 0, t2: 7_000_000, t3: 7_100_000, t4: 4_100_000 };
        assert_eq!(s.offset_ns(), 5_000_000);
        assert_eq!(s.delay_ns(), 4_000_000);
        assert_eq!(s.bound_ns(), 7_000_000);
    }

    #[test]
    fn the_bound_holds_for_any_asymmetry() {
        let true_offset = -3_000_000i64; // station 3 ms behind local
        for (fwd, back) in [(0, 40_000_000), (40_000_000, 0), (1_000_000, 9_000_000), (20_000_000, 20_000_000)] {
            let t1 = 1_000_000_000i64;
            let t2 = t1 + fwd + true_offset;
            let t3 = t2 + 50_000;
            let t4 = t3 - true_offset + back;
            let s = ProbeSample { t1, t2, t3, t4 };
            assert!(s.bound_ns() >= true_offset.abs(), "fwd {fwd} back {back}: bound {} < |offset| {}", s.bound_ns(), true_offset.abs());
            assert_eq!(s.delay_ns(), fwd + back);
        }
    }

    #[test]
    fn estimator_uses_the_minimum_delay_sample_and_rejects_inconsistent_ones() {
        let mut e = ProbeEstimator::new(3);
        assert!(e.add(ProbeSample { t1: 0, t2: 10, t3: 10, t4: 30 }));
        assert!(e.add(ProbeSample { t1: 100, t2: 102, t3: 102, t4: 104 }));
        assert!(!e.add(ProbeSample { t1: 0, t2: 10, t3: 30, t4: 5 }), "hold longer than the round trip: negative delay");
        let est = e.estimate().unwrap();
        assert_eq!((est.delay_ns, est.samples), (4, 2));
        for i in 0..3 {
            e.add(ProbeSample { t1: 1000 + i, t2: 1100 + i, t3: 1100 + i, t4: 1200 + i });
        }
        assert_eq!(e.estimate().unwrap().delay_ns, 200, "window slid past the tight sample");
    }

    #[test]
    fn parses_chronyc_csv_tracking() {
        let line = "C0A80001,192.168.0.1,3,1790360000.123,-0.000012345,0.000001,0.000020,-3.5,0.001,0.02,0.000800000,0.000150000,64.2,Normal\n";
        let t = ChronyTracking::parse_csv(line).unwrap();
        assert_eq!(t.reference, "192.168.0.1");
        assert_eq!(t.leap_status, "Normal");
        assert_eq!(t.bound_ns(), 562_345);
        assert!(ChronyTracking::parse_csv("short,line").is_none());
    }
}

#[cfg(test)]
mod freshness_tests {
    use super::*;

    #[test]
    fn every_sample_expires_even_when_new_replies_keep_arriving() {
        let start = Instant::now();
        let mut estimator = ProbeEstimator::with_expiry(8, Duration::from_millis(100));
        assert!(estimator.add_at(ProbeSample { t1: 0, t2: 1, t3: 1, t4: 2 }, start));
        assert!(estimator.add_at(ProbeSample { t1: 100, t2: 120, t3: 120, t4: 140 }, start + Duration::from_millis(90)));
        assert_eq!(estimator.estimate_at(start + Duration::from_millis(99)).unwrap().bound_ns, 1);
        let fresh = estimator.estimate_at(start + Duration::from_millis(100)).unwrap();
        assert_eq!((fresh.bound_ns, fresh.samples), (20, 1));
        assert!(estimator.estimate_at(start + Duration::from_millis(190)).is_none());
    }

    #[test]
    fn external_clock_requires_exact_selected_source_normal_leap_and_both_limits() {
        let good = ChronyTracking { reference: "station".into(), system_offset_s: 0.00001,
            root_delay_s: 0.00002, root_dispersion_s: 0.00003, leap_status: "Normal".into() };
        assert!(good.check("^* station 2 6 377 1", "station", 2_000_000, 2_000_000).is_ok());
        for sources in ["", "^? station 2 6 0 0", "^* station-copy 2 6 377 1"] {
            assert!(good.check(sources, "station", 2_000_000, 2_000_000).is_err());
        }
        let mut bad = good.clone(); bad.leap_status = "Not synchronised".into();
        assert!(bad.check("^* station", "station", 2_000_000, 2_000_000).is_err());
        let mut bad = good.clone(); bad.system_offset_s = 0.003;
        assert!(bad.check("^* station", "station", 2_000_000, 2_000_000).unwrap_err().contains("offset"));
        let mut bad = good; bad.root_dispersion_s = 0.003;
        assert!(bad.check("^* station", "station", 2_000_000, 2_000_000).unwrap_err().contains("uncertainty"));
        assert!(ChronyTracking::parse_csv("x,station,3,0,NaN,0,0,0,0,0,0,0,1,Normal").is_none());
    }
}
