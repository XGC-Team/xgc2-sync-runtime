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
    window: VecDeque<ProbeSample>,
    capacity: usize,
}

impl ProbeEstimator {
    pub fn new(capacity: usize) -> Self {
        Self { window: VecDeque::new(), capacity: capacity.max(1) }
    }

    /// Add a completed probe. A sample with negative delay is inconsistent
    /// (a clock stepped mid-probe) and is rejected.
    pub fn add(&mut self, s: ProbeSample) -> bool {
        if s.delay_ns() < 0 || s.t3 < s.t2 || s.t4 < s.t1 {
            return false;
        }
        if self.window.len() == self.capacity {
            self.window.pop_front();
        }
        self.window.push_back(s);
        true
    }

    pub fn estimate(&self) -> Option<Estimate> {
        let best = self.window.iter().min_by_key(|s| s.delay_ns())?;
        Some(Estimate { offset_ns: best.offset_ns(), delay_ns: best.delay_ns(), bound_ns: best.bound_ns(), samples: self.window.len() })
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
        Some(Self {
            reference: f[1].to_string(),
            system_offset_s: f[4].parse().ok()?,
            root_delay_s: f[10].parse().ok()?,
            root_dispersion_s: f[11].parse().ok()?,
            leap_status: f[13].to_string(),
        })
    }

    /// chrony's maximum error: |offset| + root dispersion + root delay / 2.
    pub fn bound_ns(&self) -> i64 {
        ((self.system_offset_s.abs() + self.root_dispersion_s + self.root_delay_s / 2.0) * 1e9).ceil() as i64
    }

    /// Run `chronyc -c tracking`; None when chrony is not installed or fails.
    pub fn read() -> Option<Self> {
        let out = std::process::Command::new("chronyc").args(["-c", "tracking"]).output().ok()?;
        if !out.status.success() {
            return None;
        }
        Self::parse_csv(&String::from_utf8_lossy(&out.stdout))
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
