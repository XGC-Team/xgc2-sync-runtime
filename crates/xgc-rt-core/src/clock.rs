//! Session time and rounds (docs/time-model.md).
//!
//! Every module and host reads time only through [`Clock`]. `Wall` is the
//! chrony/PTP-disciplined host clock, used for physical and hybrid runs.
//! `Sim` is one simulator time authority. Local loop durations use
//! `std::time::Instant` and never Session time.

use std::sync::atomic::{AtomicI64, Ordering};
use std::time::{SystemTime, UNIX_EPOCH};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockDomain {
    Wall,
    Sim,
}

pub trait Clock: Send + Sync {
    /// Session time in nanoseconds.
    fn now(&self) -> i64;
    /// Current bound on |this clock − Session reference|, in ns. `u32::MAX`
    /// means unknown. A one-way delay is only as good as the sender's bound
    /// plus the receiver's bound.
    fn bound_ns(&self) -> u32;
    fn domain(&self) -> ClockDomain;
    /// Update the bound from a measurement (probe, chrony). Clocks with a
    /// fixed bound ignore it.
    fn set_bound_ns(&self, _bound_ns: u32) {}
}

/// The host's disciplined `CLOCK_REALTIME`. In Z1 the bound is supplied by
/// the caller: 0 when every node shares one host clock, as in single-process
/// loopback. From Z2 on it comes from chrony and the in-band probe.
#[derive(Debug)]
pub struct WallClock {
    bound_ns: std::sync::atomic::AtomicU32,
}

impl WallClock {
    pub fn new(bound_ns: u32) -> Self {
        Self { bound_ns: std::sync::atomic::AtomicU32::new(bound_ns) }
    }
}

impl Clock for WallClock {
    fn now(&self) -> i64 {
        let since = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default();
        i64::try_from(since.as_nanos()).unwrap_or(i64::MAX)
    }

    fn bound_ns(&self) -> u32 {
        self.bound_ns.load(Ordering::Relaxed)
    }

    fn domain(&self) -> ClockDomain {
        ClockDomain::Wall
    }

    fn set_bound_ns(&self, bound_ns: u32) {
        self.bound_ns.store(bound_ns, Ordering::Relaxed);
    }
}

/// The host clock displaced by a fixed offset: a stand-in for a node whose
/// clock is off, so probe and bound can be tested on one machine.
#[derive(Debug)]
pub struct SkewedClock {
    wall: WallClock,
    offset_ns: i64,
}

impl SkewedClock {
    pub fn new(offset_ns: i64) -> Self {
        Self { wall: WallClock::new(u32::MAX), offset_ns }
    }
}

impl Clock for SkewedClock {
    fn now(&self) -> i64 {
        self.wall.now() + self.offset_ns
    }

    fn bound_ns(&self) -> u32 {
        self.wall.bound_ns()
    }

    fn domain(&self) -> ClockDomain {
        ClockDomain::Wall
    }

    fn set_bound_ns(&self, bound_ns: u32) {
        self.wall.set_bound_ns(bound_ns);
    }
}

/// A clock advanced explicitly: the simulator authority and deterministic
/// tests. Stamps taken from one authority have a bound of 0.
#[derive(Debug, Default)]
pub struct ManualClock {
    now: AtomicI64,
}

impl ManualClock {
    pub fn new(start: i64) -> Self {
        Self { now: AtomicI64::new(start) }
    }

    pub fn set(&self, t: i64) {
        self.now.store(t, Ordering::SeqCst);
    }

    pub fn advance(&self, dt: i64) -> i64 {
        self.now.fetch_add(dt, Ordering::SeqCst) + dt
    }
}

impl Clock for ManualClock {
    fn now(&self) -> i64 {
        self.now.load(Ordering::SeqCst)
    }

    fn bound_ns(&self) -> u32 {
        0
    }

    fn domain(&self) -> ClockDomain {
        ClockDomain::Sim
    }
}

/// Rounds derived locally from a shared epoch (runtime-sync ADR 0003):
/// round `k` starts at `e0 + k·period` on every node, with no network tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RoundSchedule {
    pub e0: i64,
    pub period: i64,
    /// Offset from round start by which a sample for that round must be sent.
    pub publish_deadline: i64,
}

impl RoundSchedule {
    pub fn new(e0: i64, period: i64, publish_deadline: i64) -> Self {
        assert!(period > 0, "round period must be positive");
        Self { e0, period, publish_deadline }
    }

    /// Round in progress at `t`, or `None` before the epoch.
    pub fn round_at(&self, t: i64) -> Option<u64> {
        (t >= self.e0).then(|| ((t - self.e0) / self.period) as u64)
    }

    pub fn start(&self, k: u64) -> i64 {
        self.e0 + (k as i64) * self.period
    }

    pub fn deadline(&self, k: u64) -> i64 {
        self.start(k) + self.publish_deadline
    }

    /// The first round boundary strictly after `t` (the epoch itself before it).
    pub fn next_boundary_after(&self, t: i64) -> i64 {
        match self.round_at(t) {
            None => self.e0,
            Some(k) => self.start(k + 1),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounds_are_derived_from_the_epoch_only() {
        let s = RoundSchedule::new(1_000, 50, 20);
        assert_eq!(s.round_at(999), None);
        assert_eq!(s.round_at(1_000), Some(0));
        assert_eq!(s.round_at(1_049), Some(0));
        assert_eq!(s.round_at(1_050), Some(1));
        assert_eq!(s.deadline(2), 1_120);
        assert_eq!(s.next_boundary_after(0), 1_000);
        assert_eq!(s.next_boundary_after(1_050), 1_100);
    }
}
