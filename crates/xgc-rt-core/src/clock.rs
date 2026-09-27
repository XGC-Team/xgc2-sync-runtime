//! Session time and rounds (docs/time-model.md).
//!
//! Every module and host reads time only through [`Clock`]. `Wall` is the
//! chrony/PTP-disciplined host clock, used for physical and hybrid runs.
//! `Sim` is one simulator time authority. Local loop durations use
//! `std::time::Instant` and never Session time.

use std::sync::atomic::{AtomicI64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use std::sync::Mutex;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ClockDomain {
    Wall,
    Sim,
}

#[derive(Debug, Clone, Copy)]
pub struct DispatchStamp {
    pub generation: u64,
    pub time: i64,
    pub runnable: bool,
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
    /// Production source dispatch generation; None preserves ordinary clocks.
    fn dispatch_stamp(&self) -> Option<DispatchStamp> { None }
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

/// One externally owned simulator authority. Receipt and liveness use steady
/// time; accepted Session time is never extrapolated or rewritten as Unix time.
#[derive(Debug)]
pub struct SourceClock {
    state: Mutex<SourceState>,
    stale: Duration,
    max_advance_ns: i64,
}
#[derive(Debug)]
struct SourceState {
    time: Option<i64>,
    committed: Option<i64>,
    generation: u64,
    last_advance: Option<Instant>,
    armed: bool,
    fault: Option<String>,
}
#[derive(Debug, Clone)]
pub struct SourceSnapshot {
    pub time: Option<i64>,
    pub generation: u64,
    pub runnable: bool,
    pub fault: Option<String>,
}
impl SourceClock {
    pub fn new(stale: Duration, max_advance_ns: i64) -> Self {
        assert!(!stale.is_zero() && max_advance_ns > 0);
        Self { state: Mutex::new(SourceState { time: None, committed: None, generation: 0, last_advance: None, armed: false, fault: None }), stale, max_advance_ns }
    }
    pub fn observe(&self, time: i64) -> Result<bool, String> {
        let mut s = self.state.lock().unwrap();
        if let Some(f) = &s.fault { return Err(f.clone()); }
        let reference = if s.armed { s.committed.or(s.time) } else { s.time };
        let bad = if time > i64::MAX - 10_000_000_000 { Some("simulator time exceeds safe scheduling range") }
            else if time < 0 { Some("negative simulator time") }
            else if s.time.is_some_and(|old| time < old) { Some("simulator time moved backward; new Session required") }
            else if reference.is_some_and(|old| time - old > self.max_advance_ns) { Some("simulator time advance exceeds frozen limit; new Session required") }
            else { None };
        if let Some(f) = bad { s.fault = Some(f.into()); s.armed = false; return Err(f.into()); }
        let advance = s.time != Some(time);
        if advance {
            s.time = Some(time);
            s.generation += 1;
            s.last_advance = Some(Instant::now());
        }
        Ok(advance)
    }
    pub fn unavailable(&self) { self.state.lock().unwrap().last_advance = None; }
    pub fn commit_host_time(&self) { let mut s = self.state.lock().unwrap(); s.committed = s.time; }
    pub fn arm(&self, armed: bool) { let mut s = self.state.lock().unwrap(); s.armed = armed; if armed { s.committed = s.time; } }
    pub fn fail(&self, reason: impl Into<String>) {
        let mut s = self.state.lock().unwrap();
        if s.fault.is_none() { s.fault = Some(reason.into()); }
        s.armed = false;
    }
    pub fn snapshot(&self) -> SourceSnapshot {
        let s = self.state.lock().unwrap();
        SourceSnapshot { time: s.time, generation: s.generation,
            runnable: s.armed && s.fault.is_none() && s.last_advance.is_some_and(|t| t.elapsed() < self.stale),
            fault: s.fault.clone() }
    }
}
impl Clock for SourceClock {
    fn now(&self) -> i64 { self.state.lock().unwrap().time.unwrap_or(0) }
    fn bound_ns(&self) -> u32 { u32::MAX }
    fn domain(&self) -> ClockDomain { ClockDomain::Sim }
    fn dispatch_stamp(&self) -> Option<DispatchStamp> {
        let s = self.snapshot();
        Some(DispatchStamp { generation: s.generation, time: s.time.unwrap_or(0), runnable: s.runnable })
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

#[cfg(test)]
mod source_tests {
    use super::*;
    #[test]
    fn source_zero_pause_duplicate_resume_and_unknown_bound() {
        let c = SourceClock::new(Duration::from_millis(15), 100);
        assert_eq!(c.snapshot().time, None);
        assert!(!c.snapshot().runnable);
        assert!(c.observe(0).unwrap());
        assert_eq!(c.snapshot().time, Some(0));
        c.arm(true);
        let generation = c.snapshot().generation;
        assert!(c.snapshot().runnable);
        std::thread::sleep(Duration::from_millis(20));
        assert!(!c.observe(0).unwrap());
        assert!(
            !c.snapshot().runnable,
            "duplicate samples cannot freshen frozen time"
        );
        assert_eq!(c.snapshot().generation, generation);
        assert!(c.observe(1).unwrap());
        assert!(c.snapshot().runnable);
        assert_eq!(c.bound_ns(), u32::MAX);
    }
    #[test]
    fn source_backward_latches_and_preserves_last_valid_time() {
        let c = SourceClock::new(Duration::from_secs(1), 10);
        c.observe(50).unwrap();
        c.arm(true);
        assert!(c.observe(49).unwrap_err().contains("backward"));
        assert!(c.observe(51).is_err());
        assert_eq!(c.now(), 50);
        assert!(!c.snapshot().runnable);
    }
    #[test]
    fn source_no_fast_backlog_replay_beyond_last_host_commit() {
        let c = SourceClock::new(Duration::from_secs(1), 10);
        c.observe(50).unwrap();
        c.arm(true);
        c.observe(56).unwrap();
        assert!(
            c.observe(62).is_err(),
            "two individually small jumps exceed the host commit limit"
        );
        assert_eq!(c.now(), 56);
    }
}
