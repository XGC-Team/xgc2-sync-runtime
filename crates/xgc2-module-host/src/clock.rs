//! The host clock that `now_ns()`, step contexts, producer stamps and period timers share.
//!
//! `steady` reads CLOCK_MONOTONIC. `external` follows one designated state channel whose
//! payload is an `i64` nanosecond time (schema [`CLOCK_SCHEMA`]); in simulation the entity's
//! ROS edge module publishes `/clock` there. Timers then run on that time, so a simulation
//! that runs faster or slower than real time drives the modules at the same ratio.

use std::sync::atomic::{AtomicBool, AtomicI64, Ordering};

/// Schema id of the clock channel; the payload is one native-endian `int64_t`.
pub const CLOCK_SCHEMA: &str = "xgc2.clock.v1";
pub const CLOCK_PAYLOAD_SIZE: u32 = 8;
pub const CLOCK_PAYLOAD_ALIGN: u32 = 8;

/// CLOCK_MONOTONIC in nanoseconds.
pub fn steady_ns() -> i64 {
    let mut ts = libc::timespec { tv_sec: 0, tv_nsec: 0 };
    // SAFETY: `ts` is a valid out pointer; CLOCK_MONOTONIC always exists on Linux.
    unsafe { libc::clock_gettime(libc::CLOCK_MONOTONIC, &mut ts) };
    ts.tv_sec as i64 * 1_000_000_000 + ts.tv_nsec as i64
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Steady,
    External,
}

impl Mode {
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Steady => "steady",
            Mode::External => "external",
        }
    }
}

/// How an external sample relates to the previous one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Update {
    /// First sample: the clock became valid.
    First,
    Forward,
    /// Time moved backwards (simulation restart); timers must be re-anchored.
    Backward,
}

pub struct Clock {
    mode: Mode,
    external_ns: AtomicI64,
    valid: AtomicBool,
}

impl Clock {
    pub fn new(mode: Mode) -> Self {
        Self { mode, external_ns: AtomicI64::new(0), valid: AtomicBool::new(mode == Mode::Steady) }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// False until an external clock received its first sample.
    pub fn valid(&self) -> bool {
        self.valid.load(Ordering::Acquire)
    }

    pub fn now_ns(&self) -> i64 {
        match self.mode {
            Mode::Steady => steady_ns(),
            Mode::External => self.external_ns.load(Ordering::Acquire),
        }
    }

    /// Record an external sample. Only the clock channel's single writer calls this.
    pub fn set_external(&self, ns: i64) -> Update {
        let previous = self.external_ns.swap(ns, Ordering::AcqRel);
        if !self.valid.swap(true, Ordering::AcqRel) {
            Update::First
        } else if ns < previous {
            Update::Backward
        } else {
            Update::Forward
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn steady_clock_is_monotonic_and_valid() {
        let clock = Clock::new(Mode::Steady);
        assert!(clock.valid());
        let a = clock.now_ns();
        let b = clock.now_ns();
        assert!(b >= a && a > 0);
    }

    #[test]
    fn external_clock_reports_updates() {
        let clock = Clock::new(Mode::External);
        assert!(!clock.valid());
        assert_eq!(clock.now_ns(), 0);
        assert_eq!(clock.set_external(10), Update::First);
        assert!(clock.valid());
        assert_eq!(clock.set_external(20), Update::Forward);
        assert_eq!(clock.now_ns(), 20);
        assert_eq!(clock.set_external(5), Update::Backward);
        assert_eq!(clock.now_ns(), 5);
    }
}
