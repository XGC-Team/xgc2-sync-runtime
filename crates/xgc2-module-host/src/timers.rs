//! Period timers: one thread, one deadline heap.
//!
//! The heap holds one entry per armed instance. An entry fires at `anchor + n * period`
//! (drift-free); if the host fell behind, the periods that were skipped are counted as merged
//! and the next deadline lies in the future, so a slow step never causes a burst of catch-up
//! steps. In `steady` mode the timer thread sleeps until the earliest deadline. In `external`
//! mode time only advances when the clock channel is committed; the committing thread then
//! calls [`Timers::on_clock`], which fires whatever became due, so simulated time can run
//! faster or slower than the wall clock.
//!
//! The same thread ticks the scheduler's watchdog every [`WATCHDOG_TICK_NS`].

use crate::clock::{steady_ns, Clock, Mode, Update};
use crate::scheduler::Scheduler;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::time::Duration;

pub const WATCHDOG_TICK_NS: i64 = 10_000_000;
/// Shortest period a module may request.
pub const MIN_PERIOD_NS: i64 = 100_000;

#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Entry {
    deadline: i64,
    idx: u32,
    epoch: u64,
}

#[derive(Clone, Copy)]
struct Slot {
    generation: u32,
    period: i64,
    epoch: u64,
}

struct State {
    heap: BinaryHeap<Reverse<Entry>>,
    slots: Vec<Option<Slot>>,
    epoch: u64,
    stop: bool,
}

pub struct Timers {
    sched: Arc<Scheduler>,
    clock: Arc<Clock>,
    state: Mutex<State>,
    changed: Condvar,
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

impl Timers {
    pub fn new(sched: Arc<Scheduler>, clock: Arc<Clock>, slots: usize) -> Arc<Timers> {
        Arc::new(Timers {
            sched,
            clock,
            state: Mutex::new(State { heap: BinaryHeap::new(), slots: vec![None; slots], epoch: 0, stop: false }),
            changed: Condvar::new(),
        })
    }

    /// Arm the timer of `idx`; the first expiration is one period from now. With an external
    /// clock that has not published yet, the timer starts with the first sample.
    pub fn arm(&self, idx: u32, generation: u32, period_ns: i64) {
        let mut state = lock(&self.state);
        state.epoch += 1;
        let epoch = state.epoch;
        state.slots[idx as usize] = Some(Slot { generation, period: period_ns.max(MIN_PERIOD_NS), epoch });
        if self.clock.valid() {
            let deadline = self.clock.now_ns() + period_ns.max(MIN_PERIOD_NS);
            state.heap.push(Reverse(Entry { deadline, idx, epoch }));
            self.changed.notify_one();
        }
    }

    pub fn disarm(&self, idx: u32) {
        lock(&self.state).slots[idx as usize] = None;
    }

    pub fn is_armed(&self, idx: u32) -> bool {
        lock(&self.state).slots[idx as usize].is_some()
    }

    /// Fire everything due at `now` and schedule the next expirations.
    fn fire_due(&self, now: i64) {
        let mut fired: Vec<(u32, u32, u64)> = Vec::new();
        {
            let mut state = lock(&self.state);
            while let Some(Reverse(top)) = state.heap.peek().copied() {
                if top.deadline > now {
                    break;
                }
                state.heap.pop();
                let Some(slot) = state.slots[top.idx as usize] else { continue };
                if slot.epoch != top.epoch {
                    continue;
                }
                let skipped = (now - top.deadline) / slot.period;
                let deadline = top.deadline + (skipped + 1) * slot.period;
                state.heap.push(Reverse(Entry { deadline, idx: top.idx, epoch: top.epoch }));
                fired.push((top.idx, slot.generation, skipped as u64));
            }
        }
        for (idx, generation, merged) in fired {
            self.sched.fire_timer(idx, generation, merged);
        }
    }

    /// Restart every armed timer one period after `now`.
    fn reanchor(&self, now: i64) {
        let mut state = lock(&self.state);
        state.heap.clear();
        let mut epoch = state.epoch;
        for idx in 0..state.slots.len() {
            if let Some(slot) = state.slots[idx].as_mut() {
                epoch += 1;
                slot.epoch = epoch;
                let entry = Entry { deadline: now + slot.period, idx: idx as u32, epoch };
                state.heap.push(Reverse(entry));
            }
        }
        state.epoch = epoch;
    }

    /// External clock sample committed (called on the committing thread).
    pub fn on_clock(&self, update: Update, now: i64) {
        if update != Update::Forward {
            self.reanchor(now);
        }
        self.fire_due(now);
    }

    pub fn stop(&self) {
        lock(&self.state).stop = true;
        self.changed.notify_all();
    }

    /// Thread body: steady-clock deadlines and the watchdog tick.
    pub fn run(self: Arc<Self>) {
        // The default 50 us timer slack would delay every wake-up of this thread.
        // SAFETY: plain prctl call without pointers.
        unsafe { libc::prctl(libc::PR_SET_TIMERSLACK, 1u64, 0u64, 0u64, 0u64) };
        let mut next_tick = steady_ns() + WATCHDOG_TICK_NS;
        loop {
            let now = steady_ns();
            if self.clock.mode() == Mode::Steady {
                self.fire_due(now);
            }
            if now >= next_tick {
                self.sched.watchdog();
                next_tick = now + WATCHDOG_TICK_NS;
            }
            let mut wait = next_tick - now;
            let state = lock(&self.state);
            if state.stop {
                return;
            }
            if self.clock.mode() == Mode::Steady {
                if let Some(Reverse(top)) = state.heap.peek() {
                    wait = wait.min(top.deadline - now);
                }
            }
            if wait > 0 {
                let _ = self.changed.wait_timeout(state, Duration::from_nanos(wait as u64));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scheduler::{Runner, Taken, Worker, REASON_TIMER};
    use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
    use std::thread;
    use std::time::Instant;

    struct Recorder {
        stamps: Mutex<Vec<(u32, Instant)>>,
        slow_ms: AtomicU64,
        runs: AtomicUsize,
    }

    impl Runner for Recorder {
        fn run(&self, idx: u32, taken: Taken, _: &Worker) {
            assert!(taken.reasons & REASON_TIMER != 0);
            lock(&self.stamps).push((idx, Instant::now()));
            self.runs.fetch_add(1, Ordering::SeqCst);
            let slow = self.slow_ms.load(Ordering::SeqCst);
            if slow > 0 {
                thread::sleep(Duration::from_millis(slow));
            }
        }
        fn hung(&self, _: u32) {}
    }

    struct Fixture {
        sched: Arc<Scheduler>,
        timers: Arc<Timers>,
        clock: Arc<Clock>,
        recorder: Arc<Recorder>,
        thread: Option<thread::JoinHandle<()>>,
    }

    impl Fixture {
        fn new(mode: Mode) -> Fixture {
            let sched = Scheduler::new(2);
            let clock = Arc::new(Clock::new(mode));
            let timers = Timers::new(sched.clone(), clock.clone(), crate::scheduler::MAX_INSTANCES);
            let recorder = Arc::new(Recorder { stamps: Mutex::new(Vec::new()), slow_ms: AtomicU64::new(0), runs: AtomicUsize::new(0) });
            sched.start(recorder.clone());
            let runner = timers.clone();
            let thread = Some(thread::spawn(move || runner.run()));
            Fixture { sched, timers, clock, recorder, thread }
        }

        fn instance(&self, period_ns: i64) -> (u32, u32) {
            let (idx, generation) = self.sched.alloc().unwrap();
            self.sched.resume(idx);
            self.timers.arm(idx, generation, period_ns);
            (idx, generation)
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            self.timers.stop();
            if let Some(thread) = self.thread.take() {
                thread.join().unwrap();
            }
            self.sched.shutdown(Duration::from_secs(1));
        }
    }

    #[test]
    fn periodic_timer_fires_at_the_period_without_drift() {
        let f = Fixture::new(Mode::Steady);
        let (idx, _) = f.instance(5_000_000);
        thread::sleep(Duration::from_millis(250));
        f.timers.disarm(idx);
        let stamps: Vec<_> = lock(&f.recorder.stamps).iter().map(|(_, at)| *at).collect();
        // 250 ms / 5 ms = 50 expirations; allow scheduling noise on a shared machine.
        assert!((35..=51).contains(&stamps.len()), "{} expirations", stamps.len());
        let span = stamps.last().unwrap().duration_since(stamps[0]).as_secs_f64();
        let mean = span / (stamps.len() - 1) as f64;
        assert!((0.0045..0.0058).contains(&mean), "mean interval {mean}");
        // An expiration raised just before the disarm may still be running.
        thread::sleep(Duration::from_millis(30));
        let count = f.recorder.runs.load(Ordering::SeqCst);
        thread::sleep(Duration::from_millis(30));
        assert_eq!(f.recorder.runs.load(Ordering::SeqCst), count, "disarmed timer kept firing");
    }

    #[test]
    fn a_slow_task_merges_periods_instead_of_bursting() {
        let f = Fixture::new(Mode::Steady);
        f.recorder.slow_ms.store(30, Ordering::SeqCst);
        let (idx, _) = f.instance(5_000_000);
        thread::sleep(Duration::from_millis(200));
        f.timers.disarm(idx);
        let runs = f.recorder.runs.load(Ordering::SeqCst);
        assert!(runs <= 8, "{runs} runs; 30 ms steps cannot fit more than 7 into 200 ms");
        let merged = f.sched.cell(idx).missed_periods.load(Ordering::Relaxed);
        assert!(merged >= 20, "only {merged} periods were reported as missed");
    }

    #[test]
    fn set_period_replaces_the_old_schedule() {
        let f = Fixture::new(Mode::Steady);
        let (idx, generation) = f.instance(1_000_000_000);
        thread::sleep(Duration::from_millis(20));
        assert_eq!(f.recorder.runs.load(Ordering::SeqCst), 0);
        f.timers.arm(idx, generation, 5_000_000);
        thread::sleep(Duration::from_millis(100));
        assert!(f.recorder.runs.load(Ordering::SeqCst) >= 10);
    }

    #[test]
    fn external_clock_drives_the_timers() {
        let f = Fixture::new(Mode::External);
        let (idx, _) = f.instance(100_000_000);
        // No time has been published: nothing fires, however long we wait.
        thread::sleep(Duration::from_millis(60));
        assert_eq!(f.recorder.runs.load(Ordering::SeqCst), 0);
        let publish = |ns: i64| {
            let update = f.clock.set_external(ns);
            f.timers.on_clock(update, ns);
        };
        publish(1_000_000_000);
        // The first sample anchors the timer: the first expiration is one period later.
        thread::sleep(Duration::from_millis(30));
        assert_eq!(f.recorder.runs.load(Ordering::SeqCst), 0);
        publish(1_099_000_000);
        publish(1_100_000_000);
        wait_runs(&f, 1);
        // A jump of 5 periods fires once and reports four merged periods.
        publish(1_600_000_000);
        wait_runs(&f, 2);
        assert_eq!(f.sched.cell(idx).missed_periods.load(Ordering::Relaxed), 4);
        // Time moving backwards (simulation restart) re-anchors instead of waiting for 1.7 s.
        publish(50_000_000);
        publish(150_000_000);
        wait_runs(&f, 3);
    }

    fn wait_runs(f: &Fixture, expected: usize) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while f.recorder.runs.load(Ordering::SeqCst) < expected {
            assert!(Instant::now() < deadline, "expected {expected} runs, got {}", f.recorder.runs.load(Ordering::SeqCst));
            thread::sleep(Duration::from_millis(1));
        }
        thread::sleep(Duration::from_millis(20));
        assert_eq!(f.recorder.runs.load(Ordering::SeqCst), expected);
    }
}
