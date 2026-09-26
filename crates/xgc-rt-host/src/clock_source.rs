//! Host-owned, steady-time polling of a pinned native time source.
use crate::{clock_source_abi::*, host::HealthLog, plugin::LoadedPlugin};
use std::ffi::{c_char, CString};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc, Mutex,
};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};
use xgc_rt_core::{clock::SourceClock, manifest::ClockSourceSpec};

pub struct ClockSource {
    pub clock: Arc<SourceClock>,
    spec: ClockSourceSpec,
    library: Arc<LoadedPlugin>,
    config: CString,
    stop: Arc<AtomicBool>,
    thread: Option<JoinHandle<()>>,
    last_poll: Arc<Mutex<Instant>>,
}
fn text(bytes: &[c_char]) -> Result<String, String> {
    let end = bytes
        .iter()
        .position(|c| *c == 0)
        .ok_or("clock source text is not terminated")?;
    String::from_utf8(bytes[..end].iter().map(|c| *c as u8).collect())
        .map_err(|_| "clock source text is not UTF-8".into())
}
impl ClockSource {
    pub fn new(
        spec: ClockSourceSpec,
        library: Arc<LoadedPlugin>,
        clock: Arc<SourceClock>,
        node_name: &str,
    ) -> Result<Self, String> {
        library.clock_source_vtable().map_err(|e| e.0)?;
        let mut config = toml::Table::new();
        config.insert("node_name".into(), node_name.into());
        config.insert("topic".into(), spec.topic.clone().into());
        config.insert(
            "expected_publisher".into(),
            spec.expected_publisher.clone().into(),
        );
        config.insert(
            "queue_capacity".into(),
            i64::from(spec.queue_capacity).into(),
        );
        let config = CString::new(toml::to_string(&config).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
        Ok(Self {
            clock,
            spec,
            library,
            config,
            stop: Arc::new(AtomicBool::new(false)),
            thread: None,
            last_poll: Arc::new(Mutex::new(Instant::now())),
        })
    }
    pub fn start(&mut self, health: Arc<HealthLog>) -> Result<(), String> {
        let lib = self.library.clone();
        let vt = lib.clock_source_vtable().map_err(|e| e.0)?;
        let (spec, config, clock, stop) = (
            self.spec.clone(),
            self.config.clone(),
            self.clock.clone(),
            self.stop.clone(),
        );
        let last_poll = self.last_poll.clone();
        self.thread = Some(std::thread::Builder::new().name("xgc-clock-source".into()).spawn(move || {
            // Keep library alive even if an uncooperative native call is abandoned.
            let _library = lib;
            let instance = unsafe { vt.create.unwrap()() };
            if instance.is_null() { clock.fail("clock source create returned NULL"); return; }
            let mut observation = Observation::default();
            let started = unsafe { vt.start.unwrap()(instance, config.as_ptr(), &mut observation) };
            if started != OK {
                clock.fail(format!("clock source start failed: {}", text(&observation.error).unwrap_or_else(|e| e)));
            }
            let mut gate = u32::MAX;
            let mut sequence = 0;
            let mut last_heartbeat = Instant::now();
            while !stop.load(Ordering::Acquire) && clock.snapshot().fault.is_none() {
                observation = Observation::default();
                let result = unsafe { vt.poll.unwrap()(instance, spec.poll_wall_ms * 1_000_000, &mut observation) };
                *last_poll.lock().unwrap() = Instant::now();
                let valid = validate_observation(result, &observation, &spec.expected_publisher, sequence);
                match valid {
                    Err(e) => clock.fail(e),
                    Ok(Some(time)) => {
                        sequence = observation.sequence;
                        if let Err(e) = clock.observe(time) { clock.fail(e); }
                    }
                    Ok(None) if observation.publisher_count == 0 => clock.unavailable(),
                    Ok(None) => (),
                }
                let state = clock.snapshot();
                let next = if state.fault.is_some() { GATE_FAULT } else if state.runnable { GATE_OPEN } else { GATE_CLOSED };
                if gate != next {
                    if unsafe { vt.set_gate.unwrap()(instance, next) } != OK { clock.fail("clock source output gate failed"); }
                    gate = next;
                    health.event(serde_json::json!({"event":"clock_source_gate", "gate":gate, "generation":state.generation, "fault":state.fault}));
                }
                if last_heartbeat.elapsed() >= Duration::from_secs(1) {
                    health.event(serde_json::json!({"event":"clock_source_heartbeat", "runnable":state.runnable, "generation":state.generation, "source_sequence":sequence, "world_instance_id":spec.world_instance_id, "publisher":spec.expected_publisher}));
                    last_heartbeat = Instant::now();
                }
            }
            clock.arm(false);
            unsafe {
                let gate = if clock.snapshot().fault.is_some() { GATE_FAULT } else { GATE_CLOSED };
                if vt.set_gate.unwrap()(instance, gate) != OK { clock.fail("clock source final output gate failed"); }
                vt.stop.unwrap()(instance);
                vt.destroy.unwrap()(instance);
            }
        }).map_err(|e| e.to_string())?);
        Ok(())
    }
    pub fn wait_first(&self, stop: &AtomicBool) -> Result<(), String> {
        let begin = Instant::now();
        loop {
            let s = self.clock.snapshot();
            if let Some(e) = s.fault {
                return Err(e);
            }
            if s.time.is_some() {
                return Ok(());
            }
            if stop.load(Ordering::Acquire) {
                return Ok(());
            }
            if begin.elapsed() >= Duration::from_millis(self.spec.startup_timeout_wall_ms) {
                self.clock
                    .fail("clock source startup timeout (steady time)");
                return Err("clock source startup timeout (steady time)".into());
            }
            std::thread::sleep(Duration::from_millis(2));
        }
    }
    /// A native poll that ceases returning is a fault, not a healthy pause.
    pub fn check_health(&self) {
        if self.last_poll.lock().unwrap().elapsed()
            > Duration::from_millis(self.spec.poll_wall_ms + 250)
        {
            self.clock
                .fail("clock source exceeded its steady-time poll bound");
        }
        if self.thread.as_ref().is_some_and(|t| t.is_finished())
            && !self.stop.load(Ordering::Acquire)
            && self.clock.snapshot().fault.is_none()
        {
            self.clock.fail("clock source worker exited unexpectedly");
        }
    }
    pub fn shutdown(&mut self) -> Result<(), String> {
        self.clock.arm(false);
        self.stop.store(true, Ordering::Release);
        if let Some(thread) = self.thread.take() {
            let start = Instant::now();
            while !thread.is_finished()
                && start.elapsed() < Duration::from_millis(self.spec.poll_wall_ms + 250)
            {
                std::thread::sleep(Duration::from_millis(2));
            }
            if !thread.is_finished() {
                self.clock
                    .fail("clock source did not stop within steady-time bound; worker abandoned");
                return Err(
                    "clock source did not stop within steady-time bound; worker abandoned".into(),
                );
            }
            if thread.join().is_err() {
                self.clock.fail("clock source worker panicked");
                return Err("clock source worker panicked".into());
            }
        }
        Ok(())
    }
}
impl Drop for ClockSource {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}

fn validate_observation(
    result: i32,
    o: &Observation,
    publisher: &str,
    sequence: u64,
) -> Result<Option<i64>, String> {
    if result == ERROR {
        return Err(format!("clock source: {}", text(&o.error)?));
    }
    if result != OK && result != AGAIN {
        return Err(format!("unknown clock source status {result}"));
    }
    if o.dropped != 0 {
        return Err("clock source queue dropped observations".into());
    }
    if o.publisher_count > 1 {
        return Err("multiple simulator time publishers; new Session required".into());
    }
    if result == AGAIN {
        return Ok(None);
    }
    if o.publisher_count != 1 || text(&o.publisher)? != publisher {
        return Err("simulator time authority mismatch; new Session required".into());
    }
    if o.sequence <= sequence {
        return Err("clock source sequence did not increase".into());
    }
    if o.time_ns < 0 {
        return Err("negative simulator time".into());
    }
    Ok(Some(o.time_ns))
}
