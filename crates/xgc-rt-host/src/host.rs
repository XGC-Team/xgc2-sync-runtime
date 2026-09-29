//! The aggregator: one process that loads the manifest's modules and runs
//! them on Session rounds.
//!
//! **Threads.**
//! - One thread per module (D9) makes every vtable call of that module, from
//!   create to destroy, so a slow or blocking module stalls only itself.
//! - The main thread (`run`) routes frames that arrive over the link into
//!   module inputs, runs the clock probe and the watchdog, and counts rounds.
//! - Transport IO threads only run the endpoint sink (stamp, verify, audit,
//!   enqueue). One writer thread appends step records.
//!
//! **Handoff inside the process is memory only.** A module's output write
//! puts one shared, immutable sample (`Arc<Sample>`) into each same-process
//! reader's input, sets the reader's dirty bit and notifies its thread.
//! Nothing is encoded, stamped or sent for a same-process hop. An input is a
//! bounded queue (every sample; full drops the oldest) or, with
//! `latest = true`, holds only the newest sample: the writer swaps it in and
//! the reader takes it.
//!
//! **The link** (transport) is used only when the roster has other nodes:
//! outputs are then also sent on it, and frames from other nodes land in the
//! same inputs, so a module cannot tell a local writer from a remote one.
//! Only link frames are audited as frames; same-process steps are recorded
//! in `steps.jsonl` (module, round, start, end, and which samples it read):
//! every step, or with `audit.steps_every = N` every step of one round in N
//! and every step that failed or overran its budget.
//!
//! **Each step sees one consistent snapshot.** At step start the module
//! thread takes every sample queued at that moment; `next` reads only from
//! that snapshot. Samples that arrive during the step wait for the next one,
//! and samples the step did not read go back to the front of their queue.
//!
//! **Wake rule.** A module thread sleeps until its next round boundary (for
//! `on_round`/`both`), a new sample (for `on_dirty`/`both`), a pending
//! restart or stop. It never busy-polls.
//!
//! **Stop.** Each module thread finishes its current step, deactivates and
//! destroys its instance. A sample written after its reader stopped is not
//! read (at most one per hop).
//!
//! **Watchdog.** A step longer than the module's budget (`step_budget_ms`,
//! default one period) marks it Degraded; the next step within budget
//! recovers it. A step longer than 10× the budget is a hang (a lifecycle
//! call such as activate or destroy gets at least 5 s): a thread can't
//! be killed safely, so the instance is abandoned (never called again, its
//! memory left alone) and, if the restart policy allows, a new thread and
//! instance take over after the backoff. After `session.max_abandoned`
//! abandons the aggregator stops and reports it, so the Agent restarts the
//! process.
//!
//! **Soundness.** Plugins call back through the raw `host` pointer, which
//! points at that module thread's `Slot`. Only that thread touches the slot;
//! the rest of the module's state is behind `Mutex`es and atomics.

use std::collections::{BTreeMap, VecDeque};
use std::ffi::{c_char, c_void, CStr, CString};
use std::fs::{File, OpenOptions};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc;
use std::sync::{Arc, Condvar, Mutex, RwLock};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use serde::Serialize;
use xgc_rt_abi::*;
use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::audit::{AuditSink, OverflowSite};
use xgc_rt_core::clock::{Clock, ClockDomain, RoundSchedule};
use xgc_rt_core::envelope::Header;
use xgc_rt_core::lifecycle::{Event, Lifecycle, State};
use xgc_rt_core::manifest::{Manifest, RestartKind, RestartPolicy, Resolved, Trigger};
use xgc_rt_core::transport::{Transport, TransportContext};
use xgc_rt_core::{ChannelId, OriginId};

use crate::endpoint::{Endpoint, RxFrame, DEFAULT_RX_QUEUE};
use crate::plugin::{self, LoadedPlugin};

pub const INBOX_CAPACITY: usize = 1024;
/// A step this many budgets long is a hang.
pub const HANG_FACTOR: u32 = 10;
/// How long a non-step call (create, configure, activate, deactivate,
/// domain_state, destroy) may take before it counts as hung. These may do
/// real I/O, e.g. ROS registration.
pub const LIFECYCLE_GRACE: Duration = Duration::from_secs(5);
/// Upper bound on any sleep, so a stop is noticed.
const MAX_WAIT: Duration = Duration::from_millis(100);

#[derive(Debug)]
pub struct HostError(pub String);

impl std::fmt::Display for HostError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for HostError {}

fn herr<T>(m: impl Into<String>) -> Result<T, HostError> {
    Err(HostError(m.into()))
}

/// Append-only `health.jsonl`: timings, lifecycle transitions, plugin logs
/// and faults. Module threads enqueue; the writer owns disk and stderr I/O.
pub struct HealthLog {
    output: LineLog,
    clock: Arc<dyn Clock>,
    started: Instant,
}

impl HealthLog {
    fn open(path: &Path, clock: Arc<dyn Clock>, echo: bool) -> std::io::Result<Self> {
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(Self { output: LineLog::spawn(file, echo, "xgc-health")?, clock, started: Instant::now() })
    }

    pub fn event(&self, value: serde_json::Value) {
        let mut line = serde_json::json!({ "t": self.clock.now(), "steady_elapsed_ns": self.started.elapsed().as_nanos() as u64 });
        if let (Some(obj), serde_json::Value::Object(extra)) = (line.as_object_mut(), value) {
            obj.extend(extra);
        }
        let _ = self.output.tx.send(Some(line.to_string()));
    }
}

/// Shared writer for health and step logs. Neither disk nor stderr backpressure
/// belongs on a module thread.
struct LineLog {
    tx: mpsc::Sender<Option<String>>,
    writer: Mutex<Option<JoinHandle<()>>>,
}

impl LineLog {
    fn spawn<W: Write + Send + 'static>(mut out: W, echo: bool, name: &str) -> std::io::Result<Self> {
        let (tx, rx) = mpsc::channel::<Option<String>>();
        let writer = std::thread::Builder::new().name(name.into()).spawn(move || {
            while let Ok(Some(line)) = rx.recv() {
                let _ = writeln!(out, "{line}");
                if echo {
                    eprintln!("{line}");
                }
            }
            let _ = out.flush();
        })?;
        Ok(Self { tx, writer: Mutex::new(Some(writer)) })
    }

    fn sender(&self) -> mpsc::Sender<Option<String>> {
        self.tx.clone()
    }

    /// Flush and stop, even if abandoned threads still hold senders.
    fn finish(&self) {
        let _ = self.tx.send(None);
        if let Some(w) = self.writer.lock().unwrap().take() {
            let _ = w.join();
        }
    }
}

impl Drop for LineLog {
    fn drop(&mut self) {
        self.finish();
    }
}

// --- samples and inputs -----------------------------------------------------

/// One sample in memory, shared by every reader without copying.
struct Sample {
    origin: OriginId,
    seq: u64,
    round: u64,
    t_produce: i64,
    t_tx: i64,
    t_rx: i64,
    payload: Vec<u8>,
    /// Set for frames that came over the link; their first read is audited.
    link_header: Option<Header>,
}

impl Sample {
    fn from_frame(frame: RxFrame) -> Self {
        let h = frame.header;
        Self {
            origin: h.origin,
            seq: h.seq,
            round: h.round,
            t_produce: h.t_produce,
            t_tx: h.t_tx,
            t_rx: frame.t_rx,
            payload: frame.payload,
            link_header: Some(h),
        }
    }
}

#[derive(Clone)]
struct InputSpec {
    channel: ChannelId,
    origins: Vec<OriginId>,
    latest: bool,
}

struct Input {
    spec: InputSpec,
    queue: VecDeque<Arc<Sample>>,
}

struct Inner {
    /// By port index; `None` for out-ports.
    inputs: Vec<Option<Input>>,
    dirty: u64,
    /// Same-process samples dropped from a full queue.
    dropped: u64,
    schedule: Option<RoundSchedule>,
    stop: bool,
}

/// One running instance of a module: its thread's inputs and wake signal.
/// A restart after a hang gets a new one; the abandoned one is left alone.
struct Instance {
    inner: Mutex<Inner>,
    wake: Condvar,
    /// Monotonic ns (since host start) + 1 when the current vtable call
    /// began; 0 when the thread is not inside plugin code.
    call_started: AtomicU64,
    /// The current call is `step` (budgeted), not a lifecycle call.
    in_step: AtomicBool,
    abandoned: AtomicBool,
    done: AtomicBool,
}

impl Instance {
    fn new(inputs: &[Option<InputSpec>], schedule: Option<RoundSchedule>) -> Arc<Self> {
        let inputs = inputs.iter().map(|s| s.clone().map(|spec| Input { spec, queue: VecDeque::new() })).collect();
        Arc::new(Self {
            inner: Mutex::new(Inner { inputs, dirty: 0, dropped: 0, schedule, stop: false }),
            wake: Condvar::new(),
            call_started: AtomicU64::new(0),
            in_step: AtomicBool::new(false),
            abandoned: AtomicBool::new(false),
            done: AtomicBool::new(false),
        })
    }

    fn deliver(&self, port: u32, sample: Arc<Sample>, audit: &dyn AuditSink, now: i64) {
        self.deliver_all([(port, sample)], audit, now);
    }

    /// Put a batch into the inputs under one lock with one notify, so the
    /// module never steps on half of a batch that arrived together.
    fn deliver_all(&self, batch: impl IntoIterator<Item = (u32, Arc<Sample>)>, audit: &dyn AuditSink, now: i64) {
        let mut g = self.inner.lock().unwrap();
        let mut any = false;
        for (port, sample) in batch {
            any |= Self::push(&mut g, port, sample, audit, now);
        }
        drop(g);
        if any {
            self.wake.notify_one();
        }
    }

    fn push(g: &mut Inner, port: u32, sample: Arc<Sample>, audit: &dyn AuditSink, now: i64) -> bool {
        let Some(Some(input)) = g.inputs.get_mut(port as usize) else { return false };
        let mut lost = false;
        if input.spec.latest {
            input.queue.clear();
        } else if input.queue.len() >= INBOX_CAPACITY {
            input.queue.pop_front();
            lost = true;
        }
        let (channel, origin, link) = (input.spec.channel, sample.origin, sample.link_header.is_some());
        input.queue.push_back(sample);
        g.dirty |= 1u64 << port;
        if lost {
            if link {
                audit.overflow(OverflowSite::Inbox, channel, origin, now);
            } else {
                g.dropped += 1;
            }
        }
        true
    }

    fn set_schedule(&self, schedule: RoundSchedule) {
        self.inner.lock().unwrap().schedule = Some(schedule);
        self.wake.notify_one();
    }

    fn stop(&self) {
        self.inner.lock().unwrap().stop = true;
        self.wake.notify_one();
    }
}

// --- modules ----------------------------------------------------------------

struct OutRoute {
    channel: ChannelId,
    /// Same-process readers: (module index, in-port).
    readers: Vec<(usize, u32)>,
}

#[derive(Default)]
struct Status {
    fsm: Lifecycle,
    restarts: u32,
    abandons: u32,
    last_error: Option<String>,
    domain_state: String,
    restart_at: Option<Instant>,
    /// Degraded by the watchdog (overrun), not by the module's request.
    overrun: bool,
}

struct Module {
    name: String,
    /// `name` as a JSON string, for step records.
    name_json: String,
    lib: Arc<LoadedPlugin>,
    trigger: Trigger,
    /// Step again this long after the previous step (`wake_ms`).
    wake: Option<Duration>,
    restart: RestartPolicy,
    config: CString,
    budget: Duration,
    inputs: Vec<Option<InputSpec>>,
    outputs: Vec<Option<OutRoute>>,
    /// Optional ports the manifest left unbound, by port index.
    unbound: Vec<bool>,
    current: RwLock<Arc<Instance>>,
    status: Mutex<Status>,
    thread: Mutex<Option<JoinHandle<()>>>,
    steps: AtomicU64,
    published: AtomicU64,
    consumed: AtomicU64,
}

impl Module {
    fn instance(&self) -> Arc<Instance> {
        self.current.read().unwrap().clone()
    }

    fn hang(&self) -> Duration {
        self.budget * HANG_FACTOR
    }

    /// The hang limit for the call the instance is in now.
    fn limit(&self, inst: &Instance) -> Duration {
        if inst.in_step.load(Ordering::Acquire) {
            self.hang()
        } else {
            self.hang().max(LIFECYCLE_GRACE)
        }
    }
}

/// Shared by the main thread and every module thread.
struct Runtime {
    node_id: OriginId,
    link: bool,
    clock: Arc<dyn Clock>,
    endpoint: Arc<Endpoint>,
    health: Arc<HealthLog>,
    steps: LineLog,
    /// Record every step of one round in this many (`audit.steps_every`).
    steps_every: u64,
    modules: Vec<Module>,
    started: Instant,
}

impl Runtime {
    fn mono_ns(&self) -> u64 {
        self.started.elapsed().as_nanos() as u64 + 1
    }

    fn transition(&self, m: usize, event: Event, detail: Option<&str>) {
        let mut st = self.modules[m].status.lock().unwrap();
        self.transition_locked(m, &mut st, event, detail);
    }

    fn transition_locked(&self, m: usize, st: &mut Status, event: Event, detail: Option<&str>) {
        let from = st.fsm.state();
        let name = &self.modules[m].name;
        match st.fsm.apply(event) {
            Ok(to) => self.health.event(serde_json::json!({
                "event": "transition", "plugin": name, "from": from.name(), "to": to.name(), "cause": format!("{event:?}"), "detail": detail,
            })),
            Err(e) => self.health.event(serde_json::json!({ "event": "invalid_transition", "plugin": name, "error": e.to_string() })),
        }
    }

    /// Fault the module and, if the policy allows, schedule a restart.
    fn fault(&self, m: usize, what: &str) {
        let module = &self.modules[m];
        let mut st = module.status.lock().unwrap();
        st.last_error = Some(what.to_string());
        self.transition_locked(m, &mut st, Event::Fault, Some(what));
        if module.restart.policy == RestartKind::OnError && st.restarts < module.restart.max {
            st.restart_at = Some(Instant::now() + Duration::from_millis(module.restart.backoff_ms));
        }
    }
}

// --- host API callbacks (the module's own thread, inside a vtable call) ------

/// Per module thread. `api.host` points here.
struct Slot {
    rt: Arc<Runtime>,
    module: usize,
    instance: Arc<Instance>,
    api: XgcHostApi,
    current: Option<Arc<Sample>>,
    /// This step's snapshot of each input, by port.
    staged: Vec<VecDeque<Arc<Sample>>>,
    local_seq: Vec<u64>,
    /// Samples read during the current step: (port, origin, seq).
    reads: Vec<(u32, OriginId, u64)>,
    round: u64,
    degrade_request: Option<String>,
    recover_request: bool,
    steps_tx: mpsc::Sender<Option<String>>,
}

unsafe fn slot<'a>(host: *mut c_void) -> &'a mut Slot {
    &mut *host.cast::<Slot>()
}

unsafe extern "C" fn api_publish(host: *mut c_void, port: u32, round: u64, data: *const u8, len: u32) -> XgcStatus {
    let s = slot(host);
    let rt = s.rt.clone();
    if rt.clock.dispatch_stamp().is_some_and(|s| !s.runnable) { return XGC_OK; } // source stopped: discard, never re-stamp later
    let module = &rt.modules[s.module];
    if module.unbound.get(port as usize) == Some(&true) && module.lib.ports[port as usize].is_out {
        return XGC_OK; // an unbound optional output: dropped
    }
    let Some(Some(route)) = module.outputs.get(port as usize) else {
        return XGC_ERR_INVALID;
    };
    if len > 0 && data.is_null() {
        return XGC_ERR_INVALID;
    }
    let payload = if len == 0 { &[][..] } else { std::slice::from_raw_parts(data, len as usize) };
    let t_produce = rt.clock.now();
    let seq = if rt.link {
        match rt.endpoint.publish(route.channel, round, t_produce, payload) {
            Ok(header) => header.seq,
            Err(e) => {
                rt.health.event(serde_json::json!({ "event": "publish_error", "plugin": module.name, "port": port, "error": e.0 }));
                return XGC_ERR;
            }
        }
    } else {
        s.local_seq[port as usize] += 1;
        s.local_seq[port as usize]
    };
    if !route.readers.is_empty() {
        let sample = Arc::new(Sample {
            origin: rt.node_id,
            seq,
            round,
            t_produce,
            t_tx: t_produce,
            t_rx: t_produce,
            payload: payload.to_vec(),
            link_header: None,
        });
        for &(reader, in_port) in &route.readers {
            rt.modules[reader].instance().deliver(in_port, sample.clone(), rt.endpoint.audit().as_ref(), t_produce);
        }
    }
    module.published.fetch_add(1, Ordering::Relaxed);
    XGC_OK
}

unsafe extern "C" fn api_next(host: *mut c_void, port: u32, out: *mut XgcSampleView) -> XgcStatus {
    let s = slot(host);
    if out.is_null() {
        return XGC_ERR_INVALID;
    }
    let module = &s.rt.modules[s.module];
    if !matches!(module.inputs.get(port as usize), Some(Some(_))) {
        let unbound_input = module.unbound.get(port as usize) == Some(&true) && !module.lib.ports[port as usize].is_out;
        return if unbound_input { XGC_ERR_AGAIN } else { XGC_ERR_INVALID };
    }
    let Some(sample) = s.staged[port as usize].pop_front() else {
        return XGC_ERR_AGAIN;
    };
    if let Some(header) = &sample.link_header {
        s.rt.endpoint.audit().consumed(header, s.rt.clock.now());
    }
    s.rt.modules[s.module].consumed.fetch_add(1, Ordering::Relaxed);
    s.reads.push((port, sample.origin, sample.seq));
    let current = s.current.insert(sample);
    *out = XgcSampleView {
        origin: current.origin,
        reserved: 0,
        len: current.payload.len() as u32,
        seq: current.seq,
        round: current.round,
        t_produce: current.t_produce,
        t_tx: current.t_tx,
        t_rx: current.t_rx,
        data: current.payload.as_ptr(),
    };
    XGC_OK
}

unsafe extern "C" fn api_now(host: *mut c_void) -> i64 {
    slot(host).rt.clock.now()
}

unsafe fn c_text(ptr: *const c_char) -> String {
    if ptr.is_null() {
        String::new()
    } else {
        CStr::from_ptr(ptr).to_string_lossy().into_owned()
    }
}

unsafe extern "C" fn api_log(host: *mut c_void, level: XgcLogLevel, message: *const c_char) {
    let s = slot(host);
    let level = match level {
        XGC_LOG_DEBUG => "debug",
        XGC_LOG_INFO => "info",
        XGC_LOG_WARN => "warn",
        _ => "error",
    };
    let name = &s.rt.modules[s.module].name;
    s.rt.health.event(serde_json::json!({ "event": "log", "plugin": name, "level": level, "message": c_text(message) }));
}

unsafe extern "C" fn api_request_degrade(host: *mut c_void, reason: *const c_char) {
    let s = slot(host);
    s.degrade_request = Some(c_text(reason));
    s.recover_request = false;
}

unsafe extern "C" fn api_request_recover(host: *mut c_void) {
    let s = slot(host);
    s.recover_request = true;
    s.degrade_request = None;
}

unsafe extern "C" fn api_port_origins(host: *mut c_void, port: u32, out: *mut u16, cap: u32) -> u32 {
    let s = slot(host);
    let Some(Some(spec)) = s.rt.modules[s.module].inputs.get(port as usize) else {
        return 0;
    };
    if !out.is_null() {
        for (i, o) in spec.origins.iter().take(cap as usize).enumerate() {
            *out.add(i) = *o;
        }
    }
    spec.origins.len() as u32
}

unsafe extern "C" fn api_node_id(host: *mut c_void) -> u16 {
    slot(host).rt.node_id
}

// --- the module thread ------------------------------------------------------

struct SendPtr(*mut Slot);
// SAFETY: the slot is created for, and only used by, one module thread.
unsafe impl Send for SendPtr {}

fn spawn_module(rt: &Arc<Runtime>, m: usize, instance: Arc<Instance>, ready: Option<mpsc::Sender<usize>>) -> Result<(), HostError> {
    let module = &rt.modules[m];
    let slot = Box::new(Slot {
        rt: rt.clone(),
        module: m,
        instance,
        api: XgcHostApi {
            abi_version: XGC_RT_ABI_VERSION,
            abi_minor: XGC_RT_ABI_MINOR,
            host: std::ptr::null_mut(),
            publish: api_publish,
            next: api_next,
            now: api_now,
            log: api_log,
            request_degrade: api_request_degrade,
            request_recover: api_request_recover,
            port_origins: api_port_origins,
            node_id: api_node_id,
        },
        current: None,
        staged: vec![VecDeque::new(); module.inputs.len()],
        local_seq: vec![0; module.outputs.len()],
        reads: Vec::new(),
        round: 0,
        degrade_request: None,
        recover_request: false,
        steps_tx: rt.steps.sender(),
    });
    let ptr = Box::into_raw(slot);
    unsafe { (*ptr).api.host = ptr.cast() };
    let ptr = SendPtr(ptr);
    let handle = std::thread::Builder::new()
        .name(format!("xgc-{}", module.name))
        .spawn(move || {
            let ptr = ptr;
            let abandoned = ModuleThread { slot: ptr.0, instance: std::ptr::null_mut() }.run(ready);
            if !abandoned {
                // SAFETY: allocated above; no plugin instance refers to it any more.
                drop(unsafe { Box::from_raw(ptr.0) });
            }
            // An abandoned slot is left alone: the stuck instance may still
            // call back through it.
        })
        .map_err(|e| HostError(format!("spawn module thread: {e}")))?;
    *module.thread.lock().unwrap() = Some(handle);
    Ok(())
}

struct ModuleThread {
    slot: *mut Slot,
    instance: *mut c_void,
}

impl ModuleThread {
    fn s(&self) -> &mut Slot {
        // SAFETY: only this thread uses the slot, and no reference is held
        // across a vtable call.
        unsafe { &mut *self.slot }
    }

    fn rt(&self) -> Arc<Runtime> {
        self.s().rt.clone()
    }

    fn inst(&self) -> Arc<Instance> {
        self.s().instance.clone()
    }

    /// Mark the start and end of plugin code, for the watchdog. Returns
    /// false when the instance was abandoned meanwhile: the caller must then
    /// never call the plugin again.
    fn call<T>(&self, f: impl FnOnce() -> T) -> Option<T> {
        let (rt, inst) = (self.rt(), self.inst());
        inst.call_started.store(rt.mono_ns(), Ordering::Release);
        let out = f();
        inst.call_started.store(0, Ordering::Release);
        (!inst.abandoned.load(Ordering::Acquire)).then_some(out)
    }

    /// Run until stop. Returns true when the instance was abandoned.
    fn run(mut self, ready: Option<mpsc::Sender<usize>>) -> bool {
        let (rt, inst) = (self.rt(), self.inst());
        let m = self.s().module;
        let module = &rt.modules[m];
        if !self.bring_up() {
            return true;
        }
        if let Some(ready) = ready {
            let _ = ready.send(m);
        }
        let mut last_round = None;
        let mut activated = false;
        let mut last_step = Instant::now();
        let mut last_generation = None;
        loop {
            let schedule = {
                let g = inst.inner.lock().unwrap();
                if g.stop {
                    break;
                }
                g.schedule
            };
            let source = rt.clock.dispatch_stamp();
            if source.is_some_and(|s| !s.runnable || last_generation == Some(s.generation)) {
                // Session time is frozen: dirty inputs/wake_ms cannot create
                // more domain work. Stop remains a steady-time wakeup.
                let mut g = inst.inner.lock().unwrap();
                if source.is_some_and(|s| !s.runnable) {
                    // Frozen input queues are not an output replay backlog.
                    for input in g.inputs.iter_mut().flatten() { input.queue.clear(); }
                    g.dirty = 0;
                }
                let _ = inst.wake.wait_timeout_while(g, Duration::from_millis(1), |g| !g.stop).unwrap();
                continue;
            }
            let now = source.map_or_else(|| rt.clock.now(), |s| s.time);
            if let Some(k) = schedule.and_then(|s| s.round_at(now)) {
                let schedule = schedule.unwrap();
                if !activated {
                    activated = true;
                    if !self.activate() {
                        return true;
                    }
                }
                let restart_due = {
                    let st = module.status.lock().unwrap();
                    st.fsm.state() == State::Error && st.restart_at.is_some_and(|t| Instant::now() >= t)
                };
                if restart_due && !self.restart_in_place() {
                    return true;
                }
                if rt.clock.dispatch_stamp().is_some_and(|s| !s.runnable || source.is_some_and(|old| old.generation != s.generation)) {
                    continue; // activation/restart may have outlived this accepted source stamp
                }
                let advanced = last_round != Some(k);
                if let Some(p) = last_round.filter(|p| k > p + 1 && module.trigger != Trigger::OnDirty) {
                    // Woke late (load or a long step): one step for the
                    // current round; the skipped rounds are recorded.
                    rt.health.event(serde_json::json!({ "event": "rounds_skipped", "plugin": module.name, "from": p + 1, "to": k - 1 }));
                }
                last_round = Some(k);
                let wake_due = module.wake.is_some_and(|w| last_step.elapsed() >= w);
                if wake_due || advanced {
                    last_step = Instant::now();
                }
                last_generation = source.map(|s| s.generation);
                if !self.step(&schedule, k, advanced, wake_due, source.map(|s| s.time)) {
                    return true;
                }
            }
            // Sleep until the next boundary, a new sample, a restart or stop.
            let now = rt.clock.now();
            let mut timeout = MAX_WAIT;
            if let Some(s) = schedule {
                if s.round_at(now).is_none() || module.trigger != Trigger::OnDirty {
                    timeout = timeout.min(Duration::from_nanos((s.next_boundary_after(now) - now).max(0) as u64));
                }
            }
            if let Some(at) = module.status.lock().unwrap().restart_at {
                timeout = timeout.min(at.saturating_duration_since(Instant::now()));
            }
            if let Some(w) = module.wake {
                timeout = timeout.min(w.saturating_sub(last_step.elapsed()));
            }
            let wakes_on_input = module.trigger != Trigger::OnRound;
            let g = inst.inner.lock().unwrap();
            let _ = inst
                .wake
                .wait_timeout_while(g, timeout, |g| !g.stop && !(wakes_on_input && g.dirty != 0) && g.schedule == schedule)
                .unwrap();
        }
        self.finish()
    }

    /// create + configure: Unconfigured → Inactive. False when abandoned.
    fn bring_up(&mut self) -> bool {
        let rt = self.rt();
        let m = self.s().module;
        let (lib, api, config) = {
            let s = self.s();
            let module = &s.rt.modules[m];
            (module.lib.clone(), &s.api as *const XgcHostApi, module.config.as_ptr())
        };
        let Some(instance) = self.call(|| unsafe { (lib.vtbl.create)(api) }) else { return false };
        if instance.is_null() {
            rt.fault(m, "create returned NULL");
            return true;
        }
        self.instance = instance;
        let Some(status) = self.call(|| unsafe { (lib.vtbl.configure)(instance, config) }) else { return false };
        if status == XGC_OK {
            rt.transition(m, Event::Configure, None);
        } else {
            rt.fault(m, &format!("configure returned {status}"));
        }
        true
    }

    fn activate(&mut self) -> bool {
        let rt = self.rt();
        let m = self.s().module;
        if rt.modules[m].status.lock().unwrap().fsm.state() != State::Inactive {
            return true;
        }
        let (f, instance) = (rt.modules[m].lib.vtbl.activate, self.instance);
        let Some(status) = self.call(|| unsafe { f(instance) }) else { return false };
        if status == XGC_OK {
            rt.transition(m, Event::Activate, None);
        } else {
            rt.fault(m, &format!("activate returned {status}"));
        }
        true
    }

    fn destroy(&mut self) -> bool {
        let rt = self.rt();
        let m = self.s().module;
        if self.instance.is_null() {
            return true;
        }
        let (f, instance) = (rt.modules[m].lib.vtbl.destroy, self.instance);
        let alive = self.call(|| unsafe { f(instance) }).is_some();
        self.instance = std::ptr::null_mut();
        self.s().current = None;
        alive
    }

    fn restart_in_place(&mut self) -> bool {
        let rt = self.rt();
        let m = self.s().module;
        if !self.destroy() {
            return false;
        }
        rt.transition(m, Event::Reset, Some("restart policy"));
        {
            let mut st = rt.modules[m].status.lock().unwrap();
            st.restarts += 1;
            st.restart_at = None;
        }
        self.inst().inner.lock().unwrap().dirty = 0;
        self.bring_up() && self.activate()
    }

    fn step(&mut self, schedule: &RoundSchedule, k: u64, advanced: bool, wake_due: bool, source_time: Option<i64>) -> bool {
        let rt = self.rt();
        let m = self.s().module;
        let module = &rt.modules[m];
        if !module.status.lock().unwrap().fsm.state().runs() {
            return true;
        }
        let inst = self.inst();
        let dirty = {
            let mut g = inst.inner.lock().unwrap();
            let run = match module.trigger {
                Trigger::OnRound => advanced || wake_due,
                Trigger::OnDirty => g.dirty != 0 || wake_due,
                Trigger::Both => advanced || wake_due || g.dirty != 0,
            };
            if !run {
                return true;
            }
            let staged = &mut self.s().staged;
            for (input, snapshot) in g.inputs.iter_mut().zip(staged.iter_mut()) {
                if let Some(input) = input {
                    std::mem::swap(&mut input.queue, snapshot);
                }
            }
            std::mem::take(&mut g.dirty)
        };
        let ctx = XgcStepCtx {
            round: k,
            now: source_time.unwrap_or_else(|| rt.clock.now()),
            round_start: schedule.start(k),
            deadline: schedule.deadline(k),
            dirty_ports: dirty,
            round_advanced: u32::from(advanced),
            reserved: 0,
        };
        self.s().round = k;
        self.s().reads.clear();
        module.steps.fetch_add(1, Ordering::Relaxed);
        let (f, instance) = (module.lib.vtbl.step, self.instance);
        let t0 = source_time.unwrap_or_else(|| rt.clock.now());
        let began = Instant::now();
        self.inst().in_step.store(true, Ordering::Release);
        let status = self.call(|| unsafe { f(instance, &ctx) });
        self.inst().in_step.store(false, Ordering::Release);
        let Some(status) = status else { return false };
        let took = began.elapsed();
        let t1 = rt.clock.now();
        {
            // Unread samples go back in front of anything newer.
            let mut g = inst.inner.lock().unwrap();
            for (input, snapshot) in g.inputs.iter_mut().zip(self.s().staged.iter_mut()) {
                if let (Some(input), false) = (input, snapshot.is_empty()) {
                    while let Some(sample) = snapshot.pop_back() {
                        input.queue.push_front(sample);
                    }
                }
            }
        }
        // At 100 robots a line per step is ~40k lines/s; a sampled log keeps
        // whole rounds, and never drops a failed or slow step.
        if k % rt.steps_every == 0 || status != XGC_OK || took > module.budget {
            let line = step_record(&module.name_json, k, t0, t1, &self.s().reads);
            let _ = self.s().steps_tx.send(Some(line));
        }
        if status != XGC_OK {
            rt.fault(m, &format!("step returned {status} in round {k}"));
            return true;
        }
        let (degrade, recover) = {
            let s = self.s();
            (s.degrade_request.take(), std::mem::take(&mut s.recover_request))
        };
        let mut st = module.status.lock().unwrap();
        let state = st.fsm.state();
        if let Some(reason) = degrade {
            if state == State::Active {
                st.overrun = false;
                rt.transition_locked(m, &mut st, Event::Degrade, Some(&reason));
            }
        } else if took > module.budget {
            rt.health.event(serde_json::json!({ "event": "overrun", "plugin": module.name, "round": k, "step_ms": took.as_secs_f64() * 1e3, "budget_ms": module.budget.as_secs_f64() * 1e3 }));
            if state == State::Active {
                st.overrun = true;
                rt.transition_locked(m, &mut st, Event::Degrade, Some("step over budget"));
            }
        } else if state == State::Degraded && (recover || st.overrun) {
            st.overrun = false;
            rt.transition_locked(m, &mut st, Event::Recover, None);
        }
        true
    }

    /// Orderly stop: running → Inactive, read the domain state, destroy,
    /// Finalized. Returns true when abandoned.
    fn finish(mut self) -> bool {
        let rt = self.rt();
        let m = self.s().module;
        let module = &rt.modules[m];
        if module.status.lock().unwrap().fsm.state().runs() {
            let (f, instance) = (module.lib.vtbl.deactivate, self.instance);
            let Some(status) = self.call(|| unsafe { f(instance) }) else { return true };
            if status == XGC_OK {
                rt.transition(m, Event::Deactivate, None);
            } else {
                rt.fault(m, &format!("deactivate returned {status}"));
            }
        }
        if !self.instance.is_null() {
            let (f, instance) = (module.lib.vtbl.domain_state, self.instance);
            let Some(text) = self.call(|| unsafe { c_text(f(instance)) }) else { return true };
            module.status.lock().unwrap().domain_state = text;
        }
        if !self.destroy() {
            return true;
        }
        self.inst().done.store(true, Ordering::Release);
        false
    }
}

/// One `steps.jsonl` line, byte for byte what serializing
/// `{"m", "k", "t0", "t1", "in": [[port, origin, seq], ...]}` with serde_json
/// writes (keys sorted), without building a JSON tree per step.
fn step_record(name_json: &str, k: u64, t0: i64, t1: i64, reads: &[(u32, OriginId, u64)]) -> String {
    use std::fmt::Write as _;
    let mut line = String::with_capacity(56 + name_json.len() + 24 * reads.len());
    line.push_str("{\"in\":[");
    for (index, &(port, origin, seq)) in reads.iter().enumerate() {
        if index > 0 {
            line.push(',');
        }
        let _ = write!(line, "[{port},{origin},{seq}]");
    }
    let _ = write!(line, "],\"k\":{k},\"m\":{name_json},\"t0\":{t0},\"t1\":{t1}}}");
    line
}

// --- host -------------------------------------------------------------------

#[derive(Debug, Clone, Default, Serialize)]
pub struct StartupTimings {
    /// Milliseconds from `Host::new` to each milestone.
    pub manifest_ms: f64,
    pub plugins_loaded_ms: f64,
    pub ports_ready_ms: f64,
    /// Every plugin created and configured (each on its own thread).
    pub configured_ms: f64,
    pub clock_ok_ms: f64,
    pub first_round_ms: f64,
}

#[derive(Debug, Clone, Serialize)]
pub struct PluginSummary {
    pub name: String,
    pub library: String,
    pub version: String,
    pub sha256: String,
    pub state: String,
    pub domain_state: String,
    pub steps: u64,
    pub published: u64,
    pub consumed: u64,
    /// Same-process samples dropped from this module's full input queues.
    pub dropped: u64,
    pub restarts: u32,
    /// Hung instances abandoned by the watchdog.
    pub abandons: u32,
    pub last_error: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct RunSummary {
    pub session: String,
    pub node: String,
    pub transport: String,
    pub e0_ns: i64,
    pub period_ns: i64,
    pub rounds: u64,
    /// Main-thread wake-ups: one per round boundary plus one per receive
    /// burst. It is the idle-cost evidence for the wake rule.
    pub wakeups: u64,
    pub timings: StartupTimings,
    pub plugins: Vec<PluginSummary>,
    /// Set when the aggregator stopped early because too many hung modules
    /// were abandoned.
    pub aborted: Option<String>,
    pub audit_dir: PathBuf,
}

pub struct Host {
    manifest: Manifest,
    resolved: Resolved,
    rt: Arc<Runtime>,
    audit: Arc<FileAudit>,
    transport_kind: String,
    timings: StartupTimings,
    started: Instant,
    run_dir: PathBuf,
    clock_service: Option<crate::clock_service::ClockService>,
    clock_source: Option<crate::clock_source::ClockSource>,
}

pub struct HostOptions {
    /// Echo health events to stderr.
    pub echo_health: bool,
    pub rx_queue: usize,
}

impl Default for HostOptions {
    fn default() -> Self {
        Self { echo_health: false, rx_queue: DEFAULT_RX_QUEUE }
    }
}

impl Host {
    /// Load, validate and wire everything. No plugin instance exists yet:
    /// `run` creates each on its own thread. Relative plugin and audit paths
    /// resolve against `base_dir`, the manifest's directory.
    pub fn new(
        manifest: Manifest,
        base_dir: &Path,
        transport: Box<dyn Transport>,
        clock: Arc<dyn Clock>,
        opts: HostOptions,
    ) -> Result<Self, HostError> {
        if manifest.clock_source.is_some() {
            return herr("a manifest clock source requires Host::with_manifest_clock");
        }
        Self::build(manifest, base_dir, transport, clock, None, opts)
    }

    /// Production entry point. The manifest selects one source for both
    /// colocated and distributed placement; absence preserves wall behavior.
    pub fn with_manifest_clock(manifest: Manifest, base_dir: &Path, transport: Box<dyn Transport>, opts: HostOptions) -> Result<Self, HostError> {
        manifest.resolve().map_err(|e| HostError(e.0))?;
        let source = manifest.clock_source.as_ref().map(|c| Arc::new(xgc_rt_core::clock::SourceClock::new(Duration::from_millis(c.stale_after_wall_ms), c.max_advance_ns)));
        let clock: Arc<dyn Clock> = match &source {
            Some(c) => c.clone(),
            None => Arc::new(xgc_rt_core::clock::WallClock::new(0)),
        };
        Self::build(manifest, base_dir, transport, clock, source, opts)
    }

    fn build(manifest: Manifest, base_dir: &Path, transport: Box<dyn Transport>, clock: Arc<dyn Clock>, source: Option<Arc<xgc_rt_core::clock::SourceClock>>, opts: HostOptions) -> Result<Self, HostError> {
        let started = Instant::now();
        let mut timings = StartupTimings::default();
        let resolved = manifest.resolve().map_err(|e| HostError(e.0))?;
        timings.manifest_ms = ms_since(started);
        let s = &manifest.session;
        let node_id = resolved.node_id;

        let run_dir = base_dir.join(&manifest.audit.dir);
        let meta = NodeMeta {
            format: String::new(),
            session: s.id.clone(),
            node: s.node.clone(),
            node_id,
            roster: s.roster.clone(),
            channels: resolved.channels.iter().map(|c| c.name.clone()).collect(),
            clock_domain: match clock.domain() {
                ClockDomain::Wall => "wall".into(),
                ClockDomain::Sim => "sim".into(),
            },
            audit_queue_drops: 0,
            records_written: 0,
            complete: false,
        };
        let audit = Arc::new(FileAudit::create(&run_dir, meta, clock.clone()).map_err(|e| HostError(format!("audit: {e}")))?);
        if let Some(source) = &manifest.clock_source {
            let identity = serde_json::json!({"schema":"xgc-clock-source/1", "kind":"ros1_sim", "world_instance_id":source.world_instance_id,
                "topic":source.topic, "expected_publisher":source.expected_publisher, "epoch_ns":s.epoch_ns});
            std::fs::write(audit.dir().join("clock_source.json"), serde_json::to_vec_pretty(&identity).unwrap())
                .map_err(|e| HostError(format!("clock source audit identity: {e}")))?;
        }
        let health = Arc::new(
            HealthLog::open(&audit.dir().join("health.jsonl"), clock.clone(), opts.echo_health)
                .map_err(|e| HostError(format!("health log: {e}")))?,
        );
        let steps = File::create(audit.dir().join("steps.jsonl"))
            .and_then(|file| LineLog::spawn(BufWriter::new(file), false, "xgc-steps"))
            .map_err(|e| HostError(format!("step log: {e}")))?;

        // Load and validate every plugin before opening the transport.
        let mut loaded = Vec::new();
        for (decl, bindings) in manifest.plugins.iter().zip(&resolved.bindings) {
            let path = base_dir.join(&decl.path);
            let lib = plugin::load(&path, decl.sha256.as_deref()).map_err(|e| HostError(e.0))?;
            for port in &lib.ports {
                let Some((channel, origins)) = bindings.get(&port.name) else {
                    if port.optional {
                        health.event(serde_json::json!({ "event": "unbound_optional_port", "plugin": decl.name, "port": port.name }));
                        continue;
                    }
                    return herr(format!("plugin {}: port {} is not bound in the manifest", decl.name, port.name));
                };
                let chan = &resolved.channels[*channel as usize];
                if chan.qos != port.qos {
                    return herr(format!(
                        "plugin {}: port {} is {:?} but channel {} is {:?}",
                        decl.name, port.name, port.qos, chan.name, chan.qos
                    ));
                }
                if !port.is_out && origins.is_empty() {
                    return herr(format!("plugin {}: in-port {} has no origins", decl.name, port.name));
                }
            }
            if let Some(extra) = bindings.keys().find(|k| !lib.ports.iter().any(|p| &p.name == *k)) {
                return herr(format!("plugin {}: manifest binds unknown port {extra}", decl.name));
            }
            health.event(serde_json::json!({
                "event": "loaded", "plugin": decl.name, "library": lib.name, "version": lib.version, "sha256": lib.sha256,
            }));
            loaded.push((decl, bindings, lib));
        }
        // One writer per channel per node, and one schema per channel.
        let mut writers: BTreeMap<ChannelId, &str> = BTreeMap::new();
        let mut schemas: BTreeMap<ChannelId, (&str, &str)> = BTreeMap::new();
        for (decl, bindings, lib) in &loaded {
            for port in lib.ports.iter().filter(|p| bindings.contains_key(&p.name)) {
                let channel = bindings[&port.name].0;
                if port.is_out {
                    if let Some(other) = writers.insert(channel, &decl.name) {
                        return herr(format!("channel {} has two writers on this node: {other} and {}", resolved.channels[channel as usize].name, decl.name));
                    }
                }
                if let Some((schema, owner)) = schemas.insert(channel, (&port.schema_id, &decl.name)) {
                    if schema != port.schema_id {
                        return herr(format!(
                            "channel {}: {} uses schema {schema} but {} uses {}",
                            resolved.channels[channel as usize].name, owner, decl.name, port.schema_id
                        ));
                    }
                }
            }
        }
        timings.plugins_loaded_ms = ms_since(started);

        // Same-process wiring: every in-port that lists this node reads the
        // local writer of its channel from memory.
        let link = s.roster.len() > 1;
        let inputs: Vec<Vec<Option<InputSpec>>> = loaded
            .iter()
            .map(|(decl, bindings, lib)| {
                lib.ports
                    .iter()
                    .map(|p| {
                        (!p.is_out && bindings.contains_key(&p.name)).then(|| {
                            let (channel, origins) = bindings[&p.name].clone();
                            InputSpec { channel, origins, latest: decl.bind[&p.name].latest }
                        })
                    })
                    .collect()
            })
            .collect();
        let local_readers = |channel: ChannelId| -> Vec<(usize, u32)> {
            let mut readers = Vec::new();
            for (j, ports) in inputs.iter().enumerate() {
                for (q, spec) in ports.iter().enumerate() {
                    if let Some(spec) = spec {
                        if spec.channel == channel && spec.origins.contains(&node_id) {
                            readers.push((j, q as u32));
                        }
                    }
                }
            }
            readers
        };

        let ctx = TransportContext {
            session: s.id.clone(),
            node: s.node.clone(),
            node_id,
            roster: s.roster.clone(),
            channels: resolved.channels.clone(),
        };
        let transport_kind = transport.kind().to_owned();
        let audit_sink: Arc<dyn AuditSink> = audit.clone();
        let endpoint = Endpoint::open(transport, &ctx, clock.clone(), audit_sink, opts.rx_queue)
            .map_err(|e| HostError(format!("transport {transport_kind}: {e}")))?;

        let mut modules = Vec::new();
        let mut out_channels = BTreeMap::new();
        let mut in_streams: BTreeMap<ChannelId, Vec<OriginId>> = BTreeMap::new();
        for ((decl, bindings, lib), inputs) in loaded.into_iter().zip(inputs.iter().cloned()) {
            let unbound = lib.ports.iter().map(|p| !bindings.contains_key(&p.name)).collect();
            let outputs = lib
                .ports
                .iter()
                .map(|p| {
                    (p.is_out && bindings.contains_key(&p.name)).then(|| {
                        let channel = bindings[&p.name].0;
                        out_channels.insert(channel, ());
                        OutRoute { channel, readers: local_readers(channel) }
                    })
                })
                .collect();
            for spec in inputs.iter().flatten() {
                let set = in_streams.entry(spec.channel).or_default();
                for o in spec.origins.iter().filter(|o| **o != node_id) {
                    if !set.contains(o) {
                        set.push(*o);
                    }
                }
            }
            let config = toml::to_string(&decl.config).map_err(|e| HostError(format!("plugin {} config: {e}", decl.name)))?;
            let budget = decl.step_budget_ms.map_or(Duration::from_nanos(resolved.period_ns as u64), |ms| Duration::from_secs_f64(ms / 1e3));
            modules.push(Module {
                name: decl.name.clone(),
                name_json: serde_json::to_string(&decl.name).map_err(|e| HostError(format!("plugin {} name: {e}", decl.name)))?,
                lib: Arc::new(lib),
                trigger: decl.trigger,
                wake: decl.wake_ms.map(|ms| Duration::from_secs_f64(ms / 1e3)),
                restart: decl.restart,
                config: CString::new(config).map_err(|_| HostError(format!("plugin {} config has NUL", decl.name)))?,
                budget,
                current: RwLock::new(Instance::new(&inputs, None)),
                inputs,
                outputs,
                unbound,
                status: Mutex::new(Status::default()),
                thread: Mutex::new(None),
                steps: AtomicU64::new(0),
                published: AtomicU64::new(0),
                consumed: AtomicU64::new(0),
            });
        }
        if link {
            for channel in out_channels.keys() {
                endpoint.declare_out(*channel).map_err(|e| HostError(e.0))?;
            }
            for (channel, origins) in in_streams.iter().filter(|(_, o)| !o.is_empty()) {
                endpoint.declare_in(*channel, origins).map_err(|e| HostError(e.0))?;
            }
        }
        let clock_service = match resolved.clock.clone() {
            None => None,
            Some(spec) => Some(
                crate::clock_service::ClockService::new(
                    spec,
                    node_id,
                    manifest.session.roster.len(),
                    endpoint.clone(),
                    &audit.dir().join("clock.jsonl"),
                )
                .map_err(|e| HostError(format!("clock probe: {e}")))?,
            ),
        };
        timings.ports_ready_ms = ms_since(started);

        let clock_source = match (manifest.clock_source.clone(), source) {
            (Some(spec), Some(source)) => {
                let module = modules.iter().find(|m| m.name == spec.plugin).unwrap();
                let decl = manifest.plugins.iter().find(|p| p.name == spec.plugin).unwrap();
                let node_name = decl.config["node_name"].as_str().unwrap();
                Some(crate::clock_source::ClockSource::new(spec, module.lib.clone(), source, node_name).map_err(HostError)?)
            }
            _ => None,
        };
        let steps_every = manifest.audit.steps_every.max(1);
        let rt = Arc::new(Runtime { node_id, link, clock, endpoint, health, steps, steps_every, modules, started });
        Ok(Self { manifest, resolved, rt, audit, transport_kind, timings, started, run_dir, clock_service, clock_source })
    }

    /// Deliver link frames to every input that listens for their channel
    /// and origin.
    fn route(&mut self, frames: Vec<RxFrame>) {
        let rt = &self.rt;
        if rt.clock.dispatch_stamp().is_some_and(|s| !s.runnable) { return; }
        let mut batches: Vec<Vec<(u32, Arc<Sample>)>> = vec![Vec::new(); rt.modules.len()];
        for frame in frames {
            if let Some(cs) = self.clock_service.as_mut() {
                if cs.on_frame(&frame) {
                    continue;
                }
            }
            let (channel, origin) = (frame.header.channel, frame.header.origin);
            let sample = Arc::new(Sample::from_frame(frame));
            for (m, module) in rt.modules.iter().enumerate() {
                for (q, spec) in module.inputs.iter().enumerate() {
                    if spec.as_ref().is_some_and(|s| s.channel == channel && s.origins.contains(&origin)) {
                        batches[m].push((q as u32, sample.clone()));
                    }
                }
            }
        }
        let now = rt.clock.now();
        for (module, batch) in rt.modules.iter().zip(batches).filter(|(_, b)| !b.is_empty()) {
            module.instance().deliver_all(batch, rt.endpoint.audit().as_ref(), now);
        }
    }

    /// Abandon hung instances, start replacements whose backoff passed, and
    /// return when to look again. Sets `aborted` past the abandon limit.
    fn watchdog(&self, schedule: Option<RoundSchedule>, abandoned: &mut u32, aborted: &mut Option<String>) -> Option<Instant> {
        let rt = &self.rt;
        let now_mono = rt.mono_ns();
        let mut next: Option<Instant> = None;
        for (m, module) in rt.modules.iter().enumerate() {
            let inst = module.instance();
            let started = inst.call_started.load(Ordering::Acquire);
            if started != 0 && !inst.abandoned.load(Ordering::Acquire) {
                let hang = module.limit(&inst).as_nanos() as u64;
                let busy = now_mono.saturating_sub(started);
                if busy <= hang {
                    let at = Instant::now() + Duration::from_nanos(hang - busy + 1);
                    next = Some(next.map_or(at, |n| n.min(at)));
                    continue;
                }
                inst.abandoned.store(true, Ordering::Release);
                *abandoned += 1;
                {
                    let mut st = module.status.lock().unwrap();
                    st.abandons += 1;
                }
                rt.health.event(serde_json::json!({ "event": "abandoned", "plugin": module.name, "busy_ms": busy as f64 / 1e6, "hang_ms": hang as f64 / 1e6 }));
                rt.fault(m, &format!("hung for more than {} ms; instance abandoned", hang / 1_000_000));
                if *abandoned >= self.manifest.session.max_abandoned && aborted.is_none() {
                    *aborted = Some(format!("{abandoned} hung module instance(s) abandoned"));
                }
            }
            // A replacement for an abandoned instance, once the backoff passed.
            if inst.abandoned.load(Ordering::Acquire) && aborted.is_none() {
                let due = module.status.lock().unwrap().restart_at;
                match due {
                    Some(at) if Instant::now() >= at => {
                        let fresh = Instance::new(&module.inputs, schedule);
                        *module.current.write().unwrap() = fresh.clone();
                        {
                            let mut st = module.status.lock().unwrap();
                            st.restarts += 1;
                            st.restart_at = None;
                        }
                        rt.transition(m, Event::Reset, Some("replace abandoned instance"));
                        if let Err(e) = spawn_module(rt, m, fresh, None) {
                            rt.fault(m, &e.0);
                        }
                    }
                    Some(at) => next = Some(next.map_or(at, |n| n.min(at))),
                    None => {}
                }
            }
        }
        next
    }

    /// Run until `stop` is set, `session.run_for_ms` after E0 elapses, or
    /// too many hung modules were abandoned.
    pub fn run(mut self, stop: &AtomicBool) -> Result<RunSummary, HostError> {
        let rt = self.rt.clone();
        let mut abandoned = 0u32;
        let mut aborted = None;
        if let Some(source) = self.clock_source.as_mut() {
            source.start(rt.health.clone()).map_err(HostError)?;
            if let Err(reason) = source.wait_first(stop) {
                rt.health.event(serde_json::json!({"event":"clock_source_startup_failed", "reason":reason}));
                let _ = source.shutdown();
                self.audit.finish().map_err(|e| HostError(e.to_string()))?;
                return Err(HostError(reason));
            }
        }

        // Start one thread per module; each creates and configures its own
        // instance. Wait for all, watching for hangs in create/configure.
        let (ready_tx, ready_rx) = mpsc::channel();
        for m in 0..rt.modules.len() {
            spawn_module(&rt, m, rt.modules[m].instance(), Some(ready_tx.clone()))?;
        }
        drop(ready_tx);
        let mut pending = rt.modules.len();
        while pending > 0 {
            match ready_rx.recv_timeout(Duration::from_millis(20)) {
                Ok(_) => pending -= 1,
                Err(mpsc::RecvTimeoutError::Disconnected) => break,
                Err(mpsc::RecvTimeoutError::Timeout) => {
                    let before = abandoned;
                    self.watchdog(None, &mut abandoned, &mut aborted);
                    pending -= (abandoned - before) as usize;
                }
            }
            if stop.load(Ordering::Relaxed) {
                break;
            }
        }
        self.timings.configured_ms = ms_since(self.started);

        // Clock gate: a probe client waits until the bound is within the gate
        // (at least 3 samples), probing every 100 ms. On timeout it runs on
        // with frames flagged CLOCK_DEGRADED, and the health log says so.
        if let Some(timeout) = self.clock_service.as_ref().filter(|c| c.is_client()).map(|c| c.gate_timeout()) {
            let deadline = Instant::now() + timeout;
            loop {
                let frames = rt.endpoint.drain();
                self.route(frames);
                let cs = self.clock_service.as_mut().unwrap();
                if cs.within_gate() || Instant::now() >= deadline || stop.load(Ordering::Relaxed) {
                    break;
                }
                cs.tick(true);
                rt.endpoint.wait(Duration::from_millis(20));
            }
            let cs = self.clock_service.as_ref().unwrap();
            let estimate = cs.estimate();
            if cs.within_gate() {
                rt.health.event(serde_json::json!({ "event": "clock_gate_passed", "estimate": format!("{estimate:?}") }));
            } else {
                rt.endpoint.set_clock_degraded(true);
                rt.health.event(serde_json::json!({ "event": "clock_gate_timeout", "estimate": format!("{estimate:?}") }));
            }
        }
        self.timings.clock_ok_ms = ms_since(self.started);
        rt.health.event(serde_json::json!({ "event": "startup", "timings": self.timings }));

        let s = &self.manifest.session;
        let e0 = s.epoch_ns.unwrap_or_else(|| rt.clock.now() + (s.start_delay_ms as i64) * 1_000_000);
        if s.epoch_ns.is_none() && rt.link {
            // Rounds are only agreed absolute times across nodes when every
            // node has the same E0 (the Session's epoch_ns). A local E0 still
            // runs, but its boundaries are this process's own.
            rt.health.event(serde_json::json!({ "event": "epoch_local_only", "e0": e0, "roster": s.roster.len() }));
        }
        let schedule = RoundSchedule::new(e0, self.resolved.period_ns, self.resolved.publish_deadline_ns);
        let stop_at = s.run_for_ms.map(|ms| e0 + (ms as i64) * 1_000_000);
        rt.health.event(serde_json::json!({ "event": "epoch", "e0": e0, "period_ns": schedule.period }));
        if let Some(source) = self.clock_source.as_ref() {
            if source.clock.snapshot().fault.is_some() {
                aborted = source.clock.snapshot().fault;
            } else if !stop.load(Ordering::Relaxed) && rt.clock.now() >= e0 {
                source.clock.fail("shared simulator epoch passed before startup completed; new Session required");
                aborted = source.clock.snapshot().fault;
            } else {
                source.clock.arm(!stop.load(Ordering::Relaxed));
            }
        }
        for module in &rt.modules {
            module.instance().set_schedule(schedule);
        }

        let mut last_round: Option<u64> = None;
        let mut rounds = 0u64;
        let mut wakeups = 0u64;
        let mut last_liveness = Instant::now();
        loop {
            if let Some(source) = self.clock_source.as_ref() {
                source.check_health();
                source.clock.commit_host_time();
                if let Some(reason) = source.clock.snapshot().fault { aborted = Some(reason); }
                if last_liveness.elapsed() >= Duration::from_secs(1) {
                    let snapshot = source.clock.snapshot();
                    rt.health.event(serde_json::json!({"event":"host_liveness", "clock_runnable":snapshot.runnable, "clock_generation":snapshot.generation}));
                    last_liveness = Instant::now();
                }
            }
            let now = rt.clock.now();
            if stop.load(Ordering::Relaxed) || stop_at.is_some_and(|t| now >= t) || aborted.is_some() {
                break;
            }
            let frames = rt.endpoint.drain();
            self.route(frames);
            if let Some(cs) = self.clock_service.as_mut() {
                cs.tick(false);
            }
            if let Some(k) = schedule.round_at(now) {
                if last_round.is_none() {
                    self.timings.first_round_ms = ms_since(self.started);
                    rt.health.event(serde_json::json!({ "event": "first_round", "first_round_ms": self.timings.first_round_ms }));
                }
                if last_round != Some(k) {
                    rounds += 1;
                    last_round = Some(k);
                }
            }
            let watch = self.watchdog(Some(schedule), &mut abandoned, &mut aborted);
            // Sleep until the next boundary, the stop time, a watchdog or
            // probe deadline, or a received frame.
            let now = rt.clock.now();
            let mut wake = schedule.next_boundary_after(now);
            if let Some(t) = stop_at {
                wake = wake.min(t);
            }
            let mut timeout = Duration::from_nanos((wake - now).max(0) as u64);
            for at in watch.into_iter().chain(self.clock_service.as_ref().and_then(|c| c.next_due())) {
                timeout = timeout.min(at.saturating_duration_since(Instant::now()));
            }
            // Before the shared epoch there are no module/frame wakeups. Poll
            // on the source's wall cadence so normal advances are committed
            // without accumulating into a false max-advance fault.
            if now < e0 {
                if let Some(spec) = &self.manifest.clock_source {
                    timeout = timeout.min(Duration::from_millis(spec.poll_wall_ms));
                }
            }
            rt.endpoint.wait(timeout.min(MAX_WAIT));
            wakeups += 1;
        }
        // Close the external output gate before ordinary module deactivation.
        if let Some(source) = self.clock_source.as_mut() {
            if let Err(reason) = source.shutdown() { aborted = Some(reason); }
        }
        if let Some(reason) = &aborted {
            rt.health.event(serde_json::json!({ "event": "aborted", "reason": reason }));
        }

        // Orderly stop: every module thread deactivates, reads its domain
        // state and destroys its instance. A module still inside plugin code
        // after its hang time is abandoned.
        for module in &rt.modules {
            module.instance().stop();
        }
        loop {
            let mut waiting = false;
            let now_mono = rt.mono_ns();
            for (m, module) in rt.modules.iter().enumerate() {
                let inst = module.instance();
                if inst.done.load(Ordering::Acquire) || inst.abandoned.load(Ordering::Acquire) {
                    continue;
                }
                let started = inst.call_started.load(Ordering::Acquire);
                if started != 0 && now_mono.saturating_sub(started) > module.limit(&inst).as_nanos() as u64 {
                    inst.abandoned.store(true, Ordering::Release);
                    module.status.lock().unwrap().abandons += 1;
                    rt.fault(m, "hung at stop; instance abandoned");
                    continue;
                }
                let finished = module.thread.lock().unwrap().as_ref().map_or(true, |h| h.is_finished());
                waiting |= !finished;
            }
            if !waiting {
                break;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        for module in &rt.modules {
            let handle = module.thread.lock().unwrap().take();
            if let Some(h) = handle.filter(|h| h.is_finished()) {
                let _ = h.join();
            }
        }

        let plugins = rt
            .modules
            .iter()
            .map(|module| {
                let st = module.status.lock().unwrap();
                PluginSummary {
                    name: module.name.clone(),
                    library: module.lib.name.clone(),
                    version: module.lib.version.clone(),
                    sha256: module.lib.sha256.clone(),
                    state: st.fsm.state().name().into(),
                    domain_state: st.domain_state.clone(),
                    steps: module.steps.load(Ordering::Relaxed),
                    published: module.published.load(Ordering::Relaxed),
                    consumed: module.consumed.load(Ordering::Relaxed),
                    dropped: module.instance().inner.lock().unwrap().dropped,
                    restarts: st.restarts,
                    abandons: st.abandons,
                    last_error: st.last_error.clone(),
                }
            })
            .collect();
        for module in rt.modules.iter().filter(|m| m.instance().done.load(Ordering::Acquire)) {
            let _ = module.status.lock().unwrap().fsm.apply(Event::Shutdown);
        }
        rt.endpoint.close();
        let summary = RunSummary {
            session: self.manifest.session.id.clone(),
            node: self.manifest.session.node.clone(),
            transport: self.transport_kind.into(),
            e0_ns: e0,
            period_ns: schedule.period,
            rounds,
            wakeups,
            timings: self.timings.clone(),
            plugins,
            aborted,
            audit_dir: self.run_dir.clone(),
        };
        rt.health.event(serde_json::json!({ "event": "stopped", "rounds": rounds, "wakeups": wakeups }));
        rt.steps.finish();
        rt.health.output.finish();
        self.audit.finish().map_err(|e| HostError(format!("audit: {e}")))?;
        Ok(summary)
    }
}

fn ms_since(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

#[cfg(test)]
mod log_tests {
    use super::*;

    #[test]
    fn a_step_record_is_what_serde_json_writes() {
        for (name, k, t0, t1, reads) in [
            ("estimation", 0u64, 0i64, 1i64, vec![]),
            ("plant_3", 17, -5, 1_234_567_890_123, vec![(0u32, 0 as OriginId, 1u64)]),
            ("edge/\"quoted\"\\\u{1}", u64::MAX, i64::MIN, i64::MAX, vec![(63, OriginId::MAX, u64::MAX), (2, 1, 9)]),
        ] {
            let reads_json: Vec<_> = reads.iter().map(|&(p, o, q)| [p as u64, o as u64, q]).collect();
            let expected = serde_json::json!({ "m": name, "k": k, "t0": t0, "t1": t1, "in": reads_json }).to_string();
            assert_eq!(step_record(&serde_json::to_string(name).unwrap(), k, t0, t1, &reads), expected);
        }
    }

    struct PausedWriter {
        entered: Option<mpsc::Sender<()>>,
        resume: mpsc::Receiver<()>,
        bytes: Arc<Mutex<Vec<u8>>>,
    }

    impl Write for PausedWriter {
        fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
            if let Some(entered) = self.entered.take() {
                entered.send(()).unwrap();
                self.resume.recv().unwrap();
            }
            self.bytes.lock().unwrap().extend_from_slice(bytes);
            Ok(bytes.len())
        }

        fn flush(&mut self) -> std::io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn health_events_do_not_wait_for_the_writer_and_finish_drains_them() {
        let (entered_tx, entered_rx) = mpsc::channel();
        let (resume_tx, resume_rx) = mpsc::channel();
        let bytes = Arc::new(Mutex::new(Vec::new()));
        let log = Arc::new(HealthLog {
            output: LineLog::spawn(PausedWriter {
                entered: Some(entered_tx), resume: resume_rx, bytes: bytes.clone(),
            }, false, "test-health").unwrap(),
            clock: Arc::new(xgc_rt_core::clock::WallClock::new(0)),
            started: Instant::now(),
        });
        log.event(serde_json::json!({"event":"first"}));
        entered_rx.recv_timeout(Duration::from_secs(2)).unwrap();
        let (sent_tx, sent_rx) = mpsc::channel();
        let producer_log = log.clone();
        let producer = std::thread::spawn(move || {
            producer_log.event(serde_json::json!({"event":"second"}));
            let _ = sent_tx.send(());
        });
        let sent_while_writer_paused = sent_rx.recv_timeout(Duration::from_secs(2)).is_ok();
        resume_tx.send(()).unwrap();
        producer.join().unwrap();
        log.output.finish();
        assert!(sent_while_writer_paused, "plugin logging waited for output I/O");
        let text = String::from_utf8(bytes.lock().unwrap().clone()).unwrap();
        let events: Vec<serde_json::Value> = text.lines().map(|line| serde_json::from_str(line).unwrap()).collect();
        assert_eq!(events.len(), 2);
        assert_eq!(events[0]["event"], "first");
        assert_eq!(events[1]["event"], "second");
        assert!(events[0]["steady_elapsed_ns"].is_u64());
        assert!(events[1]["t"].is_i64());
    }
}
