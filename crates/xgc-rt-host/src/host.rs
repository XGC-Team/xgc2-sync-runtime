//! The host executor.
//!
//! **Threads.**
//! - One executor thread runs every plugin vtable call, so a plugin is never
//!   called concurrently.
//! - Transport IO threads only run the endpoint sink (stamp, verify, audit,
//!   enqueue).
//! - One audit writer thread writes records.
//!
//! No other thread touches a plugin.
//!
//! **Wake rule (dirty loop).** The executor sleeps on the receive queue until
//! the next round boundary or until a frame arrives, and never busy-polls.
//! - A plugin with trigger `on_round` steps once per round.
//! - A plugin with trigger `on_dirty` steps only when one of its in-ports
//!   got a *new* sample since its last step. Dirty bits are edge-set on
//!   routing and cleared after each step. Unread samples stay queued but do
//!   not retrigger.
//!
//! **Soundness.** Plugins call back into their `Slot` through the raw
//! `host` pointer. The executor therefore holds no Rust reference to a
//! slot across a vtable call: it copies the function pointer and instance
//! out, calls, then re-borrows.

use std::collections::{BTreeMap, VecDeque};
use std::ffi::{c_char, c_void, CStr, CString};
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use serde::Serialize;
use xgc_rt_abi::*;
use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::audit::{AuditSink, OverflowSite};
use xgc_rt_core::clock::{Clock, ClockDomain, RoundSchedule};
use xgc_rt_core::lifecycle::{Event, Lifecycle, State};
use xgc_rt_core::manifest::{Manifest, RestartKind, Resolved, Trigger};
use xgc_rt_core::transport::{Transport, TransportContext};
use xgc_rt_core::{ChannelId, OriginId};

use crate::endpoint::{Endpoint, RxFrame, DEFAULT_RX_QUEUE};
use crate::plugin::{self, LoadedPlugin};

pub const INBOX_CAPACITY: usize = 1024;

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

enum PortRuntime {
    In { channel: ChannelId, origins: Vec<OriginId>, inbox: VecDeque<RxFrame> },
    Out { channel: ChannelId },
}

/// Per-plugin state. It is boxed, so `api.host` can point at it for the
/// plugin's lifetime.
struct Slot {
    name: String,
    node_id: OriginId,
    lib: LoadedPlugin,
    trigger: Trigger,
    restart: xgc_rt_core::manifest::RestartPolicy,
    config: CString,
    fsm: Lifecycle,
    api: XgcHostApi,
    instance: *mut c_void,
    ports: Vec<PortRuntime>,
    dirty: u64,
    current: Option<RxFrame>,
    round: u64,
    endpoint: Arc<Endpoint>,
    health: Arc<HealthLog>,
    degrade_request: Option<String>,
    recover_request: bool,
    restarts: u32,
    restart_at: Option<Instant>,
    steps: u64,
    published: u64,
    consumed: u64,
    last_error: Option<String>,
}

/// Append-only `health.jsonl`: timings, lifecycle transitions, plugin logs
/// and faults.
pub struct HealthLog {
    file: Mutex<Option<File>>,
    clock: Arc<dyn Clock>,
    echo: bool,
}

impl HealthLog {
    fn open(path: &Path, clock: Arc<dyn Clock>, echo: bool) -> std::io::Result<Self> {
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(Self { file: Mutex::new(Some(file)), clock, echo })
    }

    pub fn event(&self, value: serde_json::Value) {
        let mut line = serde_json::json!({ "t": self.clock.now() });
        if let (Some(obj), serde_json::Value::Object(extra)) = (line.as_object_mut(), value) {
            obj.extend(extra);
        }
        let text = line.to_string();
        if self.echo {
            eprintln!("{text}");
        }
        if let Some(f) = self.file.lock().unwrap().as_mut() {
            let _ = writeln!(f, "{text}");
        }
    }
}

// --- host API callbacks (executor thread only, inside a vtable call) -------

unsafe fn slot<'a>(host: *mut c_void) -> &'a mut Slot {
    &mut *host.cast::<Slot>()
}

unsafe extern "C" fn api_publish(host: *mut c_void, port: u32, round: u64, data: *const u8, len: u32) -> XgcStatus {
    let s = slot(host);
    let Some(PortRuntime::Out { channel }) = s.ports.get(port as usize) else {
        return XGC_ERR_INVALID;
    };
    if len > 0 && data.is_null() {
        return XGC_ERR_INVALID;
    }
    let payload = if len == 0 { &[][..] } else { std::slice::from_raw_parts(data, len as usize) };
    let t_produce = s.endpoint.clock().now();
    match s.endpoint.publish(*channel, round, t_produce, payload) {
        Ok(_) => {
            s.published += 1;
            XGC_OK
        }
        Err(e) => {
            s.health.event(serde_json::json!({ "event": "publish_error", "plugin": s.name, "port": port, "error": e.0 }));
            XGC_ERR
        }
    }
}

unsafe extern "C" fn api_next(host: *mut c_void, port: u32, out: *mut XgcSampleView) -> XgcStatus {
    let s = slot(host);
    if out.is_null() {
        return XGC_ERR_INVALID;
    }
    let Some(PortRuntime::In { inbox, .. }) = s.ports.get_mut(port as usize) else {
        return XGC_ERR_INVALID;
    };
    let Some(frame) = inbox.pop_front() else {
        return XGC_ERR_AGAIN;
    };
    s.endpoint.audit().consumed(&frame.header, s.endpoint.clock().now());
    s.consumed += 1;
    let current = s.current.insert(frame);
    *out = XgcSampleView {
        origin: current.header.origin,
        reserved: 0,
        len: current.payload.len() as u32,
        seq: current.header.seq,
        round: current.header.round,
        t_produce: current.header.t_produce,
        t_tx: current.header.t_tx,
        t_rx: current.t_rx,
        data: current.payload.as_ptr(),
    };
    XGC_OK
}

unsafe extern "C" fn api_now(host: *mut c_void) -> i64 {
    slot(host).endpoint.clock().now()
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
    s.health.event(serde_json::json!({ "event": "log", "plugin": s.name, "level": level, "message": c_text(message) }));
}

unsafe extern "C" fn api_request_degrade(host: *mut c_void, reason: *const c_char) {
    let s = slot(host);
    s.degrade_request = Some(c_text(reason));
    s.recover_request = false;
}

unsafe extern "C" fn api_port_origins(host: *mut c_void, port: u32, out: *mut u16, cap: u32) -> u32 {
    let s = slot(host);
    let Some(PortRuntime::In { origins, .. }) = s.ports.get(port as usize) else {
        return 0;
    };
    if !out.is_null() {
        for (i, o) in origins.iter().take(cap as usize).enumerate() {
            *out.add(i) = *o;
        }
    }
    origins.len() as u32
}

unsafe extern "C" fn api_node_id(host: *mut c_void) -> u16 {
    slot(host).node_id
}

unsafe extern "C" fn api_request_recover(host: *mut c_void) {
    let s = slot(host);
    s.recover_request = true;
    s.degrade_request = None;
}

// --- host -----------------------------------------------------------------

#[derive(Debug, Clone, Default, Serialize)]
pub struct StartupTimings {
    /// Milliseconds from `Host::new` to each milestone.
    pub manifest_ms: f64,
    pub plugins_loaded_ms: f64,
    pub ports_ready_ms: f64,
    /// Every plugin created and configured (on the executor thread).
    pub configured_ms: f64,
    pub clock_ok_ms: f64,
    pub peers_ready_ms: f64,
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
    pub restarts: u32,
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
    /// Executor wake-ups: one per round boundary plus one per receive
    /// burst. It is the idle-cost evidence for the dirty-loop rule.
    pub wakeups: u64,
    pub timings: StartupTimings,
    pub plugins: Vec<PluginSummary>,
    pub audit_dir: PathBuf,
}

pub struct Host {
    manifest: Manifest,
    resolved: Resolved,
    clock: Arc<dyn Clock>,
    endpoint: Arc<Endpoint>,
    audit: Arc<FileAudit>,
    health: Arc<HealthLog>,
    slots: Vec<*mut Slot>,
    transport_kind: &'static str,
    timings: StartupTimings,
    started: Instant,
    run_dir: PathBuf,
    clock_service: Option<crate::clock_service::ClockService>,
}

// SAFETY: slots are only touched by the thread that runs the host.
unsafe impl Send for Host {}

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
    /// Load, validate and wire everything up to "ports ready". No plugin
    /// instance exists yet: `run` creates them on the executor thread. Relative plugin and audit paths resolve against
    /// `base_dir`, the manifest's directory.
    pub fn new(
        manifest: Manifest,
        base_dir: &Path,
        transport: Box<dyn Transport>,
        clock: Arc<dyn Clock>,
        opts: HostOptions,
    ) -> Result<Self, HostError> {
        let started = Instant::now();
        let mut timings = StartupTimings::default();
        let resolved = manifest.resolve().map_err(|e| HostError(e.0))?;
        timings.manifest_ms = ms_since(started);
        let s = &manifest.session;

        let run_dir = base_dir.join(&manifest.audit.dir);
        let meta = NodeMeta {
            format: String::new(),
            session: s.id.clone(),
            node: s.node.clone(),
            node_id: resolved.node_id,
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
        let health = Arc::new(
            HealthLog::open(&audit.dir().join("health.jsonl"), clock.clone(), opts.echo_health)
                .map_err(|e| HostError(format!("health log: {e}")))?,
        );

        // Load and validate every plugin before opening the transport.
        let mut loaded = Vec::new();
        for (decl, bindings) in manifest.plugins.iter().zip(&resolved.bindings) {
            let path = base_dir.join(&decl.path);
            let lib = plugin::load(&path, decl.sha256.as_deref()).map_err(|e| HostError(e.0))?;
            for port in &lib.ports {
                let Some((channel, origins)) = bindings.get(&port.name) else {
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
        // One publisher per channel per node, and one schema per channel.
        let mut writers: BTreeMap<ChannelId, &str> = BTreeMap::new();
        let mut schemas: BTreeMap<ChannelId, (&str, &str)> = BTreeMap::new();
        for (decl, bindings, lib) in &loaded {
            for port in &lib.ports {
                let channel = bindings[&port.name].0;
                if port.is_out {
                    if let Some(other) = writers.insert(channel, &decl.name) {
                        return herr(format!("channel {} has two publishers on this node: {other} and {}", resolved.channels[channel as usize].name, decl.name));
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

        let ctx = TransportContext {
            session: s.id.clone(),
            node: s.node.clone(),
            node_id: resolved.node_id,
            roster: s.roster.clone(),
            channels: resolved.channels.clone(),
        };
        let transport_kind = transport.kind();
        let audit_sink: Arc<dyn AuditSink> = audit.clone();
        let endpoint = Endpoint::open(transport, &ctx, clock.clone(), audit_sink, opts.rx_queue)
            .map_err(|e| HostError(format!("transport {transport_kind}: {e}")))?;

        let mut out_channels = BTreeMap::new();
        let mut in_streams: BTreeMap<ChannelId, Vec<OriginId>> = BTreeMap::new();
        let mut slots = Vec::new();
        for (decl, bindings, lib) in loaded {
            let ports = lib
                .ports
                .iter()
                .map(|p| {
                    let (channel, origins) = bindings[&p.name].clone();
                    if p.is_out {
                        out_channels.insert(channel, ());
                        PortRuntime::Out { channel }
                    } else {
                        let set = in_streams.entry(channel).or_default();
                        for o in &origins {
                            if !set.contains(o) {
                                set.push(*o);
                            }
                        }
                        PortRuntime::In { channel, origins, inbox: VecDeque::new() }
                    }
                })
                .collect();
            let config = toml::to_string(&decl.config).map_err(|e| HostError(format!("plugin {} config: {e}", decl.name)))?;
            let slot = Box::new(Slot {
                name: decl.name.clone(),
                lib,
                trigger: decl.trigger,
                restart: decl.restart,
                config: CString::new(config).map_err(|_| HostError(format!("plugin {} config has NUL", decl.name)))?,
                fsm: Lifecycle::default(),
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
                node_id: resolved.node_id,
                instance: std::ptr::null_mut(),
                ports,
                dirty: 0,
                current: None,
                round: 0,
                endpoint: endpoint.clone(),
                health: health.clone(),
                degrade_request: None,
                recover_request: false,
                restarts: 0,
                restart_at: None,
                steps: 0,
                published: 0,
                consumed: 0,
                last_error: None,
            });
            let ptr = Box::into_raw(slot);
            unsafe { (*ptr).api.host = ptr.cast() };
            slots.push(ptr);
        }
        for channel in out_channels.keys() {
            endpoint.declare_out(*channel).map_err(|e| HostError(e.0))?;
        }
        for (channel, origins) in &in_streams {
            endpoint.declare_in(*channel, origins).map_err(|e| HostError(e.0))?;
        }
        let clock_service = match resolved.clock.clone() {
            None => None,
            Some(spec) => Some(
                crate::clock_service::ClockService::new(
                    spec,
                    resolved.node_id,
                    manifest.session.roster.len(),
                    endpoint.clone(),
                    &audit.dir().join("clock.jsonl"),
                )
                .map_err(|e| HostError(format!("clock probe: {e}")))?,
            ),
        };
        timings.ports_ready_ms = ms_since(started);

        let host = Self {
            manifest,
            resolved,
            clock,
            endpoint,
            audit,
            health,
            slots,
            transport_kind,
            timings,
            started,
            run_dir,
            clock_service,
        };
        // Plugins are created and configured in `run`, on the executor
        // thread: the ABI promises every vtable call happens there, and
        // wrapped code (e.g. libxgc2-state-machine) enforces thread ownership.
        Ok(host)
    }

    #[allow(clippy::mut_from_ref)]
    fn slot(&self, i: usize) -> &mut Slot {
        let ptr = self.slots[i];
        // SAFETY: executor thread only, and never held across a vtable call.
        unsafe { &mut *ptr }
    }

    fn transition(&self, i: usize, event: Event, detail: Option<&str>) {
        let s = self.slot(i);
        let from = s.fsm.state();
        match s.fsm.apply(event) {
            Ok(to) => self.health.event(serde_json::json!({
                "event": "transition", "plugin": s.name, "from": from.name(), "to": to.name(), "cause": format!("{event:?}"), "detail": detail,
            })),
            Err(e) => self.health.event(serde_json::json!({ "event": "invalid_transition", "plugin": s.name, "error": e.to_string() })),
        }
    }

    fn fault(&self, i: usize, what: &str) {
        let s = self.slot(i);
        s.last_error = Some(what.to_string());
        self.transition(i, Event::Fault, Some(what));
        let s = self.slot(i);
        if s.restart.policy == RestartKind::OnError && s.restarts < s.restart.max {
            s.restart_at = Some(Instant::now() + Duration::from_millis(s.restart.backoff_ms));
        }
    }

    /// create + configure: Unconfigured → Inactive.
    fn bring_up(&self, i: usize) -> Result<(), HostError> {
        let (create, configure, api, config) = {
            let s = self.slot(i);
            (s.lib.vtbl.create, s.lib.vtbl.configure, &s.api as *const XgcHostApi, s.config.as_ptr())
        };
        let instance = unsafe { create(api) };
        if instance.is_null() {
            self.fault(i, "create returned NULL");
            return Ok(());
        }
        self.slot(i).instance = instance;
        let status = unsafe { configure(instance, config) };
        if status == XGC_OK {
            self.transition(i, Event::Configure, None);
        } else {
            self.fault(i, &format!("configure returned {status}"));
        }
        Ok(())
    }

    fn activate(&self, i: usize) {
        if self.slot(i).fsm.state() != State::Inactive {
            return;
        }
        let (f, instance) = (self.slot(i).lib.vtbl.activate, self.slot(i).instance);
        let status = unsafe { f(instance) };
        if status == XGC_OK {
            self.transition(i, Event::Activate, None);
        } else {
            self.fault(i, &format!("activate returned {status}"));
        }
    }

    fn destroy_instance(&self, i: usize) {
        let (f, instance) = (self.slot(i).lib.vtbl.destroy, self.slot(i).instance);
        if !instance.is_null() {
            unsafe { f(instance) };
            let s = self.slot(i);
            s.instance = std::ptr::null_mut();
            s.current = None;
        }
    }

    fn restart(&self, i: usize) {
        self.destroy_instance(i);
        self.transition(i, Event::Reset, Some("restart policy"));
        {
            let s = self.slot(i);
            s.restarts += 1;
            s.restart_at = None;
            s.dirty = 0;
        }
        let _ = self.bring_up(i);
        self.activate(i);
    }

    fn route(&mut self, frames: Vec<RxFrame>) {
        for frame in frames {
            if let Some(cs) = self.clock_service.as_mut() {
                if cs.on_frame(&frame) {
                    continue;
                }
            }
            for i in 0..self.slots.len() {
                let s = self.slot(i);
                for (p, port) in s.ports.iter_mut().enumerate() {
                    if let PortRuntime::In { channel, origins, inbox } = port {
                        if *channel == frame.header.channel && origins.contains(&frame.header.origin) {
                            if inbox.len() >= INBOX_CAPACITY {
                                inbox.pop_front();
                                self.endpoint.audit().overflow(OverflowSite::Inbox, *channel, frame.header.origin, self.clock.now());
                            }
                            inbox.push_back(frame.clone());
                            s.dirty |= 1u64 << p;
                        }
                    }
                }
            }
        }
    }

    fn step(&self, i: usize, schedule: &RoundSchedule, k: u64, advanced: bool) {
        let (f, instance, ctx) = {
            let s = self.slot(i);
            let run = match s.trigger {
                Trigger::OnRound => advanced,
                Trigger::OnDirty => s.dirty != 0,
                Trigger::Both => advanced || s.dirty != 0,
            };
            if !run || !s.fsm.state().runs() {
                return;
            }
            s.round = k;
            let ctx = XgcStepCtx {
                round: k,
                now: self.clock.now(),
                round_start: schedule.start(k),
                deadline: schedule.deadline(k),
                dirty_ports: s.dirty,
                round_advanced: u32::from(advanced),
                reserved: 0,
            };
            s.dirty = 0;
            s.steps += 1;
            (s.lib.vtbl.step, s.instance, ctx)
        };
        let status = unsafe { f(instance, &ctx) };
        if status != XGC_OK {
            self.fault(i, &format!("step returned {status} in round {k}"));
            return;
        }
        let (degrade, recover, state) = {
            let s = self.slot(i);
            (s.degrade_request.take(), std::mem::take(&mut s.recover_request), s.fsm.state())
        };
        match (degrade, recover, state) {
            (Some(reason), _, State::Active) => self.transition(i, Event::Degrade, Some(&reason)),
            (None, true, State::Degraded) => self.transition(i, Event::Recover, None),
            _ => {}
        }
    }

    /// Run until `stop` is set or `session.run_for_ms` after E0 elapses.
    pub fn run(mut self, stop: &AtomicBool) -> Result<RunSummary, HostError> {
        for i in 0..self.slots.len() {
            self.bring_up(i)?;
        }
        self.timings.configured_ms = ms_since(self.started);
        // Clock gate: a probe client waits until the bound is within the gate
        // (at least 3 samples), probing every 100 ms. On timeout it runs on
        // with frames flagged CLOCK_DEGRADED, and the health log says so.
        if let Some(timeout) = self.clock_service.as_ref().filter(|c| c.is_client()).map(|c| c.gate_timeout()) {
            let deadline = Instant::now() + timeout;
            loop {
                let frames = self.endpoint.drain();
                self.route(frames);
                let cs = self.clock_service.as_mut().unwrap();
                if cs.within_gate() || Instant::now() >= deadline || stop.load(Ordering::Relaxed) {
                    break;
                }
                cs.tick(true);
                self.endpoint.wait(Duration::from_millis(20));
            }
            let cs = self.clock_service.as_ref().unwrap();
            let estimate = cs.estimate();
            if cs.within_gate() {
                self.health.event(serde_json::json!({ "event": "clock_gate_passed", "estimate": format!("{estimate:?}") }));
            } else {
                self.endpoint.set_clock_degraded(true);
                self.health.event(serde_json::json!({ "event": "clock_gate_timeout", "estimate": format!("{estimate:?}") }));
            }
        }
        self.timings.clock_ok_ms = ms_since(self.started);
        // Peers: every out-channel has a matching subscriber, or the timeout
        // passes. The host then runs Degraded-by-evidence: the audit shows
        // the loss.
        let ready = self.endpoint.wait_ready(Duration::from_millis(self.manifest.session.peer_timeout_ms));
        self.timings.peers_ready_ms = ms_since(self.started);
        if !ready {
            self.health.event(serde_json::json!({ "event": "peers_timeout", "after_ms": self.timings.peers_ready_ms }));
        }
        self.health.event(serde_json::json!({ "event": "startup", "timings": self.timings }));

        let s = &self.manifest.session;
        let e0 = s.epoch_ns.unwrap_or_else(|| self.clock.now() + (s.start_delay_ms as i64) * 1_000_000);
        let schedule = RoundSchedule::new(e0, self.resolved.period_ns, self.resolved.publish_deadline_ns);
        let stop_at = s.run_for_ms.map(|ms| e0 + (ms as i64) * 1_000_000);
        self.health.event(serde_json::json!({ "event": "epoch", "e0": e0, "period_ns": schedule.period }));

        let mut last_round: Option<u64> = None;
        let mut rounds = 0u64;
        let mut wakeups = 0u64;
        let mut activated = false;
        loop {
            let now = self.clock.now();
            if stop.load(Ordering::Relaxed) || stop_at.is_some_and(|t| now >= t) {
                break;
            }
            let frames = self.endpoint.drain();
            self.route(frames);
            if let Some(cs) = self.clock_service.as_mut() {
                cs.tick(false);
            }
            if let Some(k) = schedule.round_at(now) {
                if !activated {
                    for i in 0..self.slots.len() {
                        self.activate(i);
                    }
                    activated = true;
                    self.timings.first_round_ms = ms_since(self.started);
                    self.health.event(serde_json::json!({ "event": "first_round", "first_round_ms": self.timings.first_round_ms }));
                }
                let advanced = last_round != Some(k);
                if advanced {
                    rounds += 1;
                    last_round = Some(k);
                }
                for i in 0..self.slots.len() {
                    self.step(i, &schedule, k, advanced);
                }
                for i in 0..self.slots.len() {
                    let due = self.slot(i).fsm.state() == State::Error && self.slot(i).restart_at.is_some_and(|t| Instant::now() >= t);
                    if due {
                        self.restart(i);
                    }
                }
            }
            // Sleep until the next boundary, a pending restart, the stop
            // time, or a received frame.
            let now = self.clock.now();
            let mut wake = schedule.next_boundary_after(now);
            if let Some(t) = stop_at {
                wake = wake.min(t);
            }
            let mut timeout = Duration::from_nanos((wake - now).max(0) as u64);
            for i in 0..self.slots.len() {
                if let Some(at) = self.slot(i).restart_at {
                    timeout = timeout.min(at.saturating_duration_since(Instant::now()));
                }
            }
            if let Some(at) = self.clock_service.as_ref().and_then(|c| c.next_due()) {
                timeout = timeout.min(at.saturating_duration_since(Instant::now()));
            }
            // Wake at least every 100 ms so a stop signal is noticed.
            self.endpoint.wait(timeout.min(Duration::from_millis(100)));
            wakeups += 1;
        }

        // Orderly stop: running → Inactive → Finalized, then destroy.
        for i in 0..self.slots.len() {
            if self.slot(i).fsm.state().runs() {
                let (f, instance) = (self.slot(i).lib.vtbl.deactivate, self.slot(i).instance);
                let status = unsafe { f(instance) };
                if status == XGC_OK {
                    self.transition(i, Event::Deactivate, None);
                } else {
                    self.fault(i, &format!("deactivate returned {status}"));
                }
            }
        }
        let plugins = (0..self.slots.len())
            .map(|i| {
                let domain_state = {
                    let (f, instance) = (self.slot(i).lib.vtbl.domain_state, self.slot(i).instance);
                    if instance.is_null() {
                        String::new()
                    } else {
                        unsafe { c_text(f(instance)) }
                    }
                };
                let s = self.slot(i);
                PluginSummary {
                    name: s.name.clone(),
                    library: s.lib.name.clone(),
                    version: s.lib.version.clone(),
                    sha256: s.lib.sha256.clone(),
                    state: s.fsm.state().name().into(),
                    domain_state,
                    steps: s.steps,
                    published: s.published,
                    consumed: s.consumed,
                    restarts: s.restarts,
                    last_error: s.last_error.clone(),
                }
            })
            .collect();
        for i in 0..self.slots.len() {
            self.destroy_instance(i);
            let _ = self.slot(i).fsm.apply(Event::Shutdown);
        }
        self.endpoint.close();
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
            audit_dir: self.run_dir.clone(),
        };
        self.health.event(serde_json::json!({ "event": "stopped", "rounds": rounds, "wakeups": wakeups }));
        self.audit.finish().map_err(|e| HostError(format!("audit: {e}")))?;
        Ok(summary)
    }
}

impl Drop for Host {
    fn drop(&mut self) {
        for i in 0..self.slots.len() {
            self.destroy_instance(i);
        }
        for ptr in self.slots.drain(..) {
            // SAFETY: allocated by Box::into_raw in `new`, and freed once.
            drop(unsafe { Box::from_raw(ptr) });
        }
    }
}

fn ms_since(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}
