//! Aggregate domain administration; no RPC or JSON enters module steps.
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    fs::OpenOptions,
    io::Read,
    os::unix::fs::OpenOptionsExt,
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        mpsc, Arc, Mutex,
    },
    thread,
    time::Duration,
};
use tokio::sync::oneshot;
use xgc2_xrpc::{
    handler, Fault, Limits, Method, PolicyOptions, Runtime, RuntimeOptions, RuntimePolicy,
};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{host::HealthLog, Host, HostOptions};

const DOCUMENT_BYTES: usize = 1 << 20;
const CONFIGURATION_BYTES: usize = 64 << 10;
const MAX_MODULES: usize = 128;
const CONTROL_CAPACITY: usize = 4;
const MAX_LOAD_ATTEMPTS: u64 = 32;

pub fn startup_policy() -> Result<RuntimePolicy, String> {
    use xgc2_xrpc::policy::PolicyOverride;
    let mut options = PolicyOptions::default();
    for (name, value) in [
        ("HOST_MAX_CONNECTIONS", 8),
        ("HOST_MAX_IN_FLIGHT", CONTROL_CAPACITY as u32),
    ] {
        options.explicit.insert(
            name.into(),
            PolicyOverride::integer(value, "sync-runtime/control"),
        );
        options.ceilings.insert(name.into(), value);
    }
    for (name, value) in [
        ("MAX_REQUEST_BYTES", DOCUMENT_BYTES as u32),
        ("MAX_RESPONSE_BYTES", DOCUMENT_BYTES as u32),
    ] {
        options.ceilings.insert(name.into(), value);
    }
    let policy =
        RuntimePolicy::resolve_os(std::env::vars_os(), options).map_err(|e| e.to_string())?;
    policy
        .check_applied([
            "HOST_MAX_CONNECTIONS",
            "HOST_MAX_IN_FLIGHT",
            "MAX_REQUEST_BYTES",
            "MAX_RESPONSE_BYTES",
            "MAX_HEADER_BYTES",
            "CALL_TIMEOUT_MS",
            "HEADER_TIMEOUT_MS",
            "IDLE_TIMEOUT_MS",
            "SHUTDOWN_TIMEOUT_MS",
        ])
        .map_err(|e| e.to_string())?;
    if policy
        .integer("MAX_RESPONSE_BYTES")
        .map_err(|e| e.to_string())?
        < DOCUMENT_BYTES as u32
    {
        return Err(
            "sync-runtime requires MAX_RESPONSE_BYTES >= 1048576 for bounded module health".into(),
        );
    }
    Ok(policy)
}

enum Command {
    Call {
        route: String,
        value: Value,
        deadline: tokio::time::Instant,
        reply: oneshot::Sender<Result<Value, Fault>>,
    },
    Shutdown,
}

/// Manager unwinding must not detach an admitted native supervisor.
struct NativeRun {
    stop: Arc<AtomicBool>,
    running: Option<thread::JoinHandle<()>>,
}
impl NativeRun {
    fn join(&mut self) -> Result<(), Fault> {
        if let Some(run) = self.running.take() {
            run.join()
                .map_err(|_| Fault::new("internal", "native supervisor panicked"))?;
        }
        Ok(())
    }
}
impl Drop for NativeRun {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        let _ = self.join();
    }
}

struct Definition {
    manifest: Manifest,
    base: PathBuf,
    source_sha256: String,
}
struct State {
    phase: &'static str,
    generation: u64,
    applied_revision: Option<u64>,
    modules: Vec<String>,
    configuration: Value,
    summary: Value,
    last_error: Option<String>,
    health: Option<Arc<HealthLog>>,
    restart_required: bool,
    source_manifest_sha256: Option<String>,
    notifications: tokio::sync::watch::Sender<u64>,
}
impl State {
    fn configuration(&self) -> Value {
        let evidence = self
            .health
            .as_ref()
            .and_then(|h| h.configuration_snapshot())
            .map(bounded_native);
        let applied = evidence
            .as_ref()
            .filter(|e| e["applied"] == true)
            .map(|_| self.generation)
            .or(self.applied_revision);
        json!({"desired_revision":self.generation, "applied_revision":applied,
            "persisted_revision":null, "persistence":"ephemeral",
            "live_change":false, "application":"native-configure-on-start",
            "modules":self.configuration, "evidence":evidence})
    }
    fn value(&self) -> Value {
        let live = if self.phase == "running" {
            self.health
                .as_ref()
                .and_then(|h| h.snapshot())
                .map(bounded_native)
        } else {
            None
        };
        json!({"state":self.phase,"generation":self.generation,"modules":self.modules,
            "summary":self.summary,"last_error":self.last_error,"live":live,
            "configuration":self.configuration(),"restart_required":self.restart_required,
            "source_manifest_sha256":self.source_manifest_sha256,
            "audit":self.health.as_ref().map(|h|h.audit_snapshot()),
            "event_revision":*self.notifications.borrow()})
    }
}

/// These are bootstrap grants, never inferred from an administration request.
#[derive(Clone)]
pub struct Grants {
    pub module_root: PathBuf,
    pub audit_root: PathBuf,
    pub document_root: PathBuf,
}
impl Grants {
    pub fn new(
        module_root: PathBuf,
        audit_root: PathBuf,
        document_root: PathBuf,
    ) -> Result<Self, String> {
        if !module_root.is_absolute() || !audit_root.is_absolute() || !document_root.is_absolute() {
            return Err("bootstrap roots must be absolute".into());
        }
        let module_root = module_root.canonicalize().map_err(|e| e.to_string())?;
        let audit_root = audit_root.canonicalize().map_err(|e| e.to_string())?;
        let document_root = document_root.canonicalize().map_err(|e| e.to_string())?;
        if !module_root.is_dir() || !audit_root.is_dir() || !document_root.is_dir() {
            return Err("bootstrap grants must be directories".into());
        }
        Ok(Self {
            module_root,
            audit_root,
            document_root,
        })
    }
    fn validate(&self, definition: &Definition) -> Result<(), String> {
        let base = definition.base.canonicalize().map_err(|e| e.to_string())?;
        if !base.starts_with(&self.document_root) {
            return Err("base_dir outside document grant".into());
        }
        if definition.manifest.plugins.len() > MAX_MODULES {
            return Err("more than 128 modules".into());
        }
        if definition.manifest.session.id.len() > 256
            || definition.manifest.session.node.len() > 256
            || definition
                .manifest
                .plugins
                .iter()
                .any(|module| module.name.len() > 128)
        {
            return Err("session identities and module names exceed management bounds".into());
        }
        if definition.manifest.audit.max_bytes > 256 * 1024 * 1024 {
            return Err("audit.max_bytes exceeds 256 MiB host ceiling".into());
        }
        for path in definition
            .manifest
            .plugins
            .iter()
            .map(|p| &p.path)
            .chain(definition.manifest.transport.path.iter())
        {
            let actual = base.join(path).canonicalize().map_err(|e| e.to_string())?;
            if !actual.starts_with(&self.module_root) || !actual.is_file() {
                return Err("module or transport outside regular-file grant".into());
            }
        }
        if let Some(clock) = &definition.manifest.clock_source {
            // This field names a module instance. Its declared library path was
            // checked with every other module above; it is not another path.
            if !definition
                .manifest
                .plugins
                .iter()
                .any(|module| module.name == clock.plugin)
            {
                return Err("clock source module is absent".into());
            }
        }
        // Resolve existing ancestors too; a symlink cannot redirect an audit write.
        let audit = confined_output(&base.join(&definition.manifest.audit.dir), &self.audit_root)?;
        if !audit.starts_with(&self.audit_root) {
            return Err("audit outside write grant".into());
        }
        Ok(())
    }
}

fn confined_output(path: &Path, root: &Path) -> Result<PathBuf, String> {
    let mut ancestor = path.to_owned();
    let mut suffix = Vec::new();
    while !ancestor.exists() {
        suffix.push(ancestor.file_name().ok_or("invalid audit path")?.to_owned());
        if !ancestor.pop() {
            return Err("invalid audit path".into());
        }
    }
    let mut actual = ancestor.canonicalize().map_err(|e| e.to_string())?;
    if !actual.starts_with(root) {
        return Err("audit outside write grant".into());
    }
    for component in suffix.into_iter().rev() {
        actual.push(component);
    }
    // Reject lexical traversal, including paths whose final component is missing.
    if path
        .components()
        .any(|p| matches!(p, std::path::Component::ParentDir))
    {
        return Err("audit traversal is invalid".into());
    }
    Ok(actual)
}

pub fn read_document(path: &Path) -> Result<String, String> {
    if !path.is_absolute() {
        return Err("manifest_path must be absolute".into());
    }
    // O_NONBLOCK prevents a caller-controlled FIFO from trapping the manager.
    let file = OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(path)
        .map_err(|e| e.to_string())?;
    if !file.metadata().map_err(|e| e.to_string())?.is_file() {
        return Err("manifest must be a regular file".into());
    }
    let mut bytes = Vec::new();
    file.take((DOCUMENT_BYTES + 1) as u64)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if bytes.len() > DOCUMENT_BYTES {
        return Err("manifest exceeds 1 MiB".into());
    }
    String::from_utf8(bytes).map_err(|e| e.to_string())
}

fn manifest_input(value: &Value, grants: &Grants) -> Result<Definition, String> {
    let object = value.as_object().ok_or("load requires object")?;
    let (text, base) = if let Some(path) = object.get("manifest_path").and_then(Value::as_str) {
        if object.len() != 1 {
            return Err("manifest_path is exclusive".into());
        }
        let path = PathBuf::from(path);
        let parent = path
            .parent()
            .ok_or("manifest parent missing")?
            .canonicalize()
            .map_err(|e| e.to_string())?;
        if !parent.starts_with(&grants.document_root) {
            return Err("manifest outside read grant".into());
        }
        (read_document(&path)?, parent)
    } else {
        if object.len() != 2 {
            return Err("manifest_toml and base_dir are required".into());
        }
        let text = object
            .get("manifest_toml")
            .and_then(Value::as_str)
            .ok_or("manifest_toml required")?;
        let base = PathBuf::from(
            object
                .get("base_dir")
                .and_then(Value::as_str)
                .ok_or("base_dir required")?,
        );
        if !base.is_absolute() || text.len() > DOCUMENT_BYTES {
            return Err("absolute base_dir and bounded manifest required".into());
        }
        (text.to_owned(), base)
    };
    let manifest = Manifest::from_toml_str(&text).map_err(|e| e.to_string())?;
    manifest.resolve().map_err(|e| e.to_string())?;
    let definition = Definition {
        manifest,
        base,
        source_sha256: format!("{:x}", Sha256::digest(text.as_bytes())),
    };
    grants.validate(&definition)?;
    Ok(definition)
}

fn no_fields(value: &Value) -> Result<(), Fault> {
    if value.as_object().is_some_and(|o| o.is_empty()) {
        Ok(())
    } else {
        Err(Fault::new("invalid_argument", "method takes no fields"))
    }
}
fn revision(value: &Value, state: &State) -> Result<(), Fault> {
    let expected = value
        .get("expected_revision")
        .and_then(Value::as_u64)
        .ok_or_else(|| Fault::new("invalid_argument", "expected_revision required"))?;
    if expected != state.generation {
        return Err(Fault::new("conflict", "configuration revision changed"));
    }
    Ok(())
}
fn configuration(manifest: &Manifest) -> Result<Value, String> {
    let value = Value::Object(
        manifest
            .plugins
            .iter()
            .map(|p| (p.name.clone(), serde_json::to_value(&p.config).unwrap()))
            .collect(),
    );
    if serde_json::to_vec(&value).map_err(|e| e.to_string())?.len() > CONFIGURATION_BYTES {
        return Err("published module configuration exceeds 64 KiB JSON allowance".into());
    }
    Ok(value)
}

// Native details have an explicit preview bound; lifecycle states, counters,
// revisions and the actual audit path remain intact. Full evidence is retained
// by the native audit owner, rather than making health unreadable.
fn bounded_native(mut value: Value) -> Value {
    fn visit(value: &mut Value, key: &str) -> bool {
        match value {
            Value::String(text) if key != "audit_dir" => {
                let mut bytes = 2;
                let mut end = 0;
                for character in text.chars() {
                    let cost = match character {
                        '"' | '\\' | '\n' | '\r' | '\t' | '\u{0008}' | '\u{000c}' => 2,
                        c if c < '\u{0020}' => 6,
                        c => c.len_utf8(),
                    };
                    if bytes + cost > 512 {
                        text.truncate(end);
                        return true;
                    }
                    bytes += cost;
                    end += character.len_utf8();
                }
                false
            }
            Value::Array(values) => values
                .iter_mut()
                .map(|v| visit(v, ""))
                .fold(false, |a, b| a || b),
            Value::Object(values) => values
                .iter_mut()
                .map(|(k, v)| visit(v, k))
                .fold(false, |a, b| a || b),
            _ => false,
        }
    }
    let truncated = visit(&mut value, "");
    if let Some(object) = value.as_object_mut() {
        object.insert("native_detail_truncated".into(), json!(truncated));
    }
    value
}
fn inactive(state: &State) -> Result<(), Fault> {
    if matches!(state.phase, "running" | "stopping" | "loading") || state.restart_required {
        Err(Fault::new(
            "conflict",
            "stop runtime before changing modules; abandoned native work requires process restart",
        ))
    } else {
        Ok(())
    }
}

fn build(
    definition: &Definition,
    grants: &Grants,
    boot: &str,
    revision: u64,
    attempt: u64,
    echo: bool,
    rpc: &xgc_rt_host::rpc::RpcBinding,
) -> Result<Host, String> {
    grants.validate(definition)?;
    let mut manifest = definition.manifest.clone();
    // A new invocation never truncates the previous invocation's run evidence.
    let run_root = confined_output(
        &definition.base.join(&manifest.audit.dir),
        &grants.audit_root,
    )?;
    manifest.audit.dir =
        run_root.join(format!("host-{boot}/revision-{revision}/attempt-{attempt}"));
    super::build_host(
        manifest,
        &definition.base,
        HostOptions {
            echo_health: echo,
            rpc: Some(rpc.clone()),
            ..HostOptions::default()
        },
    )
}

pub fn serve(
    socket: &Path,
    instance_id: String,
    initial: Option<PathBuf>,
    grants: Grants,
    policy: RuntimePolicy,
    echo: bool,
    process_stop: &AtomicBool,
) -> Result<bool, String> {
    let limits = Limits {
        discovery_routes: vec!["/v1/describe".into()],
        ..Limits::from_policy(&policy).map_err(|e| e.to_string())?
    };
    let shutdown_timeout = limits.shutdown_timeout;
    // A held observer consumes a call and a connection. Always leave room for
    // a separate management request, including when deployment lowers limits.
    let observer_capacity = 2
        .min(limits.in_flight.saturating_sub(1))
        .min(limits.connections.saturating_sub(1));
    let mut runtime = Runtime::new(RuntimeOptions {
        blocking_workers: CONTROL_CAPACITY,
        ..RuntimeOptions::from_policy(&policy).map_err(|e| e.to_string())?
    })
    .map_err(|e| e.to_string())?;
    let module_rpc = xgc_rt_host::rpc::RpcBinding::new(runtime.handle(), limits.clone())
        .map_err(|e| e.to_string())?;
    let service_ref = json!({"target_id":xgc2_xrpc::local_target_id().map_err(|e|e.to_string())?,
        "service":"sync-runtime","api_version":"v1","instance_id":instance_id,
        "profile":"http.v1","endpoint":{"kind":"unix","address":socket}});
    let state = Arc::new(Mutex::new(State {
        phase: "empty",
        generation: 0,
        applied_revision: None,
        modules: Vec::new(),
        configuration: json!({}),
        summary: Value::Null,
        last_error: None,
        health: None,
        restart_required: false,
        source_manifest_sha256: None,
        notifications: tokio::sync::watch::channel(0).0,
    }));
    let (commands, receiver) = mpsc::sync_channel::<Command>(CONTROL_CAPACITY);
    let manager_state = state.clone();
    let boot = instance_id.clone();
    let manager = thread::Builder::new().name("xgc-module-manager".into()).spawn(move || {
        let mut definition: Option<Definition> = None;
        let mut loaded: Option<Host> = None;
        let mut native = NativeRun { stop: Arc::new(AtomicBool::new(false)), running: None };
        let mut attempt = 0;
        while let Ok(command) = receiver.recv() {
            let Command::Call { route, value, deadline, reply } = command else { break };
            if tokio::time::Instant::now() >= deadline { let _ = reply.send(Err(Fault::new("deadline_exceeded", "expired before module admission"))); continue; }
            let mut result = (|| -> Result<Value, Fault> {
                match route.as_str() {
                    "/v1/load" => {
                        inactive(&manager_state.lock().unwrap())?;
                        if definition.is_some() { return Err(Fault::new("conflict", "unload existing module graph before load")); }
                        let next = manifest_input(&value, &grants).map_err(|e| Fault::new("invalid_argument", e))?;
                        let next_configuration=configuration(&next.manifest).map_err(|e|Fault::new("resource_exhausted",e))?;
                        let revision = manager_state.lock().unwrap().generation + 1;
                        if attempt>=MAX_LOAD_ATTEMPTS {return Err(Fault::new("resource_exhausted","host attempt allowance exhausted; evidence owner must retain this boot before a fresh process"));}
                        attempt += 1;
                        let host = build(&next, &grants, &boot, revision, attempt, echo, &module_rpc).map_err(|e| Fault::new("invalid_argument", e))?;
                        let mut state = manager_state.lock().unwrap();
                        state.generation = revision; state.applied_revision = None;
                        state.modules = next.manifest.plugins.iter().map(|p|p.name.clone()).collect();
                        state.configuration = next_configuration; state.summary = Value::Null;
                        state.source_manifest_sha256=Some(next.source_sha256.clone());
                        state.last_error = None; state.health = Some(host.health_observer()); state.phase = "loaded";
                        state.health.as_ref().unwrap().set_event_notifications(state.notifications.clone());
                        loaded = Some(host); definition = Some(next);
                        Ok(state.value())
                    }
                    "/v1/configure" => {
                        let mut state = manager_state.lock().unwrap();
                        inactive(&state)?; revision(&value, &state)?;
                        let fields = value.as_object().ok_or_else(|| Fault::new("invalid_argument", "object required"))?;
                        if fields.len() != 3 || !fields.contains_key("modules") || value["persist"] != false {
                            return Err(Fault::new("invalid_argument", "expected_revision, modules and persist:false required; durable save unsupported"));
                        }
                        let patches = value["modules"].as_object().filter(|p| !p.is_empty() && p.len() <= MAX_MODULES)
                            .ok_or_else(|| Fault::new("invalid_argument", "bounded nonempty module config map required"))?;
                        let current = definition.as_mut().ok_or_else(|| Fault::new("conflict", "load modules before configure"))?;
                        let mut candidate = current.manifest.clone();
                        for (name, config) in patches {
                            let module = candidate.plugins.iter_mut().find(|p| &p.name == name)
                                .ok_or_else(|| Fault::new("not_found", format!("unknown module {name}")))?;
                            module.config = serde_json::from_value(config.clone()).map_err(|_| Fault::new("invalid_argument", "module configuration must be a TOML-compatible object"))?;
                        }
                        candidate.resolve().map_err(|e| Fault::new("invalid_argument", e.to_string()))?;
                        let next_configuration=configuration(&candidate).map_err(|e|Fault::new("resource_exhausted",e))?;
                        // Graph configuration is desired only. Native validation and application
                        // run on each module's existing C ABI thread during start.
                        let previous = loaded.take(); state.health = None; state.generation += 1;
                        state.source_manifest_sha256=None;
                        current.manifest = candidate; state.configuration = next_configuration;
                        state.phase = "loading"; state.last_error = None; state.summary = Value::Null;
                        drop(state);
                        // Module/transport destructors and audit writer joins are
                        // native manager work, never performed under the query lock.
                        drop(previous);
                        let mut state=manager_state.lock().unwrap();state.phase="configured";
                        Ok(state.configuration())
                    }
                    "/v1/unload" => {
                        let mut state = manager_state.lock().unwrap(); inactive(&state)?; revision(&value, &state)?;
                        if value.as_object().map_or(true,|o|o.len()!=1) { return Err(Fault::new("invalid_argument", "unload requires expected_revision only")); }
                        state.phase="loading"; state.health=None;
                        let previous=loaded.take();drop(state);
                        native.join()?;drop(previous);definition=None;
                        let mut state=manager_state.lock().unwrap();state.modules.clear();
                        state.configuration = json!({}); state.generation += 1; state.phase = "empty";
                        state.applied_revision=None;
                        state.source_manifest_sha256=None;
                        Ok(state.value())
                    }
                    "/v1/start" => {
                        no_fields(&value)?; inactive(&manager_state.lock().unwrap())?;
                        native.join()?;
                        if loaded.is_none() {
                            let current = definition.as_ref().ok_or_else(|| Fault::new("conflict", "load modules before start"))?;
                            if attempt>=MAX_LOAD_ATTEMPTS {return Err(Fault::new("resource_exhausted","host attempt allowance exhausted"));}
                            attempt += 1;
                            let generation = manager_state.lock().unwrap().generation;
                            let host = build(current, &grants, &boot, generation, attempt, echo, &module_rpc)
                                .map_err(|e| Fault::new("invalid_argument", e))?;
                            let mut state=manager_state.lock().unwrap();state.health = Some(host.health_observer());
                            state.health.as_ref().unwrap().set_event_notifications(state.notifications.clone());loaded = Some(host);
                        }
                        let host = loaded.take().unwrap(); native.stop.store(false, Ordering::Relaxed);
                        let state = manager_state.clone(); let stop = native.stop.clone();
                        let mut locked = manager_state.lock().unwrap();
                        native.running = Some(thread::Builder::new().name("xgc-runtime-supervisor".into()).spawn(move || {
                            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(||host.run(&stop)));
                            let result = result.unwrap_or_else(|_|Err(xgc_rt_host::host::HostError("native supervisor panicked".into())));
                            let mut state = state.lock().unwrap();
                            if state.health.as_ref().and_then(|h|h.configuration_snapshot()).is_some_and(|v|v["applied"]==true) { state.applied_revision = Some(state.generation); }
                            match result {
                                Ok(summary) => {
                                    state.restart_required = summary.plugins.iter().any(|p|p.abandons > 0);
                                    let clean = summary.aborted.is_none() && !state.restart_required && summary.audit.complete() && summary.plugins.iter().all(|p|p.state!="error" && p.last_error.is_none());
                                    state.last_error = (!clean).then(||"module execution failed; inspect native summary".into());
                                    state.summary = bounded_native(serde_json::to_value(summary).unwrap_or(Value::Null)); state.phase = if clean {"stopped"} else {"error"};
                                }
                                Err(error) => { state.phase="error"; state.last_error=Some(error.to_string()); state.restart_required=true; }
                            }
                            state.notifications.send_modify(|revision|*revision+=1);
                        }).map_err(|e| Fault::new("unavailable", e.to_string()))?);
                        locked.phase="running"; locked.applied_revision=None;
                        locked.summary=Value::Null; locked.last_error=None;
                        Ok(json!({"status":"accepted","generation":locked.generation}))
                    }
                    "/v1/stop" => {
                        no_fields(&value)?; let mut state=manager_state.lock().unwrap();
                        if state.restart_required {return Err(Fault::new("conflict","native work is not quiescent; process restart required"));}
                        if state.phase=="running" { state.phase="stopping"; native.stop.store(true,Ordering::Relaxed); }
                        Ok(json!({"status":if state.phase=="stopping" {"accepted"} else {"completed"},"state":state.phase}))
                    }
                    _ => Err(Fault::new("not_found", "unknown runtime route")),
                }
            })();
            if let Ok(value)=result.as_mut() {
                let state=manager_state.lock().unwrap();
                state.notifications.send_modify(|revision|*revision+=1);
                if let Some(object)=value.as_object_mut(){object.insert("event_revision".into(),json!(*state.notifications.borrow()));}
            }
            let _ = reply.send(result);
        }
        native.stop.store(true,Ordering::Relaxed);
        let _=native.join();
        // Actual module and manager work finish before endpoint lease release.
        drop(loaded);
    }).map_err(|e|e.to_string())?;
    let handler_commands = commands.clone();
    let handler_state = state.clone();
    let reference = service_ref.clone();
    let effective_policy = policy.effective();
    let resources = runtime.handle();
    let observers = Arc::new(tokio::sync::Semaphore::new(observer_capacity));
    let rpc = xgc2_xrpc::Host::bind(
        &runtime,
        socket,
        instance_id,
        limits,
        true,
        handler(move |context, route, value| {
            let commands = handler_commands.clone();
            let state = handler_state.clone();
            let reference = reference.clone();
            let effective_policy = effective_policy.clone();
            let observers = observers.clone();
            let resources = resources.clone();
            async move {
                if context.method == Method::GET {
                    if let Some(raw) = route.strip_prefix("/v1/observe/") {
                        let after = raw
                            .parse::<u64>()
                            .ok()
                            .filter(|v| v.to_string() == raw)
                            .ok_or_else(|| {
                                Fault::new("invalid_argument", "canonical event revision required")
                            })?;
                        let _slot = observers.try_acquire_owned().map_err(|_| {
                            Fault::new(
                                "resource_exhausted",
                                "held observation allowance exhausted; management capacity reserved",
                            )
                        })?;
                        let mut events = state.lock().unwrap().notifications.subscribe();
                        loop {
                            let current = *events.borrow_and_update();
                            if current > after {
                                return Ok(state.lock().unwrap().value());
                            }
                            if current < after {
                                return Err(Fault::new(
                                    "conflict",
                                    "event revision belongs to another instance or future event",
                                ));
                            }
                            events.changed().await.map_err(|_| {
                                Fault::new("unavailable", "runtime observer closed")
                            })?;
                        }
                    }
                    return match route.as_str() {
                        "/v1/describe" => Ok(
                            json!({"service_ref":reference,"modules_limit":MAX_MODULES,"control_capacity":CONTROL_CAPACITY,"load_attempts_limit":MAX_LOAD_ATTEMPTS,"held_observers_limit":observer_capacity,"configuration_schema":"sync-runtime-modules/1"}),
                        ),
                        "/v1/health" | "/v1/modules" => {
                            let mut value = state.lock().unwrap().value();
                            value["rpc_resources"] =
                                serde_json::to_value(resources.stats()).unwrap();
                            Ok(value)
                        }
                        "/v1/configuration" => Ok(state.lock().unwrap().configuration()),
                        "/v1/policy" => Ok(serde_json::to_value(effective_policy).unwrap()),
                        _ => Err(Fault::new("not_found", "unknown runtime query")),
                    };
                }
                if context.method != Method::POST {
                    return Err(Fault::new("not_found", "runtime mutations require POST"));
                }
                let deadline = context.deadline;
                // This SDK-owned closure holds the endpoint lease after caller cancellation,
                // until the admitted manager operation actually completes.
                context
                    .blocking(move || {
                        let (reply, result) = oneshot::channel();
                        commands
                            .try_send(Command::Call {
                                route,
                                value,
                                deadline,
                                reply,
                            })
                            .map_err(|error| match error {
                                mpsc::TrySendError::Full(_) => {
                                    Fault::new("resource_exhausted", "runtime control queue full")
                                }
                                mpsc::TrySendError::Disconnected(_) => Fault::new(
                                    "unavailable",
                                    "native manager stopped; process restart required",
                                ),
                            })?;
                        result
                            .blocking_recv()
                            .map_err(|_| Fault::new("unavailable", "runtime manager stopped"))?
                    })
                    .await?
            }
        }),
    );
    let mut rpc = match rpc {
        Ok(rpc) => rpc,
        Err(error) => {
            let _ = commands.send(Command::Shutdown);
            let _ = manager.join();
            return Err(error.to_string());
        }
    };
    println!("{}", json!({"service_ref":service_ref}));
    let execution = (|| -> Result<(), String> {
        if let Some(path) = initial {
            for (route, value) in [
                ("/v1/load", json!({"manifest_path":path})),
                ("/v1/start", json!({})),
            ] {
                let (reply, result) = oneshot::channel();
                commands
                    .send(Command::Call {
                        route: route.into(),
                        value,
                        deadline: tokio::time::Instant::now() + Duration::from_secs(30),
                        reply,
                    })
                    .map_err(|e| e.to_string())?;
                result
                    .blocking_recv()
                    .map_err(|e| e.to_string())?
                    .map_err(|e| e.to_string())?;
            }
        }
        while !process_stop.load(Ordering::Relaxed) {
            if manager.is_finished() {
                return Err("module manager ended unexpectedly".into());
            }
            thread::sleep(Duration::from_millis(50));
        }
        Ok(())
    })();
    let _ = commands.send(Command::Shutdown);
    let joined = manager.join();
    let retain_ownership = joined.is_err()
        || state
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .restart_required;
    if retain_ownership {
        {
            let mut state = state.lock().unwrap_or_else(|error| error.into_inner());
            state.phase = "error";
            state.restart_required = true;
            if joined.is_err() {
                state.last_error =
                    Some("manager join failed; native quiescence is unproven".into());
            }
        }
        // A supervisor receipt can report an abandoned executor while that
        // native executor is still doing work. SDK callback drain cannot prove
        // that executor stopped. Keep the listener, lease and process Runtime
        // until the process owner actually exits, including if stdout blocks.
        // This is one terminal fault per boot: restart/unload is already fenced.
        std::mem::forget(rpc);
        std::mem::forget(runtime);
        let state = state.lock().unwrap_or_else(|error| error.into_inner());
        if !state.summary.is_null() {
            println!("{}", state.summary);
        }
        eprintln!(
            "xgc-rt-host: native work is not quiescent; RPC ownership retained until process exit"
        );
        return Ok(false);
    }
    rpc.close().map_err(|e| e.to_string())?;
    runtime.close(shutdown_timeout).map_err(|e| e.to_string())?;
    execution?;
    joined.map_err(|_| "module manager panicked".to_owned())?;
    let state = state.lock().unwrap();
    if !state.summary.is_null() {
        println!("{}", state.summary);
    }
    Ok(state.last_error.is_none() && !state.restart_required)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn nonregular_manifest_is_rejected_without_reading() {
        let path = std::env::temp_dir().join(format!("sol7-fifo-{}", std::process::id()));
        let name = std::ffi::CString::new(path.to_str().unwrap()).unwrap();
        assert_eq!(unsafe { libc::mkfifo(name.as_ptr(), 0o600) }, 0);
        assert!(read_document(&path).unwrap_err().contains("regular"));
        std::fs::remove_file(path).unwrap();
    }
    #[test]
    fn configuration_reports_native_evidence_and_retains_previous_revision() {
        let state = State {
            phase: "configured",
            generation: 2,
            applied_revision: Some(1),
            modules: vec![],
            configuration: json!({}),
            summary: Value::Null,
            last_error: None,
            health: None,
            restart_required: false,
            source_manifest_sha256: None,
            notifications: tokio::sync::watch::channel(0).0,
        };
        assert_eq!(state.configuration()["desired_revision"], 2);
        assert_eq!(state.configuration()["applied_revision"], 1);
        assert!(revision(&json!({"expected_revision":1}), &state).is_err());
        assert!(state.configuration()["persisted_revision"].is_null());
    }
}
