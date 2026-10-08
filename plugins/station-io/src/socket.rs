//! Domain control adapter. HTTP admission and blocking callbacks belong to the
//! injected process XRPC owner. The RT step only takes and completes a fixed record.
use crate::wire::{self, COMMAND_LEN, TIMELINE_LEN};
use serde_json::{json, Value};
use std::{
    path::Path,
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::sync::{mpsc, oneshot};
use xgc2_xrpc::{
    BlockingClient, Fault, Limits, Method, PolicyOptions, Runtime, RuntimeOptions, RuntimePolicy,
};

use xgc2_xrpc::ffi::{BindOptions, ForeignHost, ForeignRequest, ForeignRuntime};

pub const CAPACITY: usize = 16;
pub enum Request {
    Command([u8; COMMAND_LEN]),
    Mission([u8; TIMELINE_LEN]),
}
pub struct Pending {
    pub request: Request,
    pub deadline: tokio::time::Instant,
    reply: oneshot::Sender<Result<(), &'static str>>,
}
impl Pending {
    pub fn expired(&self) -> bool {
        self.reply.is_closed() || tokio::time::Instant::now() >= self.deadline
    }
    /// Called strictly after local publish has returned; this is not execution completion.
    pub fn complete(self, result: Result<(), &'static str>) {
        let _ = self.reply.send(result);
    }
}
struct Admission {
    open: bool,
    sender: mpsc::Sender<Pending>,
}

pub struct Control {
    host: Option<ForeignHost>,
    admission: Arc<Mutex<Admission>>,
    pending: mpsc::Receiver<Pending>,
    // release consumes the official foreign handle even on error. Keep that
    // error latched rather than retry an already consumed pointer or claim a
    // later deactivate succeeded. Stop/close errors retain the actual handle.
    release_error: Option<String>,
}
impl Control {
    pub fn bind(
        runtime: &ForeignRuntime,
        path: &Path,
        instance_id: &str,
        authority: bool,
        command: bool,
        mission: bool,
    ) -> Result<Self, String> {
        let mut caps = runtime.baseline_caps();
        caps.max_connections = caps.max_connections.min((CAPACITY + 2) as u64);
        caps.max_in_flight = caps.max_in_flight.min((CAPACITY + 2) as u64);
        caps.max_request_bytes = caps.max_request_bytes.min(4096);
        caps.max_response_bytes = caps.max_response_bytes.min(4096);
        caps.call_timeout_ms = caps.call_timeout_ms.min(2000);
        caps.shutdown_timeout_ms = caps.shutdown_timeout_ms.min(2000);
        let (sender, pending) = mpsc::channel(CAPACITY);
        let admission = Arc::new(Mutex::new(Admission { open: true, sender }));
        let callback_admission = Arc::clone(&admission);
        let service_ref = json!({
            "target_id": xgc2_xrpc::local_target_id().map_err(|e| e.to_string())?,
            "service": "station-io", "api_version": "v1",
            "instance_id": instance_id, "profile": "http.v1",
            "endpoint": {"kind": "unix", "address": path},
        });
        let host = runtime
            .bind_http(
                path,
                instance_id,
                BindOptions {
                    caps: Some(caps),
                    discovery_routes: vec!["/v1/describe".into()],
                    reclaim_unreachable: true,
                },
                move |request, output| {
                    control_request(
                        &callback_admission,
                        &service_ref,
                        authority,
                        command,
                        mission,
                        request,
                        output,
                    )
                },
            )
            .map_err(|e| e.to_string())?;
        Ok(Self {
            host: Some(host),
            admission,
            pending,
            release_error: None,
        })
    }
    /// At most one fixed-size handoff per step, no wait and no parser/network IO.
    pub fn poll(&mut self) -> Option<Pending> {
        self.pending.try_recv().ok()
    }
    pub fn close(&mut self) -> Result<(), String> {
        if let Some(error) = &self.release_error {
            return Err(error.clone());
        }
        let Some(host) = self.host.as_mut() else {
            return Ok(());
        };
        // Fence native admission first. Even a failed stop must fence our queue
        // and release queued receipt waiters, while retaining the foreign host.
        let stopped = host.stop().map_err(|e| e.to_string());
        {
            let mut gate = self
                .admission
                .lock()
                .map_err(|_| "station admission poisoned")?;
            gate.open = false;
            self.pending.close();
            while let Ok(pending) = self.pending.try_recv() {
                pending.complete(Err("station stopped before publication"));
            }
        }
        stopped?;
        host.close().map_err(|e| e.to_string())?;
        // Successful close proves actual callbacks have returned. The official
        // release completes userdata release synchronously before native unload.
        let result = self
            .host
            .take()
            .unwrap()
            .release()
            .map_err(|e| e.to_string());
        if let Err(error) = &result {
            self.release_error = Some(format!("station foreign host release failed: {error}"));
        }
        result
    }
}
impl Drop for Control {
    fn drop(&mut self) {
        let _ = self.close();
        // If native close failed, ForeignHost::drop retains live callback
        // userdata and the origin's actual module code pin through completion.
    }
}

fn control_request(
    admission: &Mutex<Admission>,
    service_ref: &Value,
    authority: bool,
    command: bool,
    mission: bool,
    context: ForeignRequest<'_>,
    output: &mut [u8],
) -> Result<usize, Fault> {
    // Copy the original absolute deadline now. No borrowed ForeignRequest or
    // request-body pointer is ever placed in the RT queue.
    let deadline = context.deadline()?;
    let value = {
        let gate = admission
            .lock()
            .map_err(|_| Fault::new("internal", "station admission poisoned"))?;
        if !gate.open {
            return Err(Fault::new("unavailable", "station control stopped"));
        }
        if context.path == "/v1/describe" && context.method == "GET" {
            Some(json!({"service_ref": service_ref}))
        } else if context.path == "/v1/health" && context.method == "GET" {
            Some(json!({"state":"active","authority":authority,
                "capabilities":{"command":command,"mission":mission},
                "queue_capacity":CAPACITY,"queue_available":gate.sender.capacity(),
                "completion":"local-publication-only","persistence":"ephemeral"}))
        } else {
            None
        }
    };
    if let Some(value) = value {
        return write_response(&value, output);
    }
    if context.method != "POST" {
        return Err(Fault::new("not_found", "station mutations require POST"));
    }
    if !authority {
        return Err(Fault::new("conflict", "not the frozen authority"));
    }
    let value: Value = serde_json::from_slice(context.body)
        .map_err(|_| Fault::new("invalid_argument", "invalid station JSON"))?;
    let object = value
        .as_object()
        .ok_or_else(|| Fault::new("invalid_argument", "object required"))?;
    let request = match context.path {
        "/v1/command" => {
            if !command {
                return Err(Fault::new(
                    "conflict",
                    "command capability is not configured",
                ));
            }
            if object.len() != 1 {
                return Err(Fault::new(
                    "invalid_argument",
                    "token is the only command field",
                ));
            }
            let token = object
                .get("token")
                .and_then(Value::as_str)
                .filter(|s| wire::valid_command_token(s))
                .ok_or_else(|| {
                    Fault::new(
                        "invalid_argument",
                        "command token is not a controller string",
                    )
                })?;
            Request::Command(wire::command_payload(token))
        }
        "/v1/mission" => {
            if !mission {
                return Err(Fault::new(
                    "conflict",
                    "mission capability is not configured",
                ));
            }
            if object.len() != 1 {
                return Err(Fault::new(
                    "invalid_argument",
                    "timeline is the only mission field",
                ));
            }
            let values = object
                .get("timeline")
                .and_then(Value::as_array)
                .filter(|v| v.len() == TIMELINE_LEN)
                .ok_or_else(|| Fault::new("invalid_argument", "timeline must be 240 bytes"))?;
            let mut bytes = [0; TIMELINE_LEN];
            for (out, value) in bytes.iter_mut().zip(values) {
                *out = value
                    .as_u64()
                    .filter(|n| *n <= 255)
                    .ok_or_else(|| Fault::new("invalid_argument", "timeline byte invalid"))?
                    as u8;
            }
            if !wire::timeline_schema_ok(&bytes) {
                return Err(Fault::new("invalid_argument", "timeline schema must be 1"));
            }
            Request::Mission(bytes)
        }
        _ => return Err(Fault::new("not_found", "unknown station route")),
    };
    let (reply, received) = oneshot::channel();
    {
        let gate = admission
            .lock()
            .map_err(|_| Fault::new("internal", "station admission poisoned"))?;
        if !gate.open {
            return Err(Fault::new("unavailable", "station control stopped"));
        }
        if tokio::time::Instant::now() >= deadline {
            return Err(Fault::new(
                "deadline_exceeded",
                "station deadline before handoff",
            ));
        }
        gate.sender
            .try_send(Pending {
                request,
                deadline,
                reply,
            })
            .map_err(|error| match error {
                mpsc::error::TrySendError::Full(_) => {
                    Fault::new("resource_exhausted", "station command queue full")
                }
                mpsc::error::TrySendError::Closed(_) => {
                    Fault::new("unavailable", "station control stopped")
                }
            })?;
    }
    // This callback runs on the origin SDK's bounded blocking pool. Keep its
    // native admission, endpoint lease and code pin until the RT owner actually
    // completes or drains the handoff, even after the HTTP caller times out.
    // ForeignRequest provides an absolute deadline, not a caller-abort token.
    match received.blocking_recv() {
        Ok(Ok(())) => write_response(&json!({"status":"queued"}), output),
        Ok(Err(error)) => Err(Fault::new("unavailable", error)),
        Err(_) => Err(Fault::new(
            "unavailable",
            "station stopped before publication",
        )),
    }
}

fn write_response(value: &Value, output: &mut [u8]) -> Result<usize, Fault> {
    let mut cursor = std::io::Cursor::new(output);
    serde_json::to_writer(&mut cursor, value).map_err(|_| {
        Fault::new(
            "resource_exhausted",
            "station response exceeds bounded output",
        )
    })?;
    Ok(cursor.position() as usize)
}
pub fn command(path: &Path, instance_id: &str, token: &str) -> Result<(), String> {
    transact(path, instance_id, "/v1/command", json!({"token":token}))
}
pub fn mission(path: &Path, instance_id: &str, bytes: &[u8]) -> Result<(), String> {
    transact(path, instance_id, "/v1/mission", json!({"timeline":bytes}))
}

fn cli_policy(
    environment: impl IntoIterator<Item = (std::ffi::OsString, std::ffi::OsString)>,
) -> Result<RuntimePolicy, String> {
    // The CLI is one transaction in its own process. Its caller supplies one
    // startup snapshot; module hosts instead consume the aggregate's owner.
    let mut options = PolicyOptions {
        capabilities: ["http", "rpc", "transport", "client_pool", "client_registry"]
            .into_iter()
            .map(str::to_owned)
            .collect(),
        ..PolicyOptions::default()
    };
    let bounds = [
        ("MAX_HEADER_BYTES", 8192),
        ("MAX_REQUEST_BYTES", 4096),
        ("MAX_RESPONSE_BYTES", 4096),
        ("CALL_TIMEOUT_MS", 2000),
        ("IDLE_TIMEOUT_MS", 2000),
        ("CLIENT_MAX_CONNECTIONS", 1),
        ("CLIENT_MAX_REFERENCES", 1),
        ("CLIENT_REFERENCE_IDLE_TIMEOUT_MS", 2000),
    ];
    for (name, bound) in bounds {
        options.explicit.insert(
            name.into(),
            xgc2_xrpc::policy::PolicyOverride::integer(bound, "station-io-cli"),
        );
        options.ceilings.insert(name.into(), bound);
    }
    let policy = RuntimePolicy::resolve_os(environment, options).map_err(|e| e.to_string())?;
    // HTTP client IO has no independently enforced server header timer.
    // check_applied rejects it even though header byte limits use HTTP policy.
    policy
        .check_applied(bounds.into_iter().map(|(name, _)| name))
        .map_err(|e| e.to_string())?;
    Ok(policy)
}

fn transact(path: &Path, instance_id: &str, route: &str, value: Value) -> Result<(), String> {
    // No native owner or endpoint is created before startup policy admission.
    let policy = cli_policy(std::env::vars_os())?;
    transact_with_policy(path, instance_id, route, value, policy)
}

fn transact_with_policy(
    path: &Path,
    instance_id: &str,
    route: &str,
    value: Value,
    policy: RuntimePolicy,
) -> Result<(), String> {
    let limits = Limits::from_policy(&policy).map_err(|e| e.to_string())?;
    let deadline = std::time::Instant::now() + limits.call_timeout;
    let remaining = || {
        deadline
            .checked_duration_since(std::time::Instant::now())
            .filter(|budget| !budget.is_zero())
            .ok_or_else(|| "NotSent: station CLI transaction deadline exceeded".to_owned())
    };
    let mut runtime = Runtime::new(RuntimeOptions {
        // This process admits one outgoing transaction and one native session;
        // host-only policy knobs are rejected rather than reported as applied.
        max_connections: 1,
        max_calls: 1,
        blocking_workers: 1,
        ..RuntimeOptions::from_policy(&policy).map_err(|e| e.to_string())?
    })
    .map_err(|e| e.to_string())?;
    let result: Result<(), String> = (|| {
        let mut client =
            BlockingClient::unix_with_limits(&runtime, path, instance_id, limits.clone())
                .map_err(|e| e.to_string())?;
        if instance_id.is_empty() {
            let reference = client
                .request(Method::GET, "/v1/describe", None, remaining()?, None)
                .map_err(|e| e.to_string())?;
            let id = reference["service_ref"]["instance_id"]
                .as_str()
                .filter(|s| !s.is_empty())
                .ok_or("station discovery omitted instance")?;
            client = BlockingClient::unix_with_limits(&runtime, path, id, limits.clone())
                .map_err(|e| e.to_string())?;
        }
        let result = client
            .call(route, value, remaining()?)
            .map_err(|e| e.to_string())?;
        if result.get("status").and_then(Value::as_str) != Some("queued") {
            return Err("outcome unknown: invalid publication receipt".into());
        }
        Ok(())
    })();
    // Client handles are gone before the explicit owner is joined. A mutation's
    // original transport disposition is preserved if shutdown also fails.
    let closed = runtime
        .close(Duration::from_secs(2))
        .map_err(|e| e.to_string());
    result?;
    closed
        .map_err(|e| format!("locally queued; CLI runtime shutdown failed: {e}; do not replay"))?;
    println!("queued");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{sync::mpsc as stdmpsc, thread, time::Instant};
    use xgc2_xrpc::{handler, Host};

    struct TestOwner {
        runtime: Runtime,
        _export: xgc2_xrpc::ffi::RuntimeExport,
        foreign: ForeignRuntime,
    }

    impl TestOwner {
        fn new(options: RuntimeOptions, limits: Limits) -> Self {
            let runtime = Runtime::new(options).unwrap();
            // Unit tests are linked into this executable. Actual separately
            // linked module/library pins are exercised by shared_rpc.rs.
            let export =
                xgc2_xrpc::ffi::RuntimeExport::new(runtime.handle(), limits, Arc::new(())).unwrap();
            let foreign = unsafe { ForeignRuntime::from_api(export.api()) }.unwrap();
            Self {
                runtime,
                _export: export,
                foreign,
            }
        }
        fn default() -> Self {
            Self::new(RuntimeOptions::default(), Limits::default())
        }
    }
    #[test]
    fn receipt_waits_for_publication_and_expired_call_is_not_published() {
        let directory = tempfile::Builder::new()
            .permissions(std::os::unix::fs::PermissionsExt::from_mode(0o700))
            .tempdir()
            .unwrap();
        let path = directory.path().join("station.sock");
        let owner = TestOwner::default();
        let mut control =
            Control::bind(&owner.foreign, &path, "epoch-1", true, true, true).unwrap();
        let clientpath = path.clone();
        let (sent, received) = stdmpsc::channel();
        let worker = thread::spawn(move || {
            let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
            let client = BlockingClient::unix(&runtime, clientpath, "epoch-1").unwrap();
            sent.send(client.call(
                "/v1/command",
                json!({"token":"hold"}),
                Duration::from_secs(1),
            ))
            .unwrap();
        });
        let deadline = Instant::now() + Duration::from_secs(1);
        let pending = loop {
            if let Some(pending) = control.poll() {
                break pending;
            }
            assert!(Instant::now() < deadline);
            thread::yield_now();
        };
        assert!(
            received.try_recv().is_err(),
            "queued must not precede publish"
        );
        assert!(matches!(&pending.request,Request::Command(bytes) if &bytes[..4]==b"hold"));
        pending.complete(Ok(()));
        assert_eq!(received.recv().unwrap().unwrap()["status"], "queued");
        worker.join().unwrap();
        let mut runtime = Runtime::new(RuntimeOptions::default()).unwrap();
        let client = xgc2_xrpc::Client::unix(&runtime.handle(), &path, "epoch-1").unwrap();
        let foreign = tokio::runtime::Builder::new_current_thread()
            .enable_all()
            .build()
            .unwrap();
        foreign.block_on(async {
            let call = tokio::spawn(async move {
                client
                    .call(
                        "/v1/command",
                        json!({"token":"stop"}),
                        Duration::from_secs(2),
                    )
                    .await
            });
            let admitted_by = Instant::now() + Duration::from_secs(1);
            while control.pending.len() != 1 {
                assert!(
                    Instant::now() < admitted_by,
                    "request never entered handoff queue"
                );
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            assert_eq!(control.pending.capacity(), CAPACITY - 1);
            // Cancel after actual handoff admission, not a short timer that
            // might expire before Runtime setup or native connection dispatch.
            call.abort();
            assert!(call.await.unwrap_err().is_cancelled());
            // Foreign callbacks keep the queue record and origin admission
            // until actual receipt. Caller abort is not a foreign cancel token;
            // publication is rejected at its copied absolute deadline.
            assert_eq!(control.pending.len(), 1);
            assert_eq!(control.pending.capacity(), CAPACITY - 1);
            let pending = control.poll().unwrap();
            let cancelled_by = Instant::now() + Duration::from_secs(3);
            while !pending.expired() {
                assert!(
                    Instant::now() < cancelled_by,
                    "cancelled request remained publishable"
                );
                tokio::time::sleep(Duration::from_millis(1)).await;
            }
            assert!(matches!(&pending.request, Request::Command(bytes) if &bytes[..4] == b"stop"));
            // Same gate as StationIo::step: expired handoffs are discarded.
            assert!(pending.expired());
            drop(pending);
            assert!(control.poll().is_none());
            assert_eq!(control.pending.capacity(), CAPACITY);
        });
        drop(foreign);
        runtime.close(Duration::from_secs(2)).unwrap();
    }

    #[test]
    fn cli_policy_applies_client_bounds_and_rejects_unenforced_settings() {
        let policy = cli_policy(std::iter::empty()).unwrap();
        let limits = Limits::from_policy(&policy).unwrap();
        let runtime = RuntimeOptions::from_policy(&policy).unwrap();
        assert_eq!(limits.header_bytes, 8192);
        assert_eq!(limits.body_bytes, 4096);
        assert_eq!(limits.response_bytes, 4096);
        assert_eq!(limits.call_timeout, Duration::from_secs(2));
        assert_eq!(limits.client_connections, 1);
        assert_eq!(runtime.max_sessions, 1);
        assert_eq!(
            policy.fields()["CALL_TIMEOUT_MS"].source,
            xgc2_xrpc::policy::PolicySource::Deployment
        );
        let policy = cli_policy([("XGC2_XRPC_CALL_TIMEOUT_MS".into(), "100".into())]).unwrap();
        assert_eq!(
            Limits::from_policy(&policy).unwrap().call_timeout,
            Duration::from_millis(100)
        );
        assert_eq!(
            policy.fields()["CALL_TIMEOUT_MS"].source,
            xgc2_xrpc::policy::PolicySource::Environment
        );
        for (name, value) in [
            ("MAX_REQUEST_BYTES", "4097"),
            ("MAX_RESPONSE_BYTES", "4097"),
            ("MAX_HEADER_BYTES", "8193"),
            ("CALL_TIMEOUT_MS", "2001"),
            ("IDLE_TIMEOUT_MS", "2001"),
            ("CLIENT_MAX_CONNECTIONS", "2"),
            ("CLIENT_MAX_REFERENCES", "2"),
            ("CLIENT_REFERENCE_IDLE_TIMEOUT_MS", "2001"),
            ("CALL_TIMEOUT_MS", "0100"),
            ("HEADER_TIMEOUT_MS", "100"),
            ("HOST_MAX_CONNECTIONS", "1"),
            ("SHUTDOWN_TIMEOUT_MS", "100"),
            ("GRPC_MAX_STREAMS_PER_CONNECTION", "1"),
            ("LOG_LEVEL", "debug"),
            ("UNKNOWN", "1"),
        ] {
            let error =
                cli_policy([(format!("XGC2_XRPC_{name}").into(), value.into())]).unwrap_err();
            assert!(error.contains(name), "{name}: {error}");
        }
    }

    #[test]
    fn cli_discovers_fresh_instance_reuses_session_and_applies_response_limit() {
        use std::sync::{
            atomic::{AtomicUsize, Ordering},
            Arc,
        };
        let directory = tempfile::Builder::new()
            .permissions(std::os::unix::fs::PermissionsExt::from_mode(0o700))
            .tempdir()
            .unwrap();
        let path = directory.path().join("station.sock");
        let mut runtime = Runtime::new(RuntimeOptions::default()).unwrap();
        let mutations = Arc::new(AtomicUsize::new(0));
        let applied = mutations.clone();
        let mut host = Host::bind(
            &runtime,
            &path,
            "fresh-station".into(),
            Limits {
                discovery_routes: vec!["/v1/describe".into()],
                ..Limits::default()
            },
            false,
            handler(move |context, path, value| {
                let applied = applied.clone();
                async move {
                    if context.method == Method::GET && path == "/v1/describe" {
                        return Ok(json!({"service_ref":{"instance_id":"fresh-station"}}));
                    }
                    assert_eq!(context.method, Method::POST);
                    assert_eq!(path, "/v1/command");
                    applied.fetch_add(1, Ordering::SeqCst);
                    if value["token"] == "oversized-receipt" {
                        Ok(json!({"status":"queued","padding":"x".repeat(512)}))
                    } else {
                        Ok(json!({"status":"queued"}))
                    }
                }
            }),
        )
        .unwrap();
        transact_with_policy(
            &path,
            "",
            "/v1/command",
            json!({"token":"hold"}),
            cli_policy(std::iter::empty()).unwrap(),
        )
        .unwrap();
        assert_eq!(mutations.load(Ordering::SeqCst), 1);
        // Discovery and fenced mutation share the one admitted native session.
        assert_eq!(host.stats.accepted.load(Ordering::Relaxed), 1);
        let policy = cli_policy([("XGC2_XRPC_MAX_RESPONSE_BYTES".into(), "256".into())]).unwrap();
        let error = transact_with_policy(
            &path,
            "",
            "/v1/command",
            json!({"token":"oversized-receipt"}),
            policy,
        )
        .unwrap_err();
        assert!(error.contains("OutcomeUnknown"), "{error}");
        assert_eq!(
            mutations.load(Ordering::SeqCst),
            2,
            "lost receipt must not replay mutation"
        );
        assert_eq!(host.stats.accepted.load(Ordering::Relaxed), 2);
        host.close().unwrap();
        runtime.close(Duration::from_secs(2)).unwrap();
    }
    #[test]
    fn pending_record_is_bounded_and_authority_checked_before_handoff() {
        assert!(std::mem::size_of::<Pending>() < 320);
        let directory = tempfile::Builder::new()
            .permissions(std::os::unix::fs::PermissionsExt::from_mode(0o700))
            .tempdir()
            .unwrap();
        let path = directory.path().join("station.sock");
        let owner = TestOwner::default();
        let mut control =
            Control::bind(&owner.foreign, &path, "epoch-1", false, false, false).unwrap();
        let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
        let client = BlockingClient::unix(&runtime, path, "epoch-1").unwrap();
        assert!(client
            .call(
                "/v1/command",
                json!({"token":"hold"}),
                Duration::from_secs(1)
            )
            .is_err());
        assert!(control.poll().is_none());
        assert_eq!(
            client
                .request(
                    Method::GET,
                    "/v1/health",
                    None,
                    Duration::from_secs(1),
                    None
                )
                .unwrap()["queue_capacity"],
            16
        );
    }

    #[test]
    fn timed_out_callback_keeps_owner_until_actual_pending_receipt() {
        let directory = tempfile::Builder::new()
            .permissions(std::os::unix::fs::PermissionsExt::from_mode(0o700))
            .tempdir()
            .unwrap();
        let path = directory.path().join("station.sock");
        let owner = TestOwner::new(
            RuntimeOptions {
                max_connections: 2,
                max_calls: 2,
                blocking_workers: 1,
                ..RuntimeOptions::default()
            },
            Limits {
                connections: 2,
                in_flight: 2,
                shutdown_timeout: Duration::from_millis(40),
                ..Limits::default()
            },
        );
        let mut control =
            Control::bind(&owner.foreign, &path, "retained", true, true, false).unwrap();
        let caller_path = path.clone();
        let caller = thread::spawn(move || {
            let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
            let client = BlockingClient::unix(&runtime, caller_path, "retained").unwrap();
            client.call(
                "/v1/command",
                json!({"token":"hold"}),
                Duration::from_millis(100),
            )
        });
        let by = Instant::now() + Duration::from_secs(2);
        while control.pending.len() != 1 {
            assert!(Instant::now() < by, "foreign callback never queued");
            thread::sleep(Duration::from_millis(1));
        }
        // Model an RT owner that has taken the record but has not completed it.
        let pending = control.poll().unwrap();
        assert!(caller.join().unwrap().is_err());
        let stats = owner.runtime.handle().stats();
        assert_eq!(
            (stats.hosts, stats.in_flight, stats.blocking_jobs),
            (1, 1, 1)
        );
        let close = control.close().unwrap_err();
        assert!(close.contains("5:"), "{close}");
        assert!(
            control.host.is_some(),
            "failed close must retain origin handle"
        );
        assert!(control.release_error.is_none());
        assert!(!control.admission.lock().unwrap().open);
        assert!(xgc2_xrpc::UnixLease::reserve(&path, true).is_err());
        assert!(
            pending.expired(),
            "RT commit must reject the original absolute deadline"
        );
        pending.complete(Err("deadline before local publication"));
        control.close().unwrap();
        assert!(control.host.is_none());
        let stats = owner.runtime.handle().stats();
        assert_eq!(
            (stats.hosts, stats.in_flight, stats.blocking_jobs),
            (0, 0, 0)
        );
        assert!(
            !stats.closing,
            "closing one endpoint cannot close process Runtime"
        );
        let lease = xgc2_xrpc::UnixLease::reserve(&path, false).unwrap();
        drop(lease);
    }

    #[test]
    fn foreign_controls_share_root_call_budget_and_close_independently() {
        let directory = tempfile::Builder::new()
            .permissions(std::os::unix::fs::PermissionsExt::from_mode(0o700))
            .tempdir()
            .unwrap();
        let a = directory.path().join("a.sock");
        let b = directory.path().join("b.sock");
        let owner = TestOwner::new(
            RuntimeOptions {
                max_connections: 4,
                max_calls: 1,
                blocking_workers: 1,
                ..RuntimeOptions::default()
            },
            Limits {
                connections: 2,
                in_flight: 1,
                body_bytes: 1024,
                response_bytes: 1024,
                call_timeout: Duration::from_millis(500),
                ..Limits::default()
            },
        );
        let mut first = Control::bind(&owner.foreign, &a, "a", true, true, false).unwrap();
        let mut second = Control::bind(&owner.foreign, &b, "b", true, true, false).unwrap();
        assert_eq!(owner.runtime.handle().stats().hosts, 2);
        let caller = thread::spawn(move || {
            let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
            let client = BlockingClient::unix(&runtime, a, "a").unwrap();
            client.call(
                "/v1/command",
                json!({"token":"hold"}),
                Duration::from_millis(150),
            )
        });
        let by = Instant::now() + Duration::from_secs(2);
        while first.pending.len() != 1 {
            assert!(
                Instant::now() < by,
                "first endpoint did not consume root slot"
            );
            thread::sleep(Duration::from_millis(1));
        }
        let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
        let client = BlockingClient::unix(&runtime, b, "b").unwrap();
        let error = client
            .request(
                Method::GET,
                "/v1/health",
                None,
                Duration::from_secs(1),
                None,
            )
            .unwrap_err();
        assert!(error.message.contains("resource_exhausted"), "{error}");
        assert!(second.poll().is_none());
        assert_eq!(owner.runtime.handle().stats().blocking_jobs, 1);
        assert!(caller.join().unwrap().is_err());
        first.close().unwrap();
        assert_eq!(owner.runtime.handle().stats().hosts, 1);
        assert!(!owner.runtime.handle().stats().closing);
        assert_eq!(
            client
                .request(
                    Method::GET,
                    "/v1/health",
                    None,
                    Duration::from_secs(1),
                    None
                )
                .unwrap()["state"],
            "active"
        );
        let error = client
            .call(
                "/v1/command",
                json!({"token":"hold","padding":"x".repeat(2048)}),
                Duration::from_secs(1),
            )
            .unwrap_err();
        assert!(error.message.contains("resource_exhausted"), "{error}");
        assert!(
            second.poll().is_none(),
            "root's 1024B request cap must not be expanded to 4096B"
        );
        second.close().unwrap();
        assert_eq!(owner.runtime.handle().stats().hosts, 0);
    }
}
