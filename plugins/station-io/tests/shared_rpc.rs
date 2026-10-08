//! Actual Host + separately linked station SO ownership, not controller/flight
//! acceptance. ManualClock controls when the native RT owner can take a record;
//! neither the station handler nor the SDK transport is reimplemented here.
use std::{
    path::{Path, PathBuf},
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use serde_json::{json, Value};
use xgc2_xrpc::{
    handler, BlockingClient, CallError, Disposition, Limits, Method, Runtime, RuntimeOptions,
};
use xgc_rt_core::{clock::ManualClock, manifest::Manifest};
use xgc_rt_host::{Host, HostOptions, RpcBinding, RunSummary};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};
use zenoh::Wait;

struct Running {
    stop: Arc<AtomicBool>,
    worker: Option<JoinHandle<RunSummary>>,
}
impl Running {
    fn start(host: Host) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let worker = thread::spawn(move || host.run(&flag).unwrap());
        Self {
            stop,
            worker: Some(worker),
        }
    }
    fn finish(&mut self) -> RunSummary {
        self.stop.store(true, Ordering::Relaxed);
        self.worker.take().unwrap().join().unwrap()
    }
}
impl Drop for Running {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
    }
}

fn wait_for(label: &str, condition: impl Fn() -> bool) {
    let by = Instant::now() + Duration::from_secs(5);
    while !condition() {
        assert!(Instant::now() < by, "{label} did not become true");
        thread::sleep(Duration::from_millis(2));
    }
}

fn command(
    path: &Path,
    instance: &str,
    token: &'static str,
    timeout: Duration,
) -> JoinHandle<Result<Value, CallError>> {
    let path = path.to_owned();
    let instance = instance.to_owned();
    thread::spawn(move || {
        let mut runtime = Runtime::new(RuntimeOptions::default()).unwrap();
        let result = {
            let client = BlockingClient::unix(&runtime, path, instance).unwrap();
            client.call("/v1/command", json!({"token":token}), timeout)
        };
        runtime.close(Duration::from_secs(2)).unwrap();
        result
    })
}

fn describe(client: &BlockingClient) -> String {
    client
        .request(
            Method::GET,
            "/v1/describe",
            None,
            Duration::from_secs(1),
            None,
        )
        .unwrap()["service_ref"]["instance_id"]
        .as_str()
        .unwrap()
        .to_owned()
}

#[test]
fn two_native_station_modules_share_one_root_owner_and_retain_timed_out_handoffs() {
    let station = std::env::var_os("STATION_IO_ELF")
        .map(PathBuf::from)
        .unwrap_or_else(|| {
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/debug/libstation_io.so")
        });
    assert!(station.is_file(), "build the actual station SO first");
    let directory = tempfile::Builder::new()
        .prefix("sol7-station-shared-rpc-")
        .permissions(std::os::unix::fs::PermissionsExt::from_mode(0o700))
        .tempdir()
        .unwrap()
        .keep();
    eprintln!("station shared RPC evidence: {}", directory.display());
    let a = directory.join("a.sock");
    let b = directory.join("b.sock");
    let management_path = directory.join("root.sock");
    let port = std::net::TcpListener::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port();
    let endpoint = format!("tcp/127.0.0.1:{port}");
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config
        .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
        .unwrap();
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .unwrap();
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .unwrap();
    let _peer = zenoh::open(config).wait().unwrap();

    // One explicit test composition root. Each native station export inherits
    // these exact budgets through the official C table and actual library pin.
    let mut root = Runtime::new(RuntimeOptions {
        max_connections: 4,
        max_calls: 2,
        blocking_workers: 2,
        ..RuntimeOptions::default()
    })
    .unwrap();
    let limits = Limits {
        connections: 4,
        in_flight: 2,
        body_bytes: 4096,
        response_bytes: 4096,
        shutdown_timeout: Duration::from_millis(100),
        ..Limits::default()
    };
    let rpc = RpcBinding::new(root.handle(), limits.clone()).unwrap();
    let mut management = xgc2_xrpc::Host::bind(
        &root,
        &management_path,
        "root".into(),
        limits,
        false,
        handler(|context, path, _| async move {
            if context.method == Method::GET && path == "/v1/health" {
                Ok(json!({"state":"active"}))
            } else {
                Err(xgc2_xrpc::Fault::new(
                    "not_found",
                    "unknown test root route",
                ))
            }
        }),
    )
    .unwrap();

    let sha = xgc_rt_host::plugin::sha256_hex(&std::fs::read(&station).unwrap());
    let mut text = r#"[session]
id="native-shared-station-owner"
node="n1"
roster=["n1"]
period_ms=1
epoch_ns=1000000000
run_for_ms=10000
[transport]
kind="loopback"
[audit]
dir="audit"
[[channel]]
name="commands"
qos="event"
[[channel]]
name="commands-1"
qos="event"
"#
    .to_owned();
    for (index, path) in [&a, &b].into_iter().enumerate() {
        let name = format!("station-{index}");
        let robot = format!("xgc2e-{index:020x}");
        let channel = if index == 0 { "commands" } else { "commands-1" };
        text += &format!("\n[[plugin]]\nname={name:?}\npath={:?}\nsha256={sha:?}\ntrigger=\"on_round\"\nconfig={{robot_id={robot:?},zenoh_connect={endpoint:?},command_socket={:?},authority=true,command=true,mission=false}}\nbind={{command={{channel={channel:?}}}}}\n", station.to_str().unwrap(), path.to_str().unwrap());
    }
    let clock = Arc::new(ManualClock::new(1_000_000_000));
    let host = Host::new(
        Manifest::from_toml_str(&text).unwrap(),
        directory.as_path(),
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
        clock.clone(),
        HostOptions {
            rpc: Some(rpc),
            ..HostOptions::default()
        },
    )
    .unwrap();
    let observer = host.health_observer();
    let mut running = Running::start(host);
    wait_for("both native station endpoints", || a.exists() && b.exists());
    wait_for("both actual first native RT steps", || {
        observer.snapshot().is_some_and(|v| {
            v["modules"].as_array().is_some_and(|modules| {
                modules.len() == 2
                    && modules
                        .iter()
                        .all(|module| module["steps"].as_u64().is_some_and(|n| n >= 1))
            })
        })
    });
    assert_eq!(
        root.handle().stats().hosts,
        3,
        "root listener plus two injected module hosts"
    );

    let mut discovery_runtime = Runtime::new(RuntimeOptions::default()).unwrap();
    let (instance_a, instance_b) = {
        let first = BlockingClient::unix(&discovery_runtime, &a, "").unwrap();
        let second = BlockingClient::unix(&discovery_runtime, &b, "").unwrap();
        (describe(&first), describe(&second))
    };
    assert_ne!(
        instance_a, instance_b,
        "each activation owns a distinct incarnation"
    );
    discovery_runtime.close(Duration::from_secs(2)).unwrap();
    wait_for("discovery connections actually closed", || {
        root.handle().stats().inbound_connections == 0
    });
    // Keep the call-quota negative case below distinct from connection quota:
    // two mutation connections plus this management connection fit under four.
    let mut client_runtime = Runtime::new(RuntimeOptions::default()).unwrap();
    let health = BlockingClient::unix(&client_runtime, &management_path, "root").unwrap();

    // Session time is held after the real first step. Requests enter the actual
    // two module queues, but caller timers do not advance Session time or run RT.
    let first = command(&a, &instance_a, "prepare", Duration::from_millis(200));
    let second = command(&b, &instance_b, "prepare", Duration::from_millis(200));
    wait_for("shared root's two actual blocking callbacks", || {
        let stats = root.handle().stats();
        stats.in_flight == 2 && stats.blocking_jobs == 2
    });
    let stats = root.handle().stats();
    assert!(stats.inbound_connections <= 4);
    let error = health
        .request(
            Method::GET,
            "/v1/health",
            None,
            Duration::from_secs(1),
            None,
        )
        .unwrap_err();
    assert!(error.message.contains("resource_exhausted"), "{error}");
    let expired_receipts = [first.join().unwrap(), second.join().unwrap()];
    assert!(expired_receipts.iter().all(Result::is_err));
    let retained = root.handle().stats();
    assert_eq!(
        (retained.hosts, retained.in_flight, retained.blocking_jobs),
        (3, 2, 2)
    );
    assert!(xgc2_xrpc::UnixLease::reserve(&a, true).is_err());
    assert!(xgc2_xrpc::UnixLease::reserve(&b, true).is_err());

    // Actual next RT steps reject the copied expired deadlines and release the
    // corresponding callback receipts. They cannot publish these stale commands.
    clock.advance(2_000_000);
    wait_for("expired records actually discarded by native RT", || {
        let stats = root.handle().stats();
        stats.in_flight == 0 && stats.blocking_jobs == 0
    });
    wait_for("native expired-stage observation", || {
        observer.snapshot().is_some_and(|v| {
            v["modules"].as_array().is_some_and(|modules| {
                modules
                    .iter()
                    .all(|module| module["steps"].as_u64().is_some_and(|steps| steps >= 2))
            })
        })
    });
    let expired = observer.snapshot().unwrap();
    std::fs::write(
        directory.join("expired.json"),
        serde_json::to_vec_pretty(&json!({
            "token":"prepare", "caller_receipts":format!("{expired_receipts:?}"),
            "native":expired, "rpc_resources":root.handle().stats(),
        }))
        .unwrap(),
    )
    .unwrap();
    for module in expired["modules"].as_array().unwrap() {
        assert_eq!(
            module["published"], 0,
            "expired prepare must not publish: {expired}"
        );
    }
    assert_eq!(
        health
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

    let first = command(&a, &instance_a, "hold", Duration::from_secs(1));
    let second = command(&b, &instance_b, "hold", Duration::from_secs(1));
    wait_for("both native publication handoffs", || {
        root.handle().stats().blocking_jobs == 2
    });
    assert!(
        !first.is_finished() && !second.is_finished(),
        "receipt cannot precede RT publication"
    );
    clock.advance(2_000_000);
    assert_eq!(first.join().unwrap().unwrap()["status"], "queued");
    assert_eq!(second.join().unwrap().unwrap()["status"], "queued");
    wait_for("publication callbacks actually quiesced", || {
        let stats = root.handle().stats();
        stats.in_flight == 0 && stats.blocking_jobs == 0
    });
    wait_for("native accepted-stage observation", || {
        observer.snapshot().is_some_and(|v| {
            v["modules"].as_array().is_some_and(|modules| {
                modules
                    .iter()
                    .all(|module| module["steps"].as_u64().is_some_and(|steps| steps >= 3))
            })
        })
    });
    let accepted = observer.snapshot().unwrap();
    std::fs::write(
        directory.join("accepted.json"),
        serde_json::to_vec_pretty(&json!({
            "token":"hold", "caller_receipts":[{"status":"queued"},{"status":"queued"}],
            "native":accepted, "rpc_resources":root.handle().stats(),
        }))
        .unwrap(),
    )
    .unwrap();
    for module in accepted["modules"].as_array().unwrap() {
        assert_eq!(
            module["published"], 1,
            "only confirmed hold may publish before stop: {accepted}"
        );
    }

    // Stop is a request to the aggregate owner, not an atomic rollback of a
    // module step already eligible before deactivation fences its endpoint.
    let pending = command(&a, &instance_a, "stop", Duration::from_secs(1));
    wait_for("actual pending handoff before native stop", || {
        root.handle().stats().blocking_jobs == 1
    });
    let summary = running.finish();
    let receipt = pending.join().unwrap();
    std::fs::write(
        directory.join("stop-race.json"),
        serde_json::to_vec_pretty(&json!({
            "token":"stop", "caller_receipt":format!("{receipt:?}"),
            "replay_allowed":false, "native":summary, "rpc_resources":root.handle().stats(),
        }))
        .unwrap(),
    )
    .unwrap();
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    assert_eq!(summary.plugins.len(), 2);
    for module in &summary.plugins {
        assert_eq!(module.abandons, 0, "{module:?}");
        if module.name == "station-0" {
            assert!(
                (1..=2).contains(&module.published),
                "stop race may commit at most once: {module:?}"
            );
            if let Ok(value) = &receipt {
                assert_eq!(value["status"], "queued");
                assert_eq!(
                    module.published, 2,
                    "known receipt requires local stop publication"
                );
            } else {
                assert_ne!(
                    receipt.as_ref().unwrap_err().disposition,
                    Disposition::NotSent,
                    "this request reached the actual callback before stop"
                );
            }
        } else {
            assert_eq!(module.published, 1, "station-1 has no stop-racing request");
        }
    }
    let stats = root.handle().stats();
    assert_eq!(
        (stats.hosts, stats.in_flight, stats.blocking_jobs),
        (1, 0, 0)
    );
    assert!(!stats.closing);
    assert!(!a.exists() && !b.exists());
    assert_eq!(
        health
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
    drop(health);
    client_runtime.close(Duration::from_secs(2)).unwrap();
    management.close().unwrap();
    drop(management);
    assert_eq!(root.handle().stats().hosts, 0);
    root.close(Duration::from_secs(2)).unwrap();
}
