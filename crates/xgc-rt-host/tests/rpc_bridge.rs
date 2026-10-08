#![cfg(target_os = "linux")]

use serde_json::json;
use std::{
    ffi::CString,
    fs,
    os::unix::{ffi::OsStrExt, fs::PermissionsExt},
    path::{Path, PathBuf},
    process::Command,
    sync::{
        atomic::{AtomicBool, AtomicUsize, Ordering},
        Arc,
    },
    thread,
    time::{Duration, Instant},
};
use xgc2_xrpc::{handler, BlockingClient, Limits, Runtime, RuntimeOptions};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions, RpcBinding};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

struct Fixture {
    root: PathBuf,
    library: PathBuf,
}
static NEXT_FIXTURE: AtomicUsize = AtomicUsize::new(0);
impl Fixture {
    fn new() -> Self {
        let root = std::env::temp_dir().join(format!(
            "xgc-rpc-bridge-{}-{}",
            std::process::id(),
            NEXT_FIXTURE.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir(&root).unwrap();
        fs::set_permissions(&root, fs::Permissions::from_mode(0o700)).unwrap();
        let product = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .canonicalize()
            .unwrap();
        let sdk = product.join("../xrpc/rust/include").canonicalize().unwrap();
        let library = root.join("bridge.so");
        let output = Command::new("cc")
            .args([
                "-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared", "-pthread",
            ])
            .arg("-I")
            .arg(product.join("abi/include"))
            .arg("-I")
            .arg(sdk)
            .arg(Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/rpc_bridge_fixture.c"))
            .arg("-o")
            .arg(&library)
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "C DSO build: {}",
            String::from_utf8_lossy(&output.stderr)
        );
        Self { root, library }
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.root);
    }
}

fn wait(mut predicate: impl FnMut() -> bool) {
    let until = Instant::now() + Duration::from_secs(3);
    while !predicate() {
        assert!(
            Instant::now() < until,
            "bridge did not reach expected native state"
        );
        thread::sleep(Duration::from_millis(2));
    }
}

// RTLD_NOLOAD observes the actual loader reference state rather than silently
// reloading the fixture and masking an absent module code pin.
fn loaded(path: &Path) -> bool {
    let name = CString::new(path.as_os_str().as_bytes()).unwrap();
    let handle = unsafe { libc::dlopen(name.as_ptr(), libc::RTLD_NOW | libc::RTLD_NOLOAD) };
    if handle.is_null() {
        return false;
    }
    assert_eq!(unsafe { libc::dlclose(handle) }, 0);
    true
}

struct Running {
    stop: Arc<AtomicBool>,
    thread: Option<thread::JoinHandle<xgc_rt_host::RunSummary>>,
    library: PathBuf,
}
impl Drop for Running {
    fn drop(&mut self) {
        // Complete an owned SDK callback even after an assertion fails. An
        // already abandoned native Slot/DLL deliberately remains pinned until
        // test-process exit: SDK callback completion cannot undo abandonment.
        if loaded(&self.library) {
            let library = unsafe { libloading::Library::new(&self.library) }.unwrap();
            let open =
                unsafe { library.get::<unsafe extern "C" fn()>(b"bridge_open_gate\0") }.unwrap();
            unsafe { open() };
        }
        self.stop.store(true, Ordering::Release);
        if let Some(handle) = self.thread.take() {
            let _ = handle.join();
        }
    }
}

#[test]
fn failed_native_deactivate_keeps_slot_and_dll_after_sdk_callback_drains() {
    let fixture = Fixture::new();
    assert!(!loaded(&fixture.library));
    let mut runtime = Runtime::new(RuntimeOptions {
        blocking_workers: 1,
        max_calls: 8,
        max_connections: 8,
        ..RuntimeOptions::default()
    })
    .unwrap();
    let limits = Limits {
        connections: 2,
        in_flight: 4,
        body_bytes: 256,
        response_bytes: 256,
        shutdown_timeout: Duration::from_millis(30),
        ..Limits::default()
    };
    let binding = RpcBinding::new(runtime.handle(), limits.clone()).unwrap();
    let first = fixture.root.join("first.sock");
    let second = fixture.root.join("second.sock");
    let manifest = Manifest::from_toml_str(&format!(
        r#"
[session]
id = "rpc-bridge"
node = "local"
roster = ["local"]
period_ms = 5
start_delay_ms = 0
[transport]
kind = "loopback"
[audit]
dir = "audit"
[[plugin]]
name = "first"
path = {:?}
trigger = "on_round"
config = {{ socket = {:?} }}
[[plugin]]
name = "second"
path = {:?}
trigger = "on_round"
config = {{ socket = {:?} }}
"#,
        fixture.library, first, fixture.library, second
    ))
    .unwrap();
    let host = Host::with_manifest_clock(
        manifest,
        &fixture.root,
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
        HostOptions {
            rpc: Some(binding.clone()),
            ..HostOptions::default()
        },
    )
    .unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let run_stop = stop.clone();
    let mut running = Running {
        stop,
        library: fixture.library.clone(),
        thread: Some(thread::spawn(move || host.run(&run_stop).unwrap())),
    };
    wait(|| runtime.handle().stats().hosts == 2);
    let observer = unsafe { libloading::Library::new(&fixture.library) }.unwrap();
    let count =
        unsafe { observer.get::<unsafe extern "C" fn(u32) -> u32>(b"bridge_count\0") }.unwrap();
    wait(|| unsafe { count(1) } == 2);
    assert_eq!(unsafe { count(0) }, 2);
    assert_eq!(unsafe { count(1) }, 2);
    assert_eq!(
        unsafe { count(7) },
        2,
        "both modules inherit the same resolved caps"
    );
    assert_eq!(
        unsafe { count(5) },
        0,
        "getter is unavailable in configure/step"
    );
    let client1 =
        BlockingClient::unix_with_limits(&runtime, &first, "bridge:fixture", limits.clone())
            .unwrap();
    let client2 =
        BlockingClient::unix_with_limits(&runtime, &second, "bridge:fixture", limits.clone())
            .unwrap();
    assert_eq!(
        client1
            .call("/echo", json!({}), Duration::from_secs(1))
            .unwrap(),
        json!({"ok":true})
    );
    assert_eq!(
        client2
            .call("/echo", json!({}), Duration::from_secs(1))
            .unwrap(),
        json!({"ok":true})
    );
    let caller =
        thread::spawn(move || client1.call("/block", json!({}), Duration::from_millis(100)));
    wait(|| runtime.handle().stats().blocking_jobs == 1 && unsafe { count(4) } == 1);
    assert!(
        client2
            .call("/echo", json!({}), Duration::from_millis(100))
            .unwrap_err()
            .message
            .contains("resource_exhausted"),
        "the other module must share the root's one blocking slot"
    );
    assert!(
        caller.join().unwrap().is_err(),
        "lost caller receipt cannot stop the native callback"
    );
    assert_eq!(runtime.handle().stats().blocking_jobs, 1);
    assert_eq!(runtime.handle().stats().in_flight, 1);
    drop(client2);
    drop(observer);
    drop(binding);
    running.stop.store(true, Ordering::Release);
    let stopped_at = Instant::now();
    let summary = running.thread.take().unwrap().join().unwrap();
    assert!(
        stopped_at.elapsed() < Duration::from_secs(1),
        "module close uses the inherited finite cap"
    );
    let held = summary
        .plugins
        .iter()
        .find(|plugin| plugin.name == "first")
        .unwrap();
    let clean = summary
        .plugins
        .iter()
        .find(|plugin| plugin.name == "second")
        .unwrap();
    assert_eq!(
        held.abandons, 1,
        "failed deactivate must become one terminal native abandonment"
    );
    assert!(held
        .last_error
        .as_ref()
        .unwrap()
        .contains("quiescence unproven"));
    assert_eq!(clean.abandons, 0);
    assert!(clean.last_error.is_none());
    assert_eq!(
        summary
            .plugins
            .iter()
            .map(|plugin| plugin.abandons)
            .sum::<u32>(),
        1
    );
    assert!(
        loaded(&fixture.library),
        "the actual callback must retain the library after aggregate Host drop"
    );
    assert!(
        xgc2_xrpc::Host::bind(
            &runtime,
            &first,
            "replacement".into(),
            limits.clone(),
            true,
            handler(|_, _, _| async { Ok(json!({})) })
        )
        .is_err(),
        "the original endpoint lease remains held"
    );
    assert_eq!(runtime.handle().stats().blocking_jobs, 1);
    assert_eq!(runtime.handle().stats().hosts, 1);

    let retained = unsafe { libloading::Library::new(&fixture.library) }.unwrap();
    let count =
        unsafe { retained.get::<unsafe extern "C" fn(u32) -> u32>(b"bridge_count\0") }.unwrap();
    assert_eq!(
        unsafe { count(6) },
        1,
        "exactly the held callback reports close timeout"
    );
    assert_eq!(
        unsafe { count(8) },
        1,
        "destroy is called only for the module with proven native quiescence"
    );
    assert_eq!(
        unsafe { count(2) },
        1,
        "held native userdata survives because destroy was not authorized"
    );
    let open = unsafe { retained.get::<unsafe extern "C" fn()>(b"bridge_open_gate\0") }.unwrap();
    unsafe { open() };
    wait(|| runtime.handle().stats().hosts == 0 && runtime.handle().stats().blocking_jobs == 0);
    assert_eq!(
        unsafe { count(8) },
        1,
        "SDK drain must never trigger late native destroy"
    );
    assert_eq!(
        unsafe { count(2) },
        1,
        "the abandoned native instance is permanently retained"
    );
    assert_eq!(unsafe { count(5) }, 0);
    drop(retained);
    assert!(
        loaded(&fixture.library),
        "native deactivate failure pins Slot/DLL until process exit even after SDK drain"
    );
    let mut replacement = xgc2_xrpc::Host::bind(
        &runtime,
        &first,
        "replacement".into(),
        limits,
        true,
        handler(|_, _, _| async { Ok(json!({})) }),
    )
    .unwrap();
    replacement.close().unwrap();
    drop(replacement);
    drop(running);
    runtime.close(Duration::from_secs(1)).unwrap();
    assert!(
        loaded(&fixture.library),
        "closing the SDK Runtime is not proof that native code can unload"
    );
}

#[test]
fn successful_native_close_destroys_modules_and_unloads_actual_dll() {
    let fixture = Fixture::new();
    let mut runtime = Runtime::new(RuntimeOptions {
        blocking_workers: 1,
        ..RuntimeOptions::default()
    })
    .unwrap();
    let limits = Limits {
        connections: 2,
        in_flight: 4,
        body_bytes: 256,
        response_bytes: 256,
        shutdown_timeout: Duration::from_millis(30),
        ..Limits::default()
    };
    let binding = RpcBinding::new(runtime.handle(), limits.clone()).unwrap();
    let first = fixture.root.join("first.sock");
    let second = fixture.root.join("second.sock");
    let manifest = Manifest::from_toml_str(&format!(
        r#"
[session]
id = "rpc-bridge-normal"
node = "local"
roster = ["local"]
period_ms = 5
start_delay_ms = 0
[transport]
kind = "loopback"
[audit]
dir = "audit"
[[plugin]]
name = "first"
path = {:?}
trigger = "on_round"
config = {{ socket = {:?} }}
[[plugin]]
name = "second"
path = {:?}
trigger = "on_round"
config = {{ socket = {:?} }}
"#,
        fixture.library, first, fixture.library, second
    ))
    .unwrap();
    let host = Host::with_manifest_clock(
        manifest,
        &fixture.root,
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
        HostOptions {
            rpc: Some(binding.clone()),
            ..HostOptions::default()
        },
    )
    .unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let run_stop = stop.clone();
    let mut running = Running {
        stop,
        library: fixture.library.clone(),
        thread: Some(thread::spawn(move || host.run(&run_stop).unwrap())),
    };
    wait(|| runtime.handle().stats().hosts == 2);
    let observer = unsafe { libloading::Library::new(&fixture.library) }.unwrap();
    let count =
        unsafe { observer.get::<unsafe extern "C" fn(u32) -> u32>(b"bridge_count\0") }.unwrap();
    wait(|| unsafe { count(1) } == 2);
    let client1 =
        BlockingClient::unix_with_limits(&runtime, &first, "bridge:fixture", limits.clone())
            .unwrap();
    let client2 =
        BlockingClient::unix_with_limits(&runtime, &second, "bridge:fixture", limits.clone())
            .unwrap();
    assert_eq!(
        client1
            .call("/echo", json!({}), Duration::from_secs(1))
            .unwrap(),
        json!({"ok":true})
    );
    assert_eq!(
        client2
            .call("/echo", json!({}), Duration::from_secs(1))
            .unwrap(),
        json!({"ok":true})
    );
    assert_eq!(
        unsafe { count(7) },
        2,
        "both C modules inherited the root policy"
    );
    drop(client1);
    drop(client2);
    running.stop.store(true, Ordering::Release);
    let summary = running.thread.take().unwrap().join().unwrap();
    assert!(summary.aborted.is_none());
    assert!(summary
        .plugins
        .iter()
        .all(|plugin| plugin.abandons == 0 && plugin.last_error.is_none()));
    assert_eq!(
        unsafe { count(6) },
        0,
        "native close succeeded for both modules"
    );
    assert_eq!(
        unsafe { count(8) },
        2,
        "each successfully deactivated module is destroyed once"
    );
    assert_eq!(
        unsafe { count(2) },
        2,
        "handler and native userdata released after actual completion"
    );
    assert_eq!(unsafe { count(5) }, 0);
    assert!(!first.exists() && !second.exists());
    wait(|| runtime.handle().stats().hosts == 0 && runtime.handle().stats().blocking_jobs == 0);
    drop(observer);
    drop(binding);
    assert!(
        !loaded(&fixture.library),
        "a clean native module lifecycle permits actual DLL unload"
    );
    drop(running);
    runtime.close(Duration::from_secs(1)).unwrap();
}
