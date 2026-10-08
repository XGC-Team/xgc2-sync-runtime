//! Real native deactivation failures must retain their callbacks and library.
use serde_json::{json, Value};
use std::{
    io::{BufRead, BufReader},
    os::unix::fs::PermissionsExt,
    path::{Path, PathBuf},
    process::{Child, Command, Stdio},
    sync::{atomic::AtomicBool, Arc},
    time::{Duration, Instant},
};
use xgc2_xrpc::{BlockingClient, Method, Runtime, RuntimeOptions};
use xgc_rt_core::{clock::WallClock, manifest::Manifest};
use xgc_rt_host::{Host, HostOptions, RunSummary};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

struct NativeFixture {
    root: tempfile::TempDir,
    library: PathBuf,
    deactivate: PathBuf,
    domain: PathBuf,
    destroy: PathBuf,
    unload: PathBuf,
}

impl NativeFixture {
    fn new(fail_deactivate: bool) -> Self {
        let root = tempfile::tempdir().unwrap();
        std::fs::set_permissions(root.path(), std::fs::Permissions::from_mode(0o700)).unwrap();
        for name in ["modules", "documents", "audit"] {
            std::fs::create_dir(root.path().join(name)).unwrap();
        }
        let library = root.path().join("modules/module.so");
        let deactivate = root.path().join("deactivate.marker");
        let domain = root.path().join("domain.marker");
        let destroy = root.path().join("destroy.marker");
        let unload = root.path().join("unload.marker");
        let c_string = |path: &Path| serde_json::to_string(path.to_str().unwrap()).unwrap();
        let source = format!(
            r#"#include "xgc_rt.h"
#include <stdio.h>
#include <stdlib.h>
static void mark(const char* path) {{ FILE* f = fopen(path, "a"); if (f) {{ fputs("called\n", f); fclose(f); }} }}
static void* create(const xgc_host_api* host) {{ (void)host; return malloc(1); }}
static xgc_status configure(void* self, const char* text) {{ (void)self; (void)text; return XGC_OK; }}
static xgc_status activate(void* self) {{ (void)self; return XGC_OK; }}
static xgc_status step(void* self, const xgc_step_ctx* ctx) {{ (void)self; (void)ctx; return XGC_OK; }}
static xgc_status deactivate(void* self) {{ (void)self; mark({}); return {}; }}
static const char* domain_state(void* self) {{ (void)self; mark({}); return "quiescent"; }}
static void destroy(void* self) {{ mark({}); free(self); }}
__attribute__((destructor)) static void unload(void) {{ mark({}); }}
static const xgc_plugin_vtbl vtbl = {{ create, configure, activate, step, deactivate, destroy, domain_state }};
static const xgc_plugin_descriptor descriptor = {{ XGC_RT_ABI_VERSION, 0, "deactivate-fixture", "0.1.0", NULL, &vtbl }};
const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {{ return &descriptor; }}
"#,
            c_string(&deactivate),
            if fail_deactivate { "XGC_ERR" } else { "XGC_OK" },
            c_string(&domain),
            c_string(&destroy),
            c_string(&unload),
        );
        let source_path = root.path().join("modules/fixture.c");
        std::fs::write(&source_path, source).unwrap();
        let include = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../abi/include");
        assert!(Command::new("cc")
            .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-I"])
            .arg(include)
            .arg(source_path)
            .arg("-o")
            .arg(&library)
            .status()
            .unwrap()
            .success());
        Self {
            root,
            library,
            deactivate,
            domain,
            destroy,
            unload,
        }
    }

    fn manifest(&self) -> String {
        format!(
            r#"[session]
id = "deactivate-fixture"
node = "n1"
roster = ["n1"]
period_ms = 10
start_delay_ms = 10
run_for_ms = 100
[transport]
kind = "loopback"
[audit]
dir = {:?}
[[plugin]]
name = "fixture"
path = {:?}
expected_name = "deactivate-fixture"
expected_version = "0.1.0"
trigger = "on_round"
"#,
            self.root.path().join("audit").to_str().unwrap(),
            self.library.to_str().unwrap(),
        )
    }

    fn run(&self) -> RunSummary {
        let host = Host::new(
            Manifest::from_toml_str(&self.manifest()).unwrap(),
            self.root.path(),
            Box::new(LoopbackTransport::new(LoopbackBus::new())),
            Arc::new(WallClock::new(0)),
            HostOptions::default(),
        )
        .unwrap();
        host.run(&AtomicBool::new(false)).unwrap()
    }

    fn assert_retained(&self, pid: u32) {
        assert!(self.deactivate.exists(), "native deactivate was called");
        assert!(
            !self.domain.exists(),
            "domain_state must not follow failed deactivate"
        );
        assert!(
            !self.destroy.exists(),
            "destroy must not follow failed deactivate"
        );
        assert!(!self.unload.exists(), "callback library must remain loaded");
        let maps = std::fs::read_to_string(format!("/proc/{pid}/maps")).unwrap();
        assert!(
            maps.contains(self.library.to_str().unwrap()),
            "library retained without an extra dlopen handle"
        );
    }
}

#[test]
fn native_deactivate_error_abandons_slot_and_retains_library() {
    let fixture = NativeFixture::new(true);
    let summary = fixture.run();
    let plugin = &summary.plugins[0];
    assert!(plugin.steps > 0);
    assert_eq!(plugin.state, "error");
    assert_eq!(plugin.abandons, 1);
    assert_eq!(plugin.domain_state, "");
    assert!(plugin
        .last_error
        .as_ref()
        .unwrap()
        .contains("deactivate returned 1; quiescence unproven"));
    fixture.assert_retained(std::process::id());
    let health = std::fs::read_to_string(summary.audit_dir.join("n1/health.jsonl")).unwrap();
    let abandoned: Vec<Value> = health
        .lines()
        .map(|line| serde_json::from_str::<Value>(line).unwrap())
        .filter(|value| value["event"] == "abandoned")
        .collect();
    assert_eq!(abandoned.len(), 1);
    assert_eq!(abandoned[0]["plugin"], "fixture");
    assert_eq!(abandoned[0]["phase"], "deactivate");
    assert_eq!(abandoned[0]["status"], 1);
    assert!(
        summary.audit.complete(),
        "the failure evidence was fully recorded"
    );
}

#[test]
fn native_deactivate_success_destroys_instance_and_unloads_library() {
    let fixture = NativeFixture::new(false);
    let summary = fixture.run();
    let plugin = &summary.plugins[0];
    assert!(plugin.steps > 0);
    assert_eq!(plugin.state, "inactive");
    assert_eq!(plugin.abandons, 0);
    assert_eq!(plugin.domain_state, "quiescent");
    assert!(plugin.last_error.is_none());
    for marker in [
        &fixture.deactivate,
        &fixture.domain,
        &fixture.destroy,
        &fixture.unload,
    ] {
        assert_eq!(std::fs::read_to_string(marker).unwrap(), "called\n");
    }
    let maps = std::fs::read_to_string("/proc/self/maps").unwrap();
    assert!(!maps.contains(fixture.library.to_str().unwrap()));
}

struct OwnedChild(Child);
impl Drop for OwnedChild {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

#[test]
fn manager_requires_restart_and_refuses_unload_after_native_deactivate_error() {
    let fixture = NativeFixture::new(true);
    let socket = fixture.root.path().join("control.sock");
    let launch = || {
        Command::new(env!("CARGO_BIN_EXE_xgc-rt-host"))
            .arg("--control-socket")
            .arg(&socket)
            .arg("--module-root")
            .arg(fixture.root.path().join("modules"))
            .arg("--document-root")
            .arg(fixture.root.path().join("documents"))
            .arg("--audit-root")
            .arg(fixture.root.path().join("audit"))
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .unwrap()
    };
    let mut child = OwnedChild(launch());
    let mut output = BufReader::new(child.0.stdout.take().unwrap());
    let mut line = String::new();
    output.read_line(&mut line).unwrap();
    let reference: Value = serde_json::from_str(&line).unwrap();
    let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
    let client = BlockingClient::unix(
        &runtime,
        &socket,
        reference["service_ref"]["instance_id"].as_str().unwrap(),
    )
    .unwrap();
    client
        .call(
            "/v1/load",
            json!({"manifest_toml":fixture.manifest(),
        "base_dir":fixture.root.path().join("documents")}),
            Duration::from_secs(2),
        )
        .unwrap();
    client
        .call("/v1/start", json!({}), Duration::from_secs(2))
        .unwrap();
    let until = Instant::now() + Duration::from_secs(5);
    let health = loop {
        let value: Value = client
            .request(
                Method::GET,
                "/v1/health",
                None,
                Duration::from_secs(2),
                None,
            )
            .unwrap();
        if value["state"] == "error" {
            break value;
        }
        assert!(
            Instant::now() < until,
            "native failure did not complete: {value}"
        );
        std::thread::sleep(Duration::from_millis(10));
    };
    assert_eq!(health["restart_required"], true);
    assert_eq!(health["summary"]["plugins"][0]["abandons"], 1);
    assert_eq!(health["summary"]["plugins"][0]["state"], "error");
    fixture.assert_retained(child.0.id());
    for (route, input) in [
        ("/v1/unload", json!({"expected_revision":1})),
        ("/v1/start", json!({})),
        ("/v1/stop", json!({})),
        (
            "/v1/configure",
            json!({"expected_revision":1,"persist":false,"modules":{"fixture":{}}}),
        ),
    ] {
        let error = client
            .call(route, input, Duration::from_secs(2))
            .unwrap_err();
        assert!(
            error.to_string().contains("process restart"),
            "{route}: {error}"
        );
    }
    fixture.assert_retained(child.0.id());
    assert!(
        xgc2_xrpc::Host::bind(
            &runtime,
            &socket,
            "replacement-probe".into(),
            xgc2_xrpc::Limits::default(),
            true,
            xgc2_xrpc::handler(|_, _, _| async { Ok(json!({})) }),
        )
        .is_err(),
        "the abandoned aggregate still owns the process endpoint"
    );
    assert_eq!(unsafe { libc::kill(child.0.id() as i32, libc::SIGTERM) }, 0);
    let until = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.0.try_wait().unwrap() {
            assert!(
                !status.success(),
                "abandoned native work must not report a clean exit"
            );
            break;
        }
        assert!(Instant::now() < until, "owned aggregate did not stop");
        std::thread::sleep(Duration::from_millis(10));
    }
    // Fault retention ends at actual process exit. The SDK, after acquiring
    // the exclusive lease, reclaims that process's unreachable stale socket.
    let mut replacement = OwnedChild(launch());
    let mut replacement_output = BufReader::new(replacement.0.stdout.take().unwrap());
    let mut ready = String::new();
    replacement_output.read_line(&mut ready).unwrap();
    let new_reference: Value = serde_json::from_str(&ready).unwrap();
    assert_ne!(
        new_reference["service_ref"]["instance_id"],
        reference["service_ref"]["instance_id"]
    );
    let new_client = BlockingClient::unix(
        &runtime,
        &socket,
        new_reference["service_ref"]["instance_id"]
            .as_str()
            .unwrap(),
    )
    .unwrap();
    assert_eq!(
        new_client
            .request(
                Method::GET,
                "/v1/health",
                None,
                Duration::from_secs(2),
                None
            )
            .unwrap()["state"],
        "empty"
    );
    assert_eq!(
        unsafe { libc::kill(replacement.0.id() as i32, libc::SIGTERM) },
        0
    );
    assert!(replacement.0.wait().unwrap().success());
    assert!(
        !socket.exists(),
        "normal native shutdown releases the endpoint"
    );
}
