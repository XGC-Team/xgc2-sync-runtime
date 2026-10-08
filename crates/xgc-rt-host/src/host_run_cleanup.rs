//! Test-only scheduler faults exercise the real C executor cleanup path.
use super::*;
use std::process::Command;
use xgc_rt_core::clock::WallClock;
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

struct NativeFixture {
    root: tempfile::TempDir,
    libraries: Vec<PathBuf>,
    release: PathBuf,
}

impl NativeFixture {
    fn new(count: usize, hang_deactivate: bool) -> Self {
        let root = tempfile::tempdir().unwrap();
        let release = root.path().join("release");
        let mut libraries = Vec::new();
        for index in 0..count {
            let literal = |name: &str| {
                serde_json::to_string(
                    root.path()
                        .join(format!("{index}-{name}"))
                        .to_str()
                        .unwrap(),
                )
                .unwrap()
            };
            let source = format!(
                r#"#define _POSIX_C_SOURCE 200809L
#include "xgc_rt.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
static void mark(const char* path) {{ FILE* f=fopen(path,"a"); if(f) {{ fputs("called\n",f); fclose(f); }} }}
static void* create(const xgc_host_api* host) {{ (void)host; mark({}); return malloc(1); }}
static xgc_status configure(void* self,const char* text) {{ (void)self; (void)text; mark({}); return XGC_OK; }}
static xgc_status activate(void* self) {{ (void)self; mark({}); return XGC_OK; }}
static xgc_status step(void* self,const xgc_step_ctx* ctx) {{ (void)self; (void)ctx; mark({}); return XGC_OK; }}
static xgc_status deactivate(void* self) {{ (void)self; mark({});
if ({hang_deactivate}) {{ while(access({},F_OK)!=0) {{ struct timespec t={{0,10000000}}; nanosleep(&t,NULL); }} }}
return XGC_OK; }}
static const char* domain_state(void* self) {{ (void)self; mark({}); return "stopped"; }}
static void destroy(void* self) {{ mark({}); free(self); }}
__attribute__((destructor)) static void unload(void) {{ mark({}); }}
static const xgc_plugin_vtbl vtbl={{create,configure,activate,step,deactivate,destroy,domain_state}};
static const xgc_plugin_descriptor descriptor={{XGC_RT_ABI_VERSION,0,"cleanup-fixture","0.1.0",NULL,&vtbl}};
const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {{ return &descriptor; }}
"#,
                literal("create"),
                literal("configure"),
                literal("activate"),
                literal("step"),
                literal("deactivate"),
                serde_json::to_string(release.to_str().unwrap()).unwrap(),
                literal("domain"),
                literal("destroy"),
                literal("unload"),
                hang_deactivate = u8::from(hang_deactivate),
            );
            let source_path = root.path().join(format!("{index}.c"));
            let library = root.path().join(format!("module-{index}.so"));
            std::fs::write(&source_path, source).unwrap();
            assert!(Command::new("cc")
                .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared", "-I"])
                .arg(Path::new(env!("CARGO_MANIFEST_DIR")).join("../../abi/include"))
                .arg(source_path)
                .arg("-o")
                .arg(&library)
                .status()
                .unwrap()
                .success());
            libraries.push(library);
        }
        Self {
            root,
            libraries,
            release,
        }
    }

    fn host(&self) -> Host {
        let mut text = r#"[session]
id="cleanup-fixture"
node="n1"
roster=["n1"]
period_ms=10
start_delay_ms=10
run_for_ms=500
[transport]
kind="loopback"
[audit]
dir="audit"
"#
        .to_owned();
        for (index, library) in self.libraries.iter().enumerate() {
            text.push_str(&format!(
                r#"
[[plugin]]
name="fixture{index}"
path={:?}
expected_name="cleanup-fixture"
trigger="on_round"
"#,
                library.to_str().unwrap()
            ));
        }
        Host::new(
            Manifest::from_toml_str(&text).unwrap(),
            self.root.path(),
            Box::new(LoopbackTransport::new(LoopbackBus::new())),
            Arc::new(WallClock::new(0)),
            HostOptions::default(),
        )
        .unwrap()
    }

    fn marker(&self, index: usize, name: &str) -> PathBuf {
        self.root.path().join(format!("{index}-{name}"))
    }

    fn assert_once(&self, index: usize, name: &str) {
        assert_eq!(
            std::fs::read_to_string(self.marker(index, name)).unwrap(),
            "called\n",
            "{index}-{name}"
        );
    }

    fn events(&self) -> Vec<serde_json::Value> {
        std::fs::read_to_string(self.root.path().join("audit/n1/health.jsonl"))
            .unwrap()
            .lines()
            .map(|line| serde_json::from_str(line).unwrap())
            .collect()
    }

    fn assert_error_evidence(&self, reason: &str) {
        let events = self.events();
        assert!(
            events.iter().any(|event| event["event"] == "aborted"
                && event["phase"] == "exception_cleanup"
                && event["reason"].as_str().unwrap().contains(reason)),
            "{events:?}"
        );
        assert!(events
            .iter()
            .any(|event| event["event"] == "stopped" && event["phase"] == "exception_cleanup"));
    }
}

#[test]
fn partial_native_thread_creation_failure_joins_created_executors() {
    let fixture = NativeFixture::new(2, false);
    let host = fixture.host();
    host.rt.spawn_failure_at.store(1, Ordering::Relaxed);
    let runtime = Arc::downgrade(&host.rt);
    let error = host.run(&AtomicBool::new(false)).unwrap_err();
    assert!(error
        .0
        .contains("spawn module thread: injected creation failure"));
    assert!(
        runtime.upgrade().is_none(),
        "neither started Slot nor rejected Slot leaks Runtime"
    );
    for name in ["create", "configure", "domain", "destroy", "unload"] {
        fixture.assert_once(0, name);
    }
    for name in [
        "create",
        "configure",
        "activate",
        "step",
        "domain",
        "destroy",
    ] {
        assert!(
            !fixture.marker(1, name).exists(),
            "second module never entered native code"
        );
    }
    fixture.assert_once(1, "unload");
    fixture.assert_error_evidence("spawn module thread");
}

#[test]
fn scheduler_panic_stops_and_joins_live_native_executors() {
    let fixture = NativeFixture::new(1, false);
    let mut host = fixture.host();
    host.panic_after_step = true;
    let runtime = Arc::downgrade(&host.rt);
    let error = host.run(&AtomicBool::new(false)).unwrap_err();
    assert!(error.0.contains("host scheduler panicked"));
    assert!(
        runtime.upgrade().is_none(),
        "native executor and Slot finished before returning"
    );
    for name in [
        "create",
        "configure",
        "activate",
        "deactivate",
        "domain",
        "destroy",
        "unload",
    ] {
        fixture.assert_once(0, name);
    }
    assert!(fixture.marker(0, "step").exists());
    fixture.assert_error_evidence("scheduler panicked");
}

#[test]
fn scheduler_panic_with_unquiescent_native_stop_abandons_and_retains_library() {
    let fixture = NativeFixture::new(1, true);
    let mut host = fixture.host();
    host.panic_after_step = true;
    let runtime = Arc::downgrade(&host.rt);
    let began = Instant::now();
    let error = host.run(&AtomicBool::new(false)).unwrap_err();
    assert!(error.0.contains("host scheduler panicked"));
    assert!(began.elapsed() >= LIFECYCLE_GRACE);
    assert!(began.elapsed() < LIFECYCLE_GRACE + Duration::from_secs(2));
    fixture.assert_once(0, "deactivate");
    for name in ["domain", "destroy", "unload"] {
        assert!(
            !fixture.marker(0, name).exists(),
            "{name} must not follow unproven quiescence"
        );
    }
    let runtime = runtime
        .upgrade()
        .expect("abandoned Slot retains Runtime and DLL");
    let instance = runtime.modules[0].instance();
    assert!(instance.abandoned.load(Ordering::Acquire));
    assert!(!instance.done.load(Ordering::Acquire));
    {
        let status = runtime.modules[0].status.lock().unwrap();
        assert_eq!(status.abandons, 1);
        assert_eq!(status.fsm.state(), State::Error);
        assert!(status.last_error.as_ref().unwrap().contains("hung at stop"));
    }
    assert!(std::fs::read_to_string("/proc/self/maps")
        .unwrap()
        .contains(fixture.libraries[0].to_str().unwrap()));
    fixture.assert_error_evidence("scheduler panicked");
    assert!(fixture
        .events()
        .iter()
        .any(|event| event["event"] == "abandoned"
            && event["phase"] == "stop"
            && event["plugin"] == "fixture0"));
    // Release the owned native fixture; even after the late callback returns,
    // the host must not call domain_state/destroy or unload its retained Slot.
    std::fs::write(&fixture.release, b"release").unwrap();
    let until = Instant::now() + Duration::from_secs(1);
    loop {
        let finished = runtime.modules[0]
            .thread
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .is_finished();
        if finished {
            break;
        }
        assert!(Instant::now() < until);
        std::thread::sleep(Duration::from_millis(2));
    }
    runtime.modules[0]
        .thread
        .lock()
        .unwrap()
        .take()
        .unwrap()
        .join()
        .unwrap();
    for name in ["domain", "destroy", "unload"] {
        assert!(!fixture.marker(0, name).exists());
    }
}

#[test]
fn guard_drop_during_rust_unwind_joins_native_executor_once() {
    let fixture = NativeFixture::new(1, false);
    let host = fixture.host();
    let runtime = Arc::downgrade(&host.rt);
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let _guard = ModuleStopGuard::new(host.rt.clone());
        let (ready_tx, ready_rx) = mpsc::channel();
        spawn_module(&host.rt, 0, host.rt.modules[0].instance(), Some(ready_tx)).unwrap();
        ready_rx.recv_timeout(Duration::from_secs(1)).unwrap();
        panic!("unwind guard test");
    }));
    assert!(result.is_err());
    assert!(host.rt.modules[0].thread.lock().unwrap().is_none());
    assert!(host.rt.modules[0].instance().done.load(Ordering::Acquire));
    host.finish_evidence().unwrap();
    drop(host);
    assert!(runtime.upgrade().is_none());
    for name in ["create", "configure", "domain", "destroy", "unload"] {
        fixture.assert_once(0, name);
    }
}
