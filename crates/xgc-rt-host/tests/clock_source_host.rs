//! Real host/FFI contract tests. The compiled C source is a controlled time
//! authority, not ROS/Gazebo evidence; ROS integration has a separate gate.
use std::path::{Path, PathBuf};
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};
use std::time::{Duration, Instant};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const SOURCE: &str = r#"
#define _POSIX_C_SOURCE 200809L
#include "xgc_rt.h"
#include "xgc_clock_source.h"
#include <stdatomic.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
_Static_assert(sizeof(xgc_clock_observation_v1)==544, "clock observation size");
_Static_assert(offsetof(xgc_clock_observation_v1,publisher)==32, "publisher offset");
_Static_assert(offsetof(xgc_clock_observation_v1,error)==288, "error offset");
_Static_assert(sizeof(xgc_clock_source_descriptor_v1)==16, "descriptor size");
static _Atomic long long stamp=0, steps=0;
static _Atomic int mode=0, gate=0, activating=0, inconsistent=0, clock_created=0;
static unsigned long long seq=0;
void test_stamp(long long v) { atomic_store(&stamp,v); }
void test_mode(int v) { atomic_store(&mode,v); }
long long test_steps(void) { return atomic_load(&steps); }
int test_gate(void) { return atomic_load(&gate); }
int test_activating(void) { return atomic_load(&activating); }
int test_inconsistent(void) { return atomic_load(&inconsistent); }
int test_clock_created(void) { return atomic_load(&clock_created); }
static void* create_clock(void) { atomic_fetch_add(&clock_created,1);return &seq; }
static int start_clock(void* s,const char* c,xgc_clock_observation_v1* o) { (void)s;(void)c;(void)o;return 0; }
static int poll_clock(void* s,uint64_t ns,xgc_clock_observation_v1* o) {
 (void)s;struct timespec t={0,(long)ns};nanosleep(&t,0);
 int m=atomic_load(&mode); o->publisher_count=m==4?2:1; o->dropped=m==5?1:0;
 if(m==1) return 1;
 if(m==6) { struct timespec stall={0,800000000};nanosleep(&stall,0); }
 o->time_ns=atomic_load(&stamp);o->sequence=++seq;
 strcpy(o->publisher,m==3?"/wrong":"/gazebo");return 0;
}
static int set_gate(void* s,uint32_t v) { (void)s;atomic_store(&gate,v);return 0; }
static void stop_clock(void* s) { (void)s;atomic_store(&gate,0); }
static void destroy_clock(void* s) {(void)s;}
static const xgc_clock_source_vtbl_v1 cv={create_clock,start_clock,poll_clock,set_gate,stop_clock,destroy_clock};
static const xgc_clock_source_descriptor_v1 cd={1,0,&cv};
const xgc_clock_source_descriptor_v1* xgc_rt_clock_source_v1(void) {return &cd;}
static void* create_module(const xgc_host_api* h) {(void)h;return &seq;}
static xgc_status configure(void* s,const char* c) {(void)s;(void)c;return XGC_OK;}
static xgc_status life(void* s) {(void)s;atomic_store(&activating,1);while(atomic_load(&mode)==7){struct timespec wait={0,1000000};nanosleep(&wait,0);}return XGC_OK;}
static xgc_status step(void* s,const xgc_step_ctx* c) {(void)s;if((c->now-10000000)/1000000!=(int64_t)c->round)atomic_store(&inconsistent,1);atomic_fetch_add(&steps,1);return XGC_OK;}
static const char* state(void* s) {(void)s;return "controlled-source";}
static const xgc_plugin_vtbl pv={create_module,configure,life,step,life,destroy_clock,state};
static const xgc_plugin_descriptor pd={1,0,"controlled-source","1",0,&pv};
const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {return &pd;}
"#;
struct Fixture {
    dir: PathBuf,
    lib: libloading::Library,
    manifest: Manifest,
    text: String,
}

#[test]
fn aggregate_control_loads_clock_source_by_module_instance_name() {
    use std::io::{BufRead, BufReader};
    use std::os::unix::fs::PermissionsExt;
    use std::process::{Command, Stdio};
    use xgc2_xrpc::{BlockingClient, Runtime, RuntimeOptions};

    struct OwnedChild(std::process::Child);
    impl Drop for OwnedChild {
        fn drop(&mut self) {
            let _ = self.0.kill();
            let _ = self.0.wait();
        }
    }

    let fixture = Fixture::new("aggregate-control-name");
    let endpoint = tempfile::tempdir().unwrap();
    std::fs::set_permissions(endpoint.path(), std::fs::Permissions::from_mode(0o700)).unwrap();
    let socket = endpoint.path().join("control.sock");
    let audit = fixture.dir.join("audit");
    std::fs::create_dir(&audit).unwrap();
    let child = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host"))
        .arg("--control-socket").arg(&socket)
        .arg("--module-root").arg(&fixture.dir)
        .arg("--document-root").arg(&fixture.dir)
        .arg("--audit-root").arg(&audit)
        .stdout(Stdio::piped()).stderr(Stdio::inherit())
        .spawn().unwrap();
    let mut child = OwnedChild(child);
    let mut output = BufReader::new(child.0.stdout.take().unwrap());
    let mut line = String::new();
    output.read_line(&mut line).unwrap();
    let reference: serde_json::Value = serde_json::from_str(&line).unwrap();
    let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
    let client = BlockingClient::unix(&runtime, &socket,
        reference["service_ref"]["instance_id"].as_str().unwrap()).unwrap();
    let loaded = client.call("/v1/load", serde_json::json!({
        "manifest_toml":fixture.text, "base_dir":fixture.dir,
    }), Duration::from_secs(2)).unwrap();
    assert_eq!(loaded["state"], "loaded");
    assert_eq!(loaded["modules"][0], "ros_io");
    assert!(loaded["configuration"]["applied_revision"].is_null());
    client.call("/v1/unload", serde_json::json!({"expected_revision":1}),
        Duration::from_secs(2)).unwrap();
    assert_eq!(unsafe { libc::kill(child.0.id() as i32, libc::SIGTERM) }, 0);
    let until = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.0.try_wait().unwrap() {
            assert!(status.success());
            break;
        }
        assert!(Instant::now() < until, "aggregate clock fixture did not stop");
        std::thread::sleep(Duration::from_millis(10));
    }
    assert!(!socket.exists());
}

impl Fixture {
    fn new(name: &str) -> Self {
        let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("clock-source-{name}"));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let source = dir.join("source.c");
        let out = dir.join("source.so");
        std::fs::write(&source, SOURCE).unwrap();
        let include = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../abi/include");
        let status = std::process::Command::new("cc")
            .args([
                "-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-I",
            ])
            .arg(include)
            .arg(source)
            .arg("-o")
            .arg(&out)
            .status()
            .unwrap();
        assert!(status.success());
        let sha = xgc_rt_host::plugin::sha256_hex(&std::fs::read(&out).unwrap());
        let text = format!(
            r#"
[session]
id="clock-{name}"
node="n1"
roster=["n1"]
period_ms=1
epoch_ns=10000000
[transport]
kind="loopback"
[audit]
dir="audit"
[clock_source]
kind="ros1_sim"
plugin="ros_io"
topic="/clock"
expected_publisher="/gazebo"
world_instance_id="test-world-instance"
startup_timeout_wall_ms=500
stale_after_wall_ms=50
max_advance_ns=50000000
poll_wall_ms=2
queue_capacity=256
[[plugin]]
name="ros_io"
path="source.so"
sha256="{sha}"
trigger="both"
wake_ms=0.1
step_budget_ms=10
[plugin.config]
node_name="host_n1"
"#
        );
        Self {
            dir,
            lib: unsafe { libloading::Library::new(out).unwrap() },
            manifest: Manifest::from_toml_str(&text).unwrap(),
            text,
        }
    }
    fn host(&self) -> Host {
        Host::with_manifest_clock(
            self.manifest.clone(),
            &self.dir,
            Box::new(LoopbackTransport::new(LoopbackBus::new())),
            HostOptions::default(),
        )
        .unwrap()
    }
    fn stamp(&self, t: i64) {
        unsafe {
            self.lib
                .get::<unsafe extern "C" fn(i64)>(b"test_stamp\0")
                .unwrap()(t)
        }
    }
    fn mode(&self, m: i32) {
        unsafe {
            self.lib
                .get::<unsafe extern "C" fn(i32)>(b"test_mode\0")
                .unwrap()(m)
        }
    }
    fn steps(&self) -> i64 {
        unsafe {
            self.lib
                .get::<unsafe extern "C" fn() -> i64>(b"test_steps\0")
                .unwrap()()
        }
    }
    fn gate(&self) -> i32 {
        unsafe {
            self.lib
                .get::<unsafe extern "C" fn() -> i32>(b"test_gate\0")
                .unwrap()()
        }
    }
    fn epoch_ready(&self) {
        wait(|| {
            std::fs::read_to_string(self.dir.join("audit/n1/health.jsonl"))
                .unwrap_or_default()
                .contains("\"event\":\"epoch\"")
        });
    }
}
fn wait(mut f: impl FnMut() -> bool) {
    let start = Instant::now();
    while !f() {
        assert!(start.elapsed() < Duration::from_secs(3), "timeout");
        std::thread::sleep(Duration::from_millis(2));
    }
}
#[test]
fn real_host_freezes_duplicate_and_silent_clock_resumes_then_stops_on_wall_time() {
    let f = Fixture::new("pause");
    let host = f.host();
    let stop = Arc::new(AtomicBool::new(false));
    let s = stop.clone();
    let thread = std::thread::spawn(move || host.run(&s).unwrap());
    f.epoch_ready();
    assert_eq!(f.steps(), 0, "zero is a sample, before the shared epoch");
    f.stamp(10_000_000);
    wait(|| f.steps() > 0);
    let count = f.steps();
    std::thread::sleep(Duration::from_millis(1100));
    assert_eq!(
        f.steps(),
        count,
        "duplicate time plus wake_ms must not advance modules"
    );
    assert_eq!(f.gate(), 0);
    let health = std::fs::read_to_string(f.dir.join("audit/n1/health.jsonl")).unwrap();
    assert!(health.contains("host_liveness") && health.contains("steady_elapsed_ns"));
    f.mode(1);
    std::thread::sleep(Duration::from_millis(70));
    assert_eq!(f.steps(), count);
    f.mode(0);
    f.stamp(11_000_000);
    wait(|| f.steps() > count);
    f.mode(1);
    std::thread::sleep(Duration::from_millis(70));
    let begin = Instant::now();
    stop.store(true, Ordering::Release);
    let summary = thread.join().unwrap();
    assert!(begin.elapsed() < Duration::from_millis(600));
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    assert!(summary.plugins.iter().all(|p| p.state == "inactive"));
    assert_eq!(f.gate(), 0);
}
#[test]
fn real_host_faults_on_reset_jump_authority_multiple_publishers_and_drop() {
    for (name, stamp, mode, reason) in [
        ("reset", 9_000_000, 0, "backward"),
        ("jump", 80_000_000, 0, "advance"),
        ("authority", 11_000_000, 3, "authority"),
        ("multiple", 11_000_000, 4, "multiple"),
        ("drops", 11_000_000, 5, "dropped"),
    ] {
        let f = Fixture::new(name);
        let host = f.host();
        let thread = std::thread::spawn(move || host.run(&AtomicBool::new(false)).unwrap());
        f.epoch_ready();
        f.stamp(10_000_000);
        wait(|| f.steps() > 0);
        f.mode(mode);
        f.stamp(stamp);
        wait(|| thread.is_finished());
        let summary = thread.join().unwrap();
        assert!(
            summary.aborted.as_deref().unwrap_or("").contains(reason),
            "{name}: {:?}",
            summary.aborted
        );
        assert_eq!(f.gate(), 0);
    }
}
#[test]
fn real_host_startup_timeout_and_late_epoch_reject_before_steps() {
    let f = Fixture::new("timeout");
    f.mode(1);
    let begin = Instant::now();
    let error = f.host().run(&AtomicBool::new(false)).unwrap_err();
    assert!(error.0.contains("startup timeout"));
    assert!(begin.elapsed() < Duration::from_secs(2));
    assert_eq!(f.steps(), 0);
    let f = Fixture::new("late");
    f.stamp(10_000_000);
    let summary = f.host().run(&AtomicBool::new(false)).unwrap();
    assert!(summary.aborted.unwrap().contains("epoch passed"));
    assert_eq!(f.steps(), 0);
}
#[test]
fn real_host_abandons_unbounded_source_and_returns_fault() {
    let f = Fixture::new("hung");
    let host = f.host();
    let thread = std::thread::spawn(move || host.run(&AtomicBool::new(false)).unwrap());
    f.epoch_ready();
    f.stamp(10_000_000);
    wait(|| f.steps() > 0);
    f.mode(6);
    let begin = Instant::now();
    wait(|| thread.is_finished());
    let summary = thread.join().unwrap();
    assert!(summary.aborted.unwrap().contains("clock source"));
    assert!(begin.elapsed() < Duration::from_secs(2));
    // Let the abandoned controlled call return before unloading our observer.
    f.mode(1);
    std::thread::sleep(Duration::from_millis(850));
}
#[test]
fn strict_manifest_rejects_unpinned_local_epoch_probe_and_unknown_source_keys() {
    let f = Fixture::new("manifest");
    f.manifest.resolve().unwrap();
    let mut m = f.manifest.clone();
    m.session.epoch_ns = None;
    assert!(m.resolve().is_err());
    let mut m = f.manifest.clone();
    m.plugins[0].sha256 = None;
    assert!(m.resolve().is_err());
    let mut m = f.manifest.clone();
    m.clock_source.as_mut().unwrap().poll_wall_ms = 51;
    assert!(m.resolve().is_err());
    let mut m = f.manifest.clone();
    m.clock_source.as_mut().unwrap().expected_publisher = "gazebo".into();
    assert!(m.resolve().is_err());
    let mut m = f.manifest.clone();
    m.clock_source.as_mut().unwrap().world_instance_id = " ".into();
    assert!(m.resolve().is_err());
    let mut m = f.manifest.clone();
    m.session.run_for_ms = Some(u64::MAX);
    assert!(m.resolve().is_err());
    for bad in [
        f.text.replace("kind=\"ros1_sim\"", "kind=\"ros1_sim\"\nunknown=true"),
        f.text.replace("kind=\"ros1_sim\"", "kind=\"wall\""),
        f.text.replace("poll_wall_ms=2", "poll_wall_ms=\"2\""),
    ] { assert!(Manifest::from_toml_str(&bad).is_err(), "accepted strict-source violation"); }
    let probe = f.text.replace("[clock_source]", "[clock]\nrole=\"server\"\nserver=\"n1\"\n[clock_source]");
    assert!(Manifest::from_toml_str(&probe).unwrap().resolve().is_err());
    assert_eq!(
        std::mem::size_of::<xgc_rt_host::clock_source_abi::Observation>(),
        544
    );
    assert_eq!(
        std::mem::offset_of!(xgc_rt_host::clock_source_abi::Observation, error),
        288
    );
}

#[test]
fn accepted_generation_time_and_round_stay_consistent_across_slow_activation() {
    let f = Fixture::new("activation-race");
    f.mode(7);
    let host = f.host();
    let stop = Arc::new(AtomicBool::new(false));
    let thread_stop = stop.clone();
    let thread = std::thread::spawn(move || host.run(&thread_stop).unwrap());
    f.epoch_ready();
    f.stamp(10_000_000);
    wait(|| unsafe { f.lib.get::<unsafe extern "C" fn() -> i32>(b"test_activating\0").unwrap()() != 0 });
    assert_eq!(f.steps(), 0);
    f.stamp(11_000_000);
    std::thread::sleep(Duration::from_millis(15));
    f.mode(0);
    wait(|| f.steps() > 0);
    stop.store(true, Ordering::Release);
    assert!(thread.join().unwrap().aborted.is_none());
    assert_eq!(unsafe { f.lib.get::<unsafe extern "C" fn() -> i32>(b"test_inconsistent\0").unwrap()() }, 0,
        "a new source time must not be attached to an older generation/round");
}

#[test]
fn absent_source_retains_wall_execution_without_starting_native_clock_service() {
    let mut f = Fixture::new("wall-default");
    f.manifest.clock_source = None;
    f.manifest.session.epoch_ns = None;
    f.manifest.session.start_delay_ms = 1;
    f.manifest.session.run_for_ms = Some(30);
    let summary = f.host().run(&AtomicBool::new(false)).unwrap();
    assert!(summary.aborted.is_none());
    assert!(summary.e0_ns > 1_000_000_000_000_000_000);
    assert!(summary.plugins[0].steps > 0);
    assert_eq!(unsafe { f.lib.get::<unsafe extern "C" fn() -> i32>(b"test_clock_created\0").unwrap()() }, 0);
}

#[test]
fn normal_advances_before_a_distant_epoch_do_not_accumulate_as_a_jump() {
    let mut f = Fixture::new("future-epoch");
    f.manifest.session.epoch_ns = Some(1_000_000_000);
    let host = f.host();
    let stop = Arc::new(AtomicBool::new(false));
    let s = stop.clone();
    let thread = std::thread::spawn(move || host.run(&s).unwrap());
    f.epoch_ready();
    for n in 1..=40 {
        f.stamp(n * 5_000_000);
        std::thread::sleep(Duration::from_millis(6));
    }
    stop.store(true, Ordering::Release);
    let summary = thread.join().unwrap();
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    assert_eq!(summary.rounds, 0);
    assert_eq!(f.steps(), 0, "observing pre-epoch time cannot run domain steps");
}
