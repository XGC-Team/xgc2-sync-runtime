//! Private Zenoh uplink plus a real command consumer. String fixtures are not the source.
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};
use zenoh::Wait;

fn artifact(env: &str, name: &str) -> PathBuf {
    if let Some(path) = std::env::var_os(env) {
        return PathBuf::from(path);
    }
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/debug").join(name)
}

fn nonce() -> u64 {
    static NEXT: AtomicU64 = AtomicU64::new(1);
    let time = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as u64;
    time ^ NEXT.fetch_add(1, Ordering::Relaxed)
}

fn free_port() -> u16 {
    let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
    listener.local_addr().unwrap().port()
}

fn peer_listen(endpoint: &str) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config.insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]")).unwrap();
    config.insert_json5("scouting/multicast/enabled", "false").unwrap();
    config.insert_json5("scouting/gossip/enabled", "false").unwrap();
    config
}

fn compile_plugin(source: &Path, output: &Path, define: Option<&str>) {
    let include = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../abi/include");
    let mut command = Command::new("cc");
    command.args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-I"]).arg(&include);
    if let Some(define) = define {
        command.arg(define);
    }
    let status = command.arg(source).arg("-o").arg(output).status().unwrap();
    assert!(status.success(), "cc {}", source.display());
}

fn plugin_line(name: &str, path: &Path, config: &str, bind: &str) -> String {
    let sha = xgc_rt_host::plugin::sha256_hex(&std::fs::read(path).unwrap());
    format!(
        "\n[[plugin]]\nname={name:?}\npath={:?}\nsha256={sha:?}\ntrigger=\"on_round\"\nstep_budget_ms=20\nconfig={config}\nbind={bind}\n",
        path.to_str().unwrap()
    )
}

fn cli(bin: &Path, args: &[&str]) -> std::process::Output {
    Command::new(bin).args(args).output().expect("station-io-cmd")
}

struct Uplink {
    samples: Arc<Mutex<Vec<(String, String)>>>,
    _subscriber: zenoh::pubsub::Subscriber<()>,
    _session: zenoh::Session,
}

fn listen(endpoint: &str) -> Uplink {
    let session = zenoh::open(peer_listen(endpoint)).wait().unwrap();
    let samples = Arc::new(Mutex::new(Vec::<(String, String)>::new()));
    let slot = Arc::clone(&samples);
    let subscriber = session
        .declare_subscriber("xgc2/*/up/**")
        .callback(move |sample| {
            let key = sample.key_expr().to_string();
            let body = String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned();
            slot.lock().unwrap().push((key, body));
        })
        .wait()
        .unwrap();
    Uplink { samples, _subscriber: subscriber, _session: session }
}

fn run_host(manifest: String, dir: &Path, stop: &AtomicBool) -> xgc_rt_host::RunSummary {
    std::fs::write(dir.join("node.toml"), &manifest).unwrap();
    let host = Host::new(
        Manifest::from_toml_str(&manifest).unwrap(),
        dir,
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
        Arc::new(WallClock::new(0)),
        HostOptions::default(),
    )
    .unwrap_or_else(|e| panic!("host: {e} manifest:\n{manifest}"));
    host.run(stop).unwrap_or_else(|e| panic!("run: {e}"))
}

fn required_artifact(variable: &str) -> PathBuf {
    let path = PathBuf::from(std::env::var_os(variable).unwrap_or_else(|| panic!("{variable} must name the real loadable plugin; run plugins/station-io/test.sh")));
    assert!(path.is_absolute() && path.is_file(), "{variable}: {}", path.display());
    path
}

struct RunningHost {
    stop: Arc<AtomicBool>,
    worker: Option<thread::JoinHandle<xgc_rt_host::RunSummary>>,
}
impl RunningHost {
    fn start(manifest: String, dir: &Path) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let flag = Arc::clone(&stop);
        let dir = dir.to_owned();
        Self { stop, worker: Some(thread::spawn(move || run_host(manifest, &dir, &flag))) }
    }
    fn wait_socket(&self, socket: &Path) {
        let deadline = Instant::now() + Duration::from_secs(8);
        while !socket.exists() && Instant::now() < deadline {
            assert!(!self.worker.as_ref().unwrap().is_finished(), "host exited before command socket");
            thread::sleep(Duration::from_millis(20));
        }
        assert!(socket.exists(), "command socket was not created");
    }
    fn finish(&mut self) -> xgc_rt_host::RunSummary {
        self.stop.store(true, Ordering::Relaxed);
        self.worker.take().unwrap().join().unwrap()
    }
}
impl Drop for RunningHost {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(worker) = self.worker.take() { let _ = worker.join(); }
    }
}

fn status_seen(uplink: &Uplink, state: &str) -> bool {
    uplink.samples.lock().unwrap().iter().any(|(key, body)| {
        if !key.ends_with("/forwarder_hb") { return false; }
        let value: serde_json::Value = serde_json::from_str(body).unwrap();
        value["channels"].as_array().is_some_and(|channels| {
            channels.iter().any(|channel| channel["id"] == "controller" && channel["text"] == state)
        })
    })
}

#[test]
fn takeoff_reaches_real_smc_and_fs150_and_station_keeps_canonical_fields() {
    let station = artifact("STATION_IO_ELF", "libstation_io.so");
    let vehicle = required_artifact("LIGHTWEIGHT_VEHICLE_ELF");
    let controller = required_artifact("CTL_PX4_ELF");
    let bin = artifact("STATION_IO_CMD", "station-io-cmd");
    assert!(station.is_file() && bin.is_file(), "run plugins/station-io/test.sh");
    let id = nonce();
    let endpoint = format!("tcp/127.0.0.1:{}", free_port());
    let socket = format!("/tmp/xgc-sio-{id}.sock");
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("station-chain-{id}"));
    std::fs::create_dir_all(&dir).unwrap();
    let mission = dir.join("mission.bin");
    let mut timeline = vec![0u8; 240];
    timeline[..4].copy_from_slice(&1u32.to_le_bytes());
    std::fs::write(&mission, &timeline).unwrap();
    // This sink checks only mission-byte IPC; it is not the flight controller.
    let sink = dir.join("mission.so");
    let receipt = dir.join("mission.txt");
    compile_plugin(&Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/mission_sink.c"), &sink,
        Some(&format!("-DOUTPUT=\"{}\"", receipt.display())));
    let uplink = listen(&endpoint);
    let epoch = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_nanos() as u64 + 2_000_000_000;
    let mut text = format!(r#"[session]
id="station-lightweight-command-chain"
node="n1"
roster=["n1"]
period_ms=1
epoch_ns={epoch}
run_for_ms=45000
[transport]
kind="loopback"
[audit]
dir="audit"
"#);
    for (name, qos) in [("command", "event"), ("mission", "event"), ("pose", "state"),
        ("velocity", "state"), ("paired", "state"), ("status", "state"), ("imu", "state"),
        ("fcu_state", "state"), ("setpoint", "control"), ("attitude", "control"), ("fcu_request", "event")] {
        text += &format!("\n[[channel]]\nname={name:?}\nqos={qos:?}\n");
    }
    text += &plugin_line("station-io", &station,
        &format!("{{robot_id=\"xgc2e-0123456789abcdef0123\",zenoh_connect={endpoint:?},command_socket={socket:?},authority=true,command=true,mission=true,frame_id=\"world\",child_frame_id=\"uav1/base_link\"}}"),
        r#"{local_pose={channel="pose",from=["n1"]},local_velocity={channel="velocity",from=["n1"]},imu={channel="imu",from=["n1"]},fcu_state={channel="fcu_state",from=["n1"]},controller_status={channel="status",from=["n1"]},command={channel="command"},mission_request={channel="mission"}}"#);
    text += &plugin_line("lightweight-fs150", &vehicle,
        &format!("{{model=\"fs150\",epoch_ns={epoch},step_ms=1,output_ms=10,initial_pose=[1.0,2.0,0.0,0.0]}}"),
        r#"{pose={channel="pose"},velocity={channel="velocity"},imu={channel="imu"},fcu_state={channel="fcu_state"},paired_state={channel="paired"},setpoint={channel="setpoint",from=["n1"]},attitude_command={channel="attitude",from=["n1"]},fcu_request={channel="fcu_request",from=["n1"]}}"#);
    text += &plugin_line("controller-smc", &controller,
        r#"{time_source="session",tracking_backend="smc",takeoff_altitude=1.0,planning_period=0.1}"#,
        r#"{local_pose={channel="pose",from=["n1"]},vrpn_pose={channel="pose",from=["n1"]},local_velocity={channel="velocity",from=["n1"]},imu={channel="imu",from=["n1"]},fcu_state={channel="fcu_state",from=["n1"]},command={channel="command",from=["n1"]},setpoint={channel="setpoint"},attitude_command={channel="attitude"},fcu_request_full={channel="fcu_request"},status={channel="status"}}"#);
    text += &plugin_line("mission-sink", &sink, "{}", r#"{mission_request={channel="mission",from=["n1"]}}"#);
    let mut running = RunningHost::start(text, &dir);
    let socket_path = PathBuf::from(&socket);
    running.wait_socket(&socket_path);
    let ready_deadline = Instant::now() + Duration::from_secs(12);
    while Instant::now() < ready_deadline && !status_seen(&uplink, "Ready") {
        thread::sleep(Duration::from_millis(25));
    }
    assert!(status_seen(&uplink, "Ready"), "real controller did not finish its sensor self-check");
    let takeoff = cli(&bin, &[&socket, "command", "takeoff"]);
    assert!(takeoff.status.success(), "takeoff stderr={}", String::from_utf8_lossy(&takeoff.stderr));
    assert_eq!(takeoff.stdout, b"queued\n");
    let admitted = cli(&bin, &[&socket, "mission-file", mission.to_str().unwrap()]);
    assert!(admitted.status.success(), "mission stderr={}", String::from_utf8_lossy(&admitted.stderr));
    let deadline = Instant::now() + Duration::from_secs(20);
    let (mut lifted, mut velocity, mut imu, mut armed, mut consumed) = (false, false, false, false, false);
    while Instant::now() < deadline && !(lifted && velocity && imu && armed && consumed && status_seen(&uplink, "Hover")) {
        for (key, body) in uplink.samples.lock().unwrap().iter() {
            assert!(!key.ends_with("/power"), "unbound battery was invented: {body}");
            let value: serde_json::Value = serde_json::from_str(body).unwrap();
            assert_eq!(value["v"], 1);
            assert!(value["sequence"].as_u64().is_some_and(|v| v > 0), "{body}");
            assert!(value["t_ms"].as_i64().is_some_and(|v| v > 0), "{body}");
            if key.ends_with("/local_pose") {
                assert_eq!(value["frame_id"], "world");
                assert_eq!(value["child_frame_id"], "uav1/base_link");
                assert!(value["orientation"]["w"].as_f64().is_some_and(f64::is_finite));
                lifted |= value["position"]["z"].as_f64().is_some_and(|z| z > 0.5);
            } else if key.ends_with("/local_velocity") {
                velocity |= value["linear"].is_object() && value["angular"].is_object();
            } else if key.ends_with("/imu") {
                imu |= value["orientation"].is_null() && value["covariance"]["orientation"].is_null()
                    && value["covariance"]["angular_velocity"].is_null()
                    && value["covariance"]["linear_acceleration"].is_null()
                    && value["angular_velocity"]["z"].as_f64().is_some_and(f64::is_finite)
                    && value["linear_acceleration"]["z"].as_f64().is_some_and(f64::is_finite);
            } else if key.ends_with("/flight_state") {
                armed |= value["connected"] == true && value["armed"] == true && value["mode"] == "OFFBOARD";
                assert!(value["landed_state"].is_null(), "unprovided landed state was invented: {body}");
            }
        }
        consumed = std::fs::read_to_string(&receipt).unwrap_or_default().contains("mission 240");
        thread::sleep(Duration::from_millis(25));
    }
    let hovered = status_seen(&uplink, "Hover");
    let summary = running.finish();
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    assert!(lifted && armed && hovered, "queued takeoff was not consumed by real SMC/FS150: lifted={lifted} armed={armed} hovered={hovered}");
    assert!(velocity && imu && consumed, "canonical fields/mission IPC: velocity={velocity} imu={imu} consumed={consumed}");
    assert!(!socket_path.exists(), "Stop left the command socket owned");
    let replacement = std::os::unix::net::UnixListener::bind(&socket_path).unwrap();
    drop(replacement);
    std::fs::remove_file(&socket_path).unwrap();
    thread::sleep(Duration::from_millis(150));
    let stopped_count = uplink.samples.lock().unwrap().len();
    thread::sleep(Duration::from_millis(150));
    assert_eq!(stopped_count, uplink.samples.lock().unwrap().len(), "station kept publishing after release");
}

#[test]
fn unbound_command_and_mission_are_rejected_with_transport_fixture_only() {
    let station = artifact("STATION_IO_ELF", "libstation_io.so");
    let bin = artifact("STATION_IO_CMD", "station-io-cmd");
    assert!(station.is_file() && bin.is_file(), "run plugins/station-io/test.sh");
    let id = nonce();
    let endpoint = format!("tcp/127.0.0.1:{}", free_port());
    let socket = format!("/tmp/xgc-sio-off-{id}.sock");
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("station-reject-{id}"));
    std::fs::create_dir_all(&dir).unwrap();
    let source = dir.join("paired-source.so");
    compile_plugin(&Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/paired_source.c"), &source, None);
    let mission = dir.join("mission.bin");
    let mut timeline = vec![0u8; 240]; timeline[..4].copy_from_slice(&1u32.to_le_bytes());
    std::fs::write(&mission, timeline).unwrap();
    let uplink = listen(&endpoint);
    let mut text = String::from(r#"[session]
id="station-capability-rejection"
node="n1"
roster=["n1"]
period_ms=1
start_delay_ms=40
run_for_ms=6000
[transport]
kind="loopback"
[audit]
dir="audit"
[[channel]]
name="paired"
qos="state"
"#);
    text += &plugin_line("station-io", &station,
        &format!("{{robot_id=\"xgc2e-0123456789abcdef0123\",zenoh_connect={endpoint:?},command_socket={socket:?},authority=true,command=false,mission=false}}"),
        r#"{paired_state={channel="paired",from=["n1"]}}"#);
    text += &plugin_line("paired-source-test-fixture", &source, "{}", r#"{paired_state={channel="paired"}}"#);
    let mut running = RunningHost::start(text, &dir);
    let socket_path = PathBuf::from(&socket);
    running.wait_socket(&socket_path);
    for (args, expected) in [
        (vec![socket.as_str(), "command", "takeoff"], "command capability is not configured"),
        (vec![socket.as_str(), "mission-file", mission.to_str().unwrap()], "mission capability is not configured"),
    ] {
        let result = cli(&bin, &args);
        assert!(!result.status.success());
        assert!(String::from_utf8_lossy(&result.stderr).contains(expected));
    }
    let summary = running.finish();
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    assert!(!socket_path.exists());
    assert!(!uplink.samples.lock().unwrap().iter().any(|(key, _)| key.ends_with("/flight_state")), "transport fixture became flight evidence");
}
