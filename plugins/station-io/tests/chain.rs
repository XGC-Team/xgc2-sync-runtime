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

#[test]
fn command_and_mission_are_consumed_and_zenoh_keeps_real_fields() {
    let station = artifact("STATION_IO_ELF", "libstation_io.so");
    let vehicle = artifact("NUMERIC_VEHICLE_ELF", "libnumeric_vehicle.so");
    let bin = artifact("STATION_IO_CMD", "station-io-cmd");
    assert!(station.is_file() && vehicle.is_file() && bin.is_file(), "run plugins/station-io/test.sh so the native CLI and plugins exist");
    let id = nonce();
    let endpoint = format!("tcp/127.0.0.1:{}", free_port());
    let socket = format!("/tmp/xgc-sio-{id}.sock");
    let mission = format!("/tmp/xgc-sio-{id}.bin");
    let mut timeline = vec![0u8; 240];
    timeline[..4].copy_from_slice(&1u32.to_le_bytes());
    std::fs::write(&mission, &timeline).unwrap();
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("station-chain-{id}"));
    std::fs::create_dir_all(&dir).unwrap();
    let imu = dir.join("imu.so");
    let sink = dir.join("mission.so");
    let receipt = dir.join("mission.txt");
    compile_plugin(&Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/imu_source.c"), &imu, None);
    compile_plugin(
        &Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/mission_sink.c"),
        &sink,
        Some(&format!("-DOUTPUT=\"{}\"", receipt.display())),
    );
    let uplink = listen(&endpoint);
    let robot = "xgc2e-0123456789abcdef0123";
    let mut text = format!(
        r#"[session]
id="station-io-chain"
node="n1"
roster=["n1"]
period_ms=10
start_delay_ms=40
run_for_ms=8000
[transport]
kind="loopback"
[audit]
dir="audit"
[[channel]]
name="command"
qos="event"
[[channel]]
name="mission"
qos="event"
[[channel]]
name="paired"
qos="state"
[[channel]]
name="status"
qos="state"
[[channel]]
name="imu"
qos="state"
[[channel]]
name="pva"
qos="control"
"#
    );
    let station_config = format!(
        "{{robot_id={robot:?},zenoh_connect={endpoint:?},command_socket={socket:?},authority=true,command=true,mission=true}}"
    );
    text += &plugin_line(
        "station-io",
        &station,
        &station_config,
        r#"{paired_state={channel="paired",from=["n1"]},controller_status={channel="status",from=["n1"]},imu={channel="imu",from=["n1"]},command={channel="command"},mission_request={channel="mission"}}"#,
    );
    text += &plugin_line(
        "numeric-vehicle",
        &vehicle,
        "{initial_position=[1.0,2.0,3.0],initial_velocity=[0.0,0.0,0.0]}",
        r#"{command={channel="command",from=["n1"]},position_target={channel="pva",from=["n1"]},paired_state={channel="paired"},controller_state={channel="status"}}"#,
    );
    text += &plugin_line("imu-source", &imu, "{}", r#"{imu={channel="imu"}}"#);
    text += &plugin_line(
        "mission-sink",
        &sink,
        "{}",
        r#"{mission_request={channel="mission",from=["n1"]}}"#,
    );
    let stop = Arc::new(AtomicBool::new(false));
    let stop_host = Arc::clone(&stop);
    let manifest = text.clone();
    let host_dir = dir.clone();
    let runner = thread::spawn(move || run_host(manifest, &host_dir, &stop_host));
    let socket_path = PathBuf::from(&socket);
    let ready_at = Instant::now() + Duration::from_secs(8);
    while !socket_path.exists() && Instant::now() < ready_at {
        assert!(!runner.is_finished(), "host exited before the command socket existed");
        thread::sleep(Duration::from_millis(20));
    }
    assert!(socket_path.exists(), "command socket was not created");
    let takeoff = cli(&bin, &[&socket, "command", "takeoff"]);
    assert!(takeoff.status.success(), "takeoff stderr={}", String::from_utf8_lossy(&takeoff.stderr));
    assert_eq!(takeoff.stdout, b"queued\n");
    let configured_deadline = Instant::now() + Duration::from_secs(4);
    let mut configured = false;
    while Instant::now() < configured_deadline && !configured {
        for (_key, body) in uplink.samples.lock().unwrap().iter() {
            if body.contains("\"text\":\"Ready\"") {
                panic!("queued takeoff was treated as numeric execution: {body}");
            }
            if body.contains("\"text\":\"Configured\"") {
                configured = true;
            }
        }
        thread::sleep(Duration::from_millis(30));
    }
    assert!(configured, "controller stayed unpublished after a queued ctl-px4 token");
    let prepared = cli(&bin, &[&socket, "command", "prepare"]);
    assert!(prepared.status.success(), "prepare stderr={}", String::from_utf8_lossy(&prepared.stderr));
    assert_eq!(prepared.stdout, b"queued\n");
    let admitted = cli(&bin, &[&socket, "mission-file", &mission]);
    assert!(admitted.status.success(), "mission stderr={}", String::from_utf8_lossy(&admitted.stderr));
    assert_eq!(admitted.stdout, b"queued\n");
    let deadline = Instant::now() + Duration::from_secs(8);
    let mut pose = false;
    let mut velocity_honest = false;
    let mut imu_honest = false;
    let mut ready = false;
    let mut consumed = false;
    while Instant::now() < deadline && !(pose && velocity_honest && imu_honest && ready && consumed) {
        let samples = uplink.samples.lock().unwrap().clone();
        for (key, body) in &samples {
            assert!(!key.ends_with("/flight_state"), "unbound FCU was published: {body}");
            assert!(!key.ends_with("/power"), "unbound battery was published: {body}");
            let value: serde_json::Value = serde_json::from_str(body).unwrap_or_else(|e| panic!("{e}: {body}"));
            if key.ends_with("/local_pose") {
                pose = value["position"]["x"] == 1.0;
            }
            if key.ends_with("/local_velocity") {
                velocity_honest = value["angular"].is_null() && value["linear"].is_object();
            }
            if key.ends_with("/imu") {
                imu_honest = value["orientation"].is_null()
                    && value["covariance"]["orientation"].is_null()
                    && value["covariance"]["angular_velocity"].is_null()
                    && value["covariance"]["linear_acceleration"].is_null()
                    && value["angular_velocity"]["z"] == 0.25
                    && value["linear_acceleration"]["z"] == 9.81;
                assert!(!body.contains("[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"), "{body}");
            }
            if key.ends_with("/forwarder_hb") {
                ready = value["channels"].as_array().is_some_and(|channels| {
                    channels.iter().any(|channel| channel["id"] == "controller" && channel["text"] == "Ready")
                });
            }
        }
        if receipt.is_file() {
            consumed = std::fs::read_to_string(&receipt).unwrap_or_default().contains("mission 240");
        }
        thread::sleep(Duration::from_millis(30));
    }
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    assert!(pose, "local_pose did not carry the vehicle position");
    assert!(velocity_honest, "paired local_velocity invented an angular measurement");
    assert!(imu_honest, "imu covariance was forged or the sample did not arrive");
    assert!(ready, "controller status did not become semantic heartbeat text Ready");
    assert!(consumed, "mission bytes were queued without a consumer");
    let _ = std::fs::remove_file(&mission);
}

#[test]
fn unbound_command_and_mission_are_rejected() {
    let station = artifact("STATION_IO_ELF", "libstation_io.so");
    let vehicle = artifact("NUMERIC_VEHICLE_ELF", "libnumeric_vehicle.so");
    let bin = artifact("STATION_IO_CMD", "station-io-cmd");
    assert!(station.is_file() && vehicle.is_file() && bin.is_file(), "run plugins/station-io/test.sh");
    let id = nonce();
    let endpoint = format!("tcp/127.0.0.1:{}", free_port());
    let socket = format!("/tmp/xgc-sio-off-{id}.sock");
    let mission = format!("/tmp/xgc-sio-off-{id}.bin");
    let mut timeline = vec![0u8; 240];
    timeline[..4].copy_from_slice(&1u32.to_le_bytes());
    std::fs::write(&mission, &timeline).unwrap();
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("station-reject-{id}"));
    std::fs::create_dir_all(&dir).unwrap();
    let uplink = listen(&endpoint);
    let robot = "xgc2e-0123456789abcdef0123";
    let mut text = format!(
        r#"[session]
id="station-io-reject"
node="n1"
roster=["n1"]
period_ms=10
start_delay_ms=40
run_for_ms=6000
[transport]
kind="loopback"
[audit]
dir="audit"
[[channel]]
name="command"
qos="event"
[[channel]]
name="paired"
qos="state"
[[channel]]
name="status"
qos="state"
[[channel]]
name="pva"
qos="control"
"#
    );
    let station_config = format!(
        "{{robot_id={robot:?},zenoh_connect={endpoint:?},command_socket={socket:?},authority=true,command=false,mission=false}}"
    );
    text += &plugin_line(
        "station-io",
        &station,
        &station_config,
        r#"{paired_state={channel="paired",from=["n1"]},controller_status={channel="status",from=["n1"]}}"#,
    );
    text += &plugin_line(
        "numeric-vehicle",
        &vehicle,
        "{initial_position=[4.0,0.0,0.0],initial_velocity=[0.0,0.0,0.0]}",
        r#"{command={channel="command",from=["n1"]},position_target={channel="pva",from=["n1"]},paired_state={channel="paired"},controller_state={channel="status"}}"#,
    );
    let stop = Arc::new(AtomicBool::new(false));
    let stop_host = Arc::clone(&stop);
    let manifest = text;
    let host_dir = dir.clone();
    let runner = thread::spawn(move || run_host(manifest, &host_dir, &stop_host));
    let socket_path = PathBuf::from(&socket);
    let ready_at = Instant::now() + Duration::from_secs(8);
    while !socket_path.exists() && Instant::now() < ready_at {
        assert!(!runner.is_finished(), "host exited before the command socket existed");
        thread::sleep(Duration::from_millis(20));
    }
    let rejected = cli(&bin, &[&socket, "command", "prepare"]);
    assert!(!rejected.status.success(), "unbound command was queued");
    let stderr = String::from_utf8_lossy(&rejected.stderr);
    assert!(stderr.contains("command capability is not configured"), "{stderr}");
    let rejected_mission = cli(&bin, &[&socket, "mission-file", &mission]);
    assert!(!rejected_mission.status.success());
    assert!(String::from_utf8_lossy(&rejected_mission.stderr).contains("mission capability is not configured"));
    let deadline = Instant::now() + Duration::from_secs(4);
    let mut configured = false;
    while Instant::now() < deadline && !configured {
        for (_key, body) in uplink.samples.lock().unwrap().iter() {
            if body.contains("\"text\":\"Ready\"") {
                panic!("rejected command still moved the controller to Ready: {body}");
            }
            if body.contains("\"text\":\"Configured\"") {
                configured = true;
            }
        }
        thread::sleep(Duration::from_millis(30));
    }
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    assert!(configured, "controller status never reached the heartbeat");
    let _ = std::fs::remove_file(&mission);
}
