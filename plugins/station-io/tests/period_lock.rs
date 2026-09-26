//! 1 ms period and no step_budget_ms: hang limit is period * HANG_FACTOR = 10 ms.
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};
use zenoh::Wait;

fn elf(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/debug").join(name)
}

fn peer(endpoint: &str) -> zenoh::Session {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").unwrap();
    config.insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]")).unwrap();
    config.insert_json5("scouting/multicast/enabled", "false").unwrap();
    config.insert_json5("scouting/gossip/enabled", "false").unwrap();
    zenoh::open(config).wait().unwrap()
}

fn plugin_line(name: &str, path: &Path, config: &str, bind: &str) -> String {
    let sha = xgc_rt_host::plugin::sha256_hex(&std::fs::read(path).unwrap());
    format!(
        "\n[[plugin]]\nname={name:?}\npath={:?}\nsha256={sha:?}\ntrigger=\"on_round\"\nconfig={config}\nbind={bind}\n",
        path.to_str().unwrap()
    )
}

fn run(manifest: String, dir: &Path, stop: &AtomicBool) -> xgc_rt_host::RunSummary {
    std::fs::create_dir_all(dir).unwrap();
    let host = Host::new(
        Manifest::from_toml_str(&manifest).unwrap(),
        dir,
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
        Arc::new(WallClock::new(0)),
        HostOptions::default(),
    )
    .unwrap_or_else(|e| panic!("{e}\n{manifest}"));
    host.run(stop).unwrap()
}

fn station_manifest(robot: &str, endpoint: &str, socket: &str, station: &Path, plant: &Path, command: bool) -> String {
    let mut text = r#"[session]
id="station-10ms"
node="n1"
roster=["n1"]
period_ms=1
start_delay_ms=40
run_for_ms=20000
[transport]
kind="loopback"
[audit]
dir="audit"
[[channel]]
name="paired"
qos="state"
[[channel]]
name="status"
qos="state"
[[channel]]
name="command"
qos="event"
[[channel]]
name="pva"
qos="control"
"#
    .to_string();
    let config = format!(
        "{{robot_id={robot:?},zenoh_connect={endpoint:?},command_socket={socket:?},authority=true,command={command},mission=false}}"
    );
    text += &plugin_line(
        "station-io",
        station,
        &config,
        r#"{paired_state={channel="paired",from=["n1"]},controller_status={channel="status",from=["n1"]},command={channel="command"}}"#,
    );
    text += &plugin_line("paired-source", plant, "{}", r#"{paired_state={channel="paired"}}"#);
    text
}

#[test]
fn one_host_at_one_millisecond_answers_prepare_without_abandon() {
    let station = elf("libstation_io.so");
    let vehicle = elf("libnumeric_vehicle.so");
    let bin = elf("station-io-cmd");
    assert!(station.is_file() && vehicle.is_file() && bin.is_file());
    let port = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let endpoint = format!("tcp/127.0.0.1:{port}");
    let _peer = peer(&endpoint);
    let socket = format!("/tmp/xgc-sio-1ms-{}.sock", std::process::id());
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join("station-1ms");
    let mut text = format!(
        r#"[session]
id="station-1ms"
node="n1"
roster=["n1"]
period_ms=1
start_delay_ms=40
run_for_ms=2000
[transport]
kind="loopback"
[audit]
dir="audit"
[[channel]]
name="paired"
qos="state"
[[channel]]
name="status"
qos="state"
[[channel]]
name="command"
qos="event"
[[channel]]
name="pva"
qos="control"
"#
    );
    let config = format!(
        "{{robot_id=\"xgc2e-0123456789abcdef0001\",zenoh_connect={endpoint:?},command_socket={socket:?},authority=true,command=true,mission=false}}"
    );
    text += &plugin_line(
        "station-io",
        &station,
        &config,
        r#"{paired_state={channel="paired",from=["n1"]},controller_status={channel="status",from=["n1"]},command={channel="command"}}"#,
    );
    text += &plugin_line(
        "numeric-vehicle",
        &vehicle,
        "{initial_position=[1.0,0.0,0.0],initial_velocity=[0.2,0.0,0.0]}",
        r#"{command={channel="command",from=["n1"]},position_target={channel="pva",from=["n1"]},paired_state={channel="paired"},controller_state={channel="status"}}"#,
    );
    let stop = Arc::new(AtomicBool::new(false));
    let stop_host = Arc::clone(&stop);
    let manifest = text;
    let host_dir = dir.clone();
    let runner = thread::spawn(move || run(manifest, &host_dir, &stop_host));
    let socket_path = PathBuf::from(&socket);
    let ready = Instant::now() + Duration::from_secs(8);
    while !socket_path.exists() && Instant::now() < ready {
        assert!(!runner.is_finished(), "host exited before the socket");
        thread::sleep(Duration::from_millis(5));
    }
    let output = Command::new(&bin).args([&socket, "command", "prepare"]).output().unwrap();
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(!stderr.contains("os error 11"), "{stderr}");
    assert!(output.status.success(), "{stderr}");
    assert_eq!(output.stdout, b"queued\n");
    thread::sleep(Duration::from_millis(400));
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    let station_summary = summary.plugins.iter().find(|p| p.name == "station-io").unwrap();
    assert_eq!(station_summary.abandons, 0, "{station_summary:?}");
    assert!(station_summary.steps > 200, "steps {}", station_summary.steps);
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
}

#[test]
fn eight_host_child() {
    let Ok(index) = std::env::var("STATION_EIGHT_CHILD") else { return };
    let index: usize = index.parse().unwrap();
    let endpoint = std::env::var("STATION_EIGHT_ENDPOINT").unwrap();
    let socket = std::env::var("STATION_EIGHT_SOCKET").unwrap();
    let dir = PathBuf::from(std::env::var("STATION_EIGHT_DIR").unwrap());
    let station = PathBuf::from(std::env::var("STATION_EIGHT_STATION").unwrap());
    let plant = PathBuf::from(std::env::var("STATION_EIGHT_PLANT").unwrap());
    let robot = format!("xgc2e-{index:020x}");
    let manifest = station_manifest(&robot, &endpoint, &socket, &station, &plant, false);
    let stop = Arc::new(AtomicBool::new(false));
    let stop_host = Arc::clone(&stop);
    let flag = dir.join("stop");
    thread::spawn(move || {
        while !flag.exists() {
            thread::sleep(Duration::from_millis(5));
        }
        stop_host.store(true, Ordering::Relaxed);
    });
    let summary = run(manifest, &dir, &stop);
    let station_summary = summary.plugins.iter().find(|p| p.name == "station-io").unwrap();
    let text = format!(
        "steps={} abandons={} aborted={:?} err={:?}",
        station_summary.steps, station_summary.abandons, summary.aborted, station_summary.last_error
    );
    std::fs::write(dir.join("result.txt"), &text).unwrap();
    assert_eq!(station_summary.abandons, 0, "{text}");
    assert!(station_summary.steps > 200, "{text}");
    assert!(summary.aborted.is_none(), "{text}");
}

#[test]
fn eight_hosts_keep_stepping_inside_ten_milliseconds() {
    let station = elf("libstation_io.so");
    let bin = elf("station-io-cmd");
    let plant_src = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/paired_source.c");
    let plant = Path::new(env!("CARGO_TARGET_TMPDIR")).join("paired-source.so");
    let include = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../abi/include");
    assert!(Command::new("cc")
        .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-I"])
        .arg(&include)
        .arg(&plant_src)
        .arg("-o")
        .arg(&plant)
        .status()
        .unwrap()
        .success());
    let port = std::net::TcpListener::bind("127.0.0.1:0").unwrap().local_addr().unwrap().port();
    let endpoint = format!("tcp/127.0.0.1:{port}");
    let _peer = peer(&endpoint);
    let mut children = Vec::new();
    for index in 0..8 {
        let socket = format!("/tmp/xgc-sio-8p-{index}-{}.sock", std::process::id());
        let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("station-8p-{index}"));
        std::fs::create_dir_all(&dir).unwrap();
        let _ = std::fs::remove_file(dir.join("stop"));
        let child = Command::new(std::env::current_exe().unwrap())
            .args(["--exact", "eight_host_child", "--test-threads=1", "--nocapture"])
            .env("STATION_EIGHT_CHILD", index.to_string())
            .env("STATION_EIGHT_ENDPOINT", &endpoint)
            .env("STATION_EIGHT_SOCKET", &socket)
            .env("STATION_EIGHT_DIR", &dir)
            .env("STATION_EIGHT_STATION", &station)
            .env("STATION_EIGHT_PLANT", &plant)
            .stdout(std::process::Stdio::piped())
            .stderr(std::process::Stdio::piped())
            .spawn()
            .unwrap();
        children.push((index, socket, dir, Some(child)));
    }
    let mut failures = Vec::new();
    for (index, socket, _, child) in &mut children {
        let Some(child) = child.as_mut() else { continue };
        let deadline = Instant::now() + Duration::from_secs(8);
        while !Path::new(&*socket).exists() && Instant::now() < deadline {
            if let Some(status) = child.try_wait().unwrap() {
                failures.push(format!("host {index} exited before activate: {status}"));
                break;
            }
            thread::sleep(Duration::from_millis(5));
        }
        if !Path::new(&*socket).exists() {
            failures.push(format!("host {index} did not activate"));
        }
    }
    let mut replies = Vec::new();
    if failures.is_empty() {
        for (_, socket, _, _) in &children {
            let output = Command::new(&bin).args([socket, "command", "prepare"]).output().unwrap();
            replies.push((socket.clone(), output.status.success(), String::from_utf8_lossy(&output.stderr).into_owned()));
        }
        thread::sleep(Duration::from_millis(300));
    }
    for (_, _, dir, _) in &children {
        let _ = std::fs::write(dir.join("stop"), b"1");
    }
    for (index, _, dir, child) in &mut children {
        let Some(child) = child.take() else { continue };
        let output = child.wait_with_output().unwrap();
        let result = std::fs::read_to_string(dir.join("result.txt")).unwrap_or_default();
        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            failures.push(format!("host {index} status={} result={result} stderr={stderr}", output.status));
        }
    }
    assert!(failures.is_empty(), "{}", failures.join("\n"));
    for (socket, ok, stderr) in replies {
        assert!(!stderr.contains("os error 11"), "{socket} {stderr}");
        assert!(!ok && stderr.contains("command capability is not configured"), "{socket} {stderr}");
    }
}
