//! The `xgc2-module-host` binary: `--check`, start-up, the control socket and signals.

mod common;

use common::*;
use serde_json::Value;
use std::io::{BufRead, BufReader};
use std::process::{Child, Command, Stdio};
use std::time::{Duration, Instant};
use xgc2_xrpc::{BlockingClient, Method, Runtime, RuntimeOptions};

const BIN: &str = env!("CARGO_BIN_EXE_xgc2-module-host");

fn manifest(dir: &std::path::Path, socket: Option<&std::path::Path>) -> std::path::PathBuf {
    let control = socket.map_or(String::new(), |s| format!("[control]\nsocket = \"{}\"\n", s.display()));
    let text = format!(
        "entity = \"cli-test\"\n{control}[host]\nworkers = 2\n[[module]]\nname = \"p\"\npath = \"{}\"\n[[module]]\nname = \"c\"\npath = \"{}\"\n[[instance]]\nname = \"src\"\nmodule = \"p\"\nperiod_ms = 5\n[instance.bind]\nout = \"s\"\n[[instance]]\nname = \"sink\"\nmodule = \"c\"\n[instance.bind]\nstate_in = \"s\"\n",
        module("producer_state").display(),
        module("consumer").display()
    );
    let path = dir.join("entity.toml");
    std::fs::write(&path, text).unwrap();
    path
}

fn private_dir() -> tempfile::TempDir {
    use std::os::unix::fs::PermissionsExt;
    tempfile::Builder::new().permissions(std::fs::Permissions::from_mode(0o700)).tempdir().unwrap()
}

fn wait_exit(child: &mut Child, within: Duration) -> std::process::ExitStatus {
    let deadline = Instant::now() + within;
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            return status;
        }
        assert!(Instant::now() < deadline, "process did not exit within {within:?}");
        sleep_ms(10);
    }
}

#[test]
fn check_validates_without_starting() {
    let dir = private_dir();
    let path = manifest(dir.path(), None);
    let output = Command::new(BIN).arg("--manifest").arg(&path).arg("--check").output().unwrap();
    assert!(output.status.success(), "{}", String::from_utf8_lossy(&output.stderr));
    let report: Value = serde_json::from_slice(&output.stdout).unwrap();
    assert_eq!(report["entity"], "cli-test");
    assert_eq!(report["instances"], serde_json::json!(["src", "sink"]));
    assert_eq!(report["channels"][0]["name"], "s");

    let bad = dir.path().join("bad.toml");
    std::fs::write(&bad, "entity = \"x\"\n[[instance]]\nname = \"i\"\nmodule = \"nope\"\nperiod_ms = 0\n").unwrap();
    let output = Command::new(BIN).arg("--manifest").arg(&bad).arg("--check").output().unwrap();
    assert_eq!(output.status.code(), Some(2));
    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(stderr.contains("module \"nope\" is not declared") && stderr.contains("period_ms"), "{stderr}");
}

#[test]
fn usage_errors_exit_with_two() {
    for args in
        [vec![], vec!["--bogus"], vec!["--manifest"], vec!["--manifest", "/nonexistent/m.toml"], vec!["--workers", "x", "--manifest", "m"]]
    {
        let output = Command::new(BIN).args(&args).output().unwrap();
        assert_eq!(output.status.code(), Some(2), "{args:?}");
        assert!(!output.stderr.is_empty(), "{args:?}");
    }
}

#[test]
fn it_serves_the_control_plane_until_sigterm() {
    let dir = private_dir();
    let socket = dir.path().join("module.sock");
    let path = manifest(dir.path(), Some(&socket));
    let mut child = Command::new(BIN).arg("--manifest").arg(&path).stdout(Stdio::piped()).stderr(Stdio::piped()).spawn().unwrap();
    let mut first = String::new();
    BufReader::new(child.stdout.take().unwrap()).read_line(&mut first).unwrap();
    let announced: Value = serde_json::from_str(&first).unwrap_or_else(|e| panic!("first line {first:?}: {e}"));
    let service = &announced["service_ref"];
    assert_eq!((service["service"].as_str(), service["profile"].as_str()), (Some("xgc2-module"), Some("http.v1")));
    assert_eq!(service["endpoint"]["address"], socket.to_str().unwrap());
    let runtime = Runtime::new(RuntimeOptions::default()).unwrap();
    let id = service["instance_id"].as_str().unwrap();
    let client = BlockingClient::unix(&runtime, &socket, id).unwrap();
    let health = || client.request(Method::GET, "/v1/health", None, Duration::from_secs(5), None).unwrap();
    wait_until("the entity runs", Duration::from_secs(10), || {
        health()["instances"].as_array().is_some_and(|list| list.iter().any(|i| i["name"] == "sink" && count(&i["steps"]) > 5))
    });
    let discovery = BlockingClient::unix(&runtime, &socket, "").unwrap();
    let described = discovery.request(Method::GET, "/v1/describe", None, Duration::from_secs(5), None).unwrap();
    assert_eq!((described["ready"].as_bool(), described["instance_id"].as_str()), (Some(true), Some(id)));
    // SAFETY: signalling our own child.
    unsafe { libc::kill(child.id() as i32, libc::SIGTERM) };
    let status = wait_exit(&mut child, Duration::from_secs(10));
    assert_eq!(status.code(), Some(0));
    assert!(!socket.exists());
    drop(client);
    let _ = runtime;
}

#[test]
fn a_failing_start_exits_without_serving() {
    let dir = private_dir();
    let socket = dir.path().join("module.sock");
    let text = format!(
        "entity = \"x\"\n[control]\nsocket = \"{}\"\n[[module]]\nname = \"m\"\npath = \"/nonexistent/libm.so\"\n",
        socket.display()
    );
    let path = dir.path().join("bad.toml");
    std::fs::write(&path, text).unwrap();
    let output = Command::new(BIN).arg("--manifest").arg(&path).output().unwrap();
    assert_eq!(output.status.code(), Some(2));
    assert!(String::from_utf8_lossy(&output.stderr).contains("libm.so"));
    assert!(!socket.exists());
}
