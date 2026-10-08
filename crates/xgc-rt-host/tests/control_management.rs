//! Real aggregate process + native C module + shared SDK, without a station.
use serde_json::{json, Value};
use std::{
    io::{BufRead, BufReader},
    os::unix::fs::PermissionsExt,
    path::Path,
    process::{Child, Command, Stdio},
    time::{Duration, Instant},
};
use xgc2_xrpc::{BlockingClient, Method, Runtime, RuntimeOptions};

struct OwnedChild(Child);
impl Drop for OwnedChild {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}
fn query(client: &BlockingClient, path: &str) -> Value {
    client
        .request(Method::GET, path, None, Duration::from_secs(2), None)
        .unwrap_or_else(|error|panic!("query {path} failed: {error:?}"))
}
fn wait_state(client: &BlockingClient, wanted: &str) -> Value {
    let until = Instant::now() + Duration::from_secs(5);
    loop {
        let value = query(client, "/v1/health");
        if value["state"] == wanted {
            return value;
        }
        assert!(Instant::now() < until, "wanted {wanted}, observed {value}");
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn native_configuration_revisions_unload_grants_and_nonregular_input() {
    let root = tempfile::tempdir().unwrap();
    std::fs::set_permissions(root.path(), std::fs::Permissions::from_mode(0o700)).unwrap();
    for name in ["modules", "documents", "audit"] {
        std::fs::create_dir(root.path().join(name)).unwrap();
    }
    let modules = root.path().join("modules");
    let documents = root.path().join("documents");
    let audit = root.path().join("audit");
    let source = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../plugins/c-stub/c_stub.c");
    let include = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../abi/include");
    let library = modules.join("module.so");
    let failed_configure = modules.join("fail-configure");
    let unload_entered = modules.join("unload-entered");
    let unload_release = modules.join("unload-release");
    let unload_pause = modules.join("pause-unload");
    let native = std::fs::read_to_string(source)
        .unwrap()
        .replace(
            "#include \"xgc_rt.h\"",
            "#include \"xgc_rt.h\"\n#include <unistd.h>\n#include <stdio.h>",
        )
        .replacen(
            "c_stub* self = p;",
            &format!(
                "c_stub* self = p; if (access({:?}, F_OK) == 0) return XGC_ERR_INVALID;",
                failed_configure.to_str().unwrap()
            ),
            1,
        );
    let native = format!(
        "{native}\n__attribute__((destructor)) static void pause_library_unload(void) {{\n\
        if (access({:?}, F_OK) != 0) return;\n\
        FILE* entered = fopen({:?}, \"w\"); if (entered) fclose(entered);\n\
        for (int i=0; i<300 && access({:?}, F_OK) != 0; ++i) sleep_ms(10);\n\
        }}\n",
        unload_pause.to_str().unwrap(),
        unload_entered.to_str().unwrap(),
        unload_release.to_str().unwrap()
    );
    let source = modules.join("fixture.c");
    std::fs::write(&source, native).unwrap();
    assert!(Command::new("cc")
        .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC", "-I"])
        .arg(include)
        .arg(&source)
        .arg("-o")
        .arg(&library)
        .status()
        .unwrap()
        .success());
    let socket = root.path().join("control.sock");
    let child = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host"))
        .arg("--control-socket")
        .arg(&socket)
        .arg("--module-root")
        .arg(&modules)
        .arg("--document-root")
        .arg(&documents)
        .arg("--audit-root")
        .arg(&audit)
        .env("XGC2_XRPC_HOST_MAX_IN_FLIGHT", "3")
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .unwrap();
    let mut child = OwnedChild(child);
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
    assert_eq!(
        query(&client, "/v1/describe")["service_ref"],
        reference["service_ref"]
    );
    let policy = query(&client, "/v1/policy");
    assert_eq!(policy["fields"]["HOST_MAX_IN_FLIGHT"]["value"], 3);
    assert_eq!(
        policy["fields"]["HOST_MAX_IN_FLIGHT"]["source"],
        "environment"
    );
    let manifest = format!(
        r#"[session]
id="module-control-fixture"
node="n1"
roster=["n1"]
period_ms=10
start_delay_ms=10
[transport]
kind="loopback"
[audit]
dir={:?}
[[channel]]
name="command"
qos="event"
[[plugin]]
name="fixture"
path={:?}
expected_name="c-stub"
trigger="on_round"
config={{fail_after=0}}
bind={{cmd={{channel="command",from=["n1"]}}}}
"#,
        audit.to_str().unwrap(),
        library.to_str().unwrap()
    );
    let input = json!({"manifest_toml":manifest,"base_dir":documents});
    let loaded = client
        .call("/v1/load", input.clone(), Duration::from_secs(2))
        .unwrap();
    assert_eq!(loaded["state"], "loaded");
    assert!(loaded["configuration"]["applied_revision"].is_null());
    let revision = loaded["event_revision"].as_u64().unwrap();
    assert!(client
        .request(
            Method::GET,
            &format!("/v1/observe/{revision}"),
            None,
            Duration::from_millis(50),
            None
        )
        .is_err());
    assert!(client
        .call("/v1/load", input.clone(), Duration::from_secs(2))
        .is_err());
    assert!(client
        .call(
            "/v1/configure",
            json!({"expected_revision":0,"persist":false,"modules":{"fixture":{}}}),
            Duration::from_secs(2)
        )
        .is_err());
    assert!(client
        .call(
            "/v1/configure",
            json!({"expected_revision":1,"persist":true,"modules":{"fixture":{}}}),
            Duration::from_secs(2)
        )
        .is_err());
    assert!(client
        .call(
            "/v1/configure",
            json!({"expected_revision":1,"persist":false,"modules":{"missing":{}}}),
            Duration::from_secs(2)
        )
        .is_err());
    let desired = client
        .call(
            "/v1/configure",
            json!({"expected_revision":1,"persist":false,"modules":{"fixture":{"fail_after":0}}}),
            Duration::from_secs(2),
        )
        .unwrap();
    assert_eq!(desired["desired_revision"], 2);
    assert!(desired["applied_revision"].is_null());
    assert!(client.call("/v1/configure", json!({
        "expected_revision":2,"persist":false,"modules":{"fixture":{"large":"\t".repeat(40000)}}
    }),Duration::from_secs(2)).is_err());
    assert_eq!(query(&client, "/v1/configuration")["desired_revision"], 2);
    let observed = query(&client, &format!("/v1/observe/{revision}"));
    assert!(observed["event_revision"].as_u64().unwrap() > revision);
    client
        .call("/v1/start", json!({}), Duration::from_secs(2))
        .unwrap();
    let until = Instant::now() + Duration::from_secs(4);
    loop {
        let health = query(&client, "/v1/health");
        if health["configuration"]["applied_revision"] == 2
            && health["live"]["modules"][0]["state"] == "active"
        {
            break;
        }
        assert!(
            Instant::now() < until,
            "native configuration/active evidence absent: {health}"
        );
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(client
        .call(
            "/v1/configure",
            json!({"expected_revision":2,"persist":false,"modules":{"fixture":{}}}),
            Duration::from_secs(2)
        )
        .is_err());
    client
        .call("/v1/stop", json!({}), Duration::from_secs(2))
        .unwrap();
    let stopped = wait_state(&client, "stopped");
    assert!(stopped["live"].is_null());
    assert_eq!(stopped["configuration"]["applied_revision"], 2);
    assert!(stopped["summary"]["plugins"][0]["steps"].as_u64().unwrap() > 0);
    let first_audit = stopped["summary"]["audit_dir"].as_str().unwrap().to_owned();
    client
        .call("/v1/start", json!({}), Duration::from_secs(2))
        .unwrap();
    client
        .call("/v1/stop", json!({}), Duration::from_secs(2))
        .unwrap();
    let next = wait_state(&client, "stopped");
    assert_ne!(first_audit, next["summary"]["audit_dir"].as_str().unwrap());
    assert!(Path::new(&first_audit).join("n1/meta.json").is_file());
    // This is the same desired revision, but a new native configure attempt.
    // Previous success must not be used as evidence after this one fails.
    std::fs::write(&failed_configure, b"fail").unwrap();
    client
        .call("/v1/start", json!({}), Duration::from_secs(2))
        .unwrap();
    let until = Instant::now() + Duration::from_secs(3);
    loop {
        let value = query(&client, "/v1/health");
        if value["configuration"]["evidence"]["applied"] == false {
            assert!(
                value["configuration"]["applied_revision"].is_null(),
                "{value}"
            );
            break;
        }
        assert!(
            Instant::now() < until,
            "native configure failure absent: {value}"
        );
        std::thread::sleep(Duration::from_millis(10));
    }
    client
        .call("/v1/stop", json!({}), Duration::from_secs(2))
        .unwrap();
    wait_state(&client, "error");
    std::fs::remove_file(&failed_configure).unwrap();
    client
        .call(
            "/v1/unload",
            json!({"expected_revision":2}),
            Duration::from_secs(2),
        )
        .unwrap();
    assert_eq!(query(&client, "/v1/health")["state"], "empty");
    assert!(query(&client, "/v1/configuration")["applied_revision"].is_null());
    let fifo = documents.join("input.toml");
    let name = std::ffi::CString::new(fifo.to_str().unwrap()).unwrap();
    assert_eq!(unsafe { libc::mkfifo(name.as_ptr(), 0o600) }, 0);
    assert!(client
        .call(
            "/v1/load",
            json!({"manifest_path":fifo}),
            Duration::from_secs(1)
        )
        .is_err());
    assert_eq!(query(&client, "/v1/health")["state"], "empty");
    let file_manifest = documents.join("large-config.toml");
    let large = input["manifest_toml"].as_str().unwrap().replace(
        "config={fail_after=0}",
        &format!("config={{literal='{}'}}", "\t".repeat(600000)),
    );
    assert!(large.len() < 1 << 20);
    xgc_rt_core::manifest::Manifest::from_toml_str(&large).unwrap();
    std::fs::write(&file_manifest, large).unwrap();
    let generation = query(&client, "/v1/health")["generation"].clone();
    assert!(client
        .call(
            "/v1/load",
            json!({"manifest_path":file_manifest}),
            Duration::from_secs(2)
        )
        .is_err());
    assert_eq!(query(&client, "/v1/health")["generation"], generation);
    let failed = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host"))
        .arg("--manifest")
        .arg(&fifo)
        .output()
        .unwrap();
    assert!(!failed.status.success());
    assert!(String::from_utf8_lossy(&failed.stderr).contains("regular"));
    for flag in [
        "--control-socket",
        "--module-root",
        "--document-root",
        "--audit-root",
    ] {
        let failed = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host"))
            .arg("--manifest")
            .arg(&fifo)
            .arg(flag)
            .output()
            .unwrap();
        assert!(!failed.status.success());
        assert!(String::from_utf8_lossy(&failed.stderr).contains("requires a value"));
    }
    let mut outside = input.clone();
    outside["base_dir"] = json!(modules);
    assert!(client
        .call("/v1/load", outside, Duration::from_secs(1))
        .is_err());
    // A real dlclose destructor can wait. It must run outside the state query
    // lock so SDK IO and health remain responsive while unload owns the work.
    let loaded = client
        .call("/v1/load", input.clone(), Duration::from_secs(2))
        .unwrap();
    let generation = loaded["generation"].as_u64().unwrap();
    assert!(loaded["configuration"]["applied_revision"].is_null());
    std::fs::write(&unload_pause, b"pause").unwrap();
    let unloading = BlockingClient::unix(
        &runtime,
        &socket,
        reference["service_ref"]["instance_id"].as_str().unwrap(),
    )
    .unwrap();
    let worker = std::thread::spawn(move || {
        unloading.call(
            "/v1/unload",
            json!({"expected_revision":generation}),
            Duration::from_secs(4),
        )
    });
    let until = Instant::now() + Duration::from_secs(2);
    while !unload_entered.exists() {
        assert!(Instant::now() < until, "native unload did not enter");
        std::thread::sleep(Duration::from_millis(10));
    }
    let started = Instant::now();
    let pending = query(&client, "/v1/health");
    assert_eq!(pending["state"], "loading");
    assert!(started.elapsed() < Duration::from_millis(500));
    assert!(
        !worker.is_finished(),
        "native destructor work was released early"
    );
    std::fs::write(&unload_release, b"release").unwrap();
    assert_eq!(worker.join().unwrap().unwrap()["state"], "empty");
    assert_eq!(unsafe { libc::kill(child.0.id() as i32, libc::SIGTERM) }, 0);
    let until = Instant::now() + Duration::from_secs(3);
    let status = loop {
        if let Some(status) = child.0.try_wait().unwrap() {
            break status;
        }
        assert!(Instant::now() < until);
        std::thread::sleep(Duration::from_millis(20));
    };
    assert!(status.success());
    assert!(!socket.exists());
    drop(client);
    let failed = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host"))
        .arg("--control-socket")
        .arg(&socket)
        .arg("--module-root")
        .arg(&modules)
        .arg("--document-root")
        .arg(&documents)
        .arg("--audit-root")
        .arg(&audit)
        .env("XGC2_XRPC_HOST_MAX_IN_FLIGHT", "5")
        .output()
        .unwrap();
    assert!(!failed.status.success());
    assert!(String::from_utf8_lossy(&failed.stderr).contains("HOST_MAX_IN_FLIGHT"));
    assert!(!socket.exists());
    let child = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host"))
        .arg("--control-socket")
        .arg(&socket)
        .arg("--module-root")
        .arg(&modules)
        .arg("--document-root")
        .arg(&documents)
        .arg("--audit-root")
        .arg(&audit)
        .env("XGC2_XRPC_HOST_MAX_IN_FLIGHT", "1")
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .unwrap();
    let mut child = OwnedChild(child);
    let mut output = BufReader::new(child.0.stdout.take().unwrap());
    line.clear();
    output.read_line(&mut line).unwrap();
    let reference: Value = serde_json::from_str(&line).unwrap();
    let client = BlockingClient::unix(
        &runtime,
        &socket,
        reference["service_ref"]["instance_id"].as_str().unwrap(),
    )
    .unwrap();
    assert_eq!(query(&client, "/v1/describe")["held_observers_limit"], 0);
    let started = Instant::now();
    assert!(client
        .request(
            Method::GET,
            "/v1/observe/0",
            None,
            Duration::from_secs(2),
            None
        )
        .is_err());
    assert!(started.elapsed() < Duration::from_millis(500));
    assert_eq!(
        client
            .call("/v1/load", input, Duration::from_secs(2))
            .unwrap()["state"],
        "loaded"
    );
    std::fs::write(&unload_release, b"release").unwrap();
    assert_eq!(unsafe { libc::kill(child.0.id() as i32, libc::SIGTERM) }, 0);
    let until = Instant::now() + Duration::from_secs(3);
    loop {
        if let Some(status) = child.0.try_wait().unwrap() {
            assert!(status.success());
            break;
        }
        assert!(Instant::now() < until);
        std::thread::sleep(Duration::from_millis(10));
    }
}
