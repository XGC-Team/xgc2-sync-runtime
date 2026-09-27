//! These are deployment boundary tests. Tiny local ELF files stand in for
//! bundle payloads: passing does not assert native module/ROS readiness.
use serde_json::{json, Value};
use std::fs;
use std::io::{BufRead, BufReader};
use std::os::unix::fs::{symlink, PermissionsExt};
use std::path::{Path, PathBuf};
use std::process::{Command, Stdio};
use std::sync::atomic::{AtomicUsize, Ordering};
use xgc_rt_host::deployment::*;

struct Case {
    root: PathBuf,
    bundle: PathBuf,
    state: PathBuf,
    input: Deployment,
}
impl Drop for Case {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.root);
    }
}
impl Case {
    fn new() -> Self {
        static NEXT: AtomicUsize = AtomicUsize::new(0);
        let root = std::env::temp_dir().join(format!(
            "xgc-deploy-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        fs::create_dir(&root).unwrap();
        let bundle = root.join("bundle");
        for dir in ["bin", "plugins", "lib"] {
            fs::create_dir_all(bundle.join(dir)).unwrap();
        }
        let actual_elf = fs::read("/bin/true").unwrap();
        let pin = |path: &str| {
            fs::write(bundle.join(path), &actual_elf).unwrap();
            BundleFile {
                path: path.into(),
                sha256: sha256(&actual_elf),
            }
        };
        let descriptor = Bundle {
            schema_version: 1,
            platform: target_platform().unwrap().into(),
            composition_sha256: composition_sha256(),
            host: pin("bin/xgc-rt-host"),
            plugins: Plugins {
                ros_io: pin("plugins/ros.so"),
                rigid_state: Some(pin("plugins/rigid.so")),
                hover_thrust: Some(pin("plugins/hover.so")),
                controller: Some(pin("plugins/ctl.so")),
                reference: Some(pin("plugins/ref.so")),
                numeric_vehicle: None, plan_dmpc: None, dmpc_rounds: None, station_io: None,
            },
            libraries: vec![pin("lib/actual.so.1")],
            links: vec![BundleLink {
                path: "lib/actual.so".into(),
                target: "actual.so.1".into(),
            }],
        };
        symlink("actual.so.1", bundle.join("lib/actual.so")).unwrap();
        let bytes = serde_json::to_vec(&descriptor).unwrap();
        fs::write(bundle.join(BUNDLE_FILE), &bytes).unwrap();
        let configuration = configuration();
        let configuration_json = serde_json::to_string(&configuration).unwrap();
        let input = Deployment {
            schema_version: 1,
            session_id: "experiment-52".into(),
            node_id: "robot-208".into(),
            robot_namespace: "uav2".into(),
            platform: target_platform().unwrap().into(),
            bundle_sha256: sha256(&bytes),
            composition_id: COMPOSITION_ID.into(),
            composition_sha256: composition_sha256(),
            configuration_sha256: sha256(configuration_json.as_bytes()),
            configuration_json,
        };
        Self {
            state: root.join("state"),
            root,
            bundle,
            input,
        }
    }
    fn raw(&self) -> String {
        serde_json::to_string(&self.input).unwrap()
    }
    fn prepare(&self) -> Result<Prepared> {
        prepare(&self.raw(), &self.bundle, &self.state)
    }
    fn config(&mut self, f: impl FnOnce(&mut Value)) {
        let mut value: Value = serde_json::from_str(&self.input.configuration_json).unwrap();
        f(&mut value);
        self.input.configuration_json = serde_json::to_string(&value).unwrap();
        self.input.configuration_sha256 = sha256(self.input.configuration_json.as_bytes());
    }
    fn descriptor(&mut self, f: impl FnOnce(&mut Bundle)) {
        let mut value: Bundle =
            serde_json::from_slice(&fs::read(self.bundle.join(BUNDLE_FILE)).unwrap()).unwrap();
        f(&mut value);
        let bytes = serde_json::to_vec(&value).unwrap();
        self.input.bundle_sha256 = sha256(&bytes);
        fs::write(self.bundle.join(BUNDLE_FILE), bytes).unwrap();
    }
    fn select_composition(&mut self, id: &str) {
        self.input.composition_id = id.into();
        self.input.composition_sha256 = composition(id).unwrap().sha256();
        let digest = self.input.composition_sha256.clone();
        self.descriptor(|b| b.composition_sha256 = digest);
    }
    fn rejected(&self, needle: &str) {
        match self.prepare() {
            Err(error) => assert!(error.contains(needle), "expected {needle}: {error}"),
            Ok(_) => panic!("accepted invalid deployment: {needle}"),
        }
    }
}
fn configuration() -> Value {
    json!({
        "input_time_domain":"wall-unix","ros_master_uri":"http://172.30.251.251:11311","ros_ip":"172.30.251.102","takeoff_altitude_m":2.3,
        "topics":{
            "imu_topic":"/uav2/mavros/imu/data_raw","pose_topic":"/mocap/tracked_robot/pose","vision_pose_topic":"/uav2/mavros/vision_pose/pose",
            "rigid_state_estimate_topic":"/uav2/alg/state_estimator/state","fcu_state_topic":"/uav2/mavros/state","local_pose_topic":"/uav2/mavros/local_position/pose",
            "local_velocity_topic":"/uav2/mavros/local_position/velocity_local","fcu_imu_topic":"/uav2/mavros/imu/data","battery_topic":"/uav2/mavros/battery",
            "command_topic":"/experiment/command","alg_setpoint_topic":"/uav2/alg/setpoint_raw/local","attitude_target_topic":"/uav2/mavros/setpoint_raw/target_attitude",
            "setpoint_topic":"/uav2/mavros/setpoint_raw/local","attitude_rate_topic":"/uav2/mavros/setpoint_raw/attitude","status_topic":"/uav2/custom/statustext","fcu_request_topic":"/uav2/mavros"
        },
        "calibration":{"verified":false,"field_offset_xyz":[0.1,-0.2,0.3],"field_offset_rpy":[0,0,0.25],"imu_to_vrpn_marker_xyz":[0.02,0.01,-0.01],"imu_to_vrpn_marker_rpy":[0,0,0],
        "provenance":{"kind":"unverified","source_id":"robot-config-52","source_sha256":sha256(b"unverified calibration record"),"robot_asset_id":"asset-208"}}
    })
}
fn manifest(p: &Prepared) -> toml::Value {
    fs::read_to_string(&p.receipt.manifest_path)
        .unwrap()
        .parse()
        .unwrap()
}
fn plugin<'a>(m: &'a toml::Value, name: &str) -> &'a toml::Value {
    m["plugin"]
        .as_array()
        .unwrap()
        .iter()
        .find(|p| p["name"].as_str() == Some(name))
        .unwrap()
}

#[test]
fn actual_identity_roles_transforms_and_timing_reach_resolvable_manifest() {
    let case = Case::new();
    let p = case.prepare().unwrap();
    let m = manifest(&p);
    assert_eq!(m["session"]["id"].as_str(), Some("experiment-52"));
    assert_eq!(m["session"]["node"].as_str(), Some("robot-208"));
    assert_eq!(
        m["session"]["roster"].as_array().unwrap(),
        &vec![toml::Value::from("robot-208")]
    );
    assert!(m["session"].get("run_for_ms").is_none());
    assert_eq!(m["session"]["period_ms"].as_integer(), Some(1));
    assert_eq!(m["transport"]["kind"].as_str(), Some("loopback"));
    assert!(m.get("clock").is_none());
    assert_eq!(m["plugin"].as_array().unwrap().len(), 5);
    let topics = configuration()["topics"].clone();
    let cfg = &plugin(&m, "ros_io")["config"];
    for (role, value) in topics.as_object().unwrap() {
        assert_eq!(cfg[role].as_str(), value.as_str(), "role {role}");
    }
    let rigid = &plugin(&m, "rigid-state")["config"];
    assert_eq!(rigid["extrinsic_verified"].as_bool(), Some(false));
    assert_eq!(rigid["field_offset_xyz"][1].as_float(), Some(-0.2));
    assert_eq!(
        plugin(&m, "ctl-px4")["config"]["takeoff_altitude"].as_float(),
        Some(2.3)
    );
    assert_eq!(
        plugin(&m, "ctl-px4")["step_budget_ms"].as_integer(),
        Some(10)
    );
    for module in m["plugin"].as_array().unwrap() {
        for (_, bind) in module["bind"].as_table().unwrap() {
            if let Some(origins) = bind.get("from") {
                assert_eq!(
                    origins.as_array().unwrap(),
                    &vec![toml::Value::from("robot-208")]
                );
            }
        }
        assert_eq!(module["sha256"].as_str().unwrap().len(), 64);
    }
    assert!(!p.receipt.live_readiness);
    assert!(p.receipt.audit_path.is_dir());
    assert_eq!(
        p.receipt.manifest_sha256,
        sha256(&fs::read(&p.receipt.manifest_path).unwrap())
    );
    assert_eq!(
        fs::metadata(p.receipt.manifest_path.parent().unwrap())
            .unwrap()
            .permissions()
            .mode()
            & 0o777,
        0o700
    );
    assert_eq!(
        fs::metadata(&p.receipt.manifest_path)
            .unwrap()
            .permissions()
            .mode()
            & 0o777,
        0o600
    );
}
#[test]
fn strict_envelope_and_nested_schema_and_original_json_hash() {
    let mut c = Case::new();
    let raw = c.raw();
    assert!(Deployment::parse(&raw.replacen('{', "{\"unexpected\":1,", 1)).is_err());
    assert!(Deployment::parse(&raw.replacen('{', "{\"schema_version\":1,", 1)).is_err());
    c.config(|v| v["calibration"]["unexpected"] = json!(1));
    c.rejected("configuration schema");
    let mut c = Case::new();
    c.input.configuration_json =
        c.input
            .configuration_json
            .replacen('{', "{\"input_time_domain\":\"wall-unix\",", 1);
    c.input.configuration_sha256 = sha256(c.input.configuration_json.as_bytes());
    c.rejected("duplicate field");
    let mut c = Case::new();
    c.input.configuration_json.push(' ');
    c.rejected("bytes/digest mismatch");
    c.input.configuration_sha256 = sha256(c.input.configuration_json.as_bytes());
    assert!(c.prepare().is_ok());
    let mut c = Case::new();
    c.config(|v| {
        v.as_object_mut()
            .unwrap()
            .remove("calibration")
            .map(|_| ())
            .unwrap()
    });
    c.rejected("missing field");
}
#[test]
fn unsupported_composition_platform_simtime_and_noncanonical_names_rejected() {
    for (field, value) in [
        ("composition_id", "dmpc"),
        ("platform", if target_platform().unwrap() == "linux-amd64" { "linux-arm64" } else { "linux-amd64" }),
        ("composition_sha256", &"0".repeat(64)),
        ("session_id", "../escape"),
        ("node_id", "bad/node"),
        ("robot_namespace", "/uav2"),
        ("robot_namespace", "uav2/./child"),
        ("robot_namespace", "uav2//child"),
    ] {
        let c = Case::new();
        let mut v = serde_json::to_value(&c.input).unwrap();
        v[field] = json!(value);
        assert!(
            Deployment::parse(&v.to_string()).is_err(),
            "{field}={value}"
        );
    }
    let mut c = Case::new();
    c.config(|v| v["input_time_domain"] = json!("gazebo-sim"));
    c.rejected("configuration schema");
}
#[test]
fn topic_ownership_aliasing_and_endpoint_validation() {
    for value in [
        "relative/pose",
        "/fleet//pose",
        "/fleet/../pose",
        "/fleet/~pose",
    ] {
        let mut c = Case::new();
        c.config(|v| v["topics"]["pose_topic"] = json!(value));
        c.rejected("invalid absolute ROS name");
    }
    let mut c = Case::new();
    c.config(|v| v["topics"]["setpoint_topic"] = json!("/uav1/mavros/setpoint"));
    c.rejected("outside the selected robot namespace");
    let mut c = Case::new();
    c.config(|v| v["topics"]["pose_topic"] = v["topics"]["command_topic"].clone());
    c.rejected("distinct names");
    for value in [
        "http://localhost:11311",
        "http://127.0.0.1:0",
        "http://127.0.0.1:11311/",
        "https://127.0.0.1:11311",
        "http://0.0.0.0:11311",
    ] {
        let mut c = Case::new();
        c.config(|v| v["ros_master_uri"] = json!(value));
        assert!(c.prepare().is_err(), "{value}");
    }
    for value in ["0.0.0.0", "224.0.0.1", "255.255.255.255", "hostname"] {
        let mut c = Case::new();
        c.config(|v| v["ros_ip"] = json!(value));
        c.rejected("unicast IPv4");
    }
}
#[test]
fn calibration_claim_requires_matching_explicit_provenance() {
    let mut c = Case::new();
    c.config(|v| v["calibration"]["verified"] = json!(true));
    c.rejected("verification/provenance disagreement");
    c.config(|v| v["calibration"]["provenance"]["kind"] = json!("simulation-model"));
    let p = c.prepare().unwrap();
    assert_eq!(
        plugin(&manifest(&p), "rigid-state")["config"]["extrinsic_verified"].as_bool(),
        Some(true)
    );
    drop(p);
    let mut c = Case::new();
    c.config(|v| v["calibration"]["provenance"]["source_sha256"] = json!(""));
    c.rejected("explicit calibration provenance");
    let mut c = Case::new();
    c.config(|v| v["calibration"]["field_offset_xyz"] = json!([1, 2]));
    c.rejected("configuration schema");
    let mut c = Case::new();
    c.config(|v| v["takeoff_altitude_m"] = json!(-1));
    c.rejected("finite and positive");
}
#[test]
fn foreign_elf_is_rejected_even_with_matching_file_digest() {
    let mut c = Case::new();
    let path = c.bundle.join("plugins/ctl.so");
    let mut bytes = fs::read(&path).unwrap();
    let foreign: u16 = if target_elf_machine().unwrap() == 62 { 183 } else { 62 };
    bytes[18..20].copy_from_slice(&foreign.to_le_bytes());
    fs::write(path, &bytes).unwrap();
    c.descriptor(|b| b.plugins.controller.as_mut().unwrap().sha256 = sha256(&bytes));
    c.rejected("plugins/ctl.so is not little-endian ELF64");
    assert!(!c.state.exists());
}

#[test]
fn artifacts_and_descriptor_bytes_are_verified_before_state_creation() {
    let c = Case::new();
    fs::write(c.bundle.join("plugins/ctl.so"), b"wrong").unwrap();
    c.rejected("short ELF");
    assert!(!c.state.exists());
    let c = Case::new();
    let path = c.bundle.join("plugins/ctl.so");
    let mut bytes = fs::read(&path).unwrap();
    let n = bytes.len() - 1;
    bytes[n] ^= 1;
    fs::write(path, bytes).unwrap();
    c.rejected("artifact hash mismatch");
    assert!(!c.state.exists());
    let c = Case::new();
    fs::write(c.bundle.join(BUNDLE_FILE), b"{}").unwrap();
    c.rejected("descriptor bytes/digest mismatch");
    let mut c = Case::new();
    c.descriptor(|b| b.libraries.push(b.plugins.controller.clone().unwrap()));
    c.rejected("duplicate or unsafe bundle path");
    let mut c = Case::new();
    c.descriptor(|b| b.plugins.controller.as_mut().unwrap().path = "plugins/./ctl.so".into());
    c.rejected("unsafe bundle path");
}
#[test]
fn symlinked_state_and_artifacts_and_link_chains_are_refused() {
    let c = Case::new();
    let outside = c.root.join("outside");
    fs::create_dir(&outside).unwrap();
    symlink(&outside, &c.state).unwrap();
    c.rejected("without symlinks");
    assert_eq!(fs::read_dir(outside).unwrap().count(), 0);
    let c = Case::new();
    fs::create_dir_all(c.state.join(&c.input.session_id)).unwrap();
    symlink(
        &c.bundle,
        c.state.join(&c.input.session_id).join(&c.input.node_id),
    )
    .unwrap();
    c.rejected("without symlinks");
    let c = Case::new();
    fs::remove_file(c.bundle.join("plugins/ctl.so")).unwrap();
    symlink("ros.so", c.bundle.join("plugins/ctl.so")).unwrap();
    c.rejected("without symlinks");
    let mut c = Case::new();
    symlink("actual.so", c.bundle.join("lib/alias.so")).unwrap();
    c.descriptor(|b| {
        b.links.push(BundleLink {
            path: "lib/alias.so".into(),
            target: "actual.so".into(),
        })
    });
    c.rejected("directly to a distinct indexed file");
    let c = Case::new();
    fs::remove_file(c.bundle.join("lib/actual.so")).unwrap();
    symlink("../plugins/ctl.so", c.bundle.join("lib/actual.so")).unwrap();
    c.rejected("differs from its declared target");
    let c = Case::new();
    let dirty = PathBuf::from(format!("{}/./state", c.root.display()));
    assert!(prepare(&c.raw(), &c.bundle, &dirty).is_err());
}
#[test]
fn identity_lock_frozen_configuration_and_fresh_atomic_generations() {
    let mut c = Case::new();
    let first = c.prepare().unwrap();
    let path = first.receipt.manifest_path.clone();
    let bytes = fs::read(&path).unwrap();
    c.rejected("active/locked");
    let gen1 = first.receipt.generation.clone();
    drop(first);
    let second = c.prepare().unwrap();
    assert_ne!(gen1, second.receipt.generation);
    assert_ne!(path, second.receipt.manifest_path);
    drop(second);
    c.config(|v| v["takeoff_altitude_m"] = json!(2.8));
    c.rejected("different frozen configuration");
    assert_eq!(fs::read(path).unwrap(), bytes);
    let dir = c
        .state
        .join(&c.input.session_id)
        .join(&c.input.node_id)
        .join("generations");
    let entries: Vec<_> = fs::read_dir(dir).unwrap().map(|e| e.unwrap()).collect();
    assert_eq!(entries.len(), 2);
    for entry in entries {
        assert!(entry
            .file_name()
            .to_str()
            .unwrap()
            .starts_with("generation-"));
        for file in ["node.toml", "deployment.json", "receipt.json", "audit"] {
            assert!(entry.path().join(file).exists());
        }
    }
}
#[test]
fn racing_prepares_have_one_live_owner() {
    let c = Case::new();
    let barrier = std::sync::Arc::new(std::sync::Barrier::new(2));
    let results = std::thread::scope(|scope| {
        let joins: Vec<_> = (0..2)
            .map(|_| {
                let b = barrier.clone();
                let c = &c;
                scope.spawn(move || {
                    b.wait();
                    c.prepare()
                })
            })
            .collect();
        joins
            .into_iter()
            .map(|j| j.join().unwrap())
            .collect::<Vec<_>>()
    });
    assert_eq!(results.iter().filter(|r| r.is_ok()).count(), 1);
    assert!(results
        .iter()
        .find_map(|r| r.as_ref().err())
        .unwrap()
        .contains("active/locked"));
}
#[test]
fn target_roots_are_explicit_clean_and_unambiguous() {
    assert!(managed_root(None, None).is_err());
    assert!(managed_root(Some(""), None).is_err());
    assert!(managed_root(Some("/a"), Some("/b")).is_err());
    assert_eq!(
        managed_root(Some("/agent/private"), None).unwrap(),
        Path::new("/agent/private/sync-runtime")
    );
    assert_eq!(
        managed_root(Some("/same"), Some("/same")).unwrap(),
        Path::new("/same/sync-runtime")
    );
    for root in [
        "relative",
        "/bad/./root",
        "/bad/../root",
        "/bad//root",
        "/bad/",
    ] {
        assert!(managed_root(Some(root), None).is_err(), "{root}");
    }
}
#[test]
fn verified_host_descriptor_survives_directory_entry_replacement() {
    let c = Case::new();
    let p = c.prepare().unwrap();
    let old = fs::read(&p.host_path).unwrap();
    fs::rename(&p.host_path, c.bundle.join("bin/old")).unwrap();
    fs::write(&p.host_path, b"replacement").unwrap();
    assert_eq!(fs::read(p.pinned_host_path()).unwrap(), old);
}
#[test]
fn real_exec_preserves_pid_frozen_ros_environment_and_process_lifetime_lock() {
    // Real native process execution, deliberately a lifecycle probe, not ROS.
    for id in [COMPOSITION_ID, PX4_LOCAL_COMPOSITION_ID] {
        let mut c = Case::new();
        c.select_composition(id);
        let source = c.root.join("host.c");
        fs::write(&source,r#"#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
int main(int argc,char**argv){
 printf("%d|%s|%s|%s|%s|%s|%s\n",getpid(),argv[1],argv[2],getenv("ROS_MASTER_URI"),getenv("ROS_IP"),getenv("ROS_NAMESPACE"),getenv("ROS_HOSTNAME")?"BAD":"unset"); fflush(stdout);
 return getchar()=='q'?0:3;
}"#).unwrap();
        let output = Command::new("cc")
            .arg(&source)
            .arg("-o")
            .arg(c.bundle.join("bin/xgc-rt-host"))
            .output()
            .expect("deployment lifecycle test requires a native C compiler");
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let host_sha = sha256(&fs::read(c.bundle.join("bin/xgc-rt-host")).unwrap());
        c.descriptor(|b| b.host.sha256 = host_sha);
        let managed = c.root.join("managed");
        c.state = managed.join("sync-runtime");
        let mut child = Command::new(env!("CARGO_BIN_EXE_xgc-rt-render"))
            .args([
                "run",
                "--bundle-root",
                c.bundle.to_str().unwrap(),
                "--deployment-json",
                &c.raw(),
            ])
            .env_remove("XGC_CORE_MANAGED_ROOT")
            .env("XGC_AGENT_MANAGED_ROOT", &managed)
            .env("ROS_HOSTNAME", "inherited-wrong")
            .env("ROS_IP", "1.2.3.4")
            .env("ROS_MASTER_URI", "http://wrong:1")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let mut stdout = BufReader::new(child.stdout.take().unwrap());
        let mut receipt = String::new();
        stdout.read_line(&mut receipt).unwrap();
        let receipt: Value = serde_json::from_str(&receipt).expect("renderer receipt before exec");
        let mut line = String::new();
        stdout.read_line(&mut line).unwrap();
        assert_eq!(
            line.trim(),
            format!(
                "{}|--manifest|{}|http://172.30.251.251:11311|172.30.251.102|/uav2|unset",
                child.id(),
                receipt["manifest_path"].as_str().unwrap()
            )
        );
        c.rejected("active/locked");
        use std::io::Write;
        child.stdin.take().unwrap().write_all(b"q").unwrap();
        assert!(child.wait().unwrap().success());
        let next = c.prepare().unwrap();
        assert_ne!(
            next.receipt.generation,
            receipt["generation"].as_str().unwrap()
        );
    }
}
#[test]
fn cli_rejects_duplicate_flags_and_run_state_override() {
    let bin = env!("CARGO_BIN_EXE_xgc-rt-render");
    let description = Command::new(bin).arg("describe").output().unwrap();
    assert!(description.status.success());
    let value: Value = serde_json::from_slice(&description.stdout).unwrap();
    assert_eq!(
        value["composition_sha256"],
        sha256(value["composition_bytes"].as_str().unwrap().as_bytes())
    );
    for args in [
        vec!["describe", "--extra"],
        vec!["prepare", "--bundle-root", "/a", "--bundle-root", "/b"],
        vec![
            "run",
            "--state-root",
            "/bad",
            "--bundle-root",
            "/bundle",
            "--deployment-json",
            "{}",
        ],
    ] {
        assert!(!Command::new(bin)
            .args(args)
            .output()
            .unwrap()
            .status
            .success());
    }
}

#[test]
fn describe_selects_only_compiled_compositions_and_preserves_legacy_default() {
    let bin = env!("CARGO_BIN_EXE_xgc-rt-render");
    let default = Command::new(bin).arg("describe").output().unwrap();
    assert!(default.status.success());
    assert_eq!(
        composition_sha256(),
        "e9589ea20818504ab2e698ac0ffd8db21e0410581c41c70147e4ebe06b37b985"
    );
    for id in [COMPOSITION_ID, PX4_LOCAL_COMPOSITION_ID] {
        let output = Command::new(bin)
            .args(["describe", "--composition-id", id])
            .output()
            .unwrap();
        assert!(output.status.success());
        if id == COMPOSITION_ID {
            assert_eq!(output.stdout, default.stdout);
        }
        let value: Value = serde_json::from_slice(&output.stdout).unwrap();
        assert_eq!(
            value,
            json!({"schema_version":1,"composition_id":id,
            "composition_sha256":composition(id).unwrap().sha256(),
            "composition_bytes":composition(id).unwrap().bytes,"platform":target_platform().unwrap(),
            "input_time_domain":"wall-unix","managed_launch":"run","live_readiness":false})
        );
    }
    for args in [
        vec!["describe", "--composition-id"],
        vec!["describe", "--composition-id", "unknown"],
        vec!["describe", "--composition-id", "plan-dmpc/v1"],
        vec![
            "describe",
            "--composition-id",
            PX4_LOCAL_COMPOSITION_ID,
            "--composition-id",
            COMPOSITION_ID,
        ],
        vec![
            "describe",
            "--composition-id",
            PX4_LOCAL_COMPOSITION_ID,
            "extra",
        ],
        vec!["prepare", "--composition-id", PX4_LOCAL_COMPOSITION_ID],
        vec!["run", "--composition-id", PX4_LOCAL_COMPOSITION_ID],
    ] {
        let output = Command::new(bin).args(&args).output().unwrap();
        assert_eq!(output.status.code(), Some(2), "{args:?}");
        assert!(
            output.stdout.is_empty(),
            "rejected selector must produce no receipt"
        );
    }
}

#[test]
fn composition_id_digest_and_single_bundle_must_all_match_before_publication() {
    let ids = [COMPOSITION_ID, PX4_LOCAL_COMPOSITION_ID];
    for id in ids {
        for digest_id in ids {
            for bundle_id in ids {
                let mut c = Case::new();
                c.input.composition_id = id.into();
                c.input.composition_sha256 = composition(digest_id).unwrap().sha256();
                c.descriptor(|b| b.composition_sha256 = composition(bundle_id).unwrap().sha256());
                if id == digest_id && id == bundle_id {
                    let p = c.prepare().unwrap();
                    let m = manifest(&p);
                    let cfg = &plugin(&m, "ctl-px4")["config"];
                    let px4 = id == PX4_LOCAL_COMPOSITION_ID;
                    assert_eq!(
                        cfg["tracking_backend"].as_str(),
                        Some(if px4 { "px4_local" } else { "dfbc" })
                    );
                    assert_eq!(cfg.get("reference_analytic_type").is_none(), px4);
                    assert_eq!(m["session"]["period_ms"].as_integer(), Some(1));
                    assert_eq!(m["plugin"].as_array().unwrap().len(), 5);
                    assert_eq!(
                        p.receipt.composition_sha256,
                        composition(id).unwrap().sha256()
                    );
                } else {
                    assert!(
                        c.prepare().is_err(),
                        "accepted id={id}, digest={digest_id}, bundle={bundle_id}"
                    );
                    assert!(!c.state.exists(), "mismatch created state");
                }
            }
        }
    }
}

#[test]
fn switching_composition_cannot_replace_an_existing_frozen_identity() {
    for (old, new) in [
        (COMPOSITION_ID, PX4_LOCAL_COMPOSITION_ID),
        (PX4_LOCAL_COMPOSITION_ID, COMPOSITION_ID),
    ] {
        let mut c = Case::new();
        c.select_composition(old);
        let p = c.prepare().unwrap();
        let path = p.receipt.manifest_path.clone();
        let bytes = fs::read(&path).unwrap();
        drop(p);
        c.select_composition(new);
        c.rejected("different frozen configuration");
        assert_eq!(fs::read(path).unwrap(), bytes);
    }
}

#[test]
fn second_composition_keeps_strict_configuration_and_fixed_descriptor_roles() {
    let mut c = Case::new();
    c.select_composition(PX4_LOCAL_COMPOSITION_ID);
    c.config(|v| v["tracking_backend"] = json!("dfbc"));
    c.rejected("configuration schema");
    let mut c = Case::new();
    c.select_composition(PX4_LOCAL_COMPOSITION_ID);
    let mut descriptor: Value =
        serde_json::from_slice(&fs::read(c.bundle.join(BUNDLE_FILE)).unwrap()).unwrap();
    descriptor["plugins"]["planner"] = descriptor["plugins"]["controller"].clone();
    let bytes = serde_json::to_vec(&descriptor).unwrap();
    c.input.bundle_sha256 = sha256(&bytes);
    fs::write(c.bundle.join(BUNDLE_FILE), bytes).unwrap();
    c.rejected("bundle schema");
    assert!(!c.state.exists());
}

#[test]
fn unrelated_exec_cannot_extend_a_released_identity_lock() {
    let c = Case::new();
    let first = c.prepare().unwrap();
    let mut unrelated = Command::new("sleep").arg("5").spawn().unwrap();
    drop(first);
    let next = c.prepare();
    unrelated.kill().unwrap();
    unrelated.wait().unwrap();
    assert!(
        next.is_ok(),
        "unrelated child inherited another identity's lock"
    );
}

#[test]
fn nonregular_bundle_file_is_rejected_without_blocking() {
    use std::ffi::CString;
    use std::os::unix::ffi::OsStrExt;
    let c = Case::new();
    let path = c.bundle.join("plugins/ctl.so");
    fs::remove_file(&path).unwrap();
    let path = CString::new(path.as_os_str().as_bytes()).unwrap();
    assert_eq!(unsafe { libc::mkfifo(path.as_ptr(), 0o600) }, 0);
    c.rejected("expected a regular file");
    assert!(!c.state.exists());
}

#[test]
fn original_json_scalar_types_are_not_coerced_or_enum_tagged_objects() {
    for value in [
        json!(null),
        json!(1),
        json!(true),
        json!({"wall-unix": null}),
    ] {
        let mut c = Case::new();
        c.config(|v| v["input_time_domain"] = value);
        c.rejected("configuration schema");
    }
    for value in [json!(null), json!("false"), json!(0)] {
        let mut c = Case::new();
        c.config(|v| v["calibration"]["verified"] = value);
        c.rejected("configuration schema");
    }
    let mut c = Case::new();
    c.config(|v| v["calibration"]["provenance"]["kind"] = json!({"unverified": null}));
    c.rejected("configuration schema");
    for value in [json!(null), json!("2.3"), json!(true)] {
        let mut c = Case::new();
        c.config(|v| v["takeoff_altitude_m"] = value);
        c.rejected("configuration schema");
    }
    let c = Case::new();
    let trailing = format!("{} {{}}", c.raw());
    assert!(Deployment::parse(&trailing).is_err());
    let mut input: Value = serde_json::from_str(&c.raw()).unwrap();
    input["schema_version"] = json!(1.0);
    assert!(Deployment::parse(&input.to_string()).is_err());
}

#[test]
fn frozen_identifier_and_utf8_byte_length_boundaries() {
    for id in ["_node", "-node", "", &"n".repeat(65)] {
        let mut c = Case::new();
        c.input.node_id = id.into();
        c.rejected("Session/node identity");
    }
    let mut c = Case::new();
    c.input.node_id = "n".repeat(64);
    c.input.session_id = "s".repeat(64);
    assert!(c.prepare().is_ok());
    for ns in ["_uav2", "2uav", "group/uav2", &"u".repeat(65)] {
        let mut c = Case::new();
        c.input.robot_namespace = ns.into();
        c.rejected("robot namespace");
    }
    let mut c = Case::new();
    c.config(|v| {
        v["calibration"]["provenance"]["source_id"] = json!("é".repeat(128));
        v["topics"]["pose_topic"] = json!(format!("/{}", "p".repeat(127)));
    });
    assert!(c.prepare().is_ok());
    let mut c = Case::new();
    c.config(|v| v["calibration"]["provenance"]["source_id"] = json!("é".repeat(129)));
    c.rejected("calibration provenance");
    let mut c = Case::new();
    c.config(|v| v["topics"]["pose_topic"] = json!(format!("/{}", "p".repeat(128))));
    c.rejected("invalid absolute ROS name");
    let mut c = Case::new();
    c.config(|v| v["topics"]["pose_topic"] = json!("/_private/pose"));
    c.rejected("invalid absolute ROS name");
}

fn simulation() -> Value {
    json!({"epoch_ns":2000000000,"topic":"/clock","expected_publisher":"/experiment_world",
        "world_instance_id":"world-generation-52","startup_timeout_wall_ms":3000,
        "stale_after_wall_ms":100,"max_advance_ns":50000000,"poll_wall_ms":5,"queue_capacity":256})
}

#[test]
fn simulation_uses_shared_epoch_and_existing_source_in_both_control_profiles() {
    for id in [COMPOSITION_ID, PX4_LOCAL_COMPOSITION_ID] {
        let mut c = Case::new();
        c.select_composition(id);
        c.config(|v| {
            v["input_time_domain"] = json!("ros1-sim");
            v["simulation"] = simulation();
        });
        let prepared = c.prepare().unwrap();
        assert_eq!(prepared.receipt.input_time_domain, "ros1-sim");
        let m = manifest(&prepared);
        assert_eq!(m["session"]["epoch_ns"].as_integer(), Some(2000000000));
        assert_eq!(m["session"]["period_ms"].as_integer(), Some(1));
        assert_eq!(m["clock_source"]["kind"].as_str(), Some("ros1_sim"));
        assert_eq!(m["clock_source"]["plugin"].as_str(), Some("ros_io"));
        assert_eq!(m["clock_source"]["expected_publisher"].as_str(), Some("/experiment_world"));
        assert_eq!(m["clock_source"]["world_instance_id"].as_str(), Some("world-generation-52"));
        assert_eq!(m["clock_source"]["queue_capacity"].as_integer(), Some(256));
        assert!(m.get("clock").is_none());
        assert_eq!(m["plugin"].as_array().unwrap().len(), 5);
        assert_eq!(plugin(&m, "ctl-px4")["config"]["time_source"].as_str(), Some("session"));
    }
}

#[test]
fn mixed_clock_deployments_and_invalid_host_source_are_rejected() {
    let mut c = Case::new();
    c.config(|v| v["input_time_domain"] = json!("ros1-sim"));
    c.rejected("ros1-sim requires simulation");
    let mut c = Case::new();
    c.config(|v| v["simulation"] = simulation());
    c.rejected("wall-unix forbids simulation");
    let mut c = Case::new();
    c.config(|v| v["simulation"] = Value::Null);
    c.rejected("configuration schema");
    for (field, bad, reason) in [
        ("epoch_ns", json!(0), "positive shared session.epoch_ns"),
        ("expected_publisher", json!("relative"), "authority identity"),
        ("queue_capacity", json!(0), "timing/queue limits"),
    ] {
        let mut c = Case::new();
        c.config(|v| {
            v["input_time_domain"] = json!("ros1-sim");
            v["simulation"] = simulation();
            v["simulation"][field] = bad;
        });
        c.rejected(reason);
    }
}

fn hil_case(authority_self: bool) -> Case {
    let mut c = Case::new();
    c.input.node_id = "board-b".into();
    c.select_composition(HIL_COMPOSITION_ID);
    c.descriptor(|b| {
        b.plugins.numeric_vehicle = b.plugins.controller.take();
        b.plugins.plan_dmpc = b.plugins.rigid_state.take();
        b.plugins.dmpc_rounds = b.plugins.reference.take();
        b.plugins.station_io = b.plugins.hover_thrust.take();
    });
    c.config(|v| *v = json!({
        "input_time_domain":"wall-unix", "ros_master_uri":"http://127.0.0.1:11311", "ros_ip":"127.0.0.1",
        "epoch_ns":1900000000000000000_i64,
        "members":[
            {"uav_id":1,"robot_namespace":"uav1","planner_node":"board-a","control_node":"board-a"},
            {"uav_id":2,"robot_namespace":"uav2","planner_node":"board-b","control_node":"board-b"}],
        "mission_authority_node":if authority_self {"board-b"} else {"board-a"},
        "radio":{"listen":["tcp/127.0.0.1:17442"],"connect":["tcp/127.0.0.1:17441"]},
        "station":{"robot_id":"xgc2e-12345678901234567890","zenoh_connect":"tcp/127.0.0.1:17457","command_socket":"/tmp/hil-board-b.sock"},
        "scene":{"snapshot_topic":"/experiment/scene/snapshot","state_topic":"/experiment/scene/state",
            "timeline_ack_topic":"/uav2/dmpc/timeline_ack","timeline_status_topic":"/uav2/dmpc/timeline_status"},
        "planner":{"manifest":"scenarios/dmpc_comprehensive/uav2.yaml","algorithm":"legacy","scene_id":"dmpc-uav8_comprehensive","chain_n":3,"state_dim":9,"horizon":40,"sampling_time":0.1},
        "initial_position":[1.2,-3.4,0.0],"initial_velocity":[0.1,0.2,0.0]
    }));
    c
}

#[test]
fn hil_fixed_graph_has_real_port_roles_and_no_actuator_or_fake_sensor() {
    for authority_self in [false, true] {
        let c = hil_case(authority_self);
        let prepared = c.prepare().unwrap();
        let m = manifest(&prepared);
        assert_eq!(prepared.receipt.role, "numeric-hil");
        assert_eq!(prepared.receipt.actuator_namespace, None);
        assert_eq!(m["session"]["roster"], toml::Value::try_from(vec!["board-a", "board-b"]).unwrap());
        assert_eq!(m["transport"]["kind"].as_str(), Some("zenoh"));
        assert_eq!(m["session"]["period_ms"].as_integer(), Some(1));
        let rounds = plugin(&m, "dmpc-rounds");
        assert_eq!(rounds["config"]["origins"].as_array().unwrap(), &[0.into(),1.into()]);
        assert_eq!(rounds["config"]["control_origin"].as_integer(), Some(1));
        assert_eq!(rounds["config"]["planner_period_ms"].as_integer(), Some(100));
        assert_eq!(rounds["config"]["planner_epoch_ns"], m["session"]["epoch_ns"]);
        assert_eq!(rounds["bind"]["plan_in"]["from"].as_array().unwrap(), &["board-a".into()]);
        let station = plugin(&m, "station-io");
        assert_eq!(station["config"]["authority"].as_bool(), Some(authority_self));
        assert_eq!(station["bind"].get("command").is_some(), authority_self);
        assert_eq!(station["bind"].get("mission_request").is_some(), authority_self);
        assert_eq!(station["config"]["command"].as_bool(), Some(station["bind"].get("command").is_some()));
        assert_eq!(station["config"]["mission"].as_bool(), Some(station["bind"].get("mission_request").is_some()));
        assert_eq!(plugin(&m,"numeric-vehicle")["bind"]["command"]["from"].as_array().unwrap(),
            &[if authority_self {"board-b".into()} else {"board-a".into()}]);
        let ros = plugin(&m,"ros_io");
        for key in ["setpoint", "attitude_rate", "fcu_request", "command", "mission_request", "imu", "paired_state"] {
            assert!(ros["bind"].get(key).is_none());
            assert!(ros["config"].get(format!("{key}_topic")).is_none());
        }
        assert_eq!(plugin(&m,"numeric-vehicle")["config"]["initial_position"].as_array().unwrap()[0].as_float(), Some(1.2));
        assert_eq!(plugin(&m,"plan-dmpc")["bind"]["neighbor_plan"]["from"].as_array().unwrap(), &["board-b".into()]);
        let plan = &plugin(&m, "plan-dmpc")["config"];
        assert_eq!(plan["manifest"].as_str(), Some("scenarios/dmpc_comprehensive/uav2.yaml"));
        assert_eq!(plan["self_id"].as_integer(), Some(2));
        assert_eq!(plan["timeline_authority"].as_integer(), Some(if authority_self { 1 } else { 0 }));
        assert_eq!(plan["scene_id"].as_str(), Some("dmpc-uav8_comprehensive"));
        assert_eq!(plan["algorithm"].as_str(), Some("legacy"));
        assert_eq!(plan["chain_n"].as_integer(), Some(3));
        assert_eq!(plan["state_dim"].as_integer(), Some(9));
        assert_eq!(plan["horizon"].as_integer(), Some(40));
        assert_eq!(plan["sampling_time"].as_float(), Some(0.1));
        assert!(plan.get("fleet_count").is_none());
        for extra in ["takeoff_altitude", "local_takeoff_altitude", "leader_speed", "q_pos", "num_uavs", "obstacle_count"] {
            assert!(plan.get(extra).is_none(), "{extra}");
        }
    }
    let c = Case::new();
    let p = c.prepare().unwrap();
    assert_eq!(p.receipt.role, "control");
    assert_eq!(p.receipt.actuator_namespace.as_deref(), Some("uav2"));
}

#[test]
fn hil_rejects_mixed_graph_clock_authority_and_robot_ownership() {
    let mut c = hil_case(false);
    c.config(|v| v["mission_authority_node"] = json!("unlisted")); c.rejected("mission authority");
    let mut c = hil_case(false);
    c.config(|v| v["members"][1]["robot_namespace"] = json!("uav3")); c.rejected("namespace");
    let mut c = hil_case(false);
    c.config(|v| v["members"][1]["control_node"] = json!("control-board-b")); c.rejected("co-located");
    let mut c = hil_case(false);
    c.config(|v| v["members"][1]["planner_node"] = json!("board-a")); c.rejected("execution node");
    let mut c = hil_case(false);
    c.config(|v| v["calibration"] = json!({"verified":true})); c.rejected("unknown field");
    let mut c = hil_case(false);
    c.config(|v| v["radio"]["listen"] = json!([])); c.rejected("radio");
    let mut c = hil_case(false);
    c.config(|v| v["simulation"] = json!(null)); c.rejected("schema");
    let mut c = hil_case(false);
    c.config(|v| { v["planner"].as_object_mut().unwrap().remove("manifest"); }); c.rejected("manifest");
    let mut c = hil_case(false);
    c.config(|v| v["planner"]["manifest"] = json!("")); c.rejected("manifest");
    let mut c = hil_case(false);
    c.config(|v| v["planner"]["manifest"] = json!("scenarios/dmpc_comprehensive/uav2.txt"));
    let prepared = c.prepare().unwrap();
    assert_eq!(
        plugin(&manifest(&prepared), "plan-dmpc")["config"]["manifest"].as_str(),
        Some("scenarios/dmpc_comprehensive/uav2.txt")
    );
    let mut c = hil_case(false);
    c.descriptor(|b| b.plugins.controller = b.plugins.numeric_vehicle.clone()); c.rejected("roles");
    let mut c = Case::new();
    c.descriptor(|b| b.plugins.numeric_vehicle = b.plugins.controller.clone()); c.rejected("roles");
}

#[test]
fn robot_identity_supports_remote_planner_and_control_without_granting_planner_actuators() {
    let member = RobotMember { uav_id: 2, robot_namespace: "uav2".into(), planner_node: "gcs-planner-2".into(), control_node: "board-2".into() };
    assert_eq!(member.role("gcs-planner-2"), Some(NodeRole::Planner));
    assert_eq!(member.role("board-2"), Some(NodeRole::Control));
    assert_eq!(member.role("uav2"), None);
    let mut c = Case::new();
    c.input.node_id = "board-2".into();
    c.config(|v| v["robot_member"] = serde_json::to_value(&member).unwrap());
    let p = c.prepare().unwrap();
    assert_eq!(p.receipt.actuator_namespace.as_deref(), Some("uav2"));
    drop(p);
    c.input.node_id = "gcs-planner-2".into();
    c.rejected("control node");
    // No undeployed SMC/planner profile is advertised by a role-only model.
    assert!(composition("uav-dmpc-remote-smc/v1").is_err());
}

#[test]
fn hil_sim_uses_same_world_epoch_as_planner() {
    let mut c = hil_case(false);
    c.config(|v| {
        v["input_time_domain"] = json!("ros1-sim"); v["epoch_ns"] = json!(2000000000_i64);
        v["simulation"] = json!({"epoch_ns":2000000000_i64,"topic":"/clock","expected_publisher":"/world_clock",
            "world_instance_id":"world-52","startup_timeout_wall_ms":5000,"stale_after_wall_ms":2000,
            "max_advance_ns":1000000000_i64,"poll_wall_ms":2,"queue_capacity":64});
    });
    let p = c.prepare().unwrap(); let m = manifest(&p);
    assert_eq!(p.receipt.input_time_domain, "ros1-sim");
    assert_eq!(m["clock_source"]["plugin"].as_str(), Some("ros_io"));
    assert_eq!(m["session"]["epoch_ns"], plugin(&m,"dmpc-rounds")["config"]["planner_epoch_ns"]);
    drop(p);
    c.config(|v| v["simulation"]["epoch_ns"] = json!(3000000000_i64)); c.rejected("epochs differ");
}

fn copy_pin(case: &Case, src: &str, dest: &str) -> BundleFile {
    let bytes = fs::read(case.bundle.join(src)).unwrap();
    fs::write(case.bundle.join(dest), &bytes).unwrap();
    BundleFile { path: dest.into(), sha256: sha256(&bytes) }
}

fn vehicle_topics() -> Value {
    json!({
        "imu_topic":"/uav2/mavros/imu/data_raw","pose_topic":"/mocap/uav2/pose",
        "vision_pose_topic":"/uav2/mavros/vision_pose/pose",
        "fcu_state_topic":"/uav2/mavros/state","local_pose_topic":"/uav2/mavros/local_position/pose",
        "local_velocity_topic":"/uav2/mavros/local_position/velocity_local","fcu_imu_topic":"/uav2/mavros/imu/data",
        "battery_topic":"/uav2/mavros/battery","setpoint_topic":"/uav2/mavros/setpoint_raw/local",
        "status_topic":"/uav2/custom/statustext","fcu_request_topic":"/uav2/mavros/cmd"
    })
}

fn channel_names(m: &toml::Value) -> Vec<&str> {
    m["channel"].as_array().unwrap().iter().map(|c| c["name"].as_str().unwrap()).collect()
}

fn assert_plan_configure(plan: &toml::Value, self_id: i64, authority: i64) {
    assert_eq!(plan["manifest"].as_str(), Some("scenarios/dmpc_comprehensive/uav2.yaml"));
    assert_eq!(plan["self_id"].as_integer(), Some(self_id));
    assert_eq!(plan["timeline_authority"].as_integer(), Some(authority));
    assert_eq!(plan["scene_id"].as_str(), Some("dmpc-uav8_comprehensive"));
    assert_eq!(plan["algorithm"].as_str(), Some("legacy"));
    assert_eq!(plan["chain_n"].as_integer(), Some(3));
    assert_eq!(plan["state_dim"].as_integer(), Some(9));
    assert_eq!(plan["horizon"].as_integer(), Some(40));
    assert_eq!(plan["sampling_time"].as_float(), Some(0.1));
    assert!(plan.get("fleet_count").is_none());
}

fn dmpc_case(kind: &str) -> Case {
    let mut c = Case::new();
    let plan = copy_pin(&c, "plugins/ctl.so", "plugins/plan.so");
    let rounds = copy_pin(&c, "plugins/ctl.so", "plugins/rounds.so");
    let station = copy_pin(&c, "plugins/ctl.so", "plugins/station.so");
    let colocated = kind == "native";
    c.input.node_id = if kind == "planner" { "gcs-b" } else { "board-b" }.into();
    c.descriptor(|b| {
        b.plugins.rigid_state = None;
        b.plugins.hover_thrust = None;
        b.plugins.reference = None;
        b.plugins.numeric_vehicle = None;
        b.plugins.station_io = Some(station);
        if kind == "planner" { b.plugins.controller = None; }
        if kind != "smc" {
            b.plugins.plan_dmpc = Some(plan);
            b.plugins.dmpc_rounds = Some(rounds);
        }
    });
    let id = match kind {
        "native" => NATIVE_COMPOSITION_ID,
        "planner" => PLANNER_COMPOSITION_ID,
        "smc" => SMC_COMPOSITION_ID,
        _ => panic!("kind"),
    };
    c.select_composition(id);
    let members = if colocated {
        json!([
            {"uav_id":1,"robot_namespace":"uav1","planner_node":"board-a","control_node":"board-a"},
            {"uav_id":2,"robot_namespace":"uav2","planner_node":"board-b","control_node":"board-b"}
        ])
    } else {
        json!([
            {"uav_id":1,"robot_namespace":"uav1","planner_node":"gcs-a","control_node":"board-a"},
            {"uav_id":2,"robot_namespace":"uav2","planner_node":"gcs-b","control_node":"board-b"}
        ])
    };
    let mut body = json!({
        "input_time_domain":"wall-unix","ros_master_uri":"http://127.0.0.1:11311","ros_ip":"127.0.0.1",
        "epoch_ns":1900000000000000000_i64,
        "members": members,
        "mission_authority_node": if colocated {"board-a"} else {"gcs-a"},
        "radio":{"listen":["tcp/127.0.0.1:17442"],"connect":["tcp/127.0.0.1:17441"]},
        "station":{"robot_id":"xgc2e-12345678901234567890","zenoh_connect":"tcp/127.0.0.1:17457","command_socket":"/tmp/dmpc-board.sock"}
    });
    if kind != "smc" {
        body["scene"] = json!({"snapshot_topic":"/experiment/scene/snapshot","state_topic":"/experiment/scene/state",
            "timeline_ack_topic":"/uav2/dmpc/timeline_ack","timeline_status_topic":"/uav2/dmpc/timeline_status"});
        body["planner"] = json!({"manifest":"scenarios/dmpc_comprehensive/uav2.yaml","algorithm":"legacy","scene_id":"dmpc-uav8_comprehensive","chain_n":3,"state_dim":9,"horizon":40,"sampling_time":0.1});
    }
    if kind != "planner" {
        body["takeoff_altitude_m"] = json!(1.5);
        body["topics"] = vehicle_topics();
    }
    c.config(|v| *v = body);
    c
}

#[test]
fn dmpc_graphs_use_ros_paired_state_and_omit_ekf_hover_and_attitude_rate() {
    assert_eq!(composition_sha256(), "e9589ea20818504ab2e698ac0ffd8db21e0410581c41c70147e4ebe06b37b985");
    assert_eq!(composition(PX4_LOCAL_COMPOSITION_ID).unwrap().sha256(), "8017449b0396217aae10b9de9548fafcf22bc5bb8e628760c86ad4efaaf6b9f4");
    for case in [hil_case(false), dmpc_case("native"), dmpc_case("planner")] {
        let prepared = case.prepare().unwrap();
        let rendered = manifest(&prepared);
        let planner = plugin(&rendered, "plan-dmpc");
        assert_eq!(planner["bind"]["own_position"]["channel"].as_str(), Some("radio_position"));
        assert_eq!(planner["bind"]["neighbor_position"]["channel"].as_str(), Some("radio_position"));
        let node = rendered["session"]["node"].as_str().unwrap();
        let peers = planner["bind"]["neighbor_position"]["from"].as_array().unwrap();
        assert!(!peers.is_empty());
        assert!(peers.iter().all(|peer| peer.as_str() != Some(node)));
    }

    let native = dmpc_case("native");
    let prepared = native.prepare().unwrap();
    let m = manifest(&prepared);
    let names = channel_names(&m);
    for banned in ["estimate", "rigid_state", "hover_thrust", "attitude_rate", "vision_pose"] {
        assert!(!names.contains(&banned), "{banned}");
    }
    assert_eq!(prepared.receipt.role, "dmpc-native");
    assert_eq!(prepared.receipt.actuator_namespace.as_deref(), Some("uav2"));
    let ros = plugin(&m, "ros_io");
    assert!(ros["bind"]["paired_state"].get("from").is_none());
    assert_eq!(ros["bind"]["paired_state"]["channel"].as_str(), Some("paired_state"));
    assert_eq!(ros["bind"]["vision_pose"]["channel"].as_str(), Some("pose"));
    assert_eq!(ros["bind"]["vision_pose"]["from"].as_array().unwrap(), &["board-b".into()]);
    assert_ne!(ros["bind"]["vision_pose"]["channel"].as_str(), ros["bind"]["local_pose"]["channel"].as_str());
    assert_eq!(ros["config"]["vision_pose_topic"].as_str(), Some("/uav2/mavros/vision_pose/pose"));
    assert_eq!(ros["config"]["local_pose_topic"].as_str(), Some("/uav2/mavros/local_position/pose"));
    assert_eq!(ros["config"]["local_velocity_topic"].as_str(), Some("/uav2/mavros/local_position/velocity_local"));
    let ctl = plugin(&m, "ctl-px4");
    assert!(ctl["bind"].get("paired_state").is_none());
    assert!(ctl["bind"].get("estimate").is_none());
    assert!(ctl["bind"].get("hover_thrust").is_none());
    assert!(ctl["bind"].get("attitude_rate").is_none());
    assert!(ctl["bind"].get("setpoint").is_some());
    assert!(ctl["bind"].get("fcu_request").is_some());
    assert_eq!(ctl["bind"]["local_pose"]["from"].as_array().unwrap(), &["board-b".into()]);
    assert_eq!(ctl["config"]["tracking_backend"].as_str(), Some("smc"));
    assert_eq!(plugin(&m, "plan-dmpc")["bind"]["paired_state"]["from"].as_array().unwrap(), &["board-b".into()]);
    assert_plan_configure(&plugin(&m, "plan-dmpc")["config"], 2, 0);
    let station = plugin(&m, "station-io");
    assert_eq!(station["bind"]["paired_state"]["from"].as_array().unwrap(), &["board-b".into()]);
    assert_eq!(station["bind"]["imu"]["channel"].as_str(), Some("fcu_imu"));
    assert_eq!(station["bind"]["battery"]["channel"].as_str(), Some("battery"));
    assert_eq!(station["bind"]["fcu_state"]["channel"].as_str(), Some("fcu_state"));
    assert_eq!(station["config"]["authority"].as_bool(), Some(false));
    assert_eq!(station["config"]["command"].as_bool(), Some(station["bind"].get("command").is_some()));
    assert_eq!(station["config"]["mission"].as_bool(), Some(station["bind"].get("mission_request").is_some()));
    assert!(m["plugin"].as_array().unwrap().iter().all(|p| p["name"].as_str() != Some("rigid-state") && p["name"].as_str() != Some("hover-thrust")));

    let planner = dmpc_case("planner");
    let prepared = planner.prepare().unwrap();
    let m = manifest(&prepared);
    assert_eq!(prepared.receipt.role, "dmpc-planner");
    assert_eq!(prepared.receipt.actuator_namespace, None);
    assert!(plugin(&m, "ros_io")["bind"].get("paired_state").is_none());
    assert_eq!(plugin(&m, "plan-dmpc")["bind"]["paired_state"]["from"].as_array().unwrap(), &["board-b".into()]);
    assert_plan_configure(&plugin(&m, "plan-dmpc")["config"], 2, 0);
    let station = plugin(&m, "station-io");
    assert_eq!(station["bind"]["paired_state"]["from"].as_array().unwrap(), &["board-b".into()]);
    assert!(station["bind"].get("imu").is_none());
    assert_eq!(station["config"]["authority"].as_bool(), Some(false));
    assert_eq!(station["config"]["command"].as_bool(), Some(station["bind"].get("command").is_some()));
    assert_eq!(station["config"]["mission"].as_bool(), Some(station["bind"].get("mission_request").is_some()));

    let smc = dmpc_case("smc");
    let prepared = smc.prepare().unwrap();
    let m = manifest(&prepared);
    assert_eq!(prepared.receipt.role, "dmpc-smc");
    assert_eq!(prepared.receipt.actuator_namespace.as_deref(), Some("uav2"));
    assert!(plugin(&m, "ros_io")["bind"]["paired_state"].get("from").is_none());
    assert_eq!(plugin(&m, "ros_io")["bind"]["vision_pose"]["channel"].as_str(), Some("pose"));
    assert!(plugin(&m, "ctl-px4")["bind"].get("paired_state").is_none());
    assert!(plugin(&m, "ctl-px4")["bind"].get("attitude_rate").is_none());
    let station = plugin(&m, "station-io");
    assert_eq!(station["bind"]["paired_state"]["from"].as_array().unwrap(), &["board-b".into()]);
    assert_eq!(station["bind"]["imu"]["channel"].as_str(), Some("fcu_imu"));
    assert_eq!(station["bind"]["battery"]["channel"].as_str(), Some("battery"));
    assert_eq!(station["bind"]["fcu_state"]["channel"].as_str(), Some("fcu_state"));
    assert_eq!(station["config"]["authority"].as_bool(), Some(false));
    assert_eq!(station["config"]["command"].as_bool(), Some(false));
    assert_eq!(station["config"]["mission"].as_bool(), Some(false));
    assert!(station["bind"].get("command").is_none());
    assert!(station["bind"].get("mission_request").is_none());
    assert!(!channel_names(&m).contains(&"estimate"));
}

#[test]
fn dmpc_graphs_reject_ekf_hover_and_the_wrong_node() {
    let mut c = dmpc_case("native");
    c.config(|v| v["calibration"] = json!({"verified": false}));
    c.rejected("unknown field");
    let mut c = dmpc_case("native");
    c.descriptor(|b| b.plugins.rigid_state = b.plugins.controller.clone());
    c.rejected("roles");
    let mut c = dmpc_case("native");
    c.descriptor(|b| b.plugins.hover_thrust = b.plugins.controller.clone());
    c.rejected("roles");
    let mut c = dmpc_case("smc");
    c.input.node_id = "gcs-b".into();
    c.rejected("control node");
    for kind in ["native", "planner"] {
        let mut c = dmpc_case(kind);
        c.config(|v| { v["planner"].as_object_mut().unwrap().remove("manifest"); });
        c.rejected("manifest");
        let mut c = dmpc_case(kind);
        c.config(|v| v["planner"]["manifest"] = json!(""));
        c.rejected("manifest");
    }
    let mut c = dmpc_case("planner");
    c.input.node_id = "board-b".into();
    c.rejected("planner node");
}
