use super::*;

#[test]
fn native_target_mapping_is_linux_64_bit_only() {
    assert_eq!(native_target("linux", "x86_64").unwrap(), ("linux-amd64", 62));
    assert_eq!(native_target("linux", "aarch64").unwrap(), ("linux-arm64", 183));
    for (os, arch) in [
        ("linux", "x86"), ("linux", "arm"), ("linux", "riscv64"),
        ("linux", "amd64"), ("linux", "arm64"),
        ("macos", "aarch64"), ("windows", "x86_64"),
    ] {
        assert!(native_target(os, arch).is_err(), "accepted {os}/{arch}");
    }
}

#[test]
fn platform_and_elf_machine_share_the_compiled_target() {
    let expected = native_target(std::env::consts::OS, std::env::consts::ARCH);
    assert_eq!(target_platform(), expected.clone().map(|target| target.0));
    assert_eq!(target_elf_machine(), expected.map(|target| target.1));
}

#[test]
fn deployment_accepts_only_its_compiled_target_and_preserves_bytes() {
    let Ok(target) = target_platform() else { return };
    let deployment = control_deployment(target);
    let bytes = deployment.identity_bytes().unwrap();
    let (parsed, _) = Deployment::parse(std::str::from_utf8(&bytes).unwrap()).unwrap();
    assert_eq!(parsed.configuration_json, deployment.configuration_json);
    assert_eq!(parsed.identity_bytes().unwrap(), bytes);
    for wrong in ["linux-amd64", "linux-arm64", "linux-aarch64", "darwin-arm64", ""] {
        if wrong == target { continue; }
        let error = match control_deployment(wrong).validate() {
            Err(error) => error,
            Ok(_) => panic!("accepted target {wrong} on {target}"),
        };
        assert!(error.contains("compiled native target"), "{error}");
    }
}

#[test]
fn target_acceptance_does_not_bypass_existing_deployment_gates() {
    let Ok(target) = target_platform() else { return };
    let valid = control_deployment(target);
    assert!(valid.validate().is_ok());
    let mut bad = valid.clone();
    bad.configuration_json.push(' ');
    assert!(bad.validate().is_err());
    bad = valid.clone();
    bad.composition_sha256 = "ab".repeat(32);
    assert!(bad.validate().is_err());
    bad = valid.clone();
    bad.robot_namespace = "another_robot".into();
    assert!(bad.validate().is_err());
    bad = valid;
    bad.schema_version = 2;
    assert!(bad.validate().is_err());
}

fn control_deployment(platform: &str) -> Deployment {
    let mut topics = serde_json::Map::new();
    for role in [
        "imu_topic", "pose_topic", "vision_pose_topic", "rigid_state_estimate_topic",
        "fcu_state_topic", "local_pose_topic", "local_velocity_topic", "fcu_imu_topic",
        "battery_topic", "command_topic", "alg_setpoint_topic", "attitude_target_topic",
        "setpoint_topic", "attitude_rate_topic", "status_topic", "fcu_request_topic",
    ] {
        topics.insert(role.into(), serde_json::json!(format!("/uav2/{role}")));
    }
    let configuration_json = serde_json::to_string_pretty(&serde_json::json!({
        "input_time_domain": "wall-unix",
        "ros_master_uri": "http://172.30.251.251:11311", "ros_ip": "172.30.251.2",
        "takeoff_altitude_m": 2.3, "topics": topics,
        "calibration": {
            "verified": false,
            "field_offset_xyz": [0.0, 0.0, 0.0], "field_offset_rpy": [0.0, 0.0, 0.0],
            "imu_to_vrpn_marker_xyz": [0.0, 0.0, 0.0], "imu_to_vrpn_marker_rpy": [0.0, 0.0, 0.0],
            "provenance": {
                "kind": "unverified", "source_id": "w09-fixture",
                "source_sha256": "ef".repeat(32), "robot_asset_id": "fixture-only"
            }
        }
    })).unwrap();
    Deployment {
        schema_version: 1, session_id: "w09-session".into(), node_id: "uav2".into(),
        robot_namespace: "uav2".into(), platform: platform.into(), bundle_sha256: "cd".repeat(32),
        composition_id: COMPOSITION_ID.into(), composition_sha256: composition_sha256(),
        configuration_sha256: sha256(configuration_json.as_bytes()), configuration_json,
    }
}
