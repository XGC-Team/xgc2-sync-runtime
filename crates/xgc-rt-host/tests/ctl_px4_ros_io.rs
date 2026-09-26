//! The PX4 controller flies as an aggregator module: ctl-px4 (the ROS-free
//! controller core) replaces the px4_multirotor_controller ROS node. One
//! aggregator holds ros_io, the ESKF module and ctl-px4; the only ROS
//! traffic is ros_io's, and there is no controller node or roslaunch.
//!
//! plant truth -> VRPN + raw IMU -> ros_io -> ESKF (memory) -> ctl-px4
//! (memory); ESKF -> ros_io -> /uav1/mavros/vision_pose/pose -> stand-in PX4
//! local position -> ros_io -> ctl-px4 -> ros_io ->
//! /uav1/mavros/setpoint_raw/local and cmd/command, set_mode -> stand-in PX4
//! -> plant. The same stand-in (px4_standin.py) and pass criteria as
//! px4_ros_io.rs, which flies the unchanged ROS node.
//! That includes the planner path (Custom1): ros_io carries the planner's
//! /uav1/alg/setpoint_raw/local into ctl-px4's alg_setpoint input.
//!
//! ctl-px4 runs with time_source = "session" at 1 ms rounds, like the node's
//! 1 kHz loop, with the TRO configuration's values (uav_nmpc.yaml:
//! px4_local, takeoff 2.3 m, local_type_mask 3072).
//!
//! Needs ROS_PREFIX (ROS Noetic) and PX4_CORE_LIB_DIR (plus the other
//! build-ctl-px4.sh variables). Without them the test prints why and passes.

mod common;

use std::process::Stdio;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

#[test]
fn ctl_px4_module_flies_in_place_of_the_ros_node() {
    let Some(prefix) = common::ros_prefix() else {
        eprintln!("skipped: set ROS_PREFIX (Noetic)");
        return;
    };
    if std::env::var_os("PX4_CORE_LIB_DIR").is_none() {
        eprintln!("skipped: set PX4_CORE_LIB_DIR (dir with libpx4_multirotor_controller_core.so)");
        return;
    }
    let ros_io = common::ros_io_lib(&prefix).clone();
    let ctl = common::ctl_px4_lib(&prefix).clone();
    let (eskf, _) = common::est_rigid_state();
    let port = 11413;
    let master = format!("http://127.0.0.1:{port}");
    let ros_home = common::scratch("ros-home-ctl-px4");
    let _core = common::Roscore::spawn({
        let mut c = common::ros_command(&prefix, "roscore");
        c.args(["-p", &port.to_string()]).env("ROS_HOME", &ros_home).stdout(Stdio::null()).stderr(Stdio::null());
        c
    });
    std::thread::sleep(Duration::from_secs(3));
    std::env::set_var("ROS_MASTER_URI", &master);
    std::env::set_var("ROS_IP", "127.0.0.1");

    let dir = common::scratch("ctl-px4-ros-io");
    let channels: String = [
        ("imu", "state"),
        ("pose", "state"),
        ("rigid_state", "state"),
        ("vision_pose", "state"),
        ("estimate", "state"),
        ("fcu_state", "state"),
        ("local_pose", "state"),
        ("local_velocity", "state"),
        ("fcu_imu", "state"),
        ("battery", "state"),
        ("command", "event"),
        ("alg_setpoint", "control"),
        ("setpoint", "control"),
        ("fcu_request", "event"),
        ("status", "state"),
    ]
    .iter()
    .map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{q}\"\n"))
    .collect();
    let manifest = format!(
        r#"
[session]
id = "ctlpx4rosio"
node = "uav1"
roster = ["uav1"]
period_ms = 1
start_delay_ms = 100
run_for_ms = 120000

[transport]
kind = "loopback"

[audit]
dir = "audit"

{channels}
[[plugin]]
name = "ros_io"
path = "{ros_io}"
trigger = "on_round"
config = {{ node_name = "xgc_ros_io_uav1", imu_topic = "/uav1/mavros/imu/data_raw", pose_topic = "/vrpn_client_node/uav1/pose", vision_pose_topic = "/uav1/mavros/vision_pose/pose", rigid_state_estimate_topic = "/uav1/alg/state_estimator/state", fcu_state_topic = "/uav1/mavros/state", local_pose_topic = "/uav1/mavros/local_position/pose", local_velocity_topic = "/uav1/mavros/local_position/velocity_local", fcu_imu_topic = "/uav1/mavros/imu/data", battery_topic = "/uav1/mavros/battery", command_topic = "/command", alg_setpoint_topic = "/uav1/alg/setpoint_raw/local", setpoint_topic = "/uav1/mavros/setpoint_raw/local", status_topic = "/uav1/custom/statustext", fcu_request_topic = "/uav1/mavros" }}
bind = {{ imu = {{ channel = "imu" }}, pose = {{ channel = "pose" }}, vision_pose = {{ channel = "vision_pose", from = ["uav1"] }}, rigid_state_estimate = {{ channel = "estimate", from = ["uav1"] }}, fcu_state = {{ channel = "fcu_state" }}, local_pose = {{ channel = "local_pose" }}, local_velocity = {{ channel = "local_velocity" }}, fcu_imu = {{ channel = "fcu_imu" }}, battery = {{ channel = "battery" }}, command = {{ channel = "command" }}, alg_setpoint = {{ channel = "alg_setpoint" }}, setpoint = {{ channel = "setpoint", from = ["uav1"] }}, status = {{ channel = "status", from = ["uav1"] }}, fcu_request = {{ channel = "fcu_request", from = ["uav1"] }} }}

[[plugin]]
name = "rigid-state"
path = "{eskf}"
trigger = "both"
config = {{ extrinsic_verified = true }}
bind = {{ imu = {{ channel = "imu", from = ["uav1"] }}, pose = {{ channel = "pose", from = ["uav1"] }}, rigid_state = {{ channel = "rigid_state" }}, vision_pose = {{ channel = "vision_pose" }}, estimate = {{ channel = "estimate" }} }}

[[plugin]]
name = "ctl-px4"
path = "{ctl}"
trigger = "on_round"
config = {{ time_source = "session", takeoff_altitude = 2.3, local_type_mask = 3072, tracking_backend = "px4_local" }}
bind = {{ estimate = {{ channel = "estimate", from = ["uav1"] }}, local_pose = {{ channel = "local_pose", from = ["uav1"] }}, local_velocity = {{ channel = "local_velocity", from = ["uav1"] }}, imu = {{ channel = "fcu_imu", from = ["uav1"] }}, fcu_state = {{ channel = "fcu_state", from = ["uav1"] }}, battery = {{ channel = "battery", from = ["uav1"] }}, vrpn_pose = {{ channel = "pose", from = ["uav1"] }}, command = {{ channel = "command", from = ["uav1"] }}, alg_setpoint = {{ channel = "alg_setpoint", from = ["uav1"] }}, setpoint = {{ channel = "setpoint" }}, fcu_request = {{ channel = "fcu_request" }}, status = {{ channel = "status" }} }}
"#,
        ros_io = ros_io.display(),
        eskf = eskf.display(),
        ctl = ctl.display(),
    );
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(LoopbackBus::new())), Arc::new(WallClock::new(0)), HostOptions::default()).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };

    let out = common::ros_command(&prefix, "python3")
        .arg(common::workspace_root().join("crates/xgc-rt-host/tests/ros/px4_standin.py"))
        .args(["4.0", "90.0", "5.0"])
        .env("ROS_HOME", &ros_home)
        .env("ROS_MASTER_URI", &master)
        .output()
        .unwrap();
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();

    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.lines().rev().find(|l| l.starts_with('{')).unwrap_or_else(|| {
        panic!("stand-in printed no summary\nstderr: {}", String::from_utf8_lossy(&out.stderr))
    });
    println!("stand-in: {line}");
    let r: serde_json::Value = serde_json::from_str(line).unwrap();
    println!("rounds={} wakeups={}", summary.rounds, summary.wakeups);
    for p in &summary.plugins {
        println!(
            "module {} state={} domain={} steps={} consumed={} published={} dropped={} last_error={:?}",
            p.name, p.state, p.domain_state, p.steps, p.consumed, p.published, p.dropped, p.last_error
        );
        assert!(p.last_error.is_none());
    }
    let states: Vec<&str> = r["states"].as_array().unwrap().iter().map(|s| s.as_str().unwrap()).collect();
    assert!(r["ready_after_s"].is_number(), "controller never left SelfCheck: {states:?} (see {}/audit)", dir.display());
    assert!(states.contains(&"Hover"), "never reached Hover: {states:?}");
    assert!(r["hover_at_z"].as_f64().unwrap() > 2.0, "takeoff altitude (config 2.3 m)");
    assert!((r["z_after_hover"].as_f64().unwrap() - 2.3).abs() < 0.3, "holds altitude in Hover");
    assert!(r["final_z"].as_f64().unwrap() < 0.1 && r["armed_at_end"] == false, "landed and disarmed");
    assert!(r["eskf_err_p50_m"].as_f64().unwrap() < 0.05, "ESKF output tracks truth");
    assert!(r["setpoints"].as_u64().unwrap() > 100);
    // Planner path (Custom1, px4_local): the plan moves +x 1.5 m at 0.3 m/s.
    assert!(r["custom1"] == true && states.contains(&"Custom1"), "never entered Custom1: {states:?}");
    assert!((r["x_after_track"].as_f64().unwrap() - 1.5).abs() < 0.15, "follows the planner setpoints");
    assert!(r["track_err_max_m"].as_f64().unwrap() < 0.5, "tracking error bounded");
    assert!(r["arm_calls"].as_u64().unwrap() >= 1 && r["disarm_calls"].as_u64().unwrap() >= 1, "arm and disarm went through ros_io");
    assert!(r["mode_calls"].as_array().unwrap().iter().any(|m| m == "OFFBOARD"), "OFFBOARD requested through ros_io");
}
