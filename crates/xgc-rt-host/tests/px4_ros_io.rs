//! The unchanged px4_multirotor_controller (a ROS node, TRO config
//! `uav_nmpc.yaml`, tracking_backend px4_local) flies a stand-in vehicle on
//! the aggregator's ESKF module, with all ROS traffic through ros_io:
//!
//! plant truth -> VRPN + raw IMU -> ros_io -> ESKF module (memory) -> ros_io
//! -> /uav1/mavros/vision_pose/pose -> stand-in PX4 local position ->
//! controller -> /uav1/mavros/setpoint_raw/local -> stand-in PX4 -> plant.
//!
//! The flight: takeoff, Hover, the planner path (Custom1: a 10 Hz planner
//! setpoint on /uav1/alg/setpoint_raw/local moving +x 1.5 m), Hover, land.
//!
//! Needs ROS_PREFIX (ROS Noetic) and XGC2_WS (a catkin workspace where
//! px4_multirotor_controller is built, e.g. from a copy of the product
//! package). Without them the test prints why and passes.

mod common;

use std::path::PathBuf;
use std::process::Stdio;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

fn catkin_ws() -> Option<PathBuf> {
    std::env::var_os("XGC2_WS").map(PathBuf::from).filter(|w| {
        w.join("devel/setup.bash").is_file() && w.join("devel/lib/px4_multirotor_controller/px4_multirotor_controller_node").is_file()
    })
}

#[test]
fn unchanged_px4_controller_flies_on_the_aggregator_eskf_through_ros_io() {
    let (Some(prefix), Some(ws)) = (common::ros_prefix(), catkin_ws()) else {
        eprintln!("skipped: set ROS_PREFIX (Noetic) and XGC2_WS (catkin ws with px4_multirotor_controller built)");
        return;
    };
    let ros_io = common::ros_io_lib(&prefix).clone();
    let (eskf, _) = common::est_rigid_state();
    let port = 11412;
    let master = format!("http://127.0.0.1:{port}");
    let ros_home = common::scratch("ros-home-px4");
    let _core = common::Roscore::spawn({
        let mut c = common::ros_command(&prefix, "roscore");
        c.args(["-p", &port.to_string()]).env("ROS_HOME", &ros_home).stdout(Stdio::null()).stderr(Stdio::null());
        c
    });
    std::thread::sleep(Duration::from_secs(3));
    std::env::set_var("ROS_MASTER_URI", &master);
    std::env::set_var("ROS_IP", "127.0.0.1");

    let dir = common::scratch("px4-ros-io");
    let manifest = format!(
        r#"
[session]
id = "px4rosio"
node = "uav1"
roster = ["uav1"]
period_ms = 10
start_delay_ms = 100
run_for_ms = 120000

[transport]
kind = "loopback"

[audit]
dir = "audit"

[[channel]]
name = "imu"
qos = "state"
[[channel]]
name = "pose"
qos = "state"
[[channel]]
name = "rigid_state"
qos = "state"
[[channel]]
name = "vision_pose"
qos = "state"
[[channel]]
name = "estimate"
qos = "state"

[[plugin]]
name = "ros_io"
path = "{ros_io}"
trigger = "on_round"
config = {{ node_name = "xgc_ros_io_uav1", imu_topic = "/uav1/mavros/imu/data_raw", pose_topic = "/vrpn_client_node/uav1/pose", vision_pose_topic = "/uav1/mavros/vision_pose/pose", rigid_state_estimate_topic = "/uav1/alg/state_estimator/state" }}
bind = {{ imu = {{ channel = "imu" }}, pose = {{ channel = "pose" }}, vision_pose = {{ channel = "vision_pose", from = ["uav1"] }}, rigid_state_estimate = {{ channel = "estimate", from = ["uav1"] }} }}

[[plugin]]
name = "rigid-state"
path = "{eskf}"
trigger = "both"
config = {{ extrinsic_verified = true }}
bind = {{ imu = {{ channel = "imu", from = ["uav1"] }}, pose = {{ channel = "pose", from = ["uav1"] }}, rigid_state = {{ channel = "rigid_state" }}, vision_pose = {{ channel = "vision_pose" }}, estimate = {{ channel = "estimate" }} }}
"#,
        ros_io = ros_io.display(),
        eskf = eskf.display(),
    );
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(LoopbackBus::new())), Arc::new(WallClock::new(0)), HostOptions::default()).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };

    // The unchanged controller, launched with its own launch file and the
    // TRO configuration.
    let controller_log = std::fs::File::create(dir.join("controller.log")).unwrap();
    let controller = common::Roscore::spawn({
        let mut c = std::process::Command::new("bash");
        c.arg("-c")
            .arg(format!(
                "source '{}/setup.sh' && source '{}/devel/setup.bash' && exec roslaunch px4_multirotor_controller uav_nmpc_controller.launch ns:=uav1 world_boundary_json:=null",
                prefix.display(),
                ws.display()
            ))
            .env("PATH", format!("{}:{}", prefix.join("bin").display(), std::env::var("PATH").unwrap_or_default()))
            .env("ROS_HOME", &ros_home)
            .env("ROS_MASTER_URI", &master)
            .stdout(controller_log.try_clone().unwrap())
            .stderr(controller_log);
        c
    });

    // Optional: record the controller's inputs for its replay harness
    // (multirotor-controller test/replay). Set XGC_RECORD_BAG=/path/flight.bag.
    let recorder = std::env::var_os("XGC_RECORD_BAG").map(|bag| {
        common::Roscore::spawn({
            let mut c = common::ros_command(&prefix, "rosbag");
            c.args(["record", "-O"]).arg(&bag).args([
                "/uav1/alg/state_estimator/state",
                "/uav1/mavros/local_position/pose",
                "/uav1/mavros/local_position/velocity_local",
                "/uav1/mavros/imu/data",
                "/uav1/mavros/state",
                "/uav1/mavros/battery",
                "/uav1/pose",
                "/command",
                "/uav1/custom/statustext",
                "/uav1/mavros/setpoint_raw/local",
                "/uav1/alg/setpoint_raw/local",
            ]);
            c.env("ROS_HOME", &ros_home).env("ROS_MASTER_URI", &master).stdout(Stdio::null()).stderr(Stdio::null());
            c
        })
    });
    if recorder.is_some() {
        std::thread::sleep(Duration::from_secs(2));
    }

    let out = common::ros_command(&prefix, "python3")
        .arg(common::workspace_root().join("crates/xgc-rt-host/tests/ros/px4_standin.py"))
        .args(["4.0", "90.0", "5.0"])
        .env("ROS_HOME", &ros_home)
        .env("ROS_MASTER_URI", &master)
        .output()
        .unwrap();
    drop(recorder); // SIGINT: rosbag closes the bag
    drop(controller); // SIGINT to the launch group: roslaunch stops the node
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();

    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.lines().rev().find(|l| l.starts_with('{')).unwrap_or_else(|| {
        panic!("stand-in printed no summary\nstderr: {}", String::from_utf8_lossy(&out.stderr))
    });
    println!("stand-in: {line}");
    let r: serde_json::Value = serde_json::from_str(line).unwrap();
    for p in &summary.plugins {
        println!("module {} state={} domain={} steps={} last_error={:?}", p.name, p.state, p.domain_state, p.steps, p.last_error);
        assert!(p.last_error.is_none());
    }
    let states: Vec<&str> = r["states"].as_array().unwrap().iter().map(|s| s.as_str().unwrap()).collect();
    assert!(r["ready_after_s"].is_number(), "controller never left SelfCheck: {states:?} (see {}/controller.log)", dir.display());
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
}
