//! The unchanged px4_multirotor_controller (a ROS node, TRO config
//! `uav_nmpc.yaml`) flies a stand-in vehicle on the aggregator's ESKF module,
//! with all ROS traffic through ros_io:
//!
//! plant truth -> VRPN + raw IMU -> ros_io -> ESKF module (memory) -> ros_io
//! -> /uav1/mavros/vision_pose/pose -> stand-in PX4 local position ->
//! controller -> /uav1/mavros/setpoint_raw/{local,attitude} -> stand-in PX4
//! -> plant.
//!
//! One flight per tracking backend; each takes off, hovers, flies Custom1,
//! hovers and lands:
//! - px4_local (the TRO configuration): Custom1 follows a 10 Hz planner
//!   setpoint on /uav1/alg/setpoint_raw/local moving +x 1.5 m.
//! - dfbc and nmpc: Custom1 requests an analytic reference (circle entry) from
//!   the unchanged multirotor_reference_trajectory node and tracks it with
//!   body-rate + thrust targets; the stand-in flies attitude and publishes
//!   its hover thrust as the estimator would.
//!
//! Needs ROS_PREFIX (ROS Noetic) and XGC2_WS (a catkin workspace where
//! px4_multirotor_controller and multirotor_reference_trajectory are built,
//! e.g. from copies of the product packages). Without them the test prints
//! why and passes. XGC_RECORD_BAG_DIR=/dir records each flight's controller
//! inputs as <dir>/px4_flight_<backend>.bag for the controller's replay
//! harness.

mod common;

use std::path::{Path, PathBuf};
use std::process::Stdio;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

// One flight at a time (one heavy process; the flights share port numbers).
static FLIGHT: Mutex<()> = Mutex::new(());

fn catkin_ws() -> Option<PathBuf> {
    std::env::var_os("XGC2_WS").map(PathBuf::from).filter(|w| {
        w.join("devel/setup.bash").is_file() && w.join("devel/lib/px4_multirotor_controller/px4_multirotor_controller_node").is_file()
    })
}

/// A command run with the ROS environment and the catkin workspace sourced.
fn ws_command(prefix: &Path, ws: &Path, command: &str, ros_home: &Path, master: &str) -> std::process::Command {
    let mut c = std::process::Command::new("bash");
    c.arg("-c")
        .arg(format!("source '{}/setup.sh' && source '{}/devel/setup.bash' && exec {command}", prefix.display(), ws.display()))
        .env("PATH", format!("{}:{}", prefix.join("bin").display(), std::env::var("PATH").unwrap_or_default()))
        .env("ROS_HOME", ros_home)
        .env("ROS_MASTER_URI", master);
    c
}

fn fly(backend: &str) {
    let (Some(prefix), Some(ws)) = (common::ros_prefix(), catkin_ws()) else {
        eprintln!("skipped: set ROS_PREFIX (Noetic) and XGC2_WS (catkin ws with px4_multirotor_controller built)");
        return;
    };
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let reference = backend != "px4_local";
    let ros_io = common::ros_io_lib(&prefix).clone();
    let (eskf, _) = common::est_rigid_state();
    let port = 11412;
    let master = format!("http://127.0.0.1:{port}");
    let ros_home = common::scratch(&format!("ros-home-px4-{backend}"));
    let _core = common::Roscore::spawn({
        let mut c = common::ros_command(&prefix, "roscore");
        c.args(["-p", &port.to_string()]).env("ROS_HOME", &ros_home).stdout(Stdio::null()).stderr(Stdio::null());
        c
    });
    std::thread::sleep(Duration::from_secs(3));
    std::env::set_var("ROS_MASTER_URI", &master);
    std::env::set_var("ROS_IP", "127.0.0.1");

    let dir = common::scratch(&format!("px4-ros-io-{backend}"));
    let manifest = format!(
        r#"
[session]
id = "px4rosio"
node = "uav1"
roster = ["uav1"]
period_ms = 10
start_delay_ms = 100
run_for_ms = 180000

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

    // The unchanged controller with its own launch file and the TRO
    // configuration; for dfbc/nmpc only the backend and the activated
    // reference (circle entry instead of the torus knot, which needs more
    // height than a 2.3 m takeoff) change.
    let controller_root = std::env::var_os("PX4_CONTROLLER_ROOT").map(PathBuf::from)
        .unwrap_or_else(|| ws.join("src/px4_multirotor_controller"));
    let yaml = std::fs::read_to_string(controller_root.join("config/uav_nmpc.yaml")).unwrap();
    assert!(yaml.contains("tracking_backend: px4_local\n") && yaml.contains("  reference_analytic_type: 9\n"));
    let yaml = yaml
        .replace("tracking_backend: px4_local\n", &format!("tracking_backend: {backend}\n"))
        .replace("  reference_analytic_type: 9\n", "  reference_analytic_type: 3\n");
    let config_file = dir.join("uav_nmpc.yaml");
    std::fs::write(&config_file, yaml).unwrap();
    let controller_log = std::fs::File::create(dir.join("controller.log")).unwrap();
    let controller = common::Roscore::spawn({
        let mut c = ws_command(
            &prefix,
            &ws,
            &format!(
                "roslaunch px4_multirotor_controller uav_nmpc_controller.launch ns:=uav1 world_boundary_json:=null config_file:={}",
                config_file.display()
            ),
            &ros_home,
            &master,
        );
        c.stdout(controller_log.try_clone().unwrap()).stderr(controller_log);
        c
    });
    let reference_node = reference.then(|| {
        let log = std::fs::File::create(dir.join("reference.log")).unwrap();
        common::Roscore::spawn({
            let mut c = ws_command(&prefix, &ws, "roslaunch multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch ns:=uav1", &ros_home, &master);
            c.stdout(log.try_clone().unwrap()).stderr(log);
            c
        })
    });

    // Optional: record the controller's inputs for its replay harness.
    let recorder = std::env::var_os("XGC_RECORD_BAG_DIR").map(|d| {
        let bag = PathBuf::from(d).join(format!("px4_flight_{backend}.bag"));
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
                "/uav1/mavros/setpoint_raw/attitude",
                "/uav1/alg/setpoint_raw/local",
                "/uav1/hover_thrust/estimate_state",
                "/uav1/alg/multirotor_reference_trajectory/request/analytic",
                "/uav1/alg/multirotor_reference_trajectory/active/analytic",
                "/uav1/alg/multirotor_reference_trajectory/active/sampled",
            ]);
            c.env("ROS_HOME", &ros_home).env("ROS_MASTER_URI", &master).stdout(Stdio::null()).stderr(Stdio::null());
            c
        })
    });
    if recorder.is_some() {
        std::thread::sleep(Duration::from_secs(2));
    }

    let standin = common::workspace_root().join("crates/xgc-rt-host/tests/ros/px4_standin.py");
    let out = ws_command(
        &prefix,
        &ws,
        &format!("python3 '{}' 4.0 120.0 {} {}", standin.display(), if reference { "12.0" } else { "5.0" }, if reference { "reference" } else { "planner" }),
        &ros_home,
        &master,
    )
    .output()
    .unwrap();
    drop(recorder); // SIGINT: rosbag closes the bag
    drop(reference_node);
    drop(controller); // SIGINT to the launch group: roslaunch stops the node
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();

    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.lines().rev().find(|l| l.starts_with('{')).unwrap_or_else(|| {
        panic!("stand-in printed no summary\nstderr: {}", String::from_utf8_lossy(&out.stderr))
    });
    println!("stand-in ({backend}): {line}");
    let r: serde_json::Value = serde_json::from_str(line).unwrap();
    for p in &summary.plugins {
        println!("module {} state={} domain={} steps={} last_error={:?}", p.name, p.state, p.domain_state, p.steps, p.last_error);
        assert!(p.last_error.is_none());
    }
    let states: Vec<&str> = r["states"].as_array().unwrap().iter().map(|s| s.as_str().unwrap()).collect();
    assert!(r["ready_after_s"].is_number(), "controller never left SelfCheck: {states:?} (see {}/controller.log)", dir.display());
    assert!(states.contains(&"Hover"), "never reached Hover: {states:?}");
    assert!(r["hover_at_z"].as_f64().unwrap() > 2.0, "takeoff altitude (config 2.3 m)");
    assert!(r["custom1"] == true && states.contains(&"Custom1"), "never entered Custom1: {states:?} (see {}/controller.log)", dir.display());
    if reference {
        // The activated circle entry: out to a 3 m circle at 3 m height.
        assert!(r["custom1_held"] == true, "left Custom1 early: {states:?}");
        assert!(r["attitude_setpoints"].as_u64().unwrap() > 500, "body-rate + thrust targets");
        assert!(r["reach_xy_m"].as_f64().unwrap() > 1.0, "follows the activated reference");
        assert!(r["z_range_m"][0].as_f64().unwrap() > 1.0, "stays airborne in Custom1");
    } else {
        assert!((r["x_after_track"].as_f64().unwrap() - 1.5).abs() < 0.15, "follows the planner setpoints");
        assert!(r["track_err_max_m"].as_f64().unwrap() < 0.5, "tracking error bounded");
    }
    assert!((r["z_after_hover"].as_f64().unwrap() - 2.3).abs() < 0.3 || reference, "holds altitude in Hover");
    assert!(r["final_z"].as_f64().unwrap() < 0.1 && r["armed_at_end"] == false, "landed and disarmed");
    assert!(r["eskf_err_p50_m"].as_f64().unwrap() < 0.05, "ESKF output tracks truth");
    assert!(r["setpoints"].as_u64().unwrap() > 100);
}

#[test]
fn unchanged_px4_controller_flies_px4_local_on_the_aggregator_eskf() {
    fly("px4_local");
}

#[test]
fn unchanged_px4_controller_flies_dfbc_on_the_aggregator_eskf() {
    fly("dfbc");
}

#[test]
fn unchanged_px4_controller_flies_nmpc_on_the_aggregator_eskf() {
    fly("nmpc");
}
