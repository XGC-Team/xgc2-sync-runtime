//! ros_io against a real ROS master: MAVROS-style IMU and VRPN pose topics
//! go through ros_io into the ESKF module (memory handoff inside one
//! aggregator), and the ESKF's vision pose comes back out on
//! /mavros/vision_pose/pose.
//!
//! Needs ROS Noetic: set ROS_PREFIX (e.g. /opt/ros/noetic or a RoboStack
//! environment). Without it the test prints why and passes.

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
fn imu_and_vrpn_topics_run_the_eskf_module_and_its_vision_pose_goes_back_to_ros() {
    let Some(prefix) = common::ros_prefix() else {
        eprintln!("skipped: set ROS_PREFIX to a ROS Noetic install to run the ros_io test");
        return;
    };
    let ros_io = common::ros_io_lib(&prefix).clone();
    let (eskf, _) = common::est_rigid_state();
    let port = 11411;
    let master = format!("http://127.0.0.1:{port}");
    let ros_home = common::scratch("ros-home");
    let _core = common::Roscore::spawn({
        let mut c = common::ros_command(&prefix, "roscore");
        c.args(["-p", &port.to_string()]).env("ROS_HOME", &ros_home).stdout(Stdio::null()).stderr(Stdio::null());
        c
    });
    std::thread::sleep(Duration::from_secs(3));
    // ros::init in this process reads the master from the environment.
    std::env::set_var("ROS_MASTER_URI", &master);
    std::env::set_var("ROS_IP", "127.0.0.1");

    let dir = common::scratch("ros-io");
    let manifest = format!(
        r#"
[session]
id = "rosio"
node = "uav1"
roster = ["uav1"]
period_ms = 10
start_delay_ms = 100
run_for_ms = 7000

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
config = {{ node_name = "xgc_ros_io_uav1", imu_topic = "/mavros/imu/data_raw", pose_topic = "/vrpn_client_node/uav1/pose", vision_pose_topic = "/mavros/vision_pose/pose", rigid_state_estimate_topic = "/rigid_state_estimator/state" }}
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
    let player = common::ros_command(&prefix, "python3")
        .arg(common::workspace_root().join("crates/xgc-rt-host/tests/ros/eskf_chain.py"))
        .arg("4.0")
        .env("ROS_HOME", &ros_home)
        .env("ROS_MASTER_URI", &master)
        .env("ROS_IP", "127.0.0.1")
        .output()
        .unwrap();
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    let stdout = String::from_utf8_lossy(&player.stdout);
    println!("player: {stdout}{}", String::from_utf8_lossy(&player.stderr));
    for p in &summary.plugins {
        println!("{p:?}");
    }
    let result: serde_json::Value = serde_json::from_str(stdout.lines().last().unwrap()).unwrap();
    let (ros_io_s, eskf_s) = (&summary.plugins[0], &summary.plugins[1]);
    assert!(ros_io_s.published > 1000, "IMU and pose samples came in from ROS");
    let vision = result["vision"].as_u64().unwrap();
    assert!(vision > 60, "vision poses reached ROS: {vision}");
    let last = result["last"].as_array().unwrap();
    let t = last[0].as_f64().unwrap();
    let (x, y, z) = (last[1].as_f64().unwrap(), last[2].as_f64().unwrap(), last[3].as_f64().unwrap());
    let err = ((x - 0.5 * (0.8 * t).sin()).powi(2) + (y - (0.5 * (0.8 * t).cos() - 0.5)).powi(2) + (z - 1.0).powi(2)).sqrt();
    println!("last vision pose error {err:.4} m");
    assert!(err < 0.05, "vision pose error {err}");
    // The full estimate reached ROS as the node's own message type, most of
    // it from the Running state.
    assert_eq!(result["estimate_type"], "rigid_state_estimator_msgs/RigidStateEstimate");
    assert_eq!(result["estimate_md5"], "7424c514b9ac1ad1576e5e21d46362a3");
    let (estimates, running) = (result["estimates"].as_u64().unwrap(), result["running"].as_u64().unwrap());
    assert!(estimates > 300 && running * 2 > estimates, "{estimates} estimates, {running} Running");
}

#[test]
fn vendored_ros_messages_equal_their_originals() {
    let root = common::workspace_root();
    let products = root.join("../..");
    let repos = products.join("../../..");
    for (copy, original) in [
        ("rigid_state_estimator_msgs/RigidStateEstimate.msg", products.join("ros1/common/ros1-msgs/rigid_state_estimator_msgs/msg/RigidStateEstimate.msg")),
        ("formation_generator/AssumedTrajectory.msg", repos.join("academic/ros1_ws/src/planner/formation_generator/msg/AssumedTrajectory.msg")),
        ("periodic_sync/SyncTrigger.msg", repos.join("academic/ros1_ws/src/communication/periodic_sync/msg/SyncTrigger.msg")),
        (
            "unicycle_reference_trajectory_msgs/PlanarPvaReference.msg",
            products.join("ros1/common/ros1-msgs/unicycle_reference_trajectory_msgs/msg/PlanarPvaReference.msg"),
        ),
    ]
    .into_iter()
    .chain(["SceneSnapshot", "SceneObstacle", "ScenePart", "SceneGeometry", "SceneState", "SceneObstacleState"].map(|m| {
        (
            Box::leak(format!("xgc2_geometry_msgs/{m}.msg").into_boxed_str()) as &str,
            products.join(format!("ros1/simulator/convex_geometry/xgc2_geometry_msgs/msg/{m}.msg")),
        )
    }))
    {
        let Ok(want) = std::fs::read(&original) else {
            eprintln!("skipped {copy}: {} is not checked out", original.display());
            continue;
        };
        assert_eq!(std::fs::read(root.join("plugins/ros-io/msg").join(copy)).unwrap(), want, "{copy} drifted from {}", original.display());
    }
}

/// ros_io's shared-scene payloads decode with the planner's codec (the one
/// plan-dmpc uses) to exactly the message fields. Needs ROS_PREFIX and
/// FORMATION_GENERATOR_ROOT (the academic formation_generator package).
#[test]
fn scene_payloads_decode_with_the_planner_codec() {
    let (Some(prefix), Some(fg)) = (
        common::ros_prefix(),
        std::env::var_os("FORMATION_GENERATOR_ROOT").map(std::path::PathBuf::from).filter(|p| p.is_dir()),
    ) else {
        eprintln!("skipped: set ROS_PREFIX and FORMATION_GENERATOR_ROOT");
        return;
    };
    let gen = common::ros_io_lib(&prefix).parent().unwrap().join("ros-io-gen");
    let out = common::workspace_root().join("target/plugin-tests/ros");
    let tool = out.join("scene_wire_check");
    let conda_cxx = prefix.join("bin/x86_64-conda-linux-gnu-c++");
    let cxx = if conda_cxx.is_file() { conda_cxx } else { std::path::PathBuf::from("c++") };
    let eigen = std::env::var_os("EIGEN_INCLUDE").map(std::path::PathBuf::from).unwrap_or_else(|| prefix.join("include/eigen3"));
    let status = std::process::Command::new(cxx)
        .args(["-std=c++17", "-O2"])
        .arg("-I").arg(common::workspace_root().join("abi/include"))
        .arg("-I").arg(common::workspace_root().join("plugins/ros-io"))
        .arg("-I").arg(&gen)
        .arg("-I").arg(fg.join("include"))
        .arg("-I").arg(fg.join("../../common/convex_geometry/include"))
        .arg("-isystem").arg(&eigen)
        .arg("-isystem").arg(prefix.join("include"))
        .arg(common::workspace_root().join("crates/xgc-rt-host/tests/ros/scene_wire_check.cpp"))
        .arg("-o").arg(&tool)
        .arg("-L").arg(prefix.join("lib")).arg(format!("-Wl,-rpath,{}", prefix.join("lib").display()))
        .args(["-lrostime", "-lcpp_common"])
        .status()
        .unwrap();
    assert!(status.success(), "building scene_wire_check failed");
    let run = std::process::Command::new(&tool).output().unwrap();
    println!("{}", String::from_utf8_lossy(&run.stdout));
    assert!(run.status.success(), "ros_io scene payloads differ from the planner codec");
}
