//! The reference trajectory generator as an aggregator module, live: one
//! aggregator holds ros_io and ref-trajectory (session time, 10 ms rounds,
//! as the node's 100 Hz loop). Next to it runs the unchanged
//! multirotor_reference_trajectory node (its own launch file). ref_compare.py
//! sends the same requests to both (module: /uav1, node: /uav2): analytic
//! curves, a sampled reference, two MINCO waypoint plans, reset, expiry. Per
//! trajectory, the active references each side publishes must be equal field
//! for field and bit for bit, except the receive-time-dependent header stamps
//! and start times; the status sequences must match.
//!
//! Needs ROS_PREFIX (ROS Noetic), XGC2_WS (a catkin workspace where
//! multirotor_reference_trajectory and its msgs are built) and
//! REF_CORE_LIB_DIR (plus the other build-ref-trajectory.sh variables).
//! Without them the test prints why and passes.

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
        w.join("devel/setup.bash").is_file()
            && w.join("devel/lib/multirotor_reference_trajectory/multirotor_reference_trajectory_node").is_file()
    })
}

#[test]
fn ref_trajectory_module_publishes_what_the_ros_node_publishes() {
    let (Some(prefix), Some(ws)) = (common::ros_prefix(), catkin_ws()) else {
        eprintln!("skipped: set ROS_PREFIX (Noetic) and XGC2_WS (catkin ws with multirotor_reference_trajectory built)");
        return;
    };
    if std::env::var_os("REF_CORE_LIB_DIR").is_none() {
        eprintln!("skipped: set REF_CORE_LIB_DIR (dir with libmultirotor_reference_trajectory_core.so)");
        return;
    }
    let ros_io = common::ros_io_lib(&prefix).clone();
    let module = common::ref_trajectory_lib(&prefix).clone();
    let port = 11414;
    let master = format!("http://127.0.0.1:{port}");
    let ros_home = common::scratch("ros-home-ref");
    let _core = common::Roscore::spawn({
        let mut c = common::ros_command(&prefix, "roscore");
        c.args(["-p", &port.to_string()]).env("ROS_HOME", &ros_home).stdout(Stdio::null()).stderr(Stdio::null());
        c
    });
    std::thread::sleep(Duration::from_secs(3));
    std::env::set_var("ROS_MASTER_URI", &master);
    std::env::set_var("ROS_IP", "127.0.0.1");

    let dir = common::scratch("ref-trajectory-ros-io");
    let base = "/uav1/alg/multirotor_reference_trajectory";
    let manifest = format!(
        r#"
[session]
id = "refrosio"
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
name = "analytic"
qos = "event"
[[channel]]
name = "waypoint"
qos = "event"
[[channel]]
name = "sampled"
qos = "event"
[[channel]]
name = "reset"
qos = "event"
[[channel]]
name = "status"
qos = "state"
[[channel]]
name = "active_analytic"
qos = "state"
[[channel]]
name = "active_polynomial"
qos = "state"
[[channel]]
name = "active_sampled"
qos = "state"

[[plugin]]
name = "ros_io"
path = "{ros_io}"
trigger = "on_round"
config = {{ node_name = "xgc_ros_io_uav1", ref_analytic_topic = "{base}/request/analytic", ref_waypoint_topic = "{base}/request/waypoint", ref_sampled_topic = "{base}/request/sampled", ref_reset_topic = "{base}/reset", ref_status_topic = "{base}/status", ref_active_analytic_topic = "{base}/active/analytic", ref_active_polynomial_topic = "{base}/active/polynomial", ref_active_sampled_topic = "{base}/active/sampled" }}
bind = {{ ref_analytic = {{ channel = "analytic" }}, ref_waypoint = {{ channel = "waypoint" }}, ref_sampled = {{ channel = "sampled" }}, ref_reset = {{ channel = "reset" }}, ref_status = {{ channel = "status", from = ["uav1"] }}, ref_active_analytic = {{ channel = "active_analytic", from = ["uav1"] }}, ref_active_polynomial = {{ channel = "active_polynomial", from = ["uav1"] }}, ref_active_sampled = {{ channel = "active_sampled", from = ["uav1"] }} }}

[[plugin]]
name = "ref-trajectory"
path = "{module}"
trigger = "on_round"
config = {{ time_source = "session" }}
bind = {{ analytic = {{ channel = "analytic", from = ["uav1"] }}, waypoint = {{ channel = "waypoint", from = ["uav1"] }}, sampled = {{ channel = "sampled", from = ["uav1"] }}, reset = {{ channel = "reset", from = ["uav1"] }}, status = {{ channel = "status" }}, active_analytic = {{ channel = "active_analytic" }}, active_polynomial = {{ channel = "active_polynomial" }}, active_sampled = {{ channel = "active_sampled" }} }}
"#,
        ros_io = ros_io.display(),
        module = module.display(),
    );
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(LoopbackBus::new())), Arc::new(WallClock::new(0)), HostOptions::default()).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };

    // The unchanged node, with its own launch file and configuration.
    let node_log = std::fs::File::create(dir.join("node.log")).unwrap();
    let node = common::Roscore::spawn({
        let mut c = std::process::Command::new("bash");
        c.arg("-c")
            .arg(format!(
                "source '{}/setup.sh' && source '{}/devel/setup.bash' && exec roslaunch multirotor_reference_trajectory uav_multirotor_reference_trajectory.launch ns:=uav2",
                prefix.display(),
                ws.display()
            ))
            .env("PATH", format!("{}:{}", prefix.join("bin").display(), std::env::var("PATH").unwrap_or_default()))
            .env("ROS_HOME", &ros_home)
            .env("ROS_MASTER_URI", &master)
            .stdout(node_log.try_clone().unwrap())
            .stderr(node_log);
        c
    });

    let out = std::process::Command::new("bash")
        .arg("-c")
        .arg(format!(
            "source '{}/setup.sh' && source '{}/devel/setup.bash' && exec python3 '{}'",
            prefix.display(),
            ws.display(),
            common::workspace_root().join("crates/xgc-rt-host/tests/ros/ref_compare.py").display()
        ))
        .env("PATH", format!("{}:{}", prefix.join("bin").display(), std::env::var("PATH").unwrap_or_default()))
        .env("ROS_HOME", &ros_home)
        .env("ROS_MASTER_URI", &master)
        .output()
        .unwrap();
    drop(node);
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();

    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.lines().rev().find(|l| l.starts_with('{')).unwrap_or_else(|| {
        panic!("ref_compare printed no summary\nstderr: {}", String::from_utf8_lossy(&out.stderr))
    });
    println!("ref_compare: {line}");
    let r: serde_json::Value = serde_json::from_str(line).unwrap();
    for p in &summary.plugins {
        println!("module {} state={} domain={} steps={} consumed={} published={} last_error={:?}", p.name, p.state, p.domain_state, p.steps, p.consumed, p.published, p.last_error);
        assert!(p.last_error.is_none());
    }
    assert_eq!(r["ready"], true, "both sides up (see {}/node.log)", dir.display());
    let sent = r["sent"].as_array().unwrap();
    assert_eq!(sent.len(), 7, "all active references seen");
    assert!(sent.iter().all(|s| s["both"] == true), "every request activated on both sides: {sent:?}");
    assert_eq!(r["compared"].as_u64().unwrap(), 7);
    assert!(r["diffs"].as_object().unwrap().is_empty(), "module and node differ: {}", r["diffs"]);
    assert_eq!(r["states_module"], r["states_node"], "status sequences");
    assert!(r["polynomial_coeffs"].as_u64().unwrap() > 0);
}
