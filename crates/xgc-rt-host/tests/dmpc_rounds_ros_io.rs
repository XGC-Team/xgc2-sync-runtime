//! The DMPC planner's round clock as a local facade: one robot's aggregator
//! (ros_io + dmpc-rounds with the mission phase) publishes
//! formation_generator/FormationTick on the robot's own
//! /uav1/formation/mission_tick at E0 + k·P, from its own clock, with the
//! mission phase derived from data (operator /command, the controller's
//! /uav1/custom/statustext). No sync coordinator and no station clock run,
//! and the roster's other robot (uav2) never comes up: robot 1 runs anyway.
//!
//! Needs ROS_PREFIX (ROS Noetic). Without it the test prints why and passes.

mod common;

use std::process::Stdio;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

#[test]
fn the_planner_gets_its_round_clock_from_its_own_aggregator() {
    let Some(prefix) = common::ros_prefix() else {
        eprintln!("skipped: set ROS_PREFIX (Noetic)");
        return;
    };
    let ros_io = common::ros_io_lib(&prefix).clone();
    let dmpc = common::lib("dmpc_rounds");
    let port = 11415;
    let master = format!("http://127.0.0.1:{port}");
    let ros_home = common::scratch("ros-home-dmpc-beat");
    let _core = common::Roscore::spawn({
        let mut c = common::ros_command(&prefix, "roscore");
        c.args(["-p", &port.to_string()]).env("ROS_HOME", &ros_home).stdout(Stdio::null()).stderr(Stdio::null());
        c
    });
    std::thread::sleep(Duration::from_secs(3));
    std::env::set_var("ROS_MASTER_URI", &master);
    std::env::set_var("ROS_IP", "127.0.0.1");

    let clock: Arc<dyn Clock> = Arc::new(WallClock::new(0));
    let period_ms = 100;
    // An agreed absolute epoch: every robot of a fleet uses the same one.
    let e0 = (clock.now() / 1_000_000_000 + 2) * 1_000_000_000;
    let dir = common::scratch("dmpc-beat");
    let manifest = format!(
        r#"
[session]
id = "beat"
node = "uav1"
roster = ["uav1", "uav2"]
period_ms = {period_ms}
epoch_ns = {e0}
run_for_ms = 20000

[transport]
kind = "loopback"

[audit]
dir = "audit"

[[channel]]
name = "own_plan"
qos = "control"
[[channel]]
name = "dmpc/plan"
qos = "control"
[[channel]]
name = "neighbor_plans"
qos = "control"
[[channel]]
name = "sync_trigger"
qos = "control"
[[channel]]
name = "command"
qos = "event"
[[channel]]
name = "own_state"
qos = "state"
[[channel]]
name = "dmpc/state"
qos = "state"
[[channel]]
name = "formation_tick"
qos = "control"

[[plugin]]
name = "ros_io"
path = "{ros_io}"
trigger = "both"
wake_ms = 2.0
config = {{ node_name = "xgc_ros_io_uav1", slice_ms = 2.0, own_plan_topic = "/uav1/formation/assumed_trajectories", neighbor_plans_topic = "/uav1/formation/neighbor_trajectories", command_topic = "/command", controller_state_topic = "/uav1/custom/statustext", formation_tick_topic = "/uav1/formation/mission_tick" }}
bind = {{ own_plan = {{ channel = "own_plan" }}, neighbor_plans = {{ channel = "neighbor_plans", from = ["uav1"] }}, command = {{ channel = "command" }}, controller_state = {{ channel = "own_state" }}, formation_tick = {{ channel = "formation_tick", from = ["uav1"] }} }}

[[plugin]]
name = "dmpc"
path = "{dmpc}"
trigger = "both"
config = {{ uav_id = 1, participant_ids = [1], robots = ["uav1"], robot = "uav1" }}
bind = {{ own_plan = {{ channel = "own_plan", from = ["uav1"] }}, plan_in = {{ channel = "dmpc/plan", from = ["uav2"] }}, plan_out = {{ channel = "dmpc/plan" }}, neighbor_plans = {{ channel = "neighbor_plans" }}, sync_trigger = {{ channel = "sync_trigger" }}, command = {{ channel = "command", from = ["uav1"] }}, own_state = {{ channel = "own_state", from = ["uav1"] }}, state_in = {{ channel = "dmpc/state", from = ["uav2"] }}, state_out = {{ channel = "dmpc/state" }}, formation_tick = {{ channel = "formation_tick" }} }}
"#,
        ros_io = ros_io.display(),
        dmpc = dmpc,
    );
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(LoopbackBus::new())), clock.clone(), HostOptions::default()).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_nanos((e0 - clock.now()).max(0) as u64));
    // Python classes of the verbatim .msg copies (ros_io's), for the checker.
    let gen = common::workspace_root().join("target/plugin-tests/ros/py-gen");
    let msg = common::workspace_root().join("plugins/ros-io/msg");
    let genpy = prefix.join("lib/genpy/genmsg_py.py");
    for (pkg, name) in [("periodic_sync", "SyncTrigger"), ("formation_generator", "FormationTick")] {
        let out_dir = gen.join(pkg).join("msg");
        let ok = common::ros_command(&prefix, "python3")
            .arg(&genpy)
            .arg(msg.join(pkg).join(format!("{name}.msg")))
            .args(["-p", pkg, "-o"])
            .arg(&out_dir)
            .arg(format!("-Iperiodic_sync:{}", msg.join("periodic_sync").display()))
            .arg(format!("-Iformation_generator:{}", msg.join("formation_generator").display()))
            .arg(format!("-Istd_msgs:{}", prefix.join("share/std_msgs/msg").display()))
            .status()
            .unwrap()
            .success();
        assert!(ok, "genpy {pkg}/{name}");
        let init = common::ros_command(&prefix, "python3").arg(&genpy).args(["--initpy", "-p", pkg, "-o"]).arg(&out_dir).status().unwrap();
        assert!(init.success());
        std::fs::write(gen.join(pkg).join("__init__.py"), "").unwrap();
    }
    let out = common::ros_command(&prefix, "python3")
        .arg(common::workspace_root().join("crates/xgc-rt-host/tests/ros/mission_tick_check.py"))
        .env("ROS_HOME", &ros_home)
        .env("ROS_MASTER_URI", &master)
        .env("PYTHONPATH", format!("{}:{}", gen.display(), std::env::var("PYTHONPATH").unwrap_or_default()))
        .output()
        .unwrap();
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.lines().rev().find(|l| l.starts_with('{')).unwrap_or_else(|| {
        panic!("checker printed no summary\nstderr: {}", String::from_utf8_lossy(&out.stderr))
    });
    for p in &summary.plugins {
        println!("module {} state={} domain={} steps={} last_error={:?}", p.name, p.state, p.domain_state, p.steps, p.last_error);
        assert!(p.last_error.is_none());
    }
    let r: serde_json::Value = serde_json::from_str(line).unwrap();
    let ticks = r["ticks"].as_array().unwrap();
    assert!(ticks.len() >= 30, "{} ticks", ticks.len());
    let period = period_ms as f64 * 1e-3;
    let mut rolling_from = None;
    let mut last = None;
    for t in ticks {
        let (k, at, rolling, mt, rx) = (t[0].as_u64().unwrap(), t[1].as_f64().unwrap(), t[2].as_bool().unwrap(), t[3].as_f64().unwrap(), t[4].as_f64().unwrap());
        // The round's trigger time is its scheduled absolute time, E0 + k·P.
        assert!((at - (e0 as f64 * 1e-9 + k as f64 * period)).abs() < 1e-6, "round {k} at {at}");
        assert!(rx - at >= 0.0 && rx - at < 0.02, "round {k} delivered {:.4} s after its time", rx - at);
        if let Some(prev) = last {
            assert_eq!(k, prev + 1, "consecutive rounds");
        }
        last = Some(k);
        if rolling && rolling_from.is_none() {
            rolling_from = Some(k);
        }
        if let Some(k0) = rolling_from {
            assert!(rolling, "round {k}: stopped rolling");
            assert!((mt - (k - k0) as f64 * period).abs() < 1e-6, "round {k}: mission_time {mt}");
        }
    }
    assert!(rolling_from.is_some(), "never rolled");
    println!("{} local rounds, rolling from round {}", ticks.len(), rolling_from.unwrap());
}
