//! The PX4 controller flies as an aggregator module: ctl-px4 (the ROS-free
//! controller core) replaces the px4_multirotor_controller ROS node. One
//! aggregator holds ros_io, the ESKF module and ctl-px4; the only ROS
//! traffic is ros_io's, and there is no controller node or roslaunch.
//!
//! plant truth -> VRPN + raw IMU -> ros_io -> ESKF (memory) -> ctl-px4
//! (memory); ESKF -> ros_io -> /uav1/mavros/vision_pose/pose -> stand-in PX4
//! local position -> ros_io -> ctl-px4 -> ros_io ->
//! /uav1/mavros/setpoint_raw/{local,attitude} and cmd/command, set_mode ->
//! stand-in PX4 -> plant. The same stand-in (px4_standin.py) and pass
//! criteria as px4_ros_io.rs, which flies the unchanged ROS node, one flight
//! per tracking backend:
//! - px4_local: Custom1 follows the planner's /uav1/alg/setpoint_raw/local,
//!   which ros_io carries into ctl-px4's alg_setpoint input.
//! - dfbc and nmpc: the aggregator also holds ref-trajectory (the reference
//!   generator, in place of its ROS node). Custom1's reference request goes
//!   from ctl-px4 to ref-trajectory, and the active reference back, in
//!   memory; the stand-in's hover-thrust estimate comes in through ros_io.
//!
//! ctl-px4 runs with time_source = "session" at 1 ms rounds, like the node's
//! 1 kHz loop, with the TRO configuration (uav_nmpc.yaml; for dfbc/nmpc the
//! activated reference is the circle entry, as in px4_ros_io.rs).
//!
//! The two native-hover cases add est-hover-thrust as a fifth real module.
//! The plant provides actuator feedback and publishes no hover estimate; the
//! observer records and checks the estimator output consumed by ctl-px4.
//! This remains a software plant validation, not PX4 or physical flight.
//!
//! Needs ROS_PREFIX (ROS Noetic), XGC2_WS (for the stand-in's messages),
//! PX4_CORE_LIB_DIR and, for dfbc/nmpc, REF_CORE_LIB_DIR (plus the other
//! build-ctl-px4.sh / build-ref-trajectory.sh variables). Without them the
//! test prints why and passes.

mod common;

use std::path::PathBuf;
use std::process::Stdio;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use xgc_rt_core::audit::NullAudit;
use xgc_rt_core::clock::WallClock;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

// One flight at a time (one heavy process; the flights share port numbers).
static FLIGHT: Mutex<()> = Mutex::new(());

fn fly(backend: &str, native_hover: bool) {
    let reference = backend != "px4_local";
    let case_name = if native_hover { format!("{backend}-native-hover") } else { backend.to_string() };
    let (Some(prefix), Some(ws)) = (common::ros_prefix(), std::env::var_os("XGC2_WS").map(PathBuf::from)) else {
        eprintln!("skipped: set ROS_PREFIX (Noetic) and XGC2_WS");
        return;
    };
    if std::env::var_os("PX4_CORE_LIB_DIR").is_none() || (reference && std::env::var_os("REF_CORE_LIB_DIR").is_none()) {
        eprintln!("skipped: set PX4_CORE_LIB_DIR (and REF_CORE_LIB_DIR for dfbc/nmpc)");
        return;
    }
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let ros_io = common::ros_io_lib(&prefix).clone();
    let ctl = common::ctl_px4_lib(&prefix).clone();
    let (eskf, _) = common::est_rigid_state();
    let port = 11413;
    let master = format!("http://127.0.0.1:{port}");
    let ros_home = common::scratch(&format!("ros-home-ctl-px4-{case_name}"));
    let _core = common::Roscore::spawn({
        let mut c = common::ros_command(&prefix, "roscore");
        c.args(["-p", &port.to_string()]).env("ROS_HOME", &ros_home).stdout(Stdio::null()).stderr(Stdio::null());
        c
    });
    std::thread::sleep(Duration::from_secs(3));
    std::env::set_var("ROS_MASTER_URI", &master);
    std::env::set_var("ROS_IP", "127.0.0.1");

    let dir = common::scratch(&format!("ctl-px4-ros-io-{case_name}"));
    let mut channels = vec![
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
        ("attitude_rate", "control"),
        ("fcu_request", "event"),
        ("status", "state"),
        ("hover_thrust", "state"),
    ];
    if reference {
        channels.extend([
            ("ref_request", "event"),
            ("ref_status", "state"),
            ("ref_active_analytic", "state"),
            ("ref_active_sampled", "state"),
        ]);
    }
    if native_hover {
        channels.push(("attitude_target", "state"));
    }
    let hover_channel = channels.iter().position(|(name, _)| *name == "hover_thrust").unwrap() as u32;
    let channel_specs: Vec<ChannelSpec> = channels.iter().enumerate().map(|(id, (name, qos))| ChannelSpec {
        id: id as u32, name: name.to_string(), qos: match *qos { "event" => Qos::Event, "control" => Qos::Control, _ => Qos::State },
    }).collect();
    let channels: String = channels.iter().map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{q}\"\n")).collect();
    let ref_plugin = if reference {
        let module = common::ref_trajectory_lib(&prefix).clone();
        format!(
            r#"
[[plugin]]
name = "ref-trajectory"
path = "{module}"
trigger = "on_round"
config = {{ time_source = "session" }}
bind = {{ analytic = {{ channel = "ref_request", from = ["uav1"] }}, status = {{ channel = "ref_status" }}, active_analytic = {{ channel = "ref_active_analytic" }}, active_sampled = {{ channel = "ref_active_sampled" }} }}
"#,
            module = module.display()
        )
    } else {
        String::new()
    };
    let ctl_refs = if reference {
        r#", ref_request = { channel = "ref_request" }, ref_active_analytic = { channel = "ref_active_analytic", from = ["uav1"] }, ref_active_sampled = { channel = "ref_active_sampled", from = ["uav1"] }"#
    } else {
        ""
    };
    let hover_plugin = if native_hover {
        let module = std::path::PathBuf::from(
            std::env::var("HTE_NATIVE_LIBRARY")
                .expect("set HTE_NATIVE_LIBRARY to the installed domain-owned HTE ELF"),
        );
        assert!(module.is_absolute() && module.is_file(),
                "HTE_NATIVE_LIBRARY must be an actual absolute installed artifact; no source fallback");
        format!(r#"
[[plugin]]
name = "hover-thrust"
path = "{}"
trigger = "both"
config = {{ time_source = "session" }}
bind = {{ imu = {{ channel = "imu", from = ["uav1"] }}, attitude_target = {{ channel = "attitude_target", from = ["uav1"] }}, pose = {{ channel = "pose", from = ["uav1"] }}, hover_thrust = {{ channel = "hover_thrust" }} }}
"#, module.display())
    } else { String::new() };
    // Only one producer owns hover_thrust in the native-estimator run.
    let ros_hover_bind = if native_hover { r#"attitude_target = { channel = "attitude_target" }"# }
                         else { r#"hover_thrust = { channel = "hover_thrust" }"# };
    let roster = if native_hover { r#""uav1", "observer""# } else { r#""uav1""# };
    let reference_type = if reference { ", reference_analytic_type = 3" } else { "" };
    let manifest = format!(
        r#"
[session]
id = "ctlpx4rosio"
node = "uav1"
roster = [{roster}]
period_ms = 1
start_delay_ms = 100
run_for_ms = 180000

[transport]
kind = "loopback"

[audit]
dir = "audit"

{channels}
[[plugin]]
name = "ros_io"
path = "{ros_io}"
trigger = "on_round"
config = {{ node_name = "xgc_ros_io_uav1", imu_topic = "/uav1/mavros/imu/data_raw", pose_topic = "/vrpn_client_node/uav1/pose", vision_pose_topic = "/uav1/mavros/vision_pose/pose", rigid_state_estimate_topic = "/uav1/alg/state_estimator/state", fcu_state_topic = "/uav1/mavros/state", local_pose_topic = "/uav1/mavros/local_position/pose", local_velocity_topic = "/uav1/mavros/local_position/velocity_local", fcu_imu_topic = "/uav1/mavros/imu/data", battery_topic = "/uav1/mavros/battery", command_topic = "/command", alg_setpoint_topic = "/uav1/alg/setpoint_raw/local", hover_thrust_topic = "/uav1/hover_thrust/estimate_state", attitude_target_topic = "/uav1/mavros/setpoint_raw/target_attitude", setpoint_topic = "/uav1/mavros/setpoint_raw/local", attitude_rate_topic = "/uav1/mavros/setpoint_raw/attitude", status_topic = "/uav1/custom/statustext", fcu_request_topic = "/uav1/mavros" }}
bind = {{ imu = {{ channel = "imu" }}, pose = {{ channel = "pose" }}, vision_pose = {{ channel = "vision_pose", from = ["uav1"] }}, rigid_state_estimate = {{ channel = "estimate", from = ["uav1"] }}, fcu_state = {{ channel = "fcu_state" }}, local_pose = {{ channel = "local_pose" }}, local_velocity = {{ channel = "local_velocity" }}, fcu_imu = {{ channel = "fcu_imu" }}, battery = {{ channel = "battery" }}, command = {{ channel = "command" }}, alg_setpoint = {{ channel = "alg_setpoint" }}, {ros_hover_bind}, setpoint = {{ channel = "setpoint", from = ["uav1"] }}, attitude_rate = {{ channel = "attitude_rate", from = ["uav1"] }}, status = {{ channel = "status", from = ["uav1"] }}, fcu_request = {{ channel = "fcu_request", from = ["uav1"] }} }}

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
# Functional integration on a non-real-time test host. The 1 ms session
# period is not a demonstrated step deadline; retain a bounded watchdog.
step_budget_ms = 10
config = {{ time_source = "session", tracking_backend = "{backend}"{reference_type} }}
bind = {{ estimate = {{ channel = "estimate", from = ["uav1"] }}, local_pose = {{ channel = "local_pose", from = ["uav1"] }}, local_velocity = {{ channel = "local_velocity", from = ["uav1"] }}, imu = {{ channel = "fcu_imu", from = ["uav1"] }}, fcu_state = {{ channel = "fcu_state", from = ["uav1"] }}, battery = {{ channel = "battery", from = ["uav1"] }}, vrpn_pose = {{ channel = "pose", from = ["uav1"] }}, command = {{ channel = "command", from = ["uav1"] }}, alg_setpoint = {{ channel = "alg_setpoint", from = ["uav1"] }}, hover_thrust = {{ channel = "hover_thrust", from = ["uav1"] }}, setpoint = {{ channel = "setpoint" }}, attitude_rate = {{ channel = "attitude_rate" }}, fcu_request = {{ channel = "fcu_request" }}, status = {{ channel = "status" }}{ctl_refs} }}
{ref_plugin}
{hover_plugin}"#,
        ros_io = ros_io.display(),
        eskf = eskf.display(),
        ctl = ctl.display(),
    );
    std::fs::write(dir.join("manifest.toml"), &manifest).unwrap();
    let bus = LoopbackBus::new();
    let clock = Arc::new(WallClock::new(0));
    let observer = if native_hover {
        let ctx = TransportContext { session: "ctlpx4rosio".into(), node: "observer".into(), node_id: 1,
            roster: vec!["uav1".into(), "observer".into()], channels: channel_specs };
        let endpoint = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), Arc::new(NullAudit), 1 << 16).unwrap();
        endpoint.declare_in(hover_channel, &[0]).unwrap();
        Some(endpoint)
    } else { None };
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(bus)), clock, HostOptions::default()).unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };

    let standin = common::workspace_root().join("crates/xgc-rt-host/tests/ros/px4_standin.py");
    let out = std::process::Command::new("bash")
        .arg("-c")
        .arg(format!(
            "source '{}/setup.sh' && source '{}/devel/setup.bash' && exec python3 '{}' 4.0 120.0 {} {}",
            prefix.display(),
            ws.display(),
            standin.display(),
            if reference { "12.0" } else { "5.0" },
            if reference { "reference" } else { "planner" }
        ))
        .env("PATH", format!("{}:{}", prefix.join("bin").display(), std::env::var("PATH").unwrap_or_default()))
        .env("ROS_HOME", &ros_home)
        .env("ROS_MASTER_URI", &master)
        .env("XGC_STANDIN_NATIVE_HOVER", if native_hover { "1" } else { "0" })
        .output()
        .unwrap();
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();

    let stdout = String::from_utf8_lossy(&out.stdout);
    let line = stdout.lines().rev().find(|l| l.starts_with('{')).unwrap_or_else(|| {
        panic!("stand-in printed no summary\nstderr: {}", String::from_utf8_lossy(&out.stderr))
    });
    println!("stand-in ({backend}): {line}");
    let r: serde_json::Value = serde_json::from_str(line).unwrap();
    std::fs::write(dir.join("plant-summary.json"), serde_json::to_vec_pretty(&r).unwrap()).unwrap();
    if let Some(observer) = observer {
        assert_eq!(r["synthetic_hover_estimates"], 0, "stand-in must not supply an estimate");
        assert!(r["attitude_feedback_samples"].as_u64().unwrap() > 1000);
        let estimates: Vec<serde_json::Value> = observer.drain().into_iter().map(|frame| {
            assert_eq!(frame.header.channel, hover_channel);
            assert_eq!(frame.payload.len(), 64);
            let f = |i| f64::from_le_bytes(frame.payload[i..i + 8].try_into().unwrap());
            let u = |i| u32::from_le_bytes(frame.payload[i..i + 4].try_into().unwrap());
            serde_json::json!({"stamp":f(0), "hover":f(8), "raw":f(16), "last_estimate_stamp":f(40), "state":u(48), "flags":u(52), "sample_used":u(56)})
        }).collect();
        observer.close();
        std::fs::write(dir.join("hover-estimates.json"), serde_json::to_vec(&estimates).unwrap()).unwrap();
        // The upstream AirborneState has separate raw-update and publish
        // gates. sample_used only describes the publish tick, so counting it
        // misses updates between publications. Distinct last-estimate stamps
        // prove updates occurred, even when their tick did not publish.
        let airborne: Vec<_> = estimates.iter().filter(|e| e["state"] == 12).collect();
        let update_stamps: std::collections::BTreeSet<_> = airborne.iter().map(|e| e["last_estimate_stamp"].as_f64().unwrap().to_bits()).collect();
        assert!(update_stamps.len() > 100, "native estimator must use fresh airborne measurements");
        let learned: Vec<f64> = airborne.iter().map(|e| e["hover"].as_f64().unwrap()).collect();
        assert!(learned.len() > 1000, "native estimator must publish sustained airborne estimates");
        assert!(learned.iter().all(|v| v.is_finite() && *v > 0.0 && *v < 1.0));
        let tail = &learned[learned.len().saturating_sub(100)..];
        let mean = tail.iter().sum::<f64>() / tail.len() as f64;
        assert!((mean - 0.5).abs() < 0.05, "native hover estimate {mean}");
        let hte = summary.plugins.iter().find(|p| p.name == "hover-thrust").unwrap();
        assert!(hte.consumed > 1000 && hte.published > 100);
        println!("native hover: {} estimates, {} airborne, {} distinct raw-update stamps, tail mean {mean:.6}; stand-in estimates=0", estimates.len(), learned.len(), update_stamps.len());
    }
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
    assert!(r["custom1"] == true && states.contains(&"Custom1"), "never entered Custom1: {states:?}");
    if reference {
        assert!(r["custom1_held"] == true, "left Custom1 early: {states:?}");
        assert!(r["attitude_setpoints"].as_u64().unwrap() > 500, "body-rate + thrust targets");
        assert!(r["reach_xy_m"].as_f64().unwrap() > 1.0, "follows the activated reference");
        assert!(r["z_range_m"][0].as_f64().unwrap() > 1.0, "stays airborne in Custom1");
    } else {
        assert!((r["x_after_track"].as_f64().unwrap() - 1.5).abs() < 0.15, "follows the planner setpoints");
        assert!(r["track_err_max_m"].as_f64().unwrap() < 0.5, "tracking error bounded");
        assert!((r["z_after_hover"].as_f64().unwrap() - 2.3).abs() < 0.3, "holds altitude in Hover");
    }
    assert!(r["final_z"].as_f64().unwrap() < 0.1 && r["armed_at_end"] == false, "landed and disarmed");
    assert!(r["eskf_err_p50_m"].as_f64().unwrap() < 0.05, "ESKF output tracks truth");
    assert!(r["setpoints"].as_u64().unwrap() > 100);
    assert!(r["arm_calls"].as_u64().unwrap() >= 1 && r["disarm_calls"].as_u64().unwrap() >= 1, "arm and disarm went through ros_io");
    assert!(r["mode_calls"].as_array().unwrap().iter().any(|m| m == "OFFBOARD"), "OFFBOARD requested through ros_io");
}

#[test]
fn ctl_px4_module_flies_px4_local_in_place_of_the_ros_node() {
    fly("px4_local", false);
}

#[test]
fn ctl_px4_module_flies_dfbc_with_the_ref_trajectory_module() {
    fly("dfbc", false);
}

#[test]
fn ctl_px4_module_flies_nmpc_with_the_ref_trajectory_module() {
    fly("nmpc", false);
}

// Same synthetic plant, now with the real hover estimator in the memory graph.
#[test]
fn ctl_px4_dfbc_with_native_hover_estimator() {
    fly("dfbc", true);
}

#[test]
fn ctl_px4_nmpc_with_native_hover_estimator() {
    fly("nmpc", true);
}
