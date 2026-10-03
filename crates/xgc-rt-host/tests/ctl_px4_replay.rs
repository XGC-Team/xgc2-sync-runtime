//! ctl-px4 gate: the PX4 controller core as an aggregator module must
//! reproduce, byte for byte, the output of the controller's own replay
//! harness (xgc2-multirotor-controller test/replay) on the same recorded
//! flight. The module runs in a host with time_source = "input"; a feeder
//! node publishes the recorded inputs (converted to xgc payloads, envelope
//! t_produce = receive time) and a final clock sample, and collects the
//! trace port.
//!
//! Needs:
//!   ROS_PREFIX          ROS Noetic (only for the stream converter)
//!   PX4_CONTROLLER_NATIVE_LIBRARY absolute installed owning libctl_px4.so
//!   PX4_REPLAY_STREAM   the replay stream (bag_to_stream.py output)
//!   PX4_REPLAY_REF      replay_harness output on that stream
//!   PX4_REPLAY_BACKEND  optional: the tracking backend the harness ran with
//!                       (px4_local | dfbc | nmpc; default px4_local)
//!   PX4_REPLAY_REFERENCE_TYPE optional: its reference_analytic_type
//!   LD_LIBRARY_PATH for the owning installed controller/core/state-machine closure
//! Without them the test prints why and passes.

mod common;

use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

// Channel ids = ctl-px4 port indices (the stream converter emits those).
const CHANNELS: [(&str, Qos); 19] = [
    ("estimate", Qos::State),
    ("local_pose", Qos::State),
    ("local_velocity", Qos::State),
    ("imu", Qos::State),
    ("fcu_state", Qos::State),
    ("battery", Qos::State),
    ("vrpn_pose", Qos::State),
    ("command", Qos::Event),
    ("clock", Qos::Event),
    ("setpoint", Qos::Control),
    ("attitude_rate", Qos::Control),
    ("fcu_request", Qos::Event),
    ("status", Qos::State),
    ("trace", Qos::Bulk),
    ("alg_setpoint", Qos::Control),
    ("hover_thrust", Qos::State),
    ("ref_active_analytic", Qos::State),
    ("ref_active_sampled", Qos::State),
    ("ref_request", Qos::Event),
];
const INPUTS: [u32; 13] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 14, 15, 16, 17];
const CLOCK: u32 = 8;
const TRACE: u32 = 13;

fn env_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name).map(PathBuf::from).filter(|p| p.exists())
}

fn convert_stream(prefix: &std::path::Path, stream: &std::path::Path) -> Vec<(u64, u32, Vec<u8>)> {
    // Consume installed owner DTO/codec and already-generated owning ROS message
    // headers. Do not build a second ros_io plugin just to obtain those headers.
    let robotics = PathBuf::from(std::env::var_os("XGC_ROBOTICS_INTERFACES_PREFIX").unwrap()).join("include");
    let hte = PathBuf::from(std::env::var_os("XGC_HOVER_THRUST_WIRE_PREFIX").unwrap()).join("include");
    let rigid = PathBuf::from(std::env::var_os("XGC_RIGID_STATE_WIRE_PREFIX").unwrap()).join("include");
    let reference = PathBuf::from(std::env::var_os("XGC_REFERENCE_WIRE_PREFIX").unwrap()).join("include");
    let messages = PathBuf::from(std::env::var_os("XGC_ROS_REPLAY_MSGS_INCLUDE").unwrap());
    let out = common::workspace_root().join("target/plugin-tests/ros");
    std::fs::create_dir_all(&out).unwrap();
    let tool = out.join("px4_stream_to_xgc");
    let conda_cxx = prefix.join("bin/x86_64-conda-linux-gnu-c++");
    let cxx = if conda_cxx.is_file() { conda_cxx } else { PathBuf::from("c++") };
    let status = Command::new(cxx)
        .args(["-std=c++17", "-O2"])
        .arg("-I").arg(&robotics)
        .arg("-I").arg(&hte)
        .arg("-I").arg(&rigid)
        .arg("-I").arg(&reference)
        .arg("-I").arg(&messages)
        .arg("-isystem").arg(prefix.join("include"))
        .arg(common::workspace_root().join("crates/xgc-rt-host/tests/ros/px4_stream_to_xgc.cpp"))
        .arg("-o").arg(&tool)
        .arg("-L").arg(prefix.join("lib")).arg(format!("-Wl,-rpath,{}", prefix.join("lib").display()))
        .args(["-lroscpp_serialization", "-lrostime", "-lcpp_common"])
        .status()
        .unwrap();
    assert!(status.success(), "building px4_stream_to_xgc failed");
    let converted = out.join("px4_flight.xgcstream");
    assert!(Command::new(&tool).arg(stream).arg(&converted).status().unwrap().success());
    let bytes = std::fs::read(&converted).unwrap();
    assert_eq!(&bytes[..8], b"XGCPX4S1");
    let mut records = Vec::new();
    let mut i = 8;
    while i < bytes.len() {
        let t = u64::from_le_bytes(bytes[i..i + 8].try_into().unwrap());
        let port = u32::from_le_bytes(bytes[i + 8..i + 12].try_into().unwrap());
        let len = u32::from_le_bytes(bytes[i + 12..i + 16].try_into().unwrap()) as usize;
        records.push((t, port, bytes[i + 16..i + 16 + len].to_vec()));
        i += 16 + len;
    }
    records
}

#[test]
fn ctl_px4_module_reproduces_the_controller_replay_byte_for_byte() {
    let (Some(prefix), Some(_core), Some(stream), Some(reference)) = (
        common::ros_prefix(),
        env_path("PX4_CONTROLLER_NATIVE_LIBRARY"),
        env_path("PX4_REPLAY_STREAM"),
        env_path("PX4_REPLAY_REF"),
    ) else {
        eprintln!("skipped: set ROS_PREFIX, PX4_CONTROLLER_NATIVE_LIBRARY, PX4_REPLAY_STREAM and PX4_REPLAY_REF");
        return;
    };
    let lib = common::ctl_px4_lib(&prefix);
    let records = convert_stream(&prefix, &stream);
    let expected = std::fs::read(&reference).unwrap();

    let dir = common::scratch("ctl-px4-replay");
    let channels: String = CHANNELS
        .iter()
        .map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{}\"\n", format!("{q:?}").to_lowercase()))
        .collect();
    let binds: Vec<String> = INPUTS
        .iter()
        .map(|&i| CHANNELS[i as usize].0)
        .map(|n| format!("{n} = {{ channel = \"{n}\", from = [\"feeder\"] }}"))
        .chain(["setpoint = { channel = \"setpoint\" }".to_string(), "trace = { channel = \"trace\" }".to_string()])
        .collect();
    let manifest = format!(
        r#"
[session]
id = "px4replay"
node = "uav1"
roster = ["uav1", "feeder"]
period_ms = 10
start_delay_ms = 50

[transport]
kind = "loopback"

[audit]
dir = "audit"

{channels}
[[plugin]]
name = "ctl-px4"
path = "{lib}"
trigger = "on_dirty"
# A replay step catches up many ticks at once (with nmpc, one solve each).
step_budget_ms = 1000.0
config = {{ time_source = "input", trace = true, tracking_backend = "{backend}"{reference_type} }}
bind = {{ {binds} }}
"#,
        lib = lib.display(),
        binds = binds.join(", "),
        backend = std::env::var("PX4_REPLAY_BACKEND").unwrap_or_else(|_| "px4_local".into()),
        reference_type = std::env::var("PX4_REPLAY_REFERENCE_TYPE").map(|t| format!(", reference_analytic_type = {t}")).unwrap_or_default(),
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(WallClock::new(0));
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();

    let names: Vec<String> = CHANNELS.iter().map(|(n, _)| n.to_string()).collect();
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(), session: "px4replay".into(), node: "feeder".into(), node_id: 1,
                roster: vec!["uav1".into(), "feeder".into()], channels: names.clone(), clock_domain: "wall".into(),
                audit_queue_drops: 0, records_written: 0, complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext {
        session: "px4replay".into(), node: "feeder".into(), node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: CHANNELS.iter().enumerate().map(|(i, (n, q))| ChannelSpec { id: i as u32, name: n.to_string(), qos: *q }).collect(),
    };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 18).unwrap();
    for ch in INPUTS {
        feeder.declare_out(ch).unwrap();
    }
    feeder.declare_in(TRACE, &[0]).unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_millis(300));
    let mut trace = Vec::new();
    // Paced against flight time (PX4_REPLAY_SPEED times real time, default
    // 4): the links are best-effort, so a feeder far ahead of a busy module
    // (nmpc solves) would let samples be superseded before it reads them.
    let speed: f64 = std::env::var("PX4_REPLAY_SPEED").ok().and_then(|s| s.parse().ok()).unwrap_or(4.0);
    let start = std::time::Instant::now();
    let t_first = records[0].0;
    for (i, (t, port, payload)) in records.iter().enumerate() {
        let due = Duration::from_secs_f64((*t - t_first) as f64 * 1e-9 / speed);
        if let Some(wait) = due.checked_sub(start.elapsed()) {
            std::thread::sleep(wait);
        }
        feeder.publish(*port, 0, *t as i64, payload).unwrap();
        if i % 32 == 31 {
            trace.extend(feeder.drain());
        }
    }
    // End of stream: the harness runs ticks up to (last receive time + 0.5 s).
    let end = records.last().unwrap().0 as f64 * 1e-9 + 0.5;
    feeder.publish(CLOCK, 0, records.last().unwrap().0 as i64, &end.to_le_bytes()).unwrap();
    let mut idle = 0;
    while idle < 20 {
        std::thread::sleep(Duration::from_millis(50));
        let more = feeder.drain();
        idle = if more.is_empty() { idle + 1 } else { 0 };
        trace.extend(more);
    }
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    feeder.close();
    audit.finish().unwrap();

    let got: Vec<u8> = trace.iter().flat_map(|f| f.payload.iter().copied()).collect();
    let module = &summary.plugins[0];
    println!(
        "ctl-px4 {} domain={} steps={} consumed={} published={}; records {}, trace {} bytes (reference {})",
        module.state, module.domain_state, module.steps, module.consumed, module.published, records.len(), got.len(), expected.len()
    );
    assert!(module.last_error.is_none(), "{module:?}");
    if got != expected {
        std::fs::write(dir.join("trace.got.txt"), &got).unwrap();
        let (g, e) = (String::from_utf8_lossy(&got), String::from_utf8_lossy(&expected));
        let first = g.lines().zip(e.lines()).position(|(a, b)| a != b);
        panic!(
            "trace differs from the replay reference: {} vs {} lines; first difference at line {:?}\n got: {:?}\nwant: {:?}",
            g.lines().count(), e.lines().count(), first,
            first.and_then(|i| g.lines().nth(i)), first.and_then(|i| e.lines().nth(i))
        );
    }
}
