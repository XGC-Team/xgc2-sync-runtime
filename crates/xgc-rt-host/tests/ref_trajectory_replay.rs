//! ref-trajectory gate: the reference trajectory runtime as an aggregator
//! module must reproduce, byte for byte, the output of the runtime's own
//! replay harness (xgc2-multirotor-controller multirotor_reference_trajectory
//! test/replay) on the same request stream. The module runs in a host with
//! time_source = "input"; a feeder node publishes the requests (converted to
//! xgc payloads, envelope t_produce = receive time) and a final clock sample,
//! and collects the trace port.
//!
//! Needs:
//!   ROS_PREFIX          ROS Noetic (only for the stream converter)
//!   REF_CORE_LIB_DIR    dir with libmultirotor_reference_trajectory_core.so
//!   REF_REPLAY_STREAM   the request stream (make_reference_stream.py output)
//!   REF_REPLAY_REF      reference_replay_harness output on that stream
//!   REFERENCE_TRAJECTORY_NATIVE_LIBRARY installed owning native adapter
//!   XGC_REFERENCE_WIRE_PREFIX owning reference wire header install prefix
//!   XGC_ROS_REPLAY_MSGS_INCLUDE owning generated ROS message include directory
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

// Channel ids = ref-trajectory port indices (the stream converter emits those).
const CHANNELS: [(&str, Qos); 8] = [
    ("analytic", Qos::Event),
    ("sampled", Qos::Event),
    ("reset", Qos::Event),
    ("clock", Qos::Event),
    ("status", Qos::State),
    ("active_analytic", Qos::State),
    ("active_sampled", Qos::State),
    ("trace", Qos::Bulk),
];
const CLOCK: u32 = 3;
const TRACE: u32 = 7;

fn env_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name).map(PathBuf::from).filter(|p| p.exists())
}

fn convert_stream(prefix: &std::path::Path, stream: &std::path::Path) -> Vec<(u64, u32, Vec<u8>)> {
    // Consume the same owning installed wire and ROS headers as ctl_px4_replay.
    let reference = PathBuf::from(std::env::var_os("XGC_REFERENCE_WIRE_PREFIX")
        .expect("set XGC_REFERENCE_WIRE_PREFIX to the owning reference wire install prefix")).join("include");
    let messages = PathBuf::from(std::env::var_os("XGC_ROS_REPLAY_MSGS_INCLUDE")
        .expect("set XGC_ROS_REPLAY_MSGS_INCLUDE to the owning generated ROS message include directory"));
    let out = common::workspace_root().join("target/plugin-tests/ros");
    std::fs::create_dir_all(&out).unwrap();
    let tool = out.join("ref_stream_to_xgc");
    let conda_cxx = prefix.join("bin/x86_64-conda-linux-gnu-c++");
    let cxx = if conda_cxx.is_file() { conda_cxx } else { PathBuf::from("c++") };
    let status = Command::new(cxx)
        .args(["-std=c++17", "-O2"])
        .arg("-I").arg(&reference)
        .arg("-I").arg(&messages)
        .arg("-isystem").arg(prefix.join("include"))
        .arg(common::workspace_root().join("crates/xgc-rt-host/tests/ros/ref_stream_to_xgc.cpp"))
        .arg("-o").arg(&tool)
        .arg("-L").arg(prefix.join("lib")).arg(format!("-Wl,-rpath,{}", prefix.join("lib").display()))
        .args(["-lroscpp_serialization", "-lrostime", "-lcpp_common"])
        .status()
        .unwrap();
    assert!(status.success(), "building ref_stream_to_xgc failed");
    let converted = out.join("ref_requests.xgcstream");
    assert!(Command::new(&tool).arg(stream).arg(&converted).status().unwrap().success());
    let bytes = std::fs::read(&converted).unwrap();
    assert_eq!(&bytes[..8], b"XGCREFS1");
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
fn ref_trajectory_module_reproduces_the_runtime_replay_byte_for_byte() {
    let (Some(prefix), Some(_core), Some(stream), Some(reference)) = (
        common::ros_prefix(),
        env_path("REF_CORE_LIB_DIR"),
        env_path("REF_REPLAY_STREAM"),
        env_path("REF_REPLAY_REF"),
    ) else {
        eprintln!("skipped: set ROS_PREFIX, REF_CORE_LIB_DIR, REF_REPLAY_STREAM and REF_REPLAY_REF");
        return;
    };
    let lib = common::ref_trajectory_lib(&prefix).clone();
    let records = convert_stream(&prefix, &stream);
    let expected = std::fs::read(&reference).unwrap();

    let dir = common::scratch("ref-trajectory-replay");
    let channels: String = CHANNELS
        .iter()
        .map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{}\"\n", format!("{q:?}").to_lowercase()))
        .collect();
    let binds: Vec<String> = CHANNELS[..=CLOCK as usize]
        .iter()
        .map(|(n, _)| format!("{n} = {{ channel = \"{n}\", from = [\"feeder\"] }}"))
        .chain(CHANNELS[CLOCK as usize + 1..].iter().map(|(n, _)| format!("{n} = {{ channel = \"{n}\" }}")))
        .collect();
    let manifest = format!(
        r#"
[session]
id = "refreplay"
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
name = "ref-trajectory"
path = "{lib}"
trigger = "on_dirty"
config = {{ time_source = "input", trace = true }}
bind = {{ {binds} }}
"#,
        lib = lib.display(),
        binds = binds.join(", "),
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(WallClock::new(0));
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();

    let names: Vec<String> = CHANNELS.iter().map(|(n, _)| n.to_string()).collect();
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(), session: "refreplay".into(), node: "feeder".into(), node_id: 1,
                roster: vec!["uav1".into(), "feeder".into()], channels: names.clone(), clock_domain: "wall".into(),
                audit_queue_drops: 0, records_written: 0, complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext {
        session: "refreplay".into(), node: "feeder".into(), node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: CHANNELS.iter().enumerate().map(|(i, (n, q))| ChannelSpec { id: i as u32, name: n.to_string(), qos: *q }).collect(),
    };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 16).unwrap();
    for ch in 0..=CLOCK {
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
    for (t, port, payload) in &records {
        feeder.publish(*port, 0, *t as i64, payload).unwrap();
        std::thread::sleep(Duration::from_millis(2));
        trace.extend(feeder.drain());
    }
    // End of stream: the harness runs updates up to (last receive time + 1 s).
    let last = records.last().unwrap().0;
    let end = last as f64 * 1e-9 + 1.0;
    feeder.publish(CLOCK, 0, last as i64, &end.to_le_bytes()).unwrap();
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
        "ref-trajectory {} domain={} steps={} consumed={} published={}; records {}, trace {} bytes (reference {})",
        module.state, module.domain_state, module.steps, module.consumed, module.published, records.len(), got.len(), expected.len()
    );
    assert!(module.last_error.is_none(), "{module:?}");
    if got != expected {
        let (g, e) = (String::from_utf8_lossy(&got), String::from_utf8_lossy(&expected));
        let first = g.lines().zip(e.lines()).position(|(a, b)| a != b);
        panic!(
            "trace differs from the replay reference: {} vs {} lines; first difference at line {:?}\n got: {:.300?}\nwant: {:.300?}",
            g.lines().count(), e.lines().count(), first,
            first.and_then(|i| g.lines().nth(i)), first.and_then(|i| e.lines().nth(i))
        );
    }
}
