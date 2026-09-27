//! Z2e: the transport as a loadable plugin (`xgc_rt_transport_v1`,
//! abi/include/xgc_rt.h). The loopback and Zenoh transports are built as
//! transport plugins (plugins/transport-*), and a manifest names one with
//! `[transport] path`, as it names a module plugin.
//!
//! - The Z1 pipeline (five modules in Rust and C) runs in the xgc-rt-host
//!   binary with its transport loaded from the loopback plugin.
//! - The same pipeline split over two hosts in one process, the handoff
//!   between them over the loopback plugin: every sample crosses, the
//!   merged audit is valid with no loss.
//! - The pipeline split over two xgc-rt-host processes, the transport the
//!   Zenoh plugin over TCP on localhost, each manifest naming the plugin.
//!
//! audit_exact.rs (Z1 exactness) and zenoh_impaired.rs (Z2b calibration)
//! run through the plugins as well.

mod common;

use std::path::Path;
use std::process::Command;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use xgc_rt_audit::{merge_run, MergeOptions};
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions, RunSummary};

fn now_ns() -> i64 {
    WallClock::new(0).now()
}

const CHANNELS: &str = r#"
[[channel]]
name = "detections"
qos = "state"
[[channel]]
name = "state"
qos = "state"
[[channel]]
name = "plan"
qos = "control"
[[channel]]
name = "cmd"
qos = "event"
"#;

/// The Z1 pipeline's modules; `node_of` places each on a node, and a hop
/// between two nodes reads from the other one.
fn modules(node_of: &dyn Fn(&str) -> &'static str, only: Option<&str>) -> String {
    // An in-port reads its writer's node: this node's own name means the
    // module in this process (memory), another node the link.
    let from = |_reader: &str, writer: &str| node_of(writer).to_string();
    let all = [
        ("perception", common::lib("stub_perception"), "on_round", "config = { payload_bytes = 256 }\n".to_string(), "detections = { channel = \"detections\" }".to_string()),
        (
            "estimation",
            common::lib("stub_estimation"),
            "on_dirty",
            String::new(),
            format!("detections = {{ channel = \"detections\", from = [\"{}\"] }}, state = {{ channel = \"state\" }}", from("estimation", "perception")),
        ),
        (
            "planning",
            common::lib("stub_planning"),
            "both",
            String::new(),
            format!("state = {{ channel = \"state\", from = [\"{}\"] }}, plan = {{ channel = \"plan\" }}", from("planning", "estimation")),
        ),
        (
            "control",
            common::lib("stub_control"),
            "on_dirty",
            String::new(),
            format!("plan = {{ channel = \"plan\", from = [\"{}\"] }}, cmd = {{ channel = \"cmd\" }}", from("control", "planning")),
        ),
        (
            "sink",
            common::lib("c_stub"),
            "on_dirty",
            String::new(),
            format!("cmd = {{ channel = \"cmd\", from = [\"{}\"] }}", from("sink", "control")),
        ),
    ];
    all.iter()
        .filter(|m| only.map_or(true, |node| node_of(m.0) == node))
        .map(|(name, path, trigger, config, bind)| {
            format!("[[plugin]]\nname = \"{name}\"\npath = \"{path}\"\ntrigger = \"{trigger}\"\n{config}bind = {{ {bind} }}\n")
        })
        .collect()
}

fn manifest(node: &str, roster: &[&str], e0: Option<i64>, run_for_ms: u32, transport: &str, audit: &Path, body: &str) -> String {
    let roster = roster.iter().map(|r| format!("\"{r}\"")).collect::<Vec<_>>().join(", ");
    let epoch = e0.map(|e| format!("epoch_ns = {e}\n")).unwrap_or_default();
    format!(
        "[session]\nid = \"z2e\"\nnode = \"{node}\"\nroster = [{roster}]\nperiod_ms = 20\nstart_delay_ms = 50\nrun_for_ms = {run_for_ms}\n{epoch}\n[transport]\n{transport}\n\n[audit]\ndir = \"{}\"\n{CHANNELS}\n{body}",
        audit.display()
    )
}

fn plugin<'a>(summary: &'a RunSummary, name: &str) -> &'a xgc_rt_host::host::PluginSummary {
    summary.plugins.iter().find(|p| p.name == name).unwrap()
}

#[test]
fn the_z1_pipeline_runs_in_the_host_binary_with_its_transport_loaded_from_a_plugin() {
    let dir = common::scratch("z2e-binary");
    let transport = format!("kind = \"loopback\"\npath = \"{}\"", common::transport_plugin("loopback").display());
    let text = manifest("uav1", &["uav1"], None, 1000, &transport, &dir.join("audit"), &modules(&|_| "uav1", None));
    std::fs::write(dir.join("node.toml"), &text).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host")).arg("--manifest").arg(dir.join("node.toml")).output().unwrap();
    assert!(out.status.success(), "xgc-rt-host failed:\n{}", String::from_utf8_lossy(&out.stderr));
    let summary: serde_json::Value = serde_json::from_slice(&out.stdout).unwrap();
    for (name, domain) in [("perception", "tracking"), ("estimation", "converged"), ("planning", "planning"), ("control", "track"), ("sink", "counting")] {
        let p = summary["plugins"].as_array().unwrap().iter().find(|p| p["name"] == name).unwrap();
        assert_eq!(p["domain_state"], domain, "{name}: {p}");
        assert!(p["steps"].as_u64().unwrap() > 0 && p["last_error"].is_null(), "{name}: {p}");
    }
    // A manifest that names a plugin of another kind is refused.
    let wrong = text.replace("kind = \"loopback\"", "kind = \"zenoh\"");
    std::fs::write(dir.join("wrong.toml"), wrong).unwrap();
    let out = Command::new(env!("CARGO_BIN_EXE_xgc-rt-host")).arg("--manifest").arg(dir.join("wrong.toml")).output().unwrap();
    assert!(!out.status.success());
    assert!(String::from_utf8_lossy(&out.stderr).contains("a \"loopback\" transport, but the manifest says kind = \"zenoh\""), "{}", String::from_utf8_lossy(&out.stderr));
}

/// uav1: perception, estimation; uav2: planning, control, sink.
fn split(module: &str) -> &'static str {
    match module {
        "perception" | "estimation" => "uav1",
        _ => "uav2",
    }
}

#[test]
fn the_z1_pipeline_split_over_two_hosts_hands_off_over_the_loopback_plugin() {
    let dir = common::scratch("z2e-split");
    let audit = dir.join("audit");
    let e0 = now_ns() + 1_500_000_000;
    let roster = ["uav1", "uav2"];
    let transport = format!("kind = \"loopback\"\npath = \"{}\"\nbus = \"z2e-split\"", common::transport_plugin("loopback").display());
    let runners: Vec<_> = roster
        .iter()
        .map(|node| {
            let text = manifest(node, &roster, Some(e0), 1000, &transport, &audit, &modules(&split, Some(node)));
            let manifest = Manifest::from_toml_str(&text).unwrap();
            let t = &manifest.transport;
            let transport = Box::new(xgc_rt_host::transport_so::SoTransport::load(t.path.as_ref().unwrap(), None, &t.kind, &t.options).unwrap());
            let host = Host::new(manifest, &dir, transport, Arc::new(WallClock::new(0)), HostOptions::default()).unwrap();
            std::thread::spawn(move || host.run(&AtomicBool::new(false)).unwrap())
        })
        .collect();
    let summaries: Vec<RunSummary> = runners.into_iter().map(|r| r.join().unwrap()).collect();
    let (estimation, planning) = (plugin(&summaries[0], "estimation"), plugin(&summaries[1], "planning"));
    println!("estimation published {}, planning consumed {}", estimation.published, planning.consumed);
    assert!(estimation.published > 20 && planning.consumed > 20, "{estimation:?} {planning:?}");
    assert_eq!(plugin(&summaries[1], "sink").domain_state, "counting");

    let report = merge_run(&audit, MergeOptions::default()).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    let s = report.streams.iter().find(|s| s.channel == "state" && s.origin == "uav1" && s.receiver == "uav2").expect("state stream uav1 -> uav2");
    println!("state uav1 -> uav2 over the loopback plugin: expected {} received {} lost {}", s.counts.expected, s.counts.received, s.counts.lost);
    assert_eq!(s.counts.expected, estimation.published);
    assert_eq!((s.counts.lost, s.counts.duplicates, s.counts.reordered), (0, 0, 0));
}

#[test]
fn the_z1_pipeline_split_over_two_host_processes_runs_over_the_zenoh_plugin() {
    let dir = common::scratch("z2e-zenoh");
    let audit = dir.join("audit");
    let port = common::listen_port();
    let plugin_path = common::transport_plugin("zenoh");
    let e0 = now_ns() + 4_000_000_000;
    let roster = ["uav1", "uav2"];
    let children: Vec<_> = roster
        .iter()
        .map(|node| {
            let endpoints = if *node == "uav1" { format!("listen = [\"tcp/127.0.0.1:{port}\"]") } else { format!("connect = [\"tcp/127.0.0.1:{port}\"]") };
            let transport = format!("kind = \"zenoh\"\npath = \"{}\"\n{endpoints}", plugin_path.display());
            // The host waits for a remote subscriber on every out-channel,
            // also those only read in its own process (detections, plan,
            // cmd): cap that wait well before the epoch.
            let text = manifest(node, &roster, Some(e0), 2000, &transport, &audit, &modules(&split, Some(node)))
                .replace("run_for_ms = 2000\n", "run_for_ms = 2000\npeer_timeout_ms = 1000\n");
            let file = dir.join(format!("{node}.toml"));
            std::fs::write(&file, text).unwrap();
            Command::new(env!("CARGO_BIN_EXE_xgc-rt-host")).arg("--manifest").arg(&file).stdout(std::process::Stdio::piped()).stderr(std::process::Stdio::piped()).spawn().unwrap()
        })
        .collect();
    let mut summaries = Vec::new();
    for child in children {
        let out = child.wait_with_output().unwrap();
        assert!(out.status.success(), "xgc-rt-host failed:\n{}", String::from_utf8_lossy(&out.stderr));
        summaries.push(serde_json::from_slice::<serde_json::Value>(&out.stdout).unwrap());
    }
    let find = |s: &serde_json::Value, name: &str| s["plugins"].as_array().unwrap().iter().find(|p| p["name"] == name).unwrap().clone();
    let (estimation, planning) = (find(&summaries[0], "estimation"), find(&summaries[1], "planning"));

    let report = merge_run(&audit, MergeOptions::default()).unwrap();
    assert!(report.valid, "{:?}", report.invalid_reasons);
    let s = report.streams.iter().find(|s| s.channel == "state" && s.origin == "uav1" && s.receiver == "uav2").expect("state stream uav1 -> uav2");
    println!(
        "state uav1 -> uav2 over the zenoh plugin, two processes: expected {} received {} lost {} OWD p50 {:.3} ms",
        s.counts.expected,
        s.counts.received,
        s.counts.lost,
        s.owd_ns.p50 as f64 / 1e6
    );
    println!("timings uav1 {} uav2 {}", summaries[0]["timings"], summaries[1]["timings"]);
    assert!(planning["consumed"].as_u64().unwrap() > 50, "{planning}");
    assert_eq!(s.counts.expected, estimation["published"].as_u64().unwrap());
    assert!(s.counts.received * 100 >= s.counts.expected * 99, "state is best-effort, but at 50 Hz on localhost nearly all arrive: {:?}", s.counts);
}
