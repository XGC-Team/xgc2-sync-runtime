//! plan-dmpc gate: the TRO DMPC planner as an aggregator module must
//! reproduce, byte for byte, what the academic planner's own fleet replay
//! (formation_generator tools/dmpc_fleet_replay.cpp, one DmpcAgent per robot
//! in one process) recorded for one robot: its plan and its setpoint every
//! round, from the same inputs. A feeder node plays the recorded inputs
//! (measured state, neighbor plans with their rounds, formation ticks) round
//! by round and collects plan_out and setpoint.
//!
//! Needs:
//!   DMPC_LIB_DIR              libformation_generator_dmpc_{core,params,config}.so
//!                             (the formation_generator standalone build)
//!   FORMATION_GENERATOR_ROOT  the formation_generator package source
//!   DMPC_FLEET_REPLAY         optional: the replay tool of the same build
//!                             (default: DMPC_LIB_DIR/dmpc_fleet_replay)
//!   ACADOS_ROOT, YAML_CPP_LIB_DIR, EIGEN_INCLUDE, CXX (as for build-plan-dmpc.sh)
//! Without them the test prints why and passes.

mod common;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

// Channel ids = plan-dmpc port indices (the replay files use those).
const CHANNELS: [(&str, Qos); 5] = [
    ("formation_tick", Qos::Control),
    ("plan_in", Qos::Control),
    ("plan_out", Qos::Control),
    ("own_state", Qos::State),
    ("setpoint", Qos::Control),
];
const INPUTS: [u32; 3] = [0, 1, 3];
const TICK: u32 = 0;
const PLAN_OUT: u32 = 2;
const SETPOINT: u32 = 4;
const ROUNDS: u64 = 60;
const HOLD: u64 = 10;
const ROBOTS: u32 = 5;

fn env_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name).map(PathBuf::from).filter(|p| p.exists())
}

/// "XGCDMPC1", then {u64 round, u32 port, u32 len, bytes} records.
fn read_records(path: &Path) -> Vec<(u64, u32, Vec<u8>)> {
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(&bytes[..8], b"XGCDMPC1");
    let mut records = Vec::new();
    let mut i = 8;
    while i < bytes.len() {
        let round = u64::from_le_bytes(bytes[i..i + 8].try_into().unwrap());
        let port = u32::from_le_bytes(bytes[i + 8..i + 12].try_into().unwrap());
        let len = u32::from_le_bytes(bytes[i + 12..i + 16].try_into().unwrap()) as usize;
        records.push((round, port, bytes[i + 16..i + 16 + len].to_vec()));
        i += 16 + len;
    }
    records
}

fn plan_dmpc_lib(lib_dir: &Path) -> PathBuf {
    let out = common::workspace_root().join("target/plugin-tests/cpp");
    std::fs::create_dir_all(&out).unwrap();
    let lib = out.join("libplan_dmpc.so");
    let status = Command::new(common::workspace_root().join("scripts/build-plan-dmpc.sh"))
        .arg(&lib)
        .env("DMPC_LIB_DIR", lib_dir)
        .status()
        .expect("run build-plan-dmpc.sh");
    assert!(status.success(), "building plan-dmpc failed");
    lib
}

#[test]
fn plan_dmpc_module_reproduces_the_fleet_replay_byte_for_byte() {
    let (Some(lib_dir), Some(fg_root)) = (env_path("DMPC_LIB_DIR"), env_path("FORMATION_GENERATOR_ROOT")) else {
        eprintln!("skipped: set DMPC_LIB_DIR and FORMATION_GENERATOR_ROOT");
        return;
    };
    let tool = env_path("DMPC_FLEET_REPLAY").unwrap_or_else(|| lib_dir.join("dmpc_fleet_replay"));
    assert!(tool.is_file(), "no fleet replay tool at {}", tool.display());
    let lib = plan_dmpc_lib(&lib_dir);

    // The reference: robot 1 of the knot_fs150 fleet (without the scene).
    let dir = common::scratch("plan-dmpc-replay");
    let manifests: Vec<PathBuf> =
        (1..=ROBOTS).map(|i| fg_root.join(format!("test/replay/plan_dmpc/knot_fs150_uav{i}.yaml"))).collect();
    let (inputs_path, outputs_path) = (dir.join("inputs.bin"), dir.join("outputs.bin"));
    let status = Command::new(&tool)
        .args(["--record", "1", "--rounds", &ROUNDS.to_string(), "--hold", &HOLD.to_string()])
        .arg("--inputs").arg(&inputs_path)
        .arg("--outputs").arg(&outputs_path)
        .args(&manifests)
        .status()
        .expect("run dmpc_fleet_replay");
    assert!(status.success(), "dmpc_fleet_replay failed");
    let inputs = read_records(&inputs_path);
    let expected = read_records(&outputs_path);
    let expected_rolling = expected.iter().filter(|(r, p, _)| *p == SETPOINT && *r >= HOLD).count() as u64;
    assert!(expected_rolling > (ROUNDS - HOLD) * 9 / 10, "the reference must actually fly: {expected_rolling} rolling setpoints");

    let channels: String = CHANNELS
        .iter()
        .map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{}\"\n", format!("{q:?}").to_lowercase()))
        .collect();
    let manifest = format!(
        r#"
[session]
id = "dmpcreplay"
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
name = "plan-dmpc"
path = "{lib}"
trigger = "on_dirty"
step_budget_ms = 1000.0
config = {{ param_manifest = "{param_manifest}" }}
bind = {{ formation_tick = {{ channel = "formation_tick", from = ["feeder"] }}, plan_in = {{ channel = "plan_in", from = ["feeder"] }}, own_state = {{ channel = "own_state", from = ["feeder"] }}, plan_out = {{ channel = "plan_out" }}, setpoint = {{ channel = "setpoint" }} }}
"#,
        lib = lib.display(),
        param_manifest = manifests[0].display(),
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(WallClock::new(0));
    let host = Host::new(Manifest::from_toml_str(&manifest).unwrap(), &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();

    let names: Vec<String> = CHANNELS.iter().map(|(n, _)| n.to_string()).collect();
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(), session: "dmpcreplay".into(), node: "feeder".into(), node_id: 1,
                roster: vec!["uav1".into(), "feeder".into()], channels: names, clock_domain: "wall".into(),
                audit_queue_drops: 0, records_written: 0, complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext {
        session: "dmpcreplay".into(), node: "feeder".into(), node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: CHANNELS.iter().enumerate().map(|(i, (n, q))| ChannelSpec { id: i as u32, name: n.to_string(), qos: *q }).collect(),
    };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 18).unwrap();
    for ch in INPUTS {
        feeder.declare_out(ch).unwrap();
    }
    feeder.declare_in(PLAN_OUT, &[0]).unwrap();
    feeder.declare_in(SETPOINT, &[0]).unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_millis(300));

    // Round by round: after tick k, wait for the module's round-k outputs
    // the reference has, so no input queue ever holds more than one round.
    let mut want: BTreeMap<u64, usize> = BTreeMap::new();
    for (round, _, _) in &expected {
        *want.entry(*round).or_default() += 1;
    }
    let mut got: Vec<(u64, u32, Vec<u8>)> = Vec::new();
    for (i, (round, port, payload)) in inputs.iter().enumerate() {
        feeder.publish(*port, *round, i as i64, payload).unwrap();
        if *port != TICK {
            continue;
        }
        let need = want.get(round).copied().unwrap_or(0);
        let deadline = Instant::now() + Duration::from_secs(10);
        while got.iter().filter(|(r, _, _)| r == round).count() < need && Instant::now() < deadline {
            feeder.wait(Duration::from_millis(50));
            got.extend(feeder.drain().into_iter().map(|f| (f.header.round, f.header.channel, f.payload)));
        }
    }
    let mut idle = 0;
    while idle < 10 {
        std::thread::sleep(Duration::from_millis(50));
        let more: Vec<_> = feeder.drain().into_iter().map(|f| (f.header.round, f.header.channel, f.payload)).collect();
        idle = if more.is_empty() { idle + 1 } else { 0 };
        got.extend(more);
    }
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    feeder.close();
    audit.finish().unwrap();

    let module = &summary.plugins[0];
    println!(
        "plan-dmpc {} domain={} steps={} consumed={} published={}; inputs {}, outputs {} (reference {})",
        module.state, module.domain_state, module.steps, module.consumed, module.published, inputs.len(), got.len(), expected.len()
    );
    assert!(module.last_error.is_none(), "{module:?}");
    assert_eq!(module.domain_state, "rolling");
    for port in [PLAN_OUT, SETPOINT] {
        let g: Vec<_> = got.iter().filter(|(_, p, _)| *p == port).collect();
        let e: Vec<_> = expected.iter().filter(|(_, p, _)| *p == port).collect();
        assert_eq!(g.len(), e.len(), "port {port}: {} samples, reference {}", g.len(), e.len());
        if let Some(i) = g.iter().zip(&e).position(|(a, b)| a != b) {
            panic!("port {port}: sample {i} (round {} vs {}) differs from the reference", g[i].0, e[i].0);
        }
    }
}
