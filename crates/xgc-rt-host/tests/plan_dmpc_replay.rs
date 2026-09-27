//! plan-dmpc gate: reproduce fleet-replay outputs byte-for-byte on the current
//! 16-port PlanDmpc API. The test input bridge feeds rounds-equivalent neighbor
//! visibility (source round < beat, envelope = beat). No second ≤k-1 buffer.
//!
//! Requires PLAN_DMPC_CORE_PREFIX, PLAN_DMPC_ACADOS_PREFIX,
//! FORMATION_GENERATOR_ROOT, DMPC_FLEET_REPLAY. Missing paths fail (no skip-pass).
//! LD_LIBRARY_PATH puts acados lib ahead of system /usr/local.

mod common;

use common::plan_dmpc_bridge::{self as bridge, old, port, TimelineFeed, HOLD};

use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::clock::ManualClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

/// xgc.position_target/1 type_mask of the node's position hold.
const POSITION_HOLD_MASK: u16 = 8 | 16 | 32 | 64 | 128 | 256 | 2048;

static SERIAL: Mutex<()> = Mutex::new(());

#[test]
fn plan_dmpc_module_reproduces_the_fleet_replay_without_the_scene() {
    replay("plan-dmpc-replay", "knot_fs150_uav", 5, 1, &[], "", 60, 0);
}

#[test]
fn plan_dmpc_module_reproduces_the_fleet_replay_with_the_knot_fs150_scene() {
    replay(
        "plan-dmpc-replay-knot",
        "knot_fs150_full_uav",
        5,
        1,
        &["--scene", "config/scenarios/knot_fs150/scene.yaml", "--scene-gap", "30:40"],
        "",
        80,
        5,
    );
}

#[test]
fn plan_dmpc_module_reproduces_the_fleet_replay_with_the_mixed_circle_mover() {
    replay(
        "plan-dmpc-replay-mixed",
        "mixed_circle_agent",
        9,
        1,
        &[
            "--scene",
            "config/scenarios/mixed_circle/scene.yaml",
            "--spawn",
            "config/scenarios/mixed_circle/swarm_pose.yaml",
        ],
        "",
        80,
        0,
    );
}

#[test]
fn plan_dmpc_module_reproduces_the_fleet_replay_in_act1_pass_through() {
    replay(
        "plan-dmpc-replay-pass-through",
        "act1_mega_pass_through_moved_uav",
        8,
        1,
        &[
            "--scene",
            "config/scenarios/act1_mega/scene/scene.yaml",
            "--spawn",
            "config/scenarios/act1_mega/scene/mission.yaml",
            "--plant",
            "ideal",
        ],
        ", pass_through_clock = \"input\"",
        80,
        0,
    );
}

#[test]
fn plan_dmpc_module_reproduces_the_fleet_replay_for_a_mixed_circle_scout() {
    replay(
        "plan-dmpc-replay-scout",
        "mixed_circle_agent",
        9,
        6,
        &[
            "--scene",
            "config/scenarios/mixed_circle/scene.yaml",
            "--spawn",
            "config/scenarios/mixed_circle/swarm_pose.yaml",
        ],
        "",
        80,
        0,
    );
}

fn replay(
    name: &str,
    manifest: &str,
    robots: u32,
    record: u32,
    options: &[&str],
    config_extra: &str,
    rounds: u64,
    holds: usize,
) {
    let _serial = SERIAL.lock().unwrap_or_else(|e| e.into_inner());
    let env = bridge::require_env();
    let lib = bridge::plan_dmpc_lib(&env);
    let ld = bridge::library_path(&env);

    let dir = common::scratch(name);
    let manifests: Vec<PathBuf> = (1..=robots)
        .map(|i| {
            env.formation_generator_root
                .join(format!("test/replay/plan_dmpc/{manifest}{i}.yaml"))
        })
        .collect();
    let (inputs_path, outputs_path) = (dir.join("inputs.bin"), dir.join("outputs.bin"));
    let mut command = Command::new(&env.fleet_replay);
    command
        .env("LD_LIBRARY_PATH", &ld)
        .args([
            "--record",
            &record.to_string(),
            "--rounds",
            &rounds.to_string(),
            "--hold",
            &HOLD.to_string(),
        ])
        .arg("--inputs")
        .arg(&inputs_path)
        .arg("--outputs")
        .arg(&outputs_path);
    for option in options {
        if option.starts_with("config/") {
            command.arg(env.formation_generator_root.join(option));
        } else {
            command.arg(option);
        }
    }
    let status = command.args(&manifests).status().expect("run dmpc_fleet_replay");
    assert!(status.success(), "dmpc_fleet_replay failed");

    let inputs = bridge::read_records(&inputs_path);
    let expected = bridge::read_records(&outputs_path);
    let expected_rolling = expected
        .iter()
        .filter(|r| (r.port == old::SETPOINT || r.port == old::PLANAR_SETPOINT) && r.round >= HOLD)
        .count() as u64;
    assert!(
        expected_rolling > (rounds - HOLD) * 9 / 10,
        "the reference must actually fly: {expected_rolling} rolling setpoints"
    );
    let plan_rounds: Vec<u64> = expected
        .iter()
        .filter(|r| r.port == old::PLAN_OUT)
        .map(|r| r.round)
        .collect();
    let hold_rounds: Vec<u64> = expected
        .iter()
        .filter(|r| {
            r.port == old::SETPOINT
                && !plan_rounds.contains(&r.round)
                && r.payload.len() >= 98
                && u16::from_le_bytes([r.payload[96], r.payload[97]]) == POSITION_HOLD_MASK
        })
        .map(|r| r.round)
        .collect();
    assert_eq!(hold_rounds.len(), holds, "scene-hold rounds of the reference: {hold_rounds:?}");

    let beats = bridge::beats_from_records(&inputs);
    assert_eq!(beats.len() as u64, rounds, "beats vs rounds");

    let channels: String = bridge::CHANNELS
        .iter()
        .map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{}\"\n", format!("{q:?}").to_lowercase()))
        .collect();
    let t0_ns = (beats[0].trigger_time * 1e9).round() as i64;
    let manifest_toml = format!(
        r#"
[session]
id = "dmpcreplay"
node = "uav1"
roster = ["uav1", "feeder"]
period_ms = 100
start_delay_ms = 0
epoch_ns = {epoch_ns}

[transport]
kind = "loopback"

[audit]
dir = "audit"

{channels}
[[plugin]]
name = "plan-dmpc"
path = "{lib}"
trigger = "on_round"
step_budget_ms = 1000.0
config = {{ manifest = "{param_manifest}", self_id = {self_id}, timeline_authority = {self_id}, scene_id = "{scene_id}"{config_extra} }}
bind = {{ paired_state = {{ channel = "paired_state", from = ["feeder"] }}, controller_state = {{ channel = "controller_state", from = ["feeder"] }}, scene_snapshot = {{ channel = "scene_snapshot", from = ["feeder"] }}, scene_heartbeat = {{ channel = "scene_heartbeat", from = ["feeder"] }}, timeline_commit = {{ channel = "timeline_commit", from = ["feeder"] }}, neighbor_plan = {{ channel = "neighbor_plan", from = ["feeder"] }}, neighbor_position = {{ channel = "neighbor_position", from = ["feeder"] }}, sync_trigger = {{ channel = "sync_trigger", from = ["feeder"] }}, clock = {{ channel = "clock", from = ["feeder"] }}, position_target = {{ channel = "position_target" }}, own_plan = {{ channel = "own_plan" }}, planar_target = {{ channel = "planar_target" }}, timeline_status = {{ channel = "timeline_status" }}, planner_status = {{ channel = "planner_status" }}, own_position = {{ channel = "own_position" }}, round_done = {{ channel = "round_done" }} }}
"#,
        epoch_ns = t0_ns,
        lib = lib.display(),
        param_manifest = manifests[record as usize - 1].display(),
        self_id = record,
        scene_id = beats.iter().find_map(|b| b.scene.as_deref()).map(bridge::scene_id).unwrap_or(bridge::SCENE_ID),
        config_extra = config_extra,
    );

    // Ensure the host process can resolve acados/HPIPM from the verified prefix.
    std::env::set_var("LD_LIBRARY_PATH", &ld);

    let bus = LoopbackBus::new();
    // Start 1ms before E0 so the first beat's clock.set actually enters round 0.
    let clock = Arc::new(ManualClock::new(t0_ns - 1_000_000));
    let host = Host::new(
        Manifest::from_toml_str(&manifest_toml).unwrap(),
        &dir,
        Box::new(LoopbackTransport::new(bus.clone())),
        clock.clone(),
        HostOptions::default(),
    )
    .unwrap();

    let names: Vec<String> = bridge::CHANNELS.iter().map(|(n, _)| n.to_string()).collect();
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(),
                session: "dmpcreplay".into(),
                node: "feeder".into(),
                node_id: 1,
                roster: vec!["uav1".into(), "feeder".into()],
                channels: names,
                clock_domain: "sim".into(),
                audit_queue_drops: 0,
                records_written: 0,
                complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext {
        session: "dmpcreplay".into(),
        node: "feeder".into(),
        node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: bridge::CHANNELS
            .iter()
            .enumerate()
            .map(|(i, (n, q))| ChannelSpec {
                id: i as u32,
                name: n.to_string(),
                qos: *q,
            })
            .collect(),
    };
    let feeder = Endpoint::open(
        Box::new(LoopbackTransport::new(bus.clone())),
        &ctx,
        clock.clone(),
        audit.clone(),
        1 << 18,
    )
    .unwrap();
    for ch in bridge::FEEDER_OUT {
        feeder.declare_out(ch).unwrap();
    }
    for ch in bridge::FEEDER_IN {
        feeder.declare_in(ch, &[0]).unwrap();
    }

    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_millis(200));

    // Empty world queued before E0. Heartbeat refreshed every beat (0.5 s wall).
    // on_round: queue all beat inputs, then set ManualClock so the host takes one step.
    let empty_scene = bridge::empty_scene_blob();
    if beats.iter().all(|b| b.scene.is_none()) {
        feeder.publish(port::SCENE_SNAPSHOT, 0, t0_ns, &empty_scene).unwrap();
    }

    let mut timeline = TimelineFeed::new("dmpcreplay");
    let mut got: Vec<(u64, u32, Vec<u8>)> = Vec::new();
    const ROUND_DONE_TIMEOUT: Duration = Duration::from_secs(5);

    for beat in &beats {
        let t_ns = (beat.trigger_time * 1e9).round() as i64;

        if let Some(scene) = &beat.scene {
            feeder.publish(port::SCENE_SNAPSHOT, beat.beat, t_ns, scene).unwrap();
        }
        feeder
            .publish(port::SCENE_HEARTBEAT, beat.beat, t_ns, &bridge::scene_heartbeat_age(beat.scene_age))
            .unwrap();
        feeder
            .publish(port::PAIRED, beat.beat, t_ns, &beat.paired)
            .unwrap();
        feeder
            .publish(port::CONTROLLER, beat.beat, t_ns, &beat.controller)
            .unwrap();
        let commit = timeline.commit_for_beat(beat.beat);
        feeder
            .publish(port::TIMELINE_COMMIT, beat.beat, t_ns, &commit)
            .unwrap();
        // Rounds-equivalent: envelope round = current beat (not source round).
        for plan in &beat.neighbors {
            feeder
                .publish(port::NEIGHBOR_PLAN, beat.beat, t_ns, plan)
                .unwrap();
        }
        // The plugin processes the planner trigger before draining its explicit
        // hold-clock queue. Preserve every timer sample following this beat.
        for sample in &beat.clocks_after {
            feeder.publish(port::CLOCK, beat.beat, t_ns, sample).unwrap();
        }
        feeder
            .publish(port::SYNC_TRIGGER, beat.beat, t_ns, &beat.sync_trigger)
            .unwrap();

        // One host step: stamp+1ms keeps pose/controller stamps from looking future.
        clock.set(t_ns + 1_000_000);
        if let Err(reason) = wait_round_done(&feeder, &mut got, beat.beat, ROUND_DONE_TIMEOUT) {
            stop.store(true, Ordering::Relaxed);
            let _ = runner.join();
            feeder.close();
            let _ = audit.finish();
            panic!("{reason}");
        }
    }

    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    drain_feeder(&feeder, &mut got);
    feeder.close();
    audit.finish().unwrap();

    let module = &summary.plugins[0];
    let mapped = mapped_outputs(&got);
    println!(
        "plan-dmpc {} domain={} steps={} consumed={} published={}; beats {}, mapped {}, reference {}",
        module.state,
        module.domain_state,
        module.steps,
        module.consumed,
        module.published,
        beats.len(),
        mapped.len(),
        expected
            .iter()
            .filter(|r| r.port == old::PLAN_OUT || r.port == old::SETPOINT || r.port == old::PLANAR_SETPOINT)
            .count()
    );
    assert!(module.last_error.is_none(), "{module:?}");
    assert!(
        module.domain_state == "OPTIMIZING_ROLLING" || module.domain_state == "rolling",
        "expected rolling lifecycle, got {}",
        module.domain_state
    );

    for (old_port, new_port) in [
        (old::PLAN_OUT, port::OWN_PLAN),
        (old::SETPOINT, port::POSITION_TARGET),
        (old::PLANAR_SETPOINT, port::PLANAR_TARGET),
    ] {
        let g: Vec<_> = mapped.iter().filter(|(_, p, _)| *p == new_port).collect();
        let e: Vec<_> = expected.iter().filter(|r| r.port == old_port).collect();
        assert_eq!(
            g.len(),
            e.len(),
            "port {old_port}->{new_port}: {} samples, reference {}",
            g.len(),
            e.len()
        );
        if let Some(i) = g.iter().zip(&e).position(|(a, b)| a.0 != b.round || a.2.as_slice() != b.payload.as_slice()) {
            let (ga, ea) = (g[i], e[i]);
            let off = ga
                .2
                .iter()
                .zip(ea.payload.iter())
                .position(|(a, b)| a != b)
                .unwrap_or(ga.2.len().min(ea.payload.len()));
            std::fs::write("/tmp/p2n-replay-actual.bin", &ga.2).unwrap();
            std::fs::write("/tmp/p2n-replay-expected.bin", &ea.payload).unwrap();
            eprintln!(
                "p2n-replay mismatch dump: envelope_round={} first_byte_offset={} actual=/tmp/p2n-replay-actual.bin expected=/tmp/p2n-replay-expected.bin",
                ga.0, off
            );
            panic!(
                "port {old_port}->{new_port}: sample {i} (round {} vs {}, lens {} vs {}, first byte off {off}) differs; leading ok {i}",
                ga.0, ea.round, ga.2.len(), ea.payload.len()
            );
        }
    }
    let _ = env;
}

fn drain_feeder(feeder: &Endpoint, got: &mut Vec<(u64, u32, Vec<u8>)>) {
    got.extend(
        feeder
            .drain()
            .into_iter()
            .map(|f| (f.header.round, f.header.channel, f.payload)),
    );
}

/// Wait for plan-dmpc's round_done on envelope round `beat`. Hard fail on timeout
/// (caller stops the host). Do not schedule on expected setpoint/plan counts.
fn wait_round_done(
    feeder: &Endpoint,
    got: &mut Vec<(u64, u32, Vec<u8>)>,
    beat: u64,
    timeout: Duration,
) -> Result<(), String> {
    let deadline = Instant::now() + timeout;
    loop {
        drain_feeder(feeder, got);
        if got.iter().any(|(r, p, _)| *p == port::ROUND_DONE && *r == beat) {
            return Ok(());
        }
        if Instant::now() >= deadline {
            let done: Vec<u64> = got
                .iter()
                .filter(|(_, p, _)| *p == port::ROUND_DONE)
                .map(|(r, _, _)| *r)
                .collect();
            return Err(format!(
                "round_done missing for beat {beat} within {timeout:?}; have {done:?}"
            ));
        }
        feeder.wait(Duration::from_millis(5));
    }
}

/// Count only the outputs that map to old fleet-replay ports (ignore status).
fn mapped_outputs(got: &[(u64, u32, Vec<u8>)]) -> Vec<(u64, u32, Vec<u8>)> {
    got.iter()
        .filter(|(_, p, _)| {
            *p == port::OWN_PLAN || *p == port::POSITION_TARGET || *p == port::PLANAR_TARGET
        })
        .cloned()
        .collect()
}
