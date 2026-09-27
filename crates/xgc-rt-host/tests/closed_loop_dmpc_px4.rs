//! Closed loop in one aggregator: plan-dmpc (the TRO DMPC planner) feeds
//! ctl-px4 (the PX4 multirotor controller core) in memory, and a software
//! plant with a PX4/MAVROS stand-in closes the loop, deterministically.
//!
//!   plant truth -> own_state -> plan-dmpc -> setpoint -(memory)-> ctl-px4
//!   (alg_setpoint, px4_local, Custom1) -> setpoint_raw/local -> stand-in PX4
//!   -> plant -> estimate / local pose / velocity / IMU / VRPN / FCU state ->
//!   ctl-px4
//!
//! The robot is robot 1 of the unmodified knot_fs150 fleet with its scene:
//! the shared scene and the four neighbors' plans are the academic fleet
//! replay's recording (open loop; they do not see this robot's closed-loop
//! plans), its own state is the plant's.
//!
//! Lockstep, no wall-clock timing: the host runs on a manual clock that this
//! test sets. ctl-px4 runs with time_source = "input" and reports each tick
//! on tick_done; plan-dmpc reports each round on round_done. Every
//! millisecond the test publishes the sensor samples due, a clock sample, and
//! waits for the controller's tick before stepping the plant with the
//! controller's latest setpoint; every 100 ms (from t0 = 1000 s, the
//! recording's rounds) it gives plan-dmpc the round's inputs and waits for
//! the round. plan-dmpc's setpoint reaches ctl-px4 in memory stamped with
//! the manual clock. The flight: SelfCheck, Ready, "takeoff" (20 s before
//! t0), Hover, "custom1" once the planner commands, then the planner's
//! rounds (hold 10 rounds, then rolling).
//!
//! The plant and stand-in are px4_standin.py's position mode (tracking
//! backend px4_local): velocity command 1.5 (p_sp - p) + v_sp, at most
//! 1.5 m/s, acceleration (v_cmd - v) / 0.3 within 3 m/s^2, attitude along
//! the thrust, arming and set_mode answered at once; the estimate is the
//! plant truth (ESKF not in the loop).
//!
//! Gate: two flights byte-equal (every planner plan and setpoint, every
//! controller setpoint, the plant trajectory); Custom1 before the rolling
//! rounds; the robot tracks the planner's setpoints and travels.
//!
//! Needs DMPC_LIB_DIR and FORMATION_GENERATOR_ROOT (as plan_dmpc_replay.rs)
//! and PX4_CORE_LIB_DIR (plus the other build-ctl-px4.sh variables).
//! Without them the test prints why and passes.

mod common;

use common::px4_plant::{command, f64_at, f64s, cstr, read_records, Plant};

use std::collections::VecDeque;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::clock::ManualClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const T0_MS: i64 = 1_000_000; // the recording's round 0: 1000 s
const START_MS: i64 = T0_MS - 20_000;
const PERIOD_MS: i64 = 100;
const ROUNDS: i64 = 80;
const HOLD: u64 = 10;

// Channels; the name is the port name on both modules where they meet.
const CHANNELS: [(&str, Qos); 22] = [
    ("formation_tick", Qos::Control), // 0  -> plan-dmpc
    ("plan_in", Qos::Control),        // 1  -> plan-dmpc
    ("plan_out", Qos::Control),       // 2  plan-dmpc ->
    ("own_state", Qos::State),        // 3  -> plan-dmpc
    ("alg_setpoint", Qos::Control),   // 4  plan-dmpc setpoint -> ctl-px4 alg_setpoint (memory), and ->
    ("scene_snapshot", Qos::Event),   // 5  -> plan-dmpc
    ("scene_state", Qos::State),      // 6  -> plan-dmpc
    ("round_done", Qos::Event),       // 7  plan-dmpc ->
    ("estimate", Qos::State),         // 8  -> ctl-px4
    ("local_pose", Qos::State),       // 9
    ("local_velocity", Qos::State),   // 10
    ("imu", Qos::State),              // 11
    ("fcu_state", Qos::State),        // 12
    ("battery", Qos::State),          // 13
    ("vrpn_pose", Qos::State),        // 14
    ("command", Qos::Event),          // 15
    ("clock", Qos::Event),            // 16
    ("setpoint", Qos::Control),       // 17 ctl-px4 ->
    ("fcu_request", Qos::Event),      // 18
    ("status", Qos::State),           // 19
    ("tick_done", Qos::Event),        // 20
    ("planner_clock", Qos::Event),    // 21 (unused: pass-through only)
];
const FEEDER_OUT: [u32; 15] = [0, 1, 3, 5, 6, 8, 9, 10, 11, 12, 13, 14, 15, 16, 21];
const FEEDER_IN: [u32; 7] = [2, 4, 7, 17, 18, 19, 20];
// Recorded ports (dmpc_fleet_replay) -> channel.
fn recorded_channel(port: u32) -> Option<u32> {
    match port {
        0 => Some(0),
        1 => Some(1),
        5 => Some(5),
        6 => Some(6),
        _ => None, // own_state (3) comes from the plant
    }
}

fn env_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name).map(PathBuf::from).filter(|p| p.exists())
}

// --- one flight ------------------------------------------------------------

struct Flight {
    planner: Vec<(u64, u32, Vec<u8>)>, // plan-dmpc outputs (round, channel, bytes)
    controller: Vec<Vec<u8>>,          // ctl-px4 setpoints
    states: Vec<(i64, String)>,        // ctl-px4 control states, with the ms they appeared
    truth: Vec<[f64; 3]>,              // plant position per ms from START_MS
    custom1_ms: Option<i64>,
}

struct Inbox<'a> {
    feeder: &'a Endpoint,
    frames: VecDeque<(u64, u32, Vec<u8>)>,
}

impl Inbox<'_> {
    /// Wait until `done` has seen a matching frame; every frame drained on
    /// the way is kept in order.
    fn until(&mut self, what: &str, done: impl Fn(&(u64, u32, Vec<u8>)) -> bool) {
        let deadline = Instant::now() + Duration::from_secs(30);
        loop {
            if self.frames.iter().any(&done) {
                return;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            self.feeder.wait(Duration::from_millis(20));
            self.frames.extend(self.feeder.drain().into_iter().map(|f| (f.header.round, f.header.channel, f.payload)));
        }
    }
}

fn fly(name: &str, plan_dmpc: &Path, ctl_px4: &Path, manifest: &Path, recorded: &[(u64, u32, Vec<u8>)]) -> Flight {
    let dir = common::scratch(name);
    let channels: String = CHANNELS
        .iter()
        .map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{}\"\n", format!("{q:?}").to_lowercase()))
        .collect();
    let feed = |n: &str| format!("{n} = {{ channel = \"{n}\", from = [\"feeder\"] }}");
    let out = |port: &str, channel: &str| format!("{port} = {{ channel = \"{channel}\" }}");
    let planner_binds = [
        feed("formation_tick"), feed("plan_in"), feed("own_state"), feed("scene_snapshot"), feed("scene_state"),
        out("plan_out", "plan_out"), out("setpoint", "alg_setpoint"), out("round_done", "round_done"),
    ]
    .join(", ");
    let controller_binds = [
        feed("estimate"), feed("local_pose"), feed("local_velocity"), feed("imu"), feed("fcu_state"), feed("battery"),
        feed("vrpn_pose"), feed("command"), feed("clock"),
        // In memory: this node's own writer (plan-dmpc).
        "alg_setpoint = { channel = \"alg_setpoint\", from = [\"uav1\"] }".to_string(),
        out("setpoint", "setpoint"), out("fcu_request", "fcu_request"), out("status", "status"), out("tick_done", "tick_done"),
    ]
    .join(", ");
    let manifest_text = format!(
        r#"
[session]
id = "closedloop"
node = "uav1"
roster = ["uav1", "feeder"]
period_ms = 100
start_delay_ms = 0

[transport]
kind = "loopback"

[audit]
dir = "audit"

{channels}
[[plugin]]
name = "plan-dmpc"
path = "{plan_dmpc}"
trigger = "on_dirty"
step_budget_ms = 10000.0
config = {{ param_manifest = "{manifest}" }}
bind = {{ {planner_binds} }}

[[plugin]]
name = "ctl-px4"
path = "{ctl_px4}"
trigger = "on_dirty"
step_budget_ms = 10000.0
config = {{ time_source = "input", tracking_backend = "px4_local" }}
bind = {{ {controller_binds} }}
"#,
        plan_dmpc = plan_dmpc.display(),
        ctl_px4 = ctl_px4.display(),
        manifest = manifest.display(),
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(ManualClock::new(START_MS * 1_000_000));
    let host = Host::new(Manifest::from_toml_str(&manifest_text).unwrap(), &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();
    let names: Vec<String> = CHANNELS.iter().map(|(n, _)| n.to_string()).collect();
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(), session: "closedloop".into(), node: "feeder".into(), node_id: 1,
                roster: vec!["uav1".into(), "feeder".into()], channels: names, clock_domain: "sim".into(),
                audit_queue_drops: 0, records_written: 0, complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext {
        session: "closedloop".into(), node: "feeder".into(), node_id: 1,
        roster: vec!["uav1".into(), "feeder".into()],
        channels: CHANNELS.iter().enumerate().map(|(i, (n, q))| ChannelSpec { id: i as u32, name: n.to_string(), qos: *q }).collect(),
    };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 20).unwrap();
    for ch in FEEDER_OUT {
        feeder.declare_out(ch).unwrap();
    }
    for ch in FEEDER_IN {
        feeder.declare_in(ch, &[0]).unwrap();
    }
    let stop = Arc::new(AtomicBool::new(false));
    let runner = {
        let stop = stop.clone();
        std::thread::spawn(move || host.run(&stop).unwrap())
    };
    std::thread::sleep(Duration::from_millis(300));

    // The recording, split at each tick: the inputs of round k end with its tick.
    let mut rounds: Vec<Vec<(u64, u32, Vec<u8>)>> = vec![Vec::new()];
    for r in recorded {
        if let Some(ch) = recorded_channel(r.1) {
            rounds.last_mut().unwrap().push((r.0, ch, r.2.clone()));
            if ch == 0 {
                rounds.push(Vec::new());
            }
        }
    }

    // Knot_fs150 robot 1 on the ground under its seed point.
    let mut plant = Plant::new([1.2, 0.0, 0.0]);
    let mut inbox = Inbox { feeder: &feeder, frames: VecDeque::new() };
    let mut flight = Flight { planner: Vec::new(), controller: Vec::new(), states: Vec::new(), truth: Vec::new(), custom1_ms: None };
    let (mut takeoff_sent, mut custom1_sent) = (false, false);
    let mut planner_commands = false;
    let end_ms = T0_MS + ROUNDS * PERIOD_MS;
    for ms in START_MS..end_ms {
        let t_ns = ms * 1_000_000;
        let t = ms as f64 * 1e-3;
        clock.set(t_ns);
        // A planner round: its recorded inputs, the plant's state, the tick.
        if ms >= T0_MS && (ms - T0_MS) % PERIOD_MS == 0 {
            let k = ((ms - T0_MS) / PERIOD_MS) as u64;
            feeder.publish(3, k, t_ns, &plant.rigid_state(t)).unwrap();
            for (round, ch, payload) in &rounds[k as usize] {
                feeder.publish(*ch, *round, t_ns, payload).unwrap();
            }
            inbox.until("round_done", |f| f.1 == 7 && f.0 == k);
        }
        // The sensors due at t.
        let step = ms - START_MS;
        if step % 5 == 0 {
            feeder.publish(8, 0, t_ns, &plant.estimate(t)).unwrap();
            feeder.publish(11, 0, t_ns, &plant.imu(t)).unwrap();
        }
        if step % 10 == 0 {
            feeder.publish(14, 0, t_ns, &plant.pose(t)).unwrap();
            feeder.publish(9, 0, t_ns, &plant.pose(t)).unwrap();
            feeder.publish(10, 0, t_ns, &plant.twist(t)).unwrap();
        }
        if step % 100 == 0 {
            feeder.publish(12, 0, t_ns, &plant.fcu_state(t)).unwrap();
        }
        if step % 1000 == 0 {
            feeder.publish(13, 0, t_ns, &f64s(&[t, 16.4, 0.9])).unwrap();
        }
        let state = flight.states.last().map(|s| s.1.clone()).unwrap_or_default();
        if !takeoff_sent && state == "Ready" {
            feeder.publish(15, 0, t_ns, &command("takeoff")).unwrap();
            takeoff_sent = true;
        }
        if !custom1_sent && state == "Hover" && planner_commands {
            feeder.publish(15, 0, t_ns, &command("custom1")).unwrap();
            custom1_sent = true;
        }
        // The controller's tick at t.
        feeder.publish(16, 0, t_ns, &(t + 1e-7).to_le_bytes()).unwrap();
        inbox.until("tick_done", |f| f.1 == 20 && (f64_at(&f.2, 0) - t).abs() < 1e-6);
        while let Some((round, ch, payload)) = inbox.frames.pop_front() {
            match ch {
                2 | 4 => {
                    planner_commands |= ch == 4;
                    flight.planner.push((round, ch, payload));
                }
                17 => {
                    plant.setpoint = Some(payload.clone());
                    flight.controller.push(payload);
                }
                18 => plant.fcu_request(&payload),
                19 => {
                    let s = cstr(&payload[8..]);
                    if flight.states.last().map_or(true, |l| l.1 != s) {
                        if s == "Custom1" && flight.custom1_ms.is_none() {
                            flight.custom1_ms = Some(ms);
                        }
                        flight.states.push((ms, s));
                    }
                }
                _ => {}
            }
        }
        plant.step(1e-3);
        flight.truth.push(plant.p);
    }
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    feeder.close();
    audit.finish().unwrap();
    for module in &summary.plugins {
        println!("{name}: {} {} domain={} steps={} consumed={} published={}", module.name, module.state, module.domain_state, module.steps, module.consumed, module.published);
        assert!(module.last_error.is_none(), "{module:?}");
    }
    flight
}

#[test]
fn plan_dmpc_and_ctl_px4_fly_the_knot_fs150_robot_in_closed_loop() {
    let (Some(lib_dir), Some(fg_root), Some(_px4)) =
        (env_path("DMPC_LIB_DIR"), env_path("FORMATION_GENERATOR_ROOT"), env_path("PX4_CORE_LIB_DIR"))
    else {
        eprintln!("skipped: set DMPC_LIB_DIR, FORMATION_GENERATOR_ROOT and PX4_CORE_LIB_DIR");
        return;
    };
    let out = common::workspace_root().join("target/plugin-tests/cpp");
    std::fs::create_dir_all(&out).unwrap();
    let plan_dmpc = out.join("libplan_dmpc_closed_loop.so");
    let status = Command::new(common::workspace_root().join("scripts/build-plan-dmpc.sh")).arg(&plan_dmpc).status().unwrap();
    assert!(status.success(), "building plan-dmpc failed");
    let ctl_px4 = out.join("libctl_px4_closed_loop.so");
    let status = Command::new(common::workspace_root().join("scripts/build-ctl-px4.sh")).arg(&ctl_px4).status().unwrap();
    assert!(status.success(), "building ctl-px4 failed");

    // The recording: robot 1 of the knot_fs150 fleet with its scene.
    let tool = env_path("DMPC_FLEET_REPLAY").unwrap_or_else(|| lib_dir.join("dmpc_fleet_replay"));
    let dir = common::scratch("closed-loop-reference");
    let manifests: Vec<PathBuf> = (1..=5).map(|i| fg_root.join(format!("test/replay/plan_dmpc/knot_fs150_full_uav{i}.yaml"))).collect();
    let status = Command::new(&tool)
        .args(["--record", "1", "--rounds", &ROUNDS.to_string(), "--hold", &HOLD.to_string()])
        .arg("--inputs").arg(dir.join("inputs.bin"))
        .arg("--outputs").arg(dir.join("outputs.bin"))
        .arg("--scene").arg(fg_root.join("config/scenarios/knot_fs150/scene.yaml"))
        .args(&manifests)
        .status()
        .unwrap();
    assert!(status.success(), "dmpc_fleet_replay failed");
    let recorded = read_records(&dir.join("inputs.bin"));

    let a = fly("closed-loop-a", &plan_dmpc, &ctl_px4, &manifests[0], &recorded);
    let b = fly("closed-loop-b", &plan_dmpc, &ctl_px4, &manifests[0], &recorded);

    // Deterministic: the two flights are the same, byte for byte.
    assert_eq!(a.states, b.states, "controller states differ between flights");
    assert_eq!(a.planner, b.planner, "planner outputs differ between flights");
    assert_eq!(a.controller, b.controller, "controller setpoints differ between flights");
    let bits = |f: &Flight| f.truth.iter().flat_map(|p| p.iter().map(|x| x.to_bits())).collect::<Vec<_>>();
    assert!(bits(&a) == bits(&b), "plant trajectories differ between flights");

    // The flight: Custom1 before the rolling rounds.
    println!("states: {:?}", a.states);
    let custom1 = a.custom1_ms.expect("ctl-px4 never entered Custom1");
    assert!(custom1 < T0_MS + HOLD as i64 * PERIOD_MS, "Custom1 at {custom1} ms, after the rolling rounds began");
    assert_eq!(a.states.last().unwrap().1, "Custom1", "Custom1 not held to the end");

    // Tracking: each rolling planner setpoint (stamped with its activation
    // time) against the plant at that time.
    let mut errors = Vec::new();
    let mut first = None;
    for (round, ch, d) in &a.planner {
        if *ch != 4 || *round < HOLD {
            continue;
        }
        let ms = (f64_at(d, 0) * 1e3).round() as i64;
        let Some(p) = a.truth.get((ms - START_MS) as usize) else { continue };
        let sp = [f64_at(d, 1), f64_at(d, 2), f64_at(d, 3)];
        first.get_or_insert(sp);
        errors.push(((p[0] - sp[0]).powi(2) + (p[1] - sp[1]).powi(2) + (p[2] - sp[2]).powi(2)).sqrt());
    }
    errors.sort_by(|x, y| x.partial_cmp(y).unwrap());
    let (p50, max) = (errors[errors.len() / 2], errors[errors.len() - 1]);
    let (p0, p1) = (first.unwrap(), a.truth.last().unwrap());
    let travel = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2)).sqrt();
    println!(
        "closed loop: {} rolling setpoints, tracking error p50 {p50:.3} m max {max:.3} m, travel {travel:.2} m, {} controller setpoints",
        errors.len(),
        a.controller.len()
    );
    assert!(errors.len() as i64 >= ROUNDS - HOLD as i64 - 1, "too few rolling setpoints: {}", errors.len());
    assert!(p50 < 0.15 && max < 0.5, "tracking error p50 {p50:.3} m, max {max:.3} m");
    assert!(travel > 1.0, "the robot travelled {travel:.2} m");
}
