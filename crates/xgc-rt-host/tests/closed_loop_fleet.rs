//! Fleet closed loop: a whole TRO DMPC fleet flies together, each robot its
//! own aggregator (as deployed on a robot), the plans going robot to robot
//! over the link, deterministically.
//!
//! Per robot i: host `ri` holds plan-dmpc and, for a UAV, ctl-px4 (the
//! planner's setpoint reaches ctl-px4's alg_setpoint in memory, px4_local,
//! Custom1); node `feederi` is this test's plant for that robot: a UAV with
//! a PX4/MAVROS stand-in (common::px4_plant::Plant), or a Scout following
//! the planner's planar setpoint (GroundPlant; there is no UGV controller
//! module in this tree). Every plan-dmpc publishes its plan on the one
//! channel `plan` and reads the other robots' plans from it (plan_in from
//! the other robots' nodes): nothing recorded, every robot sees its peers'
//! closed-loop plans. The scene is the scenario's scene.yaml as the academic
//! fleet replay plays it.
//!
//! Lockstep on one manual clock (as closed_loop_dmpc_px4.rs, for every
//! robot at once): every 100 ms from t0 = 1000 s, each robot's measured
//! state, scene state and formation tick, then every robot's round_done;
//! every millisecond, each UAV's sensors and clock sample, then every
//! controller's tick_done, then all plants step. A robot's round-k plan is
//! sent before its round_done, so it waits in every peer's receive queue
//! ahead of that peer's tick k + 1.
//!
//! Fleets: the unmodified knot_fs150 (5 UAVs, two static posts), and the
//! unmodified mixed_circle (5 UAVs, 4 Scouts, static obstacles and a
//! constant-velocity mover) from its spawn poses.
//!
//! Gate:
//! - two flights byte-equal (every robot's plans, planner setpoints,
//!   controller setpoints, controller states, plant trajectory);
//! - every UAV: Custom1 before the rolling rounds, held; every robot tracks
//!   its planner's rolling setpoints and travels;
//! - peers' plans of round k - 1, never later: each robot's closed-loop
//!   outputs equal an offline plan-dmpc run fed strictly in order (its
//!   measured states, the scene, and before tick k every peer plan of
//!   rounds <= k - 1 as the peers published them). A plan that reached a
//!   peer late would make that peer's closed-loop round differ (checked: the
//!   same replay with the plans one round late does not reproduce robot 1).
//!
//! Needs DMPC_LIB_DIR, FORMATION_GENERATOR_ROOT and PX4_CORE_LIB_DIR (as
//! closed_loop_dmpc_px4.rs). Without them the tests print why and pass.

mod common;

use common::px4_plant::{command, cstr, f64_at, f64s, read_records, GroundPlant, Plant};

use std::collections::{BTreeMap, VecDeque};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use xgc_rt_audit::{FileAudit, NodeMeta};
use xgc_rt_core::clock::ManualClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions, RunSummary};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const T0_MS: i64 = 1_000_000;
const START_MS: i64 = T0_MS - 20_000;
const PERIOD_MS: i64 = 100;
const ROUNDS: i64 = 80;
const HOLD: u64 = 10;

const CHANNELS: [(&str, Qos); 21] = [
    ("formation_tick", Qos::Control),  // 0  feeder -> plan-dmpc
    ("plan", Qos::Control),            // 1  plan-dmpc <-> plan-dmpc (link)
    ("own_state", Qos::State),         // 2  feeder -> plan-dmpc
    ("alg_setpoint", Qos::Control),    // 3  plan-dmpc -> ctl-px4 (memory), -> feeder
    ("scene_snapshot", Qos::Event),    // 4  feeder -> plan-dmpc
    ("scene_state", Qos::State),       // 5
    ("round_done", Qos::Event),        // 6  plan-dmpc -> feeder
    ("estimate", Qos::State),          // 7  feeder -> ctl-px4
    ("local_pose", Qos::State),        // 8
    ("local_velocity", Qos::State),    // 9
    ("imu", Qos::State),               // 10
    ("fcu_state", Qos::State),         // 11
    ("battery", Qos::State),           // 12
    ("vrpn_pose", Qos::State),         // 13
    ("command", Qos::Event),           // 14
    ("clock", Qos::Event),             // 15
    ("setpoint", Qos::Control),        // 16 ctl-px4 -> feeder
    ("fcu_request", Qos::Event),       // 17
    ("status", Qos::State),            // 18
    ("tick_done", Qos::Event),         // 19
    ("planar_setpoint", Qos::Control), // 20 plan-dmpc (Scout) -> feeder
];
const FEEDER_OUT: [u32; 13] = [0, 2, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15];
const FEEDER_IN: [u32; 8] = [1, 3, 6, 16, 17, 18, 19, 20];

// One fleet at a time (one heavy process).
static FLIGHT: Mutex<()> = Mutex::new(());

fn env_path(name: &str) -> Option<PathBuf> {
    std::env::var_os(name).map(PathBuf::from).filter(|p| p.exists())
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Kind {
    Uav,
    Scout,
}

#[derive(Clone)]
struct Member {
    kind: Kind,
    manifest: PathBuf,
    spawn: [f64; 3],
    yaw: f64,
}

fn roster(n: usize) -> Vec<String> {
    (1..=n).map(|i| format!("r{i}")).chain((1..=n).map(|i| format!("feeder{i}"))).collect()
}

fn channel_toml() -> String {
    CHANNELS
        .iter()
        .map(|(n, q)| format!("[[channel]]\nname = \"{n}\"\nqos = \"{}\"\n", format!("{q:?}").to_lowercase()))
        .collect()
}

fn channel_specs() -> Vec<ChannelSpec> {
    CHANNELS.iter().enumerate().map(|(i, (n, q))| ChannelSpec { id: i as u32, name: n.to_string(), qos: *q }).collect()
}

type Frame = (u16, u64, u32, Vec<u8>); // origin, round, channel, payload

struct Inbox {
    feeder: Arc<Endpoint>,
    frames: VecDeque<Frame>,
}

impl Inbox {
    fn drain(&mut self) {
        self.frames.extend(self.feeder.drain().into_iter().map(|f| (f.header.origin, f.header.round, f.header.channel, f.payload)));
    }

    fn until(&mut self, what: &str, done: impl Fn(&Frame) -> bool) {
        let deadline = Instant::now() + Duration::from_secs(60);
        loop {
            if self.frames.iter().any(&done) {
                return;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {what}");
            self.feeder.wait(Duration::from_millis(20));
            self.drain();
        }
    }
}

fn feeder_endpoint(bus: &Arc<LoopbackBus>, clock: &Arc<ManualClock>, dir: &Path, n: usize, i: usize) -> (Arc<Endpoint>, Arc<FileAudit>) {
    let node = format!("feeder{i}");
    let node_id = (n + i - 1) as u16;
    let audit = Arc::new(
        FileAudit::create(
            &dir.join(format!("audit-{node}")),
            NodeMeta {
                format: String::new(), session: "fleet".into(), node: node.clone(), node_id,
                roster: roster(n), channels: CHANNELS.iter().map(|(c, _)| c.to_string()).collect(), clock_domain: "sim".into(),
                audit_queue_drops: 0, records_written: 0, complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext { session: "fleet".into(), node, node_id, roster: roster(n), channels: channel_specs() };
    let ep = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 20).unwrap();
    for ch in FEEDER_OUT {
        ep.declare_out(ch).unwrap();
    }
    for ch in FEEDER_IN {
        ep.declare_in(ch, &[(i - 1) as u16]).unwrap();
    }
    (ep, audit)
}

#[allow(clippy::too_many_arguments)]
fn robot_host(bus: &Arc<LoopbackBus>, clock: &Arc<ManualClock>, dir: &Path, n: usize, i: usize, member: &Member, plan_dmpc: &Path, ctl_px4: &Path) -> Host {
    let feed = |c: &str| format!("{c} = {{ channel = \"{c}\", from = [\"feeder{i}\"] }}");
    let out = |port: &str, channel: &str| format!("{port} = {{ channel = \"{channel}\" }}");
    let peers: Vec<String> = (1..=n).filter(|&j| j != i).map(|j| format!("\"r{j}\"")).collect();
    let mut planner_binds = vec![
        feed("formation_tick"),
        format!("plan_in = {{ channel = \"plan\", from = [{}] }}", peers.join(", ")),
        feed("own_state"), feed("scene_snapshot"), feed("scene_state"),
        out("plan_out", "plan"), out("round_done", "round_done"),
    ];
    let mut controller = String::new();
    if member.kind == Kind::Uav {
        planner_binds.push(out("setpoint", "alg_setpoint"));
        let controller_binds = [
            feed("estimate"), feed("local_pose"), feed("local_velocity"), feed("imu"), feed("fcu_state"), feed("battery"),
            feed("vrpn_pose"), feed("command"), feed("clock"),
            format!("alg_setpoint = {{ channel = \"alg_setpoint\", from = [\"r{i}\"] }}"),
            out("setpoint", "setpoint"), out("fcu_request", "fcu_request"), out("status", "status"), out("tick_done", "tick_done"),
        ]
        .join(", ");
        controller = format!(
            r#"
[[plugin]]
name = "ctl-px4"
path = "{}"
trigger = "on_dirty"
step_budget_ms = 10000.0
config = {{ time_source = "input", tracking_backend = "px4_local" }}
bind = {{ {controller_binds} }}
"#,
            ctl_px4.display()
        );
    } else {
        planner_binds.push(out("planar_setpoint", "planar_setpoint"));
    }
    let roster = roster(n).iter().map(|r| format!("\"{r}\"")).collect::<Vec<_>>().join(", ");
    let text = format!(
        r#"
[session]
id = "fleet"
node = "r{i}"
roster = [{roster}]
period_ms = 100
start_delay_ms = 0

[transport]
kind = "loopback"

[audit]
dir = "audit-r{i}"

{channels}
[[plugin]]
name = "plan-dmpc"
path = "{plan_dmpc}"
trigger = "on_dirty"
step_budget_ms = 10000.0
config = {{ param_manifest = "{manifest}" }}
bind = {{ {planner_binds} }}
{controller}"#,
        channels = channel_toml(),
        plan_dmpc = plan_dmpc.display(),
        manifest = member.manifest.display(),
        planner_binds = planner_binds.join(", "),
    );
    Host::new(Manifest::from_toml_str(&text).unwrap(), dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap()
}

#[derive(Default, PartialEq)]
struct Robot {
    plans: Vec<(u64, Vec<u8>)>,      // own plans (round, payload) as published on `plan`
    setpoints: Vec<(u64, Vec<u8>)>,  // planner setpoints: position targets or planar (round, payload)
    controller: Vec<Vec<u8>>,        // ctl-px4 setpoints
    states: Vec<(i64, String)>,      // ctl-px4 control states
    truth: Vec<[f64; 3]>,            // plant position per ms
    own_states: Vec<(u64, Vec<u8>)>, // measured states given to plan-dmpc
}

enum Body {
    Air(Plant),
    Ground(GroundPlant),
}

impl Body {
    fn position(&self) -> [f64; 3] {
        match self {
            Body::Air(p) => p.p,
            Body::Ground(g) => g.p,
        }
    }

    fn rigid_state(&self, t: f64) -> Vec<u8> {
        match self {
            Body::Air(p) => p.rigid_state(t),
            Body::Ground(g) => g.rigid_state(t),
        }
    }
}

fn spawn(host: Host, stop: &Arc<AtomicBool>) -> JoinHandle<RunSummary> {
    let stop = stop.clone();
    std::thread::spawn(move || host.run(&stop).unwrap())
}

/// One fleet flight. `scene` is the recorded scene inputs (round, recorded
/// port 5 | 6, payload) and `ticks` the formation ticks per round.
fn fly(name: &str, plan_dmpc: &Path, ctl_px4: &Path, members: &[Member], scene: &[(u64, u32, Vec<u8>)], ticks: &[Vec<u8>]) -> Vec<Robot> {
    let n = members.len();
    let dir = common::scratch(name);
    let bus = LoopbackBus::new();
    let clock = Arc::new(ManualClock::new(START_MS * 1_000_000));
    let hosts: Vec<Host> = members.iter().enumerate().map(|(k, m)| robot_host(&bus, &clock, &dir, n, k + 1, m, plan_dmpc, ctl_px4)).collect();
    let (endpoints, audits): (Vec<Arc<Endpoint>>, Vec<Arc<FileAudit>>) = (1..=n).map(|i| feeder_endpoint(&bus, &clock, &dir, n, i)).unzip();
    let stop = Arc::new(AtomicBool::new(false));
    let runners: Vec<JoinHandle<RunSummary>> = hosts.into_iter().map(|h| spawn(h, &stop)).collect();
    std::thread::sleep(Duration::from_millis(500));
    let mut inboxes: Vec<Inbox> = endpoints.into_iter().map(|feeder| Inbox { feeder, frames: VecDeque::new() }).collect();

    let mut bodies: Vec<Body> = members
        .iter()
        .map(|m| match m.kind {
            Kind::Uav => Body::Air(Plant::new(m.spawn)),
            Kind::Scout => Body::Ground(GroundPlant::new(m.spawn, m.yaw)),
        })
        .collect();
    let mut robots: Vec<Robot> = (0..n).map(|_| Robot::default()).collect();
    let mut takeoff_sent = vec![false; n];
    let mut custom1_sent = vec![false; n];
    let mut commanded = vec![false; n];
    let end_ms = T0_MS + ROUNDS * PERIOD_MS;
    for ms in START_MS..end_ms {
        let t_ns = ms * 1_000_000;
        let t = ms as f64 * 1e-3;
        clock.set(t_ns);
        if ms >= T0_MS && (ms - T0_MS) % PERIOD_MS == 0 {
            let k = ((ms - T0_MS) / PERIOD_MS) as u64;
            for (i, inbox) in inboxes.iter().enumerate() {
                let state = bodies[i].rigid_state(t);
                inbox.feeder.publish(2, k, t_ns, &state).unwrap();
                robots[i].own_states.push((k, state));
                for (round, port, payload) in scene.iter().filter(|r| r.0 == k) {
                    inbox.feeder.publish(if *port == 5 { 4 } else { 5 }, *round, t_ns, payload).unwrap();
                }
                inbox.feeder.publish(0, k, t_ns, &ticks[k as usize]).unwrap();
            }
            for inbox in inboxes.iter_mut() {
                inbox.until("round_done", |f| f.2 == 6 && f.1 == k);
            }
        }
        let step = ms - START_MS;
        for (i, inbox) in inboxes.iter().enumerate() {
            let Body::Air(plant) = &bodies[i] else { continue };
            let f = &inbox.feeder;
            if step % 5 == 0 {
                f.publish(7, 0, t_ns, &plant.estimate(t)).unwrap();
                f.publish(10, 0, t_ns, &plant.imu(t)).unwrap();
            }
            if step % 10 == 0 {
                f.publish(13, 0, t_ns, &plant.pose(t)).unwrap();
                f.publish(8, 0, t_ns, &plant.pose(t)).unwrap();
                f.publish(9, 0, t_ns, &plant.twist(t)).unwrap();
            }
            if step % 100 == 0 {
                f.publish(11, 0, t_ns, &plant.fcu_state(t)).unwrap();
            }
            if step % 1000 == 0 {
                f.publish(12, 0, t_ns, &f64s(&[t, 16.4, 0.9])).unwrap();
            }
            let state = robots[i].states.last().map(|s| s.1.clone()).unwrap_or_default();
            if !takeoff_sent[i] && state == "Ready" {
                f.publish(14, 0, t_ns, &command("takeoff")).unwrap();
                takeoff_sent[i] = true;
            }
            if !custom1_sent[i] && state == "Hover" && commanded[i] {
                f.publish(14, 0, t_ns, &command("custom1")).unwrap();
                custom1_sent[i] = true;
            }
            f.publish(15, 0, t_ns, &(t + 1e-7).to_le_bytes()).unwrap();
        }
        for (i, inbox) in inboxes.iter_mut().enumerate() {
            if members[i].kind == Kind::Uav {
                inbox.until("tick_done", |f| f.2 == 19 && (f64_at(&f.3, 0) - t).abs() < 1e-6);
            } else {
                // A Scout's outputs all come from its planner round, which
                // round_done already waited for.
                inbox.drain();
            }
            while let Some((_, round, ch, payload)) = inbox.frames.pop_front() {
                let robot = &mut robots[i];
                match (ch, &mut bodies[i]) {
                    (1, _) => robot.plans.push((round, payload)),
                    (3, _) => {
                        commanded[i] = true;
                        robot.setpoints.push((round, payload));
                    }
                    (20, Body::Ground(g)) => {
                        g.setpoint = Some(payload.clone());
                        robot.setpoints.push((round, payload));
                    }
                    (16, Body::Air(p)) => {
                        p.setpoint = Some(payload.clone());
                        robot.controller.push(payload);
                    }
                    (17, Body::Air(p)) => p.fcu_request(&payload),
                    (18, _) => {
                        let s = cstr(&payload[8..]);
                        if robot.states.last().map_or(true, |l| l.1 != s) {
                            robot.states.push((ms, s));
                        }
                    }
                    _ => {}
                }
            }
        }
        for (body, robot) in bodies.iter_mut().zip(robots.iter_mut()) {
            match body {
                Body::Air(p) => p.step(1e-3),
                Body::Ground(g) => g.step(1e-3),
            }
            robot.truth.push(body.position());
        }
    }
    stop.store(true, Ordering::Relaxed);
    for (i, runner) in runners.into_iter().enumerate() {
        let summary = runner.join().unwrap();
        for module in &summary.plugins {
            println!("{name} r{}: {} {} domain={} steps={} consumed={} published={}", i + 1, module.name, module.state, module.domain_state, module.steps, module.consumed, module.published);
            assert!(module.last_error.is_none(), "{module:?}");
        }
    }
    for (inbox, audit) in inboxes.iter().zip(audits) {
        inbox.feeder.close();
        audit.finish().unwrap();
    }
    robots
}

/// plan-dmpc alone, fed strictly in order: a robot's measured states, the
/// scene, and before tick k the peer plans in `peer_plans[k]` (sent with
/// round k - 1). Returns its plans and setpoints (position targets or
/// planar).
fn replay_planner(name: &str, plan_dmpc: &Path, manifest: &Path, own: &[(u64, Vec<u8>)], scene: &[(u64, u32, Vec<u8>)], ticks: &[Vec<u8>], peer_plans: &BTreeMap<u64, Vec<Vec<u8>>>) -> (Vec<(u64, Vec<u8>)>, Vec<(u64, Vec<u8>)>) {
    let dir = common::scratch(name);
    let text = format!(
        r#"
[session]
id = "replay"
node = "r"
roster = ["r", "feeder"]
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
bind = {{ formation_tick = {{ channel = "formation_tick", from = ["feeder"] }}, plan_in = {{ channel = "plan", from = ["feeder"] }}, own_state = {{ channel = "own_state", from = ["feeder"] }}, scene_snapshot = {{ channel = "scene_snapshot", from = ["feeder"] }}, scene_state = {{ channel = "scene_state", from = ["feeder"] }}, plan_out = {{ channel = "replayed_plan" }}, setpoint = {{ channel = "alg_setpoint" }}, planar_setpoint = {{ channel = "planar_setpoint" }}, round_done = {{ channel = "round_done" }} }}
"#,
        channels = channel_toml() + "[[channel]]\nname = \"replayed_plan\"\nqos = \"control\"\n",
        plan_dmpc = plan_dmpc.display(),
        manifest = manifest.display(),
    );
    let bus = LoopbackBus::new();
    let clock = Arc::new(ManualClock::new(T0_MS * 1_000_000));
    let host = Host::new(Manifest::from_toml_str(&text).unwrap(), &dir, Box::new(LoopbackTransport::new(bus.clone())), clock.clone(), HostOptions::default()).unwrap();
    let replayed = CHANNELS.len() as u32;
    let mut specs = channel_specs();
    specs.push(ChannelSpec { id: replayed, name: "replayed_plan".into(), qos: Qos::Control });
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
            NodeMeta {
                format: String::new(), session: "replay".into(), node: "feeder".into(), node_id: 1,
                roster: vec!["r".into(), "feeder".into()], channels: specs.iter().map(|c| c.name.clone()).collect(), clock_domain: "sim".into(),
                audit_queue_drops: 0, records_written: 0, complete: false,
            },
            clock.clone(),
        )
        .unwrap(),
    );
    let ctx = TransportContext { session: "replay".into(), node: "feeder".into(), node_id: 1, roster: vec!["r".into(), "feeder".into()], channels: specs };
    let feeder = Endpoint::open(Box::new(LoopbackTransport::new(bus.clone())), &ctx, clock.clone(), audit.clone(), 1 << 20).unwrap();
    for ch in [0, 1, 2, 4, 5] {
        feeder.declare_out(ch).unwrap();
    }
    for ch in [3, 6, 20, replayed] {
        feeder.declare_in(ch, &[0]).unwrap();
    }
    let stop = Arc::new(AtomicBool::new(false));
    let runner = spawn(host, &stop);
    std::thread::sleep(Duration::from_millis(300));
    let mut inbox = Inbox { feeder, frames: VecDeque::new() };
    for k in 0..ROUNDS as u64 {
        let t_ns = (T0_MS + k as i64 * PERIOD_MS) * 1_000_000;
        clock.set(t_ns);
        let f = &inbox.feeder;
        for payload in peer_plans.get(&k).into_iter().flatten() {
            f.publish(1, k.saturating_sub(1), t_ns, payload).unwrap();
        }
        for (round, payload) in own.iter().filter(|o| o.0 == k) {
            f.publish(2, *round, t_ns, payload).unwrap();
        }
        for (round, port, payload) in scene.iter().filter(|r| r.0 == k) {
            f.publish(if *port == 5 { 4 } else { 5 }, *round, t_ns, payload).unwrap();
        }
        f.publish(0, k, t_ns, &ticks[k as usize]).unwrap();
        inbox.until("round_done", |fr| fr.2 == 6 && fr.1 == k);
    }
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    assert!(summary.plugins[0].last_error.is_none(), "{:?}", summary.plugins[0]);
    inbox.drain();
    inbox.feeder.close();
    audit.finish().unwrap();
    let (mut plans, mut setpoints) = (Vec::new(), Vec::new());
    for (_, round, ch, payload) in inbox.frames {
        if ch == replayed {
            plans.push((round, payload));
        } else if ch == 3 || ch == 20 {
            setpoints.push((round, payload));
        }
    }
    (plans, setpoints)
}

struct Setup {
    plan_dmpc: PathBuf,
    ctl_px4: PathBuf,
    fg_root: PathBuf,
    tool: PathBuf,
}

fn setup() -> Option<Setup> {
    let (Some(lib_dir), Some(fg_root), Some(_px4)) =
        (env_path("DMPC_LIB_DIR"), env_path("FORMATION_GENERATOR_ROOT"), env_path("PX4_CORE_LIB_DIR"))
    else {
        eprintln!("skipped: set DMPC_LIB_DIR, FORMATION_GENERATOR_ROOT and PX4_CORE_LIB_DIR");
        return None;
    };
    let out = common::workspace_root().join("target/plugin-tests/cpp");
    std::fs::create_dir_all(&out).unwrap();
    let plan_dmpc = out.join("libplan_dmpc_fleet.so");
    let ctl_px4 = out.join("libctl_px4_fleet.so");
    static BUILT: std::sync::Once = std::sync::Once::new();
    BUILT.call_once(|| {
        assert!(Command::new(common::workspace_root().join("scripts/build-plan-dmpc.sh")).arg(&plan_dmpc).status().unwrap().success());
        assert!(Command::new(common::workspace_root().join("scripts/build-ctl-px4.sh")).arg(&ctl_px4).status().unwrap().success());
    });
    let tool = env_path("DMPC_FLEET_REPLAY").unwrap_or_else(|| lib_dir.join("dmpc_fleet_replay"));
    Some(Setup { plan_dmpc, ctl_px4, fg_root, tool })
}

/// Fly `members` twice and check the gate. `fleet_options` are the fleet
/// replay's options for the scene (a path relative to the package).
fn fly_and_check(name: &str, setup: &Setup, members: &[Member], fleet_options: &[&str], late_control: bool) {
    // The scene and the formation ticks, as the fleet replay plays them.
    let dir = common::scratch(&format!("{name}-scene"));
    let mut cmd = Command::new(&setup.tool);
    cmd.args(["--record", "1", "--rounds", &ROUNDS.to_string(), "--hold", &HOLD.to_string()])
        .arg("--inputs").arg(dir.join("inputs.bin"))
        .arg("--outputs").arg(dir.join("outputs.bin"));
    for option in fleet_options {
        if option.starts_with("config/") {
            cmd.arg(setup.fg_root.join(option));
        } else {
            cmd.arg(option);
        }
    }
    let status = cmd.args(members.iter().map(|m| &m.manifest)).status().unwrap();
    assert!(status.success(), "dmpc_fleet_replay failed");
    let recorded = read_records(&dir.join("inputs.bin"));
    let scene: Vec<(u64, u32, Vec<u8>)> = recorded.iter().filter(|r| r.1 == 5 || r.1 == 6).cloned().collect();
    let ticks: Vec<Vec<u8>> = recorded.iter().filter(|r| r.1 == 0).map(|r| r.2.clone()).collect();
    assert_eq!(ticks.len(), ROUNDS as usize);

    let a = fly(&format!("{name}-a"), &setup.plan_dmpc, &setup.ctl_px4, members, &scene, &ticks);
    let b = fly(&format!("{name}-b"), &setup.plan_dmpc, &setup.ctl_px4, members, &scene, &ticks);

    for (i, (ra, rb)) in a.iter().zip(&b).enumerate() {
        let (r, kind) = (i + 1, members[i].kind);
        assert!(ra == rb, "{name} r{r}: the two flights differ");
        if kind == Kind::Uav {
            println!("{name} r{r} states: {:?}", ra.states);
            let custom1 = ra.states.iter().find(|s| s.1 == "Custom1").map(|s| s.0).expect("never entered Custom1");
            assert!(custom1 < T0_MS + HOLD as i64 * PERIOD_MS, "{name} r{r}: Custom1 at {custom1} ms, after rolling began");
            assert_eq!(ra.states.last().unwrap().1, "Custom1", "{name} r{r}: Custom1 not held");
        }
        // Tracking (a Scout in the plane) and travel.
        let mut errors = Vec::new();
        let mut first = None;
        for (round, d) in &ra.setpoints {
            if *round < HOLD {
                continue;
            }
            let ms = (f64_at(d, 0) * 1e3).round() as i64;
            let Some(p) = ra.truth.get((ms - START_MS) as usize) else { continue };
            let sp = [f64_at(d, 1), f64_at(d, 2), if kind == Kind::Uav { f64_at(d, 3) } else { p[2] }];
            first.get_or_insert(sp);
            errors.push(((p[0] - sp[0]).powi(2) + (p[1] - sp[1]).powi(2) + (p[2] - sp[2]).powi(2)).sqrt());
        }
        errors.sort_by(|x, y| x.partial_cmp(y).unwrap());
        let (p50, max) = (errors[errors.len() / 2], errors[errors.len() - 1]);
        let (p0, p1) = (first.unwrap(), ra.truth.last().unwrap());
        let travel = ((p1[0] - p0[0]).powi(2) + (p1[1] - p0[1]).powi(2)).sqrt();
        println!(
            "{name} r{r} ({kind:?}): {} rolling setpoints, tracking p50 {p50:.3} m max {max:.3} m, travel {travel:.2} m, {} plans",
            errors.len(),
            ra.plans.len()
        );
        assert!(errors.len() as i64 >= ROUNDS - HOLD as i64 - 1, "{name} r{r}: {} rolling setpoints", errors.len());
        assert!(p50 < 0.15 && max < 0.5, "{name} r{r}: tracking p50 {p50:.3} m max {max:.3} m");
        assert!(travel > 1.0, "{name} r{r}: travelled {travel:.2} m");
    }

    // Peers' plans of round k - 1: each robot's closed-loop rounds equal a
    // strictly ordered offline run (a peer's round-r plan before tick
    // r + delay).
    let peers_of = |i: usize, delay: u64| {
        let mut plans: BTreeMap<u64, Vec<Vec<u8>>> = BTreeMap::new();
        for (j, peer) in a.iter().enumerate() {
            if j != i {
                for (round, payload) in &peer.plans {
                    plans.entry(round + delay).or_default().push(payload.clone());
                }
            }
        }
        plans
    };
    for i in 0..members.len() {
        let (plans, setpoints) =
            replay_planner(&format!("{name}-replay-r{}", i + 1), &setup.plan_dmpc, &members[i].manifest, &a[i].own_states, &scene, &ticks, &peers_of(i, 1));
        assert_eq!(plans.len(), a[i].plans.len(), "{name} r{}: plan count", i + 1);
        assert!(plans == a[i].plans, "{name} r{}: closed-loop plans differ from the in-order replay", i + 1);
        assert!(setpoints == a[i].setpoints, "{name} r{}: closed-loop setpoints differ from the in-order replay", i + 1);
    }
    if late_control {
        // The check sees a late plan: every peer plan one round later.
        let (late, _) = replay_planner(&format!("{name}-replay-late"), &setup.plan_dmpc, &members[0].manifest, &a[0].own_states, &scene, &ticks, &peers_of(0, 2));
        assert!(late != a[0].plans, "{name}: a one-round-late neighbor plan went unnoticed");
    }
    println!("{name}: {} robots, every round reproduced by the in-order replay", members.len());
}

#[test]
fn the_knot_fs150_fleet_flies_in_closed_loop_with_plans_over_the_link() {
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let Some(setup) = setup() else { return };
    // Each robot on the ground under its knot_fs150 seed point (the pentagon
    // of radius 1.2 m around the leader's start).
    let members: Vec<Member> = (0..5)
        .map(|i| {
            let a = 2.0 * std::f64::consts::PI * i as f64 / 5.0;
            Member {
                kind: Kind::Uav,
                manifest: setup.fg_root.join(format!("test/replay/plan_dmpc/knot_fs150_full_uav{}.yaml", i + 1)),
                spawn: [1.2 * a.cos(), 1.2 * a.sin(), 0.0],
                yaw: 0.0,
            }
        })
        .collect();
    fly_and_check("fleet-knot", &setup, &members, &["--scene", "config/scenarios/knot_fs150/scene.yaml"], true);
}

#[test]
fn the_mixed_circle_fleet_flies_in_closed_loop_with_plans_over_the_link() {
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let Some(setup) = setup() else { return };
    // Spawn poses of mixed_circle/swarm_pose.yaml: agents 1-5 UAVs, 6-9 Scouts.
    let spawns: [[f64; 3]; 9] = [
        [-6.0, 1.2, 0.0], [-6.0, -1.2, 0.0], [-3.0, 0.0, 0.0], [0.0, 1.2, 0.0], [0.0, -1.2, 0.0],
        [-2.1, 0.9, 0.181], [-3.9, 0.9, 0.181], [-3.9, -0.9, 0.181], [-2.1, -0.9, 0.181],
    ];
    let members: Vec<Member> = spawns
        .iter()
        .enumerate()
        .map(|(i, s)| Member {
            kind: if i < 5 { Kind::Uav } else { Kind::Scout },
            manifest: setup.fg_root.join(format!("test/replay/plan_dmpc/mixed_circle_agent{}.yaml", i + 1)),
            spawn: *s,
            yaw: std::f64::consts::FRAC_PI_2,
        })
        .collect();
    fly_and_check(
        "fleet-mixed",
        &setup,
        &members,
        &["--scene", "config/scenarios/mixed_circle/scene.yaml", "--spawn", "config/scenarios/mixed_circle/swarm_pose.yaml"],
        false,
    );
}
