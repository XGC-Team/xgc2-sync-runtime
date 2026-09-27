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
//! Over Zenoh (Block I): the same fleet with every node, robots and feeders,
//! its own Zenoh session from the Zenoh transport plugin (the robot's
//! manifest names it with `[transport] path`). Robot i listens on UDP for
//! the other robots and on TCP for its feeder; for every pair i < j, robot i
//! connects to j through its own xgc-rt-impair relay, which impairs i -> j
//! and passes j -> i through (as the C4 matrix, zenoh_matrix.rs). Only plans
//! cross between robots (and a few control envelopes Zenoh routes before
//! declarations settle, which no receiver delivers). The lockstep is the
//! same, except that from t0 each 100 ms round lasts at least 100 ms of wall
//! time, so a plan's relay delay is real time within the round, and the
//! robots tick one after another, the last first, each after the previous
//! one's round_done: a higher robot's plan of round k reaches a lower one
//! (pass-through direction) before that robot's tick k. The flight is not deterministic any more (what
//! arrives before a tick depends on the network), so the gate is:
//! - the audit against the relay truth, plan by plan: every plan i -> j the
//!   relay dropped is lost, none it dropped is received, every other loss is
//!   a datagram Zenoh discarded on receipt (a held, reordered copy that came
//!   after a later plan), nothing was received twice or out of order, the
//!   pass-through direction and robot -> feeder lose nothing, and no plan
//!   vanished before the relay;
//! - round selection, by replay: each robot's closed-loop plans and
//!   setpoints equal an offline plan-dmpc run fed, before tick k, exactly
//!   the samples the closed-loop module read up to its tick k (its host's
//!   steps.jsonl), except that peer plans of round >= k that had already
//!   arrived are held back until after tick k. Equal outputs mean those
//!   early plans did not enter round k: peers' plans of round <= k - 1;
//! - Fresh / Stale / Missing per robot, round and peer (the newest peer plan
//!   of round <= k - 1 the module had at tick k: of round k - 1, older,
//!   none), with the peer rounds that were late or never arrived;
//! - the H product gate: Custom1 before the rolling rounds, held; every
//!   robot tracks its rolling setpoints and travels.
//! Relay profiles: clean (pass-through) and C4-like (40 ms, Gilbert-Elliott
//! burst loss, 2 % duplicate, 2 % reorder; a seed per relay).
//!
//! Needs DMPC_LIB_DIR, FORMATION_GENERATOR_ROOT and PX4_CORE_LIB_DIR (as
//! closed_loop_dmpc_px4.rs). Without them the tests print why and pass.

mod common;

use common::px4_plant::{command, cstr, f64_at, f64s, read_records, GroundPlant, Plant};

use std::collections::{BTreeMap, HashMap, HashSet, VecDeque};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use xgc_rt_audit::record::{Kind as RecordKind, Record, RECORD_LEN};
use xgc_rt_audit::{merge_run, FileAudit, MergeOptions, NodeMeta};
use xgc_rt_core::clock::ManualClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, Transport, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::transport_so::SoTransport;
use xgc_rt_host::{Host, HostOptions, RunSummary};
use xgc_rt_impair::{Action, GilbertElliott, Profile, Relay, TruthRecord};
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

/// How the fleet's nodes are linked.
enum Net {
    /// One in-process loopback bus.
    Loopback(Arc<LoopbackBus>),
    /// Every node its own session of the Zenoh transport plugin; robot i
    /// (index i - 1) listens on `udp` (robots) and `tcp` (its feeder), and
    /// connects to each robot j > i through `relays[(i - 1, j - 1)]`.
    Zenoh { plugin: PathBuf, udp: Vec<u16>, tcp: Vec<u16>, relays: HashMap<(usize, usize), Relay> },
}

impl Net {
    fn zenoh(n: usize, profile: &dyn Fn(usize, usize) -> Profile) -> Net {
        // Listen ports below the ephemeral range (32768-60999), so no
        // outgoing connection takes one between here and the listen.
        static NEXT: std::sync::atomic::AtomicU16 = std::sync::atomic::AtomicU16::new(0);
        let free = || loop {
            let base = 20_000 + (std::process::id() % 1_000) as u16 * 10;
            let port = base + NEXT.fetch_add(1, Ordering::Relaxed) % 2_000;
            if std::net::TcpListener::bind(("127.0.0.1", port)).is_ok() && std::net::UdpSocket::bind(("127.0.0.1", port)).is_ok() {
                return port;
            }
        };
        let udp: Vec<u16> = (0..n).map(|_| free()).collect();
        let tcp: Vec<u16> = (0..n).map(|_| free()).collect();
        let mut relays = HashMap::new();
        for i in 0..n {
            for j in i + 1..n {
                let target = format!("127.0.0.1:{}", udp[j]).parse().unwrap();
                relays.insert((i, j), Relay::start("127.0.0.1:0".parse().unwrap(), target, profile(i, j)).unwrap());
            }
        }
        Net::Zenoh { plugin: common::transport_plugin("zenoh"), udp, tcp, relays }
    }

    /// Robot i's `[transport]` table.
    fn robot_transport(&self, n: usize, i: usize) -> String {
        match self {
            Net::Loopback(_) => "kind = \"loopback\"".into(),
            Net::Zenoh { plugin, udp, tcp, relays } => {
                let connect: Vec<String> = (i..n).map(|j| format!("\"udp/{}\"", relays[&(i - 1, j)].listen)).collect();
                format!(
                    "kind = \"zenoh\"\npath = \"{}\"\nlisten = [\"udp/127.0.0.1:{}\", \"tcp/127.0.0.1:{}\"]\nconnect = [{}]",
                    plugin.display(),
                    udp[i - 1],
                    tcp[i - 1],
                    connect.join(", ")
                )
            }
        }
    }

    /// The transport a robot's manifest names.
    fn robot_link(&self, manifest: &Manifest) -> Box<dyn Transport> {
        match self {
            Net::Loopback(bus) => Box::new(LoopbackTransport::new(bus.clone())),
            Net::Zenoh { .. } => {
                let t = &manifest.transport;
                Box::new(SoTransport::load(t.path.as_ref().unwrap(), None, &t.kind, &t.options).unwrap())
            }
        }
    }

    fn feeder_link(&self, i: usize) -> Box<dyn Transport> {
        match self {
            Net::Loopback(bus) => Box::new(LoopbackTransport::new(bus.clone())),
            Net::Zenoh { tcp, .. } => common::so_transport("zenoh", &format!("connect = [\"tcp/127.0.0.1:{}\"]\n", tcp[i - 1])),
        }
    }

    /// Stop the relays: their truth by (sender, receiver) index.
    fn truth(self) -> HashMap<(usize, usize), Vec<TruthRecord>> {
        match self {
            Net::Loopback(_) => HashMap::new(),
            Net::Zenoh { relays, .. } => relays.into_iter().map(|(k, r)| (k, r.stop())).collect(),
        }
    }
}

type Frame = (u16, u64, u32, Vec<u8>, u64); // origin, round, channel, payload, seq

struct Inbox {
    feeder: Arc<Endpoint>,
    frames: VecDeque<Frame>,
}

impl Inbox {
    fn drain(&mut self) {
        self.frames.extend(self.feeder.drain().into_iter().map(|f| (f.header.origin, f.header.round, f.header.channel, f.payload, f.header.seq)));
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

fn feeder_endpoint(net: &Net, clock: &Arc<ManualClock>, dir: &Path, n: usize, i: usize, kind: Kind) -> (Arc<Endpoint>, Arc<FileAudit>) {
    let node = format!("feeder{i}");
    let node_id = (n + i - 1) as u16;
    let audit = Arc::new(
        FileAudit::create(
            &dir.join("audit"),
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
    let ep = Endpoint::open(net.feeder_link(i), &ctx, clock.clone(), audit.clone(), 1 << 20).unwrap();
    // A Scout has no controller: its feeder sends the planner's inputs only.
    for ch in FEEDER_OUT.into_iter().filter(|&ch| kind == Kind::Uav || ch <= 5) {
        ep.declare_out(ch).unwrap();
    }
    for ch in FEEDER_IN {
        ep.declare_in(ch, &[(i - 1) as u16]).unwrap();
    }
    (ep, audit)
}

#[allow(clippy::too_many_arguments)]
fn robot_host(net: &Net, clock: &Arc<ManualClock>, dir: &Path, n: usize, i: usize, member: &Member, plan_dmpc: &Path, ctl_px4: &Path) -> Host {
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
{transport}

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
{controller}"#,
        transport = net.robot_transport(n, i),
        channels = channel_toml(),
        plan_dmpc = plan_dmpc.display(),
        manifest = member.manifest.display(),
        planner_binds = planner_binds.join(", "),
    );
    let manifest = Manifest::from_toml_str(&text).unwrap();
    let link = net.robot_link(&manifest);
    Host::new(manifest, dir, link, clock.clone(), HostOptions::default()).unwrap()
}

#[derive(Default, PartialEq)]
struct Robot {
    plans: Vec<(u64, Vec<u8>)>,      // own plans (round, payload) as published on `plan`
    setpoints: Vec<(u64, Vec<u8>)>,  // planner setpoints: position targets or planar (round, payload)
    controller: Vec<Vec<u8>>,        // ctl-px4 setpoints
    states: Vec<(i64, String)>,      // ctl-px4 control states
    truth: Vec<[f64; 3]>,            // plant position per ms
    own_states: Vec<(u64, Vec<u8>)>, // measured states given to plan-dmpc
    // By seq: this robot's plans (round, payload) as its feeder received
    // them, and what the feeder sent plan-dmpc on formation_tick, own_state,
    // scene_snapshot, scene_state ((channel, seq) -> (round, payload)).
    plan_by_seq: BTreeMap<u64, (u64, Vec<u8>)>,
    fed: BTreeMap<(u32, u64), (u64, Vec<u8>)>,
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

struct Flight {
    robots: Vec<Robot>,
    /// The run's audit directory (a node per subdirectory).
    audit: PathBuf,
    /// The relays' truth by (sender, receiver) index; empty on loopback.
    truth: HashMap<(usize, usize), Vec<TruthRecord>>,
    /// Wall time of each round from t0, ms.
    round_wall_ms: Vec<f64>,
}

/// One fleet flight. `scene` is the recorded scene inputs (round, recorded
/// port 5 | 6, payload) and `ticks` the formation ticks per round. `pace`:
/// from t0 each round takes at least PERIOD_MS of wall time.
#[allow(clippy::too_many_arguments)]
fn fly(name: &str, plan_dmpc: &Path, ctl_px4: &Path, members: &[Member], scene: &[(u64, u32, Vec<u8>)], ticks: &[Vec<u8>], net: Net, pace: bool) -> Flight {
    let n = members.len();
    let dir = common::scratch(name);
    let zenoh = matches!(net, Net::Zenoh { .. });
    let clock = Arc::new(ManualClock::new(START_MS * 1_000_000));
    let hosts: Vec<Host> = members.iter().enumerate().map(|(k, m)| robot_host(&net, &clock, &dir, n, k + 1, m, plan_dmpc, ctl_px4)).collect();
    let (endpoints, audits): (Vec<Arc<Endpoint>>, Vec<Arc<FileAudit>>) = (1..=n).map(|i| feeder_endpoint(&net, &clock, &dir, n, i, members[i - 1].kind)).unzip();
    let stop = Arc::new(AtomicBool::new(false));
    let runners: Vec<JoinHandle<RunSummary>> = hosts.into_iter().map(|h| spawn(h, &stop)).collect();
    if zenoh {
        let up = Instant::now();
        for (i, ep) in endpoints.iter().enumerate() {
            assert!(ep.wait_ready(Duration::from_secs(30)), "{name}: feeder{} not matched by r{}", i + 1, i + 1);
        }
        println!("{name}: feeders matched after {:.2} s", up.elapsed().as_secs_f64());
    }
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
    let mut wall_t0: Option<Instant> = None;
    let mut round_wall_ms = Vec::new();
    let mut round_began = Instant::now();
    for ms in START_MS..end_ms {
        let t_ns = ms * 1_000_000;
        let t = ms as f64 * 1e-3;
        clock.set(t_ns);
        if ms >= T0_MS && (ms - T0_MS) % PERIOD_MS == 0 {
            let k = ((ms - T0_MS) / PERIOD_MS) as u64;
            if pace {
                let w0 = *wall_t0.get_or_insert_with(Instant::now);
                if let Some(wait) = (w0 + Duration::from_millis(k * PERIOD_MS as u64)).checked_duration_since(Instant::now()) {
                    std::thread::sleep(wait);
                }
            }
            if k > 0 {
                round_wall_ms.push(round_began.elapsed().as_secs_f64() * 1e3);
            }
            round_began = Instant::now();
            // Over Zenoh the robots tick one after another, the last first,
            // each once the one before it finished its round and 3 ms more:
            // a robot's plan of round k can then reach a lower-numbered
            // peer (the relays' pass-through direction) before that peer's
            // tick k, which must not use it.
            let order: Vec<usize> = if zenoh { (0..n).rev().collect() } else { (0..n).collect() };
            for &i in &order {
                let inbox = &inboxes[i];
                let state = bodies[i].rigid_state(t);
                let h = inbox.feeder.publish(2, k, t_ns, &state).unwrap();
                robots[i].fed.insert((2, h.seq), (k, state.clone()));
                robots[i].own_states.push((k, state));
                for (round, port, payload) in scene.iter().filter(|r| r.0 == k) {
                    let channel = if *port == 5 { 4 } else { 5 };
                    let h = inbox.feeder.publish(channel, *round, t_ns, payload).unwrap();
                    robots[i].fed.insert((channel, h.seq), (*round, payload.clone()));
                }
                let h = inbox.feeder.publish(0, k, t_ns, &ticks[k as usize]).unwrap();
                robots[i].fed.insert((0, h.seq), (k, ticks[k as usize].clone()));
                if zenoh {
                    inboxes[i].until("round_done", |f| f.2 == 6 && f.1 == k);
                    std::thread::sleep(Duration::from_millis(3));
                }
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
            while let Some((_, round, ch, payload, seq)) = inbox.frames.pop_front() {
                let robot = &mut robots[i];
                match (ch, &mut bodies[i]) {
                    (1, _) => {
                        robot.plan_by_seq.insert(seq, (round, payload.clone()));
                        robot.plans.push((round, payload));
                    }
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
    round_wall_ms.push(round_began.elapsed().as_secs_f64() * 1e3);
    if zenoh {
        // The last round's plans are still crossing the relays.
        std::thread::sleep(Duration::from_millis(1_000));
    }
    stop.store(true, Ordering::Relaxed);
    for (i, runner) in runners.into_iter().enumerate() {
        let summary = runner.join().unwrap();
        for module in &summary.plugins {
            println!("{name} r{}: {} {} domain={} steps={} consumed={} published={}", i + 1, module.name, module.state, module.domain_state, module.steps, module.consumed, module.published);
            assert!(module.last_error.is_none(), "{module:?}");
        }
    }
    for (i, (inbox, audit)) in inboxes.iter_mut().zip(audits).enumerate() {
        inbox.drain();
        for (_, round, ch, payload, seq) in inbox.frames.drain(..) {
            println!("{name} feeder{}: after the last step, channel {ch} round {round} seq {seq}", i + 1);
            match ch {
                1 => {
                    robots[i].plan_by_seq.insert(seq, (round, payload));
                }
                3 | 20 => robots[i].setpoints.push((round, payload)),
                _ => {}
            }
        }
        inbox.feeder.close();
        audit.finish().unwrap();
    }
    Flight { robots, audit: dir.join("audit"), truth: net.truth(), round_wall_ms }
}

/// plan-dmpc alone, fed strictly in order: a robot's measured states, the
/// scene, and before tick k the peer plans in `peer_plans[k]` (sent with
/// round k - 1). Returns its plans and setpoints (position targets or
/// planar).
fn replay_planner(name: &str, plan_dmpc: &Path, manifest: &Path, own: &[(u64, Vec<u8>)], scene: &[(u64, u32, Vec<u8>)], ticks: &[Vec<u8>], peer_plans: &BTreeMap<u64, Vec<Vec<u8>>>) -> (Vec<(u64, Vec<u8>)>, Vec<(u64, Vec<u8>)>) {
    let batches: Vec<Vec<Input>> = (0..ROUNDS as u64)
        .map(|k| {
            let mut batch: Vec<Input> = peer_plans.get(&k).into_iter().flatten().map(|p| (1, k.saturating_sub(1), p.clone())).collect();
            batch.extend(own.iter().filter(|o| o.0 == k).map(|(round, p)| (2, *round, p.clone())));
            batch.extend(scene.iter().filter(|r| r.0 == k).map(|(round, port, p)| (if *port == 5 { 4 } else { 5 }, *round, p.clone())));
            batch.push((0, k, ticks[k as usize].clone()));
            batch
        })
        .collect();
    replay_batches(name, plan_dmpc, manifest, &batches)
}

/// A planner input: channel (as CHANNELS), envelope round, payload.
type Input = (u32, u64, Vec<u8>);

/// plan-dmpc alone, fed batch by batch in order, each batch ending with a
/// formation tick; the next batch goes after that tick's round_done.
fn replay_batches(name: &str, plan_dmpc: &Path, manifest: &Path, batches: &[Vec<Input>]) -> (Vec<(u64, Vec<u8>)>, Vec<(u64, Vec<u8>)>) {
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
    for batch in batches {
        let &(0, k, _) = batch.last().unwrap() else { panic!("a replay batch ends with its formation tick") };
        let t_ns = (T0_MS + k as i64 * PERIOD_MS) * 1_000_000;
        clock.set(t_ns);
        for (channel, round, payload) in batch {
            inbox.feeder.publish(*channel, *round, t_ns, payload).unwrap();
        }
        inbox.until("round_done", |fr| fr.2 == 6 && fr.1 == k);
    }
    stop.store(true, Ordering::Relaxed);
    let summary = runner.join().unwrap();
    assert!(summary.plugins[0].last_error.is_none(), "{:?}", summary.plugins[0]);
    inbox.drain();
    inbox.feeder.close();
    audit.finish().unwrap();
    let (mut plans, mut setpoints) = (Vec::new(), Vec::new());
    for (_, round, ch, payload, _) in inbox.frames {
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

/// The scene and the formation ticks, as the fleet replay plays them.
/// `fleet_options` are the fleet replay's options for the scene (a path
/// relative to the package).
fn record_scene(name: &str, setup: &Setup, members: &[Member], fleet_options: &[&str]) -> (Vec<(u64, u32, Vec<u8>)>, Vec<Vec<u8>>) {
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
    (scene, ticks)
}

/// The product gate: every UAV in Custom1 before the rolling rounds, held;
/// every robot tracks its planner's rolling setpoints (a Scout in the
/// plane) and travels. `infeasible[i]`: rolling rounds robot i's planner
/// logged as failed (no plan, no setpoint; the controller holds the last
/// setpoint), which the setpoint count allows for.
fn check_product(name: &str, members: &[Member], robots: &[Robot], infeasible: &[usize]) {
    for (i, ra) in robots.iter().enumerate() {
        let (r, kind) = (i + 1, members[i].kind);
        if kind == Kind::Uav {
            println!("{name} r{r} states: {:?}", ra.states);
            let custom1 = ra.states.iter().find(|s| s.1 == "Custom1").map(|s| s.0).expect("never entered Custom1");
            assert!(custom1 < T0_MS + HOLD as i64 * PERIOD_MS, "{name} r{r}: Custom1 at {custom1} ms, after rolling began");
            assert_eq!(ra.states.last().unwrap().1, "Custom1", "{name} r{r}: Custom1 not held");
        }
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
        let rounds: Vec<u64> = ra.setpoints.iter().map(|s| s.0).collect();
        let missing: Vec<u64> = (0..ROUNDS as u64).filter(|k| !rounds.contains(k)).collect();
        assert!(
            errors.len() as i64 >= ROUNDS - HOLD as i64 - 1 - infeasible[i] as i64,
            "{name} r{r}: {} rolling setpoints ({} infeasible rounds); setpoint rounds missing {missing:?}",
            errors.len(),
            infeasible[i]
        );
        assert!(p50 < 0.15 && max < 0.5, "{name} r{r}: tracking p50 {p50:.3} m max {max:.3} m");
        assert!(travel > 1.0, "{name} r{r}: travelled {travel:.2} m");
    }
}

/// Fly `members` twice on the loopback bus and check the gate.
fn fly_and_check(name: &str, setup: &Setup, members: &[Member], fleet_options: &[&str], late_control: bool) {
    let (scene, ticks) = record_scene(name, setup, members, fleet_options);
    let a = fly(&format!("{name}-a"), &setup.plan_dmpc, &setup.ctl_px4, members, &scene, &ticks, Net::Loopback(LoopbackBus::new()), false).robots;
    let b = fly(&format!("{name}-b"), &setup.plan_dmpc, &setup.ctl_px4, members, &scene, &ticks, Net::Loopback(LoopbackBus::new()), false).robots;
    for (i, (ra, rb)) in a.iter().zip(&b).enumerate() {
        assert!(ra == rb, "{name} r{}: the two flights differ", i + 1);
    }
    check_product(name, members, &a, &vec![0; members.len()]);

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

// plan-dmpc's input ports (plugins/plan-dmpc kPorts), as steps.jsonl names them.
const PORT_TICK: u32 = 0;
const PORT_PLAN_IN: u32 = 1;
const PORT_OWN_STATE: u32 = 3;
const PORT_SCENE_SNAPSHOT: u32 = 5;
const PORT_SCENE_STATE: u32 = 6;

/// The C4 relay profile: 40 ms, Gilbert-Elliott burst loss, 2 % duplicate,
/// 2 % reorder; a seed per relay.
fn c4_profile(n: usize) -> impl Fn(usize, usize) -> Profile {
    move |i, j| Profile {
        delay_ms: 40.0,
        jitter_ms: 0.0,
        loss: 0.0,
        gilbert_elliott: Some(GilbertElliott { p_good_bad: 0.01, p_bad_good: 0.3, loss_good: 0.01, loss_bad: 0.5 }),
        duplicate: 0.02,
        reorder: 0.02,
        seed: 0xB10C_0000 + (i * n + j) as u64,
    }
}

/// `robot`'s received link samples: (origin, seq) on `channel`.
fn received(audit: &Path, robot: &str, channel: u32) -> HashSet<(u16, u64)> {
    let bytes = std::fs::read(audit.join(robot).join("records.bin")).unwrap();
    bytes
        .chunks_exact(RECORD_LEN)
        .map(|c| Record::decode(c).unwrap())
        .filter(|r| r.kind == RecordKind::Rx && r.channel == channel)
        .map(|r| (r.origin, r.seq))
        .collect()
}

#[derive(Default, Debug)]
struct PlanAudit {
    sent: u64,
    lost: u64,
    drop: u64,
    discard: u64,
    held_discarded: u64,
    relay_duplicates: u64,
    relay_held: u64,
    lost_clean: u64,
}

/// Plan by plan, each robot's audit against the relays' truth.
fn plan_audit(name: &str, n: usize, flight: &Flight) -> PlanAudit {
    let report = merge_run(&flight.audit, MergeOptions::default()).unwrap();
    xgc_rt_audit::write_report(&report, &flight.audit.join("merged")).unwrap();
    assert!(report.valid, "{name}: {:?}", report.invalid_reasons);
    let mut crossing: BTreeMap<u32, usize> = BTreeMap::new();
    for t in flight.truth.values().flatten() {
        *crossing.entry(t.channel).or_default() += 1;
    }
    println!("{name}: envelopes through the relays by channel id: {crossing:?}");
    let got: Vec<HashSet<(u16, u64)>> = (1..=n).map(|j| received(&flight.audit, &format!("r{j}"), 1)).collect();
    let index = |node: &str| node.trim_start_matches(char::is_alphabetic).parse::<usize>().unwrap() - 1;
    let mut a = PlanAudit::default();
    let mut robot_streams = 0;
    for s in report.streams.iter().filter(|s| s.channel == "plan") {
        let what = format!("{name}: plan {} -> {}", s.origin, s.receiver);
        assert_eq!((s.counts.duplicates, s.counts.reordered), (0, 0), "{what}: best-effort Zenoh suppresses duplicates and discards late datagrams: {:?}", s.counts);
        let (i, j) = (index(&s.origin), index(&s.receiver));
        if s.receiver.starts_with("feeder") {
            assert_eq!(s.counts.lost, 0, "{what}: {:?}", s.counts);
            continue;
        }
        robot_streams += 1;
        a.sent += s.counts.expected;
        a.lost += s.counts.lost;
        if i > j {
            // The relay's pass-through direction.
            assert_eq!(s.counts.lost, 0, "{what} (pass-through): {:?}", s.counts);
            a.lost_clean += s.counts.lost;
            continue;
        }
        let by_seq: HashMap<u64, &TruthRecord> = flight.truth[&(i, j)].iter().filter(|t| t.origin == i as u16 && t.channel == 1).map(|t| (t.seq, t)).collect();
        let (mut drop, mut discard) = (0, 0);
        for q in 1..=s.counts.expected {
            let arrived = got[j].contains(&(i as u16, q));
            let t = by_seq.get(&q).unwrap_or_else(|| panic!("{what}: seq {q} never passed the relay"));
            match (t.action, arrived) {
                (Action::Drop, true) => panic!("{what}: seq {q} dropped by the relay but received"),
                (Action::Drop, false) => drop += 1,
                (action, false) => {
                    discard += 1;
                    a.held_discarded += u64::from(action == Action::Reorder);
                }
                _ => {}
            }
            a.relay_duplicates += u64::from(t.action == Action::Duplicate);
            a.relay_held += u64::from(t.action == Action::Reorder);
        }
        assert_eq!(s.counts.lost, drop + discard, "{what}: every loss is a relay drop or a receive-side discard: {:?}", s.counts);
        a.drop += drop;
        a.discard += discard;
    }
    assert_eq!(robot_streams, n * (n - 1), "{name}: a plan stream per ordered robot pair");
    a
}

/// The rounds robot i's planner logged as failed: (time, message).
fn planner_failures(audit: &Path, i: usize) -> Vec<(i64, String)> {
    let text = std::fs::read_to_string(audit.join(format!("r{i}/health.jsonl"))).unwrap();
    text.lines()
        .map(|l| serde_json::from_str::<serde_json::Value>(l).unwrap())
        .filter(|v| v["event"] == "log" && v["plugin"] == "plan-dmpc" && v["level"] == "error")
        .map(|v| (v["t"].as_i64().unwrap(), v["message"].as_str().unwrap().to_string()))
        .filter(|(_, m)| m.contains("control cycle failed"))
        .collect()
}

/// What robot i's plan-dmpc read, per tick, from its host's step log:
/// (port, origin, seq), everything up to and including the tick's step.
fn tick_reads(audit: &Path, i: usize) -> Vec<Vec<(u32, u16, u64)>> {
    let text = std::fs::read_to_string(audit.join(format!("r{i}/steps.jsonl"))).unwrap();
    let (mut batches, mut pending) = (Vec::new(), Vec::new());
    for line in text.lines() {
        let v: serde_json::Value = serde_json::from_str(line).unwrap();
        if v["m"] != "plan-dmpc" {
            continue;
        }
        let reads: Vec<(u32, u16, u64)> = v["in"]
            .as_array()
            .unwrap()
            .iter()
            .map(|r| (r[0].as_u64().unwrap() as u32, r[1].as_u64().unwrap() as u16, r[2].as_u64().unwrap()))
            .collect();
        let ticked = reads.iter().any(|r| r.0 == PORT_TICK);
        pending.extend(reads);
        if ticked {
            batches.push(std::mem::take(&mut pending));
        }
    }
    batches
}

#[derive(Default, Debug, Clone, Copy)]
struct Freshness {
    fresh: u64,   // the peer's plan of round k - 1
    stale: u64,   // an older one
    missing: u64, // none yet
    unsent: u64,  // the peer published no plan of round k - 1
    late: u64,    // a peer plan of round k - 1 read after tick k
    never: u64,   // a peer plan of round k - 1 never read
    early: u64,   // peer plans of round >= k already read at tick k
    state_behind: u64, // ticks k read before the own_state of round k
}

impl std::ops::AddAssign for Freshness {
    fn add_assign(&mut self, o: Self) {
        self.fresh += o.fresh;
        self.stale += o.stale;
        self.missing += o.missing;
        self.unsent += o.unsent;
        self.late += o.late;
        self.never += o.never;
        self.early += o.early;
        self.state_behind += o.state_behind;
    }
}

/// Robot i's closed-loop reads as replay batches, peer plans of round >= k
/// held back until after tick k, and its freshness per round and peer.
fn consumed(flight: &Flight, n: usize, i: usize) -> (Vec<Vec<Input>>, Freshness) {
    let reads = tick_reads(&flight.audit, i);
    assert_eq!(reads.len(), ROUNDS as usize, "r{i}: a step log entry per tick");
    let me = &flight.robots[i - 1];
    let feeder = (n + i - 1) as u16;
    let mut f = Freshness::default();
    let mut read_at: Vec<BTreeMap<u64, u64>> = vec![BTreeMap::new(); n]; // peer -> plan round -> tick it was first read by
    let (mut batches, mut deferred) = (Vec::new(), Vec::<Input>::new());
    for (k, reads) in reads.iter().enumerate() {
        let k = k as u64;
        let (mut batch, later): (Vec<Input>, Vec<Input>) = std::mem::take(&mut deferred).into_iter().partition(|p| p.1 < k);
        deferred = later;
        let mut tick = None;
        for &(port, origin, seq) in reads {
            if port == PORT_PLAN_IN {
                let j = origin as usize;
                assert!(j < n && j != i - 1, "r{i}: a plan from node {origin}");
                let (round, payload) = flight.robots[j].plan_by_seq.get(&seq).unwrap_or_else(|| panic!("r{i}: r{} plan seq {seq} unknown", j + 1)).clone();
                read_at[j].entry(round).or_insert(k);
                if round >= k {
                    f.early += 1;
                    deferred.push((1, round, payload));
                } else {
                    batch.push((1, round, payload));
                }
                continue;
            }
            assert_eq!(origin, feeder, "r{i}: port {port} read from node {origin}");
            let channel = match port {
                PORT_TICK => 0,
                PORT_OWN_STATE => 2,
                PORT_SCENE_SNAPSHOT => 4,
                PORT_SCENE_STATE => 5,
                _ => panic!("r{i}: plan-dmpc read port {port}"),
            };
            let (round, payload) = me.fed[&(channel, seq)].clone();
            if channel == 0 {
                assert_eq!(round, k, "r{i}: tick order");
                tick = Some((0, round, payload));
            } else {
                batch.push((channel, round, payload));
            }
        }
        let read_state = |b: &Vec<Input>| b.iter().any(|x| x.0 == 2 && x.1 == k);
        if !read_state(&batch) && !batches.iter().any(read_state) {
            f.state_behind += 1;
        }
        batch.push(tick.unwrap());
        batches.push(batch);
        if k == 0 {
            continue;
        }
        for (j, peer) in flight.robots.iter().enumerate().filter(|(j, _)| *j != i - 1) {
            let newest = read_at[j].iter().filter(|(round, at)| **round < k && **at <= k).map(|(round, _)| *round).max();
            match newest {
                Some(r) if r == k - 1 => f.fresh += 1,
                _ if !peer.plan_by_seq.values().any(|p| p.0 == k - 1) => f.unsent += 1,
                Some(_) => f.stale += 1,
                None => f.missing += 1,
            }
        }
    }
    for (j, peer) in flight.robots.iter().enumerate().filter(|(j, _)| *j != i - 1) {
        for round in peer.plan_by_seq.values().map(|p| p.0).filter(|r| *r + 1 < ROUNDS as u64) {
            match read_at[j].get(&round) {
                Some(&at) if at > round + 1 => f.late += 1,
                None => f.never += 1,
                _ => {}
            }
        }
    }
    (batches, f)
}

/// Fly `members` once over the Zenoh plugin, robot to robot through relays
/// with `profile`, and check the Block I gate. Returns the plan audit and
/// the fleet's freshness.
fn fly_zenoh_and_check(name: &str, setup: &Setup, members: &[Member], fleet_options: &[&str], profile: &dyn Fn(usize, usize) -> Profile) -> (PlanAudit, Freshness) {
    let n = members.len();
    let (scene, ticks) = record_scene(name, setup, members, fleet_options);
    let flight = fly(name, &setup.plan_dmpc, &setup.ctl_px4, members, &scene, &ticks, Net::zenoh(n, profile), true);
    let mut wall = flight.round_wall_ms.clone();
    wall.sort_by(|x, y| x.partial_cmp(y).unwrap());
    println!("{name}: round wall time p50 {:.1} ms min {:.1} ms max {:.1} ms", wall[wall.len() / 2], wall[0], wall[wall.len() - 1]);

    let audit = plan_audit(name, n, &flight);
    println!(
        "{name}: plans robot -> robot: {} sent, {} lost = relay drop {} + receive-side discard {} (held for reorder {}); relay duplicates {} (none received twice), relay held {}; pass-through direction lost {}",
        audit.sent, audit.lost, audit.drop, audit.discard, audit.held_discarded, audit.relay_duplicates, audit.relay_held, audit.lost_clean
    );
    let failures: Vec<Vec<(i64, String)>> = (1..=n).map(|i| planner_failures(&flight.audit, i)).collect();
    for (i, f) in failures.iter().enumerate() {
        for (t, message) in f {
            println!("{name} r{}: round at {:.1} s failed: {message}", i + 1, *t as f64 * 1e-9 - T0_MS as f64 * 1e-3);
        }
    }
    check_product(name, members, &flight.robots, &failures.iter().map(Vec::len).collect::<Vec<_>>());

    let mut fleet = Freshness::default();
    for i in 1..=n {
        let (batches, f) = consumed(&flight, n, i);
        println!("{name} r{i}: {f:?}");
        let (plans, setpoints) = replay_batches(&format!("{name}-replay-r{i}"), &setup.plan_dmpc, &members[i - 1].manifest, &batches);
        let published: Vec<(u64, Vec<u8>)> = flight.robots[i - 1].plan_by_seq.values().cloned().collect();
        assert_eq!(plans.len(), published.len(), "{name} r{i}: plan count");
        assert!(plans == published, "{name} r{i}: closed-loop plans differ from the replay of what it read (peer plans of round >= k held back)");
        assert!(setpoints == flight.robots[i - 1].setpoints, "{name} r{i}: closed-loop setpoints differ from the replay of what it read");
        fleet += f;
    }
    println!("{name}: fleet freshness {fleet:?}; every robot's rounds reproduced from its reads with peer plans of round >= k held back");
    (audit, fleet)
}

fn knot_members(setup: &Setup) -> Vec<Member> {
    // Each robot on the ground under its knot_fs150 seed point (the pentagon
    // of radius 1.2 m around the leader's start).
    (0..5)
        .map(|i| {
            let a = 2.0 * std::f64::consts::PI * i as f64 / 5.0;
            Member {
                kind: Kind::Uav,
                manifest: setup.fg_root.join(format!("test/replay/plan_dmpc/knot_fs150_full_uav{}.yaml", i + 1)),
                spawn: [1.2 * a.cos(), 1.2 * a.sin(), 0.0],
                yaw: 0.0,
            }
        })
        .collect()
}

fn mixed_members(setup: &Setup) -> Vec<Member> {
    // Spawn poses of mixed_circle/swarm_pose.yaml: agents 1-5 UAVs, 6-9 Scouts.
    let spawns: [[f64; 3]; 9] = [
        [-6.0, 1.2, 0.0], [-6.0, -1.2, 0.0], [-3.0, 0.0, 0.0], [0.0, 1.2, 0.0], [0.0, -1.2, 0.0],
        [-2.1, 0.9, 0.181], [-3.9, 0.9, 0.181], [-3.9, -0.9, 0.181], [-2.1, -0.9, 0.181],
    ];
    spawns
        .iter()
        .enumerate()
        .map(|(i, s)| Member {
            kind: if i < 5 { Kind::Uav } else { Kind::Scout },
            manifest: setup.fg_root.join(format!("test/replay/plan_dmpc/mixed_circle_agent{}.yaml", i + 1)),
            spawn: *s,
            yaw: std::f64::consts::FRAC_PI_2,
        })
        .collect()
}

const KNOT_SCENE: [&str; 2] = ["--scene", "config/scenarios/knot_fs150/scene.yaml"];
const MIXED_SCENE: [&str; 4] = ["--scene", "config/scenarios/mixed_circle/scene.yaml", "--spawn", "config/scenarios/mixed_circle/swarm_pose.yaml"];

#[test]
fn the_knot_fs150_fleet_flies_in_closed_loop_with_plans_over_the_link() {
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let Some(setup) = setup() else { return };
    fly_and_check("fleet-knot", &setup, &knot_members(&setup), &KNOT_SCENE, true);
}

#[test]
fn the_mixed_circle_fleet_flies_in_closed_loop_with_plans_over_the_link() {
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let Some(setup) = setup() else { return };
    fly_and_check("fleet-mixed", &setup, &mixed_members(&setup), &MIXED_SCENE, false);
}

#[test]
fn the_knot_fs150_fleet_flies_over_the_zenoh_plugin_through_clean_relays() {
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let Some(setup) = setup() else { return };
    let members = knot_members(&setup);
    let n = members.len();
    let (audit, fleet) = fly_zenoh_and_check("zfleet-knot-clean", &setup, &members, &KNOT_SCENE, &|i, j| Profile { seed: (i * n + j) as u64, ..Profile::default() });
    assert_eq!(audit.lost, 0, "clean relays: no plan lost");
    assert_eq!((fleet.stale, fleet.missing, fleet.late, fleet.never), (0, 0, 0, 0), "clean relays: every peer plan of round k - 1 read by tick k");
    assert!(fleet.early > 0, "round selection exercised: some peer plan of round k was read before tick k");
}

#[test]
fn the_knot_fs150_fleet_flies_over_impaired_zenoh() {
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let Some(setup) = setup() else { return };
    let members = knot_members(&setup);
    let (audit, fleet) = fly_zenoh_and_check("zfleet-knot-c4", &setup, &members, &KNOT_SCENE, &c4_profile(members.len()));
    assert!(audit.drop > 0 && audit.relay_duplicates > 0 && audit.relay_held > 0, "the profile exercised drop, duplicate and reorder: {audit:?}");
    assert!(fleet.stale > 0, "impaired relays: some round ran on a stale peer plan: {fleet:?}");
    assert!(fleet.early > 0, "round selection exercised: some peer plan of round k was read before tick k");
    assert_eq!(fleet.never, audit.lost, "every peer plan never read is an audited loss");
}

#[test]
fn the_mixed_circle_fleet_flies_over_impaired_zenoh() {
    let _one = FLIGHT.lock().unwrap_or_else(|e| e.into_inner());
    let Some(setup) = setup() else { return };
    let members = mixed_members(&setup);
    let (audit, _) = fly_zenoh_and_check("zfleet-mixed-c4", &setup, &members, &MIXED_SCENE, &c4_profile(members.len()));
    assert!(audit.drop > 0 && audit.relay_duplicates > 0 && audit.relay_held > 0, "the profile exercised drop, duplicate and reorder: {audit:?}");
}
