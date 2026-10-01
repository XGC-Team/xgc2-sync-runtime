//! Hosting cost of lightweight FS150 plants in one real host, in real time.
//! It measures, it does not assert, so it only runs when asked:
//!
//!   XGC_LIGHTWEIGHT_BENCH=1 cargo test --release -p xgc-rt-host \
//!       --test lightweight_bench -- --nocapture
//!
//! Optional environment:
//!   XGC_LIGHTWEIGHT_ELF            prebuilt plugin (default: build this tree's)
//!   XGC_LIGHTWEIGHT_BENCH_SECONDS  measured seconds per run (default 5)
//!   XGC_LIGHTWEIGHT_BENCH_ROBOTS   robot counts (default 1,32,100)
//!   XGC_LIGHTWEIGHT_BENCH_SHAPES   per-robot-round,per-robot-dirty,batch,
//!                                  batch-round10,platform
//!   XGC_LIGHTWEIGHT_BENCH_LOADS    idle,commanded
//!   XGC_LIGHTWEIGHT_BENCH_OUT      JSON lines results file
//!   XGC_ROS_EDGE_STANDIN_ELF       prebuilt ROS edge stand-in (default: build
//!                                  tests/bench/ros_edge_standin.cpp with $CXX)
//!
//! Shapes: `per-robot-round` is one instance per robot stepped on every 1 ms
//! round (today's manifests); `per-robot-dirty` keeps one instance per robot
//! but wakes it on input or every output period; `batch` puts up to 8 robots
//! in one instance with the same wake rule (needs the 64-port plugin);
//! `batch-round10` is a dedicated plant host whose 10 ms rounds (the output
//! period) wake each batch instance, and inputs never do.
//! `platform` is the graph Core deploys for a lightweight Session
//! (core-xgc internal/lightweightplant/manifest.go): `batch-round10` plants
//! plus each FS150's two ROS edges, a MAVROS-facing one (on input, and every
//! 5 ms command poll) and a mocap one (on input), with `slice_ms = 0.001`,
//! the output period as step budget and Core's step-log sampling. The edges
//! are ROS-free stand-ins for ros_io (tests/bench/ros_edge_standin.cpp) that
//! repeat its host-facing work; roscpp's own publishing is not included.
//! Loads: `idle` has no controls and no link (hosting cost alone);
//! `commanded` adds a feeder process stand-in on loopback that arms each
//! robot, streams 50 Hz acceleration setpoints and receives every output.
//! For `platform`, `commanded` makes each MAVROS edge write what ros_io
//! writes for a controller's ROS input instead (arm, OFFBOARD, 50 Hz
//! acceleration setpoints); the plant host has no link.
//! CPU and context switches are read per thread from /proc for the host's
//! own threads (`xgc-*`), so the feeder and test threads are excluded.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use xgc_rt_core::audit::NullAudit;
use xgc_rt_core::clock::{Clock, WallClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, Transport, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions, RunSummary};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const OUTPUT_MS: u64 = 10;
const WARMUP_MS: u64 = 1000;
const STARTUP_MS: i64 = 3000;
const BATCH: usize = 8;
const OUTPUTS: [&str; 5] = ["pose", "velocity", "imu", "fcu_state", "paired_state"];
/// Core's lightweight plant timing and step-log bound (DefaultTiming,
/// StepLogLinesPerSecond).
const COMMAND_POLL_MS: u64 = 5;
const SLICE_MS: f64 = 0.001;
const STEP_LOG_LINES_PER_SECOND: f64 = 500.0;

fn env_or(name: &str, default: &str) -> String {
    std::env::var(name).unwrap_or_else(|_| default.to_string())
}

fn plugin_elf() -> PathBuf {
    if let Ok(path) = std::env::var("XGC_LIGHTWEIGHT_ELF") {
        return PathBuf::from(path);
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().unwrap();
    let output = root.join("target/plugin-tests/cpp/liblightweight_vehicle.so");
    std::fs::create_dir_all(output.parent().unwrap()).unwrap();
    let status = Command::new(root.join("scripts/build-lightweight-vehicle.sh")).arg(&output).status().unwrap();
    assert!(status.success(), "building lightweight-vehicle ELF failed");
    output
}

/// The ROS-free ros_io stand-in for the `platform` shape.
fn edge_elf() -> PathBuf {
    if let Ok(path) = std::env::var("XGC_ROS_EDGE_STANDIN_ELF") {
        return PathBuf::from(path);
    }
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().unwrap();
    let output = root.join("target/plugin-tests/cpp/libros_edge_standin.so");
    std::fs::create_dir_all(output.parent().unwrap()).unwrap();
    let cxx = std::env::var("CXX").unwrap_or_else(|_| "c++".into());
    let status = Command::new(cxx)
        .args(["-std=c++17", "-O2", "-fPIC", "-fvisibility=hidden", "-shared", "-Wall", "-Wextra", "-Werror"])
        .arg("-I")
        .arg(root.join("abi/include"))
        .arg("-I")
        .arg(root.join("plugins/common"))
        .arg("-I")
        .arg(root.join("plugins/ros-io"))
        .arg(root.join("crates/xgc-rt-host/tests/bench/ros_edge_standin.cpp"))
        .arg("-o")
        .arg(&output)
        .status()
        .unwrap();
    assert!(status.success(), "building the ROS edge stand-in failed");
    output
}

#[derive(Clone, Copy, PartialEq)]
enum Shape {
    PerRobotRound,
    PerRobotDirty,
    Batch,
    BatchRound10,
    Platform,
}

impl Shape {
    fn name(self) -> &'static str {
        match self {
            Shape::PerRobotRound => "per-robot-round",
            Shape::PerRobotDirty => "per-robot-dirty",
            Shape::Batch => "batch",
            Shape::BatchRound10 => "batch-round10",
            Shape::Platform => "platform",
        }
    }
    /// Robots per plugin instance.
    fn group(self) -> usize {
        if matches!(self, Shape::Batch | Shape::BatchRound10 | Shape::Platform) {
            BATCH
        } else {
            1
        }
    }
}

struct Layout {
    robots: usize,
    commanded: bool,
    channels: Vec<ChannelSpec>,
}

impl Layout {
    fn new(robots: usize, commanded: bool) -> Self {
        let mut names = Vec::new();
        for r in 0..robots {
            if commanded {
                names.push((format!("sp-{r}"), Qos::Control));
                names.push((format!("req-{r}"), Qos::Event));
            }
            for port in OUTPUTS {
                names.push((format!("{port}-{r}"), Qos::State));
            }
        }
        let channels = names.into_iter().enumerate().map(|(id, (name, qos))| ChannelSpec { id: id as u32, name, qos }).collect();
        Self { robots, commanded, channels }
    }

    fn channel(&self, name: &str) -> u32 {
        self.channels.iter().find(|c| c.name == name).unwrap().id
    }

    fn roster(&self) -> Vec<String> {
        if self.commanded {
            vec!["plant".into(), "feeder".into()]
        } else {
            vec!["plant".into()]
        }
    }

    fn manifest(&self, shape: Shape, elf: &Path, epoch: i64, run_for_ms: u64) -> String {
        let roster: Vec<String> = self.roster().iter().map(|n| format!("\"{n}\"")).collect();
        let period_ms = if shape == Shape::BatchRound10 { OUTPUT_MS } else { 1 };
        let mut text = format!(
            "[session]\nid = \"lightweight-bench\"\nnode = \"plant\"\nroster = [{}]\nperiod_ms = {period_ms}\n\
             epoch_ns = {epoch}\nrun_for_ms = {run_for_ms}\n[transport]\nkind = \"loopback\"\n[audit]\ndir = \"audit\"\n",
            roster.join(", ")
        );
        for c in &self.channels {
            let qos = match c.qos {
                Qos::Control => "control",
                Qos::Event => "event",
                _ => "state",
            };
            text += &format!("[[channel]]\nname = \"{}\"\nqos = \"{qos}\"\n", c.name);
        }
        let trigger = match shape {
            Shape::PerRobotRound | Shape::BatchRound10 => "trigger = \"on_round\"\n".to_string(),
            _ => format!("trigger = \"on_dirty\"\nwake_ms = {OUTPUT_MS}\n"),
        };
        for (g, first) in (0..self.robots).step_by(shape.group()).enumerate() {
            let members = shape.group().min(self.robots - first);
            let mut bind = Vec::new();
            let mut poses = Vec::new();
            for j in 0..members {
                let r = first + j;
                let suffix = if j == 0 { String::new() } else { format!("_{j}") };
                if self.commanded {
                    bind.push(format!("setpoint{suffix} = {{ channel = \"sp-{r}\", from = [\"feeder\"] }}"));
                    bind.push(format!("fcu_request{suffix} = {{ channel = \"req-{r}\", from = [\"feeder\"] }}"));
                }
                for port in OUTPUTS {
                    bind.push(format!("{port}{suffix} = {{ channel = \"{port}-{r}\" }}"));
                }
                poses.extend([(r % 10) as f64, (r / 10) as f64, 0.0, 0.0]);
            }
            let poses: Vec<String> = poses.iter().map(|v| format!("{v:?}")).collect();
            let placement = if shape.group() > 1 {
                format!("robots = {members}, initial_poses = [{}]", poses.join(", "))
            } else {
                format!("initial_pose = [{}]", poses.join(", "))
            };
            // A plant's deadline is its output period, not the 1 ms round:
            // with the default budget a step that publishes over a busy link
            // for 10 ms is abandoned as hung.
            text += &format!(
                "[[plugin]]\nname = \"plant-{g}\"\npath = \"{}\"\n{trigger}step_budget_ms = {OUTPUT_MS}\n\
                 config = {{ model = \"fs150\", epoch_ns = {epoch}, step_ms = 1, output_ms = {OUTPUT_MS}, {placement} }}\n\
                 bind = {{ {} }}\n",
                elf.display(),
                bind.join(", ")
            );
        }
        text
    }
}

/// Core's deployment graph for `robots` FS150 (manifest.go HostManifest):
/// channels `uavN/<port>`, plants `plant-fs150-<i>` of up to 8 robots stepped
/// on the 10 ms round, and edges `ros-uavN-mavros` / `ros-uavN-mocap`.
fn platform_manifest(robots: usize, commanded: bool, plant: &Path, edge: &Path, epoch: i64, run_for_ms: u64) -> String {
    let body = |r: usize| format!("uav{}", r + 1);
    let mut text = format!(
        "[session]\nid = \"lightweight-bench\"\nnode = \"plant\"\nroster = [\"plant\"]\nperiod_ms = {OUTPUT_MS}\n\
         epoch_ns = {epoch}\nrun_for_ms = {run_for_ms}\n[transport]\nkind = \"loopback\"\n"
    );
    // Core's stepsPerSecond: each plant instance once per output period; each
    // edge once per published state, plus a poll for each commanded one.
    let outputs = 1000.0 / OUTPUT_MS as f64;
    let plant_steps = robots.div_ceil(BATCH) as f64 * outputs;
    let edge_steps = robots as f64 * (2.0 * outputs + 1000.0 / COMMAND_POLL_MS as f64);
    let steps_every = ((plant_steps + edge_steps) / STEP_LOG_LINES_PER_SECOND).ceil().max(1.0) as u64;
    text += &format!("[audit]\ndir = \"audit\"\nsteps_every = {steps_every}\n");
    for r in 0..robots {
        for (port, qos) in [("setpoint", "control"), ("fcu_request", "event"), ("pose", "state"), ("velocity", "state"), ("imu", "state"), ("fcu_state", "state")] {
            text += &format!("[[channel]]\nname = \"{}/{port}\"\nqos = \"{qos}\"\n", body(r));
        }
    }
    for (i, first) in (0..robots).step_by(BATCH).enumerate() {
        let members = BATCH.min(robots - first);
        let mut bind = Vec::new();
        let mut poses = Vec::new();
        for j in 0..members {
            let (r, suffix) = (first + j, if j == 0 { String::new() } else { format!("_{j}") });
            for port in ["setpoint", "fcu_request"] {
                bind.push(format!("{port}{suffix} = {{ channel = \"{}/{port}\", from = [\"plant\"] }}", body(r)));
            }
            for port in ["pose", "velocity", "imu", "fcu_state"] {
                bind.push(format!("{port}{suffix} = {{ channel = \"{}/{port}\" }}", body(r)));
            }
            poses.extend([(r % 10) as f64, (r / 10) as f64, 0.0, 0.0]);
        }
        let poses: Vec<String> = poses.iter().map(|v| format!("{v:?}")).collect();
        let placement = if members == 1 {
            format!("initial_pose = [{}]", poses.join(", "))
        } else {
            format!("robots = {members}, initial_poses = [{}]", poses.join(", "))
        };
        text += &format!(
            "[[plugin]]\nname = \"plant-fs150-{i}\"\npath = \"{}\"\ntrigger = \"on_round\"\nstep_budget_ms = {OUTPUT_MS}\n\
             config = {{ model = \"fs150\", epoch_ns = {epoch}, step_ms = 1, output_ms = {OUTPUT_MS}, {placement} }}\n\
             bind = {{ {} }}\n",
            plant.display(),
            bind.join(", ")
        );
    }
    for r in 0..robots {
        let (b, ns) = (body(r), format!("/{}", body(r)));
        let reader = |port: &str| format!("{{ channel = \"{b}/{port}\", from = [\"plant\"] }}");
        let commands = if commanded { ", standin_commands = true" } else { "" };
        text += &format!(
            "[[plugin]]\nname = \"ros-{b}-mavros\"\npath = \"{}\"\ntrigger = \"on_dirty\"\nwake_ms = {COMMAND_POLL_MS}\n\
             step_budget_ms = {OUTPUT_MS}\n\
             config = {{ node_name = \"xgc_lightweight_plant\", frame_id = \"map\", slice_ms = {SLICE_MS}, \
             sim_pose_topic = \"{ns}/mavros/local_position/pose\", sim_velocity_topic = \"{ns}/mavros/local_position/velocity_local\", \
             sim_odometry_topic = \"{ns}/mavros/local_position/odom\", sim_imu_topic = \"{ns}/mavros/imu/data\", \
             sim_fcu_state_topic = \"{ns}/mavros/state\", alg_setpoint_topic = \"{ns}/mavros/setpoint_raw/local\", \
             sim_fcu_request_topic = \"{ns}/mavros\"{commands} }}\n\
             bind = {{ sim_pose = {}, sim_velocity = {}, sim_imu = {}, sim_fcu_state = {}, \
             alg_setpoint = {{ channel = \"{b}/setpoint\" }}, sim_fcu_request = {{ channel = \"{b}/fcu_request\" }} }}\n",
            edge.display(),
            reader("pose"),
            reader("velocity"),
            reader("imu"),
            reader("fcu_state"),
        );
        text += &format!(
            "[[plugin]]\nname = \"ros-{b}-mocap\"\npath = \"{}\"\ntrigger = \"on_dirty\"\nstep_budget_ms = {OUTPUT_MS}\n\
             config = {{ node_name = \"xgc_lightweight_plant\", frame_id = \"world\", slice_ms = {SLICE_MS}, \
             sim_pose_topic = \"/vrpn_client_node{ns}/pose\", sim_velocity_topic = \"/vrpn_client_node{ns}/twist\", \
             sim_imu_topic = \"{ns}/mavros/imu/data_raw\" }}\n\
             bind = {{ sim_pose = {}, sim_velocity = {}, sim_imu = {} }}\n",
            edge.display(),
            reader("pose"),
            reader("velocity"),
            reader("imu"),
        );
    }
    text
}

/// Edge steps in the recorded (sampled) rounds of the window, split by how
/// many model samples each read: a step that read part of one output burst
/// (some but not all of an edge's state channels) means the edge woke while
/// the plant was still publishing.
#[derive(Default)]
struct EdgeSteps {
    recorded: u64,
    with_input: u64,
    partial: u64,
    durations_ns: Vec<u64>,
}

#[derive(Clone, Default)]
struct ThreadSample {
    comm: String,
    run_ns: u64,
    voluntary: u64,
    involuntary: u64,
}

/// schedstat run time and context switches of every thread in this process.
fn threads() -> BTreeMap<u64, ThreadSample> {
    let mut result = BTreeMap::new();
    let Ok(entries) = std::fs::read_dir(format!("/proc/{}/task", std::process::id())) else { return result };
    for entry in entries.flatten() {
        let Some(tid) = entry.file_name().to_str().and_then(|t| t.parse().ok()) else { continue };
        let base = entry.path();
        let read = |name: &str| std::fs::read_to_string(base.join(name)).unwrap_or_default();
        let status = read("status");
        let counter = |key: &str| {
            status.lines().find_map(|l| l.strip_prefix(key)).and_then(|v| v.trim().parse().ok()).unwrap_or(0)
        };
        result.insert(
            tid,
            ThreadSample {
                comm: read("comm").trim().to_string(),
                run_ns: read("schedstat").split_whitespace().next().and_then(|v| v.parse().ok()).unwrap_or(0),
                voluntary: counter("voluntary_ctxt_switches:"),
                involuntary: counter("nonvoluntary_ctxt_switches:"),
            },
        );
    }
    result
}

fn fs150_setpoint(stamp: f64, acceleration: [f64; 3]) -> Vec<u8> {
    let mut doubles = [0.0f64; 12]; // stamp, p, v, a, yaw, yaw_rate
    doubles[0] = stamp;
    doubles[7..10].copy_from_slice(&acceleration);
    let mut bytes: Vec<u8> = doubles.iter().flat_map(|v| v.to_le_bytes()).collect();
    bytes.extend_from_slice(&3135u16.to_le_bytes()); // acceleration only
    bytes.push(1);
    bytes.extend_from_slice(&[0; 5]);
    bytes
}

fn fcu_request(stamp: f64, kind: u32, arm: u32, mode: &str) -> Vec<u8> {
    let mut bytes = stamp.to_le_bytes().to_vec();
    bytes.extend_from_slice(&kind.to_le_bytes());
    bytes.extend_from_slice(&arm.to_le_bytes());
    let mut text = [0u8; 32];
    text[..mode.len()].copy_from_slice(mode.as_bytes());
    bytes.extend_from_slice(&text);
    bytes
}

/// Stand-in for the robots' controller processes: arm, OFFBOARD, then a
/// 50 Hz acceleration stream per robot; counts every received output.
fn feeder(layout: Arc<Layout>, endpoint: Arc<Endpoint>, clock: Arc<dyn Clock>, epoch: i64, stop: Arc<AtomicBool>, received: Arc<AtomicU64>) {
    while clock.now() < epoch && !stop.load(Ordering::Relaxed) {
        thread::sleep(Duration::from_millis(1));
    }
    let mut tick = 0u64;
    while !stop.load(Ordering::Relaxed) {
        let now = clock.now();
        let stamp = now as f64 * 1e-9;
        for r in 0..layout.robots {
            let phase = tick as f64 * 0.02 + r as f64;
            let sp = fs150_setpoint(stamp, [0.1 * phase.sin(), 0.1 * phase.cos(), 0.05]);
            endpoint.publish(layout.channel(&format!("sp-{r}")), tick, now, &sp).unwrap();
            if tick == 0 {
                let req = layout.channel(&format!("req-{r}"));
                endpoint.publish(req, tick, now, &fcu_request(stamp, 1, 1, "")).unwrap();
                endpoint.publish(req, tick, now, &fcu_request(stamp, 2, 0, "OFFBOARD")).unwrap();
            }
        }
        received.fetch_add(endpoint.drain().len() as u64, Ordering::Relaxed);
        tick += 1;
        let next = epoch + (tick as i64) * 20_000_000;
        let wait = next - clock.now();
        if wait > 0 {
            thread::sleep(Duration::from_nanos(wait as u64));
        }
    }
}

fn percentile(sorted: &[u64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    sorted[((sorted.len() - 1) as f64 * p).round() as usize] as f64
}

fn run(shape: Shape, robots: usize, commanded: bool, elf: &Path, edge: Option<&Path>, seconds: u64) -> serde_json::Value {
    let platform = shape == Shape::Platform;
    let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../target/lightweight-bench")
        .join(format!("{}-{}-{robots}", shape.name(), if commanded { "commanded" } else { "idle" }));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    // The platform host has no link: its commands come from its own edges.
    let fed = commanded && !platform;
    let layout = Arc::new(Layout::new(robots, fed));
    let clock: Arc<dyn Clock> = Arc::new(WallClock::new(0));
    let epoch = clock.now() + STARTUP_MS * 1_000_000;
    let run_for_ms = WARMUP_MS + seconds * 1000 + 500;
    let text = if platform {
        platform_manifest(robots, commanded, elf, edge.expect("platform needs the edge stand-in"), epoch, run_for_ms)
    } else {
        layout.manifest(shape, elf, epoch, run_for_ms)
    };
    let manifest = Manifest::from_toml_str(&text).unwrap();
    let bus = LoopbackBus::new();
    let transport: Box<dyn Transport> = Box::new(LoopbackTransport::new(bus.clone()));
    let host = Host::new(manifest, &dir, transport, clock.clone(), HostOptions::default()).unwrap();

    let stop = Arc::new(AtomicBool::new(false));
    let received = Arc::new(AtomicU64::new(0));
    let feeder_thread = fed.then(|| {
        let endpoint = Endpoint::open(
            Box::new(LoopbackTransport::new(bus.clone())),
            &TransportContext {
                session: "lightweight-bench".into(),
                node: "feeder".into(),
                node_id: 1,
                roster: layout.roster(),
                channels: layout.channels.clone(),
            },
            clock.clone(),
            Arc::new(NullAudit),
            1 << 20,
        )
        .unwrap();
        for r in 0..robots {
            endpoint.declare_out(layout.channel(&format!("sp-{r}"))).unwrap();
            endpoint.declare_out(layout.channel(&format!("req-{r}"))).unwrap();
            for port in OUTPUTS {
                endpoint.declare_in(layout.channel(&format!("{port}-{r}")), &[0]).unwrap();
            }
        }
        let (layout, clock, stop, received) = (layout.clone(), clock.clone(), stop.clone(), received.clone());
        thread::Builder::new().name("bench-feeder".into()).spawn(move || feeder(layout, endpoint, clock, epoch, stop, received)).unwrap()
    });
    let host_thread = thread::Builder::new()
        .name("xgc-host".into())
        .spawn(move || {
            let never = AtomicBool::new(false);
            host.run(&never).unwrap()
        })
        .unwrap();

    let sleep_until = |t: i64| {
        let wait = t - clock.now();
        if wait > 0 {
            thread::sleep(Duration::from_nanos(wait as u64));
        }
    };
    let window_start = epoch + (WARMUP_MS as i64) * 1_000_000;
    let window_end = window_start + (seconds as i64) * 1_000_000_000;
    sleep_until(window_start);
    let (before, received_before) = (threads(), received.load(Ordering::Relaxed));
    sleep_until(window_end);
    let (after, received_after) = (threads(), received.load(Ordering::Relaxed));
    let summary: RunSummary = host_thread.join().unwrap();
    stop.store(true, Ordering::Relaxed);
    if let Some(t) = feeder_thread {
        t.join().unwrap();
    }

    let mut host_threads = 0;
    let (mut run_ns, mut voluntary, mut involuntary) = (0u64, 0u64, 0u64);
    // Where the host's time goes: the main (link routing, rounds, watchdog)
    // thread, plant module threads, step/health log writers, audit writer.
    let mut groups: BTreeMap<&str, (u64, u64)> = BTreeMap::new();
    for (tid, end) in &after {
        let Some(start) = before.get(tid) else { continue };
        if !end.comm.starts_with("xgc-") {
            continue;
        }
        host_threads += 1;
        run_ns += end.run_ns - start.run_ns;
        voluntary += end.voluntary - start.voluntary;
        involuntary += end.involuntary - start.involuntary;
        let group = match end.comm.as_str() {
            "xgc-host" => "main",
            "xgc-steps" | "xgc-health" => "logs",
            c if c.starts_with("xgc-audit") => "audit",
            c if c.starts_with("xgc-plant") => "plants",
            c if c.starts_with("xgc-ros-") => "edges",
            _ => "other",
        };
        let entry = groups.entry(group).or_default();
        entry.0 += end.run_ns - start.run_ns;
        entry.1 += end.voluntary - start.voluntary;
    }
    let breakdown: serde_json::Map<String, serde_json::Value> = groups
        .iter()
        .map(|(name, (ns, wakes))| {
            let value = serde_json::json!({ "cpu_percent": *ns as f64 / (seconds as f64 * 1e9) * 100.0, "wakeups_per_s": *wakes as f64 / seconds as f64 });
            (name.to_string(), value)
        })
        .collect();

    // Plant steps inside the window from the host's own step records (every
    // step, or with Core's sampling every step of the recorded rounds).
    let steps_text = std::fs::read_to_string(dir.join("audit/plant/steps.jsonl")).unwrap();
    let mut durations = Vec::new();
    let (mut mavros, mut mocap) = (EdgeSteps::default(), EdgeSteps::default());
    for line in steps_text.lines() {
        let record: serde_json::Value = serde_json::from_str(line).unwrap();
        let (t0, t1) = (record["t0"].as_i64().unwrap(), record["t1"].as_i64().unwrap());
        if t0 < window_start || t0 >= window_end {
            continue;
        }
        let module = record["m"].as_str().unwrap();
        let (edge, expected) = match module {
            m if m.ends_with("-mavros") => (&mut mavros, 4),
            m if m.ends_with("-mocap") => (&mut mocap, 3),
            _ => {
                durations.push((t1 - t0) as u64);
                continue;
            }
        };
        let reads = record["in"].as_array().unwrap().len();
        edge.recorded += 1;
        edge.with_input += u64::from(reads > 0);
        edge.partial += u64::from(reads > 0 && reads % expected != 0);
        edge.durations_ns.push((t1 - t0) as u64);
    }
    durations.sort_unstable();
    let window_s = seconds as f64;
    let run_s = run_for_ms as f64 / 1e3;
    let is_edge = |name: &str| name.starts_with("ros-");
    let published: u64 = summary.plugins.iter().filter(|p| !is_edge(&p.name)).map(|p| p.published).sum();
    let plant_steps: u64 = summary.plugins.iter().filter(|p| !is_edge(&p.name)).map(|p| p.steps).sum();
    let edge_steps = |suffix: &str| summary.plugins.iter().filter(|p| is_edge(&p.name) && p.name.ends_with(suffix)).map(|p| p.steps).sum::<u64>();
    let edge_json = |e: &mut EdgeSteps, steps: u64, edges: usize| {
        e.durations_ns.sort_unstable();
        serde_json::json!({
            "steps_per_s_per_edge": if edges == 0 { 0.0 } else { steps as f64 / run_s / edges as f64 },
            "recorded": e.recorded,
            "with_input": e.with_input,
            "partial_burst_percent": if e.with_input == 0 { 0.0 } else { e.partial as f64 * 100.0 / e.with_input as f64 },
            "step_us_mean": if e.durations_ns.is_empty() { 0.0 } else { e.durations_ns.iter().sum::<u64>() as f64 / e.durations_ns.len() as f64 / 1e3 },
            "step_us_p50": percentile(&e.durations_ns, 0.5) / 1e3,
            "step_us_p99": percentile(&e.durations_ns, 0.99) / 1e3,
        })
    };
    let edges = if platform { robots } else { 0 };
    let edge_summary = serde_json::json!({
        "mavros": edge_json(&mut mavros, edge_steps("-mavros"), edges),
        "mocap": edge_json(&mut mocap, edge_steps("-mocap"), edges),
    });
    let errors: Vec<_> = summary.plugins.iter().filter_map(|p| p.last_error.clone()).collect();
    let health = std::fs::read_to_string(dir.join("audit/plant/health.jsonl")).unwrap();
    let overruns = health.lines().filter(|l| l.contains("\"event\":\"overrun\"")).count();
    let busy_ns: u64 = durations.iter().sum();
    serde_json::json!({
        "shape": shape.name(),
        "load": if commanded { "commanded" } else { "idle" },
        "robots": robots,
        "instances": summary.plugins.len(),
        "host_threads": host_threads,
        "seconds": seconds,
        "cpu_percent": run_ns as f64 / (window_s * 1e9) * 100.0,
        "cpu_ms_per_sim_s": run_ns as f64 / 1e6 / window_s,
        "wakeups_per_s": voluntary as f64 / window_s,
        "preemptions_per_s": involuntary as f64 / window_s,
        "steps_per_s": if platform { plant_steps as f64 / run_s } else { durations.len() as f64 / window_s },
        "step_us_mean": if durations.is_empty() { 0.0 } else { busy_ns as f64 / durations.len() as f64 / 1e3 },
        "step_us_p50": percentile(&durations, 0.5) / 1e3,
        "step_us_p99": percentile(&durations, 0.99) / 1e3,
        "step_us_max": percentile(&durations, 1.0) / 1e3,
        "plant_busy_ms_per_sim_s": busy_ns as f64 / 1e6 / window_s,
        "outputs_per_s": published as f64 / ((run_for_ms as f64) / 1e3),
        "outputs_received_per_s": (received_after - received_before) as f64 / window_s,
        "host_wakeups_main": summary.wakeups,
        "threads": breakdown,
        "edges": edge_summary,
        "overruns": overruns,
        "plant_errors": errors.len(),
        "first_error": errors.first(),
        "aborted": summary.aborted,
    })
}

#[test]
fn lightweight_plant_hosting_cost() {
    if std::env::var("XGC_LIGHTWEIGHT_BENCH").is_err() {
        eprintln!("set XGC_LIGHTWEIGHT_BENCH=1 to run the lightweight plant benchmark");
        return;
    }
    let elf = plugin_elf();
    let ports = xgc_rt_host::plugin::load(&elf, None).unwrap().ports.len();
    let seconds: u64 = env_or("XGC_LIGHTWEIGHT_BENCH_SECONDS", "5").parse().unwrap();
    let robots: Vec<usize> = env_or("XGC_LIGHTWEIGHT_BENCH_ROBOTS", "1,32,100").split(',').map(|v| v.trim().parse().unwrap()).collect();
    let shapes: Vec<Shape> = env_or("XGC_LIGHTWEIGHT_BENCH_SHAPES", "per-robot-round,per-robot-dirty,batch,batch-round10")
        .split(',')
        .map(|s| match s.trim() {
            "per-robot-round" => Shape::PerRobotRound,
            "per-robot-dirty" => Shape::PerRobotDirty,
            "batch" => Shape::Batch,
            "batch-round10" => Shape::BatchRound10,
            "platform" => Shape::Platform,
            other => panic!("unknown shape {other}"),
        })
        .filter(|s| s.group() == 1 || ports >= 64)
        .collect();
    let edge = shapes.contains(&Shape::Platform).then(edge_elf);
    let loads: Vec<bool> = env_or("XGC_LIGHTWEIGHT_BENCH_LOADS", "idle,commanded").split(',').map(|l| l.trim() == "commanded").collect();
    let out = env_or(
        "XGC_LIGHTWEIGHT_BENCH_OUT",
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../target/lightweight-bench/results.jsonl"),
    );
    std::fs::create_dir_all(Path::new(&out).parent().unwrap()).unwrap();
    println!("elf {} ({ports} ports), {seconds} s per run", elf.display());
    println!("| shape | load | robots | inst | thr | CPU % | wakeups/s | steps/s | step µs mean/p99 | plant ms/s | outputs/s | overruns | errors |");
    println!("|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|");
    for &commanded in &loads {
        for &n in &robots {
            for &shape in &shapes {
                let r = run(shape, n, commanded, &elf, edge.as_deref(), seconds);
                println!(
                    "| {} | {} | {} | {} | {} | {:.1} | {:.0} | {:.0} | {:.1}/{:.1} | {:.1} | {:.0} | {} | {} |",
                    r["shape"].as_str().unwrap(),
                    r["load"].as_str().unwrap(),
                    n,
                    r["instances"],
                    r["host_threads"],
                    r["cpu_percent"].as_f64().unwrap(),
                    r["wakeups_per_s"].as_f64().unwrap(),
                    r["steps_per_s"].as_f64().unwrap(),
                    r["step_us_mean"].as_f64().unwrap(),
                    r["step_us_p99"].as_f64().unwrap(),
                    r["plant_busy_ms_per_sim_s"].as_f64().unwrap(),
                    r["outputs_per_s"].as_f64().unwrap(),
                    r["overruns"],
                    r["plant_errors"],
                );
                let parts: Vec<String> = r["threads"]
                    .as_object()
                    .unwrap()
                    .iter()
                    .map(|(name, g)| format!("{name} {:.1}% {:.0}/s", g["cpu_percent"].as_f64().unwrap(), g["wakeups_per_s"].as_f64().unwrap()))
                    .collect();
                println!("|  | threads: {} |", parts.join(", "));
                if shape == Shape::Platform {
                    for kind in ["mavros", "mocap"] {
                        let e = &r["edges"][kind];
                        println!(
                            "|  | {kind} edge: {:.0} steps/s, step µs mean/p50/p99 {:.1}/{:.1}/{:.1}, partial bursts {:.2}% of {} |",
                            e["steps_per_s_per_edge"].as_f64().unwrap(),
                            e["step_us_mean"].as_f64().unwrap(),
                            e["step_us_p50"].as_f64().unwrap(),
                            e["step_us_p99"].as_f64().unwrap(),
                            e["partial_burst_percent"].as_f64().unwrap(),
                            e["with_input"],
                        );
                    }
                }
                let mut record = r.clone();
                record["elf"] = serde_json::Value::String(elf.display().to_string());
                use std::io::Write;
                let mut file = std::fs::OpenOptions::new().create(true).append(true).open(&out).unwrap();
                writeln!(file, "{record}").unwrap();
            }
        }
    }
    println!("results appended to {out}");
}
