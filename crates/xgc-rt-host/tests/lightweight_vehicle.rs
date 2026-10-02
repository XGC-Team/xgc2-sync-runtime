//! Load the lightweight vehicle ELF through the real host and advance each
//! host's independent simulated clock. The feeder and outputs use loopback
//! only. A batch instance (one thread, woken on input or at the output
//! period) must publish byte-identical states to one instance per robot
//! stepped on every 1 ms round.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, OnceLock};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use xgc_rt_core::audit::NullAudit;
use xgc_rt_core::clock::{Clock, ManualClock};
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::{ChannelSpec, Qos, TransportContext};
use xgc_rt_host::endpoint::Endpoint;
use xgc_rt_host::{Host, HostOptions, RunSummary};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const EPOCH_NS: i64 = 1_000_000_000;
const CMD_A: u32 = 0;
const POSE_A: u32 = 2;
const VELOCITY_A: u32 = 3;
const POSE_B: u32 = 4;
const VELOCITY_B: u32 = 5;
const VEHICLE_A: &str = "vehicle-a";
const VEHICLE_B: &str = "vehicle-b";
const FEEDER: &str = "feeder";

// The simulator product owns compilation and installation. Generic Host tests
// consume one explicit installed artifact and never rebuild domain sources.
fn lightweight_vehicle_elf() -> &'static Path {
    static ELF: OnceLock<PathBuf> = OnceLock::new();
    ELF.get_or_init(|| {
        let path = std::env::var_os("XGC_LIGHTWEIGHT_ELF")
            .filter(|value| !value.is_empty())
            .map(PathBuf::from)
            .expect("set XGC_LIGHTWEIGHT_ELF to the independent lightweight-sim owner's installed ELF");
        assert!(path.is_absolute() && path.is_file(),
                "XGC_LIGHTWEIGHT_ELF must name an existing absolute installed artifact: {}", path.display());
        path
    })
}

fn channels() -> Vec<ChannelSpec> {
    vec![
        ChannelSpec {
            id: 0,
            name: "cmd-a".into(),
            qos: Qos::Control,
        },
        ChannelSpec {
            id: 1,
            name: "cmd-b".into(),
            qos: Qos::Control,
        },
        ChannelSpec {
            id: 2,
            name: "pose-a".into(),
            qos: Qos::State,
        },
        ChannelSpec {
            id: 3,
            name: "velocity-a".into(),
            qos: Qos::State,
        },
        ChannelSpec {
            id: 4,
            name: "pose-b".into(),
            qos: Qos::State,
        },
        ChannelSpec {
            id: 5,
            name: "velocity-b".into(),
            qos: Qos::State,
        },
    ]
}

fn manifest(node: &str, elf: &Path) -> String {
    let (cmd, pose, velocity) = if node == VEHICLE_A {
        ("cmd-a", "pose-a", "velocity-a")
    } else {
        ("cmd-b", "pose-b", "velocity-b")
    };
    format!(
        r#"
[session]
id = "lightweight-vehicle-host-test"
node = "{node}"
roster = ["vehicle-a", "vehicle-b", "feeder"]
period_ms = 10
epoch_ns = {EPOCH_NS}

[transport]
kind = "loopback"

[audit]
dir = "audit"

[[channel]]
name = "cmd-a"
qos = "control"
[[channel]]
name = "cmd-b"
qos = "control"
[[channel]]
name = "pose-a"
qos = "state"
[[channel]]
name = "velocity-a"
qos = "state"
[[channel]]
name = "pose-b"
qos = "state"
[[channel]]
name = "velocity-b"
qos = "state"

[[plugin]]
name = "lightweight-vehicle"
path = "{}"
trigger = "both"
config = {{ model = "mecanum", epoch_ns = {EPOCH_NS}, step_ms = 1, output_ms = 10, initial_pose = [0.0, 0.0, 0.0, 1.5707963267948966] }}
bind = {{ cmd_vel = {{ channel = "{cmd}", from = ["feeder"] }}, pose = {{ channel = "{pose}" }}, velocity = {{ channel = "{velocity}" }} }}
"#,
        elf.display()
    )
}

struct HostRunner {
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<RunSummary>>,
}

impl HostRunner {
    fn start(host: Host) -> Self {
        let stop = Arc::new(AtomicBool::new(false));
        let thread_stop = stop.clone();
        let join = thread::spawn(move || host.run(&thread_stop).unwrap());
        Self {
            stop,
            join: Some(join),
        }
    }

    fn stop_and_join(mut self) -> RunSummary {
        self.stop.store(true, Ordering::Relaxed);
        self.join.take().unwrap().join().unwrap()
    }
}

impl Drop for HostRunner {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

#[derive(Debug)]
struct Pose {
    stamp: f64,
    position: [f64; 3],
    q_wxyz: [f64; 4],
}

#[derive(Debug)]
struct Twist {
    stamp: f64,
    linear: [f64; 3],
}

fn f64_at(bytes: &[u8], index: usize) -> f64 {
    let start = index * 8;
    f64::from_le_bytes(bytes[start..start + 8].try_into().unwrap())
}

fn decode_pose(bytes: &[u8]) -> Pose {
    assert_eq!(bytes.len(), 64, "xgc.pose/1 payload size");
    Pose {
        stamp: f64_at(bytes, 0),
        position: [f64_at(bytes, 1), f64_at(bytes, 2), f64_at(bytes, 3)],
        q_wxyz: [
            f64_at(bytes, 4),
            f64_at(bytes, 5),
            f64_at(bytes, 6),
            f64_at(bytes, 7),
        ],
    }
}

fn decode_twist(bytes: &[u8]) -> Twist {
    assert_eq!(bytes.len(), 56, "xgc.twist/1 payload size");
    Twist {
        stamp: f64_at(bytes, 0),
        linear: [f64_at(bytes, 1), f64_at(bytes, 2), f64_at(bytes, 3)],
    }
}

fn wait_state_pair(endpoint: &Endpoint, pose_channel: u32, velocity_channel: u32) -> (Pose, Twist) {
    let deadline = Instant::now() + Duration::from_secs(5);
    let mut poses = BTreeMap::<u64, Pose>::new();
    let mut velocities = BTreeMap::<u64, Twist>::new();
    loop {
        for frame in endpoint.drain() {
            match frame.header.channel {
                channel if channel == pose_channel => {
                    let pose = decode_pose(&frame.payload);
                    poses.insert(pose.stamp.to_bits(), pose);
                }
                channel if channel == velocity_channel => {
                    let twist = decode_twist(&frame.payload);
                    velocities.insert(twist.stamp.to_bits(), twist);
                }
                _ => {}
            }
        }
        if let Some(stamp) = poses
            .keys()
            .copied()
            .find(|stamp| velocities.contains_key(stamp))
        {
            return (
                poses.remove(&stamp).unwrap(),
                velocities.remove(&stamp).unwrap(),
            );
        }
        assert!(
            Instant::now() < deadline,
            "timed out waiting for pose/velocity pair on {pose_channel}/{velocity_channel}"
        );
        thread::sleep(Duration::from_millis(1));
    }
}

fn wait_host_epoch(audit_dir: &Path, node: &str) {
    let health = audit_dir.join(node).join("health.jsonl");
    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if std::fs::read_to_string(&health).is_ok_and(|text| text.contains("\"event\":\"epoch\"")) {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "host {node} did not reach its epoch setup"
        );
        thread::sleep(Duration::from_millis(1));
    }
}

fn assert_pair_stamp(pose: &Pose, velocity: &Twist) {
    assert_eq!(
        pose.stamp.to_bits(),
        velocity.stamp.to_bits(),
        "pose/velocity stamps must match"
    );
}

#[test]
fn real_host_loads_mecanum_elf_and_advances_without_a_remote_clock_barrier() {
    let elf = lightweight_vehicle_elf();
    let dir =
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/lightweight-vehicle-host-test");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    let bus = LoopbackBus::new();
    let clock_a = Arc::new(ManualClock::new(EPOCH_NS - 10_000_000));
    let clock_b = Arc::new(ManualClock::new(EPOCH_NS - 500_000_000));
    let roster = vec![
        VEHICLE_A.to_string(),
        VEHICLE_B.to_string(),
        FEEDER.to_string(),
    ];
    let feeder_context = TransportContext {
        session: "lightweight-vehicle-host-test".into(),
        node: FEEDER.into(),
        node_id: 2,
        roster,
        channels: channels(),
    };
    let feeder = Endpoint::open(
        Box::new(LoopbackTransport::new(bus.clone())),
        &feeder_context,
        clock_a.clone(),
        Arc::new(NullAudit),
        1 << 16,
    )
    .unwrap();
    feeder.declare_out(CMD_A).unwrap();
    feeder.declare_in(POSE_A, &[0]).unwrap();
    feeder.declare_in(VELOCITY_A, &[0]).unwrap();
    feeder.declare_in(POSE_B, &[1]).unwrap();
    feeder.declare_in(VELOCITY_B, &[1]).unwrap();

    let host_a = Host::new(
        Manifest::from_toml_str(&manifest(VEHICLE_A, elf)).unwrap(),
        &dir,
        Box::new(LoopbackTransport::new(bus.clone())),
        clock_a.clone(),
        HostOptions::default(),
    )
    .unwrap();
    let host_b = Host::new(
        Manifest::from_toml_str(&manifest(VEHICLE_B, elf)).unwrap(),
        &dir,
        Box::new(LoopbackTransport::new(bus)),
        clock_b.clone(),
        HostOptions::default(),
    )
    .unwrap();
    let runner_a = HostRunner::start(host_a);
    let runner_b = HostRunner::start(host_b);
    wait_host_epoch(&dir.join("audit"), VEHICLE_A);
    wait_host_epoch(&dir.join("audit"), VEHICLE_B);

    // A reaches the shared epoch while B remains half a second behind.
    clock_a.set(EPOCH_NS);
    let (initial, initial_velocity) = wait_state_pair(&feeder, POSE_A, VELOCITY_A);
    assert_pair_stamp(&initial, &initial_velocity);
    assert!((initial.position[0]).abs() < 1e-12 && (initial.position[1]).abs() < 1e-12);
    let root_half = std::f64::consts::FRAC_1_SQRT_2;
    assert!(
        (initial.q_wxyz[0] - root_half).abs() < 1e-12,
        "configured yaw is pi/2: {initial:?}"
    );
    assert!(
        (initial.q_wxyz[3] - root_half).abs() < 1e-12,
        "configured yaw is pi/2: {initial:?}"
    );

    // No cmd_vel: advancing simulated time must leave the position unchanged.
    clock_a.set(EPOCH_NS + 50_000_000);
    let (still, still_velocity) = wait_state_pair(&feeder, POSE_A, VELOCITY_A);
    assert_pair_stamp(&still, &still_velocity);
    assert_eq!(still.position[0], initial.position[0]);
    assert_eq!(still.position[1], initial.position[1]);

    // Body-left velocity (0, 1, 0) at yaw pi/2 points along world -x.
    let mut command = Vec::with_capacity(56);
    for value in [EPOCH_NS as f64 / 1e9 + 0.05, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0] {
        command.extend_from_slice(&value.to_le_bytes());
    }
    feeder
        .publish(CMD_A, 5, EPOCH_NS + 50_000_000, &command)
        .unwrap();

    // Loopback delivers synchronously and stamps t_rx before this clock jump.
    // Advance A by 200 ms while B's independent clock is still frozen.
    clock_a.set(EPOCH_NS + 250_000_000);
    let (final_pose, final_velocity) = wait_state_pair(&feeder, POSE_A, VELOCITY_A);
    assert_pair_stamp(&final_pose, &final_velocity);
    let active_ns = ((final_pose.stamp - (EPOCH_NS as f64 / 1e9 + 0.05)) * 1e9).round() as i64;
    assert!(
        active_ns >= 200_000_000,
        "vehicle-a must advance at least 0.2 s after cmd_vel ({active_ns} ns): {final_pose:?}"
    );
    assert!(
        final_velocity.linear[0] < 0.0,
        "body-left at yaw pi/2 must move toward world -x: {final_velocity:?}"
    );
    assert!(
        final_velocity.linear[1].abs() < 1e-6,
        "world y velocity should be near zero: {final_velocity:?}"
    );
    assert!(
        (final_pose.position[0] + 0.2).abs() <= 0.002,
        "expected 0.2 s at 1 m/s along world -x: {final_pose:?}"
    );
    assert!(
        final_pose.position[1].abs() <= 0.002,
        "world y displacement should be near zero: {final_pose:?}"
    );
    assert_eq!(
        clock_b.now(),
        EPOCH_NS - 500_000_000,
        "vehicle-b was not advanced with vehicle-a"
    );
    assert!(
        feeder
            .drain()
            .iter()
            .all(|frame| frame.header.channel != POSE_B && frame.header.channel != VELOCITY_B),
        "vehicle-b must not publish before its own clock reaches the epoch"
    );

    // B can then advance on its own; it had no command and remains stationary.
    clock_b.set(EPOCH_NS + 10_000_000);
    let (b_pose, b_velocity) = wait_state_pair(&feeder, POSE_B, VELOCITY_B);
    assert_pair_stamp(&b_pose, &b_velocity);
    assert!((b_pose.position[0]).abs() < 1e-12 && (b_pose.position[1]).abs() < 1e-12);

    let summary_a = runner_a.stop_and_join();
    let summary_b = runner_b.stop_and_join();
    let plugin_a = summary_a
        .plugins
        .iter()
        .find(|plugin| plugin.name == "lightweight-vehicle")
        .unwrap();
    let plugin_b = summary_b
        .plugins
        .iter()
        .find(|plugin| plugin.name == "lightweight-vehicle")
        .unwrap();
    assert!(
        plugin_a.steps > 0 && plugin_a.last_error.is_none(),
        "host-a plugin failed: {plugin_a:?}"
    );
    assert!(
        plugin_b.steps > 0 && plugin_b.last_error.is_none(),
        "host-b plugin failed: {plugin_b:?}"
    );
    assert_eq!(
        plugin_a.consumed, 1,
        "host-a consumes the single cmd_vel sample"
    );
    assert_eq!(
        plugin_b.consumed, 0,
        "host-b has an independent command channel"
    );
}

// --- batch vs independent plants through the real host -----------------------

const BATCH: &str = "batch";
const INDEPENDENT: &str = "independent";
const BATCH_ROBOTS: usize = 3;
const BATCH_POSES: [[f64; 4]; BATCH_ROBOTS] =
    [[0.0, 0.0, 0.0, 0.0], [1.0, -1.0, 0.5, 0.3], [-2.0, 0.5, 0.0, 1.2]];

fn batch_channels() -> Vec<ChannelSpec> {
    let mut names = Vec::new();
    for r in 0..BATCH_ROBOTS {
        names.push((format!("cmd-{r}"), Qos::Control));
        names.push((format!("req-{r}"), Qos::Event));
    }
    for host in ["b", "i"] {
        for r in 0..BATCH_ROBOTS {
            for port in ["pose", "vel", "fcu"] {
                names.push((format!("{host}-{port}-{r}"), Qos::State));
            }
        }
    }
    names
        .into_iter()
        .enumerate()
        .map(|(id, (name, qos))| ChannelSpec { id: id as u32, name, qos })
        .collect()
}

fn batch_channel(name: &str) -> u32 {
    batch_channels().iter().find(|c| c.name == name).unwrap().id
}

fn pose_list(poses: &[[f64; 4]]) -> String {
    let values: Vec<String> = poses.iter().flatten().map(|v| format!("{v:?}")).collect();
    format!("[{}]", values.join(", "))
}

/// The batch host runs all robots in one instance woken on input or every
/// output period; the independent host runs today's shape: one instance per
/// robot stepped on every 1 ms round.
fn batch_manifest(node: &str, elf: &Path) -> String {
    let mut text = format!(
        "[session]\nid = \"lightweight-batch-host-test\"\nnode = \"{node}\"\n\
         roster = [\"{BATCH}\", \"{INDEPENDENT}\", \"{FEEDER}\"]\nperiod_ms = 1\n\
         epoch_ns = {EPOCH_NS}\n[transport]\nkind = \"loopback\"\n[audit]\ndir = \"audit\"\n"
    );
    for channel in batch_channels() {
        let qos = match channel.qos {
            Qos::Control => "control",
            Qos::Event => "event",
            _ => "state",
        };
        text += &format!("[[channel]]\nname = \"{}\"\nqos = \"{qos}\"\n", channel.name);
    }
    let common = format!("model = \"fs150\", epoch_ns = {EPOCH_NS}, step_ms = 1, output_ms = 10");
    let bind = |r: usize, suffix: &str, host: &str| {
        format!(
            "setpoint{suffix} = {{ channel = \"cmd-{r}\", from = [\"{FEEDER}\"] }}, \
             fcu_request{suffix} = {{ channel = \"req-{r}\", from = [\"{FEEDER}\"] }}, \
             pose{suffix} = {{ channel = \"{host}-pose-{r}\" }}, \
             velocity{suffix} = {{ channel = \"{host}-vel-{r}\" }}, \
             fcu_state{suffix} = {{ channel = \"{host}-fcu-{r}\" }}"
        )
    };
    if node == BATCH {
        let binds: Vec<String> = (0..BATCH_ROBOTS)
            .map(|r| bind(r, &if r == 0 { String::new() } else { format!("_{r}") }, "b"))
            .collect();
        text += &format!(
            "[[plugin]]\nname = \"plants\"\npath = \"{}\"\ntrigger = \"on_dirty\"\nwake_ms = 10\n\
             config = {{ {common}, robots = {BATCH_ROBOTS}, initial_poses = {} }}\nbind = {{ {} }}\n",
            elf.display(),
            pose_list(&BATCH_POSES),
            binds.join(", ")
        );
    } else {
        for (r, pose) in BATCH_POSES.iter().enumerate() {
            text += &format!(
                "[[plugin]]\nname = \"plant-{r}\"\npath = \"{}\"\ntrigger = \"on_round\"\n\
                 config = {{ {common}, initial_pose = {} }}\nbind = {{ {} }}\n",
                elf.display(),
                pose_list(std::slice::from_ref(pose)),
                bind(r, "", "i")
            );
        }
    }
    text
}

fn position_target(stamp: f64, acceleration: [f64; 3]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(104);
    let mut doubles = [0.0f64; 12]; // stamp, p, v, a, yaw, yaw_rate
    doubles[0] = stamp;
    doubles[7..10].copy_from_slice(&acceleration);
    for value in doubles {
        bytes.extend_from_slice(&value.to_le_bytes());
    }
    bytes.extend_from_slice(&3135u16.to_le_bytes()); // acceleration only
    bytes.push(1); // world ENU
    bytes.extend_from_slice(&[0; 5]);
    bytes
}

fn fcu_request(stamp: f64, kind: u32, arm: u32, mode: &str) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(64);
    bytes.extend_from_slice(&stamp.to_le_bytes());
    bytes.extend_from_slice(&kind.to_le_bytes());
    bytes.extend_from_slice(&arm.to_le_bytes());
    let mut text = [0u8; 32];
    text[..mode.len()].copy_from_slice(mode.as_bytes());
    bytes.extend_from_slice(&text);
    bytes.extend_from_slice(&[0; 16]); // /2 correlation ID, flags and reserved
    bytes
}

/// Every output frame received so far, by (channel, stamp bits).
#[derive(Default)]
struct Outputs(BTreeMap<(u32, u64), Vec<u8>>);

impl Outputs {
    fn wait_for(&mut self, feeder: &Endpoint, keys: &[(u32, u64)]) {
        let deadline = Instant::now() + Duration::from_secs(5);
        loop {
            for frame in feeder.drain() {
                let stamp = f64_at(&frame.payload, 0).to_bits();
                let previous = self.0.insert((frame.header.channel, stamp), frame.payload);
                assert!(previous.is_none(), "channel {} published stamp {} twice", frame.header.channel, f64::from_bits(stamp));
            }
            if keys.iter().all(|key| self.0.contains_key(key)) {
                return;
            }
            assert!(Instant::now() < deadline, "timed out waiting for {keys:?}");
            thread::sleep(Duration::from_millis(1));
        }
    }
}

#[test]
fn a_batch_reproduces_independent_states_and_disabled_providers_reject_controls() {
    let elf = lightweight_vehicle_elf();
    let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/lightweight-vehicle-batch-test");
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();

    let bus = LoopbackBus::new();
    let clock = Arc::new(ManualClock::new(EPOCH_NS - 10_000_000));
    let roster = vec![BATCH.to_string(), INDEPENDENT.to_string(), FEEDER.to_string()];
    let feeder = Endpoint::open(
        Box::new(LoopbackTransport::new(bus.clone())),
        &TransportContext {
            session: "lightweight-batch-host-test".into(),
            node: FEEDER.into(),
            node_id: 2,
            roster,
            channels: batch_channels(),
        },
        clock.clone(),
        Arc::new(NullAudit),
        1 << 16,
    )
    .unwrap();
    for r in 0..BATCH_ROBOTS {
        feeder.declare_out(batch_channel(&format!("cmd-{r}"))).unwrap();
        feeder.declare_out(batch_channel(&format!("req-{r}"))).unwrap();
        for (host, origin) in [("b", 0), ("i", 1)] {
            for port in ["pose", "vel", "fcu"] {
                feeder.declare_in(batch_channel(&format!("{host}-{port}-{r}")), &[origin]).unwrap();
            }
        }
    }
    let start = |node: &str| {
        HostRunner::start(
            Host::new(
                Manifest::from_toml_str(&batch_manifest(node, elf)).unwrap(),
                &dir,
                Box::new(LoopbackTransport::new(bus.clone())),
                clock.clone(),
                HostOptions::default(),
            )
            .unwrap(),
        )
    };
    let batch = start(BATCH);
    let independent = start(INDEPENDENT);
    wait_host_epoch(&dir.join("audit"), BATCH);
    wait_host_epoch(&dir.join("audit"), INDEPENDENT);

    let send = |channel: String, stamp_ns: i64, payload: Vec<u8>| {
        feeder.publish(batch_channel(&channel), 0, stamp_ns, &payload).unwrap();
    };
    let mut outputs = Outputs::default();
    for step in 0..=40i64 {
        let now = EPOCH_NS + step * 10_000_000;
        clock.set(now);
        let stamp = now as f64 * 1e-9;
        for r in 0..BATCH_ROBOTS {
            let phase = (step * (r as i64 + 1)) as f64;
            let a = [0.2 * r as f64 - 0.1, 0.1 * phase.sin(), 0.5 + 0.1 * phase.cos()];
            // These inputs cannot activate a provider. The domain's explicit
            // CAS lifecycle is intentionally absent from this generic Host test.
            if step % 2 == 0 && !(r == 1 && step >= 15) {
                send(format!("cmd-{r}"), now, position_target(stamp, a));
            }
            if step == 0 {
                send(format!("req-{r}"), now, fcu_request(stamp, 1, 1, ""));
                send(format!("req-{r}"), now, fcu_request(stamp, 2, 0, "OFFBOARD"));
            }
        }
        if step == 15 {
            send("req-1".into(), now, fcu_request(stamp, 2, 0, "AUTO.LAND"));
        }
        if step == 35 {
            send("req-2".into(), now, fcu_request(stamp, 1, 0, ""));
        }
        let mut keys = Vec::new();
        for r in 0..BATCH_ROBOTS {
            for host in ["b", "i"] {
                for port in ["pose", "vel", "fcu"] {
                    keys.push((batch_channel(&format!("{host}-{port}-{r}")), stamp.to_bits()));
                }
            }
        }
        outputs.wait_for(&feeder, &keys);
        for r in 0..BATCH_ROBOTS {
            for port in ["pose", "vel", "fcu"] {
                let key = |host: &str| (batch_channel(&format!("{host}-{port}-{r}")), stamp.to_bits());
                assert_eq!(
                    outputs.0[&key("b")], outputs.0[&key("i")],
                    "robot {r} {port} differs at {} ms", step * 10
                );
            }
        }
    }
    let fcu = |r: usize| outputs.0[&(batch_channel(&format!("b-fcu-{r}")), ((EPOCH_NS + 400_000_000) as f64 * 1e-9).to_bits())].clone();
    let mode = |bytes: &[u8]| String::from_utf8_lossy(&bytes[16..]).trim_end_matches('\0').to_string();
    for r in 0..BATCH_ROBOTS {
        let state = fcu(r);
        assert_eq!(state[8], 0, "inactive provider {r} must stay disconnected");
        assert_eq!(state[9], 0, "inactive provider {r} must not accept arming");
        assert_ne!(mode(&state), "OFFBOARD", "inactive provider {r} must not accept mode changes");
    }

    let summary_b = batch.stop_and_join();
    let summary_i = independent.stop_and_join();
    for plugin in summary_b.plugins.iter().chain(&summary_i.plugins) {
        assert!(plugin.last_error.is_none(), "{plugin:?}");
    }
    let steps = |s: &RunSummary| s.plugins.iter().map(|p| p.steps).sum::<u64>();
    assert_eq!(summary_b.plugins.len(), 1);
    assert_eq!(summary_i.plugins.len(), BATCH_ROBOTS);
    assert!(
        steps(&summary_b) < steps(&summary_i),
        "the batch steps once per wake for all robots: {} vs {}",
        steps(&summary_b),
        steps(&summary_i)
    );
}
