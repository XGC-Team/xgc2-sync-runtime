//! Load the ground vehicle ELF through the real host and advance each host's
//! independent simulated clock. The feeder and outputs use loopback only.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;
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

fn lightweight_vehicle_elf() -> &'static Path {
    static ELF: OnceLock<PathBuf> = OnceLock::new();
    ELF.get_or_init(|| {
        let root = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .canonicalize()
            .unwrap();
        let output = root.join("target/plugin-tests/cpp/liblightweight_vehicle.so");
        std::fs::create_dir_all(output.parent().unwrap()).unwrap();
        let status = Command::new(root.join("scripts/build-lightweight-vehicle.sh"))
            .arg(&output)
            .status()
            .expect("run scripts/build-lightweight-vehicle.sh (needs C++17 and Eigen headers)");
        assert!(status.success(), "building lightweight-vehicle ELF failed");
        assert!(
            output.is_file(),
            "build script did not create {}",
            output.display()
        );
        output
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
peer_timeout_ms = 100

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
