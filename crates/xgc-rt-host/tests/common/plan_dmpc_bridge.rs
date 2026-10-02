//! Test input bridge: XGCDMPC1 fleet-replay records → current plan-dmpc ports.
//!
//! Neighbor visibility matches dmpc-rounds (not a second scheduler): at planner
//! beat `k`, publish neighbor plans whose *source* round is `< k`, with the
//! transport envelope round stamped as `k`. Do not add another ≤k-1 buffer
//! inside plan-dmpc or by republishing with envelope = source round.

#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::time::{SystemTime, UNIX_EPOCH};

use sha2::{Digest, Sha256};

/// Current plan-dmpc port indices (kPorts 0..15).
pub mod port {
    pub const PAIRED: u32 = 0;
    pub const CONTROLLER: u32 = 1;
    pub const SCENE_SNAPSHOT: u32 = 2;
    pub const SCENE_HEARTBEAT: u32 = 3;
    pub const TIMELINE_COMMIT: u32 = 4;
    pub const NEIGHBOR_PLAN: u32 = 5;
    pub const SYNC_TRIGGER: u32 = 6;
    pub const TIMELINE_STATUS: u32 = 7;
    pub const POSITION_TARGET: u32 = 8;
    pub const OWN_PLAN: u32 = 9;
    pub const PLANNER_STATUS: u32 = 10;
    pub const NEIGHBOR_POSITION: u32 = 11;
    pub const OWN_POSITION: u32 = 12;
    pub const PLANAR_TARGET: u32 = 13;
    pub const CLOCK: u32 = 14;
    pub const ROUND_DONE: u32 = 15;
}

/// Old XGCDMPC1 fleet-replay port ids.
pub mod old {
    pub const FORMATION_TICK: u32 = 0;
    pub const PLAN_IN: u32 = 1;
    pub const PLAN_OUT: u32 = 2;
    pub const OWN_STATE: u32 = 3;
    pub const SETPOINT: u32 = 4;
    pub const SCENE_SNAPSHOT: u32 = 5;
    pub const SCENE_STATE: u32 = 6;
    pub const PLANAR_SETPOINT: u32 = 7;
    pub const CLOCK: u32 = 8;
}

pub const HOLD: u64 = 10;
pub const PERIOD_NS: i64 = 100_000_000;
pub const SCENE_ID: &str = "fleet-replay-empty";
pub const SCENE_EPOCH: &str = "fleet-replay";

pub const CHANNELS: [(&str, xgc_rt_core::transport::Qos); 16] = [
    ("paired_state", xgc_rt_core::transport::Qos::State),
    ("controller_state", xgc_rt_core::transport::Qos::State),
    ("scene_snapshot", xgc_rt_core::transport::Qos::State),
    ("scene_heartbeat", xgc_rt_core::transport::Qos::State),
    ("timeline_commit", xgc_rt_core::transport::Qos::Event),
    ("neighbor_plan", xgc_rt_core::transport::Qos::Control),
    ("sync_trigger", xgc_rt_core::transport::Qos::Control),
    ("timeline_status", xgc_rt_core::transport::Qos::State),
    ("position_target", xgc_rt_core::transport::Qos::Control),
    ("own_plan", xgc_rt_core::transport::Qos::Control),
    ("planner_status", xgc_rt_core::transport::Qos::State),
    ("neighbor_position", xgc_rt_core::transport::Qos::State),
    ("own_position", xgc_rt_core::transport::Qos::State),
    ("planar_target", xgc_rt_core::transport::Qos::Control),
    ("clock", xgc_rt_core::transport::Qos::Event),
    ("round_done", xgc_rt_core::transport::Qos::Event),
];

pub const FEEDER_OUT: [u32; 9] = [
    port::PAIRED,
    port::CONTROLLER,
    port::SCENE_SNAPSHOT,
    port::SCENE_HEARTBEAT,
    port::TIMELINE_COMMIT,
    port::NEIGHBOR_PLAN,
    port::NEIGHBOR_POSITION,
    port::SYNC_TRIGGER,
    port::CLOCK,
];

pub const FEEDER_IN: [u32; 4] = [
    port::POSITION_TARGET,
    port::OWN_PLAN,
    port::PLANAR_TARGET,
    port::ROUND_DONE,
];

#[derive(Clone, Debug)]
pub struct Record {
    pub round: u64,
    pub port: u32,
    pub payload: Vec<u8>,
}

pub fn read_records(path: &Path) -> Vec<Record> {
    let bytes = std::fs::read(path).unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    assert_eq!(&bytes[..8], b"XGCDMPC1", "{}: not an XGCDMPC1 file", path.display());
    let mut records = Vec::new();
    let mut i = 8;
    while i < bytes.len() {
        let round = u64::from_le_bytes(bytes[i..i + 8].try_into().unwrap());
        let port = u32::from_le_bytes(bytes[i + 8..i + 12].try_into().unwrap());
        let len = u32::from_le_bytes(bytes[i + 12..i + 16].try_into().unwrap()) as usize;
        records.push(Record {
            round,
            port,
            payload: bytes[i + 16..i + 16 + len].to_vec(),
        });
        i += 16 + len;
    }
    records
}

pub struct Env {
    pub core_prefix: PathBuf,
    pub acados_prefix: PathBuf,
    pub formation_generator_root: PathBuf,
    pub fleet_replay: PathBuf,
}

/// Require the four explicit prefixes. No path defaults and no skip-pass.
pub fn require_env() -> Env {
    let core_prefix = require_path("PLAN_DMPC_CORE_PREFIX");
    let acados_prefix = require_path("PLAN_DMPC_ACADOS_PREFIX");
    let formation_generator_root = require_path("FORMATION_GENERATOR_ROOT");
    let fleet_replay = require_path("DMPC_FLEET_REPLAY");
    assert!(
        core_prefix.join("include/formation_generator/dmpc_scheduler/dmpc_agent.h").is_file(),
        "PLAN_DMPC_CORE_PREFIX missing agent header: {}",
        core_prefix.display()
    );
    assert!(
        acados_prefix.join("lib/libacados.so").is_file() || acados_prefix.join("lib/libacados.so.0").is_file(),
        "PLAN_DMPC_ACADOS_PREFIX missing libacados: {}",
        acados_prefix.display()
    );
    assert!(
        formation_generator_root.join("test/replay/plan_dmpc").is_dir(),
        "FORMATION_GENERATOR_ROOT missing replay manifests: {}",
        formation_generator_root.display()
    );
    assert!(fleet_replay.is_file(), "DMPC_FLEET_REPLAY is not a file: {}", fleet_replay.display());
    Env {
        core_prefix,
        acados_prefix,
        formation_generator_root,
        fleet_replay,
    }
}

fn require_path(name: &str) -> PathBuf {
    let value = std::env::var_os(name).unwrap_or_else(|| {
        panic!("{name} must be set; refused to invent a private default or skip-pass")
    });
    let path = PathBuf::from(value);
    assert!(path.exists(), "{name} does not exist: {}", path.display());
    path
}

/// acados-installed/lib first — system /usr/local blasfeo mismatches this HPIPM.
pub fn library_path(env: &Env) -> String {
    let acados_lib = env.acados_prefix.join("lib");
    let core_lib = env.core_prefix.join("lib");
    let prev = std::env::var("LD_LIBRARY_PATH").unwrap_or_default();
    format!("{}:{}:{}", acados_lib.display(), core_lib.display(), prev)
}

pub fn plan_dmpc_lib(_env: &Env) -> &'static PathBuf {
    static LIB: OnceLock<PathBuf> = OnceLock::new();
    LIB.get_or_init(|| {
        let lib = require_path("PLAN_DMPC_NATIVE_LIBRARY");
        assert!(lib.is_absolute() && lib.is_file(), "PLAN_DMPC_NATIVE_LIBRARY must name the owning installed adapter");
        lib
    })
}

/// xgc.rigid_state/1 (112 B) → xgc.dmpc.paired_state/1 (96 B).
pub fn rigid_to_paired(rigid: &[u8]) -> Vec<u8> {
    assert_eq!(rigid.len(), 112, "xgc.rigid_state/1");
    let stamp = f64_le(rigid, 0);
    let px = f64_le(rigid, 8);
    let py = f64_le(rigid, 16);
    let pz = f64_le(rigid, 24);
    let vx = f64_le(rigid, 32);
    let vy = f64_le(rigid, 40);
    let vz = f64_le(rigid, 48);
    let qw = f64_le(rigid, 56);
    let qx = f64_le(rigid, 64);
    let qy = f64_le(rigid, 72);
    let qz = f64_le(rigid, 80);
    let mut out = Vec::with_capacity(96);
    put_f64(&mut out, stamp);
    put_f64(&mut out, stamp);
    put_f64(&mut out, px);
    put_f64(&mut out, py);
    put_f64(&mut out, pz);
    put_f64(&mut out, qx);
    put_f64(&mut out, qy);
    put_f64(&mut out, qz);
    put_f64(&mut out, qw);
    put_f64(&mut out, vx);
    put_f64(&mut out, vy);
    put_f64(&mut out, vz);
    out
}

/// Strip formation_tick head → bare xgc.dmpc.sync_trigger/1 (32 B).
pub fn sync_trigger_from_formation_tick(tick: &[u8]) -> (Vec<u8>, u64, f64) {
    assert!(tick.len() >= 48, "formation_tick too short");
    let trigger = tick[16..48].to_vec();
    let sequence_id = u64::from_le_bytes(trigger[0..8].try_into().unwrap());
    let trigger_time = f64_le(&trigger, 8);
    (trigger, sequence_id, trigger_time)
}

pub fn controller_hover(stamp_sec: f64) -> Vec<u8> {
    let mut out = vec![0u8; 56];
    out[0..8].copy_from_slice(&stamp_sec.to_le_bytes());
    out[8..13].copy_from_slice(b"Hover");
    out
}

pub fn empty_scene_blob() -> Vec<u8> {
    let mut out = vec![0u8; 144];
    out[0..4].copy_from_slice(&1u32.to_le_bytes()); // schema
    // obstacle/part/vertex counts already 0
    out[16..24].copy_from_slice(&1u64.to_le_bytes()); // revision
    write_cstr(&mut out[24..56], SCENE_ID);
    write_cstr(&mut out[56..72], "world");
    write_cstr(&mut out[72..136], SCENE_EPOCH);
    out
}

pub fn scene_heartbeat_now() -> Vec<u8> {
    let wall = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64();
    let mut out = vec![0u8; 8];
    out.copy_from_slice(&wall.to_le_bytes());
    out
}

/// Timeline commits that reproduce fleet-replay hold (k < HOLD) then rolling.
pub struct TimelineFeed {
    hold_request: [u8; 240],
    rolling_request: Option<[u8; 240]>,
    rolling_from: u64,
}

impl TimelineFeed {
    pub fn new(session: &str) -> Self {
        let mut hold = [0u8; 240];
        // schema=1, kind=3 (hold), revision=1, command_id=1
        hold[0..4].copy_from_slice(&1u32.to_le_bytes());
        hold[4..8].copy_from_slice(&3u32.to_le_bytes());
        hold[8..16].copy_from_slice(&1u64.to_le_bytes());
        hold[16..24].copy_from_slice(&1u64.to_le_bytes());
        // predecessor 0, digest 0, effective_round 0, anchor 0, rolling 0
        write_cstr(&mut hold[112..176], session);
        write_cstr(&mut hold[176..240], "feeder");
        Self {
            hold_request: hold,
            rolling_request: None,
            rolling_from: HOLD,
        }
    }

    /// Commit bytes for planner beat `k` (envelope + DmpcTick.round).
    pub fn commit_for_beat(&mut self, k: u64) -> Vec<u8> {
        if k < self.rolling_from {
            return pack_commit(&self.hold_request, k, 0);
        }
        if self.rolling_request.is_none() {
            let mut req = [0u8; 240];
            req[0..4].copy_from_slice(&1u32.to_le_bytes());
            req[4..8].copy_from_slice(&2u32.to_le_bytes()); // resume
            req[8..16].copy_from_slice(&2u64.to_le_bytes()); // revision
            req[16..24].copy_from_slice(&2u64.to_le_bytes()); // command_id
            req[24..32].copy_from_slice(&1u64.to_le_bytes()); // predecessor_revision
            // predecessor_digest: sha256 of hold request (plugin recomputes from bytes;
            // push_commit compares against stored digest from previous push)
            let digest = sha256_240(&self.hold_request);
            req[32..64].copy_from_slice(&digest);
            req[64..72].copy_from_slice(&self.rolling_from.to_le_bytes()); // effective_round
            // anchor_ns = 0
            req[80..84].copy_from_slice(&1u32.to_le_bytes()); // rolling
            write_cstr(&mut req[112..176], cstr_of(&self.hold_request[112..176]));
            write_cstr(&mut req[176..240], "feeder");
            self.rolling_request = Some(req);
        }
        let req = self.rolling_request.as_ref().unwrap();
        let mission_ns = (k - self.rolling_from) as i64 * PERIOD_NS;
        pack_commit(req, k, mission_ns)
    }
}

fn pack_commit(request: &[u8; 240], round_k: u64, mission_ns: i64) -> Vec<u8> {
    let mut out = Vec::with_capacity(256);
    out.extend_from_slice(request);
    out.extend_from_slice(&round_k.to_le_bytes());
    out.extend_from_slice(&mission_ns.to_le_bytes());
    out
}

fn cstr_of(bytes: &[u8]) -> &str {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    std::str::from_utf8(&bytes[..end]).unwrap()
}

/// One feeder beat: rounds-equivalent neighbor feed + paired/controller/commit/trigger.
pub struct BeatInputs {
    pub beat: u64,
    pub trigger_time: f64,
    pub paired: Vec<u8>,
    pub controller: Vec<u8>,
    pub neighbors: Vec<Vec<u8>>,
    pub sync_trigger: Vec<u8>,
    pub clocks_after: Vec<Vec<u8>>,
    pub scene: Option<Vec<u8>>,
    pub scene_age: f64,
}

/// Walk old records and emit beats. Neighbor payloads are those with source
/// round `< beat` that appear before the formation_tick of that beat (fleet-
/// replay order). Caller stamps the transport envelope as `beat` (rounds out).
pub fn beats_from_records(inputs: &[Record]) -> Vec<BeatInputs> {
    let mut beats = Vec::new();
    let mut pending_paired: Option<Vec<u8>> = None;
    let mut pending_neighbors: Vec<(u64, Vec<u8>)> = Vec::new();
    let mut scene = SceneFeed::default();
    let mut fresh_scene = None;
    for rec in inputs {
        match rec.port {
            old::OWN_STATE => pending_paired = Some(rigid_to_paired(&rec.payload)),
            old::PLAN_IN => pending_neighbors.push((rec.round, rec.payload.clone())),
            old::CLOCK => {
                let beat: &mut BeatInputs = beats.last_mut().expect("clock follows a planner beat");
                beat.clocks_after.push(rec.payload.clone());
            },
            old::FORMATION_TICK => {
                let (sync, beat, trigger_time) = sync_trigger_from_formation_tick(&rec.payload);
                let neighbors = pending_neighbors
                    .drain(..)
                    .filter(|(source, _)| *source < beat)
                    .map(|(_, p)| p)
                    .collect();
                let paired = pending_paired
                    .take()
                    .unwrap_or_else(|| panic!("formation_tick round {beat} has no own_state"));
                beats.push(BeatInputs {
                    beat,
                    trigger_time,
                    paired,
                    controller: controller_hover(trigger_time),
                    neighbors,
                    sync_trigger: sync,
                    clocks_after: Vec::new(),
                    scene: fresh_scene.take(),
                    scene_age: scene.last_stamp.map(|stamp| trigger_time - stamp).unwrap_or(0.0),
                });
            }
            old::SCENE_SNAPSHOT => scene.snapshot(&rec.payload),
            old::SCENE_STATE => fresh_scene = Some(scene.state(&rec.payload)),
            _ => {}
        }
    }
    beats
}

fn f64_le(buf: &[u8], at: usize) -> f64 {
    f64::from_le_bytes(buf[at..at + 8].try_into().unwrap())
}

fn put_f64(out: &mut Vec<u8>, v: f64) {
    out.extend_from_slice(&v.to_le_bytes());
}

fn write_cstr(dst: &mut [u8], s: &str) {
    assert!(s.len() < dst.len(), "cstr '{s}' does not fit");
    dst[..s.len()].copy_from_slice(s.as_bytes());
}

fn sha256_240(data: &[u8; 240]) -> [u8; 32] {
    Sha256::digest(data).into()
}

// Repack only the test recorder's scene data. Production uses ros_dmpc_edge;
// this feeds its fixed scene port with the same poses, twists and geometry.
#[derive(Default)]
struct SceneFeed {
    header: Vec<u8>,
    obstacles: Vec<Vec<u8>>,
    parts: Vec<Vec<u8>>,
    vertices: Vec<u8>,
    last_stamp: Option<f64>,
}
struct Reader<'a>(&'a [u8]);
impl<'a> Reader<'a> {
    fn take(&mut self, n: usize) -> &'a [u8] {
        let (value, rest) = self.0.split_at(n);
        self.0 = rest;
        value
    }
    fn u32(&mut self) -> u32 { u32::from_le_bytes(self.take(4).try_into().unwrap()) }
    fn text(&mut self) -> &'a str {
        let n = self.u32() as usize;
        std::str::from_utf8(self.take(n)).unwrap()
    }
}
impl SceneFeed {
    fn snapshot(&mut self, payload: &[u8]) {
        let mut r = Reader(payload);
        r.take(8); // stamp: freshness comes from state, as in the ROS bridge
        let revision = r.take(8);
        let count = r.u32();
        r.take(4);
        let frame = r.text();
        let scene_id = r.text();
        let epoch = r.text();
        self.header = vec![0; 144];
        self.header[..4].copy_from_slice(&1u32.to_le_bytes());
        self.header[4..8].copy_from_slice(&count.to_le_bytes());
        self.header[16..24].copy_from_slice(revision);
        write_cstr(&mut self.header[24..56], scene_id);
        write_cstr(&mut self.header[56..72], frame);
        write_cstr(&mut self.header[72..136], epoch);
        self.obstacles.clear(); self.parts.clear(); self.vertices.clear();
        for index in 0..count {
            let mut body = vec![0; 240];
            write_cstr(&mut body[..64], r.text());
            write_cstr(&mut body[64..112], r.text());
            body[120..176].copy_from_slice(r.take(56));
            body[112..116].copy_from_slice(r.take(4));
            write_cstr(&mut body[224..240], r.text());
            let parts = r.u32();
            self.obstacles.push(body);
            for _ in 0..parts {
                let mut part = vec![0; 120];
                part[..4].copy_from_slice(&index.to_le_bytes());
                write_cstr(&mut part[8..24], r.text());
                part[64..120].copy_from_slice(r.take(56));
                let kind: u32 = match r.text() {
                    "cylinder" => 0, "box" => 1, "capsule" => 2,
                    "sphere" => 3, "convex" => 4, other => panic!("geometry {other}"),
                };
                part[4..8].copy_from_slice(&kind.to_le_bytes());
                let size = r.take(24);
                let radius = r.take(8);
                let height = r.take(8);
                if kind == 1 { part[24..48].copy_from_slice(size); }
                else if kind != 4 {
                    part[24..32].copy_from_slice(radius);
                    if kind != 3 { part[32..40].copy_from_slice(height); }
                }
                let vertices = r.u32();
                part[56..60].copy_from_slice(&((self.vertices.len() / 24) as u32).to_le_bytes());
                part[60..64].copy_from_slice(&vertices.to_le_bytes());
                self.vertices.extend_from_slice(r.take(vertices as usize * 24));
                let triangles = r.u32();
                r.take(triangles as usize * 4); // support mapping uses vertices only
                self.parts.push(part);
            }
        }
        assert!(r.0.is_empty());
        self.header[8..12].copy_from_slice(&(self.parts.len() as u32).to_le_bytes());
        self.header[12..16].copy_from_slice(&((self.vertices.len() / 24) as u32).to_le_bytes());
    }
    fn state(&mut self, payload: &[u8]) -> Vec<u8> {
        let mut r = Reader(payload);
        let raw_stamp = f64::from_le_bytes(r.take(8).try_into().unwrap());
        // fleet_replay's scene decoder, like ros::Time, rounds source stamps
        // to integer nanoseconds before the planner observes moving geometry.
        let sec = raw_stamp.floor();
        let stamp = sec + ((raw_stamp - sec) * 1e9).round() * 1e-9;
        self.last_stamp = Some(stamp);
        self.header[136..144].copy_from_slice(&stamp.to_le_bytes());
        r.take(8); // scene time
        assert_eq!(r.take(8), &self.header[16..24]);
        r.take(4); // playing; the bridge carries each supplied state
        let count = r.u32();
        assert_eq!(r.text(), cstr_of(&self.header[56..72]));
        assert_eq!(r.text(), cstr_of(&self.header[72..136]));
        assert_eq!(count as usize, self.obstacles.len());
        for _ in 0..count {
            let id = r.text();
            let body = self.obstacles.iter_mut().find(|b| cstr_of(&b[..64]) == id).unwrap();
            body[120..224].copy_from_slice(r.take(104));
        }
        assert!(r.0.is_empty());
        let mut out = self.header.clone();
        for body in &self.obstacles { out.extend_from_slice(body); }
        for part in &self.parts { out.extend_from_slice(part); }
        out.extend_from_slice(&self.vertices);
        out
    }
}

pub fn scene_id(blob: &[u8]) -> &str { cstr_of(&blob[24..56]) }

pub fn scene_heartbeat_age(age: f64) -> Vec<u8> {
    let wall = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs_f64();
    // Replay the source-clock freshness decision onto a wall-clock receipt.
    // A captured sample exactly 0.5 s old is still fresh; test execution time
    // must not make the native consumer expire it one beat earlier.
    let receipt = if (age * 1e9).round() > 500_000_000.0 { wall - 1.0 } else { wall };
    receipt.to_le_bytes().to_vec()
}
