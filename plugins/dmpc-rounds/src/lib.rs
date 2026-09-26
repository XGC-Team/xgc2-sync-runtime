//! TRO DMPC, Phase 1: the academic planner node runs unchanged as a ROS node
//! on each robot; this module replaces what connected the robots.
//!
//! - **Local rounds instead of the central sync coordinator.** On every round
//!   boundary `k` it writes `sync_trigger` (periodic_sync/SyncTrigger:
//!   sequence k, trigger time = round start `E0 + k·P`). `ros_io` publishes it
//!   on the robot's own `/formation/sync_trigger`. No robot waits on another
//!   robot's tick.
//! - **Plans between robots go over the link.** `own_plan` is the node's
//!   `/formation/assumed_trajectories` as `ros_io` reads it. Samples whose
//!   `uav_id` is not this robot's are neighbor plans echoed back through ROS
//!   and are ignored. The robot's own plan is sent on `plan_out` (channel
//!   `dmpc/plan`) for the current round.
//! - **Neighbor plans back into ROS.** Each newer neighbor plan from `plan_in`
//!   is written once on `neighbor_plans`, which `ros_io` publishes on the local
//!   `/formation/assumed_trajectories`, where the node expects them.
//!
//! - **Mission phase, locally (optional).** With `robots` and `robot` set, it
//!   also derives the formation's mission phase (rolling, mission_time) on
//!   its own rounds, as the station's formation_mission_clock.py did centrally
//!   (see `mission`), and writes `formation_tick`
//!   (formation_generator/FormationTick) for the planner's
//!   swarm_mission_clock mode. The inputs are data: the operator `command`,
//!   the robot's own controller state (`own_state`), and the peers' states
//!   over the link (`state_in`; its own goes out on `state_out` every round,
//!   with its phase for the audit).
//!
//! The local SyncTrigger / FormationTick is a facade for the unchanged ROS
//! planner. The beat is this robot's round boundary on its aligned OS clock:
//! no robot, station or network message is a timing authority.
//!
//! Payloads: `xgc.dmpc.assumed_trajectory/1`, `xgc.dmpc.sync_trigger/1`,
//! `xgc.dmpc.mission_state/1`, `xgc.dmpc.formation_tick/1`
//! (abi/include/xgc_schemas_v1.h). Config: `uav_id` (this robot's id in the
//! messages), `participant_ids` (the trigger's active ids), `stale_rounds`
//! (how many rounds a neighbor plan may lag before it counts as missing,
//! default 2); for the mission phase `robots` (names, in participant_ids
//! order), `robot` (this robot's name), `peer_gate` ("start", default: peers
//! gate only the start; or "always", the station clock's rule),
//! `state_timeout` (1.0 s) ages the newest accepted `own_state` by
//! `now - t_produce` on the session clock, not by planner round.
//! `max_trigger_gap` (0.5 s), `duration` (0: none).
//! `planner_period_ms = 100` selects the 1 ms host / 100 ms planner schedule
//! (`round::PlannerCursor`); omitting it keeps the host round. With that
//! schedule, `origins`, `plan_n`, `plan_horizon` and `plan_rest_count` together
//! map each transport origin to one participant id and reject a neighbor plan
//! before the cache when the id, finiteness, or n / N+1 / rest count disagree.
//! Every `step` output round is the planner k. See `round`.
//!
//! Domain state: `waiting` until the node published its own plan, then
//! `complete` or `partial` (every neighbor fresh for the last round or not).

use std::collections::BTreeMap;

pub mod mission;
pub mod round;
pub mod timeline;

use xgc_rt_abi::neighbor::{NeighborExchange, NeighborStatus};
use xgc_rt_abi::*;

const OWN_PLAN: u32 = 0;
const PLAN_IN: u32 = 1;
const PLAN_OUT: u32 = 2;
const NEIGHBOR_PLANS: u32 = 3;
const SYNC_TRIGGER: u32 = 4;
const COMMAND: u32 = 5;
const OWN_STATE: u32 = 6;
const STATE_IN: u32 = 7;
const STATE_OUT: u32 = 8;
const FORMATION_TICK: u32 = 9;
const MISSION_REQUEST: u32 = 10;
const TIMELINE_ACK: u32 = 11;
const TIMELINE_STATUS: u32 = 12;
const TIMELINE_COMMIT: u32 = 13;

/// Bytes before the doubles in `xgc.dmpc.assumed_trajectory/1`.
pub const PLAN_HEADER: usize = 32;

/// The `uav_id` of an assumed-trajectory payload, if it is well formed.
pub fn plan_uav_id(p: &[u8]) -> Option<u32> {
    if p.len() < PLAN_HEADER {
        return None;
    }
    let u = |o: usize| u32::from_le_bytes(p[o..o + 4].try_into().unwrap());
    let (states, steps, rest) = (u(12) as usize, u(16) as usize, u(20) as usize);
    let doubles = states.checked_mul(steps)?.checked_add(rest)?;
    let bytes = doubles.checked_mul(8)?.checked_add(PLAN_HEADER)?;
    (p.len() == bytes).then(|| u(8))
}

/// `xgc.dmpc.sync_trigger/1` for round `k`.
pub fn sync_trigger(k: u64, trigger_time: f64, published_time: f64, ids: &[u32]) -> Vec<u8> {
    let mut out = Vec::with_capacity(32 + 4 * ids.len());
    out.extend_from_slice(&k.to_le_bytes());
    out.extend_from_slice(&trigger_time.to_le_bytes());
    out.extend_from_slice(&published_time.to_le_bytes());
    out.extend_from_slice(&(ids.len() as u32).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    for id in ids {
        out.extend_from_slice(&id.to_le_bytes());
    }
    out
}

fn text(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}

/// `xgc.dmpc.mission_state/1`.
pub fn mission_state(stamp: f64, round: u64, uav_id: u32, rolling: bool, mission_time: f64, state: &str) -> Vec<u8> {
    let mut out = Vec::with_capacity(80);
    out.extend_from_slice(&stamp.to_le_bytes());
    out.extend_from_slice(&round.to_le_bytes());
    out.extend_from_slice(&uav_id.to_le_bytes());
    out.extend_from_slice(&u32::from(rolling).to_le_bytes());
    out.extend_from_slice(&mission_time.to_le_bytes());
    let mut name = [0u8; 48];
    let n = state.len().min(47);
    name[..n].copy_from_slice(&state.as_bytes()[..n]);
    out.extend_from_slice(&name);
    out
}

/// (round, uav_id, rolling, mission_time, state) of an `xgc.dmpc.mission_state/1`.
pub fn read_mission_state(p: &[u8]) -> Option<(u64, u32, bool, f64, String)> {
    (p.len() == 80).then(|| {
        (
            u64::from_le_bytes(p[8..16].try_into().unwrap()),
            u32::from_le_bytes(p[16..20].try_into().unwrap()),
            u32::from_le_bytes(p[20..24].try_into().unwrap()) != 0,
            f64::from_le_bytes(p[24..32].try_into().unwrap()),
            text(&p[32..80]),
        )
    })
}

/// `xgc.dmpc.formation_tick/1`: the phase head, then the round's trigger.
pub fn formation_tick(rolling: bool, mission_time: f64, trigger: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(16 + trigger.len());
    out.extend_from_slice(&mission_time.to_le_bytes());
    out.extend_from_slice(&u32::from(rolling).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(trigger);
    out
}

pub struct DmpcRounds {
    host: Host,
    uav_id: u32,
    participant_ids: Vec<u32>,
    stale_rounds: u64,
    nx: Option<NeighborExchange>,
    /// Newest (round, seq) written on `neighbor_plans`, per origin.
    forwarded: BTreeMap<u16, (u64, u64)>,
    sent_own: bool,
    complete: bool,
    phase: Option<mission::MissionPhase>,
    own_state: String,
    /// This robot's phase per recent round, to compare with its peers'.
    history: BTreeMap<u64, (bool, f64)>,
    /// Peer reports whose phase for a round differed from this robot's.
    pub disagreements: u64,
    rolling: bool,
    clock: round::PlannerCursor,
    admission: Option<round::Admission>,
    planner_skips: u64,
    timeline: Option<timeline::Timeline>,
    /// Produce time (session ns) of the newest accepted control sample.
    /// Fresh while that state is Custom1 and `0 <= now - t_produce <= state_timeout`.
    control_t: Option<i64>,
    state_timeout_ns: i64,
    /// Envelope origin of this agent's control/board node. Neighbor plan
    /// origins are planner nodes and are not accepted as this state.
    control_origin: Option<u16>,
}

fn config_error(key: &str) -> String {
    format!("dmpc-rounds: invalid config {key}")
}

fn config_f64(table: &toml::Table, key: &str, default: f64) -> Result<f64, String> {
    match table.get(key) {
        None => Ok(default),
        Some(v) => v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).ok_or_else(|| config_error(key)),
    }
}

fn timeout_ns(seconds: f64) -> Result<i64, String> {
    if !seconds.is_finite() || seconds < 0.0 {
        return Err(config_error("state_timeout"));
    }
    let ns = (seconds * 1e9).round();
    if !ns.is_finite() || ns < 0.0 || ns > i64::MAX as f64 {
        return Err(config_error("state_timeout"));
    }
    Ok(ns as i64)
}

impl DmpcRounds {
    /// Accept a control sample only when it is a new in-window observation.
    /// Future, already-expired, and out-of-order envelopes are dropped so
    /// they cannot open a fresh window or move the stored produce time.
    /// A newer in-window non-Custom1 replaces the sample and drops tracking.
    fn accept_control(&mut self, origin: u16, t_produce: i64, state: &str, now: i64) -> bool {
        if self.control_origin.is_some() && self.control_origin != Some(origin) {
            return false;
        }
        let Some(age) = now.checked_sub(t_produce) else {
            return false;
        };
        if age < 0 || age > self.state_timeout_ns {
            return false;
        }
        if self.control_t.is_some_and(|prev| t_produce <= prev) {
            return false;
        }
        self.own_state = state.to_string();
        self.control_t = Some(t_produce);
        true
    }

    fn control_fresh(&self, now: i64) -> bool {
        let Some(t) = self.control_t else {
            return false;
        };
        self.own_state == "Custom1" && now.checked_sub(t).is_some_and(|age| (0..=self.state_timeout_ns).contains(&age))
    }

    /// Mission-phase inputs of this step: operator commands, this robot's
    /// controller state, and peers' states (with the phase they had).
    fn mission_inputs(&mut self, now: i64) {
        let ordered = self.timeline.is_some();
        let own_name = if self.phase.is_some() { self.own_name() } else { String::new() };
        if self.phase.is_some() && !ordered {
            while let Some(s) = self.host.next(COMMAND) {
                self.phase.as_mut().unwrap().command(&text(s.data));
            }
        } else {
            while self.host.next(COMMAND).is_some() {}
        }
        let mut own_in = Vec::new();
        while let Some(s) = self.host.next(OWN_STATE) {
            own_in.push((s.origin, s.t_produce, s.data.to_vec()));
        }
        for (origin, t_produce, data) in own_in {
            // xgc.controller_status/1: stamp, then the state name.
            // When set, only the control/board origin counts. A planner
            // origin on this port is not readiness.
            if data.len() == 56 && self.accept_control(origin, t_produce, &text(&data[8..]), now) && self.phase.is_some() && !ordered {
                self.phase.as_mut().unwrap().observe(&own_name, &self.own_state, t_produce as f64 * 1e-9);
            }
        }
        if ordered || self.phase.is_none() {
            while self.host.next(STATE_IN).is_some() {}
            return;
        }
        while let Some(s) = self.host.next(STATE_IN) {
            let Some((round, uav, rolling, mission_time, state)) = read_mission_state(s.data) else { continue };
            let Some(idx) = self.participant_ids.iter().position(|&id| id == uav) else { continue };
            if uav == self.uav_id {
                continue;
            }
            let phase = self.phase.as_mut().unwrap();
            let name = phase.robots()[idx].clone();
            phase.observe(&name, &state, s.t_produce as f64 * 1e-9);
            if let Some(&(own_rolling, own_time)) = self.history.get(&round) {
                if own_rolling != rolling || (own_time - mission_time).abs() > 1e-9 {
                    self.disagreements += 1;
                    self.host.log(
                        XGC_LOG_WARN,
                        &format!("dmpc-rounds: round {round}: uav {uav} phase ({rolling}, {mission_time:.3}) differs from ours ({own_rolling}, {own_time:.3})"),
                    );
                }
            }
        }
    }

    fn own_name(&self) -> String {
        let idx = self.participant_ids.iter().position(|&id| id == self.uav_id);
        match (&self.phase, idx) {
            (Some(p), Some(i)) => p.robots()[i].clone(),
            _ => String::new(),
        }
    }

    /// The round's mission phase: FormationTick for the planner, and this
    /// robot's state + phase for its peers.
    fn mission_round(&mut self, ctx: &XgcStepCtx, planner_k: u64, trigger_s: f64, trigger: &[u8]) -> Result<(), String> {
        let Some(phase) = self.phase.as_mut() else { return Ok(()) };
        let now = ctx.now as f64 * 1e-9;
        if let Some((rolling, mission_time)) = phase.tick(planner_k, trigger_s, now, true) {
            self.rolling = rolling;
            self.history.insert(planner_k, (rolling, mission_time));
            while self.history.len() > 64 {
                self.history.pop_first();
            }
            let tick = formation_tick(rolling, mission_time, trigger);
            self.host.publish(FORMATION_TICK, planner_k, &tick).map_err(|s| format!("publish formation tick: {s}"))?;
            let state = mission_state(now, planner_k, self.uav_id, rolling, mission_time, &self.own_state);
            self.host.publish(STATE_OUT, planner_k, &state).map_err(|s| format!("publish mission state: {s}"))?;
        }
        Ok(())
    }
}

impl Plugin for DmpcRounds {
    fn create(host: Host) -> Self {
        Self {
            host,
            uav_id: 0,
            participant_ids: Vec::new(),
            stale_rounds: 2,
            nx: None,
            forwarded: BTreeMap::new(),
            sent_own: false,
            complete: false,
            phase: None,
            own_state: String::new(),
            history: BTreeMap::new(),
            disagreements: 0,
            rolling: false,
            clock: round::PlannerCursor::legacy(),
            admission: None,
            planner_skips: 0,
            timeline: None,
            control_t: None,
            state_timeout_ns: 1_000_000_000,
            control_origin: None,
        }
    }

    fn configure(&mut self, config: &str) -> Result<(), String> {
        let table: toml::Table = config.parse().map_err(|e| format!("dmpc-rounds config: {e}"))?;
        let int = |v: &toml::Value, key: &str| v.as_integer().and_then(|i| u32::try_from(i).ok()).ok_or_else(|| config_error(key));
        let state_timeout = config_f64(&table, "state_timeout", 1.0)?;
        self.state_timeout_ns = timeout_ns(state_timeout)?;
        self.uav_id = int(table.get("uav_id").ok_or("dmpc-rounds: uav_id is required")?, "uav_id")?;
        if let Some(ids) = table.get("participant_ids") {
            let ids = ids.as_array().ok_or_else(|| config_error("participant_ids"))?;
            self.participant_ids = ids.iter().map(|v| int(v, "participant_ids")).collect::<Result<_, _>>()?;
        }
        if let Some(v) = table.get("stale_rounds") {
            self.stale_rounds = u64::from(int(v, "stale_rounds")?);
        }
        if let Some(robots) = table.get("robots") {
            let robots: Vec<String> = robots
                .as_array()
                .ok_or_else(|| config_error("robots"))?
                .iter()
                .map(|v| v.as_str().map(str::to_string).ok_or_else(|| config_error("robots")))
                .collect::<Result<_, _>>()?;
            if robots.len() != self.participant_ids.len() {
                return Err("dmpc-rounds: robots must name every participant_ids entry, in order".into());
            }
            let own = table.get("robot").and_then(|v| v.as_str()).ok_or("dmpc-rounds: robot is required with robots")?;
            let gate = match table.get("peer_gate").and_then(|v| v.as_str()).unwrap_or("start") {
                "start" => mission::PeerGate::Start,
                "always" => mission::PeerGate::Always,
                _ => return Err(config_error("peer_gate")),
            };
            let num = |key: &str, default: f64| -> Result<f64, String> {
                table.get(key).map_or(Ok(default), |v| v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).ok_or_else(|| config_error(key)))
            };
            let phase = mission::MissionPhase::new(robots, own.to_string(), gate).map_err(|e| format!("dmpc-rounds: {e}"))?;
            self.phase = Some(phase.with_limits(state_timeout, num("max_trigger_gap", 0.5)?, num("duration", 0.0)?));
        }
        self.clock = if let Some(v) = table.get("planner_period_ms") {
            let ms = v.as_integer().ok_or_else(|| config_error("planner_period_ms"))?;
            if ms != round::PLANNER_PERIOD_MS {
                return Err("dmpc-rounds: planner_period_ms must be 100".into());
            }
            let epoch = match table.get("planner_epoch_ns") {
                None => 0,
                Some(e) => e.as_integer().ok_or_else(|| config_error("planner_epoch_ns"))?,
            };
            round::PlannerCursor::hundred_to_one(epoch)
        } else {
            round::PlannerCursor::legacy()
        };
        self.planner_skips = 0;
        let origins = table.get("origins");
        let plan_n = table.get("plan_n");
        let plan_horizon = table.get("plan_horizon");
        let plan_rest = table.get("plan_rest_count");
        self.admission = match (origins, plan_n, plan_horizon, plan_rest) {
            (None, None, None, None) => None,
            (Some(o), Some(n), Some(h), Some(r)) => {
                let origins = o.as_array().ok_or_else(|| config_error("origins"))?;
                let origins = origins.iter().map(|v| int(v, "origins")).collect::<Result<Vec<_>, _>>()?;
                let map = round::OriginMap::new(&self.participant_ids, &origins)?;
                if !map.contains_id(self.uav_id) {
                    return Err("dmpc-rounds: uav_id must be one participant id".into());
                }
                Some(round::Admission { map, shape: round::PlanShape::new(int(n, "plan_n")?, int(h, "plan_horizon")?, int(r, "plan_rest_count")?)? })
            }
            _ => return Err("dmpc-rounds: origins, plan_n, plan_horizon and plan_rest_count are set together".into()),
        };
        self.control_origin = match table.get("control_origin") {
            None => None,
            Some(v) => Some(u16::try_from(int(v, "control_origin")?).map_err(|_| config_error("control_origin"))?),
        };
        self.timeline = match table.get("timeline_mode").and_then(|v| v.as_str()) {
            None => None,
            Some("ordered-timeline-v1") => {
                if table.get("planner_period_ms").and_then(|v| v.as_integer()) != Some(timeline::PERIOD_NS / 1_000_000) {
                    return Err("dmpc-rounds: ordered-timeline-v1 requires planner_period_ms = 100".into());
                }
                let session = table.get("session_id").and_then(|v| v.as_str()).ok_or("dmpc-rounds: session_id is required")?;
                let authority = int(table.get("timeline_authority").ok_or("dmpc-rounds: timeline_authority is required")?, "timeline_authority")?;
                let authority = u16::try_from(authority).map_err(|_| config_error("timeline_authority"))?;
                let lead = match table.get("timeline_lead") {
                    None => timeline::MIN_LEAD,
                    Some(v) => u64::from(int(v, "timeline_lead")?),
                };
                Some(timeline::Timeline::open(session, authority, lead)?)
            }
            Some(_) => return Err("dmpc-rounds: unknown timeline_mode".into()),
        };
        Ok(())
    }

    fn activate(&mut self) -> Result<(), String> {
        self.nx = Some(NeighborExchange::new(&self.host, PLAN_IN, PLAN_OUT, self.stale_rounds));
        Ok(())
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        let beat = self.clock.observe(ctx)?;
        let k = beat.k;
        self.mission_inputs(ctx.now);
        if self.timeline.is_some() {
            let mut requests = Vec::new();
            while let Some(s) = self.host.next(MISSION_REQUEST) {
                requests.push((s.origin, s.data.to_vec()));
            }
            let mut acks = Vec::new();
            for (origin, data) in requests {
                match self.timeline.as_mut().unwrap().offer(origin, &data, k) {
                    Ok(timeline::Decision::Accepted { ack }) | Ok(timeline::Decision::Duplicate { ack }) => acks.push(ack),
                    Err(reason) => self.host.log(XGC_LOG_WARN, &format!("dmpc-rounds: rejected timeline request: {}", reason.reason())),
                }
            }
            for ack in acks {
                self.host.publish(TIMELINE_ACK, k, &ack).map_err(|s| format!("publish timeline ack: {s}"))?;
            }
        } else {
            while self.host.next(MISSION_REQUEST).is_some() {}
        }
        // The robot's own plan, as the node published it.
        let mut own = None;
        let mut own_in = Vec::new();
        while let Some(s) = self.host.next(OWN_PLAN) {
            own_in.push(s.data.to_vec());
        }
        for data in own_in {
            if let Some(admission) = &self.admission {
                if let Err(reason) = round::check_plan_shape(&admission.shape, self.uav_id, &data) {
                    self.host.log(XGC_LOG_WARN, &format!("dmpc-rounds: dropped own plan: {}", reason.reason()));
                    continue;
                }
            } else {
                match plan_uav_id(&data) {
                    Some(id) if id == self.uav_id => {}
                    Some(_) => continue, // a neighbor plan echoed back through ROS
                    None => {
                        self.host.log(XGC_LOG_WARN, "dmpc-rounds: malformed own plan dropped");
                        continue;
                    }
                }
            }
            own = Some(data);
        }
        // Neighbor plans: keep the newest, and pass each newer one to ROS once.
        {
            let nx = self.nx.as_mut().ok_or("not active")?;
            let mut incoming = Vec::new();
            while let Some(s) = self.host.next(PLAN_IN) {
                incoming.push((s.origin, s.round, s.seq, s.t_produce, s.data.to_vec()));
            }
            let mut fresh = Vec::new();
            for (origin, round, seq, t_produce, data) in incoming {
                if let Some(admission) = &self.admission {
                    if let Err(reason) = round::admit_plan(admission, self.uav_id, origin, &data) {
                        self.host.log(XGC_LOG_WARN, &format!("dmpc-rounds: dropped neighbor plan origin {origin}: {}", reason.reason()));
                        continue;
                    }
                }
                if !nx.offer(k, origin, round, seq, t_produce, &data) {
                    self.host.log(XGC_LOG_WARN, &format!("dmpc-rounds: dropped neighbor plan origin {origin} round {round} at planner round {k}"));
                    continue;
                }
                if self.forwarded.get(&origin).map_or(true, |&f| (round, seq) > f) {
                    self.forwarded.insert(origin, (round, seq));
                    fresh.push(data);
                }
            }
            for plan in fresh {
                self.host.publish(NEIGHBOR_PLANS, k, &plan).map_err(|s| format!("publish neighbor plan: {s}"))?;
            }
            if let Some(plan) = own {
                nx.publish(&self.host, k, &plan).map_err(|s| format!("publish plan: {s}"))?;
                self.sent_own = true;
            }
            if beat.advance {
                let snap = nx.snapshot(k, ctx.now);
                self.complete = snap.neighbors.iter().all(|n| n.status == NeighborStatus::Fresh);
            }
        }
        if beat.advance {
            self.planner_skips += beat.skipped;
            let trigger_s = beat.trigger_ns as f64 * 1e-9;
            let trigger = sync_trigger(k, trigger_s, ctx.now as f64 * 1e-9, &self.participant_ids);
            self.host.publish(SYNC_TRIGGER, k, &trigger).map_err(|s| format!("publish sync trigger: {s}"))?;
            if self.timeline.is_some() {
                let fresh = self.control_fresh(ctx.now);
                let view = self.timeline.as_mut().unwrap().on_round(k, beat.skipped, &self.own_state, fresh);
                self.rolling = view.rolling;
                self.host.publish(TIMELINE_STATUS, k, &view.status).map_err(|s| format!("publish timeline status: {s}"))?;
                self.host.publish(TIMELINE_COMMIT, k, &view.commit).map_err(|s| format!("publish timeline commit: {s}"))?;
                let tick = formation_tick(view.rolling, view.mission_s, &trigger);
                self.host.publish(FORMATION_TICK, k, &tick).map_err(|s| format!("publish formation tick: {s}"))?;
                let state = mission_state(ctx.now as f64 * 1e-9, k, self.uav_id, view.rolling, view.mission_s, &self.own_state);
                self.host.publish(STATE_OUT, k, &state).map_err(|s| format!("publish mission state: {s}"))?;
            } else {
                self.mission_round(ctx, k, trigger_s, &trigger)?;
            }
        }
        Ok(())
    }

    fn deactivate(&mut self) -> Result<(), String> {
        if let Some(phase) = &self.phase {
            self.host.log(
                XGC_LOG_INFO,
                &format!("dmpc-rounds: mission phase audit: {} peer-lost rounds, {} phase disagreements", phase.peers_lost, self.disagreements),
            );
        }
        Ok(())
    }

    fn domain_state(&self) -> &'static std::ffi::CStr {
        match (self.sent_own, self.complete) {
            (false, _) => cstr!("waiting"),
            (true, true) => cstr!("complete"),
            (true, false) => cstr!("partial"),
        }
    }
}

export_plugin! {
    plugin: DmpcRounds,
    name: "dmpc-rounds",
    version: "0.1.0",
    ports: [
        ("own_plan", XGC_PORT_IN, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL),
        ("plan_in", XGC_PORT_IN, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL),
        ("plan_out", XGC_PORT_OUT, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL),
        ("neighbor_plans", XGC_PORT_OUT, "xgc.dmpc.assumed_trajectory/1", XGC_QOS_CONTROL),
        ("sync_trigger", XGC_PORT_OUT, "xgc.dmpc.sync_trigger/1", XGC_QOS_CONTROL),
        ("command", XGC_PORT_IN_OPTIONAL, "xgc.command/1", XGC_QOS_EVENT),
        ("own_state", XGC_PORT_IN_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE),
        ("state_in", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.mission_state/1", XGC_QOS_STATE),
        ("state_out", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.mission_state/1", XGC_QOS_STATE),
        ("formation_tick", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.formation_tick/1", XGC_QOS_CONTROL),
        ("mission_request", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.mission_timeline/1", XGC_QOS_EVENT),
        ("timeline_ack", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.timeline_ack/1", XGC_QOS_EVENT),
        ("timeline_status", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.timeline_status/1", XGC_QOS_STATE),
        ("timeline_commit", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.mission_commit/1", XGC_QOS_EVENT),
    ],
}

#[cfg(test)]
mod tests {
    use super::*;

    fn plan(uav: u32, states: u32, steps: u32, rest: u32) -> Vec<u8> {
        let mut p = vec![0u8; PLAN_HEADER];
        p[8..12].copy_from_slice(&uav.to_le_bytes());
        p[12..16].copy_from_slice(&states.to_le_bytes());
        p[16..20].copy_from_slice(&steps.to_le_bytes());
        p[20..24].copy_from_slice(&rest.to_le_bytes());
        p.resize(PLAN_HEADER + 8 * (states * steps + rest) as usize, 0);
        p
    }

    #[test]
    fn rejects_overflowing_plan_counts_without_panicking() {
        for (states, steps, rest) in [
            (0x80000000u32, 0x40000000u32, 0u32),
            (u32::MAX, u32::MAX, 0),
            (u32::MAX, u32::MAX, u32::MAX),
            (0x10000, 0x10000, 0),
            (1, 1, u32::MAX),
            (0, 0, u32::MAX),
        ] {
            let mut p = vec![0u8; PLAN_HEADER];
            p[8..12].copy_from_slice(&1u32.to_le_bytes());
            p[12..16].copy_from_slice(&states.to_le_bytes());
            p[16..20].copy_from_slice(&steps.to_le_bytes());
            p[20..24].copy_from_slice(&rest.to_le_bytes());
            assert_eq!(plan_uav_id(&p), None);
        }
    }

    #[test]
    fn plan_ids_are_read_only_from_well_formed_payloads() {
        assert_eq!(plan_uav_id(&plan(3, 9, 51, 9)), Some(3));
        let mut short = plan(3, 9, 51, 0);
        short.pop();
        assert_eq!(plan_uav_id(&short), None);
        assert_eq!(plan_uav_id(&[0u8; 8]), None);
    }

    #[test]
    fn a_trigger_carries_the_round_and_its_scheduled_start() {
        let t = sync_trigger(7, 1.5, 1.5004, &[1, 2, 3]);
        assert_eq!(t.len(), 32 + 12);
        assert_eq!(u64::from_le_bytes(t[0..8].try_into().unwrap()), 7);
        assert_eq!(f64::from_le_bytes(t[8..16].try_into().unwrap()), 1.5);
        assert_eq!(u32::from_le_bytes(t[24..28].try_into().unwrap()), 3);
    }

    use std::collections::VecDeque;
    use std::ffi::c_void;
    use std::os::raw::c_char;

    struct SampleIn {
        origin: u16,
        seq: u64,
        round: u64,
        t_produce: i64,
        data: Vec<u8>,
    }
    struct Published {
        port: u32,
        round: u64,
        data: Vec<u8>,
    }
    struct Harness {
        api: XgcHostApi,
        inputs: Vec<VecDeque<SampleIn>>,
        published: Vec<Published>,
        origins: Vec<u16>,
        held: Vec<u8>,
    }

    unsafe extern "C" fn publish(host: *mut c_void, port: u32, round: u64, data: *const u8, len: u32) -> XgcStatus {
        let harness = &mut *(host as *mut Harness);
        let bytes = if len == 0 { Vec::new() } else { std::slice::from_raw_parts(data, len as usize).to_vec() };
        harness.published.push(Published { port, round, data: bytes });
        XGC_OK
    }
    unsafe extern "C" fn next_sample(host: *mut c_void, port: u32, out: *mut XgcSampleView) -> XgcStatus {
        let harness = &mut *(host as *mut Harness);
        let Some(sample) = harness.inputs.get_mut(port as usize).and_then(|q| q.pop_front()) else {
            return XGC_ERR_AGAIN;
        };
        harness.held = sample.data;
        std::ptr::write(out, XgcSampleView {
            origin: sample.origin,
            reserved: 0,
            len: harness.held.len() as u32,
            seq: sample.seq,
            round: sample.round,
            t_produce: sample.t_produce,
            t_tx: 0,
            t_rx: 0,
            data: if harness.held.is_empty() { std::ptr::null() } else { harness.held.as_ptr() },
        });
        XGC_OK
    }
    unsafe extern "C" fn now_ns(_: *mut c_void) -> i64 { 0 }
    unsafe extern "C" fn log(_: *mut c_void, _: XgcLogLevel, _: *const c_char) {}
    unsafe extern "C" fn degrade(_: *mut c_void, _: *const c_char) {}
    unsafe extern "C" fn recover(_: *mut c_void) {}
    unsafe extern "C" fn port_origins_cb(host: *mut c_void, _: u32, out: *mut u16, cap: u32) -> u32 {
        let harness = &*(host as *mut Harness);
        let n = harness.origins.len() as u32;
        if !out.is_null() && cap >= n {
            for (i, id) in harness.origins.iter().enumerate() {
                *out.add(i) = *id;
            }
        }
        n
    }
    unsafe extern "C" fn node_id(_: *mut c_void) -> u16 { 0 }

    struct Rig {
        plugin: DmpcRounds,
        harness: Box<Harness>,
    }

    impl Rig {
        fn open(config: &str, origins: &[u16]) -> Self {
            let mut harness = Box::new(Harness {
                api: XgcHostApi {
                    abi_version: 1,
                    abi_minor: 2,
                    host: std::ptr::null_mut(),
                    publish,
                    next: next_sample,
                    now: now_ns,
                    log,
                    request_degrade: degrade,
                    request_recover: recover,
                    port_origins: port_origins_cb,
                    node_id,
                },
                inputs: (0..16).map(|_| VecDeque::new()).collect(),
                published: Vec::new(),
                origins: origins.to_vec(),
                held: Vec::new(),
            });
            harness.api.host = (&mut *harness) as *mut Harness as *mut c_void;
            let api = &harness.api as *const XgcHostApi;
            let mut plugin = DmpcRounds::create(unsafe { Host::from_raw(api) });
            plugin.configure(config).unwrap();
            plugin.activate().unwrap();
            Self { plugin, harness }
        }

        fn push(&mut self, port: u32, origin: u16, round: u64, seq: u64, data: Vec<u8>) {
            self.push_at(port, origin, round, seq, 0, data);
        }

        fn push_at(&mut self, port: u32, origin: u16, round: u64, seq: u64, t_produce: i64, data: Vec<u8>) {
            self.harness.inputs[port as usize].push_back(SampleIn { origin, seq, round, t_produce, data });
        }

        /// `own_state` produced at planner boundary `k` (`t_produce = k · 100ms`).
        fn push_state(&mut self, origin: u16, k: u64, seq: u64, data: Vec<u8>) {
            self.push_at(OWN_STATE, origin, k, seq, k as i64 * timeline::PERIOD_NS, data);
        }

        fn step_ns(&mut self, host_round: u64, now: i64, advanced: bool) -> Result<(), String> {
            self.plugin.step(&XgcStepCtx {
                round: host_round,
                now,
                round_start: now,
                deadline: now,
                dirty_ports: 0,
                round_advanced: u32::from(advanced),
                reserved: 0,
            })
        }

        fn step_at(&mut self, k: u64, advanced: bool) -> Result<(), String> {
            self.step_ns(k.saturating_mul(100), k as i64 * timeline::PERIOD_NS, advanced)
        }

        fn of(&self, port: u32) -> Vec<&Published> {
            self.harness.published.iter().filter(|p| p.port == port).collect()
        }
    }

    fn ordered_config(robot: &str) -> String {
        format!(
            "uav_id = {robot}\nparticipant_ids = [1, 2]\nplanner_period_ms = 100\ntimeline_mode = \"ordered-timeline-v1\"\nsession_id = \"sess\"\ntimeline_authority = 7\n"
        )
    }

    fn named_state(name: &str) -> Vec<u8> {
        let mut p = vec![0u8; 56];
        let n = name.len().min(47);
        p[8..8 + n].copy_from_slice(&name.as_bytes()[..n]);
        p
    }

    fn custom1() -> Vec<u8> {
        named_state("Custom1")
    }

    fn arm_rolling(cfg: &str, origin: u16) -> Rig {
        let mut rig = Rig::open(cfg, &[]);
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        rig.push(MISSION_REQUEST, 7, 0, 1, start);
        for k in 0..=5 {
            rig.push_state(origin, k, 1, custom1());
            rig.step_at(k, true).unwrap();
        }
        rig
    }

    fn u64_at(bytes: &[u8], off: usize) -> u64 {
        u64::from_le_bytes(bytes[off..off + 8].try_into().unwrap())
    }
    fn i64_at(bytes: &[u8], off: usize) -> i64 {
        i64::from_le_bytes(bytes[off..off + 8].try_into().unwrap())
    }

    #[test]
    fn step_outputs_use_planner_k_and_two_modules_share_one_timeline() {
        let mut a = Rig::open(&ordered_config("1"), &[]);
        let mut b = Rig::open(&ordered_config("2"), &[]);
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        let start_digest = {
            use sha2::{Digest, Sha256};
            let mut hasher = Sha256::new();
            hasher.update(&start);
            let digest: [u8; 32] = hasher.finalize().into();
            digest
        };
        a.push(MISSION_REQUEST, 9, 0, 1, start.clone());
        a.push(MISSION_REQUEST, 7, 0, 2, start.clone());
        a.step_at(0, true).unwrap();
        assert_eq!(a.of(TIMELINE_ACK).len(), 1, "ack before the effective round, and only for the envelope sender");
        assert_eq!(a.of(TIMELINE_ACK)[0].round, 0);
        assert_eq!(a.of(TIMELINE_COMMIT).len(), 1);
        assert_eq!(u64_at(&a.of(TIMELINE_STATUS).last().unwrap().data, 56), 0, "not applied before effective round");
        a.push(MISSION_REQUEST, 7, 0, 3, start.clone());
        a.step_at(0, false).unwrap();
        assert_eq!(a.of(TIMELINE_ACK).len(), 2, "identical retry acks again");
        assert_eq!(a.of(TIMELINE_COMMIT).len(), 1, "dirty step does not commit");
        b.push(MISSION_REQUEST, 7, 0, 1, start);
        b.step_at(0, true).unwrap();
        for k in 1..=4 {
            a.push_state(0, k, 1, custom1());
            b.push_state(0, k, 1, custom1());
            a.step_at(k, true).unwrap();
            b.step_at(k, true).unwrap();
        }
        a.push_state(0, 5, 1, custom1());
        b.push_state(0, 5, 1, custom1());
        a.step_at(5, true).unwrap();
        b.step_at(5, true).unwrap();
        let status_a = &a.of(TIMELINE_STATUS).last().unwrap().data;
        let status_b = &b.of(TIMELINE_STATUS).last().unwrap().data;
        assert_eq!(status_a, status_b);
        assert_eq!(u64_at(status_a, 0), 1);
        assert_eq!(u64_at(status_a, 56), 1);
        assert_eq!(i64_at(status_a, 64), 0);
        assert_eq!(a.of(SYNC_TRIGGER).last().unwrap().round, 5);
        assert_eq!(u64_at(&a.of(SYNC_TRIGGER).last().unwrap().data, 0), 5);
        assert_ne!(a.of(SYNC_TRIGGER).last().unwrap().round, 500);
        a.push_state(0, 6, 1, custom1());
        b.push_state(0, 6, 1, custom1());
        a.step_at(6, true).unwrap();
        b.step_at(6, true).unwrap();
        let later_a = &a.of(TIMELINE_STATUS).last().unwrap().data;
        let later_b = &b.of(TIMELINE_STATUS).last().unwrap().data;
        assert_eq!(later_a, later_b);
        assert_eq!(i64_at(later_a, 64), timeline::PERIOD_NS);
        assert_eq!(i64_at(&a.of(TIMELINE_COMMIT).last().unwrap().data, 248), timeline::PERIOD_NS);
        assert_eq!(u64_at(&a.of(TIMELINE_COMMIT).last().unwrap().data, 240), 6);
        let stale = timeline::encode_request("sess", 2, 11, 1, &start_digest, 12, timeline::KIND_GOAL, true, 0, 0, [15.0, -15.0, 2.0]);
        a.push(MISSION_REQUEST, 7, 6, 4, stale);
        a.step_at(6, false).unwrap();
        assert_eq!(u64_at(&a.of(TIMELINE_STATUS).last().unwrap().data, 0), 1, "old anchor does not commit");
        let goal = timeline::encode_request("sess", 2, 11, 1, &start_digest, 12, timeline::KIND_GOAL, true, 700_000_000, 0, [15.0, -15.0, 2.0]);
        a.push_state(0, 7, 1, custom1());
        b.push_state(0, 7, 1, custom1());
        a.push(MISSION_REQUEST, 7, 7, 5, goal.clone());
        b.push(MISSION_REQUEST, 7, 7, 5, goal);
        a.step_at(7, true).unwrap();
        b.step_at(7, true).unwrap();
        assert_eq!(u64_at(&a.of(TIMELINE_STATUS).last().unwrap().data, 0), 2);
        assert_eq!(u64_at(&b.of(TIMELINE_STATUS).last().unwrap().data, 0), 2);
        assert_eq!(a.of(TIMELINE_ACK).last().unwrap().round, 7);
        assert_eq!(u64_at(&a.of(TIMELINE_STATUS).last().unwrap().data, 56), 1, "goal is not applied before its effective round");
    }

    #[test]
    fn a_lost_request_stays_held_and_custom1_does_not_start_it() {
        let mut held = Rig::open(&ordered_config("1"), &[]);
        let mut live = Rig::open(&ordered_config("1"), &[]);
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        live.push(MISSION_REQUEST, 7, 0, 1, start);
        for k in 0..=6 {
            held.push_state(0, k, 1, custom1());
            live.push_state(0, k, 1, custom1());
            held.step_at(k, true).unwrap();
            live.step_at(k, true).unwrap();
        }
        let held_status = &held.of(TIMELINE_STATUS).last().unwrap().data;
        assert_eq!(u64_at(held_status, 0), 0);
        assert_eq!(i64_at(held_status, 64), 0);
        assert_eq!(u32::from_le_bytes(held_status[72..76].try_into().unwrap()), 1);
        assert_ne!(i64_at(&live.of(TIMELINE_STATUS).last().unwrap().data, 64), 0);
        let mut jumped = Rig::open(&ordered_config("1"), &[]);
        jumped.push_state(0, 0, 1, custom1());
        jumped.step_at(0, true).unwrap();
        jumped.push_state(0, 3, 1, custom1());
        jumped.step_at(3, true).unwrap();
        let fault = u32::from_le_bytes(jumped.of(TIMELINE_STATUS).last().unwrap().data[76..80].try_into().unwrap());
        assert_eq!(fault, timeline::FAULT_CLOCK);
        assert_eq!(u32::from_le_bytes(jumped.of(TIMELINE_STATUS).last().unwrap().data[72..76].try_into().unwrap()), 1);
    }

    fn commit_rolling(bytes: &[u8]) -> u32 {
        u32::from_le_bytes(bytes[80..84].try_into().unwrap())
    }

    #[test]
    fn step_commit_holds_on_clock_fault_pending_goal_and_stale_controller() {
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        let mut clock = Rig::open(&ordered_config("1"), &[]);
        clock.push(MISSION_REQUEST, 7, 0, 1, start.clone());
        for k in 0..=7 {
            clock.push_state(0, k, 1, custom1());
            clock.step_at(k, true).unwrap();
        }
        clock.step_at(10, true).unwrap();
        let held = &clock.of(TIMELINE_COMMIT).last().unwrap().data;
        assert_eq!(u64_at(held, 8), 1, "fault does not invent a revision");
        assert_eq!(commit_rolling(held), 0);
        assert_eq!(i64_at(held, 72), i64_at(held, 248));
        assert_eq!(i64_at(held, 248), 2 * timeline::PERIOD_NS);
        assert_eq!(u32::from_le_bytes(clock.of(FORMATION_TICK).last().unwrap().data[8..12].try_into().unwrap()), 0);
        assert_eq!(u32::from_le_bytes(clock.of(TIMELINE_STATUS).last().unwrap().data[72..76].try_into().unwrap()), 1);
        let frozen = i64_at(held, 248);
        clock.step_at(11, true).unwrap();
        assert_eq!(i64_at(&clock.of(TIMELINE_COMMIT).last().unwrap().data, 248), frozen);
        assert_eq!(commit_rolling(&clock.of(TIMELINE_COMMIT).last().unwrap().data), 0);

        let mut pending = Rig::open(&ordered_config("1"), &[]);
        let goal = timeline::encode_request("sess", 2, 11, 1, &{
            use sha2::{Digest, Sha256};
            Sha256::digest(&start).into()
        }, 12, timeline::KIND_GOAL, true, 700_000_000, 0, [15.0, -15.0, 2.0]);
        pending.push(MISSION_REQUEST, 7, 0, 1, start.clone());
        pending.push(MISSION_REQUEST, 7, 0, 2, goal.clone());
        pending.step_at(0, true).unwrap();
        assert_eq!(pending.of(TIMELINE_ACK).len(), 1);
        assert_eq!(u64_at(&pending.of(TIMELINE_STATUS).last().unwrap().data, 0), 1);
        for k in 1..=5 {
            pending.push_state(0, k, 1, custom1());
            pending.step_at(k, true).unwrap();
        }
        assert_eq!(u64_at(&pending.of(TIMELINE_STATUS).last().unwrap().data, 56), 1);
        pending.push_state(0, 5, 2, custom1());
        pending.push(MISSION_REQUEST, 7, 5, 3, goal);
        pending.step_at(5, false).unwrap();
        assert_eq!(pending.of(TIMELINE_ACK).len(), 2);

        // Same-k freshness treated one silent planner round as expiry. The
        // configured timeout is 1.0s and the last Custom1 is produced at k=5
        // (500ms): k=6 is 100ms old and k=15 is exactly 1.0s, so both stay
        // rolling. k=16 is 1.1s and Holds. A later Custom1 does not unlatch.
        let mut stale_cfg = ordered_config("1");
        stale_cfg.push_str("state_timeout = 1.0\n");
        let mut stale = Rig::open(&stale_cfg, &[]);
        stale.push(MISSION_REQUEST, 7, 0, 1, start);
        for k in 0..=5 {
            stale.push_state(0, k, 1, custom1());
            stale.step_at(k, true).unwrap();
        }
        stale.step_at(6, true).unwrap();
        let still = &stale.of(TIMELINE_COMMIT).last().unwrap().data;
        assert_eq!(commit_rolling(still), 1, "100ms since produce is inside the 1.0s timeout");
        assert_eq!(i64_at(still, 248), timeline::PERIOD_NS);
        for k in 7..=15 {
            stale.step_at(k, true).unwrap();
            assert_eq!(commit_rolling(&stale.of(TIMELINE_COMMIT).last().unwrap().data), 1, "k={k}");
        }
        assert_eq!(i64_at(&stale.of(TIMELINE_COMMIT).last().unwrap().data, 248), 10 * timeline::PERIOD_NS);
        stale.step_at(16, true).unwrap();
        let paused = &stale.of(TIMELINE_COMMIT).last().unwrap().data;
        assert_eq!(commit_rolling(paused), 0);
        assert_eq!(i64_at(paused, 248), 11 * timeline::PERIOD_NS);
        stale.push_state(0, 17, 1, custom1());
        stale.step_at(17, true).unwrap();
        assert_eq!(commit_rolling(&stale.of(TIMELINE_COMMIT).last().unwrap().data), 0);
        assert_eq!(i64_at(&stale.of(TIMELINE_COMMIT).last().unwrap().data, 248), 11 * timeline::PERIOD_NS);
        assert_eq!(u32::from_le_bytes(stale.of(TIMELINE_STATUS).last().unwrap().data[76..80].try_into().unwrap()), timeline::FAULT_NOT_READY);
    }

    #[test]
    fn step_goal_effective_without_controller_holds_the_commit() {
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        let start_digest = {
            use sha2::{Digest, Sha256};
            let dig: [u8; 32] = Sha256::digest(&start).into();
            dig
        };
        let goal = timeline::encode_request("sess", 2, 11, 1, &start_digest, 12, timeline::KIND_GOAL, true, 700_000_000, 0, [15.0, -15.0, 2.0]);
        let mut cfg = ordered_config("1");
        cfg.push_str("state_timeout = 1.0\n");
        let mut rig = Rig::open(&cfg, &[]);
        rig.push(MISSION_REQUEST, 7, 0, 1, start);
        // Last Custom1 is produced at k=1 (100ms). Age is exactly 1.0s at k=11
        // and 1.1s at the goal's effective round k=12. A sample on k=11 would
        // still be fresh at k=12; the hold is this timeout, not a missing packet.
        for k in 0..=5 {
            if k == 1 {
                rig.push_state(0, k, 1, custom1());
            }
            rig.step_at(k, true).unwrap();
        }
        rig.push(MISSION_REQUEST, 7, 6, 2, goal);
        rig.step_at(6, true).unwrap();
        assert_eq!(u64_at(&rig.of(TIMELINE_STATUS).last().unwrap().data, 0), 2);
        assert_eq!(u64_at(&rig.of(TIMELINE_STATUS).last().unwrap().data, 56), 1);
        for k in 7..=11 {
            rig.step_at(k, true).unwrap();
            assert_eq!(commit_rolling(&rig.of(TIMELINE_COMMIT).last().unwrap().data), 1, "k={k}");
        }
        rig.step_at(12, true).unwrap();
        let held = &rig.of(TIMELINE_COMMIT).last().unwrap().data;
        assert_eq!(commit_rolling(held), 0);
        assert_eq!(u64_at(held, 8), 1, "the unapplied goal does not replace the running revision");
        assert_eq!(i64_at(held, 72), 700_000_000);
        assert_eq!(i64_at(held, 248), 700_000_000);
        rig.push_state(0, 13, 1, custom1());
        rig.step_at(13, true).unwrap();
        let later = &rig.of(TIMELINE_COMMIT).last().unwrap().data;
        assert_eq!(commit_rolling(later), 0);
        assert_eq!(i64_at(later, 248), 700_000_000);
        assert_eq!(u32::from_le_bytes(rig.of(TIMELINE_STATUS).last().unwrap().data[76..80].try_into().unwrap()), timeline::FAULT_NOT_READY);
    }

    #[test]
    fn step_board_origin_is_the_only_controller_source() {
        let mut cfg = ordered_config("1");
        cfg.push_str("control_origin = 4\n");
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        let mut rig = Rig::open(&cfg, &[]);
        rig.push(MISSION_REQUEST, 7, 0, 1, start);
        for k in 0..=5 {
            rig.push_state(1, k, 1, custom1());
            rig.step_at(k, true).unwrap();
        }
        assert_eq!(u64_at(&rig.of(TIMELINE_STATUS).last().unwrap().data, 56), 0);
        assert_eq!(commit_rolling(&rig.of(TIMELINE_COMMIT).last().unwrap().data), 0);
        rig.push_state(4, 6, 1, custom1());
        rig.step_at(6, true).unwrap();
        assert_eq!(commit_rolling(&rig.of(TIMELINE_COMMIT).last().unwrap().data), 0, "a late board sample does not apply the missed start");
    }

    /// Custom1 produced 99ms before a planner boundary stays fresh when that
    /// boundary step has no new sample. Age is 99ms; default state_timeout is
    /// 1s. Shifted +5 periods so the start is already rolling: produce at
    /// 501ms, judge at 600ms.
    #[test]
    fn step_custom1_99ms_before_boundary_stays_fresh() {
        let mut rig = Rig::open(&ordered_config("1"), &[]);
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        rig.push(MISSION_REQUEST, 7, 0, 1, start);
        for k in 0..=5 {
            rig.push_state(0, k, 1, custom1());
            rig.step_at(k, true).unwrap();
        }
        assert_eq!(commit_rolling(&rig.of(TIMELINE_COMMIT).last().unwrap().data), 1);
        rig.push_at(OWN_STATE, 0, 5, 2, 501_000_000, custom1());
        rig.step_ns(501, 501_000_000, true).unwrap();
        rig.step_at(6, true).unwrap();
        let commit = &rig.of(TIMELINE_COMMIT).last().unwrap().data;
        assert_eq!(commit_rolling(commit), 1, "99ms-old Custom1 is inside state_timeout; a silent boundary is not expiry");
        assert_eq!(i64_at(commit, 248), timeline::PERIOD_NS);
        assert_eq!(u32::from_le_bytes(rig.of(TIMELINE_STATUS).last().unwrap().data[76..80].try_into().unwrap()), timeline::FAULT_NONE);
    }

    #[test]
    fn step_non_custom1_revokes_tracking_immediately() {
        let start = timeline::encode_request("sess", 1, 10, 0, &[0u8; 32], 5, timeline::KIND_START, true, 0, 0, [0.0; 3]);
        let mut rig = Rig::open(&ordered_config("1"), &[]);
        rig.push(MISSION_REQUEST, 7, 0, 1, start.clone());
        for k in 0..=5 {
            rig.push_state(0, k, 1, custom1());
            rig.step_at(k, true).unwrap();
        }
        rig.push_at(OWN_STATE, 0, 6, 2, 6 * timeline::PERIOD_NS, named_state("Idle"));
        rig.step_at(6, true).unwrap();
        let held = &rig.of(TIMELINE_COMMIT).last().unwrap().data;
        assert_eq!(commit_rolling(held), 0, "a new in-window non-Custom1 drops tracking on that round");
        assert_eq!(i64_at(held, 248), timeline::PERIOD_NS);
        assert_eq!(u32::from_le_bytes(rig.of(TIMELINE_STATUS).last().unwrap().data[76..80].try_into().unwrap()), timeline::FAULT_NOT_READY);
        rig.push_state(0, 7, 1, custom1());
        rig.step_at(7, true).unwrap();
        assert_eq!(commit_rolling(&rig.of(TIMELINE_COMMIT).last().unwrap().data), 0);
        assert_eq!(i64_at(&rig.of(TIMELINE_COMMIT).last().unwrap().data, 248), timeline::PERIOD_NS);

        let mut future = Rig::open(&ordered_config("1"), &[]);
        future.push(MISSION_REQUEST, 7, 0, 1, start);
        for k in 0..=5 {
            future.push_state(0, k, 1, custom1());
            future.step_at(k, true).unwrap();
        }
        future.push_at(OWN_STATE, 0, 6, 2, 6 * timeline::PERIOD_NS + 1_000_000_000, named_state("Idle"));
        future.step_at(6, true).unwrap();
        assert_eq!(commit_rolling(&future.of(TIMELINE_COMMIT).last().unwrap().data), 1, "a future non-Custom1 does not revoke");
        assert_eq!(i64_at(&future.of(TIMELINE_COMMIT).last().unwrap().data, 248), timeline::PERIOD_NS);
    }

    #[test]
    fn step_stale_envelopes_cannot_refresh_control_age() {
        let mut cfg = ordered_config("1");
        cfg.push_str("state_timeout = 0.25\n");
        let fault = |rig: &Rig| u32::from_le_bytes(rig.of(TIMELINE_STATUS).last().unwrap().data[76..80].try_into().unwrap());

        // Older produce time must not replace the 500ms sample. At k=7 that
        // sample is 200ms old (<= 250ms). Adopting 400ms would be 300ms and Hold.
        let mut ooo = arm_rolling(&cfg, 0);
        ooo.push_at(OWN_STATE, 0, 6, 2, 400_000_000, custom1());
        ooo.step_at(6, true).unwrap();
        ooo.step_at(7, true).unwrap();
        assert_eq!(commit_rolling(&ooo.of(TIMELINE_COMMIT).last().unwrap().data), 1);
        ooo.step_at(8, true).unwrap();
        assert_eq!(commit_rolling(&ooo.of(TIMELINE_COMMIT).last().unwrap().data), 0);
        assert_eq!(fault(&ooo), timeline::FAULT_NOT_READY);

        // Future produce time must not become the stamp. At k=8 the real
        // sample is expired; 750ms would still be 50ms old.
        let mut future = arm_rolling(&cfg, 0);
        future.push_at(OWN_STATE, 0, 6, 2, 750_000_000, custom1());
        future.step_at(6, true).unwrap();
        future.step_at(7, true).unwrap();
        assert_eq!(commit_rolling(&future.of(TIMELINE_COMMIT).last().unwrap().data), 1);
        future.step_at(8, true).unwrap();
        assert_eq!(commit_rolling(&future.of(TIMELINE_COMMIT).last().unwrap().data), 0);

        // Newer than the stored stamp, but already past timeout at arrival.
        let mut expired = arm_rolling(&cfg, 0);
        expired.step_at(6, true).unwrap();
        expired.step_at(7, true).unwrap();
        expired.push_at(OWN_STATE, 0, 8, 2, 540_000_000, custom1());
        expired.step_at(8, true).unwrap();
        assert_eq!(commit_rolling(&expired.of(TIMELINE_COMMIT).last().unwrap().data), 0, "an already-expired envelope does not refresh age");

        // A newer in-window sample does move the produce time.
        let mut renewed = arm_rolling(&cfg, 0);
        renewed.step_at(6, true).unwrap();
        renewed.step_at(7, true).unwrap();
        renewed.push_at(OWN_STATE, 0, 8, 2, 600_000_000, custom1());
        renewed.step_at(8, true).unwrap();
        assert_eq!(commit_rolling(&renewed.of(TIMELINE_COMMIT).last().unwrap().data), 1, "600ms is 200ms old at k=8, inside 250ms");
        renewed.step_at(9, true).unwrap();
        assert_eq!(commit_rolling(&renewed.of(TIMELINE_COMMIT).last().unwrap().data), 0);

        let mut sourced = ordered_config("1");
        sourced.push_str("state_timeout = 0.25\ncontrol_origin = 4\n");
        let mut foreign = arm_rolling(&sourced, 4);
        foreign.push_at(OWN_STATE, 9, 6, 2, 600_000_000, custom1());
        foreign.step_at(6, true).unwrap();
        foreign.step_at(7, true).unwrap();
        assert_eq!(commit_rolling(&foreign.of(TIMELINE_COMMIT).last().unwrap().data), 1);
        foreign.step_at(8, true).unwrap();
        assert_eq!(commit_rolling(&foreign.of(TIMELINE_COMMIT).last().unwrap().data), 0, "a planner origin does not extend the board sample");

        let mut idle = arm_rolling(&sourced, 4);
        idle.push_at(OWN_STATE, 9, 6, 2, 600_000_000, named_state("Idle"));
        idle.step_at(6, true).unwrap();
        assert_eq!(commit_rolling(&idle.of(TIMELINE_COMMIT).last().unwrap().data), 1, "a foreign non-Custom1 does not revoke");
    }
}
