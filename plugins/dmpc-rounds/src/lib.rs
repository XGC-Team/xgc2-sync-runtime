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
//! `state_timeout` (1.0 s), `max_trigger_gap` (0.5 s), `duration` (0: none).
//!
//! Domain state: `waiting` until the node published its own plan, then
//! `complete` or `partial` (every neighbor fresh for the last round or not).

use std::collections::BTreeMap;

pub mod mission;

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
    (p.len() == PLAN_HEADER + doubles * 8).then(|| u(8))
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
}

fn config_error(key: &str) -> String {
    format!("dmpc-rounds: invalid config {key}")
}

impl DmpcRounds {
    /// Mission-phase inputs of this step: operator commands, this robot's
    /// controller state, and peers' states (with the phase they had).
    fn mission_inputs(&mut self) {
        if self.phase.is_none() {
            return;
        }
        let own_name = self.own_name();
        while let Some(s) = self.host.next(COMMAND) {
            self.phase.as_mut().unwrap().command(&text(s.data));
        }
        while let Some(s) = self.host.next(OWN_STATE) {
            // xgc.controller_status/1: stamp, then the state name.
            if s.data.len() == 56 {
                self.own_state = text(&s.data[8..]);
                self.phase.as_mut().unwrap().observe(&own_name, &self.own_state, s.t_produce as f64 * 1e-9);
            }
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
    fn mission_round(&mut self, ctx: &XgcStepCtx, trigger: &[u8]) -> Result<(), String> {
        let Some(phase) = self.phase.as_mut() else { return Ok(()) };
        let now = ctx.now as f64 * 1e-9;
        if let Some((rolling, mission_time)) = phase.tick(ctx.round, ctx.round_start as f64 * 1e-9, now, true) {
            self.rolling = rolling;
            self.history.insert(ctx.round, (rolling, mission_time));
            while self.history.len() > 64 {
                self.history.pop_first();
            }
            let tick = formation_tick(rolling, mission_time, trigger);
            self.host.publish(FORMATION_TICK, ctx.round, &tick).map_err(|s| format!("publish formation tick: {s}"))?;
            let state = mission_state(now, ctx.round, self.uav_id, rolling, mission_time, &self.own_state);
            self.host.publish(STATE_OUT, ctx.round, &state).map_err(|s| format!("publish mission state: {s}"))?;
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
        }
    }

    fn configure(&mut self, config: &str) -> Result<(), String> {
        let table: toml::Table = config.parse().map_err(|e| format!("dmpc-rounds config: {e}"))?;
        let int = |v: &toml::Value, key: &str| v.as_integer().and_then(|i| u32::try_from(i).ok()).ok_or_else(|| config_error(key));
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
            self.phase = Some(phase.with_limits(num("state_timeout", 1.0)?, num("max_trigger_gap", 0.5)?, num("duration", 0.0)?));
        }
        Ok(())
    }

    fn activate(&mut self) -> Result<(), String> {
        self.nx = Some(NeighborExchange::new(&self.host, PLAN_IN, PLAN_OUT, self.stale_rounds));
        Ok(())
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        self.mission_inputs();
        // The robot's own plan, as the node published it.
        let mut own = None;
        while let Some(s) = self.host.next(OWN_PLAN) {
            match plan_uav_id(s.data) {
                Some(id) if id == self.uav_id => own = Some(s.data.to_vec()),
                Some(_) => {} // a neighbor plan echoed back through ROS
                None => self.host.log(XGC_LOG_WARN, "dmpc-rounds: malformed own plan dropped"),
            }
        }
        // Neighbor plans: keep the newest, and pass each newer one to ROS once.
        let nx = self.nx.as_mut().ok_or("not active")?;
        let mut fresh = Vec::new();
        while let Some(s) = self.host.next(PLAN_IN) {
            nx.offer(s.origin, s.round, s.seq, s.t_produce, s.data);
            if self.forwarded.get(&s.origin).map_or(true, |&f| (s.round, s.seq) > f) {
                self.forwarded.insert(s.origin, (s.round, s.seq));
                fresh.push(s.data.to_vec());
            }
        }
        for plan in fresh {
            self.host.publish(NEIGHBOR_PLANS, ctx.round, &plan).map_err(|s| format!("publish neighbor plan: {s}"))?;
        }
        if let Some(plan) = own {
            nx.publish(&self.host, ctx.round, &plan).map_err(|s| format!("publish plan: {s}"))?;
            self.sent_own = true;
        }
        if ctx.round_advanced != 0 {
            let snap = nx.snapshot(ctx.round, ctx.now);
            self.complete = snap.neighbors.iter().all(|n| n.status == NeighborStatus::Fresh);
            let trigger = sync_trigger(ctx.round, ctx.round_start as f64 * 1e-9, ctx.now as f64 * 1e-9, &self.participant_ids);
            self.host.publish(SYNC_TRIGGER, ctx.round, &trigger).map_err(|s| format!("publish sync trigger: {s}"))?;
            self.mission_round(ctx, &trigger)?;
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
}
