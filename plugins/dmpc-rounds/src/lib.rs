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
//! Payloads: `xgc.dmpc.assumed_trajectory/1` and `xgc.dmpc.sync_trigger/1`
//! (abi/include/xgc_schemas_v1.h). Config: `uav_id` (this robot's id in the
//! messages), `participant_ids` (the trigger's active ids), `stale_rounds`
//! (how many rounds a neighbor plan may lag before it counts as missing,
//! default 2).
//!
//! Domain state: `waiting` until the node published its own plan, then
//! `complete` or `partial` (every neighbor fresh for the last round or not).

use std::collections::BTreeMap;

use xgc_rt_abi::neighbor::{NeighborExchange, NeighborStatus};
use xgc_rt_abi::*;

const OWN_PLAN: u32 = 0;
const PLAN_IN: u32 = 1;
const PLAN_OUT: u32 = 2;
const NEIGHBOR_PLANS: u32 = 3;
const SYNC_TRIGGER: u32 = 4;

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
}

fn config_error(key: &str) -> String {
    format!("dmpc-rounds: invalid config {key}")
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
        Ok(())
    }

    fn activate(&mut self) -> Result<(), String> {
        self.nx = Some(NeighborExchange::new(&self.host, PLAN_IN, PLAN_OUT, self.stale_rounds));
        Ok(())
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
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
