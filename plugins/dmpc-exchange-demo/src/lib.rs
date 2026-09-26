//! DMPC-shaped exchange without the optimizer: every round a node publishes
//! a plan the size of the TRO `AssumedTrajectory` (9 × 51 f64) and records
//! which neighbor plans it would have solved with.
//!
//! Ports: plan_in / plan_out (control), completeness (state). The
//! completeness payload is `round u64, count u32, 0 u32`, then per neighbor
//! `origin u16, status u8 (0 fresh, 1 stale, 2 missing), 0 u8, stale_n u32,
//! plan_round u64` (u64::MAX when never seen).

use xgc_rt_abi::neighbor::{NeighborExchange, NeighborStatus};
use xgc_rt_abi::*;

const PLAN_IN: u32 = 0;
const PLAN_OUT: u32 = 1;
const COMPLETENESS: u32 = 2;
const PLAN_BYTES: usize = 9 * 51 * 8;

pub struct Demo {
    host: Host,
    nx: Option<NeighborExchange>,
    last_completeness: f64,
}

impl Plugin for Demo {
    fn create(host: Host) -> Self {
        Self { host, nx: None, last_completeness: 1.0 }
    }

    fn activate(&mut self) -> Result<(), String> {
        self.nx = Some(NeighborExchange::new(&self.host, PLAN_IN, PLAN_OUT, 2));
        Ok(())
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        let nx = self.nx.as_mut().ok_or("not active")?;
        nx.absorb(&mut self.host, ctx.round);
        if ctx.round_advanced == 0 {
            return Ok(());
        }
        let snap = nx.snapshot(ctx.round, ctx.now);
        self.last_completeness = snap.completeness();
        let mut out = Vec::with_capacity(16 + 16 * snap.neighbors.len());
        out.extend_from_slice(&ctx.round.to_le_bytes());
        out.extend_from_slice(&(snap.neighbors.len() as u32).to_le_bytes());
        out.extend_from_slice(&0u32.to_le_bytes());
        for n in &snap.neighbors {
            let (code, stale) = match n.status {
                NeighborStatus::Fresh => (0u8, 0u32),
                NeighborStatus::Stale(k) => (1, k as u32),
                NeighborStatus::Missing => (2, 0),
            };
            out.extend_from_slice(&n.origin.to_le_bytes());
            out.push(code);
            out.push(0);
            out.extend_from_slice(&stale.to_le_bytes());
            out.extend_from_slice(&n.round.unwrap_or(u64::MAX).to_le_bytes());
        }
        self.host.publish(COMPLETENESS, ctx.round, &out).map_err(|s| format!("publish completeness: {s}"))?;
        let mut plan = vec![0u8; PLAN_BYTES];
        plan[..8].copy_from_slice(&ctx.round.to_le_bytes());
        nx.publish(&self.host, ctx.round, &plan).map_err(|s| format!("publish plan: {s}"))
    }

    fn domain_state(&self) -> &'static std::ffi::CStr {
        if self.last_completeness >= 1.0 {
            cstr!("complete")
        } else {
            cstr!("partial")
        }
    }
}

export_plugin! {
    plugin: Demo,
    name: "dmpc-exchange-demo",
    version: "0.1.0",
    ports: [
        ("plan_in", XGC_PORT_IN, "xgc.test.assumed_trajectory/1", XGC_QOS_CONTROL),
        ("plan_out", XGC_PORT_OUT, "xgc.test.assumed_trajectory/1", XGC_QOS_CONTROL),
        ("completeness", XGC_PORT_OUT, "xgc.test.round_completeness/1", XGC_QOS_STATE),
    ],
}
