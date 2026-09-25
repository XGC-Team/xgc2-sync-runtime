//! Planning stub: once per round, publishes a plan from the latest state.
//! When no state arrives for more than `stale_rounds` rounds it asks the
//! host to degrade it, and asks to recover when state returns.
//! Domain FSM: `idle` → `planning` ⇄ `stale`.

use xgc_rt_abi::*;

#[derive(Clone, Copy, PartialEq)]
enum Domain {
    Idle,
    Planning,
    Stale,
}

pub struct Planning {
    host: Host,
    domain: Domain,
    last_state_round: Option<u64>,
    stale_rounds: u64,
}

const STATE: u32 = 0;
const PLAN: u32 = 1;

impl Plugin for Planning {
    fn create(host: Host) -> Self {
        Self { host, domain: Domain::Idle, last_state_round: None, stale_rounds: 3 }
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        let mut fresh = false;
        while self.host.next(STATE).is_some() {
            fresh = true;
        }
        if fresh {
            self.last_state_round = Some(ctx.round);
            if self.domain == Domain::Stale {
                self.host.request_recover();
            }
            self.domain = Domain::Planning;
        }
        let Some(last) = self.last_state_round else {
            return Ok(());
        };
        if ctx.round.saturating_sub(last) > self.stale_rounds && self.domain != Domain::Stale {
            self.domain = Domain::Stale;
            self.host.request_degrade("state is stale");
        }
        if ctx.round_advanced != 0 && self.domain == Domain::Planning {
            let mut payload = [0u8; 16];
            payload[0..8].copy_from_slice(&ctx.round.to_le_bytes());
            payload[8..16].copy_from_slice(&last.to_le_bytes());
            self.host.publish(PLAN, ctx.round, &payload).map_err(|s| format!("publish: {s}"))?;
        }
        Ok(())
    }

    fn domain_state(&self) -> &'static std::ffi::CStr {
        match self.domain {
            Domain::Idle => cstr!("idle"),
            Domain::Planning => cstr!("planning"),
            Domain::Stale => cstr!("stale"),
        }
    }
}

export_plugin! {
    plugin: Planning,
    name: "stub-planning",
    version: "0.1.0",
    ports: [
        ("state", XGC_PORT_IN, "xgc.stub.state/1", XGC_QOS_STATE),
        ("plan", XGC_PORT_OUT, "xgc.stub.plan/1", XGC_QOS_CONTROL),
    ],
}
