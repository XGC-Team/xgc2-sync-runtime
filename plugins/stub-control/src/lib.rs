//! Control stub: turns each new plan into one command.
//! Domain FSM: `hold` until the first plan, then `track`.

use xgc_rt_abi::*;

pub struct Control {
    host: Host,
    plans: u64,
}

const PLAN: u32 = 0;
const CMD: u32 = 1;

impl Plugin for Control {
    fn create(host: Host) -> Self {
        Self { host, plans: 0 }
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        while let Some(plan) = self.host.next(PLAN) {
            self.plans += 1;
            let mut payload = [0u8; 16];
            payload[0..8].copy_from_slice(&plan.round.to_le_bytes());
            payload[8..16].copy_from_slice(&self.plans.to_le_bytes());
            self.host.publish(CMD, ctx.round, &payload).map_err(|s| format!("publish: {s}"))?;
        }
        Ok(())
    }

    fn domain_state(&self) -> &'static std::ffi::CStr {
        if self.plans == 0 {
            cstr!("hold")
        } else {
            cstr!("track")
        }
    }
}

export_plugin! {
    plugin: Control,
    name: "stub-control",
    version: "0.1.0",
    ports: [
        ("plan", XGC_PORT_IN, "xgc.stub.plan/1", XGC_QOS_CONTROL),
        ("cmd", XGC_PORT_OUT, "xgc.stub.cmd/1", XGC_QOS_EVENT),
    ],
}
