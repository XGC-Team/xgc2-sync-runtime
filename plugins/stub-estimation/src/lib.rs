//! Estimation stub: on every new batch of detections, publishes one state
//! sample. Its domain FSM goes from `initializing` to `converged` after 5
//! inputs.

use xgc_rt_abi::*;

pub struct Estimation {
    host: Host,
    inputs: u64,
}

const DETECTIONS: u32 = 0;
const STATE: u32 = 1;

impl Plugin for Estimation {
    fn create(host: Host) -> Self {
        Self { host, inputs: 0 }
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        let mut latest = None;
        while let Some(sample) = self.host.next(DETECTIONS) {
            self.inputs += 1;
            latest = Some(sample.round);
        }
        let Some(source_round) = latest else {
            return Ok(());
        };
        let mut payload = [0u8; 24];
        payload[0..8].copy_from_slice(&source_round.to_le_bytes());
        payload[8..16].copy_from_slice(&self.inputs.to_le_bytes());
        payload[16..24].copy_from_slice(&ctx.now.to_le_bytes());
        self.host.publish(STATE, ctx.round, &payload).map_err(|s| format!("publish: {s}"))
    }

    fn domain_state(&self) -> &'static std::ffi::CStr {
        if self.inputs < 5 {
            cstr!("initializing")
        } else {
            cstr!("converged")
        }
    }
}

export_plugin! {
    plugin: Estimation,
    name: "stub-estimation",
    version: "0.1.0",
    ports: [
        ("detections", XGC_PORT_IN, "xgc.stub.detections/1", XGC_QOS_STATE),
        ("state", XGC_PORT_OUT, "xgc.stub.state/1", XGC_QOS_STATE),
    ],
}
