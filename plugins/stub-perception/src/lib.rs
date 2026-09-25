//! Perception stub: publishes one detections sample per round. Its domain
//! FSM goes from `searching` to `tracking` after 3 rounds. Config:
//! `payload_bytes` (default 64, minimum 16).

use xgc_rt_abi::*;

pub struct Perception {
    host: Host,
    payload_bytes: usize,
    rounds: u64,
}

const DETECTIONS: u32 = 0;

impl Plugin for Perception {
    fn create(host: Host) -> Self {
        Self { host, payload_bytes: 64, rounds: 0 }
    }

    fn configure(&mut self, config: &str) -> Result<(), String> {
        let table: toml::Table = config.parse().map_err(|e| format!("config: {e}"))?;
        if let Some(v) = table.get("payload_bytes") {
            let n = v.as_integer().ok_or("payload_bytes must be an integer")?;
            self.payload_bytes = usize::try_from(n).map_err(|_| "payload_bytes must be positive")?.max(16);
        }
        Ok(())
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        if ctx.round_advanced == 0 {
            return Ok(());
        }
        self.rounds += 1;
        let mut payload = vec![0u8; self.payload_bytes];
        payload[0..8].copy_from_slice(&ctx.round.to_le_bytes());
        payload[8..16].copy_from_slice(&(ctx.round % 5).to_le_bytes()); // "objects seen"
        self.host.publish(DETECTIONS, ctx.round, &payload).map_err(|s| format!("publish: {s}"))
    }

    fn domain_state(&self) -> &'static std::ffi::CStr {
        if self.rounds < 3 {
            cstr!("searching")
        } else {
            cstr!("tracking")
        }
    }
}

export_plugin! {
    plugin: Perception,
    name: "stub-perception",
    version: "0.1.0",
    ports: [("detections", XGC_PORT_OUT, "xgc.stub.detections/1", XGC_QOS_STATE)],
}
