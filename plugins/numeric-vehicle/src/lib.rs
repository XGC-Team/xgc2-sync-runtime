//! Actuator-free numerical vehicle for planning/wireless HIL. No ROS/FCU access.
pub mod model;
use model::{Config, Model, Segment, State};
use xgc_rt_abi::*;

pub struct NumericVehicle {
    host: Host,
    model: Option<Model>,
}
impl Plugin for NumericVehicle {
    fn create(host: Host) -> Self {
        Self { host, model: None }
    }
    fn configure(&mut self, text: &str) -> Result<(), String> {
        let config: Config =
            toml::from_str(text).map_err(|e| format!("numeric-vehicle config: {e}"))?;
        self.model = Some(Model::new(config)?);
        Ok(())
    }
    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        let model = self.model.as_mut().ok_or("numeric-vehicle unconfigured")?;
        model.advance(ctx.now)?;
        while let Some(sample) = self.host.next(0) {
            let bytes = sample.data.to_vec();
            let result = command(&bytes).and_then(|text| model.command(text, ctx.now));
            if let Err(reason) = result {
                self.host
                    .log(XGC_LOG_WARN, &format!("numeric-vehicle command: {reason}"));
            }
        }
        while let Some(sample) = self.host.next(1) {
            let arrival = sample.t_rx;
            let segment = Segment::decode(sample.data);
            let effective = segment.as_ref().ok().map(|s| s.start);
            let result = segment.and_then(|s| model.segment(s, ctx.now));
            let outcome = match result {
                Ok(()) => "accepted".to_string(),
                Err(e) => e,
            };
            self.host.log(XGC_LOG_INFO,&format!("numeric-vehicle segment effective_ns={effective:?} arrival_ns={arrival} apply_now_ns={} age_ns={:?} outcome={outcome}",ctx.now,effective.map(|t|ctx.now-t)));
        }
        let mut paired = Vec::with_capacity(96);
        let stamp = ctx.now as f64 * 1e-9;
        for v in [stamp, stamp]
            .into_iter()
            .chain(model.position)
            .chain([0.0, 0.0, 0.0, 1.0])
            .chain(model.velocity)
        {
            paired.extend_from_slice(&v.to_le_bytes());
        }
        let mut status = [0u8; 56];
        status[..8].copy_from_slice(&stamp.to_le_bytes());
        let name = model.state.name().as_bytes();
        status[8..8 + name.len()].copy_from_slice(name);
        self.host
            .publish(2, ctx.round, &paired)
            .map_err(|e| format!("paired state publish: {e}"))?;
        self.host
            .publish(3, ctx.round, &status)
            .map_err(|e| format!("status publish: {e}"))?;
        Ok(())
    }
    fn domain_state(&self) -> &'static std::ffi::CStr {
        match self.model.as_ref().map(|m| m.state) {
            None => cstr!("unconfigured"),
            Some(State::Configured) => cstr!("Configured"),
            Some(State::Ready) => cstr!("Ready"),
            Some(State::Custom1) => cstr!("Custom1"),
            Some(State::Hold) => cstr!("Hold"),
            Some(State::Stopped) => cstr!("Stopped"),
            Some(State::Fault) => cstr!("Fault"),
        }
    }
}
fn command(bytes: &[u8]) -> Result<&str, String> {
    if bytes.len() != 64 {
        return Err("command must be 64 bytes".into());
    }
    let end = bytes
        .iter()
        .position(|b| *b == 0)
        .ok_or("unterminated command")?;
    if bytes[end..].iter().any(|b| *b != 0) {
        return Err("noncanonical command padding".into());
    }
    std::str::from_utf8(&bytes[..end]).map_err(|_| "command is not UTF-8".into())
}
export_plugin! {
    plugin:NumericVehicle,name:"numeric-vehicle",version:"0.1.0",
    ports:[
        ("command",XGC_PORT_IN,"xgc.command/1",XGC_QOS_EVENT),
        ("position_target",XGC_PORT_IN,"xgc.position_target/1",XGC_QOS_CONTROL),
        ("paired_state",XGC_PORT_OUT,"xgc.dmpc.paired_state/1",XGC_QOS_STATE),
        ("controller_state",XGC_PORT_OUT,"xgc.controller_status/1",XGC_QOS_STATE),
    ],
}
