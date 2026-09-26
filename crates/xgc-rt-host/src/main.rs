//! `xgc-rt-host --manifest FILE [--echo-health]`
//!
//! Runs one node's plugins until `session.run_for_ms` elapses or the process
//! gets SIGINT or SIGTERM, then prints the run summary as JSON. Exit codes:
//! 0 all plugins stopped cleanly, 1 a plugin ended in Error or hung modules
//! made the aggregator stop early, 2 startup failed.

use std::path::PathBuf;
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::Transport;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};
use xgc_rt_transport_zenoh::{ZenohOptions, ZenohTransport};

static STOP: AtomicBool = AtomicBool::new(false);

extern "C" fn on_signal(_: i32) {
    STOP.store(true, Ordering::Relaxed);
}

extern "C" {
    fn signal(signum: i32, handler: extern "C" fn(i32)) -> usize;
}

fn main() -> ExitCode {
    // SAFETY: the handler only stores to an atomic.
    unsafe {
        signal(2, on_signal);
        signal(15, on_signal);
    }
    match run() {
        Ok(clean) => ExitCode::from(if clean { 0 } else { 1 }),
        Err(e) => {
            eprintln!("xgc-rt-host: {e}");
            ExitCode::from(2)
        }
    }
}

fn run() -> Result<bool, String> {
    let mut manifest_path = None;
    let mut opts = HostOptions::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => manifest_path = args.next().map(PathBuf::from),
            "--echo-health" => opts.echo_health = true,
            _ => return Err("usage: xgc-rt-host --manifest FILE [--echo-health]".into()),
        }
    }
    let path = manifest_path.ok_or("--manifest is required")?;
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let manifest = Manifest::from_toml_str(&text).map_err(|e| e.to_string())?;
    let base = path.parent().map(PathBuf::from).unwrap_or_default();
    let transport: Box<dyn Transport> = match manifest.transport.kind.as_str() {
        // A single-process bus: every roster node on it must run in this
        // process.
        "loopback" => Box::new(LoopbackTransport::new(LoopbackBus::new())),
        "zenoh" => Box::new(ZenohTransport::new(ZenohOptions::from_table(&manifest.transport.options).map_err(|e| e.0)?)),
        other => return Err(format!("transport kind {other:?} is not available in this build")),
    };
    // One host clock, so the bound is 0 on loopback. Z2 sets it from chrony
    // and the probe.
    let clock = Arc::new(WallClock::new(0));
    let host = Host::new(manifest, &base, transport, clock, opts).map_err(|e| e.to_string())?;
    let summary = host.run(&STOP).map_err(|e| e.to_string())?;
    println!("{}", serde_json::to_string_pretty(&summary).map_err(|e| e.to_string())?);
    Ok(summary.aborted.is_none() && summary.plugins.iter().all(|p| p.state != "error" && p.last_error.is_none()))
}
