//! `xgc-rt-host --manifest FILE [--echo-health]`
//!
//! Runs one node's plugins until `session.run_for_ms` elapses or the process
//! gets SIGINT or SIGTERM, then prints the run summary as JSON. Exit codes:
//! 0 all plugins stopped cleanly, 1 a plugin ended in Error or hung modules
//! made the aggregator stop early, 2 startup failed.

mod control;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::atomic::{AtomicBool, Ordering};

use xgc_rt_core::manifest::Manifest;
use xgc_rt_core::transport::Transport;
use xgc_rt_host::transport_so::SoTransport;
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
    let mut control_socket = None;
    let mut module_root = None;
    let mut audit_root = None;
    let mut document_root = None;
    let mut opts = HostOptions::default();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--manifest" => {
                manifest_path = Some(PathBuf::from(
                    args.next().ok_or("--manifest requires a value")?,
                ))
            }
            "--echo-health" => opts.echo_health = true,
            "--control-socket" => {
                control_socket = Some(PathBuf::from(
                    args.next().ok_or("--control-socket requires a value")?,
                ))
            }
            "--module-root" => {
                module_root = Some(PathBuf::from(
                    args.next().ok_or("--module-root requires a value")?,
                ))
            }
            "--audit-root" => {
                audit_root = Some(PathBuf::from(
                    args.next().ok_or("--audit-root requires a value")?,
                ))
            }
            "--document-root" => {
                document_root = Some(PathBuf::from(
                    args.next().ok_or("--document-root requires a value")?,
                ))
            }
            _ => return Err("usage: xgc-rt-host --manifest FILE [--echo-health]".into()),
        }
    }
    let policy = control::startup_policy()?;
    let instance_id = xgc2_xrpc::new_instance_id().map_err(|e| e.to_string())?;
    if let Some(socket) = control_socket {
        manifest_path = manifest_path
            .map(|path| {
                if path.is_absolute() {
                    Ok(path)
                } else {
                    std::env::current_dir().map(|base| base.join(path))
                }
            })
            .transpose()
            .map_err(|e| e.to_string())?;
        // An initial manifest is a bootstrap document from the process owner.
        // Administration-only launches must receive explicit directory grants.
        if let Some(path) = manifest_path.as_ref() {
            let path = path.to_owned();
            let base = path.parent().ok_or("manifest parent missing")?.to_owned();
            let text = control::read_document(&path)?;
            let manifest = Manifest::from_toml_str(&text).map_err(|e| e.to_string())?;
            if module_root.is_none() {
                module_root = Some(base.clone());
            }
            if document_root.is_none() {
                document_root = Some(base.clone());
            }
            if audit_root.is_none() {
                // The trusted bootstrap grants its declared audit location.
                let path = base.join(&manifest.audit.dir);
                std::fs::create_dir_all(&path).map_err(|e| e.to_string())?;
                audit_root = Some(path);
            }
        }
        let grants = control::Grants::new(
            module_root.ok_or("--module-root required without bootstrap manifest")?,
            audit_root.ok_or("--audit-root required without bootstrap manifest")?,
            document_root.ok_or("--document-root required without bootstrap manifest")?,
        )?;
        return control::serve(
            &socket,
            instance_id,
            manifest_path,
            grants,
            policy,
            opts.echo_health,
            &STOP,
        );
    }
    let path = manifest_path.ok_or("--manifest or --control-socket is required")?;
    let path = if path.is_absolute() {
        path
    } else {
        std::env::current_dir()
            .map_err(|e| e.to_string())?
            .join(path)
    };
    let text = control::read_document(&path)?;
    let manifest = Manifest::from_toml_str(&text).map_err(|e| e.to_string())?;
    let base = path.parent().map(PathBuf::from).unwrap_or_default();
    let limits = xgc2_xrpc::Limits::from_policy(&policy).map_err(|e| e.to_string())?;
    let shutdown_timeout = limits.shutdown_timeout;
    let mut rpc_runtime = xgc2_xrpc::Runtime::new(
        xgc2_xrpc::RuntimeOptions::from_policy(&policy).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    opts.rpc = Some(
        xgc_rt_host::rpc::RpcBinding::new(rpc_runtime.handle(), limits)
            .map_err(|e| e.to_string())?,
    );
    let host = build_host(manifest, &base, opts)?;
    let result = host.run(&STOP).map_err(|e| e.to_string());
    let closed = rpc_runtime
        .close(shutdown_timeout)
        .map_err(|e| e.to_string());
    let summary = result?;
    println!(
        "{}",
        serde_json::to_string_pretty(&summary).map_err(|e| e.to_string())?
    );
    closed?;
    Ok(summary.aborted.is_none()
        && summary.audit.complete()
        && summary
            .plugins
            .iter()
            .all(|p| p.state != "error" && p.last_error.is_none()))
}

fn build_host(manifest: Manifest, base: &Path, opts: HostOptions) -> Result<Host, String> {
    let spec = &manifest.transport;
    let transport: Box<dyn Transport> = match (&spec.path, spec.kind.as_str()) {
        (Some(path), kind) => Box::new(
            SoTransport::load(
                &base.join(path),
                spec.sha256.as_deref(),
                kind,
                &spec.options,
            )
            .map_err(|e| e.0)?,
        ),
        (None, "loopback") => Box::new(LoopbackTransport::new(LoopbackBus::new())),
        (None, "zenoh") => Box::new(ZenohTransport::new(
            ZenohOptions::from_table(&spec.options).map_err(|e| e.0)?,
        )),
        (None, other) => {
            return Err(format!(
                "transport kind {other:?} is not built in; name its plugin"
            ))
        }
    };
    Host::with_manifest_clock(manifest, base, transport, opts).map_err(|e| e.to_string())
}
