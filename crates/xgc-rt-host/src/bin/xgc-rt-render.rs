//! Trusted target-local launcher; it never discovers or changes a Session.
use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;
use std::process::ExitCode;
use xgc_rt_host::deployment::{self, Result};

fn execute() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let action = args
        .next()
        .ok_or("usage: xgc-rt-render describe|prepare|run [options]")?;
    if action == "describe" {
        let id = match args.next().as_deref() {
            None => deployment::COMPOSITION_ID.to_owned(),
            Some("--composition-id") => args.next().ok_or("--composition-id requires a value")?,
            Some(_) => return Err("describe accepts only --composition-id <id>".into()),
        };
        if args.next().is_some() {
            return Err("describe accepts exactly one composition selector".into());
        }
        let composition = deployment::composition(&id)?;
        println!(
            "{}",
            serde_json::json!({"schema_version":1,"composition_id":composition.id,"composition_sha256":composition.sha256(),"composition_bytes":composition.bytes,"platform":"linux-amd64","input_time_domain":"wall-unix","managed_launch":"run","live_readiness":false})
        );
        return Ok(());
    }
    if action != "prepare" && action != "run" {
        return Err("unknown command".into());
    }
    let mut options = BTreeMap::new();
    while let Some(key) = args.next() {
        if !["--bundle-root", "--state-root", "--deployment-json"].contains(&key.as_str()) {
            return Err(format!("unknown option {key}"));
        }
        let value = args.next().ok_or("option missing value")?;
        if options.insert(key, value).is_some() {
            return Err("duplicate option".into());
        }
    }
    let bundle_root = PathBuf::from(
        options
            .remove("--bundle-root")
            .ok_or("--bundle-root is required")?,
    );
    let raw = options
        .remove("--deployment-json")
        .ok_or("--deployment-json is required")?;
    let state_root = if action == "prepare" {
        PathBuf::from(
            options
                .remove("--state-root")
                .ok_or("prepare requires --state-root")?,
        )
    } else {
        if options.contains_key("--state-root") {
            return Err("run uses only the target-owned managed root".into());
        }
        deployment::managed_root(
            std::env::var("XGC_AGENT_MANAGED_ROOT").ok().as_deref(),
            std::env::var("XGC_CORE_MANAGED_ROOT").ok().as_deref(),
        )?
    };
    let prepared = deployment::prepare(&raw, &bundle_root, &state_root)?;
    println!(
        "{}",
        serde_json::to_string(&prepared.receipt).map_err(|e| e.to_string())?
    );
    std::io::stdout().flush().map_err(|e| e.to_string())?;
    if action == "prepare" {
        return Ok(());
    }
    // No shell; the PID/PGID owned by the ordinary HostDriver survives exec.
    // Prepared (including its inherited lock FD) remains alive until exec.
    prepared.exec_host()
}
fn main() -> ExitCode {
    match execute() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("xgc-rt-render: {error}");
            ExitCode::from(2)
        }
    }
}
