//! `xgc-rt-audit merge <run-dir> [--out DIR] [--grace-ms N] [--window-ms N]`

use std::path::PathBuf;
use std::process::ExitCode;

use xgc_rt_audit::{merge_run, write_report, MergeOptions};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match run(&args) {
        Ok(valid) => {
            if valid {
                ExitCode::SUCCESS
            } else {
                ExitCode::from(3)
            }
        }
        Err(e) => {
            eprintln!("xgc-rt-audit: {e}");
            ExitCode::from(2)
        }
    }
}

fn run(args: &[String]) -> Result<bool, String> {
    let usage = "usage: xgc-rt-audit merge <run-dir> [--out DIR] [--grace-ms N] [--window-ms N]";
    let (Some("merge"), Some(run_dir)) = (args.first().map(String::as_str), args.get(1)) else {
        return Err(usage.into());
    };
    let run_dir = PathBuf::from(run_dir);
    let mut out = run_dir.join("merged");
    let mut opts = MergeOptions::default();
    let mut rest = args[2..].iter();
    while let Some(flag) = rest.next() {
        let value = rest.next().ok_or(usage)?;
        let ms = || value.parse::<f64>().map(|v| (v * 1e6) as i64).map_err(|_| format!("{flag}: {value:?} is not a number"));
        match flag.as_str() {
            "--out" => out = PathBuf::from(value),
            "--grace-ms" => opts.grace_ns = ms()?,
            "--window-ms" => opts.window_ns = ms()?.max(1),
            _ => return Err(usage.into()),
        }
    }
    let report = merge_run(&run_dir, opts).map_err(|e| e.to_string())?;
    write_report(&report, &out).map_err(|e| e.to_string())?;
    print!("{}", xgc_rt_audit::merge::markdown(&report));
    eprintln!("wrote {}", out.display());
    Ok(report.valid)
}
