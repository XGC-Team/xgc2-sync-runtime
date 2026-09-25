//! `xgc-rt-impair --listen ADDR --target ADDR --profile FILE.toml [--truth OUT.jsonl] [--seconds N]`

use std::process::ExitCode;

use xgc_rt_impair::{Profile, Relay};

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let get = |flag: &str| args.iter().position(|a| a == flag).and_then(|i| args.get(i + 1)).cloned();
    let (Some(listen), Some(target), Some(profile)) = (get("--listen"), get("--target"), get("--profile")) else {
        eprintln!("usage: xgc-rt-impair --listen ADDR --target ADDR --profile FILE.toml [--truth OUT.jsonl] [--seconds N]");
        return ExitCode::from(2);
    };
    let run = || -> Result<(), String> {
        let profile: Profile = toml::from_str(&std::fs::read_to_string(&profile).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
        let relay = Relay::start(listen.parse().map_err(|e| format!("{e}"))?, target.parse().map_err(|e| format!("{e}"))?, profile)
            .map_err(|e| e.to_string())?;
        eprintln!("relaying {} -> {target}", relay.listen);
        let seconds: u64 = get("--seconds").and_then(|s| s.parse().ok()).unwrap_or(u64::MAX);
        std::thread::sleep(std::time::Duration::from_secs(seconds.min(365 * 24 * 3600)));
        let truth = relay.stop();
        if let Some(path) = get("--truth") {
            let lines: String = truth.iter().map(|r| serde_json::to_string(r).unwrap() + "\n").collect();
            std::fs::write(path, lines).map_err(|e| e.to_string())?;
        }
        Ok(())
    };
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("xgc-rt-impair: {e}");
            ExitCode::from(2)
        }
    }
}
