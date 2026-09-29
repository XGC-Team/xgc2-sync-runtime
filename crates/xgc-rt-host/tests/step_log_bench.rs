//! Cost of the step log for a graph the size of the 100-robot plant's ROS
//! edges: 200 modules stepping every 5 ms round, about 40,000 steps/s. It
//! measures, it does not assert, so it only runs when asked:
//!
//!   XGC_STEP_LOG_BENCH=1 cargo test --release -p xgc-rt-host \
//!       --test step_log_bench -- --nocapture
//!
//! `XGC_STEP_LOG_BENCH_EVERY` lists the `audit.steps_every` values (default
//! 1,200); `XGC_STEP_LOG_BENCH_SECONDS` is the run length (default 5).

mod common;

use std::fmt::Write as _;
use std::sync::atomic::AtomicBool;
use std::sync::Arc;

use xgc_rt_core::clock::WallClock;
use xgc_rt_core::manifest::Manifest;
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};

const MODULES: usize = 200;
const PERIOD_MS: u64 = 5;

/// User plus system CPU of this process, in clock ticks (USER_HZ = 100).
fn cpu_ticks() -> u64 {
    let stat = std::fs::read_to_string("/proc/self/stat").unwrap();
    let fields: Vec<&str> = stat.rsplit(')').next().unwrap().split_whitespace().collect();
    fields[11].parse::<u64>().unwrap() + fields[12].parse::<u64>().unwrap()
}

fn manifest(session: &str, every: u64, run_for_ms: u64) -> String {
    let mut text = format!(
        "[session]\nid = \"{session}\"\nnode = \"n1\"\nroster = [\"n1\"]\nperiod_ms = {PERIOD_MS}\nstart_delay_ms = 200\nrun_for_ms = {run_for_ms}\n\n\
         [transport]\nkind = \"loopback\"\n\n[audit]\ndir = \"audit\"\nsteps_every = {every}\n"
    );
    for index in 0..MODULES {
        let _ = write!(
            text,
            "\n[[channel]]\nname = \"out{index}\"\nqos = \"state\"\n\n[[plugin]]\nname = \"edge{index}\"\npath = \"{}\"\ntrigger = \"on_round\"\nstep_budget_ms = 50\nconfig = {{ payload_bytes = 64 }}\nbind = {{ detections = {{ channel = \"out{index}\" }} }}\n",
            common::lib("stub_perception"),
        );
    }
    text
}

#[test]
fn step_log_cost_at_the_plant_edge_bound() {
    if std::env::var("XGC_STEP_LOG_BENCH").is_err() {
        eprintln!("set XGC_STEP_LOG_BENCH=1 to run the step log benchmark");
        return;
    }
    let seconds: u64 = std::env::var("XGC_STEP_LOG_BENCH_SECONDS").map_or(5, |v| v.parse().unwrap());
    let everies: Vec<u64> = std::env::var("XGC_STEP_LOG_BENCH_EVERY")
        .unwrap_or_else(|_| "1,200".into())
        .split(',')
        .map(|v| v.trim().parse().unwrap())
        .collect();
    for every in everies {
        let dir = common::scratch(&format!("step-log-{every}"));
        let text = manifest(&format!("steplog{every}"), every, seconds * 1000);
        std::fs::write(dir.join("node.toml"), &text).unwrap();
        let host = Host::new(
            Manifest::from_toml_str(&text).unwrap(),
            &dir,
            Box::new(LoopbackTransport::new(LoopbackBus::new())),
            Arc::new(WallClock::new(0)),
            HostOptions::default(),
        )
        .unwrap();
        let before = cpu_ticks();
        let summary = host.run(&AtomicBool::new(false)).unwrap();
        let ticks = cpu_ticks() - before;
        let steps: u64 = summary.plugins.iter().map(|p| p.steps).sum();
        let bytes = std::fs::metadata(summary.audit_dir.join("n1/steps.jsonl")).unwrap().len();
        let run_s = seconds as f64 + 0.2;
        println!(
            "steps_every={every:>4}: {steps} steps ({:.0}/s), steps.jsonl {:.2} MB ({:.2} MB/s), process CPU {:.1}% of one core",
            steps as f64 / run_s,
            bytes as f64 / 1e6,
            bytes as f64 / 1e6 / run_s,
            ticks as f64 / run_s,
        );
    }
}
