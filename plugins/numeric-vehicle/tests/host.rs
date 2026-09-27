//! Actual host loads the actual cdylib plus a controlled C ABI producer/observer.
use std::{
    path::{Path, PathBuf},
    sync::{atomic::AtomicBool, Arc},
};
use xgc_rt_core::{clock::WallClock, manifest::Manifest};
use xgc_rt_host::{Host, HostOptions};
use xgc_rt_transport_loopback::{LoopbackBus, LoopbackTransport};
#[test]
fn real_host_consumes_header_timed_pva_and_emits_command_driven_state() {
    check_host(false);
}

#[test]
fn timed_model_keeps_pva_timing_without_publishing_on_every_host_round() {
    check_host(true);
}

fn check_host(timed: bool) {
    let plugin = PathBuf::from(
        std::env::var("NUMERIC_VEHICLE_ELF")
            .expect("run plugins/numeric-vehicle/test.sh; no optional native skip"),
    );
    assert!(plugin.is_file());
    let root = Path::new(env!("CARGO_MANIFEST_DIR"));
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("numeric-vehicle-host-{nonce}"));
    eprintln!("numeric-vehicle receipt_dir={}", dir.display());
    std::fs::create_dir_all(&dir).unwrap();
    let out = dir.join("observations.txt");
    let driver = dir.join("driver.so");
    assert!(std::process::Command::new("cc")
        .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC"])
        .arg(format!("-DOUTPUT=\"{}\"", out.display()))
        .arg("-I")
        .arg(root.join("../../abi/include"))
        .arg(root.join("tests/wire_driver.c"))
        .arg("-o")
        .arg(&driver)
        .status()
        .unwrap()
        .success());
    let mut text = r#"
[session]
id="numeric-hil-private"
node="n1"
roster=["n1"]
period_ms=1
start_delay_ms=30
run_for_ms=450
[transport]
kind="loopback"
[audit]
dir="audit"
[[channel]]
name="command"
qos="event"
[[channel]]
name="pva"
qos="control"
[[channel]]
name="paired"
qos="state"
[[channel]]
name="status"
qos="state"
"#
    .to_string();
    for (name, path, config, bind) in [
        (
            "numeric-vehicle",
            plugin,
            r#"{initial_position=[1.0,2.0,3.0],initial_velocity=[0.2,0.0,0.0]}"#,
            r#"{command={channel="command",from=["n1"]},position_target={channel="pva",from=["n1"]},paired_state={channel="paired"},controller_state={channel="status"}}"#,
        ),
        (
            "wire",
            driver,
            "{}",
            r#"{command={channel="command"},position_target={channel="pva"},paired_state={channel="paired",from=["n1"]},controller_state={channel="status",from=["n1"]}}"#,
        ),
    ] {
        let sha = xgc_rt_host::plugin::sha256_hex(&std::fs::read(&path).unwrap());
        let trigger = if timed && name == "numeric-vehicle" {
            "trigger=\"on_dirty\"\nwake_ms=10"
        } else {
            "trigger=\"on_round\""
        };
        text+=&format!("\n[[plugin]]\nname={name:?}\npath={:?}\nsha256={sha:?}\n{trigger}\nstep_budget_ms=10\nconfig={config}\nbind={bind}\n",path.to_str().unwrap());
    }
    std::fs::write(dir.join("node.toml"), &text).unwrap();
    let host = Host::new(
        Manifest::from_toml_str(&text).unwrap(),
        &dir,
        Box::new(LoopbackTransport::new(LoopbackBus::new())),
        Arc::new(WallClock::new(0)),
        HostOptions::default(),
    )
    .unwrap();
    let summary = host.run(&AtomicBool::new(false)).unwrap();
    std::fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary).unwrap(),
    )
    .unwrap();
    assert!(summary.aborted.is_none(), "{:?}", summary.aborted);
    for p in &summary.plugins {
        assert!(p.last_error.is_none() && p.state == "inactive", "{p:?}");
        if timed && p.name == "numeric-vehicle" {
            assert!((30..65).contains(&p.steps), "model steps: {}", p.steps);
        } else {
            assert!(p.steps > 100, "{} steps: {}", p.name, p.steps);
        }
    }
    let lines = std::fs::read_to_string(&out).unwrap();
    let mut positions = Vec::new();
    let mut states = Vec::new();
    let mut effective = 0.0;
    for line in lines.lines() {
        let fields: Vec<_> = line.split_whitespace().collect();
        match fields[0] {
            "P" => positions.push((
                fields[1].parse::<f64>().unwrap(),
                fields[2].parse::<f64>().unwrap(),
                fields[3].parse::<f64>().unwrap(),
            )),
            "S" => states.push((fields[1].parse::<f64>().unwrap(), fields[2])),
            "E" => effective = fields[1].parse().unwrap(),
            _ => panic!("bad C observation"),
        }
    }
    for state in ["Configured", "Ready", "Custom1", "Hold", "Stopped"] {
        assert!(states.iter().any(|s| s.1 == state), "missing {state}");
    }
    let ready = states.iter().find(|s| s.1 == "Ready").unwrap().0;
    let base = *positions
        .iter()
        .find(|p| p.0 >= ready && p.0 < effective)
        .unwrap();
    let stop = states.iter().find(|s| s.1 == "Stopped").unwrap().0;
    for &(t, p, v) in positions
        .iter()
        .filter(|p| p.0 >= base.0 && p.0 < stop - 0.005)
    {
        let active = (t - effective).clamp(0.0, 0.1);
        let coast = (t - effective - 0.1).max(0.0);
        let expected = base.1 + 0.2 * (t - base.0) + active * active + 0.2 * coast;
        assert!(
            (p - expected).abs() < 2e-6,
            "position {p} != {expected} at {t}"
        );
        assert!(
            (v - (0.2 + 2.0 * active)).abs() < 2e-6,
            "velocity {v} at {t}"
        );
    }
    let frozen: Vec<_> = positions.iter().filter(|p| p.0 >= stop + 0.005).collect();
    assert!(frozen.len() > if timed { 10 } else { 30 });
    assert!(frozen
        .iter()
        .all(|p| p.1 == frozen[0].1 && p.2 == frozen[0].2));
}
