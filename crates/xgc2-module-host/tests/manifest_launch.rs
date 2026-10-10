//! Manifests with real libraries: starting an entity, and every way a manifest can be wrong
//! once the libraries are known.

mod common;

use common::*;
use std::path::Path;
use std::time::Duration;
use xgc2_module_host::host::ModuleHost;
use xgc2_module_host::launch;
use xgc2_module_host::manifest::Manifest;

fn parse(text: &str) -> Manifest {
    Manifest::parse(text, Path::new("/")).unwrap_or_else(|e| panic!("{e}"))
}

fn lib(name: &str) -> String {
    module(name).to_str().unwrap().to_owned()
}

fn start(text: &str) -> Result<ModuleHost, String> {
    launch::launch(&parse(text)).map_err(|e| e.to_string())
}

struct Running(ModuleHost);

impl Drop for Running {
    fn drop(&mut self) {
        self.0.shutdown();
    }
}

#[test]
fn a_manifest_brings_up_the_entity() {
    let text = format!(
        r#"
entity = "scout1"
[host]
workers = 2
[[module]]
name = "producer"
path = "{producer}"
sha256 = "{producer_sha}"
[[module]]
name = "stage"
path = "{stage}"
[[module]]
name = "consumer"
path = "{consumer}"
[[channel]]
name = "raw"
max_readers = 3
[[instance]]
name = "sink"
module = "consumer"
[instance.bind]
state_in = "cooked"
[[instance]]
name = "pass"
module = "stage"
[instance.bind]
in = "raw"
out = "cooked"
[[instance]]
name = "src"
module = "producer"
period_ms = 2.0
[instance.config]
id = 9
burst = 1
[instance.bind]
out = "raw"
"#,
        producer = lib("producer_state"),
        producer_sha = xgc2_module_host::loader::sha256_hex(&std::fs::read(module("producer_state")).unwrap()),
        stage = lib("passthrough"),
        consumer = lib("consumer"),
    );
    let host = Running(start(&text).expect("manifest starts"));
    wait_until("ready", Duration::from_secs(5), || host.0.is_ready());
    wait_until("data crosses both hops", Duration::from_secs(5), || count(&detail(&host.0, "sink")["state_updates"]) > 10);
    // The TOML config table reached the producer as JSON.
    assert_eq!(detail(&host.0, "sink")["last_producer"], 9);
    assert_eq!(channel(&host.0, "raw")["max_readers"], 3);
    let (ready, facts) = host.0.describe();
    assert!(ready && facts["entity"] == "scout1", "{facts}");
    // Instances are listed in the manifest's order.
    let names: Vec<String> =
        host.0.health()["instances"].as_array().unwrap().iter().map(|i| i["name"].as_str().unwrap().to_owned()).collect();
    assert_eq!(names, ["sink", "pass", "src"]);
}

#[test]
fn problems_that_need_the_libraries_stop_the_start() {
    let consumer = lib("consumer");
    let producer = lib("producer_state");
    let events = lib("producer_event");
    let manifest = |extra: &str| {
        format!(
            "entity = \"e\"\n[[module]]\nname = \"c\"\npath = \"{consumer}\"\n[[module]]\nname = \"p\"\npath = \"{producer}\"\n[[module]]\nname = \"ev\"\npath = \"{events}\"\n{extra}"
        )
    };
    let cases: Vec<(String, &str)> = vec![
        (manifest("[[instance]]\nname = \"a\"\nmodule = \"c\"\n[instance.bind]\nnope = \"x\"\n"), "module test_consumer has no port nope"),
        (manifest("[[instance]]\nname = \"a\"\nmodule = \"p\"\n[instance.bind]\nout = \"s\"\n[[instance]]\nname = \"b\"\nmodule = \"p\"\n[instance.bind]\nout = \"s\"\n"), "already has a writer"),
        (manifest("[[instance]]\nname = \"a\"\nmodule = \"p\"\n[instance.bind]\nout = \"s\"\n[[instance]]\nname = \"b\"\nmodule = \"c\"\n[instance.bind]\nevent_in = \"s\"\n"), "state"),
        (manifest("[[instance]]\nname = \"a\"\nmodule = \"ev\"\n[instance.bind]\nout = \"s\"\n[[instance]]\nname = \"b\"\nmodule = \"c\"\n[instance.bind]\nstate_in = \"s\"\n"), "event"),
        (manifest("[[channel]]\nname = \"s\"\ndepth = 4\n[[instance]]\nname = \"a\"\nmodule = \"ev\"\n[instance.bind]\nout = \"s\"\n"), "below the 16"),
        (
            format!("entity = \"e\"\n[[module]]\nname = \"c\"\npath = \"{consumer}\"\nsha256 = \"{}\"\n", "ab".repeat(32)),
            "does not match the pinned",
        ),
        ("entity = \"e\"\n[[module]]\nname = \"c\"\npath = \"/nonexistent/libx.so\"\n".to_owned(), "libx.so"),
        (format!("entity = \"e\"\n[[module]]\nname = \"c\"\npath = \"{}\"\n", repo_root().join("README.md").display()), "README.md"),
    ];
    for (text, expected) in cases {
        let error = start(&text).err().unwrap_or_else(|| panic!("expected a failure containing {expected:?} for\n{text}"));
        assert!(error.contains(expected), "expected {expected:?} in {error:?}");
    }
}

#[test]
fn check_reports_every_problem_and_starts_nothing() {
    let consumer = lib("consumer");
    let producer = lib("producer_state");
    let text = format!(
        r#"
entity = "e"
[[module]]
name = "c"
path = "{consumer}"
[[module]]
name = "p"
path = "{producer}"
[[module]]
name = "gone"
path = "/nonexistent/libgone.so"
[[instance]]
name = "a"
module = "p"
[instance.bind]
out = "s"
[[instance]]
name = "b"
module = "p"
[instance.bind]
out = "s"
[[instance]]
name = "c1"
module = "c"
[instance.bind]
bogus = "s"
event_in = "s"
"#
    );
    let problems = launch::check(&parse(&text)).unwrap_err().problems.join("\n");
    for expected in ["libgone.so", "already has a writer", "no port bogus"] {
        assert!(problems.contains(expected), "missing {expected:?} in\n{problems}");
    }
    let good = format!(
        "entity = \"e\"\n[[module]]\nname = \"c\"\npath = \"{consumer}\"\n[[module]]\nname = \"p\"\npath = \"{producer}\"\n[[module]]\nname = \"s\"\npath = \"{}\"\n[[instance]]\nname = \"a\"\nmodule = \"p\"\n[instance.bind]\nout = \"x\"\n[[instance]]\nname = \"b\"\nmodule = \"c\"\n[instance.bind]\nstate_in = \"x\"\n[[instance]]\nname = \"lonely\"\nmodule = \"s\"\n",
        lib("passthrough")
    );
    let report = launch::check(&parse(&good)).expect("a good manifest checks");
    assert_eq!(report["instances"], serde_json::json!(["a", "b", "lonely"]));
    let channels = report["channels"].as_array().unwrap();
    assert!(channels.iter().any(|c| c["name"] == "x" && c["writers"] == 1 && c["readers"] == 1), "{report}");
    assert_eq!(report["warnings"], serde_json::json!(["required input lonely.in has no producer in this manifest"]));
}

#[test]
fn autostart_false_creates_but_does_not_run() {
    let text = format!(
        "entity = \"e\"\n[[module]]\nname = \"p\"\npath = \"{}\"\n[[instance]]\nname = \"idle\"\nmodule = \"p\"\nperiod_ms = 1\nautostart = false\nrequired = false\n",
        lib("producer_state")
    );
    let host = Running(start(&text).unwrap());
    sleep_ms(50);
    let idle = instance(&host.0, "idle");
    assert_eq!((idle["state"].as_str(), idle["steps"].as_u64()), (Some("created"), Some(0)));
    assert!(host.0.is_ready(), "an unstarted optional instance does not block readiness");
    host.0.start_instance("idle").unwrap();
    wait_until("steps", Duration::from_secs(5), || count(&instance(&host.0, "idle")["steps"]) > 3);
}

#[test]
fn the_external_clock_channel_comes_from_the_manifest() {
    let text = format!(
        "entity = \"sim\"\n[clock]\nmode = \"external\"\nchannel = \"clock\"\n[[module]]\nname = \"clock\"\npath = \"{}\"\n[[module]]\nname = \"p\"\npath = \"{}\"\n[[instance]]\nname = \"sim\"\nmodule = \"clock\"\n[instance.config]\nstep_ms = 20\ninterval_us = 500\ncount = 25\n[instance.bind]\ntime = \"clock\"\n[[instance]]\nname = \"tick\"\nmodule = \"p\"\nperiod_ms = 100\n",
        lib("sim_clock"),
        lib("producer_state")
    );
    let host = Running(start(&text).unwrap());
    wait_until("simulation done", Duration::from_secs(5), || host.0.describe().1["clock"]["now_ns"] == 500_000_000);
    sleep_ms(30);
    // Samples every 20 ms from 20 to 500; the first anchors the timer at 120, then 220, 320, 420.
    assert_eq!(count(&instance(&host.0, "tick")["timer_fires"]), 4);
    assert_eq!(channel(&host.0, "clock")["writers"], 1);
}
