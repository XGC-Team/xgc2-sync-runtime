//! Data handoff between module instances: zero-copy state, dirty coalescing, bounded
//! event queues, several writers, binding checks.

mod common;

use common::*;
use std::time::Duration;
use xgc2_module_host::host::HostError;

#[test]
fn state_samples_are_handed_over_in_place() {
    let f = Fixture::new("zero-copy");
    f.load_module("producer_state");
    f.load_module("consumer");
    f.add(spec("producer", "producer_state", 2.0, r#"{"id":7}"#, &[("out", "samples")]));
    f.add(spec("consumer", "consumer", 0.0, "{}", &[("state_in", "samples")]));
    wait_until("consumer reads 50 samples", Duration::from_secs(10), || count(&f.detail("consumer")["state_updates"]) >= 50);
    let detail = f.detail("consumer");
    assert_eq!(count(&detail["zero_copy_bad"]), 0, "the consumer saw a copy: {detail}");
    assert_eq!(count(&detail["repeat_bad"]), 0, "{detail}");
    assert_eq!(count(&detail["bad_seq"]), 0, "{detail}");
    assert_eq!(count(&detail["wrong_size"]), 0, "{detail}");
    assert_eq!(count(&detail["last_producer"]), 7);
    let channel = f.channel("samples");
    assert_eq!(channel["kind"], "state");
    assert_eq!((channel["writers"].as_u64(), channel["readers"].as_u64()), (Some(1), Some(1)));
    assert!(count(&channel["commits"]) >= 50);
    assert_eq!(count(&channel["stalls"]), 0);
    let consumer = f.instance("consumer");
    assert!(count(&consumer["handoff_latency"]["count"]) >= 50, "{consumer}");
    assert_eq!(consumer["state"], "running");
}

#[test]
fn many_commits_before_a_step_cost_one_step() {
    let f = Fixture::new("coalescing");
    f.load_module("producer_state");
    f.load_module("consumer");
    // 20 commits per producer step, a consumer that needs 15 ms per step.
    f.add(spec("producer", "producer_state", 5.0, r#"{"burst":20}"#, &[("out", "samples")]));
    f.add(spec("consumer", "consumer", 0.0, r#"{"work_us":15000}"#, &[("state_in", "samples")]));
    wait_until("producer committed 400 samples", Duration::from_secs(10), || count(&f.channel("samples")["commits"]) >= 400);
    f.host.stop_instance("producer").unwrap();
    // After the producer stopped, the consumer ends on the newest sample.
    let commits = count(&f.channel("samples")["commits"]);
    wait_until("consumer catches up with the newest sample", Duration::from_secs(10), || {
        count(&f.detail("consumer")["last_state_seq"]) == commits
    });
    let consumer = f.instance("consumer");
    let steps = count(&consumer["steps"]);
    assert!(steps < commits / 3, "{steps} consumer steps for {commits} commits");
    assert!(count(&consumer["coalesced_dirties"]) > commits / 2, "{consumer}");
    assert_eq!(count(&consumer["input_commits"]), commits);
    let detail = f.detail("consumer");
    assert_eq!(count(&detail["bad_seq"]), 0);
    assert_eq!(count(&detail["zero_copy_bad"]), 0);
    // Not every commit is seen: the consumer jumps to the newest.
    assert!(count(&detail["state_updates"]) < commits);
}

#[test]
fn event_queue_is_lossless_until_full_then_counts_drops() {
    let f = Fixture::new("events");
    f.load_module("producer_event");
    f.load_module("consumer");
    // 40 events per 20 ms into a queue of 16, drained by a consumer that needs 30 ms per step.
    f.add(spec("producer", "producer_event", 20.0, r#"{"burst":40}"#, &[("out", "events")]));
    f.add(spec("consumer", "consumer", 0.0, r#"{"work_us":30000}"#, &[("event_in", "events")]));
    wait_until("producer sent and dropped events", Duration::from_secs(10), || {
        let producer = f.detail("producer");
        count(&producer["dropped"]) > 0 && count(&producer["sent"]) >= 64
    });
    f.host.stop_instance("producer").unwrap();
    let producer = f.detail("producer");
    let (sent, dropped) = (count(&producer["sent"]), count(&producer["dropped"]));
    let channel = f.channel("events");
    assert_eq!(count(&channel["drops"]), dropped, "every refused write_begin is a counted drop");
    assert_eq!(count(&channel["commits"]), sent);
    assert_eq!(channel["depth"], 16);
    wait_until("consumer drained the queue", Duration::from_secs(10), || count(&f.detail("consumer")["events"]) == sent);
    let consumer = f.detail("consumer");
    assert_eq!(count(&consumer["event_disorder"]), 0, "FIFO order broken: {consumer}");
    // Events were lost only to the full queue: what was accepted arrived, with gaps where drops happened.
    assert!(count(&consumer["event_gaps"]) > 0);
}

#[test]
fn several_writers_share_an_event_channel_and_keep_their_own_order() {
    let f = Fixture::new("multi-writer");
    f.load_module("producer_event");
    f.load_module("consumer");
    f.add(spec("consumer", "consumer", 0.0, "{}", &[("event_in", "bus")]));
    for id in 1..=3 {
        f.add(spec(&format!("producer{id}"), "producer_event", 1.0, &format!(r#"{{"id":{id},"burst":2}}"#), &[("out", "bus")]));
    }
    let channel = f.channel("bus");
    assert_eq!(channel["writers"], 3);
    wait_until("all producers sent 100 events", Duration::from_secs(10), || {
        (1..=3).all(|id| count(&f.detail(&format!("producer{id}"))["sent"]) >= 100)
    });
    for id in 1..=3 {
        f.host.stop_instance(&format!("producer{id}")).unwrap();
    }
    let sent: u64 = (1..=3).map(|id| count(&f.detail(&format!("producer{id}"))["sent"])).sum();
    wait_until("consumer drained", Duration::from_secs(10), || count(&f.detail("consumer")["events"]) == sent);
    let consumer = f.detail("consumer");
    assert_eq!(count(&consumer["event_disorder"]), 0, "{consumer}");
    assert_eq!(count(&consumer["event_gaps"]), 0, "nothing was dropped, so nothing may be missing: {consumer}");
}

#[test]
fn an_aborted_event_claim_never_reaches_a_reader() {
    let f = Fixture::new("abort");
    f.load_module("producer_event");
    f.load_module("consumer");
    f.add(spec("consumer", "consumer", 0.0, "{}", &[("event_in", "bus")]));
    // Every third claimed event is given back with write_abort instead of being committed.
    f.add(spec("producer", "producer_event", 2.0, r#"{"burst":7,"abort_every":3}"#, &[("out", "bus")]));
    wait_until("events flow", Duration::from_secs(10), || count(&f.detail("producer")["sent"]) >= 70);
    f.host.stop_instance("producer").unwrap();
    let producer = f.detail("producer");
    let (sent, aborted) = (count(&producer["sent"]), count(&producer["aborted"]));
    assert!(aborted >= 30, "{producer}");
    assert_eq!(count(&producer["dropped"]), 0, "the queue never fills: {producer}");
    wait_until("the consumer drained the queue", Duration::from_secs(10), || count(&f.detail("consumer")["events"]) == sent);
    let channel = f.channel("bus");
    assert_eq!(count(&channel["commits"]), sent, "an aborted claim is not a commit: {channel}");
    let consumer = f.detail("consumer");
    assert_eq!(count(&consumer["event_disorder"]), 0, "{consumer}");
    // The consumer sees the producer's own sequence numbers: one hole per aborted claim, except
    // for a claim aborted last, which leaves no later event to show the hole.
    let gaps = count(&consumer["event_gaps"]);
    assert!(gaps == aborted || gaps + 1 == aborted, "{gaps} gaps for {aborted} aborted claims");
}

#[test]
fn every_event_reader_gets_every_event() {
    let f = Fixture::new("broadcast");
    f.load_module("producer_event");
    f.load_module("consumer");
    f.add(spec("a", "consumer", 0.0, "{}", &[("event_in", "bus")]));
    f.add(spec("b", "consumer", 0.0, "{}", &[("event_in", "bus")]));
    f.add(spec("producer", "producer_event", 2.0, r#"{"burst":3}"#, &[("out", "bus")]));
    wait_until("events flow", Duration::from_secs(10), || count(&f.detail("producer")["sent"]) >= 60);
    f.host.stop_instance("producer").unwrap();
    let sent = count(&f.detail("producer")["sent"]);
    wait_until("both readers drained", Duration::from_secs(10), || {
        count(&f.detail("a")["events"]) == sent && count(&f.detail("b")["events"]) == sent
    });
    assert_eq!(f.channel("bus")["readers"], 2);
}

#[test]
fn unconnected_outputs_get_a_private_channel_that_others_can_join_later() {
    let f = Fixture::new("private");
    f.load_module("producer_state");
    f.load_module("consumer");
    f.add(spec("producer", "producer_state", 2.0, "{}", &[]));
    assert!(has_channel(&f.host, "producer.out"));
    wait_until("commits", Duration::from_secs(5), || count(&f.channel("producer.out")["commits"]) > 3);
    f.add(spec("late", "consumer", 0.0, "{}", &[("state_in", "producer.out")]));
    wait_until("late consumer sees data", Duration::from_secs(5), || count(&f.detail("late")["state_updates"]) > 0);
}

#[test]
fn bindings_are_checked_against_ports_and_payloads() {
    let f = Fixture::new("bindings");
    f.load_module("producer_state");
    f.load_module("producer_event");
    f.load_module("consumer");
    f.add(spec("p1", "producer_state", 0.0, "{}", &[("out", "samples")]));
    let refused = |result: Result<serde_json::Value, HostError>| match result {
        Err(HostError::Invalid(message)) | Err(HostError::Conflict(message)) => message,
        other => panic!("expected a refusal, got {other:?}"),
    };
    // A second writer on a state channel.
    let message = refused(f.host.add_instance(spec("p2", "producer_state", 0.0, "{}", &[("out", "samples")])));
    assert!(message.contains("already has a writer"), "{message}");
    // A state input on an event channel, and an event input on a state channel.
    f.add(spec("e1", "producer_event", 0.0, "{}", &[("out", "events")]));
    let message = refused(f.host.add_instance(spec("c1", "consumer", 0.0, "{}", &[("state_in", "events")])));
    assert!(message.contains("event") && message.contains("state"), "{message}");
    let message = refused(f.host.add_instance(spec("c2", "consumer", 0.0, "{}", &[("event_in", "samples")])));
    assert!(message.contains("state"), "{message}");
    // Unknown port, bad channel name, duplicate instance name.
    let message = refused(f.host.add_instance(spec("c3", "consumer", 0.0, "{}", &[("nope", "samples")])));
    assert!(message.contains("no port nope"), "{message}");
    let message = refused(f.host.add_instance(spec("c4", "consumer", 0.0, "{}", &[("state_in", "bad name")])));
    assert!(message.contains("not a valid name"), "{message}");
    let message = refused(f.host.add_instance(spec("p1", "producer_state", 0.0, "{}", &[])));
    assert!(message.contains("exists"), "{message}");
    // Nothing leaked from the failed adds.
    let names: Vec<String> =
        f.host.health()["instances"].as_array().unwrap().iter().map(|i| i["name"].as_str().unwrap().to_owned()).collect();
    assert_eq!(names, ["p1", "e1"]);
    assert!(!has_channel(&f.host, "bad name"));
}

#[test]
fn reader_capacity_is_bounded_by_the_channel() {
    let f = Fixture::with("capacity", |options| {
        options.hints.insert("samples".into(), xgc2_module_host::plan::Hint { depth: None, max_readers: Some(2) });
    });
    f.load_module("producer_state");
    f.load_module("consumer");
    f.add(spec("producer", "producer_state", 5.0, "{}", &[("out", "samples")]));
    f.add(spec("c1", "consumer", 0.0, "{}", &[("state_in", "samples")]));
    f.add(spec("c2", "consumer", 0.0, "{}", &[("state_in", "samples")]));
    let error = f.host.add_instance(spec("c3", "consumer", 0.0, "{}", &[("state_in", "samples")])).unwrap_err();
    assert!(error.to_string().contains("readers"), "{error}");
    assert_eq!(f.channel("samples")["max_readers"], 2);
}
