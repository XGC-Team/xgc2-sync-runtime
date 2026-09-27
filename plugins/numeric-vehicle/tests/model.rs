use numeric_vehicle::model::*;
fn target(start: i64, a: f64) -> Segment {
    let mut b = [0u8; 104];
    for (i, v) in [
        (0, start as f64 * 1e-9),
        (8, 1.0 + 0.2 * (start as f64 * 1e-9 - 1.0)),
        (16, 2.0),
        (24, 3.0),
        (32, 0.2),
        (56, a),
    ] {
        b[i..i + 8].copy_from_slice(&v.to_le_bytes());
    }
    b[98] = 1;
    Segment::decode(&b).unwrap()
}
fn model() -> Model {
    Model::new(Config {
        initial_position: [1.0, 2.0, 3.0],
        initial_velocity: [0.2, 0.0, 0.0],
    })
    .unwrap()
}
fn close(a: f64, b: f64) {
    assert!((a - b).abs() < 1e-12, "{a} != {b}");
}
fn ready(m: &mut Model) {
    m.command("prepare", 1_000_000_000).unwrap();
    m.command("custom1", 1_000_000_000).unwrap();
}
#[test]
fn future_acceleration_respects_activation_and_never_teleports_to_reference() {
    let mut m = model();
    ready(&mut m);
    m.segment(target(1_050_000_000, 2.0), 1_000_000_000)
        .unwrap();
    m.advance(1_040_000_000).unwrap();
    close(m.position[0], 1.008);
    close(m.velocity[0], 0.2);
    // Crossing the activation boundary in one host tick integrates both intervals.
    m.advance(1_100_000_000).unwrap();
    close(m.position[0], 1.0225);
    close(m.velocity[0], 0.3);
    close(m.position[1], 2.0);
    close(m.position[2], 3.0);
    m.advance(1_160_000_000).unwrap();
    close(m.position[0], 1.044);
    close(m.velocity[0], 0.4);
    assert_eq!(m.state, State::Hold);
    assert_eq!(m.acceleration, [0.0; 3]);
}
#[test]
fn late_segment_uses_only_remaining_interval_and_expired_packet_cannot_rewind() {
    let mut m = model();
    ready(&mut m);
    m.segment(target(1_050_000_000, 2.0), 1_080_000_000)
        .unwrap();
    close(m.position[0], 1.016);
    close(m.velocity[0], 0.2);
    m.advance(1_150_000_000).unwrap();
    // No retroactive integration: it starts from the actual state at 1.08.
    // Feedback reduces the missing 30 ms of acceleration without a jump.
    assert!(m.position[0] > 1.0349 && m.position[0] < 1.04);
    assert!(m.velocity[0] > 0.34 && m.velocity[0] < 0.4);
    assert!(m
        .segment(target(1_000_000_000, 9.0), 1_150_000_000)
        .unwrap_err()
        .contains("expired"));
    m.advance(1_170_000_000).unwrap();
    let before = (m.position, m.velocity);
    m.segment(target(1_150_000_000, 1.0), 1_170_000_000)
        .unwrap();
    assert_eq!((m.position, m.velocity), before);
    assert_eq!(m.state, State::Custom1);
}
#[test]
fn commands_own_readiness_hold_and_terminal_stop() {
    let mut m = model();
    m.advance(1_000_000_000).unwrap();
    assert_eq!(m.state, State::Configured);
    assert!(m.command("custom1", 1_000_000_000).is_err());
    assert!(m
        .segment(target(1_050_000_000, 2.0), 1_000_000_000)
        .is_err());
    ready(&mut m);
    m.segment(target(1_050_000_000, 2.0), 1_000_000_000)
        .unwrap();
    m.command("hold", 1_060_000_000).unwrap();
    let v = m.velocity;
    assert!(m
        .segment(target(1_100_000_000, 8.0), 1_070_000_000)
        .is_err());
    m.advance(1_100_000_000).unwrap();
    assert_eq!(m.velocity, v);
    m.command("stop", 1_100_000_000).unwrap();
    let p = m.position;
    m.advance(2_000_000_000).unwrap();
    assert_eq!(m.position, p);
    assert_eq!(m.velocity, v);
    assert!(m.command("prepare", 2_000_000_000).is_err());
}
#[test]
fn duplicate_conflict_and_invalid_force_frame_are_observable() {
    let mut m = model();
    ready(&mut m);
    let s = target(1_050_000_000, 2.0);
    m.segment(s.clone(), 1_000_000_000).unwrap();
    m.segment(s, 1_000_000_000).unwrap();
    assert!(m
        .segment(target(1_050_000_000, 3.0), 1_000_000_000)
        .is_err());
    assert_eq!(m.state, State::Fault);
    for (offset, value) in [(97, 2), (98, 8), (96, 64), (96, 1), (96, 8)] {
        let mut b = [0u8; 104];
        b[..8].copy_from_slice(&1.0f64.to_le_bytes());
        b[98] = 1;
        b[offset] = value;
        assert!(Segment::decode(&b).is_err());
    }
}
#[test]
fn successor_at_expiry_is_continuous_and_backward_time_is_refused() {
    let mut m = model();
    ready(&mut m);
    m.segment(target(1_000_000_000, 2.0), 1_000_000_000)
        .unwrap();
    let mut successor = target(1_100_000_000, -1.0);
    successor.position[0] = 1.03;
    successor.velocity[0] = 0.4;
    m.segment(successor, 1_000_000_000).unwrap();
    m.advance(1_150_000_000).unwrap();
    close(m.position[0], 1.04875);
    close(m.velocity[0], 0.35);
    assert_eq!(m.state, State::Custom1);
    assert!(m.advance(1_100_000_000).is_err());
}

#[test]
fn pva_feedback_removes_initial_error_without_resetting_the_plant() {
    let mut m = model();
    ready(&mut m);
    for k in 0..30 {
        let now = 1_000_000_000 + k * PERIOD_NS;
        let mut reference = target(now, 0.0);
        reference.position = [2.0, 2.0, 3.0];
        reference.velocity = [0.0; 3];
        let before = (m.position, m.velocity);
        m.segment(reference, now).unwrap();
        assert_eq!((m.position, m.velocity), before);
        m.advance(now + PERIOD_NS).unwrap();
    }
    assert!((m.position[0] - 2.0).abs() < 0.0001);
    assert!(m.velocity[0].abs() < 0.0003);
    close(m.position[1], 2.0);
    close(m.position[2], 3.0);
}

#[test]
fn a_constant_pva_segment_is_independent_of_host_step_partition() {
    let mut whole = model();
    let mut split = model();
    for m in [&mut whole, &mut split] {
        ready(m);
        let mut s = target(1_000_000_000, 0.3);
        s.position[0] += 0.5;
        s.velocity[0] -= 0.2;
        m.segment(s, 1_000_000_000).unwrap();
    }
    whole.advance(1_100_000_000).unwrap();
    for k in 1..=100 {
        split.advance(1_000_000_000 + k * 1_000_000).unwrap();
    }
    close(whole.position[0], split.position[0]);
    close(whole.velocity[0], split.velocity[0]);
}
