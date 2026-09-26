//! The formation's mission phase (rolling, mission_time), derived on each
//! robot from its own local rounds.
//!
//! This is the logic of the academic `formation_mission_clock.py`
//! (FormationMissionClock), which ran once on the station, took the central
//! sync trigger and sent one FormationTick to every planner. Here every robot
//! runs it on its own rounds (sequence k, trigger time E0 + k·P, from the
//! aligned OS clock). Its inputs are data, never a tick:
//! - the operator's command (start/track/custom1, hold/stop/hover/land,
//!   reset), as each robot receives it;
//! - each robot's controller state, as last reported with its send time. The
//!   robot's own state comes from its controller, and peers' states come over
//!   the link. A state is fresh while `now - stamp <= state_timeout` on the
//!   aligned clocks.
//!
//! Peer gate:
//! - `Start` (default, the time-triggered end state): every robot of the
//!   roster must be fresh in Custom1 for the phase to start rolling. Once
//!   rolling, only this robot's own state, the operator and the clock rules
//!   stop it. A peer that stops reporting is counted as lost and does not stop
//!   the others: one node's failure stays bounded.
//! - `Always` (the station clock's rule): every robot must stay fresh in
//!   Custom1, or the phase holds.
//!
//! Robots can disagree for a round, because their data arrive at different
//! times. dmpc-rounds sends its phase with its state each round and counts
//! disagreements for the audit.

use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PeerGate {
    Start,
    Always,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Barrier {
    Operator,
    Clock,
}

#[derive(Debug)]
pub struct MissionPhase {
    robots: Vec<String>,
    own: String,
    gate: PeerGate,
    state_timeout: f64,
    max_trigger_gap: f64,
    duration: f64,
    states: BTreeMap<String, (String, f64)>,
    requested: bool,
    rolling: bool,
    barrier: Option<Barrier>,
    mission_time: f64,
    previous: Option<(u64, f64)>,
    /// Rounds on which a roster peer was not fresh while this robot rolled.
    pub peers_lost: u64,
}

impl MissionPhase {
    pub fn new(robots: Vec<String>, own: String, gate: PeerGate) -> Result<Self, String> {
        let unique: std::collections::BTreeSet<_> = robots.iter().collect();
        if robots.is_empty() || unique.len() != robots.len() {
            return Err("robots must be a nonempty unique roster".into());
        }
        if !robots.contains(&own) {
            return Err("robot must belong to robots".into());
        }
        Ok(Self {
            robots,
            own,
            gate,
            state_timeout: 1.0,
            max_trigger_gap: 0.5,
            duration: 0.0,
            states: BTreeMap::new(),
            requested: false,
            rolling: false,
            barrier: None,
            mission_time: 0.0,
            previous: None,
            peers_lost: 0,
        })
    }

    pub fn with_limits(mut self, state_timeout: f64, max_trigger_gap: f64, duration: f64) -> Self {
        self.state_timeout = state_timeout;
        self.max_trigger_gap = max_trigger_gap;
        self.duration = duration;
        self
    }

    pub fn robots(&self) -> &[String] {
        &self.robots
    }

    pub fn command(&mut self, value: &str) {
        match value.trim().to_lowercase().as_str() {
            "track" | "custom1" | "start" => {
                self.requested = true;
                self.barrier = None;
            }
            "reset" => {
                self.hold(Barrier::Operator);
                self.mission_time = 0.0;
            }
            "hold" | "stop" | "hover" | "land" => self.hold(Barrier::Operator),
            _ => {}
        }
    }

    fn hold(&mut self, source: Barrier) {
        self.requested = false;
        self.rolling = false;
        self.barrier = Some(source);
    }

    pub fn observe(&mut self, robot: &str, state: &str, stamp: f64) {
        if self.robots.iter().any(|r| r == robot) {
            self.states.insert(robot.to_string(), (state.trim().to_string(), stamp));
        }
    }

    fn fresh_custom1(&self, robot: &str, now: f64) -> bool {
        self.states
            .get(robot)
            .is_some_and(|(s, t)| s == "Custom1" && (0.0..=self.state_timeout).contains(&(now - t)))
    }

    /// One round: `sequence` k, `trigger_time` E0 + k·P, `now` the local
    /// time. Returns (rolling, mission_time), or None for a reordered or
    /// duplicate round.
    pub fn tick(&mut self, sequence: u64, trigger_time: f64, now: f64, full_roster: bool) -> Option<(bool, f64)> {
        if !trigger_time.is_finite() || !now.is_finite() {
            self.hold(Barrier::Clock);
            return None;
        }
        if let Some((prev_seq, prev_t)) = self.previous {
            if sequence <= prev_seq || trigger_time <= prev_t {
                if trigger_time < prev_t {
                    self.hold(Barrier::Clock);
                }
                return None;
            }
            if self.rolling && (sequence != prev_seq + 1 || trigger_time - prev_t > self.max_trigger_gap) {
                // A running phase never absorbs a clock jump or missing rounds.
                self.hold(Barrier::Clock);
            }
        }
        let all_fresh = self.robots.iter().all(|r| self.fresh_custom1(r, now));
        let ready = full_roster
            && match (self.gate, self.rolling) {
                (PeerGate::Start, true) => {
                    if !all_fresh {
                        self.peers_lost += 1;
                    }
                    self.fresh_custom1(&self.own, now)
                }
                _ => all_fresh,
            };
        if ready && !self.requested && self.barrier != Some(Barrier::Clock) {
            self.requested = true;
        }
        let mut rolling = self.requested && ready;
        // A hold or a start barrier adds no mission time; resume at the phase
        // held, then add scheduled round intervals, never measured dt.
        if rolling && self.rolling {
            if let Some((_, prev_t)) = self.previous {
                self.mission_time += trigger_time - prev_t;
            }
        }
        if self.duration > 0.0 && self.mission_time >= self.duration {
            self.mission_time = self.duration;
            self.requested = false;
            rolling = false;
            self.barrier = Some(Barrier::Clock);
        }
        self.rolling = rolling;
        self.previous = Some((sequence, trigger_time));
        Some((rolling, self.mission_time))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn phase(gate: PeerGate) -> MissionPhase {
        MissionPhase::new(vec!["a".into(), "b".into()], "a".into(), gate).unwrap()
    }

    // The cases of the academic test_formation_mission_clock.py, same numbers.
    #[test]
    fn staggered_entry_does_not_start_individual_clocks() {
        let mut c = phase(PeerGate::Always);
        c.command("track");
        c.observe("a", "Custom1", 72.0);
        assert_eq!(c.tick(1, 72.0, 72.0, true), Some((false, 0.0)));
        c.observe("a", "Custom1", 112.0);
        c.observe("b", "Custom1", 112.0);
        assert_eq!(c.tick(2, 112.0, 112.0, true), Some((true, 0.0)));
        assert!((c.tick(3, 112.1, 112.14, true).unwrap().1 - 0.1).abs() < 1e-9);
    }

    #[test]
    fn stop_freezes_phase_and_resume_excludes_hold_duration() {
        for gate in [PeerGate::Always, PeerGate::Start] {
            let mut c = phase(gate);
            c.observe("a", "Custom1", 1.0);
            c.observe("b", "Custom1", 1.0);
            c.command("track");
            c.tick(1, 1.0, 1.0, true);
            c.tick(2, 1.1, 1.1, true);
            c.command("hover");
            c.observe("a", "Ready", 1.2);
            c.observe("b", "Ready", 1.2);
            assert!((c.tick(3, 1.2, 1.2, true).unwrap().1 - 0.1).abs() < 1e-9);
            c.command("track");
            c.observe("a", "Custom1", 20.0);
            c.observe("b", "Custom1", 20.0);
            assert!((c.tick(4, 20.0, 20.0, true).unwrap().1 - 0.1).abs() < 1e-9);
            assert!((c.tick(5, 20.1, 20.1, true).unwrap().1 - 0.2).abs() < 1e-9);
        }
    }

    #[test]
    fn reset_zeros_mission_time_and_a_clock_jump_holds_until_the_operator_resumes() {
        let mut c = phase(PeerGate::Start);
        c.observe("a", "Custom1", 0.0);
        c.observe("b", "Custom1", 0.0);
        c.command("start");
        c.tick(1, 0.0, 0.0, true);
        c.tick(2, 0.1, 0.1, true);
        // Round 3 missing: the phase holds and stays held without a new start.
        c.observe("a", "Custom1", 0.3);
        c.observe("b", "Custom1", 0.3);
        assert_eq!(c.tick(4, 0.3, 0.3, true).map(|r| r.0), Some(false));
        assert_eq!(c.tick(5, 0.4, 0.4, true).map(|r| r.0), Some(false));
        c.command("start");
        assert_eq!(c.tick(6, 0.5, 0.5, true).map(|r| r.0), Some(true));
        // Reset zeros the phase; as in the station clock, it re-arms by
        // itself once every robot is back in Custom1 (the robots leave Custom1
        // on hold/reset).
        c.command("reset");
        c.observe("a", "Hover", 0.6);
        assert_eq!(c.tick(7, 0.6, 0.6, true), Some((false, 0.0)));
        assert_eq!(c.tick(7, 0.6, 0.6, true), None, "duplicate round");
    }

    #[test]
    fn a_lost_peer_holds_the_station_rule_but_not_the_time_triggered_one() {
        for (gate, rolls_on) in [(PeerGate::Always, false), (PeerGate::Start, true)] {
            let mut c = phase(gate);
            c.command("start");
            for k in 0..5u64 {
                let t = k as f64 * 0.1;
                c.observe("a", "Custom1", t);
                if k < 2 {
                    c.observe("b", "Custom1", t); // b stops reporting after round 1
                }
                c.tick(k, t, t, true);
            }
            // b's last report is 0.1 s old at 0.4 s: still fresh (timeout 1 s).
            let mut last = None;
            for k in 5..30u64 {
                let t = k as f64 * 0.1;
                c.observe("a", "Custom1", t);
                last = c.tick(k, t, t, true);
            }
            assert_eq!(last.unwrap().0, rolls_on, "{gate:?}");
            assert_eq!(c.peers_lost > 0, rolls_on, "{gate:?}: loss is counted while rolling on");
        }
    }

    #[test]
    fn the_start_still_needs_every_robot() {
        let mut c = phase(PeerGate::Start);
        c.command("start");
        c.observe("a", "Custom1", 0.0);
        assert_eq!(c.tick(1, 0.0, 0.0, true), Some((false, 0.0)));
        c.observe("b", "Custom1", 0.1);
        c.observe("a", "Custom1", 0.1);
        assert_eq!(c.tick(2, 0.1, 0.1, true), Some((true, 0.0)));
    }
}
