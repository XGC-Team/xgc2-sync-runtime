//! The lifecycle state machine every module shares. The host owns all
//! transitions. A module can only *request* Degrade or Recover through the
//! host API. The transition table is the single source of truth, and
//! `tests::every_state_event_pair` pins every one of its 6 × 8 cells.

use std::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum State {
    Unconfigured,
    Inactive,
    Active,
    Degraded,
    Error,
    Finalized,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Event {
    Configure,
    Activate,
    Degrade,
    Recover,
    Deactivate,
    /// Any failed vtable call or plugin-reported fault.
    Fault,
    /// Error → Unconfigured, used only by the restart policy.
    Reset,
    Shutdown,
}

impl State {
    pub const ALL: [State; 6] =
        [State::Unconfigured, State::Inactive, State::Active, State::Degraded, State::Error, State::Finalized];

    /// States in which the host calls `step`.
    pub fn runs(self) -> bool {
        matches!(self, State::Active | State::Degraded)
    }

    pub fn name(self) -> &'static str {
        match self {
            State::Unconfigured => "unconfigured",
            State::Inactive => "inactive",
            State::Active => "active",
            State::Degraded => "degraded",
            State::Error => "error",
            State::Finalized => "finalized",
        }
    }
}

impl Event {
    pub const ALL: [Event; 8] = [
        Event::Configure,
        Event::Activate,
        Event::Degrade,
        Event::Recover,
        Event::Deactivate,
        Event::Fault,
        Event::Reset,
        Event::Shutdown,
    ];
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidTransition {
    pub from: State,
    pub event: Event,
}

impl fmt::Display for InvalidTransition {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?} is not allowed in state {}", self.event, self.from)
    }
}

impl std::error::Error for InvalidTransition {}

/// The transition table. `None` means the event is rejected in that state.
pub fn next(from: State, event: Event) -> Option<State> {
    use Event as E;
    use State as S;
    match (from, event) {
        (S::Unconfigured, E::Configure) => Some(S::Inactive),
        (S::Inactive, E::Activate) => Some(S::Active),
        (S::Active, E::Degrade) => Some(S::Degraded),
        (S::Degraded, E::Recover) => Some(S::Active),
        (S::Active | S::Degraded, E::Deactivate) => Some(S::Inactive),
        (S::Unconfigured | S::Inactive | S::Active | S::Degraded, E::Fault) => Some(S::Error),
        (S::Error, E::Reset) => Some(S::Unconfigured),
        // A running module must be deactivated before shutdown, so the host
        // always calls `deactivate` and a module never misses its stop hook.
        (S::Unconfigured | S::Inactive | S::Error, E::Shutdown) => Some(S::Finalized),
        _ => None,
    }
}

#[derive(Debug, Clone)]
pub struct Lifecycle {
    state: State,
}

impl Default for Lifecycle {
    fn default() -> Self {
        Self { state: State::Unconfigured }
    }
}

impl Lifecycle {
    pub fn state(&self) -> State {
        self.state
    }

    pub fn apply(&mut self, event: Event) -> Result<State, InvalidTransition> {
        match next(self.state, event) {
            Some(to) => {
                self.state = to;
                Ok(to)
            }
            None => Err(InvalidTransition { from: self.state, event }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_state_event_pair() {
        use Event as E;
        use State as S;
        // One row per state, in Event::ALL order:
        // Configure, Activate, Degrade, Recover, Deactivate, Fault, Reset, Shutdown
        let expected: [(S, [Option<S>; 8]); 6] = [
            (S::Unconfigured, [Some(S::Inactive), None, None, None, None, Some(S::Error), None, Some(S::Finalized)]),
            (S::Inactive, [None, Some(S::Active), None, None, None, Some(S::Error), None, Some(S::Finalized)]),
            (S::Active, [None, None, Some(S::Degraded), None, Some(S::Inactive), Some(S::Error), None, None]),
            (S::Degraded, [None, None, None, Some(S::Active), Some(S::Inactive), Some(S::Error), None, None]),
            (S::Error, [None, None, None, None, None, None, Some(S::Unconfigured), Some(S::Finalized)]),
            (S::Finalized, [None; 8]),
        ];
        assert_eq!(expected.len(), S::ALL.len());
        for (state, row) in expected {
            for (event, want) in E::ALL.into_iter().zip(row) {
                assert_eq!(next(state, event), want, "{state:?} × {event:?}");
            }
        }
    }

    #[test]
    fn restart_path_returns_to_active() {
        let mut fsm = Lifecycle::default();
        for event in [Event::Configure, Event::Activate, Event::Fault, Event::Reset, Event::Configure, Event::Activate] {
            fsm.apply(event).unwrap();
        }
        assert_eq!(fsm.state(), State::Active);
        assert!(fsm.apply(Event::Shutdown).is_err(), "shutdown must go through deactivate");
    }
}
