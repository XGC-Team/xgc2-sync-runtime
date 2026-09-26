use serde::Deserialize;
use std::collections::BTreeMap;

pub const PERIOD_NS: i64 = 100_000_000;
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub initial_position: [f64; 3],
    pub initial_velocity: [f64; 3],
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Configured,
    Ready,
    Custom1,
    Hold,
    Stopped,
    Fault,
}
impl State {
    pub fn name(self) -> &'static str {
        match self {
            Self::Configured => "Configured",
            Self::Ready => "Ready",
            Self::Custom1 => "Custom1",
            Self::Hold => "Hold",
            Self::Stopped => "Stopped",
            Self::Fault => "Fault",
        }
    }
}
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    pub start: i64,
    pub acceleration: [f64; 3],
    wire: [u8; 104],
}
impl Segment {
    pub fn decode(bytes: &[u8]) -> Result<Self, String> {
        let wire: [u8; 104] = bytes
            .try_into()
            .map_err(|_| "PositionTarget must be 104 bytes")?;
        let number = |i| f64::from_le_bytes(wire[i..i + 8].try_into().unwrap());
        if (0..12).any(|i| !number(i * 8).is_finite()) {
            return Err("nonfinite PositionTarget".into());
        }
        let seconds = number(0);
        let stamp = seconds * 1e9;
        if seconds <= 0.0 || stamp >= (i64::MAX - PERIOD_NS) as f64 {
            return Err("invalid effective timestamp".into());
        }
        let mask = u16::from_le_bytes(wire[96..98].try_into().unwrap());
        if wire[98] != 1 || mask & (64 | 128 | 256 | 512) != 0 {
            return Err("requires world ENU frame 1 and all acceleration axes, not force".into());
        }
        if wire[99..].iter().any(|b| *b != 0) {
            return Err("nonzero reserved bytes".into());
        }
        Ok(Self {
            start: stamp.round() as i64,
            acceleration: [number(56), number(64), number(72)],
            wire,
        })
    }
    pub fn end(&self) -> i64 {
        self.start + PERIOD_NS
    }
}
pub struct Model {
    pub position: [f64; 3],
    pub velocity: [f64; 3],
    pub state: State,
    pub acceleration: [f64; 3],
    now: Option<i64>,
    tracking: bool,
    active: Option<Segment>,
    pending: BTreeMap<i64, Segment>,
}
impl Model {
    pub fn new(config: Config) -> Result<Self, String> {
        if config
            .initial_position
            .iter()
            .chain(&config.initial_velocity)
            .any(|v| !v.is_finite())
        {
            return Err("initial state must be finite".into());
        }
        Ok(Self {
            position: config.initial_position,
            velocity: config.initial_velocity,
            state: State::Configured,
            acceleration: [0.0; 3],
            now: None,
            tracking: false,
            active: None,
            pending: BTreeMap::new(),
        })
    }
    fn clear(&mut self) {
        self.active = None;
        self.pending.clear();
        self.acceleration = [0.0; 3];
    }
    pub fn command(&mut self, command: &str, now: i64) -> Result<(), String> {
        self.advance(now)?;
        match command {
            "prepare" if self.state == State::Configured || self.state == State::Ready => {
                self.state = State::Ready
            }
            "custom1" | "start" if matches!(self.state, State::Ready | State::Hold) => {
                self.tracking = true;
                self.state = State::Custom1;
            }
            "hold" | "hover"
                if matches!(self.state, State::Ready | State::Custom1 | State::Hold) =>
            {
                self.tracking = false;
                self.clear();
                self.state = State::Hold;
            }
            "stop" => {
                self.tracking = false;
                self.clear();
                self.state = State::Stopped;
            }
            _ => {
                return Err(format!(
                    "command {command:?} rejected in {}",
                    self.state.name()
                ))
            }
        }
        Ok(())
    }
    pub fn segment(&mut self, segment: Segment, now: i64) -> Result<(), String> {
        self.advance(now)?;
        if !self.tracking {
            return Err("tracking not commanded".into());
        }
        if segment.end() <= now {
            return Err("segment already expired".into());
        }
        if let Some(existing) = self
            .active
            .as_ref()
            .filter(|s| s.start == segment.start)
            .or_else(|| self.pending.get(&segment.start))
        {
            if existing == &segment {
                return Ok(());
            }
            self.state = State::Fault;
            self.tracking = false;
            self.clear();
            return Err("conflicting segment at same effective timestamp".into());
        }
        if self
            .active
            .as_ref()
            .is_some_and(|s| s.start > segment.start)
        {
            return Err("segment superseded by a newer effective timestamp".into());
        }
        if segment.start <= now {
            self.acceleration = segment.acceleration;
            self.active = Some(segment);
            self.state = State::Custom1;
        } else {
            if self.pending.len() >= 64 {
                return Err("future segment queue full".into());
            }
            self.pending.insert(segment.start, segment);
        }
        Ok(())
    }
    fn integrate(&mut self, ns: i64) -> Result<(), String> {
        let h = ns as f64 * 1e-9;
        for i in 0..3 {
            self.position[i] += h * self.velocity[i] + 0.5 * h * h * self.acceleration[i];
            self.velocity[i] += h * self.acceleration[i];
        }
        if self
            .position
            .iter()
            .chain(&self.velocity)
            .any(|v| !v.is_finite())
        {
            self.state = State::Fault;
            self.tracking = false;
            self.clear();
            return Err("numerical state overflow".into());
        }
        Ok(())
    }
    pub fn advance(&mut self, now: i64) -> Result<(), String> {
        if now < 0 || self.now.is_some_and(|last| now < last) {
            return Err("model clock moved backward".into());
        }
        let Some(mut at) = self.now.replace(now) else {
            return Ok(());
        };
        if matches!(
            self.state,
            State::Configured | State::Stopped | State::Fault
        ) {
            return Ok(());
        }
        while at <= now {
            // A successor at the same instant takes over without an artificial Hold.
            if self.tracking {
                if let Some((&start, _)) = self.pending.first_key_value() {
                    if start <= at {
                        let s = self.pending.remove(&start).unwrap();
                        self.acceleration = s.acceleration;
                        self.active = Some(s);
                        self.state = State::Custom1;
                        continue;
                    }
                }
            }
            if self.active.as_ref().is_some_and(|s| s.end() <= at) {
                self.active = None;
                self.acceleration = [0.0; 3];
                self.state = State::Hold;
            }
            if at == now {
                break;
            }
            let next_start = self
                .pending
                .first_key_value()
                .map(|(k, _)| *k)
                .unwrap_or(now);
            let next_end = self.active.as_ref().map(Segment::end).unwrap_or(now);
            let until = now.min(next_start).min(next_end);
            self.integrate(until - at)?;
            at = until;
        }
        Ok(())
    }
}
