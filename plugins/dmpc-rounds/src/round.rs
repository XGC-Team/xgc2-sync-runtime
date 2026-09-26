//! Planner round for `dmpc-rounds`.
//!
//! Legacy config omits `planner_period_ms`. Planner k is then the host round,
//! and a step advances only when `round_advanced != 0`.
//!
//! `planner_period_ms = 100` is the 100:1 schedule on a 1 ms host.
//! `k = floor((round_start - planner_epoch_ns) / 100ms)`, and
//! `trigger(k) = planner_epoch_ns + k·100ms`. `planner_epoch_ns` defaults to 0.
//! A dirty or wake step (`round_advanced == 0`) never opens a planner round.
//! Host steps inside the same 100 ms open it once. A jump opens only the
//! arrived k, not the skipped ones.
//!
//! p2D consumes that beat on port `sync_trigger`
//! (`xgc.dmpc.sync_trigger/1`). The envelope round and `sequence_id` are
//! planner k. `trigger_time` is `trigger(k)` in seconds. This plugin does not
//! read a wall clock; `round_start` is the host Session time (ral `SimClock`
//! when that profile is selected).

use std::collections::BTreeMap;

use xgc_rt_abi::XgcStepCtx;

use crate::plan_uav_id;
use crate::PLAN_HEADER;

pub const HOST_PERIOD_NS: i64 = 1_000_000;
pub const PLANNER_PERIOD_NS: i64 = 100_000_000;
pub const PLANNER_PERIOD_MS: i64 = 100;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    Legacy,
    Hundred { epoch_ns: i64 },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Beat {
    pub k: u64,
    pub trigger_ns: i64,
    pub advance: bool,
    pub skipped: u64,
}

#[derive(Debug, Clone)]
pub struct PlannerCursor {
    mode: Mode,
    opened: Option<u64>,
    host_mark: Option<(u64, i64)>,
}

impl PlannerCursor {
    pub fn legacy() -> Self {
        Self { mode: Mode::Legacy, opened: None, host_mark: None }
    }

    pub fn hundred_to_one(epoch_ns: i64) -> Self {
        Self { mode: Mode::Hundred { epoch_ns }, opened: None, host_mark: None }
    }

    pub fn observe(&mut self, ctx: &XgcStepCtx) -> Result<Beat, String> {
        match self.mode {
            Mode::Legacy => Ok(Beat { k: ctx.round, trigger_ns: ctx.round_start, advance: ctx.round_advanced != 0, skipped: 0 }),
            Mode::Hundred { epoch_ns } => self.observe_hundred(ctx, epoch_ns),
        }
    }

    fn observe_hundred(&mut self, ctx: &XgcStepCtx, epoch_ns: i64) -> Result<Beat, String> {
        let k = planner_index(ctx.round_start, epoch_ns)?;
        let trigger_ns = trigger_at(epoch_ns, k)?;
        if ctx.round_advanced == 0 {
            return Ok(Beat { k, trigger_ns, advance: false, skipped: 0 });
        }
        if let Some((prev_round, prev_start)) = self.host_mark {
            if ctx.round <= prev_round {
                return Err("dmpc-rounds: host round did not advance".into());
            }
            let steps = (ctx.round - prev_round) as i128;
            let dt = (ctx.round_start as i128) - (prev_start as i128);
            if dt / steps != HOST_PERIOD_NS as i128 || dt % steps != 0 {
                return Err("dmpc-rounds: host period is not 1 ms".into());
            }
        }
        self.host_mark = Some((ctx.round, ctx.round_start));
        let (advance, skipped) = match self.opened {
            None => (true, k),
            Some(prev) if k > prev => (true, k - prev - 1),
            Some(_) => (false, 0),
        };
        if advance {
            self.opened = Some(k);
        }
        Ok(Beat { k, trigger_ns, advance, skipped })
    }
}

fn planner_index(round_start: i64, epoch_ns: i64) -> Result<u64, String> {
    if round_start < epoch_ns {
        return Err("dmpc-rounds: planner time is before the epoch".into());
    }
    let elapsed = (round_start as i128) - (epoch_ns as i128);
    u64::try_from(elapsed / PLANNER_PERIOD_NS as i128).map_err(|_| "dmpc-rounds: planner round does not fit".into())
}

fn trigger_at(epoch_ns: i64, k: u64) -> Result<i64, String> {
    let tick = (k as i128).checked_mul(PLANNER_PERIOD_NS as i128).ok_or("dmpc-rounds: planner trigger overflow")?;
    i64::try_from((epoch_ns as i128) + tick).map_err(|_| "dmpc-rounds: planner trigger overflow".into())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlanShape {
    pub n: u32,
    pub horizon: u32,
    pub rest_count: u32,
}

impl PlanShape {
    pub fn new(n: u32, horizon: u32, rest_count: u32) -> Result<Self, String> {
        if n == 0 || horizon == 0 || horizon == u32::MAX {
            return Err("dmpc-rounds: plan_n and plan_horizon must be positive".into());
        }
        if rest_count != 0 && rest_count != n {
            return Err("dmpc-rounds: plan_rest_count must be 0 or plan_n".into());
        }
        Ok(Self { n, horizon, rest_count })
    }
}

#[derive(Debug, Clone)]
pub struct OriginMap {
    by_origin: BTreeMap<u16, u32>,
    by_id: BTreeMap<u32, u16>,
}

impl OriginMap {
    pub fn new(participant_ids: &[u32], origins: &[u32]) -> Result<Self, String> {
        if participant_ids.is_empty() || participant_ids.len() != origins.len() {
            return Err("dmpc-rounds: origins must name every participant id".into());
        }
        let mut by_origin = BTreeMap::new();
        let mut by_id = BTreeMap::new();
        for (&id, &origin) in participant_ids.iter().zip(origins) {
            let origin = u16::try_from(origin).map_err(|_| "dmpc-rounds: origin does not fit u16".to_string())?;
            if by_id.insert(id, origin).is_some() {
                return Err("dmpc-rounds: duplicate participant id".into());
            }
            if by_origin.insert(origin, id).is_some() {
                return Err("dmpc-rounds: duplicate origin".into());
            }
        }
        Ok(Self { by_origin, by_id })
    }

    pub fn participant(&self, origin: u16) -> Option<u32> {
        self.by_origin.get(&origin).copied()
    }

    pub fn contains_id(&self, id: u32) -> bool {
        self.by_id.contains_key(&id)
    }
}

#[derive(Debug, Clone)]
pub struct Admission {
    pub map: OriginMap,
    pub shape: PlanShape,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanReject {
    Origin,
    Identity,
    NonFinite,
    Shape,
    Length,
}

impl PlanReject {
    pub fn reason(self) -> &'static str {
        match self {
            Self::Origin => "origin is not in the participant map",
            Self::Identity => "payload uav_id does not match the origin",
            Self::NonFinite => "stamp or trajectory value is not finite",
            Self::Shape => "n, horizon or rest count does not match",
            Self::Length => "payload length does not match its counts",
        }
    }
}

/// Check `num_states == n`, `num_timesteps == N+1`, `rest_len`, finiteness and id.
pub fn check_plan_shape(shape: &PlanShape, expected_id: u32, bytes: &[u8]) -> Result<(), PlanReject> {
    let Some(payload_id) = plan_uav_id(bytes) else {
        return Err(PlanReject::Length);
    };
    if payload_id != expected_id {
        return Err(PlanReject::Identity);
    }
    let u = |off: usize| u32::from_le_bytes(bytes[off..off + 4].try_into().unwrap());
    if u(12) != shape.n || u(16) != shape.horizon + 1 || u(20) != shape.rest_count {
        return Err(PlanReject::Shape);
    }
    if !plan_is_finite(bytes) {
        return Err(PlanReject::NonFinite);
    }
    Ok(())
}

/// Admit a neighbor plan before it can enter the cache or `forwarded`.
/// `n` is `num_states`, `horizon` is N, and `num_timesteps` must be N+1.
pub fn admit_plan(admission: &Admission, self_id: u32, origin: u16, bytes: &[u8]) -> Result<u32, PlanReject> {
    let Some(mapped) = admission.map.participant(origin) else {
        return Err(PlanReject::Origin);
    };
    check_plan_shape(&admission.shape, mapped, bytes)?;
    if mapped == self_id {
        return Err(PlanReject::Identity);
    }
    Ok(mapped)
}

fn plan_is_finite(bytes: &[u8]) -> bool {
    let stamp = f64::from_le_bytes(bytes[0..8].try_into().unwrap());
    if !stamp.is_finite() {
        return false;
    }
    let mut off = PLAN_HEADER;
    while off + 8 <= bytes.len() {
        let value = f64::from_le_bytes(bytes[off..off + 8].try_into().unwrap());
        if !value.is_finite() {
            return false;
        }
        off += 8;
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx(round: u64, round_start: i64, advanced: bool) -> XgcStepCtx {
        XgcStepCtx { round, now: round_start, round_start, deadline: round_start, dirty_ports: 0, round_advanced: u32::from(advanced), reserved: 0 }
    }

    #[test]
    fn hundred_to_one_opens_on_the_100ms_boundary_and_ignores_dirty() {
        let epoch = 1_000_000_000;
        let mut clock = PlannerCursor::hundred_to_one(epoch);
        let first = clock.observe(&ctx(0, epoch, true)).unwrap();
        assert_eq!(first.k, 0);
        assert!(first.advance);
        assert_eq!(first.trigger_ns, epoch);
        assert_eq!(first.skipped, 0);
        let inside = clock.observe(&ctx(99, epoch + 99_000_000, true)).unwrap();
        assert_eq!((inside.k, inside.advance), (0, false));
        let dirty = clock.observe(&ctx(100, epoch + 100_000_000, false)).unwrap();
        assert_eq!((dirty.k, dirty.advance, dirty.skipped), (1, false, 0));
        let next = clock.observe(&ctx(100, epoch + 100_000_000, true)).unwrap();
        assert_eq!(next.k, 1);
        assert!(next.advance);
        assert_eq!(next.trigger_ns, epoch + PLANNER_PERIOD_NS);
        assert_eq!(next.skipped, 0);
        let again = clock.observe(&ctx(100, epoch + 100_000_000, false)).unwrap();
        assert!(!again.advance);
    }

    #[test]
    fn a_jump_opens_only_the_arrived_planner_round() {
        let mut clock = PlannerCursor::hundred_to_one(0);
        assert!(clock.observe(&ctx(0, 0, true)).unwrap().advance);
        let jumped = clock.observe(&ctx(300, 300_000_000, true)).unwrap();
        assert_eq!(jumped.k, 3);
        assert!(jumped.advance);
        assert_eq!(jumped.skipped, 2);
        assert_eq!(jumped.trigger_ns, 300_000_000);
    }

    #[test]
    fn a_non_1ms_host_is_rejected() {
        let mut clock = PlannerCursor::hundred_to_one(0);
        clock.observe(&ctx(0, 0, true)).unwrap();
        let err = clock.observe(&ctx(1, 50_000_000, true)).unwrap_err();
        assert!(err.contains("1 ms"), "{err}");
    }

    fn shape() -> Admission {
        Admission {
            map: OriginMap::new(&[1, 2], &[0, 1]).unwrap(),
            shape: PlanShape::new(9, 40, 0).unwrap(),
        }
    }

    fn body(uav: u32, n: u32, steps: u32, rest: u32, stamp: f64) -> Vec<u8> {
        let mut p = vec![0u8; PLAN_HEADER + 8 * (n * steps + rest) as usize];
        p[0..8].copy_from_slice(&stamp.to_le_bytes());
        p[8..12].copy_from_slice(&uav.to_le_bytes());
        p[12..16].copy_from_slice(&n.to_le_bytes());
        p[16..20].copy_from_slice(&steps.to_le_bytes());
        p[20..24].copy_from_slice(&rest.to_le_bytes());
        p
    }

    #[test]
    fn admission_accepts_only_a_mapped_finite_plan() {
        let admission = shape();
        let ok = body(2, 9, 41, 0, 0.0);
        assert_eq!(admit_plan(&admission, 1, 1, &ok), Ok(2));
        assert_eq!(admit_plan(&admission, 1, 3, &ok), Err(PlanReject::Origin));
        assert_eq!(admit_plan(&admission, 1, 1, &body(7, 9, 41, 0, 0.0)), Err(PlanReject::Identity));
        assert_eq!(admit_plan(&admission, 1, 1, &body(1, 9, 41, 0, 0.0)), Err(PlanReject::Identity));
        assert_eq!(admit_plan(&admission, 1, 1, &body(2, 8, 41, 0, 0.0)), Err(PlanReject::Shape));
        assert_eq!(admit_plan(&admission, 1, 1, &body(2, 9, 40, 0, 0.0)), Err(PlanReject::Shape));
        assert_eq!(admit_plan(&admission, 1, 1, &body(2, 9, 41, 9, 0.0)), Err(PlanReject::Shape));
        assert_eq!(admit_plan(&admission, 1, 1, &body(2, 9, 41, 0, f64::NAN)), Err(PlanReject::NonFinite));
        let mut nan_state = body(2, 9, 41, 0, 0.0);
        nan_state[PLAN_HEADER..PLAN_HEADER + 8].copy_from_slice(&f64::INFINITY.to_le_bytes());
        assert_eq!(admit_plan(&admission, 1, 1, &nan_state), Err(PlanReject::NonFinite));
        let mut short = body(2, 9, 41, 0, 0.0);
        short.pop();
        assert_eq!(admit_plan(&admission, 1, 1, &short), Err(PlanReject::Length));
    }

    #[test]
    fn duplicate_origin_or_id_is_not_a_map() {
        assert!(OriginMap::new(&[1, 1], &[0, 1]).unwrap_err().contains("duplicate participant"));
        assert!(OriginMap::new(&[1, 2], &[4, 4]).unwrap_err().contains("duplicate origin"));
    }
}
