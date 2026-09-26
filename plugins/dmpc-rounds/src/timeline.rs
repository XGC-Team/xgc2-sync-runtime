//! Decoder for the fixed 240-byte `xgc_dmpc_mission_timeline_v1` and the
//! 256-byte commit p2D reads. The 64-byte `origin` field is not identity:
//! only the envelope sender is compared with the frozen authority. Ack is
//! emitted when the request is accepted, before `effective_round`.

use sha2::{Digest, Sha256};

pub const REQUEST_LEN: usize = 240;
pub const COMMIT_LEN: usize = 256;
pub const STATUS_LEN: usize = 160;
pub const ACK_LEN: usize = 56;
pub const SCHEMA: u32 = 1;
pub const MIN_LEAD: u64 = 5;
pub const PERIOD_NS: i64 = 100_000_000;

pub const KIND_START: u32 = 1;
pub const KIND_RESUME: u32 = 2;
pub const KIND_HOLD: u32 = 3;
pub const KIND_RESET: u32 = 4;
pub const KIND_GOAL: u32 = 5;
pub const KIND_PATTERN: u32 = 6;

pub const FAULT_NONE: u32 = 0;
pub const FAULT_NOT_READY: u32 = 1;
pub const FAULT_CLOCK: u32 = 2;
pub const FAULT_OVERFLOW: u32 = 3;
pub const FAULT_MISSED: u32 = 4;

const PATTERNS: [u32; 4] = [0, 4, 5, 6];

#[derive(Clone, Debug)]
struct Phase {
    revision: u64,
    effective: u64,
    anchor_ns: i64,
    rolling: bool,
    #[allow(dead_code)]
    kind: u32,
    raw: Vec<u8>,
}

#[derive(Clone, Debug)]
pub struct Timeline {
    session: String,
    authority: u16,
    lead: u64,
    committed: Phase,
    committed_digest: [u8; 32],
    applied: Phase,
    fault: u32,
    latched: bool,
}

#[derive(Debug, PartialEq, Eq)]
pub enum Reject {
    Origin,
    Session,
    Schema,
    Order,
    Predecessor,
    Expired,
    Restart,
    Payload,
    Overflow,
    Conflict,
    Pending,
}

impl Reject {
    pub fn reason(self) -> &'static str {
        match self {
            Self::Origin => "envelope sender is not the frozen authority",
            Self::Session => "session",
            Self::Schema => "schema or padding",
            Self::Order => "revision order",
            Self::Predecessor => "predecessor",
            Self::Expired => "effective round",
            Self::Restart => "anchor does not continue the timeline",
            Self::Payload => "goal or pattern",
            Self::Overflow => "mission time overflow",
            Self::Conflict => "same revision with a different digest",
            Self::Pending => "pending command is not effective yet",
        }
    }
}

#[derive(Debug)]
pub enum Decision {
    Accepted { ack: Vec<u8> },
    Duplicate { ack: Vec<u8> },
}

pub struct RoundView {
    pub status: Vec<u8>,
    pub commit: Vec<u8>,
    pub rolling: bool,
    pub mission_s: f64,
}

impl Timeline {
    pub fn open(session: &str, authority: u16, lead: u64) -> Result<Self, String> {
        if session.is_empty() || session.len() >= 64 || session.as_bytes().contains(&0) {
            return Err("dmpc-rounds: session_id must be a nonempty string under 64 bytes".into());
        }
        if lead < MIN_LEAD {
            return Err("dmpc-rounds: timeline_lead must be at least 5".into());
        }
        let raw = held_record(session, 0, 0, 0);
        let digest = [0u8; 32];
        let phase = Phase { revision: 0, effective: 0, anchor_ns: 0, rolling: false, kind: 0, raw };
        Ok(Self { session: session.to_string(), authority, lead, committed: phase.clone(), committed_digest: digest, applied: phase, fault: FAULT_NONE, latched: false })
    }

    /// `sender` is the transport envelope origin, never `request.origin`.
    pub fn offer(&mut self, sender: u16, bytes: &[u8], planner_k: u64) -> Result<Decision, Reject> {
        if bytes.len() != REQUEST_LEN {
            return Err(Reject::Schema);
        }
        if sender != self.authority {
            return Err(Reject::Origin);
        }
        let parsed = parse_request(bytes)?;
        if parsed.session != self.session {
            return Err(Reject::Session);
        }
        let digest = sha256(bytes);
        if parsed.revision == self.committed.revision {
            return if digest == self.committed_digest {
                Ok(Decision::Duplicate { ack: ack_bytes(parsed.revision, parsed.command_id, &digest) })
            } else {
                Err(Reject::Conflict)
            };
        }
        if self.committed.revision != self.applied.revision && !self.latched {
            return Err(Reject::Pending);
        }
        if parsed.revision != self.committed.revision + 1 {
            return Err(Reject::Order);
        }
        if parsed.predecessor != self.committed.revision || parsed.predecessor_digest != self.committed_digest {
            return Err(Reject::Predecessor);
        }
        if parsed.effective < planner_k.saturating_add(self.lead) || parsed.effective <= self.committed.effective && self.committed.revision > 0 {
            return Err(Reject::Expired);
        }
        self.check_continuity(&parsed)?;
        self.committed = Phase {
            revision: parsed.revision,
            effective: parsed.effective,
            anchor_ns: parsed.anchor_ns,
            rolling: parsed.rolling,
            kind: parsed.kind,
            raw: bytes.to_vec(),
        };
        self.committed_digest = digest;
        self.latched = false;
        self.fault = FAULT_NONE;
        Ok(Decision::Accepted { ack: ack_bytes(parsed.revision, parsed.command_id, &digest) })
    }

    pub fn on_round(&mut self, k: u64, skipped: u64, own_state: &str, controller_fresh: bool) -> RoundView {
        if skipped > 0 {
            let at = if k > skipped { k - skipped - 1 } else { k };
            self.enter_hold(at, FAULT_CLOCK);
        } else if !self.latched && self.committed.revision != self.applied.revision {
            if k == self.committed.effective {
                if self.committed.rolling && (own_state != "Custom1" || !controller_fresh) {
                    self.enter_hold(k, FAULT_NOT_READY);
                } else {
                    self.applied = self.committed.clone();
                    self.fault = FAULT_NONE;
                }
            } else if k > self.committed.effective {
                self.enter_hold(k, FAULT_MISSED);
            }
        }
        if self.applied.rolling && !self.latched && (own_state != "Custom1" || !controller_fresh) {
            self.enter_hold(k, FAULT_NOT_READY);
        }
        let mission = match mission_ns(&self.applied, k) {
            Ok(ns) => ns,
            Err(()) => {
                self.applied.rolling = false;
                self.fault = FAULT_OVERFLOW;
                self.latched = true;
                self.applied.anchor_ns
            }
        };
        RoundView {
            status: status_bytes(self, k, mission),
            commit: commit_bytes(&self.applied, k, mission),
            rolling: self.applied.rolling,
            mission_s: mission as f64 * 1e-9,
        }
    }

    fn enter_hold(&mut self, at: u64, fault: u32) {
        self.applied.anchor_ns = mission_ns(&self.applied, at).unwrap_or(self.applied.anchor_ns);
        self.applied.rolling = false;
        self.fault = fault;
        self.latched = true;
    }

    fn continuity_base(&self) -> &Phase {
        if self.latched { &self.applied } else { &self.committed }
    }

    fn check_continuity(&self, req: &Parsed) -> Result<(), Reject> {
        let base = self.continuity_base();
        let projected = mission_ns(base, req.effective).map_err(|()| Reject::Overflow)?;
        match req.kind {
            KIND_START | KIND_RESUME => {
                if base.rolling || !req.rolling || req.anchor_ns != base.anchor_ns {
                    return Err(Reject::Restart);
                }
            }
            KIND_HOLD => {
                if req.rolling || req.anchor_ns != projected {
                    return Err(Reject::Restart);
                }
            }
            KIND_RESET => {
                if req.rolling || req.anchor_ns != 0 {
                    return Err(Reject::Restart);
                }
            }
            KIND_GOAL | KIND_PATTERN => {
                // Keeping the previous anchor would make mission time at the new
                // effective round fall back to that old anchor.
                if !base.rolling || !req.rolling || req.anchor_ns != projected {
                    return Err(Reject::Restart);
                }
            }
            _ => return Err(Reject::Payload),
        }
        Ok(())
    }
}

struct Parsed {
    session: String,
    revision: u64,
    command_id: u64,
    predecessor: u64,
    predecessor_digest: [u8; 32],
    effective: u64,
    kind: u32,
    rolling: bool,
    anchor_ns: i64,
}

pub fn encode_request(session: &str, revision: u64, command_id: u64, predecessor: u64, predecessor_digest: &[u8; 32], effective: u64, kind: u32, rolling: bool, anchor_ns: i64, pattern_id: u32, goal_xyz: [f64; 3]) -> Vec<u8> {
    let mut out = vec![0u8; REQUEST_LEN];
    out[0..4].copy_from_slice(&SCHEMA.to_le_bytes());
    out[4..8].copy_from_slice(&kind.to_le_bytes());
    out[8..16].copy_from_slice(&revision.to_le_bytes());
    out[16..24].copy_from_slice(&command_id.to_le_bytes());
    out[24..32].copy_from_slice(&predecessor.to_le_bytes());
    out[32..64].copy_from_slice(predecessor_digest);
    out[64..72].copy_from_slice(&effective.to_le_bytes());
    out[72..80].copy_from_slice(&anchor_ns.to_le_bytes());
    out[80..84].copy_from_slice(&u32::from(rolling).to_le_bytes());
    out[84..88].copy_from_slice(&pattern_id.to_le_bytes());
    for (i, value) in goal_xyz.iter().enumerate() {
        out[88 + i * 8..96 + i * 8].copy_from_slice(&value.to_le_bytes());
    }
    write_cstr(&mut out[112..176], session);
    out
}

fn held_record(session: &str, revision: u64, effective: u64, anchor_ns: i64) -> Vec<u8> {
    encode_request(session, revision, 0, 0, &[0u8; 32], effective, 0, false, anchor_ns, 0, [0.0; 3])
}

fn parse_request(bytes: &[u8]) -> Result<Parsed, Reject> {
    if u32::from_le_bytes(bytes[0..4].try_into().unwrap()) != SCHEMA {
        return Err(Reject::Schema);
    }
    let kind = u32::from_le_bytes(bytes[4..8].try_into().unwrap());
    let rolling = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
    let pattern_id = u32::from_le_bytes(bytes[84..88].try_into().unwrap());
    if rolling > 1 {
        return Err(Reject::Schema);
    }
    let session = cstr_field(&bytes[112..176])?;
    let _origin = cstr_field(&bytes[176..240])?;
    if session.is_empty() {
        return Err(Reject::Session);
    }
    let goal = [
        f64::from_le_bytes(bytes[88..96].try_into().unwrap()),
        f64::from_le_bytes(bytes[96..104].try_into().unwrap()),
        f64::from_le_bytes(bytes[104..112].try_into().unwrap()),
    ];
    match kind {
        KIND_START | KIND_RESUME | KIND_HOLD | KIND_RESET => {
            if pattern_id != 0 || goal != [0.0; 3] {
                return Err(Reject::Payload);
            }
        }
        KIND_GOAL => {
            if pattern_id != 0 || goal.iter().any(|v| !v.is_finite()) {
                return Err(Reject::Payload);
            }
        }
        KIND_PATTERN => {
            if !PATTERNS.contains(&pattern_id) || goal != [0.0; 3] {
                return Err(Reject::Payload);
            }
        }
        _ => return Err(Reject::Payload),
    }
    let command_id = u64::from_le_bytes(bytes[16..24].try_into().unwrap());
    if command_id == 0 {
        return Err(Reject::Schema);
    }
    let mut predecessor_digest = [0u8; 32];
    predecessor_digest.copy_from_slice(&bytes[32..64]);
    Ok(Parsed {
        session,
        revision: u64::from_le_bytes(bytes[8..16].try_into().unwrap()),
        command_id,
        predecessor: u64::from_le_bytes(bytes[24..32].try_into().unwrap()),
        predecessor_digest,
        effective: u64::from_le_bytes(bytes[64..72].try_into().unwrap()),
        kind,
        rolling: rolling == 1,
        anchor_ns: i64::from_le_bytes(bytes[72..80].try_into().unwrap()),
    })
}

fn cstr_field(bytes: &[u8]) -> Result<String, Reject> {
    let nul = bytes.iter().position(|b| *b == 0).ok_or(Reject::Schema)?;
    if bytes[nul + 1..].iter().any(|b| *b != 0) {
        return Err(Reject::Schema);
    }
    let text = std::str::from_utf8(&bytes[..nul]).map_err(|_| Reject::Schema)?;
    if text.chars().any(|c| c.is_ascii_control()) {
        return Err(Reject::Schema);
    }
    Ok(text.to_string())
}

fn write_cstr(dst: &mut [u8], text: &str) {
    dst.fill(0);
    let n = text.len().min(dst.len() - 1);
    dst[..n].copy_from_slice(&text.as_bytes()[..n]);
}

fn mission_ns(phase: &Phase, k: u64) -> Result<i64, ()> {
    if !phase.rolling || k <= phase.effective {
        return Ok(phase.anchor_ns);
    }
    let steps = i128::from(k - phase.effective);
    let delta = steps.checked_mul(i128::from(PERIOD_NS)).ok_or(())?;
    i64::try_from(i128::from(phase.anchor_ns) + delta).map_err(|_| ())
}

fn sha256(bytes: &[u8]) -> [u8; 32] {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().into()
}

fn ack_bytes(revision: u64, command_id: u64, digest: &[u8; 32]) -> Vec<u8> {
    let mut out = vec![0u8; ACK_LEN];
    out[0..8].copy_from_slice(&revision.to_le_bytes());
    out[8..40].copy_from_slice(digest);
    out[40..48].copy_from_slice(&command_id.to_le_bytes());
    out[48..52].copy_from_slice(&1u32.to_le_bytes());
    out
}

fn commit_bytes(phase: &Phase, k: u64, mission: i64) -> Vec<u8> {
    let mut out = vec![0u8; COMMIT_LEN];
    let n = phase.raw.len().min(REQUEST_LEN);
    out[..n].copy_from_slice(&phase.raw[..n]);
    if !phase.rolling {
        out[72..80].copy_from_slice(&mission.to_le_bytes());
        out[80..84].copy_from_slice(&0u32.to_le_bytes());
    }
    out[240..248].copy_from_slice(&k.to_le_bytes());
    out[248..256].copy_from_slice(&mission.to_le_bytes());
    out
}

fn status_bytes(tl: &Timeline, k: u64, applied_mission: i64) -> Vec<u8> {
    let common = mission_ns(&tl.committed, k).unwrap_or(tl.committed.anchor_ns);
    let mut out = vec![0u8; STATUS_LEN];
    out[0..8].copy_from_slice(&tl.committed.revision.to_le_bytes());
    out[8..40].copy_from_slice(&tl.committed_digest);
    out[40..48].copy_from_slice(&tl.committed.effective.to_le_bytes());
    out[48..56].copy_from_slice(&common.to_le_bytes());
    out[56..64].copy_from_slice(&tl.applied.revision.to_le_bytes());
    out[64..72].copy_from_slice(&applied_mission.to_le_bytes());
    out[72..76].copy_from_slice(&u32::from(!tl.applied.rolling).to_le_bytes());
    out[76..80].copy_from_slice(&tl.fault.to_le_bytes());
    let reason = match tl.fault {
        FAULT_NOT_READY => "not-ready",
        FAULT_CLOCK => "clock",
        FAULT_OVERFLOW => "overflow",
        FAULT_MISSED => "missed",
        _ => "",
    };
    write_cstr(&mut out[80..160], reason);
    let _ = k;
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn start(effective: u64) -> Vec<u8> {
        encode_request("sess", 1, 10, 0, &[0u8; 32], effective, KIND_START, true, 0, 0, [0.0; 3])
    }

    #[test]
    fn same_revision_conflict_and_projected_goal_anchor() {
        let mut tl = Timeline::open("sess", 4, 5).unwrap();
        let first = start(5);
        assert!(matches!(tl.offer(4, &first, 0).unwrap(), Decision::Accepted { .. }));
        let mut other = first.clone();
        other[16] ^= 0xff;
        assert_eq!(tl.offer(4, &other, 0).unwrap_err(), Reject::Conflict);
        assert!(matches!(tl.offer(4, &first, 0).unwrap(), Decision::Duplicate { .. }));
        let early = encode_request("sess", 2, 11, 1, &sha256(&first), 12, KIND_GOAL, true, 700_000_000, 0, [15.0, -15.0, 2.0]);
        assert_eq!(tl.offer(4, &early, 0).unwrap_err(), Reject::Pending);
        for k in 0..=5 {
            tl.on_round(k, 0, "Custom1", true);
        }
        let stale_anchor = encode_request("sess", 2, 11, 1, &sha256(&first), 12, KIND_GOAL, true, 0, 0, [15.0, -15.0, 2.0]);
        assert_eq!(tl.offer(4, &stale_anchor, 5).unwrap_err(), Reject::Restart);
        let continued = encode_request("sess", 2, 11, 1, &sha256(&first), 12, KIND_GOAL, true, 700_000_000, 0, [15.0, -15.0, 2.0]);
        assert!(tl.offer(4, &continued, 0).is_ok());
    }

    #[test]
    fn payload_origin_bytes_are_not_the_authority() {
        let mut tl = Timeline::open("sess", 4, 5).unwrap();
        let mut named = start(5);
        write_cstr(&mut named[176..240], "not-the-sender");
        assert!(tl.offer(4, &named, 0).is_ok());
        let mut spoofed = start(5);
        write_cstr(&mut spoofed[176..240], "4");
        assert_eq!(tl.offer(9, &spoofed, 0).unwrap_err(), Reject::Origin);
        let mut padded = start(8);
        padded[117] = 1;
        assert_eq!(tl.offer(4, &padded, 0).unwrap_err(), Reject::Schema);
    }
}
