//! On-disk audit record (`xgc-rt-audit-records/1`): fixed 56 bytes,
//! little-endian, appended in call order to `<run>/<node>/records.bin`.
//!
//! | off | size | field      | tx          | rx            | consume     |
//! |-----|------|------------|-------------|---------------|-------------|
//! | 0   | 1    | kind       | 2           | 3             | 4           |
//! | 1   | 1    | flags      |             |               |             |
//! | 2   | 2    | origin     | own id      | sender        | sender      |
//! | 4   | 4    | channel    |             |               |             |
//! | 8   | 8    | seq        |             |               |             |
//! | 16  | 8    | round      |             |               |             |
//! | 24  | 8    | t_a        | t_produce   | t_tx          | t_produce   |
//! | 32  | 8    | t_b        | t_tx        | t_rx          | t_consume   |
//! | 40  | 4    | len        | payload     | payload       | payload     |
//! | 44  | 2    | node       | recording node id                         |
//! | 46  | 2    | reserved   |             |               |             |
//! | 48  | 4    | bound_a    | own bound   | sender bound  |             |
//! | 52  | 4    | bound_b    |             | receiver bound|             |
//!
//! Other kinds: subscribe (1, `t_b` = time), reject (5, `t_b` = t_rx, `len`
//! = frame length), overflow (6, `flags` = site, `t_b` = time).

pub const RECORD_LEN: usize = 56;
pub const FORMAT: &str = "xgc-rt-audit-records/1";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Kind {
    Subscribe = 1,
    Tx = 2,
    Rx = 3,
    Consume = 4,
    Reject = 5,
    Overflow = 6,
}

impl Kind {
    fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            1 => Kind::Subscribe,
            2 => Kind::Tx,
            3 => Kind::Rx,
            4 => Kind::Consume,
            5 => Kind::Reject,
            6 => Kind::Overflow,
            _ => return None,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Record {
    pub kind: Kind,
    pub flags: u8,
    pub origin: u16,
    pub channel: u32,
    pub seq: u64,
    pub round: u64,
    pub t_a: i64,
    pub t_b: i64,
    pub len: u32,
    pub node: u16,
    pub bound_a: u32,
    pub bound_b: u32,
}

impl Record {
    pub fn new(kind: Kind, node: u16) -> Self {
        Self { kind, flags: 0, origin: 0, channel: 0, seq: 0, round: 0, t_a: 0, t_b: 0, len: 0, node, bound_a: 0, bound_b: 0 }
    }

    pub fn encode(&self) -> [u8; RECORD_LEN] {
        let mut b = [0u8; RECORD_LEN];
        b[0] = self.kind as u8;
        b[1] = self.flags;
        b[2..4].copy_from_slice(&self.origin.to_le_bytes());
        b[4..8].copy_from_slice(&self.channel.to_le_bytes());
        b[8..16].copy_from_slice(&self.seq.to_le_bytes());
        b[16..24].copy_from_slice(&self.round.to_le_bytes());
        b[24..32].copy_from_slice(&self.t_a.to_le_bytes());
        b[32..40].copy_from_slice(&self.t_b.to_le_bytes());
        b[40..44].copy_from_slice(&self.len.to_le_bytes());
        b[44..46].copy_from_slice(&self.node.to_le_bytes());
        b[48..52].copy_from_slice(&self.bound_a.to_le_bytes());
        b[52..56].copy_from_slice(&self.bound_b.to_le_bytes());
        b
    }

    pub fn decode(b: &[u8]) -> Option<Self> {
        if b.len() != RECORD_LEN {
            return None;
        }
        let u16_at = |o: usize| u16::from_le_bytes([b[o], b[o + 1]]);
        let u32_at = |o: usize| u32::from_le_bytes(b[o..o + 4].try_into().unwrap());
        let u64_at = |o: usize| u64::from_le_bytes(b[o..o + 8].try_into().unwrap());
        Some(Self {
            kind: Kind::from_u8(b[0])?,
            flags: b[1],
            origin: u16_at(2),
            channel: u32_at(4),
            seq: u64_at(8),
            round: u64_at(16),
            t_a: u64_at(24) as i64,
            t_b: u64_at(32) as i64,
            len: u32_at(40),
            node: u16_at(44),
            bound_a: u32_at(48),
            bound_b: u32_at(52),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips() {
        let r = Record {
            kind: Kind::Rx, flags: 3, origin: 7, channel: 9, seq: u64::MAX - 1, round: 12,
            t_a: -5, t_b: i64::MAX, len: 1024, node: 2, bound_a: 1500, bound_b: 900,
        };
        assert_eq!(Record::decode(&r.encode()), Some(r));
    }
}
