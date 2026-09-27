//! `NeighborExchange`: the DMPC-facing API over two ports.
//!
//! A distributed planner publishes its own plan for round `k` and, at round
//! `k`, solves with each neighbor's plan produced for round `k − 1` (the
//! assumed trajectories of the previous round, as the TRO DMPC does).
//!
//! ```ignore
//! let mut nx = NeighborExchange::new(&host, PLAN_IN, PLAN_OUT, 1);
//! // in step():
//! nx.absorb(&mut host, ctx.round);               // non-blocking: drain new plans
//! if ctx.round_advanced != 0 {
//!     let snap = nx.snapshot(ctx.round, ctx.now);   // one entry per roster neighbor
//!     /* solve with snap.neighbors[i].status / .data */
//!     nx.publish(&host, ctx.round, &my_plan)?;
//! }
//! ```
//!
//! Status of neighbor `j` at round `k`, from the latest admitted plan
//! (highest round, then highest seq). A plan is admitted only when its
//! round is ≤ the planner round passed to `offer`; a future round is
//! refused before the cache and cannot become newest.
//! - `Fresh`: produced for round k − 1 ≤ round ≤ k;
//! - `Stale(n)`: produced for round k − 1 − n, 1 ≤ n ≤ `s_max`;
//! - `Missing`: nothing admitted, older than the stale window, or a cached
//!   round still in the future relative to this snapshot. A future round
//!   is never `Fresh`.
//!
//! Nothing blocks: the planner decides at its own deadline with whatever
//! has arrived, and the snapshot says exactly what that was. Every plan's
//! delivery is independently audited by the host (loss, OWD, age at use).
//!
//! The same contract for C and C++ is `abi/include/xgc_rt_nx.h`; a snapshot
//! goes on the wire as [`SNAPSHOT_SCHEMA`] ([`Snapshot::encode`]).

use std::collections::HashMap;

use crate::{Host, XgcStatus};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NeighborStatus {
    Fresh,
    Stale(u64),
    Missing,
}

#[derive(Debug, Clone, PartialEq)]
pub struct NeighborView<'a> {
    pub origin: u16,
    pub status: NeighborStatus,
    /// Round the plan was produced for (None when Missing and never seen).
    pub round: Option<u64>,
    /// `now − t_produce` of the plan used, ns.
    pub age_ns: Option<i64>,
    /// The plan bytes (empty when Missing).
    pub data: &'a [u8],
}

#[derive(Debug, Clone, PartialEq)]
pub struct Snapshot<'a> {
    pub round: u64,
    pub neighbors: Vec<NeighborView<'a>>,
}

/// The snapshot record: `round` u64, `count` u32, reserved u32, then per
/// neighbor 32 bytes: origin u16, status u8 (0 Fresh, 1 Stale, 2 Missing),
/// 5 reserved, stale rounds u64 (0 unless Stale), round u64 (`u64::MAX` when
/// never admitted), age ns i64 (0 when never admitted). Little-endian.
pub const SNAPSHOT_SCHEMA: &str = "xgc.dmpc.neighbor_snapshot/1";

impl Snapshot<'_> {
    /// This snapshot as a [`SNAPSHOT_SCHEMA`] record.
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(16 + 32 * self.neighbors.len());
        out.extend(self.round.to_le_bytes());
        out.extend((self.neighbors.len() as u32).to_le_bytes());
        out.extend(0u32.to_le_bytes());
        for n in &self.neighbors {
            let (status, stale) = match n.status {
                NeighborStatus::Fresh => (0u8, 0u64),
                NeighborStatus::Stale(k) => (1, k),
                NeighborStatus::Missing => (2, 0),
            };
            out.extend(n.origin.to_le_bytes());
            out.push(status);
            out.extend([0u8; 5]);
            out.extend(stale.to_le_bytes());
            out.extend(n.round.unwrap_or(u64::MAX).to_le_bytes());
            out.extend(n.age_ns.unwrap_or(0).to_le_bytes());
        }
        out
    }

    /// Fraction of neighbors that are Fresh (1.0 with no neighbors).
    pub fn completeness(&self) -> f64 {
        if self.neighbors.is_empty() {
            return 1.0;
        }
        self.neighbors.iter().filter(|n| n.status == NeighborStatus::Fresh).count() as f64 / self.neighbors.len() as f64
    }
}

#[derive(Debug, Clone)]
struct Latest {
    round: u64,
    seq: u64,
    t_produce: i64,
    data: Vec<u8>,
}

pub struct NeighborExchange {
    plan_in: u32,
    plan_out: u32,
    s_max: u64,
    neighbors: Vec<u16>,
    latest: HashMap<u16, Latest>,
}

impl NeighborExchange {
    /// Neighbors are the in-port's bound origins (manifest `from`).
    pub fn new(host: &Host, plan_in: u32, plan_out: u32, s_max: u64) -> Self {
        Self::with_neighbors(host.port_origins(plan_in), plan_in, plan_out, s_max)
    }

    pub fn with_neighbors(neighbors: Vec<u16>, plan_in: u32, plan_out: u32, s_max: u64) -> Self {
        Self { plan_in, plan_out, s_max, neighbors, latest: HashMap::new() }
    }

    pub fn neighbors(&self) -> &[u16] {
        &self.neighbors
    }

    /// Drain every new plan from the in-port, keeping the newest admitted
    /// plan per neighbor. `planner_k` is the caller's current planner round.
    /// Returns how many samples were read.
    pub fn absorb(&mut self, host: &mut Host, planner_k: u64) -> usize {
        let mut n = 0;
        while let Some(s) = host.next(self.plan_in) {
            n += 1;
            let _ = self.offer(planner_k, s.origin, s.round, s.seq, s.t_produce, s.data);
        }
        n
    }

    /// Admit one received plan at planner round `planner_k`.
    ///
    /// Returns false when `origin` is not a neighbor or `round` is in the
    /// future (`round > planner_k`). A refused plan is not cached. Among
    /// admitted plans, a strictly newer `(round, seq)` replaces the cached
    /// plan; an older or duplicate plan does not.
    pub fn offer(&mut self, planner_k: u64, origin: u16, round: u64, seq: u64, t_produce: i64, data: &[u8]) -> bool {
        if !self.neighbors.contains(&origin) || round > planner_k {
            return false;
        }
        let newer = match self.latest.get(&origin) {
            None => true,
            Some(l) => (round, seq) > (l.round, l.seq),
        };
        if newer {
            self.latest.insert(origin, Latest { round, seq, t_produce, data: data.to_vec() });
        }
        true
    }

    pub fn snapshot(&self, k: u64, now: i64) -> Snapshot<'_> {
        let expected = k.saturating_sub(1);
        let neighbors = self
            .neighbors
            .iter()
            .map(|&origin| match self.latest.get(&origin) {
                None => NeighborView { origin, status: NeighborStatus::Missing, round: None, age_ns: None, data: &[] },
                Some(l) => {
                    let status = if l.round > k {
                        NeighborStatus::Missing
                    } else if l.round >= expected {
                        NeighborStatus::Fresh
                    } else if expected - l.round <= self.s_max {
                        NeighborStatus::Stale(expected - l.round)
                    } else {
                        NeighborStatus::Missing
                    };
                    let data: &[u8] = if status == NeighborStatus::Missing { &[] } else { &l.data };
                    NeighborView { origin, status, round: Some(l.round), age_ns: Some(now - l.t_produce), data }
                }
            })
            .collect();
        Snapshot { round: k, neighbors }
    }

    pub fn publish(&self, host: &Host, round: u64, plan: &[u8]) -> Result<(), XgcStatus> {
        host.publish(self.plan_out, round, plan)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn statuses_follow_the_round_rule() {
        let mut nx = NeighborExchange::with_neighbors(vec![1, 2, 3, 4], 0, 1, 2);
        assert!(nx.offer(10, 1, 9, 5, 100, b"a")); // fresh for k = 10
        assert!(nx.offer(10, 2, 8, 4, 100, b"b")); // stale by 1
        assert!(nx.offer(10, 3, 6, 3, 100, b"c")); // 3 behind: beyond s_max = 2 -> missing
        assert!(!nx.offer(10, 9, 9, 1, 100, b"x")); // not a neighbor: ignored
        let s = nx.snapshot(10, 1_100);
        let st: Vec<_> = s.neighbors.iter().map(|n| n.status).collect();
        assert_eq!(st, vec![NeighborStatus::Fresh, NeighborStatus::Stale(1), NeighborStatus::Missing, NeighborStatus::Missing]);
        assert_eq!(s.neighbors[0].age_ns, Some(1_000));
        assert_eq!(s.neighbors[0].data, b"a");
        assert!(s.neighbors[2].data.is_empty() && s.neighbors[3].round.is_none());
        assert_eq!(s.completeness(), 0.25);
    }

    #[test]
    fn a_snapshot_encodes_as_its_record() {
        let mut nx = NeighborExchange::with_neighbors(vec![1, 2, 3], 0, 1, 2);
        assert!(nx.offer(10, 1, 9, 5, 100, b"a"));
        assert!(nx.offer(10, 2, 7, 4, 40, b"b"));
        let b = nx.snapshot(10, 1_100).encode();
        assert_eq!(b.len(), 16 + 3 * 32);
        assert_eq!(u64::from_le_bytes(b[0..8].try_into().unwrap()), 10);
        assert_eq!(u32::from_le_bytes(b[8..12].try_into().unwrap()), 3);
        let entry = |i: usize| {
            let e = &b[16 + 32 * i..16 + 32 * (i + 1)];
            let u = |o: usize| u64::from_le_bytes(e[o..o + 8].try_into().unwrap());
            (u16::from_le_bytes([e[0], e[1]]), e[2], u(8), u(16), u(24) as i64)
        };
        assert_eq!(entry(0), (1, 0, 0, 9, 1_000));
        assert_eq!(entry(1), (2, 1, 2, 7, 1_060));
        assert_eq!(entry(2), (3, 2, 0, u64::MAX, 0));
    }

    #[test]
    fn keeps_the_newest_plan_even_when_an_older_one_arrives_late() {
        let mut nx = NeighborExchange::with_neighbors(vec![1], 0, 1, 1);
        assert!(nx.offer(11, 1, 10, 7, 0, b"new"));
        assert!(nx.offer(11, 1, 9, 6, 0, b"old"));
        assert!(nx.offer(11, 1, 10, 7, 0, b"dup"));
        assert_eq!(nx.snapshot(11, 0).neighbors[0].data, b"new");
        assert!(nx.offer(11, 1, 10, 8, 0, b"newer-seq"));
        assert_eq!(nx.snapshot(11, 0).neighbors[0].data, b"newer-seq");
        assert_eq!(nx.snapshot(11, 0).neighbors[0].status, NeighborStatus::Fresh);
    }

    #[test]
    fn a_far_future_round_is_refused_and_cannot_block_the_current_plan() {
        let mut nx = NeighborExchange::with_neighbors(vec![2], 0, 1, 2);
        assert!(!nx.offer(2, 2, 1_000_000, 1, 0, b"future"));
        let blocked = &nx.snapshot(2, 1).neighbors[0];
        assert_ne!(blocked.status, NeighborStatus::Fresh);
        assert!(blocked.data.is_empty());
        assert_eq!(blocked.round, None);
        assert!(nx.offer(2, 2, 2, 2, 1, b"current"));
        let current = &nx.snapshot(3, 2).neighbors[0];
        assert_eq!(current.data, b"current");
        assert_eq!(current.status, NeighborStatus::Fresh);
        assert_eq!(current.round, Some(2));
    }

    #[test]
    fn snapshot_never_marks_a_future_round_fresh() {
        let mut nx = NeighborExchange::with_neighbors(vec![2], 0, 1, 2);
        assert!(nx.offer(1_000_000, 2, 1_000_000, 1, 0, b"future"));
        let view = &nx.snapshot(2, 1).neighbors[0];
        assert_ne!(view.status, NeighborStatus::Fresh);
        assert!(view.data.is_empty());
        assert_eq!(view.round, Some(1_000_000));
        assert_eq!(nx.snapshot(1_000_000, 1).neighbors[0].status, NeighborStatus::Fresh);
        assert_eq!(nx.snapshot(1_000_000, 1).neighbors[0].data, b"future");
    }
}
