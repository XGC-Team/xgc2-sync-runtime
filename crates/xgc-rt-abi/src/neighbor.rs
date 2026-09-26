//! `NeighborExchange`: the DMPC-facing API over two ports.
//!
//! A distributed planner publishes its own plan for round `k` and, at round
//! `k`, solves with each neighbor's plan produced for round `k − 1` (the
//! assumed trajectories of the previous round, as the TRO DMPC does).
//!
//! ```ignore
//! let mut nx = NeighborExchange::new(&host, PLAN_IN, PLAN_OUT, 1);
//! // in step():
//! nx.absorb(&mut host);                          // non-blocking: drain new plans
//! if ctx.round_advanced != 0 {
//!     let snap = nx.snapshot(ctx.round, ctx.now);   // one entry per roster neighbor
//!     /* solve with snap.neighbors[i].status / .data */
//!     nx.publish(&host, ctx.round, &my_plan)?;
//! }
//! ```
//!
//! Status of neighbor `j` at round `k`, from the latest plan received from
//! it (highest round, then highest seq):
//! - `Fresh`: produced for round ≥ k − 1;
//! - `Stale(n)`: produced for round k − 1 − n, 1 ≤ n ≤ `s_max`;
//! - `Missing`: nothing received, or older than that.
//!
//! Nothing blocks: the planner decides at its own deadline with whatever
//! has arrived, and the snapshot says exactly what that was. Every plan's
//! delivery is independently audited by the host (loss, OWD, age at use).

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

impl Snapshot<'_> {
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

    /// Drain every new plan from the in-port, keeping the newest per neighbor.
    /// Returns how many samples were read.
    pub fn absorb(&mut self, host: &mut Host) -> usize {
        let mut n = 0;
        while let Some(s) = host.next(self.plan_in) {
            n += 1;
            self.offer(s.origin, s.round, s.seq, s.t_produce, s.data);
        }
        n
    }

    /// Record one received plan (what `absorb` does per sample).
    pub fn offer(&mut self, origin: u16, round: u64, seq: u64, t_produce: i64, data: &[u8]) {
        if !self.neighbors.contains(&origin) {
            return;
        }
        let newer = match self.latest.get(&origin) {
            None => true,
            Some(l) => (round, seq) > (l.round, l.seq),
        };
        if newer {
            self.latest.insert(origin, Latest { round, seq, t_produce, data: data.to_vec() });
        }
    }

    pub fn snapshot(&self, k: u64, now: i64) -> Snapshot<'_> {
        let expected = k.saturating_sub(1);
        let neighbors = self
            .neighbors
            .iter()
            .map(|&origin| match self.latest.get(&origin) {
                None => NeighborView { origin, status: NeighborStatus::Missing, round: None, age_ns: None, data: &[] },
                Some(l) => {
                    let status = if l.round >= expected {
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
        nx.offer(1, 9, 5, 100, b"a"); // fresh for k = 10
        nx.offer(2, 8, 4, 100, b"b"); // stale by 1
        nx.offer(3, 6, 3, 100, b"c"); // 3 behind: beyond s_max = 2 -> missing
        nx.offer(9, 9, 1, 100, b"x"); // not a neighbor: ignored
        let s = nx.snapshot(10, 1_100);
        let st: Vec<_> = s.neighbors.iter().map(|n| n.status).collect();
        assert_eq!(st, vec![NeighborStatus::Fresh, NeighborStatus::Stale(1), NeighborStatus::Missing, NeighborStatus::Missing]);
        assert_eq!(s.neighbors[0].age_ns, Some(1_000));
        assert_eq!(s.neighbors[0].data, b"a");
        assert!(s.neighbors[2].data.is_empty() && s.neighbors[3].round.is_none());
        assert_eq!(s.completeness(), 0.25);
    }

    #[test]
    fn keeps_the_newest_plan_even_when_an_older_one_arrives_late() {
        let mut nx = NeighborExchange::with_neighbors(vec![1], 0, 1, 1);
        nx.offer(1, 10, 7, 0, b"new");
        nx.offer(1, 9, 6, 0, b"old");
        assert_eq!(nx.snapshot(11, 0).neighbors[0].data, b"new");
        assert_eq!(nx.snapshot(11, 0).neighbors[0].status, NeighborStatus::Fresh);
    }
}
