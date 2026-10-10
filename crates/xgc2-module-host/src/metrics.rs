//! Latency histogram for the health counters.
//!
//! Log-linear buckets with eight sub-buckets per power of two, so a reported percentile is at
//! most 12.5% above the true value. Recording is one relaxed atomic add; there is no
//! allocation after construction and no lock, so it is safe on the step path.

use serde_json::{json, Value};
use std::sync::atomic::{AtomicU64, Ordering::Relaxed};

const SUB_BITS: u32 = 3;
const SUB_COUNT: u64 = 1 << SUB_BITS;
/// Values at or above 2^45 ns (about 9.7 hours) share the last bucket.
const MAX_OCTAVE: u32 = 41;
const MAX_VALUE: u64 = (1 << (MAX_OCTAVE + SUB_BITS + 1)) - 1;
const BUCKETS: usize = (SUB_COUNT as usize) * (MAX_OCTAVE as usize + 1) + SUB_COUNT as usize;

pub struct Histogram {
    buckets: Box<[AtomicU64]>,
    count: AtomicU64,
    max: AtomicU64,
}

impl Default for Histogram {
    fn default() -> Self {
        Self::new()
    }
}

fn index(value: u64) -> usize {
    let value = value.min(MAX_VALUE);
    if value < SUB_COUNT {
        return value as usize;
    }
    let msb = 63 - value.leading_zeros();
    let octave = msb - SUB_BITS;
    let sub = ((value >> octave) & (SUB_COUNT - 1)) as usize;
    SUB_COUNT as usize + octave as usize * SUB_COUNT as usize + sub
}

/// Largest value that maps to `bucket`.
fn upper(bucket: usize) -> u64 {
    if bucket < SUB_COUNT as usize {
        return bucket as u64;
    }
    let octave = ((bucket - SUB_COUNT as usize) / SUB_COUNT as usize) as u32;
    let sub = ((bucket - SUB_COUNT as usize) % SUB_COUNT as usize) as u64;
    ((SUB_COUNT + sub + 1) << octave) - 1
}

impl Histogram {
    pub fn new() -> Self {
        Self {
            buckets: (0..BUCKETS).map(|_| AtomicU64::new(0)).collect(),
            count: AtomicU64::new(0),
            max: AtomicU64::new(0),
        }
    }

    pub fn record(&self, value: u64) {
        self.buckets[index(value)].fetch_add(1, Relaxed);
        self.count.fetch_add(1, Relaxed);
        self.max.fetch_max(value, Relaxed);
    }

    pub fn count(&self) -> u64 {
        self.count.load(Relaxed)
    }

    pub fn max(&self) -> u64 {
        self.max.load(Relaxed)
    }

    /// Value below which `percent` of the samples fall (0 when empty).
    pub fn percentile(&self, percent: f64) -> u64 {
        let count = self.count();
        if count == 0 {
            return 0;
        }
        let rank = ((percent / 100.0) * count as f64).ceil().clamp(1.0, count as f64) as u64;
        let mut seen = 0;
        for (bucket, cell) in self.buckets.iter().enumerate() {
            seen += cell.load(Relaxed);
            if seen >= rank {
                return upper(bucket).min(self.max());
            }
        }
        self.max()
    }

    pub fn summary(&self) -> Value {
        json!({
            "count": self.count(),
            "p50_ns": self.percentile(50.0),
            "p99_ns": self.percentile(99.0),
            "max_ns": self.max(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn buckets_cover_values_with_bounded_error() {
        let mut previous_upper = None;
        for bucket in 0..BUCKETS - 1 {
            let top = upper(bucket);
            if let Some(prev) = previous_upper {
                // Adjacent buckets tile the value range without gaps or overlap.
                assert_eq!(index(prev + 1), bucket);
            }
            assert_eq!(index(top), bucket);
            previous_upper = Some(top);
        }
        for shift in 4..44 {
            for sub in 0..SUB_COUNT {
                let value = (SUB_COUNT + sub) << (shift - SUB_BITS);
                let reported = upper(index(value));
                assert!(reported >= value);
                assert!((reported - value) as f64 <= value as f64 / 8.0 + 1.0, "{value} -> {reported}");
            }
        }
        assert_eq!(index(u64::MAX), BUCKETS - 1);
    }

    #[test]
    fn percentiles_follow_the_distribution() {
        let histogram = Histogram::new();
        assert_eq!(histogram.percentile(50.0), 0);
        for value in 1..=1000u64 {
            histogram.record(value * 1000);
        }
        let p50 = histogram.percentile(50.0);
        let p99 = histogram.percentile(99.0);
        assert!((500_000..=565_000).contains(&p50), "{p50}");
        assert!((990_000..=1_000_000).contains(&p99), "{p99}");
        assert_eq!(histogram.max(), 1_000_000);
        assert_eq!(histogram.percentile(100.0), 1_000_000);
        assert_eq!(histogram.count(), 1000);
    }
}
