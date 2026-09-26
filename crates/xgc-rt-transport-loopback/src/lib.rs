//! Loopback transport: every node in one process shares a [`LoopbackBus`].
//! Delivery is synchronous in the sender's `send`.
//!
//! The optional [`Impairment`] decides once per sample and receiver whether
//! to deliver, drop, duplicate, or hold for reordering, from a seeded PRNG.
//! It keeps exact ground truth per stream ([`Truth`]), so audit tests can
//! require *equality*, not closeness.
//!
//! Reordering: a held sample is released right after the next sample that
//! is actually delivered on the same stream, so it arrives after a higher
//! seq. At most one sample per stream is held at a time. Samples still held
//! at [`LoopbackBus::flush`] are released without counting as reordered.

use std::collections::{BTreeMap, HashSet};
use std::sync::{Arc, Mutex};

use xgc_rt_core::transport::{RxSink, Transport, TransportContext, TransportError};
use xgc_rt_core::{ChannelId, OriginId};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Impairment {
    pub drop: f64,
    pub duplicate: f64,
    pub reorder: f64,
    pub seed: u64,
}

/// Ground truth for one `(channel, origin, receiver)` stream.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Truth {
    pub offered: u64,
    pub dropped: u64,
    pub duplicated: u64,
    pub reordered: u64,
}

pub type StreamKey = (ChannelId, OriginId, OriginId);

#[derive(Default)]
struct Stream {
    rng: u64,
    held: Option<Vec<u8>>,
    truth: Truth,
}

#[derive(Default)]
struct Inner {
    sinks: BTreeMap<OriginId, RxSink>,
    subs: HashSet<StreamKey>,
    impairment: Option<Impairment>,
    streams: BTreeMap<StreamKey, Stream>,
}

#[derive(Default)]
pub struct LoopbackBus {
    inner: Mutex<Inner>,
}

impl LoopbackBus {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    pub fn with_impairment(impairment: Impairment) -> Arc<Self> {
        let bus = Self::new();
        bus.inner.lock().unwrap().impairment = Some(impairment);
        bus
    }

    /// Release held samples, which are then not counted as reordered.
    pub fn flush(&self) {
        let mut inner = self.inner.lock().unwrap();
        let Inner { sinks, streams, .. } = &mut *inner;
        for (&(_, _, receiver), stream) in streams.iter_mut() {
            if let (Some(frame), Some(sink)) = (stream.held.take(), sinks.get(&receiver)) {
                sink(&frame);
            }
        }
    }

    pub fn truth(&self) -> BTreeMap<StreamKey, Truth> {
        self.inner.lock().unwrap().streams.iter().map(|(k, s)| (*k, s.truth)).collect()
    }

    fn send(&self, origin: OriginId, channel: ChannelId, frame: &[u8]) {
        let mut inner = self.inner.lock().unwrap();
        let Inner { sinks, subs, impairment, streams } = &mut *inner;
        for (&receiver, sink) in sinks.iter() {
            let key = (channel, origin, receiver);
            if !subs.contains(&key) {
                continue;
            }
            let stream = streams.entry(key).or_insert_with(|| Stream { rng: seed_for(impairment, key), ..Stream::default() });
            stream.truth.offered += 1;
            let Some(imp) = impairment else {
                sink(frame);
                continue;
            };
            let roll = next_unit(&mut stream.rng);
            if roll < imp.drop {
                stream.truth.dropped += 1;
                continue;
            }
            if roll < imp.drop + imp.duplicate {
                stream.truth.duplicated += 1;
                sink(frame);
                sink(frame);
            } else if roll < imp.drop + imp.duplicate + imp.reorder && stream.held.is_none() {
                stream.held = Some(frame.to_vec());
                continue;
            } else {
                sink(frame);
            }
            if let Some(held) = stream.held.take() {
                stream.truth.reordered += 1;
                sink(&held);
            }
        }
    }
}

fn seed_for(impairment: &Option<Impairment>, key: StreamKey) -> u64 {
    let base = impairment.map_or(0, |i| i.seed);
    let mixed = base ^ ((key.0 as u64) << 32) ^ ((key.1 as u64) << 16) ^ key.2 as u64;
    splitmix(mixed) | 1
}

fn splitmix(mut z: u64) -> u64 {
    z = z.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// xorshift64*, mapped to [0, 1).
fn next_unit(state: &mut u64) -> f64 {
    let mut x = *state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    *state = x;
    (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
}

pub struct LoopbackTransport {
    bus: Arc<LoopbackBus>,
    node: Option<OriginId>,
    outs: HashSet<ChannelId>,
}

impl LoopbackTransport {
    pub fn new(bus: Arc<LoopbackBus>) -> Self {
        Self { bus, node: None, outs: HashSet::new() }
    }
}

impl Transport for LoopbackTransport {
    fn kind(&self) -> &str {
        "loopback"
    }

    fn open(&mut self, ctx: &TransportContext, sink: RxSink) -> Result<(), TransportError> {
        let mut inner = self.bus.inner.lock().unwrap();
        if inner.sinks.insert(ctx.node_id, sink).is_some() {
            return Err(TransportError(format!("node {} already opened on this bus", ctx.node)));
        }
        self.node = Some(ctx.node_id);
        Ok(())
    }

    fn declare_out(&mut self, channel: ChannelId) -> Result<(), TransportError> {
        self.outs.insert(channel);
        Ok(())
    }

    fn declare_in(&mut self, channel: ChannelId, origins: &[OriginId]) -> Result<(), TransportError> {
        let node = self.node.ok_or_else(|| TransportError("declare_in before open".into()))?;
        let mut inner = self.bus.inner.lock().unwrap();
        for &origin in origins {
            inner.subs.insert((channel, origin, node));
        }
        Ok(())
    }

    fn send(&mut self, channel: ChannelId, frame: &[u8]) -> Result<(), TransportError> {
        let node = self.node.ok_or_else(|| TransportError("send before open".into()))?;
        if !self.outs.contains(&channel) {
            return Err(TransportError(format!("channel {channel} was not declared for output")));
        }
        self.bus.send(node, channel, frame);
        Ok(())
    }

    fn close(&mut self) {
        if let Some(node) = self.node.take() {
            let mut inner = self.bus.inner.lock().unwrap();
            inner.sinks.remove(&node);
            inner.subs.retain(|&(_, _, receiver)| receiver != node);
        }
    }
}

impl Drop for LoopbackTransport {
    fn drop(&mut self) {
        self.close();
    }
}
