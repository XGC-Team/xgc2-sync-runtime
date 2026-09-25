//! The one transport interface. Loopback, shared memory and Zenoh implement
//! it, and the host is the only caller. A transport moves opaque envelope
//! frames. It never stamps, audits or decodes them: the host does that
//! beside every send and receive, so every transport is audited identically.

use std::sync::Arc;

use crate::{ChannelId, OriginId};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Qos {
    /// Best-effort, drop on congestion, real-time priority, express.
    Control,
    /// Best-effort, drop, high priority.
    State,
    /// Reliable, bounded block.
    Event,
    /// Reliable, block, low priority.
    Bulk,
}

impl Qos {
    pub fn from_abi(value: i32) -> Option<Self> {
        match value {
            0 => Some(Qos::Control),
            1 => Some(Qos::State),
            2 => Some(Qos::Event),
            3 => Some(Qos::Bulk),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChannelSpec {
    pub id: ChannelId,
    pub name: String,
    pub qos: Qos,
}

#[derive(Debug, Clone)]
pub struct TransportContext {
    pub session: String,
    pub node: String,
    pub node_id: OriginId,
    pub roster: Vec<String>,
    pub channels: Vec<ChannelSpec>,
}

impl TransportContext {
    pub fn channel(&self, id: ChannelId) -> Option<&ChannelSpec> {
        self.channels.get(id as usize)
    }
}

/// Called by the transport, on its own IO thread, with each received frame.
/// The host's sink stamps `t_rx` and enqueues the frame without blocking.
pub type RxSink = Arc<dyn Fn(&[u8]) + Send + Sync>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TransportError(pub String);

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for TransportError {}

pub trait Transport: Send {
    fn kind(&self) -> &'static str;
    fn open(&mut self, ctx: &TransportContext, sink: RxSink) -> Result<(), TransportError>;
    /// This node will publish `channel`.
    fn declare_out(&mut self, channel: ChannelId) -> Result<(), TransportError>;
    /// Deliver `channel` from exactly these origins.
    fn declare_in(&mut self, channel: ChannelId, origins: &[OriginId]) -> Result<(), TransportError>;
    fn send(&mut self, channel: ChannelId, frame: &[u8]) -> Result<(), TransportError>;
    fn close(&mut self) {}
}
