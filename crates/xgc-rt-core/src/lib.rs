//! The shared skeleton every XGC2 Sync Runtime module builds on.
//!
//! Hosts, transports and audit recorders depend on the interfaces defined
//! here. Domain plugins depend only on `xgc-rt-abi`.

pub mod audit;
pub mod clock;
pub mod envelope;
pub mod lifecycle;
pub mod manifest;
pub mod transport;

/// Interned channel id: the channel's index in the Session manifest.
pub type ChannelId = u32;
/// Interned node id: the node's index in the Session roster.
pub type OriginId = u16;
