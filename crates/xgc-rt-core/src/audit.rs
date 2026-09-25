//! The audit hook. The host calls it beside every send, receive and
//! consumption. `xgc-rt-audit` records these calls losslessly, and the
//! definitions live in docs/audit-definitions.md (`audit-def/1`).

use crate::envelope::Header;
use crate::{ChannelId, OriginId};

/// Where a bounded queue overflowed. Every overflow is counted and never
/// silent.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OverflowSite {
    /// The host receive queue, between the transport and the executor.
    RxQueue = 1,
    /// A plugin in-port inbox. The oldest sample is dropped.
    Inbox = 2,
}

pub trait AuditSink: Send + Sync {
    /// This node subscribes to `channel` from `origin` from `t` on. Streams
    /// are defined by these records, so loss is computed only for streams
    /// that were subscribed.
    fn subscribed(&self, channel: ChannelId, origin: OriginId, t: i64);
    /// A frame handed to the transport. `header.t_tx` is the stamp.
    fn sent(&self, header: &Header);
    /// A frame that decoded and verified, stamped `t_rx` by the host sink.
    fn received(&self, header: &Header, t_rx: i64);
    /// A frame that failed decode or verification, counted per node.
    fn rejected(&self, t_rx: i64, frame_len: usize);
    /// A module read this sample for the first time at `t_consume`.
    fn consumed(&self, header: &Header, t_consume: i64);
    fn overflow(&self, site: OverflowSite, channel: ChannelId, origin: OriginId, t: i64);
}

/// Audit disabled. It is for unit tests of components other than the audit.
#[derive(Debug, Default)]
pub struct NullAudit;

impl AuditSink for NullAudit {
    fn subscribed(&self, _: ChannelId, _: OriginId, _: i64) {}
    fn sent(&self, _: &Header) {}
    fn received(&self, _: &Header, _: i64) {}
    fn rejected(&self, _: i64, _: usize) {}
    fn consumed(&self, _: &Header, _: i64) {}
    fn overflow(&self, _: OverflowSite, _: ChannelId, _: OriginId, _: i64) {}
}
