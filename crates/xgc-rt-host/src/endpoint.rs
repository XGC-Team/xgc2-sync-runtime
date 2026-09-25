//! The audited send and receive path. It is the only code that stamps,
//! encodes, decodes and audits frames, so every transport is measured the
//! same way and no module or transport can skip the audit.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use xgc_rt_core::audit::{AuditSink, OverflowSite};
use xgc_rt_core::clock::Clock;
use xgc_rt_core::envelope::{self, Header, FLAG_CLOCK_DEGRADED, FLAG_SIM_CLOCK};
use xgc_rt_core::transport::{RxSink, Transport, TransportContext, TransportError};
use xgc_rt_core::{ChannelId, OriginId};

pub const DEFAULT_RX_QUEUE: usize = 1 << 14;

#[derive(Debug, Clone)]
pub struct RxFrame {
    pub header: Header,
    pub payload: Vec<u8>,
    pub t_rx: i64,
}

struct RxQueue {
    frames: Mutex<VecDeque<RxFrame>>,
    ready: Condvar,
    capacity: usize,
}

pub struct Endpoint {
    node_id: OriginId,
    clock: Arc<dyn Clock>,
    audit: Arc<dyn AuditSink>,
    transport: Mutex<Box<dyn Transport>>,
    seqs: Mutex<HashMap<ChannelId, u64>>,
    rx: Arc<RxQueue>,
    clock_degraded: std::sync::atomic::AtomicBool,
}

impl Endpoint {
    /// Open `transport` with a sink that stamps `t_rx`, verifies, audits
    /// and queues each frame, never blocking the transport thread. A full
    /// queue drops the frame *without* an rx record, so the merge counts it
    /// lost, and records an overflow, which invalidates the run.
    pub fn open(
        mut transport: Box<dyn Transport>,
        ctx: &TransportContext,
        clock: Arc<dyn Clock>,
        audit: Arc<dyn AuditSink>,
        rx_capacity: usize,
    ) -> Result<Arc<Self>, TransportError> {
        let rx = Arc::new(RxQueue { frames: Mutex::new(VecDeque::new()), ready: Condvar::new(), capacity: rx_capacity.max(1) });
        let sink: RxSink = {
            let (clock, audit, rx) = (clock.clone(), audit.clone(), rx.clone());
            Arc::new(move |frame: &[u8]| {
                let t_rx = clock.now();
                match envelope::decode(frame) {
                    Err(_) => audit.rejected(t_rx, frame.len()),
                    Ok((header, payload)) => {
                        let mut q = rx.frames.lock().unwrap();
                        if q.len() >= rx.capacity {
                            drop(q);
                            audit.overflow(OverflowSite::RxQueue, header.channel, header.origin, t_rx);
                            return;
                        }
                        audit.received(&header, t_rx);
                        q.push_back(RxFrame { header, payload: payload.to_vec(), t_rx });
                        drop(q);
                        rx.ready.notify_one();
                    }
                }
            })
        };
        transport.open(ctx, sink)?;
        Ok(Arc::new(Self {
            node_id: ctx.node_id,
            clock,
            audit,
            transport: Mutex::new(transport),
            seqs: Mutex::new(HashMap::new()),
            rx,
            clock_degraded: std::sync::atomic::AtomicBool::new(false),
        }))
    }

    pub fn clock(&self) -> &Arc<dyn Clock> {
        &self.clock
    }

    pub fn audit(&self) -> &Arc<dyn AuditSink> {
        &self.audit
    }

    pub fn set_clock_degraded(&self, degraded: bool) {
        self.clock_degraded.store(degraded, std::sync::atomic::Ordering::Relaxed);
    }

    pub fn declare_out(&self, channel: ChannelId) -> Result<(), TransportError> {
        self.transport.lock().unwrap().declare_out(channel)
    }

    /// Subscribe and write the subscribe records that define the streams.
    pub fn declare_in(&self, channel: ChannelId, origins: &[OriginId]) -> Result<(), TransportError> {
        let t = self.clock.now();
        for &origin in origins {
            self.audit.subscribed(channel, origin, t);
        }
        self.transport.lock().unwrap().declare_in(channel, origins)
    }

    /// Stamp, encode, send, then audit. `t_produce` is when the module
    /// handed the sample over, and `t_tx` is taken just before the frame is
    /// encoded for the transport. A send the transport refuses is not
    /// audited as sent, and its seq is not reused.
    pub fn publish(&self, channel: ChannelId, round: u64, t_produce: i64, payload: &[u8]) -> Result<Header, TransportError> {
        let seq = {
            let mut seqs = self.seqs.lock().unwrap();
            let s = seqs.entry(channel).or_insert(0);
            *s += 1;
            *s
        };
        let mut flags = 0;
        if self.clock.domain() == xgc_rt_core::clock::ClockDomain::Sim {
            flags |= FLAG_SIM_CLOCK;
        }
        if self.clock_degraded.load(std::sync::atomic::Ordering::Relaxed) {
            flags |= FLAG_CLOCK_DEGRADED;
        }
        let mut header = Header {
            flags,
            channel,
            origin: self.node_id,
            seq,
            round,
            t_produce,
            t_tx: 0,
            clock_bound_ns: self.clock.bound_ns(),
            payload_len: payload.len() as u32,
        };
        header.t_tx = self.clock.now();
        let frame = envelope::encode(&header, payload).map_err(|e| TransportError(e.to_string()))?;
        self.transport.lock().unwrap().send(channel, &frame)?;
        self.audit.sent(&header);
        Ok(header)
    }

    /// Take every queued frame.
    pub fn drain(&self) -> Vec<RxFrame> {
        self.rx.frames.lock().unwrap().drain(..).collect()
    }

    /// Block until a frame is queued or `timeout` passes. It returns true
    /// when frames are waiting.
    pub fn wait(&self, timeout: Duration) -> bool {
        let q = self.rx.frames.lock().unwrap();
        if !q.is_empty() {
            return true;
        }
        let (q, _) = self.rx.ready.wait_timeout_while(q, timeout, |q| q.is_empty()).unwrap();
        !q.is_empty()
    }

    pub fn wait_ready(&self, timeout: Duration) -> bool {
        self.transport.lock().unwrap().wait_ready(timeout)
    }

    pub fn close(&self) {
        self.transport.lock().unwrap().close();
    }
}
