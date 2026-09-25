//! `FileAudit`: the recording [`AuditSink`]. Calls are encoded on the
//! caller's thread, which can be the executor or a transport IO thread, and
//! queued to one writer thread through a bounded channel. A full queue never
//! blocks the data path. The drop is counted, written to `meta.json`, and
//! the merged report marks the run invalid.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;

use serde::{Deserialize, Serialize};
use xgc_rt_core::audit::{AuditSink, OverflowSite};
use xgc_rt_core::clock::Clock;
use xgc_rt_core::envelope::Header;
use xgc_rt_core::{ChannelId, OriginId};

use crate::record::{Kind, Record, FORMAT, RECORD_LEN};

pub const QUEUE_RECORDS: usize = 1 << 16;

/// Per-node run metadata, written next to `records.bin`.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NodeMeta {
    pub format: String,
    pub session: String,
    pub node: String,
    pub node_id: u16,
    pub roster: Vec<String>,
    /// Channel names, indexed by channel id.
    pub channels: Vec<String>,
    pub clock_domain: String,
    /// Records the writer could not queue. Any nonzero value invalidates the run.
    pub audit_queue_drops: u64,
    pub records_written: u64,
    pub complete: bool,
}

pub struct FileAudit {
    node: u16,
    clock: Arc<dyn Clock>,
    tx: Mutex<Option<SyncSender<[u8; RECORD_LEN]>>>,
    writer: Mutex<Option<JoinHandle<std::io::Result<u64>>>>,
    drops: AtomicU64,
    dir: PathBuf,
    meta: Mutex<NodeMeta>,
}

impl FileAudit {
    /// Create `<run_dir>/<node>/` and start the writer.
    pub fn create(run_dir: &Path, meta: NodeMeta, clock: Arc<dyn Clock>) -> std::io::Result<Self> {
        Self::create_with_capacity(run_dir, meta, clock, QUEUE_RECORDS)
    }

    /// As [`FileAudit::create`], with an explicit writer queue capacity in
    /// records.
    pub fn create_with_capacity(run_dir: &Path, meta: NodeMeta, clock: Arc<dyn Clock>, capacity: usize) -> std::io::Result<Self> {
        let dir = run_dir.join(&meta.node);
        fs::create_dir_all(&dir)?;
        let file = File::create(dir.join("records.bin"))?;
        let (tx, rx) = sync_channel::<[u8; RECORD_LEN]>(capacity.max(1));
        let writer = std::thread::Builder::new().name("xgc-audit-writer".into()).spawn(move || {
            let mut out = BufWriter::with_capacity(1 << 20, file);
            let mut n = 0u64;
            for rec in rx {
                out.write_all(&rec)?;
                n += 1;
            }
            out.flush()?;
            Ok(n)
        })?;
        let meta = NodeMeta { format: FORMAT.into(), complete: false, audit_queue_drops: 0, records_written: 0, ..meta };
        write_meta(&dir, &meta)?;
        Ok(Self {
            node: meta.node_id,
            clock,
            tx: Mutex::new(Some(tx)),
            writer: Mutex::new(Some(writer)),
            drops: AtomicU64::new(0),
            dir,
            meta: Mutex::new(meta),
        })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }

    fn push(&self, rec: Record) {
        let guard = self.tx.lock().unwrap();
        let Some(tx) = guard.as_ref() else {
            self.drops.fetch_add(1, Ordering::Relaxed);
            return;
        };
        match tx.try_send(rec.encode()) {
            Ok(()) => {}
            Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                self.drops.fetch_add(1, Ordering::Relaxed);
            }
        }
    }

    /// Flush, stop the writer and finalize `meta.json`. It is idempotent.
    pub fn finish(&self) -> std::io::Result<NodeMeta> {
        drop(self.tx.lock().unwrap().take());
        let written = match self.writer.lock().unwrap().take() {
            Some(handle) => handle.join().map_err(|_| std::io::Error::other("audit writer panicked"))??,
            None => self.meta.lock().unwrap().records_written,
        };
        let mut meta = self.meta.lock().unwrap();
        meta.records_written = written;
        meta.audit_queue_drops = self.drops.load(Ordering::Relaxed);
        meta.complete = true;
        write_meta(&self.dir, &meta)?;
        Ok(meta.clone())
    }
}

impl Drop for FileAudit {
    fn drop(&mut self) {
        let _ = self.finish();
    }
}

fn write_meta(dir: &Path, meta: &NodeMeta) -> std::io::Result<()> {
    let text = serde_json::to_string_pretty(meta).map_err(std::io::Error::other)?;
    fs::write(dir.join("meta.json"), text + "\n")
}

impl AuditSink for FileAudit {
    fn subscribed(&self, channel: ChannelId, origin: OriginId, t: i64) {
        let mut r = Record::new(Kind::Subscribe, self.node);
        r.channel = channel;
        r.origin = origin;
        r.t_b = t;
        self.push(r);
    }

    fn sent(&self, h: &Header) {
        let mut r = Record::new(Kind::Tx, self.node);
        r.origin = h.origin;
        r.channel = h.channel;
        r.seq = h.seq;
        r.round = h.round;
        r.t_a = h.t_produce;
        r.t_b = h.t_tx;
        r.len = h.payload_len;
        r.bound_a = h.clock_bound_ns;
        self.push(r);
    }

    fn received(&self, h: &Header, t_rx: i64) {
        let mut r = Record::new(Kind::Rx, self.node);
        r.origin = h.origin;
        r.channel = h.channel;
        r.seq = h.seq;
        r.round = h.round;
        r.t_a = h.t_tx;
        r.t_b = t_rx;
        r.len = h.payload_len;
        r.bound_a = h.clock_bound_ns;
        r.bound_b = self.clock.bound_ns();
        self.push(r);
    }

    fn rejected(&self, t_rx: i64, frame_len: usize) {
        let mut r = Record::new(Kind::Reject, self.node);
        r.t_b = t_rx;
        r.len = u32::try_from(frame_len).unwrap_or(u32::MAX);
        self.push(r);
    }

    fn consumed(&self, h: &Header, t_consume: i64) {
        let mut r = Record::new(Kind::Consume, self.node);
        r.origin = h.origin;
        r.channel = h.channel;
        r.seq = h.seq;
        r.round = h.round;
        r.t_a = h.t_produce;
        r.t_b = t_consume;
        r.len = h.payload_len;
        self.push(r);
    }

    fn overflow(&self, site: OverflowSite, channel: ChannelId, origin: OriginId, t: i64) {
        let mut r = Record::new(Kind::Overflow, self.node);
        r.flags = site as u8;
        r.channel = channel;
        r.origin = origin;
        r.t_b = t;
        self.push(r);
    }
}
