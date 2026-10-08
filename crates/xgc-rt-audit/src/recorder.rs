//! Bounded audit evidence. Producers encode one fixed record, try to enqueue
//! without waiting, and count loss. Four fixed writers share a per-node disk
//! allowance; exhaustion retains the prefix rather than deleting old evidence.
//! `storage.json` declares memory/disk bounds, drops and actual writer errors.

use std::collections::BTreeMap;
use std::fs::{self, OpenOptions};
use std::io::{self, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender, TrySendError};
use std::sync::{Arc, Mutex, RwLock};
use std::thread::JoinHandle;

use serde::{Deserialize, Serialize};
use xgc_rt_core::audit::{AuditSink, OverflowSite};
use xgc_rt_core::clock::Clock;
use xgc_rt_core::envelope::Header;
use xgc_rt_core::manifest::{AUDIT_METADATA_RESERVED_BYTES, DEFAULT_AUDIT_MAX_BYTES};
use xgc_rt_core::{ChannelId, OriginId};

use crate::record::{Kind, Record, FORMAT, RECORD_LEN};

pub const QUEUE_RECORDS: usize = 1 << 16;
pub const MAX_QUEUE_RECORDS: usize = 1 << 20;
const META_MAX_BYTES: usize = 64 * 1024;
const STORAGE_MAX_BYTES: usize = 32 * 1024;
const MAX_WRITERS: usize = 4;
const MAX_ERROR_BYTES: usize = 1024;

/// The disk allowance covers every append plus 128 KiB of metadata. A failed
/// write still consumes its reservation: the accounting is conservative and
/// never permits retries to grow the directory beyond the declared quota.
pub struct AuditBudget {
    max_bytes: u64,
    reserved: AtomicU64,
    writers: Mutex<BTreeMap<String, Arc<WriterState>>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StorageSnapshot {
    pub max_bytes: u64,
    pub metadata_reserved_bytes: u64,
    pub data_bytes_reserved: u64,
    pub writers: BTreeMap<String, WriterSnapshot>,
}

impl StorageSnapshot {
    pub fn complete(&self) -> bool {
        self.writers
            .values()
            .all(|w| w.finished && w.dropped_entries() == 0 && w.write_error.is_none())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WriterSnapshot {
    pub queue_capacity: usize,
    pub max_entry_bytes: usize,
    pub accepted: u64,
    pub queue_drops: u64,
    pub contention_drops: u64,
    pub oversize_drops: u64,
    pub closed_drops: u64,
    pub quota_drops: u64,
    pub io_drops: u64,
    pub entries_written: u64,
    /// Bytes accepted by the underlying writer, including any partial tail.
    pub bytes_written: u64,
    pub write_error: Option<String>,
    pub finished: bool,
}

impl WriterSnapshot {
    pub fn dropped_entries(&self) -> u64 {
        self.queue_drops
            .saturating_add(self.contention_drops)
            .saturating_add(self.oversize_drops)
            .saturating_add(self.closed_drops)
            .saturating_add(self.quota_drops)
            .saturating_add(self.io_drops)
    }
}

/// Registered once, off the realtime path. All producer counters are atomic;
/// only the disk writer sets the bounded error string.
pub struct WriterState {
    queue_capacity: usize,
    max_entry_bytes: usize,
    accepted: AtomicU64,
    queue_drops: AtomicU64,
    contention_drops: AtomicU64,
    oversize_drops: AtomicU64,
    closed_drops: AtomicU64,
    quota_drops: AtomicU64,
    io_drops: AtomicU64,
    entries_written: AtomicU64,
    bytes_written: AtomicU64,
    failed: AtomicBool,
    write_error: Mutex<Option<String>>,
    finished: AtomicBool,
}

impl AuditBudget {
    pub fn new(max_bytes: u64) -> io::Result<Arc<Self>> {
        if max_bytes < AUDIT_METADATA_RESERVED_BYTES {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "audit quota is smaller than metadata reservation",
            ));
        }
        Ok(Arc::new(Self {
            max_bytes,
            reserved: AtomicU64::new(0),
            writers: Mutex::new(BTreeMap::new()),
        }))
    }

    pub fn register_writer(
        &self,
        name: &str,
        capacity: usize,
        max_entry_bytes: usize,
    ) -> io::Result<Arc<WriterState>> {
        let mut writers = self.writers.lock().unwrap();
        if name.len() > 64 || writers.len() >= MAX_WRITERS || writers.contains_key(name) {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "audit writer name repeated or fixed writer bound exceeded",
            ));
        }
        let state = Arc::new(WriterState {
            queue_capacity: capacity,
            max_entry_bytes,
            accepted: AtomicU64::new(0),
            queue_drops: AtomicU64::new(0),
            contention_drops: AtomicU64::new(0),
            oversize_drops: AtomicU64::new(0),
            closed_drops: AtomicU64::new(0),
            quota_drops: AtomicU64::new(0),
            io_drops: AtomicU64::new(0),
            entries_written: AtomicU64::new(0),
            bytes_written: AtomicU64::new(0),
            failed: AtomicBool::new(false),
            write_error: Mutex::new(None),
            finished: AtomicBool::new(false),
        });
        writers.insert(name.into(), state.clone());
        Ok(state)
    }

    fn reserve(&self, bytes: u64) -> bool {
        let mut used = self.reserved.load(Ordering::Relaxed);
        loop {
            let Some(next) = used
                .checked_add(bytes)
                .filter(|&n| n <= self.max_bytes - AUDIT_METADATA_RESERVED_BYTES)
            else {
                return false;
            };
            match self.reserved.compare_exchange_weak(
                used,
                next,
                Ordering::Relaxed,
                Ordering::Relaxed,
            ) {
                Ok(_) => return true,
                Err(current) => used = current,
            }
        }
    }

    pub fn snapshot(&self) -> StorageSnapshot {
        StorageSnapshot {
            max_bytes: self.max_bytes,
            metadata_reserved_bytes: AUDIT_METADATA_RESERVED_BYTES,
            data_bytes_reserved: self.reserved.load(Ordering::Relaxed),
            writers: self
                .writers
                .lock()
                .unwrap()
                .iter()
                .map(|(name, state)| (name.clone(), state.snapshot()))
                .collect(),
        }
    }
}

impl WriterState {
    pub fn accepted(&self) {
        self.accepted.fetch_add(1, Ordering::Relaxed);
    }
    pub fn queue_drop(&self) {
        self.queue_drops.fetch_add(1, Ordering::Relaxed);
    }
    pub fn contention_drop(&self) {
        self.contention_drops.fetch_add(1, Ordering::Relaxed);
    }
    pub fn oversize_drop(&self) {
        self.oversize_drops.fetch_add(1, Ordering::Relaxed);
    }
    pub fn closed_drop(&self) {
        self.closed_drops.fetch_add(1, Ordering::Relaxed);
    }

    pub fn snapshot(&self) -> WriterSnapshot {
        let get = |v: &AtomicU64| v.load(Ordering::Relaxed);
        WriterSnapshot {
            queue_capacity: self.queue_capacity,
            max_entry_bytes: self.max_entry_bytes,
            accepted: get(&self.accepted),
            queue_drops: get(&self.queue_drops),
            contention_drops: get(&self.contention_drops),
            oversize_drops: get(&self.oversize_drops),
            closed_drops: get(&self.closed_drops),
            quota_drops: get(&self.quota_drops),
            io_drops: get(&self.io_drops),
            entries_written: get(&self.entries_written),
            bytes_written: get(&self.bytes_written),
            write_error: self.write_error.lock().unwrap().clone(),
            finished: self.finished.load(Ordering::Acquire),
        }
    }

    pub fn record_error(&self, error: &io::Error) {
        self.failed.store(true, Ordering::Release);
        let mut first = self.write_error.lock().unwrap();
        if first.is_none() {
            let mut text = error.to_string();
            let mut end = text.len().min(MAX_ERROR_BYTES);
            while !text.is_char_boundary(end) {
                end -= 1;
            }
            text.truncate(end);
            *first = Some(text);
        }
    }

    /// Only the background writer calls this. Continue consuming after quota
    /// exhaustion or an I/O error so finish can drain the admitted queue.
    pub fn write_entry<W: Write>(&self, out: &mut W, bytes: &[u8], budget: &AuditBudget) -> bool {
        if self.failed.load(Ordering::Acquire) {
            self.io_drops.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        if !budget.reserve(bytes.len() as u64) {
            self.quota_drops.fetch_add(1, Ordering::Relaxed);
            return false;
        }
        let mut remaining = bytes;
        while !remaining.is_empty() {
            match out.write(remaining) {
                Ok(0) => {
                    self.record_error(&io::Error::new(
                        io::ErrorKind::WriteZero,
                        "audit writer accepted zero bytes",
                    ));
                    self.io_drops.fetch_add(1, Ordering::Relaxed);
                    return false;
                }
                Ok(n) => {
                    self.bytes_written.fetch_add(n as u64, Ordering::Relaxed);
                    remaining = &remaining[n..];
                }
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) => {
                    self.record_error(&e);
                    self.io_drops.fetch_add(1, Ordering::Relaxed);
                    return false;
                }
            }
        }
        self.entries_written.fetch_add(1, Ordering::Relaxed);
        true
    }

    pub fn flush<W: Write>(&self, out: &mut W) {
        if let Err(e) = out.flush() {
            self.record_error(&e);
        }
        self.mark_finished();
    }

    pub fn mark_finished(&self) {
        self.finished.store(true, Ordering::Release);
    }

    pub fn result(&self) -> io::Result<WriterSnapshot> {
        let snapshot = self.snapshot();
        match &snapshot.write_error {
            Some(error) => Err(io::Error::other(error.clone())),
            None => Ok(snapshot),
        }
    }
}

/// Per-node metadata retains the existing wire format. `complete=false`
/// invalidates a run with any evidence loss; detailed facts are storage.json.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct NodeMeta {
    pub format: String,
    pub session: String,
    pub node: String,
    pub node_id: u16,
    pub roster: Vec<String>,
    pub channels: Vec<String>,
    pub clock_domain: String,
    pub audit_queue_drops: u64,
    pub records_written: u64,
    pub complete: bool,
}

pub struct FileAudit {
    node: u16,
    clock: Arc<dyn Clock>,
    tx: RwLock<Option<SyncSender<[u8; RECORD_LEN]>>>,
    writer: Mutex<Option<JoinHandle<()>>>,
    state: Arc<WriterState>,
    budget: Arc<AuditBudget>,
    dir: PathBuf,
    meta: Mutex<NodeMeta>,
}

impl FileAudit {
    pub fn create(run_dir: &Path, meta: NodeMeta, clock: Arc<dyn Clock>) -> io::Result<Self> {
        Self::create_with_capacity(run_dir, meta, clock, QUEUE_RECORDS)
    }

    pub fn create_with_capacity(
        run_dir: &Path,
        meta: NodeMeta,
        clock: Arc<dyn Clock>,
        capacity: usize,
    ) -> io::Result<Self> {
        Self::create_with_budget(
            run_dir,
            meta,
            clock,
            capacity,
            AuditBudget::new(DEFAULT_AUDIT_MAX_BYTES)?,
        )
    }

    pub fn create_with_budget(
        run_dir: &Path,
        meta: NodeMeta,
        clock: Arc<dyn Clock>,
        capacity: usize,
        budget: Arc<AuditBudget>,
    ) -> io::Result<Self> {
        if capacity == 0 || capacity > MAX_QUEUE_RECORDS {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "audit record queue must have 1..=1048576 entries",
            ));
        }
        let dir = run_dir.join(&meta.node);
        // An attempt is its own evidence directory. Never truncate a previous
        // attempt, including a partially created one.
        fs::create_dir_all(run_dir)?;
        fs::create_dir(&dir)?;
        let meta = NodeMeta {
            format: FORMAT.into(),
            complete: false,
            audit_queue_drops: 0,
            records_written: 0,
            ..meta
        };
        write_meta(&dir, &meta)?;
        let state = budget.register_writer("records", capacity, RECORD_LEN)?;
        write_storage(&dir, &budget.snapshot())?;
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(dir.join("records.bin"))?;
        let (tx, rx) = sync_channel::<[u8; RECORD_LEN]>(capacity);
        let writer_state = state.clone();
        let writer_budget = budget.clone();
        let writer = std::thread::Builder::new()
            .name("xgc-audit-writer".into())
            .spawn(move || {
                for rec in rx {
                    writer_state.write_entry(&mut file, &rec, &writer_budget);
                }
                writer_state.flush(&mut file);
            })?;
        Ok(Self {
            node: meta.node_id,
            clock,
            tx: RwLock::new(Some(tx)),
            writer: Mutex::new(Some(writer)),
            state,
            budget,
            dir,
            meta: Mutex::new(meta),
        })
    }

    pub fn dir(&self) -> &Path {
        &self.dir
    }
    pub fn budget(&self) -> Arc<AuditBudget> {
        self.budget.clone()
    }
    pub fn storage_snapshot(&self) -> StorageSnapshot {
        self.budget.snapshot()
    }

    fn push(&self, rec: Record) {
        let Ok(guard) = self.tx.try_read() else {
            self.state.contention_drop();
            return;
        };
        let Some(tx) = guard.as_ref() else {
            self.state.closed_drop();
            return;
        };
        match tx.try_send(rec.encode()) {
            Ok(()) => self.state.accepted(),
            Err(TrySendError::Full(_)) => self.state.queue_drop(),
            Err(TrySendError::Disconnected(_)) => self.state.closed_drop(),
        }
    }

    /// Close admission before draining; no producer retains a raw sender.
    /// Idempotent, including failure: subsequent calls return the same error.
    /// Queue/quota loss finalizes incomplete metadata; I/O failure is returned.
    pub fn finish(&self) -> io::Result<NodeMeta> {
        let mut writer = self.writer.lock().unwrap();
        drop(self.tx.write().unwrap().take());
        if let Some(handle) = writer.take() {
            if handle.join().is_err() {
                self.state
                    .record_error(&io::Error::other("audit writer panicked"));
                self.state.finished.store(true, Ordering::Release);
            }
        }
        let snapshot = self.state.snapshot();
        let storage = self.budget.snapshot();
        let mut meta = self.meta.lock().unwrap();
        meta.records_written = snapshot.entries_written;
        meta.audit_queue_drops = snapshot
            .queue_drops
            .saturating_add(snapshot.contention_drops)
            .saturating_add(snapshot.closed_drops);
        meta.complete = storage.complete();
        // Save the failure facts before propagating a writer exception.
        // If metadata itself cannot be saved, the caller sees that exception.
        if let Err(e) = write_storage(&self.dir, &storage) {
            self.state.record_error(&e);
            meta.complete = false;
        }
        if let Err(e) = write_meta(&self.dir, &meta) {
            self.state.record_error(&e);
            meta.complete = false;
        }
        // A metadata error is latched like a data error. Try to expose it in
        // the other metadata file, but never return success after a retry.
        if self.state.snapshot().write_error.is_some() {
            let _ = write_storage(&self.dir, &self.budget.snapshot());
            let _ = write_meta(&self.dir, &meta);
        }
        self.state.result()?;
        Ok(meta.clone())
    }
}

impl Drop for FileAudit {
    fn drop(&mut self) {
        let _ = self.finish();
    }
}

fn bounded_json<T: Serialize>(value: &T, maximum: usize) -> io::Result<Vec<u8>> {
    struct Limited {
        bytes: Vec<u8>,
        maximum: usize,
    }
    impl Write for Limited {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if bytes.len() > self.maximum - self.bytes.len() {
                return Err(io::Error::new(
                    io::ErrorKind::InvalidData,
                    "audit metadata exceeds its fixed reservation",
                ));
            }
            self.bytes.extend_from_slice(bytes);
            Ok(bytes.len())
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }
    let mut out = Limited {
        bytes: Vec::new(),
        maximum,
    };
    serde_json::to_writer_pretty(&mut out, value).map_err(io::Error::other)?;
    out.write_all(b"\n")?;
    Ok(out.bytes)
}

fn write_meta(dir: &Path, meta: &NodeMeta) -> io::Result<()> {
    fs::write(dir.join("meta.json"), bounded_json(meta, META_MAX_BYTES)?)
}

fn write_storage(dir: &Path, storage: &StorageSnapshot) -> io::Result<()> {
    fs::write(
        dir.join("storage.json"),
        bounded_json(storage, STORAGE_MAX_BYTES)?,
    )
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

#[cfg(test)]
mod bounded_tests {
    use super::*;
    use xgc_rt_core::clock::WallClock;

    fn directory(name: &str) -> PathBuf {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/bounded-audit-tests")
            .join(format!(
                "{name}-{}-{}",
                std::process::id(),
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_nanos()
            ));
        fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn meta() -> NodeMeta {
        NodeMeta {
            format: String::new(),
            session: "bounded".into(),
            node: "node".into(),
            node_id: 0,
            roster: vec!["node".into()],
            channels: vec!["data".into()],
            clock_domain: "wall".into(),
            audit_queue_drops: 0,
            records_written: 0,
            complete: false,
        }
    }

    #[test]
    fn records_and_logs_share_a_real_disk_quota_and_loss_invalidates_merge() {
        let run = directory("shared-quota");
        let maximum = AUDIT_METADATA_RESERVED_BYTES + 2 * RECORD_LEN as u64 + 16;
        let budget = AuditBudget::new(maximum).unwrap();
        let audit = FileAudit::create_with_budget(
            &run,
            meta(),
            Arc::new(WallClock::new(0)),
            16,
            budget.clone(),
        )
        .unwrap();
        let health = budget.register_writer("health", 1, 16).unwrap();
        let mut log = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(audit.dir().join("health.jsonl"))
            .unwrap();
        health.accepted();
        health.write_entry(&mut log, b"0123456789abcde\n", &budget);
        health.flush(&mut log);
        for t in 0..10 {
            audit.subscribed(0, 0, t);
        }
        let complete = audit.finish().unwrap();
        assert!(!complete.complete);
        assert_eq!(complete.records_written, 2);
        assert_eq!(
            complete.audit_queue_drops, 0,
            "this is disk exhaustion, not queue overflow"
        );
        let storage = audit.storage_snapshot();
        assert_eq!(storage.data_bytes_reserved, 2 * RECORD_LEN as u64 + 16);
        assert_eq!(storage.writers["records"].quota_drops, 8);
        assert_eq!(
            storage.writers["records"].bytes_written,
            2 * RECORD_LEN as u64
        );
        let stored: StorageSnapshot =
            serde_json::from_slice(&fs::read(audit.dir().join("storage.json")).unwrap()).unwrap();
        assert_eq!(stored.writers["records"].quota_drops, 8);
        let total: u64 = fs::read_dir(audit.dir())
            .unwrap()
            .map(|e| e.unwrap().metadata().unwrap().len())
            .sum();
        assert!(
            total <= maximum,
            "all files use {total} bytes, declared maximum {maximum}"
        );
        assert_eq!(
            fs::metadata(audit.dir().join("records.bin")).unwrap().len(),
            2 * RECORD_LEN as u64
        );
        let report = crate::merge::merge_run(&run, crate::merge::MergeOptions::default()).unwrap();
        assert!(
            !report.valid,
            "truncated domain evidence must never merge as valid"
        );
    }

    #[test]
    fn another_attempt_cannot_truncate_existing_evidence() {
        let run = directory("preserve-evidence");
        let audit = FileAudit::create(&run, meta(), Arc::new(WallClock::new(0))).unwrap();
        audit.subscribed(0, 0, 7);
        audit.finish().unwrap();
        let records = fs::read(audit.dir().join("records.bin")).unwrap();
        let metadata = fs::read(audit.dir().join("meta.json")).unwrap();
        assert!(FileAudit::create(&run, meta(), Arc::new(WallClock::new(0))).is_err());
        assert_eq!(fs::read(audit.dir().join("records.bin")).unwrap(), records);
        assert_eq!(fs::read(audit.dir().join("meta.json")).unwrap(), metadata);
    }

    #[test]
    fn metadata_finish_failure_is_latched_after_the_obstruction_is_removed() {
        let run = directory("metadata-error");
        let audit = FileAudit::create(&run, meta(), Arc::new(WallClock::new(0))).unwrap();
        audit.subscribed(0, 0, 7);
        let storage = audit.dir().join("storage.json");
        fs::rename(&storage, audit.dir().join("storage.previous.json")).unwrap();
        fs::create_dir(&storage).unwrap();
        let error = audit.finish().unwrap_err().to_string();
        fs::remove_dir(&storage).unwrap(); // Empty obstruction owned by this fixture.
        assert_eq!(audit.finish().unwrap_err().to_string(), error);
        let stored: StorageSnapshot = serde_json::from_slice(&fs::read(storage).unwrap()).unwrap();
        assert!(stored.writers["records"].write_error.is_some());
        let stored_meta: NodeMeta =
            serde_json::from_slice(&fs::read(audit.dir().join("meta.json")).unwrap()).unwrap();
        assert!(!stored_meta.complete);
        assert_eq!(stored_meta.records_written, 1);
    }

    #[cfg(target_os = "linux")]
    #[test]
    fn real_disk_full_is_saved_and_finish_never_turns_it_into_success() {
        let run = directory("disk-full");
        let dir = run.join("node");
        fs::create_dir(&dir).unwrap();
        let budget = AuditBudget::new(DEFAULT_AUDIT_MAX_BYTES).unwrap();
        let state = budget.register_writer("records", 4, RECORD_LEN).unwrap();
        let (tx, rx) = sync_channel::<[u8; RECORD_LEN]>(4);
        let worker_state = state.clone();
        let worker_budget = budget.clone();
        let mut full = OpenOptions::new().write(true).open("/dev/full").unwrap();
        let writer = std::thread::spawn(move || {
            for record in rx {
                worker_state.write_entry(&mut full, &record, &worker_budget);
            }
            worker_state.flush(&mut full);
        });
        let audit = FileAudit {
            node: 0,
            clock: Arc::new(WallClock::new(0)),
            tx: RwLock::new(Some(tx)),
            writer: Mutex::new(Some(writer)),
            state,
            budget,
            dir,
            meta: Mutex::new(meta()),
        };
        for t in 0..3 {
            audit.subscribed(0, 0, t);
        }
        let first = audit.finish().unwrap_err().to_string();
        assert_eq!(audit.finish().unwrap_err().to_string(), first);
        let stored: StorageSnapshot =
            serde_json::from_slice(&fs::read(audit.dir().join("storage.json")).unwrap()).unwrap();
        assert!(stored.writers["records"]
            .write_error
            .as_ref()
            .unwrap()
            .contains("No space left"));
        assert_eq!(stored.writers["records"].entries_written, 0);
        assert_eq!(stored.writers["records"].bytes_written, 0);
        assert_eq!(stored.writers["records"].io_drops, 3);
        assert!(stored.writers["records"].finished);
        let stored_meta: NodeMeta =
            serde_json::from_slice(&fs::read(audit.dir().join("meta.json")).unwrap()).unwrap();
        assert!(!stored_meta.complete);
        assert_eq!(stored_meta.records_written, 0);
    }

    struct PartialFailure(Vec<u8>);
    impl Write for PartialFailure {
        fn write(&mut self, bytes: &[u8]) -> io::Result<usize> {
            if self.0.is_empty() {
                self.0.extend_from_slice(&bytes[..3]);
                Ok(3)
            } else {
                Err(io::Error::other("partial-tail"))
            }
        }
        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    #[test]
    fn partial_write_counts_actual_bytes_without_inventing_a_completed_entry() {
        let budget = AuditBudget::new(DEFAULT_AUDIT_MAX_BYTES).unwrap();
        let state = budget.register_writer("partial", 1, 10).unwrap();
        let mut out = PartialFailure(Vec::new());
        state.write_entry(&mut out, b"0123456789", &budget);
        state.flush(&mut out);
        let snapshot = state.snapshot();
        assert_eq!(snapshot.bytes_written, 3);
        assert_eq!(snapshot.entries_written, 0);
        assert_eq!(snapshot.io_drops, 1);
        assert_eq!(out.0, b"012");
        assert_eq!(
            budget.snapshot().data_bytes_reserved,
            10,
            "a failed append retains its conservative reservation"
        );
        assert!(state.result().is_err());
    }
}
