//! Typed channels: slot arenas with zero-copy handoff.
//!
//! Every channel owns one arena. A slot is a 64-byte header (sequence, stamp, commit time, pin
//! count) followed by the payload, rounded up to whole cache lines, so a slot never shares a
//! cache line with its neighbour and payloads are 64-byte aligned. Writers fill a slot in place
//! and publish it with a commit; readers borrow the slot in place until their step ends.
//!
//! * `State` channels keep the latest value. The arena holds `max_readers + 2` slots, so the
//!   single writer always finds a free one: at most `max_readers` slots are pinned, one is the
//!   latest and one is free. The writer never waits for a reader and a reader always gets the
//!   newest complete sample. Readers pin with the usual announce/re-check protocol on the
//!   `latest` index (sequentially consistent, see `StateRing::pin_latest`).
//! * `Event` channels are bounded broadcast FIFOs of `depth` slots. Several writers claim
//!   positions under a short lock, fill their slot and publish it; every reader has its own
//!   cursor and sees every event. A slot is reused only after all readers finished the step in
//!   which they read it, so borrowed events stay valid. The queue is lossless until the slowest
//!   reader is `depth` events behind; then `begin_write` fails and the drop is counted.

use std::alloc::{alloc_zeroed, dealloc, Layout};
use std::ptr::NonNull;
use std::sync::atomic::{AtomicBool, AtomicI64, AtomicU32, AtomicU64, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError, RwLock};

pub const CACHE_LINE: usize = 64;
const NO_SLOT: u32 = u32::MAX;
const FLAG_VOID: u32 = 1;
/// Upper bound on spin rounds before a state writer reports a stall. Reached only if the
/// pin accounting is broken; it keeps a bug from becoming a livelock.
const SPIN_LIMIT: u32 = 200_000;
/// The largest arena one channel may allocate.
pub const MAX_ARENA_BYTES: u64 = 64 << 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    State,
    Event,
}

impl Kind {
    pub fn as_str(self) -> &'static str {
        match self {
            Kind::State => "state",
            Kind::Event => "event",
        }
    }
}

/// What a channel carries; every port bound to it must match exactly.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PayloadSpec {
    pub schema: String,
    pub size: u32,
    pub align: u32,
}

/// Notified on the committing thread after each commit. Must not block.
pub trait Subscriber: Send + Sync {
    fn notify(&self, bit: u64, commit_ns: i64);
}

/// A borrowed sample. The pointer stays valid until the reader's `end_step`.
#[derive(Clone, Copy, Debug)]
pub struct View {
    pub data: *const u8,
    pub size: u32,
    pub seq: u64,
    pub stamp_ns: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WriteError {
    /// Event queue full; counted as a drop.
    Full,
    /// `begin_write` called again before commit or abort.
    Pending,
    /// `commit` without `begin_write`.
    NoPending,
    /// State writer found no free slot (pin accounting broken); counted.
    Stalled,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AttachError {
    TooManyReaders,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WriterError {
    /// A state channel has exactly one writer.
    StateHasWriter,
}

/// Bytes of memory a channel allocates: `slots` slots of one header and a payload rounded up to
/// whole cache lines. State channels have `max_readers + 2` slots, event channels `depth`.
pub fn arena_bytes(kind: Kind, payload_size: u32, depth: u32, max_readers: u32) -> u64 {
    let stride = (CACHE_LINE + (payload_size as usize).max(1).next_multiple_of(CACHE_LINE)) as u64;
    let slots = match kind {
        Kind::State => u64::from(max_readers) + 2,
        Kind::Event => u64::from(depth),
    };
    stride * slots
}

#[repr(C, align(64))]
struct SlotHeader {
    /// State: commit sequence of the content. Event: position + 1 once published.
    seq: AtomicU64,
    stamp_ns: AtomicI64,
    commit_ns: AtomicI64,
    pins: AtomicU32,
    flags: AtomicU32,
}
const _: () = assert!(std::mem::size_of::<SlotHeader>() == CACHE_LINE);

struct Arena {
    base: NonNull<u8>,
    layout: Layout,
    stride: usize,
    slots: usize,
}

// SAFETY: the arena is plain memory; all shared access goes through atomics (headers) or the
// slot protocols of the rings (payloads).
unsafe impl Send for Arena {}
unsafe impl Sync for Arena {}

impl Arena {
    fn new(slots: usize, payload: usize) -> Result<Arena, String> {
        let stride = CACHE_LINE + payload.max(1).next_multiple_of(CACHE_LINE);
        let layout = Layout::from_size_align(stride * slots, CACHE_LINE).map_err(|e| format!("arena layout: {e}"))?;
        // SAFETY: the layout has a nonzero size (at least one slot of 128 bytes).
        let base = NonNull::new(unsafe { alloc_zeroed(layout) })
            .ok_or_else(|| format!("cannot allocate {} bytes for a channel", layout.size()))?;
        Ok(Arena { base, layout, stride, slots })
    }

    fn header(&self, slot: u32) -> &SlotHeader {
        debug_assert!((slot as usize) < self.slots);
        // SAFETY: in bounds; zeroed memory is a valid SlotHeader and headers are only accessed
        // through atomics.
        unsafe { &*(self.base.as_ptr().add(slot as usize * self.stride) as *const SlotHeader) }
    }

    fn payload(&self, slot: u32) -> *mut u8 {
        debug_assert!((slot as usize) < self.slots);
        // SAFETY: in bounds.
        unsafe { self.base.as_ptr().add(slot as usize * self.stride + CACHE_LINE) }
    }
}

impl Drop for Arena {
    fn drop(&mut self) {
        // SAFETY: allocated in `new` with this layout.
        unsafe { dealloc(self.base.as_ptr(), self.layout) }
    }
}

fn lock<T>(mutex: &Mutex<T>) -> MutexGuard<'_, T> {
    mutex.lock().unwrap_or_else(PoisonError::into_inner)
}

struct StateRing {
    arena: Arena,
    latest: AtomicU32,
    /// Sequence of the latest commit; readers compare it with what they consumed.
    seq: AtomicU64,
}

impl StateRing {
    fn new(max_readers: u32, payload: usize) -> Result<StateRing, String> {
        Ok(StateRing { arena: Arena::new(max_readers as usize + 2, payload)?, latest: AtomicU32::new(NO_SLOT), seq: AtomicU64::new(0) })
    }

    /// Single writer: pick a slot that is neither the latest nor pinned.
    fn begin(&self) -> Option<u32> {
        let current = self.latest.load(Ordering::Relaxed);
        for _ in 0..SPIN_LIMIT {
            for slot in 0..self.arena.slots as u32 {
                if slot != current && self.arena.header(slot).pins.load(Ordering::SeqCst) == 0 {
                    return Some(slot);
                }
            }
            // A reader may be moving its pin between slots; one more round finds the free slot.
            std::hint::spin_loop();
        }
        None
    }

    fn commit(&self, slot: u32, stamp_ns: i64, now_ns: i64) -> u64 {
        let seq = self.seq.load(Ordering::Relaxed) + 1;
        let header = self.arena.header(slot);
        header.stamp_ns.store(stamp_ns, Ordering::Relaxed);
        header.commit_ns.store(now_ns, Ordering::Relaxed);
        header.seq.store(seq, Ordering::Relaxed);
        // Publication: payload and header writes happen-before any reader that observes `slot`.
        self.latest.store(slot, Ordering::SeqCst);
        self.seq.store(seq, Ordering::Release);
        seq
    }

    /// Announce, then re-check: if `latest` still names the slot after the pin became
    /// visible, the writer cannot select it until the pin is released (its scan reads the
    /// pins after publishing a newer latest, both sequentially consistent).
    fn pin_latest(&self) -> Option<u32> {
        loop {
            let slot = self.latest.load(Ordering::Acquire);
            if slot == NO_SLOT {
                return None;
            }
            let header = self.arena.header(slot);
            header.pins.fetch_add(1, Ordering::SeqCst);
            if self.latest.load(Ordering::SeqCst) == slot {
                return Some(slot);
            }
            header.pins.fetch_sub(1, Ordering::Release);
        }
    }

    fn unpin(&self, slot: u32) {
        self.arena.header(slot).pins.fetch_sub(1, Ordering::Release);
    }

    fn view(&self, slot: u32, size: u32) -> View {
        let header = self.arena.header(slot);
        View {
            data: self.arena.payload(slot),
            size,
            seq: header.seq.load(Ordering::Relaxed),
            stamp_ns: header.stamp_ns.load(Ordering::Relaxed),
        }
    }
}

/// A reader's published progress; writers must not reuse slots at or after `next`.
struct Cursor {
    next: AtomicU64,
}

struct Claim {
    head: u64,
    cursors: Vec<Arc<Cursor>>,
}

struct EventRing {
    arena: Arena,
    depth: u64,
    claim: Mutex<Claim>,
    /// Mirror of `Claim::head` for lock-free reads.
    head: AtomicU64,
}

impl EventRing {
    fn new(depth: u32, payload: usize) -> Result<EventRing, String> {
        Ok(EventRing {
            arena: Arena::new(depth as usize, payload)?,
            depth: u64::from(depth),
            claim: Mutex::new(Claim { head: 0, cursors: Vec::new() }),
            head: AtomicU64::new(0),
        })
    }

    fn slot(&self, position: u64) -> u32 {
        (position % self.depth) as u32
    }

    /// Claim the next position, or `None` when the slowest reader is `depth` events behind.
    fn begin(&self) -> Option<u64> {
        let mut claim = lock(&self.claim);
        let gate = claim.cursors.iter().map(|cursor| cursor.next.load(Ordering::Acquire)).min().unwrap_or(claim.head);
        if claim.head - gate >= self.depth {
            return None;
        }
        let position = claim.head;
        claim.head += 1;
        self.head.store(claim.head, Ordering::Release);
        drop(claim);
        self.arena.header(self.slot(position)).flags.store(0, Ordering::Relaxed);
        Some(position)
    }

    fn publish(&self, position: u64, stamp_ns: i64, now_ns: i64, flags: u32) {
        let header = self.arena.header(self.slot(position));
        header.stamp_ns.store(stamp_ns, Ordering::Relaxed);
        header.commit_ns.store(now_ns, Ordering::Relaxed);
        header.flags.store(flags, Ordering::Relaxed);
        header.seq.store(position + 1, Ordering::Release);
    }

    fn attach(&self) -> (Arc<Cursor>, u64) {
        let mut claim = lock(&self.claim);
        let start = claim.head;
        let cursor = Arc::new(Cursor { next: AtomicU64::new(start) });
        claim.cursors.push(cursor.clone());
        (cursor, start)
    }

    fn detach(&self, cursor: &Arc<Cursor>) {
        lock(&self.claim).cursors.retain(|other| !Arc::ptr_eq(other, cursor));
    }

    /// Next published event at or after `*local`; voided claims are skipped.
    fn read_next(&self, local: &mut u64, size: u32) -> Option<View> {
        loop {
            let position = *local;
            if position >= self.head.load(Ordering::Acquire) {
                return None;
            }
            let header = self.arena.header(self.slot(position));
            if header.seq.load(Ordering::Acquire) != position + 1 {
                // Claimed but not yet committed: later events stay invisible to keep FIFO order.
                return None;
            }
            *local = position + 1;
            if header.flags.load(Ordering::Relaxed) & FLAG_VOID != 0 {
                continue;
            }
            return Some(View {
                data: self.arena.payload(self.slot(position)),
                size,
                seq: position + 1,
                stamp_ns: header.stamp_ns.load(Ordering::Relaxed),
            });
        }
    }

    fn lag(&self) -> u64 {
        let claim = lock(&self.claim);
        let gate = claim.cursors.iter().map(|cursor| cursor.next.load(Ordering::Acquire)).min();
        gate.map_or(0, |gate| claim.head - gate)
    }
}

enum Ring {
    State(StateRing),
    Event(EventRing),
}

struct Subscription {
    token: u64,
    bit: u64,
    target: Arc<dyn Subscriber>,
}

/// Observes every commit with the committed payload bytes (clock channel).
pub type CommitHook = Box<dyn Fn(&[u8]) + Send + Sync>;

pub struct Channel {
    name: String,
    spec: PayloadSpec,
    kind: Kind,
    depth: u32,
    max_readers: u32,
    ring: Ring,
    readers: AtomicU32,
    writers: AtomicU32,
    subscribers: RwLock<Vec<Subscription>>,
    next_token: AtomicU64,
    commits: AtomicU64,
    drops: AtomicU64,
    stalls: AtomicU64,
    stale_reads: AtomicU64,
    stale: AtomicBool,
    hook: Option<CommitHook>,
}

/// Per-output-port write state (`write_begin` .. `write_commit`).
#[derive(Default)]
pub struct Writer {
    pending: Option<Pending>,
}

#[derive(Clone, Copy)]
enum Pending {
    State(u32),
    Event(u64),
}

impl Writer {
    pub fn is_pending(&self) -> bool {
        self.pending.is_some()
    }
}

/// Per-input-port read state, attached to one channel.
pub struct Reader {
    token: u64,
    side: ReaderSide,
}

enum ReaderSide {
    State { pinned: Option<u32>, consumed: u64 },
    Event { cursor: Arc<Cursor>, local: u64 },
}

#[derive(Clone, Debug)]
pub struct ChannelInfo {
    pub name: String,
    pub kind: Kind,
    pub spec: PayloadSpec,
    pub depth: u32,
    pub max_readers: u32,
    pub readers: u32,
    pub writers: u32,
    pub commits: u64,
    pub drops: u64,
    pub stalls: u64,
    pub stale_reads: u64,
    pub stale: bool,
    /// Events: how far the slowest reader is behind the writers.
    pub lag: u64,
}

impl Channel {
    /// `depth` is the event queue length (ignored for state channels); `max_readers` bounds
    /// the readers that can attach at the same time. Fails when the arena would exceed
    /// [`MAX_ARENA_BYTES`] or cannot be allocated.
    pub fn new(
        name: &str,
        kind: Kind,
        spec: PayloadSpec,
        depth: u32,
        max_readers: u32,
        hook: Option<CommitHook>,
    ) -> Result<Arc<Channel>, String> {
        let bytes = arena_bytes(kind, spec.size, depth, max_readers);
        if bytes > MAX_ARENA_BYTES {
            return Err(format!("channel {name} needs {} MiB of slots; the limit is {} MiB", bytes >> 20, MAX_ARENA_BYTES >> 20));
        }
        let payload = spec.size as usize;
        let ring = match kind {
            Kind::State => Ring::State(StateRing::new(max_readers, payload)?),
            Kind::Event => Ring::Event(EventRing::new(depth, payload)?),
        };
        Ok(Arc::new(Channel {
            name: name.to_owned(),
            spec,
            kind,
            depth: if kind == Kind::Event { depth } else { 0 },
            max_readers,
            ring,
            readers: AtomicU32::new(0),
            writers: AtomicU32::new(0),
            subscribers: RwLock::new(Vec::new()),
            next_token: AtomicU64::new(1),
            commits: AtomicU64::new(0),
            drops: AtomicU64::new(0),
            stalls: AtomicU64::new(0),
            stale_reads: AtomicU64::new(0),
            stale: AtomicBool::new(false),
            hook,
        }))
    }

    pub fn name(&self) -> &str {
        &self.name
    }

    pub fn kind(&self) -> Kind {
        self.kind
    }

    pub fn spec(&self) -> &PayloadSpec {
        &self.spec
    }

    pub fn depth(&self) -> u32 {
        self.depth
    }

    pub fn max_readers(&self) -> u32 {
        self.max_readers
    }

    pub fn writer_count(&self) -> u32 {
        self.writers.load(Ordering::Acquire)
    }

    pub fn reader_count(&self) -> u32 {
        self.readers.load(Ordering::Acquire)
    }

    /// Register a bound writer port. State channels accept one, event channels any number.
    pub fn add_writer(&self) -> Result<(), WriterError> {
        match self.kind {
            Kind::State => self
                .writers
                .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
                .map(|_| ())
                .map_err(|_| WriterError::StateHasWriter),
            Kind::Event => {
                self.writers.fetch_add(1, Ordering::AcqRel);
                Ok(())
            }
        }
    }

    pub fn remove_writer(&self) {
        let _ = self.writers.fetch_update(Ordering::AcqRel, Ordering::Acquire, |n| n.checked_sub(1));
    }

    /// The producer is hung or failed: readers still see the last sample, flagged stale.
    pub fn set_stale(&self, stale: bool) {
        self.stale.store(stale, Ordering::Release);
    }

    pub fn is_stale(&self) -> bool {
        self.stale.load(Ordering::Acquire)
    }

    pub fn info(&self) -> ChannelInfo {
        ChannelInfo {
            name: self.name.clone(),
            kind: self.kind,
            spec: self.spec.clone(),
            depth: self.depth,
            max_readers: self.max_readers,
            readers: self.reader_count(),
            writers: self.writer_count(),
            commits: self.commits.load(Ordering::Relaxed),
            drops: self.drops.load(Ordering::Relaxed),
            stalls: self.stalls.load(Ordering::Relaxed),
            stale_reads: self.stale_reads.load(Ordering::Relaxed),
            stale: self.is_stale(),
            lag: match &self.ring {
                Ring::Event(ring) => ring.lag(),
                Ring::State(_) => 0,
            },
        }
    }

    /// Reserve a slot and return its payload pointer. The caller owns the slot until
    /// `commit` or `abort`.
    pub fn begin_write(&self, writer: &mut Writer) -> Result<*mut u8, WriteError> {
        if writer.pending.is_some() {
            return Err(WriteError::Pending);
        }
        match &self.ring {
            Ring::State(ring) => match ring.begin() {
                Some(slot) => {
                    writer.pending = Some(Pending::State(slot));
                    Ok(ring.arena.payload(slot))
                }
                None => {
                    self.stalls.fetch_add(1, Ordering::Relaxed);
                    Err(WriteError::Stalled)
                }
            },
            Ring::Event(ring) => match ring.begin() {
                Some(position) => {
                    writer.pending = Some(Pending::Event(position));
                    Ok(ring.arena.payload(ring.slot(position)))
                }
                None => {
                    self.drops.fetch_add(1, Ordering::Relaxed);
                    Err(WriteError::Full)
                }
            },
        }
    }

    /// Publish the pending slot and wake the subscribed readers. Returns the sequence number.
    pub fn commit(&self, writer: &mut Writer, stamp_ns: i64, now_ns: i64) -> Result<u64, WriteError> {
        let pending = writer.pending.take().ok_or(WriteError::NoPending)?;
        let (seq, payload) = match (&self.ring, pending) {
            (Ring::State(ring), Pending::State(slot)) => (ring.commit(slot, stamp_ns, now_ns), ring.arena.payload(slot)),
            (Ring::Event(ring), Pending::Event(position)) => {
                ring.publish(position, stamp_ns, now_ns, 0);
                (position + 1, ring.arena.payload(ring.slot(position)))
            }
            _ => return Err(WriteError::NoPending),
        };
        self.commits.fetch_add(1, Ordering::Relaxed);
        self.stale.store(false, Ordering::Release);
        if let Some(hook) = &self.hook {
            // SAFETY: the slot was just published by this writer and holds `size` bytes.
            hook(unsafe { std::slice::from_raw_parts(payload, self.spec.size as usize) });
        }
        let subscribers = self.subscribers.read().unwrap_or_else(PoisonError::into_inner);
        for subscription in subscribers.iter() {
            subscription.target.notify(subscription.bit, now_ns);
        }
        Ok(seq)
    }

    /// Give the pending slot back. A claimed event position is published as void so that
    /// readers skip it instead of waiting for it forever.
    pub fn abort(&self, writer: &mut Writer) {
        if let Some(Pending::Event(position)) = writer.pending.take() {
            if let Ring::Event(ring) = &self.ring {
                ring.publish(position, 0, 0, FLAG_VOID);
            }
        }
    }

    /// Attach a reader. `bit` is delivered to `target` on every commit.
    pub fn attach_reader(&self, target: Arc<dyn Subscriber>, bit: u64) -> Result<Reader, AttachError> {
        self.readers
            .fetch_update(Ordering::AcqRel, Ordering::Acquire, |n| (n < self.max_readers).then_some(n + 1))
            .map_err(|_| AttachError::TooManyReaders)?;
        let token = self.next_token.fetch_add(1, Ordering::Relaxed);
        let side = match &self.ring {
            Ring::State(_) => ReaderSide::State { pinned: None, consumed: 0 },
            Ring::Event(ring) => {
                let (cursor, start) = ring.attach();
                ReaderSide::Event { cursor, local: start }
            }
        };
        self.subscribers.write().unwrap_or_else(PoisonError::into_inner).push(Subscription { token, bit, target });
        Ok(Reader { token, side })
    }

    /// Point a reader's commit notifications at another target (hot replace of its instance).
    pub fn retarget_reader(&self, reader: &mut Reader, target: Arc<dyn Subscriber>, bit: u64) {
        let mut subscribers = self.subscribers.write().unwrap_or_else(PoisonError::into_inner);
        if let Some(subscription) = subscribers.iter_mut().find(|subscription| subscription.token == reader.token) {
            subscription.target = target;
            subscription.bit = bit;
        }
    }

    /// Detach a reader; its pins and cursor stop holding slots.
    pub fn detach_reader(&self, mut reader: Reader) {
        self.end_step(&mut reader);
        self.subscribers.write().unwrap_or_else(PoisonError::into_inner).retain(|subscription| subscription.token != reader.token);
        if let (Ring::Event(ring), ReaderSide::Event { cursor, .. }) = (&self.ring, &reader.side) {
            ring.detach(cursor);
        }
        self.readers.fetch_sub(1, Ordering::AcqRel);
    }

    /// State channels: the newest sample. The first call of a step pins it; later calls in the
    /// same step return the same sample.
    pub fn read_latest(&self, reader: &mut Reader) -> Option<View> {
        let (Ring::State(ring), ReaderSide::State { pinned, consumed }) = (&self.ring, &mut reader.side) else {
            return None;
        };
        if pinned.is_none() {
            let slot = ring.pin_latest()?;
            *pinned = Some(slot);
            *consumed = ring.arena.header(slot).seq.load(Ordering::Relaxed);
            if self.is_stale() {
                self.stale_reads.fetch_add(1, Ordering::Relaxed);
            }
        }
        pinned.map(|slot| ring.view(slot, self.spec.size))
    }

    /// Event channels: the next unread event, borrowed until `end_step`.
    pub fn read_next(&self, reader: &mut Reader) -> Option<View> {
        let (Ring::Event(ring), ReaderSide::Event { local, .. }) = (&self.ring, &mut reader.side) else {
            return None;
        };
        let view = ring.read_next(local, self.spec.size);
        if view.is_some() && self.is_stale() {
            self.stale_reads.fetch_add(1, Ordering::Relaxed);
        }
        view
    }

    /// Release what the step borrowed: unpin the state slot, publish the event cursor.
    pub fn end_step(&self, reader: &mut Reader) {
        match (&self.ring, &mut reader.side) {
            (Ring::State(ring), ReaderSide::State { pinned, .. }) => {
                if let Some(slot) = pinned.take() {
                    ring.unpin(slot);
                }
            }
            (Ring::Event(_), ReaderSide::Event { cursor, local }) => {
                cursor.next.store(*local, Ordering::Release);
            }
            _ => {}
        }
    }

    /// Is there anything this reader has not consumed yet? Used to drop spurious dirty bits.
    pub fn has_unread(&self, reader: &Reader) -> bool {
        match (&self.ring, &reader.side) {
            (Ring::State(ring), ReaderSide::State { consumed, .. }) => ring.seq.load(Ordering::Acquire) > *consumed,
            (Ring::Event(ring), ReaderSide::Event { local, .. }) => *local < ring.head.load(Ordering::Acquire),
            _ => false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicUsize;
    use std::thread;
    use std::time::{Duration, Instant};

    #[derive(Default)]
    struct Counter {
        notified: AtomicUsize,
        bits: AtomicU64,
    }

    impl Subscriber for Counter {
        fn notify(&self, bit: u64, _commit_ns: i64) {
            self.notified.fetch_add(1, Ordering::SeqCst);
            self.bits.fetch_or(bit, Ordering::SeqCst);
        }
    }

    fn spec(size: u32) -> PayloadSpec {
        PayloadSpec { schema: "test.v1".into(), size, align: 8 }
    }

    fn write_u64(channel: &Channel, writer: &mut Writer, value: u64) {
        let pointer = channel.begin_write(writer).expect("slot") as *mut u64;
        // SAFETY: the slot holds at least 8 bytes and is 64-byte aligned.
        unsafe { pointer.write(value) };
        channel.commit(writer, value as i64, 1).expect("commit");
    }

    fn read_u64(view: View) -> u64 {
        // SAFETY: the view points at a committed 8-byte payload.
        unsafe { (view.data as *const u64).read() }
    }

    #[test]
    fn slots_are_cache_line_aligned_and_headers_are_contiguous() {
        let channel = Channel::new("a", Kind::State, spec(100), 0, 3, None).unwrap();
        let Ring::State(ring) = &channel.ring else { unreachable!() };
        assert_eq!(ring.arena.slots, 5);
        assert_eq!(ring.arena.stride, 64 + 128);
        for slot in 0..5 {
            assert_eq!(ring.arena.payload(slot) as usize % CACHE_LINE, 0);
            assert_eq!(ring.arena.header(slot) as *const SlotHeader as usize % CACHE_LINE, 0);
            assert_eq!(ring.arena.payload(slot) as usize - ring.arena.header(slot) as *const SlotHeader as usize, 64);
        }
    }

    #[test]
    fn state_reader_gets_latest_and_borrowed_view_is_stable() {
        let channel = Channel::new("s", Kind::State, spec(8), 0, 2, None).unwrap();
        let target = Arc::new(Counter::default());
        let mut reader = channel.attach_reader(target.clone(), 1 << 3).unwrap();
        let mut writer = Writer::default();
        assert!(channel.read_latest(&mut reader).is_none());
        assert!(!channel.has_unread(&reader));
        write_u64(&channel, &mut writer, 1);
        assert_eq!(target.notified.load(Ordering::SeqCst), 1);
        assert_eq!(target.bits.load(Ordering::SeqCst), 1 << 3);
        assert!(channel.has_unread(&reader));
        let first = channel.read_latest(&mut reader).unwrap();
        assert_eq!((read_u64(first), first.seq, first.stamp_ns), (1, 1, 1));
        // Many commits while the view is borrowed never touch the pinned slot.
        for value in 2..=50 {
            write_u64(&channel, &mut writer, value);
            assert_eq!(read_u64(first), 1);
        }
        // The same step keeps returning the same sample.
        assert_eq!(channel.read_latest(&mut reader).unwrap().seq, 1);
        channel.end_step(&mut reader);
        assert!(channel.has_unread(&reader));
        let newest = channel.read_latest(&mut reader).unwrap();
        assert_eq!((read_u64(newest), newest.seq), (50, 50));
        channel.end_step(&mut reader);
        assert!(!channel.has_unread(&reader));
        channel.detach_reader(reader);
        assert_eq!(channel.reader_count(), 0);
    }

    #[test]
    fn state_reader_capacity_and_single_writer_are_enforced() {
        let channel = Channel::new("s", Kind::State, spec(8), 0, 2, None).unwrap();
        let target = Arc::new(Counter::default());
        let a = channel.attach_reader(target.clone(), 1).unwrap();
        let _b = channel.attach_reader(target.clone(), 2).unwrap();
        assert_eq!(channel.attach_reader(target.clone(), 4).err(), Some(AttachError::TooManyReaders));
        channel.detach_reader(a);
        assert!(channel.attach_reader(target, 4).is_ok());
        assert_eq!(channel.add_writer(), Ok(()));
        assert_eq!(channel.add_writer(), Err(WriterError::StateHasWriter));
        channel.remove_writer();
        assert_eq!(channel.add_writer(), Ok(()));
    }

    #[test]
    fn state_writer_never_waits_with_every_reader_pinned() {
        let readers = 4;
        let channel = Channel::new("s", Kind::State, spec(8), 0, readers, None).unwrap();
        let target = Arc::new(Counter::default());
        let mut writer = Writer::default();
        write_u64(&channel, &mut writer, 0);
        let mut pinned: Vec<_> = (0..readers).map(|i| channel.attach_reader(target.clone(), 1 << i).unwrap()).collect();
        for reader in &mut pinned {
            channel.read_latest(reader).unwrap();
        }
        // Readers pin one slot each, one slot is latest: the writer still finds a free one.
        for value in 1..100 {
            write_u64(&channel, &mut writer, value);
        }
        assert_eq!(channel.info().stalls, 0);
    }

    #[test]
    fn state_stress_readers_never_see_torn_samples() {
        const WORDS: usize = 32;
        let readers = 3;
        let channel = Channel::new("s", Kind::State, spec(8 * WORDS as u32), 0, readers, None).unwrap();
        let stop = Arc::new(AtomicBool::new(false));
        let target = Arc::new(Counter::default());
        let mut handles = Vec::new();
        for i in 0..readers {
            let channel = channel.clone();
            let stop = stop.clone();
            let mut reader = channel.attach_reader(target.clone(), 1 << i).unwrap();
            handles.push(thread::spawn(move || {
                let (mut last, mut reads) = (0u64, 0u64);
                while !stop.load(Ordering::Relaxed) {
                    if let Some(view) = channel.read_latest(&mut reader) {
                        // SAFETY: borrowed, committed payload of WORDS u64 values.
                        let words = unsafe { std::slice::from_raw_parts(view.data as *const u64, WORDS) };
                        assert!(words.iter().all(|w| *w == words[0]), "torn sample");
                        assert!(view.seq >= last, "sequence went backwards");
                        assert_eq!(words[0], view.seq);
                        last = view.seq;
                        reads += 1;
                    }
                    channel.end_step(&mut reader);
                }
                reads
            }));
        }
        let mut writer = Writer::default();
        let deadline = Instant::now() + Duration::from_millis(300);
        let mut seq = 0u64;
        while Instant::now() < deadline {
            let pointer = channel.begin_write(&mut writer).expect("writer never waits") as *mut u64;
            seq += 1;
            for word in 0..WORDS {
                // SAFETY: the pending slot holds WORDS u64 values.
                unsafe { pointer.add(word).write(seq) };
            }
            channel.commit(&mut writer, 0, 0).unwrap();
        }
        stop.store(true, Ordering::Relaxed);
        let reads: u64 = handles.into_iter().map(|h| h.join().unwrap()).sum();
        assert!(reads > 0 && seq > 1000, "reads {reads} writes {seq}");
        assert_eq!(channel.info().stalls, 0);
    }

    #[test]
    fn event_queue_is_fifo_lossless_until_full_and_counts_drops() {
        let channel = Channel::new("e", Kind::Event, spec(8), 4, 4, None).unwrap();
        let target = Arc::new(Counter::default());
        let mut reader = channel.attach_reader(target, 1).unwrap();
        let mut writer = Writer::default();
        for value in 0..4 {
            write_u64(&channel, &mut writer, value);
        }
        assert_eq!(channel.begin_write(&mut writer).err(), Some(WriteError::Full));
        assert_eq!(channel.begin_write(&mut writer).err(), Some(WriteError::Full));
        assert_eq!(channel.info().drops, 2);
        // Reading inside a step does not free slots: they stay borrowed until the step ends.
        let mut seen = Vec::new();
        while let Some(view) = channel.read_next(&mut reader) {
            seen.push(read_u64(view));
        }
        assert_eq!(seen, [0, 1, 2, 3]);
        assert_eq!(channel.begin_write(&mut writer).err(), Some(WriteError::Full));
        channel.end_step(&mut reader);
        assert!(!channel.has_unread(&reader));
        write_u64(&channel, &mut writer, 4);
        let view = channel.read_next(&mut reader).unwrap();
        assert_eq!((read_u64(view), view.seq), (4, 5));
        assert!(channel.read_next(&mut reader).is_none());
    }

    #[test]
    fn every_event_reader_has_its_own_cursor() {
        let channel = Channel::new("e", Kind::Event, spec(8), 8, 4, None).unwrap();
        let target = Arc::new(Counter::default());
        let mut fast = channel.attach_reader(target.clone(), 1).unwrap();
        let mut slow = channel.attach_reader(target, 2).unwrap();
        let mut writer = Writer::default();
        for value in 0..6 {
            write_u64(&channel, &mut writer, value);
        }
        let count = |channel: &Channel, reader: &mut Reader| {
            let mut n = 0;
            while channel.read_next(reader).is_some() {
                n += 1;
            }
            channel.end_step(reader);
            n
        };
        assert_eq!(count(&channel, &mut fast), 6);
        // The slow reader holds the queue: 6 of 8 slots are still in use.
        write_u64(&channel, &mut writer, 6);
        write_u64(&channel, &mut writer, 7);
        assert_eq!(channel.begin_write(&mut writer).err(), Some(WriteError::Full));
        assert_eq!(channel.info().lag, 8);
        assert_eq!(count(&channel, &mut slow), 8);
        assert_eq!(count(&channel, &mut fast), 2);
        assert!(channel.begin_write(&mut writer).is_ok());
    }

    #[test]
    fn late_event_reader_starts_at_the_head() {
        let channel = Channel::new("e", Kind::Event, spec(8), 4, 2, None).unwrap();
        let mut writer = Writer::default();
        for value in 0..3 {
            write_u64(&channel, &mut writer, value);
        }
        let mut late = channel.attach_reader(Arc::new(Counter::default()), 1).unwrap();
        assert!(channel.read_next(&mut late).is_none());
        write_u64(&channel, &mut writer, 3);
        assert_eq!(read_u64(channel.read_next(&mut late).unwrap()), 3);
    }

    #[test]
    fn uncommitted_claim_blocks_later_events_and_abort_releases_them() {
        let channel = Channel::new("e", Kind::Event, spec(8), 8, 2, None).unwrap();
        let mut reader = channel.attach_reader(Arc::new(Counter::default()), 1).unwrap();
        let mut slow = Writer::default();
        let mut fast = Writer::default();
        let _held = channel.begin_write(&mut slow).unwrap();
        write_u64(&channel, &mut fast, 7);
        // FIFO order: event 1 is committed, but position 0 is still being written.
        assert!(channel.read_next(&mut reader).is_none());
        assert_eq!(channel.begin_write(&mut slow).err(), Some(WriteError::Pending));
        channel.abort(&mut slow);
        let view = channel.read_next(&mut reader).unwrap();
        assert_eq!((read_u64(view), view.seq), (7, 2));
        assert!(channel.read_next(&mut reader).is_none());
        assert_eq!(channel.commit(&mut slow, 0, 0).err(), Some(WriteError::NoPending));
    }

    #[test]
    fn several_writers_keep_per_writer_order() {
        const WRITERS: u64 = 4;
        const PER_WRITER: u64 = 2000;
        let channel = Channel::new("e", Kind::Event, spec(16), 64, 2, None).unwrap();
        let mut reader = channel.attach_reader(Arc::new(Counter::default()), 1).unwrap();
        let handles: Vec<_> = (0..WRITERS)
            .map(|id| {
                let channel = channel.clone();
                thread::spawn(move || {
                    let mut writer = Writer::default();
                    let mut sent = 0;
                    while sent < PER_WRITER {
                        match channel.begin_write(&mut writer) {
                            Ok(pointer) => {
                                // SAFETY: the slot holds 16 bytes.
                                unsafe {
                                    (pointer as *mut u64).write(id);
                                    (pointer as *mut u64).add(1).write(sent);
                                }
                                channel.commit(&mut writer, 0, 0).unwrap();
                                sent += 1;
                            }
                            Err(WriteError::Full) => thread::yield_now(),
                            Err(other) => panic!("{other:?}"),
                        }
                    }
                })
            })
            .collect();
        let mut next = [0u64; WRITERS as usize];
        let mut total = 0;
        let mut last_seq = 0;
        let deadline = Instant::now() + Duration::from_secs(20);
        while total < WRITERS * PER_WRITER && Instant::now() < deadline {
            while let Some(view) = channel.read_next(&mut reader) {
                // SAFETY: committed 16-byte payload.
                let (id, n) = unsafe { ((view.data as *const u64).read(), (view.data as *const u64).add(1).read()) };
                assert_eq!(n, next[id as usize], "writer {id} reordered");
                next[id as usize] += 1;
                assert!(view.seq > last_seq);
                last_seq = view.seq;
                total += 1;
            }
            channel.end_step(&mut reader);
            thread::yield_now();
        }
        for handle in handles {
            handle.join().unwrap();
        }
        assert_eq!(total, WRITERS * PER_WRITER);
    }

    #[test]
    fn commit_hook_sees_the_payload_and_stale_clears_on_commit() {
        let seen = Arc::new(AtomicU64::new(0));
        let observed = seen.clone();
        let hook: CommitHook = Box::new(move |bytes| {
            observed.store(u64::from_ne_bytes(bytes.try_into().unwrap()), Ordering::SeqCst);
        });
        let channel = Channel::new("clock", Kind::State, spec(8), 0, 1, Some(hook)).unwrap();
        let mut writer = Writer::default();
        channel.set_stale(true);
        write_u64(&channel, &mut writer, 42);
        assert_eq!(seen.load(Ordering::SeqCst), 42);
        assert!(!channel.is_stale());
    }
}
