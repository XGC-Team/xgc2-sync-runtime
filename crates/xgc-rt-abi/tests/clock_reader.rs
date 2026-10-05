//! Capability/lifetime checks for the real plugin-side SDK entry, including
//! old allocations shorter than the new XgcHostApi. No fallback clock exists.
use std::ffi::{c_char, c_void, CStr};
use std::sync::{Arc, atomic::{AtomicI64, AtomicUsize, Ordering}};
use xgc_rt_abi::*;

struct Owner {
    time: Arc<AtomicI64>,
    acquired: Arc<AtomicUsize>,
    released: Arc<AtomicUsize>,
}
struct Lease { time: Arc<AtomicI64>, released: Arc<AtomicUsize> }

unsafe extern "C" fn read(opaque: *mut c_void, out: *mut i64) -> XgcStatus {
    if opaque.is_null() || out.is_null() { return XGC_ERR_INVALID; }
    out.write((&*opaque.cast::<Lease>()).time.load(Ordering::Acquire));
    XGC_OK
}
unsafe extern "C" fn release(opaque: *mut c_void) {
    let lease = Box::from_raw(opaque.cast::<Lease>());
    lease.released.fetch_add(1, Ordering::Release);
}
unsafe extern "C" fn acquire(host: *mut c_void, size: u32, out: *mut XgcClockReaderV1) -> XgcStatus {
    if size as usize != std::mem::size_of::<XgcClockReaderV1>() || out.is_null() {
        return XGC_ERR_INVALID;
    }
    let owner = &*host.cast::<Owner>();
    owner.acquired.fetch_add(1, Ordering::Release);
    out.write(XgcClockReaderV1 {
        opaque: Box::into_raw(Box::new(Lease { time: owner.time.clone(), released: owner.released.clone() })).cast(),
        now: Some(read), release: Some(release),
    });
    XGC_OK
}
unsafe extern "C" fn publish(_: *mut c_void, _: u32, _: u64, _: *const u8, _: u32) -> XgcStatus { XGC_OK }
unsafe extern "C" fn next(_: *mut c_void, _: u32, _: *mut XgcSampleView) -> XgcStatus { XGC_ERR_AGAIN }
unsafe extern "C" fn ordinary_now(_: *mut c_void) -> i64 { 987_654_321 }
unsafe extern "C" fn log(_: *mut c_void, _: XgcLogLevel, _: *const c_char) {}
unsafe extern "C" fn degrade(_: *mut c_void, _: *const c_char) {}
unsafe extern "C" fn recover(_: *mut c_void) {}
unsafe extern "C" fn origins(_: *mut c_void, _: u32, _: *mut u16, _: u32) -> u32 { 0 }
unsafe extern "C" fn node(_: *mut c_void) -> u16 { 0 }
fn table(owner: &mut Owner) -> XgcHostApi {
    XgcHostApi {
        abi_version: 1, abi_minor: 3, host: (owner as *mut Owner).cast(),
        publish, next, now: ordinary_now, log, request_degrade: degrade, request_recover: recover,
        port_origins: origins, node_id: node, acquire_clock_reader: Some(acquire),
    }
}

#[test]
fn short_old_prefixes_are_rejected_before_any_tail_read() {
    #[repr(C, align(8))] struct Prefix { version: u32, minor: u32 }
    for (version, minor) in [(1, 0), (1, 1), (1, 2), (99, 3)] {
        let old = Prefix { version, minor }; // just 8 bytes, no new tail allocation
        let host = unsafe { Host::from_raw((&old as *const Prefix).cast()) };
        assert!(matches!(host.acquire_clock_reader(), Err(XGC_ERR_INVALID)));
    }
    #[repr(C)] struct OldMinor2 { version: u32, minor: u32, old_tail: [usize; 9] }
    let old = OldMinor2 { version: 1, minor: 2, old_tail: [0; 9] };
    assert_eq!(std::mem::size_of_val(&old), 80);
    let host = unsafe { Host::from_raw((&old as *const OldMinor2).cast()) };
    assert!(matches!(host.acquire_clock_reader(), Err(XGC_ERR_INVALID)));
}

#[test]
fn new_table_missing_accessor_is_rejected_and_sdk_reader_outlives_table() {
    let acquired = Arc::new(AtomicUsize::new(0));
    let released = Arc::new(AtomicUsize::new(0));
    let time = Arc::new(AtomicI64::new(0));
    let mut owner = Owner { time: time.clone(), acquired: acquired.clone(), released: released.clone() };
    let mut api = table(&mut owner);
    api.acquire_clock_reader = None;
    let host = unsafe { Host::from_raw(&api) };
    assert!(matches!(host.acquire_clock_reader(), Err(XGC_ERR_INVALID)));
    assert_eq!(acquired.load(Ordering::Acquire), 0);
    api.acquire_clock_reader = Some(acquire);
    let reader = unsafe { Host::from_raw(&api) }.acquire_clock_reader().unwrap();
    drop(api);
    drop(owner); // cold acquisition context/ordinary Host API no longer exists
    let reading = std::thread::spawn(move || {
        assert_eq!(reader.now(), Ok(0));
        reader
    });
    let reader = reading.join().unwrap();
    time.store(42, Ordering::Release);
    assert_eq!(reader.now(), Ok(42));
    assert_eq!(acquired.load(Ordering::Acquire), 1);
    assert_eq!(released.load(Ordering::Acquire), 0);
    assert_eq!(Arc::strong_count(&time), 2); // no retain on each now
    drop(reader); // after all reading threads have joined
    assert_eq!(released.load(Ordering::Acquire), 1);
    assert_eq!(Arc::strong_count(&time), 1);
}

static ORDINARY_CREATES: AtomicUsize = AtomicUsize::new(0);
struct OrdinaryPlugin { stamp: i64 }
impl Plugin for OrdinaryPlugin {
    fn create(host: Host) -> Self {
        ORDINARY_CREATES.fetch_add(1, Ordering::SeqCst);
        Self { stamp: host.now() } // ordinary method forms a full API reference
    }
    fn step(&mut self, _: &XgcStepCtx) -> Result<(), String> { Ok(()) }
    fn domain_state(&self) -> &'static CStr { cstr!("ordinary") }
}

#[test]
fn sdk_create_rejects_old_prefix_before_entering_any_user_plugin() {
    #[repr(C, align(8))] struct Prefix { version: u32, minor: u32 }
    ORDINARY_CREATES.store(0, Ordering::SeqCst);
    assert!(unsafe { shim::create::<OrdinaryPlugin>(std::ptr::null()) }.is_null());
    for (version, minor) in [(1, 0), (1, 1), (1, 2), (99, 3)] {
        let old = Prefix { version, minor };
        assert!(unsafe { shim::create::<OrdinaryPlugin>((&old as *const Prefix).cast()) }.is_null());
    }
    assert_eq!(ORDINARY_CREATES.load(Ordering::SeqCst), 0);
    let mut owner = Owner {
        time: Arc::new(AtomicI64::new(0)), acquired: Arc::new(AtomicUsize::new(0)),
        released: Arc::new(AtomicUsize::new(0)),
    };
    let mut api = table(&mut owner);
    api.acquire_clock_reader = None;
    assert!(unsafe { shim::create::<OrdinaryPlugin>(&api) }.is_null());
    assert_eq!(ORDINARY_CREATES.load(Ordering::SeqCst), 0);
    api.acquire_clock_reader = Some(acquire);
    let instance = unsafe { shim::create::<OrdinaryPlugin>(&api) };
    assert!(!instance.is_null());
    assert_eq!(ORDINARY_CREATES.load(Ordering::SeqCst), 1);
    assert_eq!(unsafe { (*instance.cast::<OrdinaryPlugin>()).stamp }, 987_654_321);
    assert_eq!(owner.acquired.load(Ordering::Acquire), 0); // ordinary plugin is not a reader consumer
    unsafe { shim::destroy::<OrdinaryPlugin>(instance) };
}
