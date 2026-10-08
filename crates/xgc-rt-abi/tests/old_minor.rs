//! Probe genuinely shorter previous-host tables against the plugin facade.
#![cfg(all(target_os = "linux", target_pointer_width = "64"))]

use std::{
    ffi::{c_char, c_void},
    mem::size_of,
    ptr,
};
use xgc_rt_abi::*;

#[repr(C)]
struct Minor0 {
    abi_version: u32,
    abi_minor: u32,
    host: *mut c_void,
    publish: unsafe extern "C" fn(*mut c_void, u32, u64, *const u8, u32) -> XgcStatus,
    next: unsafe extern "C" fn(*mut c_void, u32, *mut XgcSampleView) -> XgcStatus,
    now: unsafe extern "C" fn(*mut c_void) -> i64,
    log: unsafe extern "C" fn(*mut c_void, XgcLogLevel, *const c_char),
    degrade: unsafe extern "C" fn(*mut c_void, *const c_char),
    recover: unsafe extern "C" fn(*mut c_void),
}

#[repr(C)]
struct Minor1 {
    prefix: Minor0,
    origins: unsafe extern "C" fn(*mut c_void, u32, *mut u16, u32) -> u32,
    node: unsafe extern "C" fn(*mut c_void) -> u16,
}

unsafe extern "C" fn publish(_: *mut c_void, _: u32, _: u64, _: *const u8, _: u32) -> XgcStatus {
    XGC_OK
}
unsafe extern "C" fn next(_: *mut c_void, _: u32, _: *mut XgcSampleView) -> XgcStatus {
    XGC_ERR_AGAIN
}
unsafe extern "C" fn now(_: *mut c_void) -> i64 {
    73
}
unsafe extern "C" fn log(_: *mut c_void, _: XgcLogLevel, _: *const c_char) {}
unsafe extern "C" fn degrade(_: *mut c_void, _: *const c_char) {}
unsafe extern "C" fn recover(_: *mut c_void) {}
unsafe extern "C" fn origins(_: *mut c_void, _: u32, out: *mut u16, cap: u32) -> u32 {
    if cap > 0 {
        out.write(11);
    }
    1
}
unsafe extern "C" fn node(_: *mut c_void) -> u16 {
    11
}

fn prefix(minor: u32) -> Minor0 {
    Minor0 {
        abi_version: 1,
        abi_minor: minor,
        host: ptr::null_mut(),
        publish,
        next,
        now,
        log,
        degrade,
        recover,
    }
}

unsafe extern "C" {
    fn getpagesize() -> i32;
    fn mmap(
        address: *mut c_void,
        len: usize,
        protection: i32,
        flags: i32,
        fd: i32,
        offset: i64,
    ) -> *mut c_void;
    fn mprotect(address: *mut c_void, len: usize, protection: i32) -> i32;
    fn munmap(address: *mut c_void, len: usize) -> i32;
}

struct Guarded {
    base: *mut c_void,
    page: usize,
}
impl Guarded {
    fn table<T>(&self, value: T) -> *const XgcHostApi {
        assert!(size_of::<T>() < self.page);
        let pointer = unsafe {
            self.base
                .cast::<u8>()
                .add(self.page - size_of::<T>())
                .cast::<T>()
        };
        unsafe {
            pointer.write(value);
        }
        pointer.cast()
    }
    fn new() -> Self {
        let page = unsafe { getpagesize() } as usize;
        let base = unsafe { mmap(ptr::null_mut(), page * 2, 3, 0x22, -1, 0) };
        assert_ne!(base as usize, usize::MAX);
        assert_eq!(
            unsafe { mprotect(base.cast::<u8>().add(page).cast(), page, 0) },
            0
        );
        Self { base, page }
    }
}
impl Drop for Guarded {
    fn drop(&mut self) {
        assert_eq!(unsafe { munmap(self.base, self.page * 2) }, 0);
    }
}

#[test]
fn minor_zero_and_one_never_read_an_appended_rpc_field() {
    assert_eq!(size_of::<Minor0>(), 64);
    assert_eq!(size_of::<Minor1>(), 80);
    let guarded = Guarded::new();
    let mut host = unsafe { Host::from_raw(guarded.table(prefix(0))) };
    assert_eq!(host.now(), 73);
    assert!(host.publish(0, 1, b"test").is_ok());
    assert!(host.next(0).is_none());
    host.log(XGC_LOG_INFO, "prefix");
    host.request_degrade("prefix");
    host.request_recover();
    assert!(host.port_origins(0).is_empty());
    assert!(host.node_id().is_none());
    assert!(host.rpc_runtime_api().is_none());

    let host = unsafe {
        Host::from_raw(guarded.table(Minor1 {
            prefix: prefix(1),
            origins,
            node,
        }))
    };
    assert_eq!(host.port_origins(0), vec![11]);
    assert_eq!(host.node_id(), Some(11));
    assert!(host.rpc_runtime_api().is_none());
}

#[test]
fn minor_three_nullable_getter_is_checked() {
    let api = XgcHostApi {
        abi_version: 1,
        abi_minor: 3,
        host: ptr::null_mut(),
        publish,
        next,
        now,
        log,
        request_degrade: degrade,
        request_recover: recover,
        port_origins: origins,
        node_id: node,
        rpc_runtime: None,
    };
    let host = unsafe { Host::from_raw(&api) };
    assert!(host.rpc_runtime_api().is_none());
}
