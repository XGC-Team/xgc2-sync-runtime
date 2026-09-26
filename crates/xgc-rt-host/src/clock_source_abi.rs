//! Exact mirror of abi/include/xgc_clock_source.h; not the domain plugin ABI.
use std::ffi::{c_char, c_void};
pub const VERSION: u32 = 1;
pub const ENTRY: &[u8] = b"xgc_rt_clock_source_v1\0";
pub const OK: i32 = 0;
pub const AGAIN: i32 = 1;
pub const ERROR: i32 = 2;
pub const GATE_CLOSED: u32 = 0;
pub const GATE_OPEN: u32 = 1;
pub const GATE_FAULT: u32 = 2;
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Observation {
    pub time_ns: i64,
    pub sequence: u64,
    pub coalesced: u32,
    pub publisher_count: u32,
    pub dropped: u64,
    pub publisher: [c_char; 256],
    pub error: [c_char; 256],
}
impl Default for Observation {
    fn default() -> Self {
        Self {
            time_ns: 0,
            sequence: 0,
            coalesced: 0,
            publisher_count: 0,
            dropped: 0,
            publisher: [0; 256],
            error: [0; 256],
        }
    }
}
#[repr(C)]
#[derive(Clone, Copy)]
pub struct VTable {
    pub create: Option<unsafe extern "C" fn() -> *mut c_void>,
    pub start: Option<unsafe extern "C" fn(*mut c_void, *const c_char, *mut Observation) -> i32>,
    pub poll: Option<unsafe extern "C" fn(*mut c_void, u64, *mut Observation) -> i32>,
    pub set_gate: Option<unsafe extern "C" fn(*mut c_void, u32) -> i32>,
    pub stop: Option<unsafe extern "C" fn(*mut c_void)>,
    pub destroy: Option<unsafe extern "C" fn(*mut c_void)>,
}
#[repr(C)]
pub struct Descriptor {
    pub abi_version: u32,
    pub reserved: u32,
    pub vtbl: *const VTable,
}
pub type Entry = unsafe extern "C" fn() -> *const Descriptor;
