//! The loopback transport as a transport plugin (`xgc_rt_transport_v1`).
//!
//! Every host in one process that loads this library shares its buses:
//! option `bus` names one (default "default"), so several hosts in one test
//! process talk to each other as the built-in transport's shared
//! `LoopbackBus` does. Options `drop`, `duplicate`, `reorder` and `seed`
//! set the seeded injector on the bus that first opens it.
//!
//! Two C functions beside the ABI read a bus back, for exactness tests:
//! `xgc_rt_loopback_flush(bus)` releases held samples, and
//! `xgc_rt_loopback_truth_json(bus, out, cap)` writes its ground truth as
//! JSON `[[channel, origin, receiver, offered, dropped, duplicated,
//! reordered], ...]` and returns the length it needs.

use std::collections::HashMap;
use std::ffi::{c_char, CStr};
use std::sync::{Arc, Mutex, OnceLock};

use xgc_rt_core::transport::{Transport, TransportContext, TransportError};
use xgc_rt_transport_loopback::{Impairment, LoopbackBus, LoopbackTransport};

fn buses() -> &'static Mutex<HashMap<String, Arc<LoopbackBus>>> {
    static BUSES: OnceLock<Mutex<HashMap<String, Arc<LoopbackBus>>>> = OnceLock::new();
    BUSES.get_or_init(Default::default)
}

fn factory(_ctx: &TransportContext, options: &toml::Table) -> Result<Box<dyn Transport>, TransportError> {
    let bad = |key: &str| TransportError(format!("transport.{key}: expected a number"));
    let number = |key: &str| -> Result<Option<f64>, TransportError> {
        match options.get(key) {
            None => Ok(None),
            Some(v) => v.as_float().or_else(|| v.as_integer().map(|i| i as f64)).map(Some).ok_or_else(|| bad(key)),
        }
    };
    if let Some(extra) = options.keys().find(|k| !["bus", "drop", "duplicate", "reorder", "seed"].contains(&k.as_str())) {
        return Err(TransportError(format!("transport.{extra} is not a loopback option")));
    }
    let name = match options.get("bus") {
        None => "default".to_string(),
        Some(v) => v.as_str().ok_or_else(|| TransportError("transport.bus: expected a string".into()))?.to_string(),
    };
    let impairment = match (number("drop")?, number("duplicate")?, number("reorder")?) {
        (None, None, None) => None,
        (d, u, r) => Some(Impairment {
            drop: d.unwrap_or(0.0),
            duplicate: u.unwrap_or(0.0),
            reorder: r.unwrap_or(0.0),
            seed: options.get("seed").and_then(|v| v.as_integer()).unwrap_or(0) as u64,
        }),
    };
    let bus = buses()
        .lock()
        .unwrap()
        .entry(name)
        .or_insert_with(|| impairment.map_or_else(LoopbackBus::new, LoopbackBus::with_impairment))
        .clone();
    Ok(Box::new(LoopbackTransport::new(bus)))
}

xgc_rt_core::export_transport!("loopback", factory);

fn bus(name: *const c_char) -> Option<Arc<LoopbackBus>> {
    if name.is_null() {
        return None;
    }
    let name = unsafe { CStr::from_ptr(name) }.to_str().ok()?;
    buses().lock().unwrap().get(name).cloned()
}

/// Release the samples `bus` holds for reordering (see `LoopbackBus::flush`).
#[no_mangle]
pub extern "C" fn xgc_rt_loopback_flush(name: *const c_char) -> i32 {
    match bus(name) {
        Some(b) => {
            b.flush();
            0
        }
        None => 2,
    }
}

/// `bus`'s ground truth as JSON into `out` (at most `cap` bytes, no NUL);
/// returns the full length, or -1 for an unknown bus.
#[no_mangle]
pub extern "C" fn xgc_rt_loopback_truth_json(name: *const c_char, out: *mut u8, cap: usize) -> isize {
    let Some(b) = bus(name) else { return -1 };
    let rows: Vec<[u64; 7]> = b
        .truth()
        .into_iter()
        .map(|((c, o, r), t)| [c as u64, o as u64, r as u64, t.offered, t.dropped, t.duplicated, t.reordered])
        .collect();
    let json = serde_json::to_vec(&rows).unwrap();
    if !out.is_null() {
        let n = json.len().min(cap);
        unsafe { std::ptr::copy_nonoverlapping(json.as_ptr(), out, n) };
    }
    json.len() as isize
}
