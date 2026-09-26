//! A transport loaded from a transport plugin (`xgc_rt_transport_v1`,
//! abi/include/xgc_rt.h): the manifest's `[transport] path`. It is a
//! [`Transport`] like the built-in ones, so the host treats every transport
//! the same way.

use std::ffi::{c_char, c_void, CStr, CString};
use std::path::Path;

use xgc_rt_core::transport::{RxSink, Transport, TransportContext, TransportError};
use xgc_rt_core::transport_abi::{Channel, Context, Entry, VTable, ENTRY, OK, VERSION};
use xgc_rt_core::{ChannelId, OriginId};

use crate::plugin::sha256_hex;

pub struct SoTransport {
    vtbl: VTable,
    handle: *mut c_void,
    kind: String,
    options: CString,
    // The sink the plugin calls; boxed so its address is stable. Dropped
    // only after the plugin is closed.
    sink: Option<Box<RxSink>>,
    // Last: the library outlives the handle (see Drop).
    library: Option<libloading::Library>,
}

// SAFETY: the plugin instance is only used through `&mut self` (the ABI's
// one-thread-at-a-time rule); the sink it calls is Send + Sync.
unsafe impl Send for SoTransport {}

unsafe extern "C" fn trampoline(sink_ctx: *mut c_void, frame: *const u8, len: u32) {
    let sink = &*(sink_ctx as *const RxSink);
    let frame = if len == 0 { &[][..] } else { std::slice::from_raw_parts(frame, len as usize) };
    let _ = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| sink(frame)));
}

fn error(what: &str, e: impl std::fmt::Display) -> TransportError {
    TransportError(format!("{what}: {e}"))
}

impl SoTransport {
    /// Load the transport plugin at `path` (checked against `sha256` when
    /// set). `kind` must match the plugin's; `options` are the manifest's
    /// [transport] options, handed to the plugin as TOML text at open.
    pub fn load(path: &Path, sha256: Option<&str>, kind: &str, options: &toml::Table) -> Result<Self, TransportError> {
        let where_ = path.display().to_string();
        let bytes = std::fs::read(path).map_err(|e| error(&where_, e))?;
        if let Some(want) = sha256 {
            let got = sha256_hex(&bytes);
            if !want.eq_ignore_ascii_case(&got) {
                return Err(TransportError(format!("{where_}: sha256 {got} does not match the manifest pin {want}")));
            }
        }
        // SAFETY: loading runs the library's initializers; only the
        // manifest's transport is loaded, and it can be pinned above.
        let library = unsafe { libloading::Library::new(path) }.map_err(|e| error(&where_, e))?;
        let entry: Entry = unsafe { *library.get::<Entry>(ENTRY).map_err(|e| error(&format!("{where_}: no xgc_rt_transport_v1"), e))? };
        let desc = unsafe { entry() };
        if desc.is_null() {
            return Err(TransportError(format!("{where_}: descriptor is null")));
        }
        let desc = unsafe { &*desc };
        if desc.abi_version != VERSION {
            return Err(TransportError(format!("{where_}: transport ABI version {} (host speaks {VERSION})", desc.abi_version)));
        }
        if desc.vtbl.is_null() || desc.kind.is_null() {
            return Err(TransportError(format!("{where_}: malformed descriptor")));
        }
        let vtbl = unsafe { *desc.vtbl };
        let plugin_kind = unsafe { CStr::from_ptr(desc.kind) }.to_str().map_err(|e| error(&where_, e))?;
        if plugin_kind != kind {
            return Err(TransportError(format!("{where_}: a {plugin_kind:?} transport, but the manifest says kind = {kind:?}")));
        }
        let missing = [
            vtbl.create.is_none(), vtbl.open.is_none(), vtbl.declare_out.is_none(), vtbl.declare_in.is_none(),
            vtbl.send.is_none(), vtbl.wait_ready.is_none(), vtbl.close.is_none(), vtbl.destroy.is_none(), vtbl.last_error.is_none(),
        ];
        if missing.iter().any(|&m| m) {
            return Err(TransportError(format!("{where_}: incomplete vtable")));
        }
        // Options are pure config. Create only after they are ready: a
        // failure here must not leave a plugin instance without a Drop.
        let options = CString::new(toml::to_string(options).map_err(|e| error("transport options", e))?).map_err(|e| error("transport options", e))?;
        let handle = unsafe { (vtbl.create.unwrap())() };
        if handle.is_null() {
            return Err(TransportError(format!("{where_}: create failed")));
        }
        Ok(Self { vtbl, handle, kind: plugin_kind.to_owned(), options, sink: None, library: Some(library) })
    }

    fn check(&self, status: i32, what: &str) -> Result<(), TransportError> {
        if status == OK {
            return Ok(());
        }
        let reason = unsafe { CStr::from_ptr((self.vtbl.last_error.unwrap())(self.handle)) }.to_string_lossy().into_owned();
        Err(TransportError(format!("{} transport {what}: {reason}", self.kind)))
    }
}

impl Transport for SoTransport {
    fn kind(&self) -> &str {
        &self.kind
    }

    fn open(&mut self, ctx: &TransportContext, sink: RxSink) -> Result<(), TransportError> {
        let cstr = |s: &str| CString::new(s).map_err(|e| error("transport context", e));
        let session = cstr(&ctx.session)?;
        let node = cstr(&ctx.node)?;
        let roster: Vec<CString> = ctx.roster.iter().map(|r| cstr(r)).collect::<Result<_, _>>()?;
        let roster_ptrs: Vec<*const c_char> = roster.iter().map(|r| r.as_ptr()).collect();
        let names: Vec<CString> = ctx.channels.iter().map(|c| cstr(&c.name)).collect::<Result<_, _>>()?;
        let channels: Vec<Channel> =
            ctx.channels.iter().zip(&names).map(|(c, n)| Channel { id: c.id, qos: c.qos as i32, name: n.as_ptr() }).collect();
        let abi = Context {
            session: session.as_ptr(),
            node: node.as_ptr(),
            node_id: ctx.node_id,
            reserved: 0,
            roster_count: roster_ptrs.len() as u32,
            roster: roster_ptrs.as_ptr(),
            channel_count: channels.len() as u32,
            channels: channels.as_ptr(),
            options: self.options.as_ptr(),
        };
        let sink = Box::new(sink);
        let sink_ctx = &*sink as *const RxSink as *mut c_void;
        self.sink = Some(sink);
        let status = unsafe { (self.vtbl.open.unwrap())(self.handle, &abi, trampoline, sink_ctx) };
        self.check(status, "open")
    }

    fn declare_out(&mut self, channel: ChannelId) -> Result<(), TransportError> {
        let status = unsafe { (self.vtbl.declare_out.unwrap())(self.handle, channel) };
        self.check(status, "declare_out")
    }

    fn declare_in(&mut self, channel: ChannelId, origins: &[OriginId]) -> Result<(), TransportError> {
        let status = unsafe { (self.vtbl.declare_in.unwrap())(self.handle, channel, origins.as_ptr(), origins.len() as u32) };
        self.check(status, "declare_in")
    }

    fn send(&mut self, channel: ChannelId, frame: &[u8]) -> Result<(), TransportError> {
        let status = unsafe { (self.vtbl.send.unwrap())(self.handle, channel, frame.as_ptr(), frame.len() as u32) };
        self.check(status, "send")
    }

    fn wait_ready(&mut self, timeout: std::time::Duration) -> bool {
        unsafe { (self.vtbl.wait_ready.unwrap())(self.handle, timeout.as_nanos().min(u64::MAX as u128) as u64) == 1 }
    }

    fn close(&mut self) {
        unsafe { (self.vtbl.close.unwrap())(self.handle) };
    }
}

impl Drop for SoTransport {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe {
                (self.vtbl.close.unwrap())(self.handle);
                (self.vtbl.destroy.unwrap())(self.handle);
            }
            self.handle = std::ptr::null_mut();
        }
        // The sink only after the plugin is gone, the library last.
        self.sink = None;
        drop(self.library.take());
    }
}
