//! The transport plugin ABI (abi/include/xgc_rt.h, `xgc_rt_transport_v1`),
//! exact mirror, and the exporter: `export_transport!` turns any
//! [`Transport`] into a loadable transport plugin. The host's loader is
//! xgc-rt-host's `SoTransport`, which is a [`Transport`] again, so the
//! in-tree trait and the C ABI are one contract seen from both sides.

use std::ffi::{c_char, c_void, CStr, CString};
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::Arc;

use crate::transport::{ChannelSpec, Qos, RxSink, Transport, TransportContext, TransportError};

pub const VERSION: u32 = 1;
pub const ENTRY: &[u8] = b"xgc_rt_transport_v1\0";

// xgc_status
pub const OK: i32 = 0;
pub const ERR: i32 = 1;
pub const ERR_INVALID: i32 = 2;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Channel {
    pub id: u32,
    pub qos: i32,
    pub name: *const c_char,
}

#[repr(C)]
pub struct Context {
    pub session: *const c_char,
    pub node: *const c_char,
    pub node_id: u16,
    pub reserved: u16,
    pub roster_count: u32,
    pub roster: *const *const c_char,
    pub channel_count: u32,
    pub channels: *const Channel,
    pub options: *const c_char,
}

pub type Sink = unsafe extern "C" fn(sink_ctx: *mut c_void, frame: *const u8, len: u32);

#[repr(C)]
#[derive(Clone, Copy)]
pub struct VTable {
    pub create: Option<unsafe extern "C" fn() -> *mut c_void>,
    pub open: Option<unsafe extern "C" fn(*mut c_void, *const Context, Sink, *mut c_void) -> i32>,
    pub declare_out: Option<unsafe extern "C" fn(*mut c_void, u32) -> i32>,
    pub declare_in: Option<unsafe extern "C" fn(*mut c_void, u32, *const u16, u32) -> i32>,
    pub send: Option<unsafe extern "C" fn(*mut c_void, u32, *const u8, u32) -> i32>,
    pub wait_ready: Option<unsafe extern "C" fn(*mut c_void, u64) -> i32>,
    pub close: Option<unsafe extern "C" fn(*mut c_void)>,
    pub destroy: Option<unsafe extern "C" fn(*mut c_void)>,
    pub last_error: Option<unsafe extern "C" fn(*mut c_void) -> *const c_char>,
}

#[repr(C)]
pub struct Descriptor {
    pub abi_version: u32,
    pub reserved: u32,
    pub kind: *const c_char,
    pub vtbl: *const VTable,
}

/// A descriptor in a `static` (its pointers are to statics).
#[repr(transparent)]
pub struct StaticDescriptor(pub Descriptor);
// SAFETY: the pointers are to 'static, immutable data.
unsafe impl Sync for StaticDescriptor {}

pub type Entry = unsafe extern "C" fn() -> *const Descriptor;

/// Builds the plugin's transport at `open`, from the host's context and the
/// manifest's [transport] options (without kind, path and sha256).
pub type Factory = fn(&TransportContext, &toml::Table) -> Result<Box<dyn Transport>, TransportError>;

/// The context the host passes, as the in-tree type. The pointers must be
/// valid, NUL-terminated UTF-8, as `xgc_transport_context` requires.
///
/// # Safety
/// `ctx` points to a valid `xgc_transport_context`.
pub unsafe fn context_from_abi(ctx: &Context) -> Result<(TransportContext, toml::Table), TransportError> {
    let text = |p: *const c_char, what: &str| -> Result<String, TransportError> {
        if p.is_null() {
            return Err(TransportError(format!("{what} is null")));
        }
        CStr::from_ptr(p).to_str().map(str::to_owned).map_err(|_| TransportError(format!("{what} is not UTF-8")))
    };
    let mut roster = Vec::with_capacity(ctx.roster_count as usize);
    for i in 0..ctx.roster_count as usize {
        roster.push(text(*ctx.roster.add(i), "roster name")?);
    }
    let mut channels = Vec::with_capacity(ctx.channel_count as usize);
    for i in 0..ctx.channel_count as usize {
        let c = &*ctx.channels.add(i);
        let qos = Qos::from_abi(c.qos).ok_or_else(|| TransportError(format!("channel {}: bad qos {}", c.id, c.qos)))?;
        channels.push(ChannelSpec { id: c.id, name: text(c.name, "channel name")?, qos });
    }
    let options = text(ctx.options, "options")?;
    let options: toml::Table = options.parse().map_err(|e| TransportError(format!("transport options: {e}")))?;
    Ok((
        TransportContext { session: text(ctx.session, "session")?, node: text(ctx.node, "node")?, node_id: ctx.node_id, roster, channels },
        options,
    ))
}

/// One exported transport instance.
pub struct Exported {
    factory: Factory,
    transport: Option<Box<dyn Transport>>,
    error: CString,
}

impl Exported {
    fn status(&mut self, r: Result<(), TransportError>) -> i32 {
        match r {
            Ok(()) => OK,
            Err(e) => {
                self.error = CString::new(e.0.replace('\0', " ")).unwrap_or_default();
                ERR
            }
        }
    }

    fn transport(&mut self) -> Result<&mut Box<dyn Transport>, TransportError> {
        self.transport.as_mut().ok_or_else(|| TransportError("transport is not open".into()))
    }
}

/// Run `f` on the instance behind `this`, turning a panic into ERR.
unsafe fn with(this: *mut c_void, f: impl FnOnce(&mut Exported) -> i32) -> i32 {
    if this.is_null() {
        return ERR_INVALID;
    }
    let e = &mut *(this as *mut Exported);
    match catch_unwind(AssertUnwindSafe(|| f(&mut *e))) {
        Ok(status) => status,
        Err(_) => {
            e.error = CString::new("transport panicked").unwrap();
            ERR
        }
    }
}

/// The vtable functions `export_transport!` wires to one factory.
#[doc(hidden)]
pub mod export {
    use super::*;

    pub fn create(factory: Factory) -> *mut c_void {
        Box::into_raw(Box::new(Exported { factory, transport: None, error: CString::default() })) as *mut c_void
    }

    /// # Safety
    /// `this` from `create`; `ctx` valid for the call.
    pub unsafe fn open(this: *mut c_void, ctx: *const Context, sink: Sink, sink_ctx: *mut c_void) -> i32 {
        if ctx.is_null() {
            return ERR_INVALID;
        }
        with(this, |e| {
            let r = (|| {
                let (ctx, options) = context_from_abi(&*ctx)?;
                let mut t = (e.factory)(&ctx, &options)?;
                let sink_ctx = sink_ctx as usize;
                let rx: RxSink = Arc::new(move |frame: &[u8]| unsafe { sink(sink_ctx as *mut c_void, frame.as_ptr(), frame.len() as u32) });
                t.open(&ctx, rx)?;
                e.transport = Some(t);
                Ok(())
            })();
            e.status(r)
        })
    }

    /// # Safety
    /// `this` from `create`.
    pub unsafe fn declare_out(this: *mut c_void, channel: u32) -> i32 {
        with(this, |e| {
            let r = e.transport().and_then(|t| t.declare_out(channel));
            e.status(r)
        })
    }

    /// # Safety
    /// `this` from `create`; `origins` holds `count` ids.
    pub unsafe fn declare_in(this: *mut c_void, channel: u32, origins: *const u16, count: u32) -> i32 {
        let origins = if count == 0 { &[][..] } else { std::slice::from_raw_parts(origins, count as usize) };
        with(this, |e| {
            let r = e.transport().and_then(|t| t.declare_in(channel, origins));
            e.status(r)
        })
    }

    /// # Safety
    /// `this` from `create`; `frame` holds `len` bytes.
    pub unsafe fn send(this: *mut c_void, channel: u32, frame: *const u8, len: u32) -> i32 {
        let frame = if len == 0 { &[][..] } else { std::slice::from_raw_parts(frame, len as usize) };
        with(this, |e| {
            let r = e.transport().and_then(|t| t.send(channel, frame));
            e.status(r)
        })
    }

    /// # Safety
    /// `this` from `create`.
    pub unsafe fn wait_ready(this: *mut c_void, timeout_ns: u64) -> i32 {
        with(this, |e| match e.transport() {
            Ok(t) => i32::from(t.wait_ready(std::time::Duration::from_nanos(timeout_ns))),
            Err(_) => 0,
        })
    }

    /// # Safety
    /// `this` from `create`.
    pub unsafe fn close(this: *mut c_void) {
        with(this, |e| {
            if let Some(t) = e.transport.as_mut() {
                t.close();
            }
            OK
        });
    }

    /// # Safety
    /// `this` from `create`, not used afterwards.
    pub unsafe fn destroy(this: *mut c_void) {
        if !this.is_null() {
            let _ = catch_unwind(AssertUnwindSafe(|| drop(Box::from_raw(this as *mut Exported))));
        }
    }

    /// # Safety
    /// `this` from `create`.
    pub unsafe fn last_error(this: *mut c_void) -> *const c_char {
        if this.is_null() {
            return b"\0".as_ptr() as *const c_char;
        }
        (*(this as *mut Exported)).error.as_ptr()
    }
}

/// Export a [`Transport`] as a transport plugin: `export_transport!("zenoh",
/// factory)` defines `xgc_rt_transport_v1` for a cdylib crate; `factory` is
/// a [`Factory`] function in the calling module.
#[macro_export]
macro_rules! export_transport {
    ($kind:literal, $factory:ident) => {
        mod __xgc_transport_export {
            use std::ffi::{c_char, c_void};
            use $crate::transport_abi::{export, Context, Descriptor, Sink, StaticDescriptor, VTable, VERSION};

            unsafe extern "C" fn create() -> *mut c_void {
                export::create(super::$factory)
            }
            unsafe extern "C" fn open(this: *mut c_void, ctx: *const Context, sink: Sink, sink_ctx: *mut c_void) -> i32 {
                export::open(this, ctx, sink, sink_ctx)
            }
            unsafe extern "C" fn declare_out(this: *mut c_void, channel: u32) -> i32 {
                export::declare_out(this, channel)
            }
            unsafe extern "C" fn declare_in(this: *mut c_void, channel: u32, origins: *const u16, count: u32) -> i32 {
                export::declare_in(this, channel, origins, count)
            }
            unsafe extern "C" fn send(this: *mut c_void, channel: u32, frame: *const u8, len: u32) -> i32 {
                export::send(this, channel, frame, len)
            }
            unsafe extern "C" fn wait_ready(this: *mut c_void, timeout_ns: u64) -> i32 {
                export::wait_ready(this, timeout_ns)
            }
            unsafe extern "C" fn close(this: *mut c_void) {
                export::close(this)
            }
            unsafe extern "C" fn destroy(this: *mut c_void) {
                export::destroy(this)
            }
            unsafe extern "C" fn last_error(this: *mut c_void) -> *const c_char {
                export::last_error(this)
            }

            static VTBL: VTable = VTable {
                create: Some(create),
                open: Some(open),
                declare_out: Some(declare_out),
                declare_in: Some(declare_in),
                send: Some(send),
                wait_ready: Some(wait_ready),
                close: Some(close),
                destroy: Some(destroy),
                last_error: Some(last_error),
            };
            static DESC: StaticDescriptor = StaticDescriptor(Descriptor {
                abi_version: VERSION,
                reserved: 0,
                kind: concat!($kind, "\0").as_ptr() as *const c_char,
                vtbl: &VTBL,
            });

            #[no_mangle]
            pub extern "C" fn xgc_rt_transport_v1() -> *const Descriptor {
                &DESC.0
            }
        }
    };
}
