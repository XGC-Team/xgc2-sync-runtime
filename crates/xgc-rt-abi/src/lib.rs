//! Rust mirror of `abi/include/xgc_rt.h` (ABI v1) and the safe plugin SDK.
//!
//! The C header is the contract. The `#[repr(C)]` types here must match it
//! field for field, and `tests/layout.rs` checks their sizes and offsets.
//! A Rust plugin implements [`Plugin`] and calls [`export_plugin!`]. It never
//! touches the raw types.

use std::ffi::{c_char, c_void, CStr, CString};
use std::panic::{catch_unwind, AssertUnwindSafe};

pub mod neighbor;

pub const XGC_RT_ABI_VERSION: u32 = 1;
pub const XGC_RT_MAX_PORTS: u32 = 64;
pub const XGC_RT_ABI_MINOR: u32 = 1;

/// `xgc_status`. It is kept as a plain integer so that an out-of-range value
/// from a foreign plugin is a checked error, never undefined behaviour.
pub type XgcStatus = i32;
pub const XGC_OK: XgcStatus = 0;
pub const XGC_ERR: XgcStatus = 1;
pub const XGC_ERR_INVALID: XgcStatus = 2;
pub const XGC_ERR_AGAIN: XgcStatus = 3;

pub type XgcPortDir = i32;
pub const XGC_PORT_IN: XgcPortDir = 0;
pub const XGC_PORT_OUT: XgcPortDir = 1;

pub type XgcQos = i32;
pub const XGC_QOS_CONTROL: XgcQos = 0;
pub const XGC_QOS_STATE: XgcQos = 1;
pub const XGC_QOS_EVENT: XgcQos = 2;
pub const XGC_QOS_BULK: XgcQos = 3;

pub type XgcLogLevel = i32;
pub const XGC_LOG_DEBUG: XgcLogLevel = 0;
pub const XGC_LOG_INFO: XgcLogLevel = 1;
pub const XGC_LOG_WARN: XgcLogLevel = 2;
pub const XGC_LOG_ERROR: XgcLogLevel = 3;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct XgcPortDecl {
    pub name: *const c_char,
    pub dir: XgcPortDir,
    pub schema_id: *const c_char,
    pub qos: XgcQos,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct XgcSampleView {
    pub origin: u16,
    pub reserved: u16,
    pub len: u32,
    pub seq: u64,
    pub round: u64,
    pub t_produce: i64,
    pub t_tx: i64,
    pub t_rx: i64,
    pub data: *const u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct XgcStepCtx {
    pub round: u64,
    pub now: i64,
    pub round_start: i64,
    pub deadline: i64,
    pub dirty_ports: u64,
    pub round_advanced: u32,
    pub reserved: u32,
}

#[repr(C)]
pub struct XgcHostApi {
    pub abi_version: u32,
    pub abi_minor: u32,
    pub host: *mut c_void,
    pub publish: unsafe extern "C" fn(*mut c_void, u32, u64, *const u8, u32) -> XgcStatus,
    pub next: unsafe extern "C" fn(*mut c_void, u32, *mut XgcSampleView) -> XgcStatus,
    pub now: unsafe extern "C" fn(*mut c_void) -> i64,
    pub log: unsafe extern "C" fn(*mut c_void, XgcLogLevel, *const c_char),
    pub request_degrade: unsafe extern "C" fn(*mut c_void, *const c_char),
    pub request_recover: unsafe extern "C" fn(*mut c_void),
    /// abi_minor >= 1
    pub port_origins: unsafe extern "C" fn(*mut c_void, u32, *mut u16, u32) -> u32,
    /// abi_minor >= 1
    pub node_id: unsafe extern "C" fn(*mut c_void) -> u16,
}

/// Entries are `Option` because a C plugin may leave one NULL. The host
/// rejects such a descriptor at load instead of calling a null pointer.
#[repr(C)]
pub struct XgcPluginVtbl {
    pub create: Option<unsafe extern "C" fn(*const XgcHostApi) -> *mut c_void>,
    pub configure: Option<unsafe extern "C" fn(*mut c_void, *const c_char) -> XgcStatus>,
    pub activate: Option<unsafe extern "C" fn(*mut c_void) -> XgcStatus>,
    pub step: Option<unsafe extern "C" fn(*mut c_void, *const XgcStepCtx) -> XgcStatus>,
    pub deactivate: Option<unsafe extern "C" fn(*mut c_void) -> XgcStatus>,
    pub destroy: Option<unsafe extern "C" fn(*mut c_void)>,
    pub domain_state: Option<unsafe extern "C" fn(*mut c_void) -> *const c_char>,
}

#[repr(C)]
pub struct XgcPluginDescriptor {
    pub abi_version: u32,
    pub port_count: u32,
    pub name: *const c_char,
    pub version: *const c_char,
    pub ports: *const XgcPortDecl,
    pub vtbl: *const XgcPluginVtbl,
}

pub type XgcPluginEntry = unsafe extern "C" fn() -> *const XgcPluginDescriptor;
pub const XGC_PLUGIN_ENTRY_SYMBOL: &[u8] = b"xgc_rt_plugin_v1\0";

/// Makes raw-pointer descriptor tables usable as `static`s. They are
/// immutable and point only at other `'static` data.
#[repr(transparent)]
pub struct StaticAbi<T>(pub T);
unsafe impl<T> Sync for StaticAbi<T> {}

// ---------------------------------------------------------------------------
// Plugin-side SDK
// ---------------------------------------------------------------------------

/// One received sample, borrowed until the next `Host::next` call.
#[derive(Debug, Clone, Copy)]
pub struct Sample<'a> {
    pub origin: u16,
    pub seq: u64,
    pub round: u64,
    pub t_produce: i64,
    pub t_tx: i64,
    pub t_rx: i64,
    pub data: &'a [u8],
}

/// The plugin's handle to host services. It is only valid inside vtable calls.
pub struct Host {
    api: *const XgcHostApi,
}

impl Host {
    /// # Safety
    /// `api` must be the pointer the host passed to `create`.
    pub unsafe fn from_raw(api: *const XgcHostApi) -> Self {
        Self { api }
    }

    fn api(&self) -> &XgcHostApi {
        // SAFETY: the host keeps the api table alive until `destroy`.
        unsafe { &*self.api }
    }

    pub fn publish(&self, port: u32, round: u64, data: &[u8]) -> Result<(), XgcStatus> {
        let api = self.api();
        let len = u32::try_from(data.len()).map_err(|_| XGC_ERR_INVALID)?;
        // SAFETY: host contract; `data` outlives the call.
        match unsafe { (api.publish)(api.host, port, round, data.as_ptr(), len) } {
            XGC_OK => Ok(()),
            status => Err(status),
        }
    }

    pub fn next(&mut self, port: u32) -> Option<Sample<'_>> {
        let api = self.api();
        let mut view = std::mem::MaybeUninit::<XgcSampleView>::zeroed();
        // SAFETY: host contract; the view stays valid until the next `next`,
        // which `&mut self` enforces through the returned lifetime.
        let status = unsafe { (api.next)(api.host, port, view.as_mut_ptr()) };
        if status != XGC_OK {
            return None;
        }
        let view = unsafe { view.assume_init() };
        let data = if view.len == 0 {
            &[][..]
        } else {
            unsafe { std::slice::from_raw_parts(view.data, view.len as usize) }
        };
        Some(Sample {
            origin: view.origin,
            seq: view.seq,
            round: view.round,
            t_produce: view.t_produce,
            t_tx: view.t_tx,
            t_rx: view.t_rx,
            data,
        })
    }

    pub fn now(&self) -> i64 {
        let api = self.api();
        unsafe { (api.now)(api.host) }
    }

    pub fn log(&self, level: XgcLogLevel, message: &str) {
        let api = self.api();
        let text = CString::new(message.replace('\0', " ")).unwrap_or_default();
        unsafe { (api.log)(api.host, level, text.as_ptr()) }
    }

    pub fn request_degrade(&self, reason: &str) {
        let api = self.api();
        let text = CString::new(reason.replace('\0', " ")).unwrap_or_default();
        unsafe { (api.request_degrade)(api.host, text.as_ptr()) }
    }

    /// Roster ids an in-port receives from; empty on a host older than
    /// ABI minor 1.
    pub fn port_origins(&self, port: u32) -> Vec<u16> {
        let api = self.api();
        if api.abi_minor < 1 {
            return Vec::new();
        }
        let mut ids = vec![0u16; 16];
        loop {
            let n = unsafe { (api.port_origins)(api.host, port, ids.as_mut_ptr(), ids.len() as u32) } as usize;
            if n <= ids.len() {
                ids.truncate(n);
                return ids;
            }
            ids.resize(n, 0);
        }
    }

    /// This node's roster id, or None on a host older than ABI minor 1.
    pub fn node_id(&self) -> Option<u16> {
        let api = self.api();
        (api.abi_minor >= 1).then(|| unsafe { (api.node_id)(api.host) })
    }

    pub fn request_recover(&self) {
        let api = self.api();
        unsafe { (api.request_recover)(api.host) }
    }
}

/// Safe plugin interface. Errors become `XGC_ERR`, and a panic is caught at
/// the boundary and also becomes `XGC_ERR`, so the host moves the plugin to
/// `Error` and never unwinds.
pub trait Plugin: Sized + 'static {
    fn create(host: Host) -> Self;
    fn configure(&mut self, _config: &str) -> Result<(), String> {
        Ok(())
    }
    fn activate(&mut self) -> Result<(), String> {
        Ok(())
    }
    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String>;
    fn deactivate(&mut self) -> Result<(), String> {
        Ok(())
    }
    /// Current domain FSM state name, e.g. `cstr!("tracking")`.
    fn domain_state(&self) -> &'static CStr;
}

fn guarded(f: impl FnOnce() -> Result<(), String>) -> XgcStatus {
    match catch_unwind(AssertUnwindSafe(f)) {
        Ok(Ok(())) => XGC_OK,
        Ok(Err(_)) | Err(_) => XGC_ERR,
    }
}

#[doc(hidden)]
pub mod shim {
    use super::*;

    pub unsafe extern "C" fn create<P: Plugin>(api: *const XgcHostApi) -> *mut c_void {
        match catch_unwind(AssertUnwindSafe(|| Box::new(P::create(Host::from_raw(api))))) {
            Ok(plugin) => Box::into_raw(plugin).cast(),
            Err(_) => std::ptr::null_mut(),
        }
    }

    pub unsafe extern "C" fn configure<P: Plugin>(this: *mut c_void, config: *const c_char) -> XgcStatus {
        let plugin = &mut *this.cast::<P>();
        let text = if config.is_null() {
            String::new()
        } else {
            CStr::from_ptr(config).to_string_lossy().into_owned()
        };
        guarded(|| plugin.configure(&text))
    }

    pub unsafe extern "C" fn activate<P: Plugin>(this: *mut c_void) -> XgcStatus {
        let plugin = &mut *this.cast::<P>();
        guarded(|| plugin.activate())
    }

    pub unsafe extern "C" fn step<P: Plugin>(this: *mut c_void, ctx: *const XgcStepCtx) -> XgcStatus {
        let plugin = &mut *this.cast::<P>();
        let ctx = *ctx;
        guarded(|| plugin.step(&ctx))
    }

    pub unsafe extern "C" fn deactivate<P: Plugin>(this: *mut c_void) -> XgcStatus {
        let plugin = &mut *this.cast::<P>();
        guarded(|| plugin.deactivate())
    }

    pub unsafe extern "C" fn destroy<P: Plugin>(this: *mut c_void) {
        if !this.is_null() {
            let _ = catch_unwind(AssertUnwindSafe(|| drop(Box::from_raw(this.cast::<P>()))));
        }
    }

    pub unsafe extern "C" fn domain_state<P: Plugin>(this: *mut c_void) -> *const c_char {
        let plugin = &*this.cast::<P>();
        catch_unwind(AssertUnwindSafe(|| plugin.domain_state().as_ptr())).unwrap_or(b"panicked\0".as_ptr() as *const c_char)
    }
}

/// A `&'static CStr` from a string literal (MSRV-safe form of `c"..."`).
#[macro_export]
macro_rules! cstr {
    ($s:literal) => {
        // SAFETY: a literal plus one trailing NUL; interior NULs are rejected
        // by `CStr::from_bytes_with_nul` in debug builds of the tests.
        unsafe { ::std::ffi::CStr::from_bytes_with_nul_unchecked(concat!($s, "\0").as_bytes()) }
    };
}

/// Export a [`Plugin`] as `xgc_rt_plugin_v1`.
///
/// ```ignore
/// xgc_rt_abi::export_plugin! {
///     plugin: Estimator,
///     name: "stub-estimation",
///     version: "0.1.0",
///     ports: [
///         ("detections", XGC_PORT_IN, "xgc.stub.detections/1", XGC_QOS_STATE),
///         ("state", XGC_PORT_OUT, "xgc.stub.state/1", XGC_QOS_STATE),
///     ],
/// }
/// ```
#[macro_export]
macro_rules! export_plugin {
    (
        plugin: $ty:ty,
        name: $name:literal,
        version: $version:literal,
        ports: [ $( ($pname:literal, $dir:expr, $schema:literal, $qos:expr) ),* $(,)? ] $(,)?
    ) => {
        const _: () = {
            use $crate::*;
            const PORT_COUNT: usize = [$($pname),*].len();
            static PORTS: StaticAbi<[XgcPortDecl; PORT_COUNT]> = StaticAbi([
                $( XgcPortDecl {
                    name: concat!($pname, "\0").as_ptr() as *const ::std::ffi::c_char,
                    dir: $dir,
                    schema_id: concat!($schema, "\0").as_ptr() as *const ::std::ffi::c_char,
                    qos: $qos,
                } ),*
            ]);
            static VTBL: XgcPluginVtbl = XgcPluginVtbl {
                create: Some(shim::create::<$ty>),
                configure: Some(shim::configure::<$ty>),
                activate: Some(shim::activate::<$ty>),
                step: Some(shim::step::<$ty>),
                deactivate: Some(shim::deactivate::<$ty>),
                destroy: Some(shim::destroy::<$ty>),
                domain_state: Some(shim::domain_state::<$ty>),
            };
            static DESCRIPTOR: StaticAbi<XgcPluginDescriptor> = StaticAbi(XgcPluginDescriptor {
                abi_version: XGC_RT_ABI_VERSION,
                port_count: PORT_COUNT as u32,
                name: concat!($name, "\0").as_ptr() as *const ::std::ffi::c_char,
                version: concat!($version, "\0").as_ptr() as *const ::std::ffi::c_char,
                ports: &PORTS.0 as *const [XgcPortDecl; PORT_COUNT] as *const XgcPortDecl,
                vtbl: &VTBL,
            });

            #[no_mangle]
            pub extern "C" fn xgc_rt_plugin_v1() -> *const XgcPluginDescriptor {
                &DESCRIPTOR.0
            }
        };
    };
}
