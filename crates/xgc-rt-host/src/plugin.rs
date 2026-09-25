//! Loading a plugin library and validating its descriptor against the ABI
//! and the manifest, before any instance is created.

use std::collections::BTreeSet;
use std::ffi::CStr;
use std::path::Path;

use sha2::{Digest, Sha256};
use xgc_rt_abi::*;
use xgc_rt_core::transport::Qos;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PortInfo {
    pub name: String,
    pub is_out: bool,
    pub schema_id: String,
    pub qos: Qos,
}

/// A validated plugin library. The vtable is copied out, so its function
/// pointers stay valid exactly as long as `_library` is alive.
pub struct LoadedPlugin {
    pub name: String,
    pub version: String,
    pub sha256: String,
    pub ports: Vec<PortInfo>,
    pub vtbl: XgcPluginVtblCopy,
    _library: libloading::Library,
}

#[derive(Clone, Copy)]
pub struct XgcPluginVtblCopy {
    pub create: unsafe extern "C" fn(*const XgcHostApi) -> *mut std::ffi::c_void,
    pub configure: unsafe extern "C" fn(*mut std::ffi::c_void, *const std::ffi::c_char) -> XgcStatus,
    pub activate: unsafe extern "C" fn(*mut std::ffi::c_void) -> XgcStatus,
    pub step: unsafe extern "C" fn(*mut std::ffi::c_void, *const XgcStepCtx) -> XgcStatus,
    pub deactivate: unsafe extern "C" fn(*mut std::ffi::c_void) -> XgcStatus,
    pub destroy: unsafe extern "C" fn(*mut std::ffi::c_void),
    pub domain_state: unsafe extern "C" fn(*mut std::ffi::c_void) -> *const std::ffi::c_char,
}

#[derive(Debug)]
pub struct LoadError(pub String);

impl std::fmt::Display for LoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for LoadError {}

pub fn sha256_hex(bytes: &[u8]) -> String {
    Sha256::digest(bytes).iter().map(|b| format!("{b:02x}")).collect()
}

fn text(ptr: *const std::ffi::c_char, what: &str) -> Result<String, LoadError> {
    if ptr.is_null() {
        return Err(LoadError(format!("{what} is null")));
    }
    // SAFETY: descriptor strings are NUL-terminated and 'static per the ABI.
    let s = unsafe { CStr::from_ptr(ptr) };
    s.to_str().map(str::to_owned).map_err(|_| LoadError(format!("{what} is not UTF-8")))
}

/// Load `path`, check its digest when `expected_sha256` is set, and
/// validate the descriptor.
pub fn load(path: &Path, expected_sha256: Option<&str>) -> Result<LoadedPlugin, LoadError> {
    let bytes = std::fs::read(path).map_err(|e| LoadError(format!("{}: {e}", path.display())))?;
    let digest = sha256_hex(&bytes);
    if let Some(want) = expected_sha256 {
        if !want.eq_ignore_ascii_case(&digest) {
            return Err(LoadError(format!("{}: sha256 {digest} does not match the manifest pin {want}", path.display())));
        }
    }
    // SAFETY: loading runs the library's initializers. Only manifest-listed
    // libraries are loaded, and they can be pinned by digest above.
    let library = unsafe { libloading::Library::new(path) }.map_err(|e| LoadError(format!("{}: {e}", path.display())))?;
    let entry: XgcPluginEntry = unsafe {
        *library
            .get::<XgcPluginEntry>(XGC_PLUGIN_ENTRY_SYMBOL)
            .map_err(|e| LoadError(format!("{}: no xgc_rt_plugin_v1: {e}", path.display())))?
    };
    let desc_ptr = unsafe { entry() };
    if desc_ptr.is_null() {
        return Err(LoadError(format!("{}: descriptor is null", path.display())));
    }
    let desc = unsafe { &*desc_ptr };
    if desc.abi_version != XGC_RT_ABI_VERSION {
        return Err(LoadError(format!("{}: ABI version {} (host speaks {XGC_RT_ABI_VERSION})", path.display(), desc.abi_version)));
    }
    if desc.port_count > XGC_RT_MAX_PORTS || (desc.port_count > 0 && desc.ports.is_null()) || desc.vtbl.is_null() {
        return Err(LoadError(format!("{}: malformed descriptor", path.display())));
    }
    let name = text(desc.name, "plugin name")?;
    let version = text(desc.version, "plugin version")?;
    let mut ports = Vec::new();
    let mut names = BTreeSet::new();
    for i in 0..desc.port_count as usize {
        let p = unsafe { &*desc.ports.add(i) };
        let port_name = text(p.name, "port name")?;
        if !names.insert(port_name.clone()) {
            return Err(LoadError(format!("{name}: port {port_name} is declared twice")));
        }
        let is_out = match p.dir {
            XGC_PORT_IN => false,
            XGC_PORT_OUT => true,
            other => return Err(LoadError(format!("{name}: port {port_name} has direction {other}"))),
        };
        let qos = Qos::from_abi(p.qos).ok_or_else(|| LoadError(format!("{name}: port {port_name} has QoS {}", p.qos)))?;
        ports.push(PortInfo { name: port_name, is_out, schema_id: text(p.schema_id, "schema id")?, qos });
    }
    let v = unsafe { &*desc.vtbl };
    let missing = |f: &str| LoadError(format!("{name}: vtable entry {f} is NULL"));
    let vtbl = XgcPluginVtblCopy {
        create: v.create.ok_or_else(|| missing("create"))?,
        configure: v.configure.ok_or_else(|| missing("configure"))?,
        activate: v.activate.ok_or_else(|| missing("activate"))?,
        step: v.step.ok_or_else(|| missing("step"))?,
        deactivate: v.deactivate.ok_or_else(|| missing("deactivate"))?,
        destroy: v.destroy.ok_or_else(|| missing("destroy"))?,
        domain_state: v.domain_state.ok_or_else(|| missing("domain_state"))?,
    };
    Ok(LoadedPlugin { name, version, sha256: digest, ports, vtbl, _library: library })
}
