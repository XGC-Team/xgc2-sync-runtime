//! Process-owned XRPC binding exported through the official C ABI to modules.
use std::{io, sync::Arc};

use xgc2_xrpc::{
    ffi::{HttpCapsV1, RuntimeExport},
    Limits, Runtime, RuntimeHandle, RuntimeOptions,
};

use crate::plugin::LoadedPlugin;

/// One process composition's resolved runtime and HTTP limits. Cloning shares
/// the same owner and pools; it never creates a module-specific runtime.
#[derive(Clone)]
pub struct RpcBinding {
    handle: RuntimeHandle,
    limits: Limits,
    // Library users without an injected process owner get one explicit owner
    // per aggregate Host, retained by every export until its last real user.
    owner: Option<Arc<Runtime>>,
}

impl RpcBinding {
    /// Inject the process owner's already resolved policy. This does not read
    /// environment variables or create any execution resources.
    pub fn new(handle: RuntimeHandle, limits: Limits) -> io::Result<Self> {
        HttpCapsV1::from_limits(&limits)?;
        Ok(Self {
            handle,
            limits,
            owner: None,
        })
    }

    pub fn handle(&self) -> &RuntimeHandle {
        &self.handle
    }

    pub fn limits(&self) -> &Limits {
        &self.limits
    }

    pub(crate) fn owned_default() -> io::Result<Self> {
        // SDK defaults are a fixed generated policy, with four blocking
        // workers. Only the binary composition root resolves process env.
        let owner = Arc::new(Runtime::new(RuntimeOptions::default())?);
        let mut binding = Self::new(owner.handle(), Limits::default())?;
        binding.owner = Some(owner);
        Ok(binding)
    }

    pub(crate) fn export(&self, library: Arc<LoadedPlugin>) -> io::Result<RuntimeExport> {
        // The origin table's retained context must pin actual module code.
        // It also retains an optional library-mode owner, so retaining a C
        // factory never causes its SDK owner to close behind the consumer.
        let code_pin: Arc<dyn Send + Sync> = Arc::new(ModuleCodePin {
            _library: library,
            _binding: self.clone(),
        });
        RuntimeExport::new(self.handle.clone(), self.limits.clone(), code_pin)
    }
}

struct ModuleCodePin {
    _library: Arc<LoadedPlugin>,
    _binding: RpcBinding,
}
