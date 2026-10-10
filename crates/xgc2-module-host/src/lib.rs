//! xgc2-module host: runs the algorithm modules of one entity in one process.
//!
//! See `docs/architecture.md` for the design; the module ABI is `include/xgc2/module.h`.

pub mod abi;
pub mod api;
pub mod channel;
pub mod clock;
pub mod control;
pub mod host;
pub mod instance;
pub mod launch;
pub mod loader;
pub mod log;
pub mod manifest;
pub mod metrics;
pub mod names;
pub mod plan;
pub mod scheduler;
pub mod timers;
