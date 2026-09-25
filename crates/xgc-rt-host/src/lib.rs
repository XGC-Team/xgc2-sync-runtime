//! `xgc-rt-host`: loads module plugins by manifest and runs them on
//! Session rounds. It stamps, audits and sends through one transport.

pub mod endpoint;
pub mod host;
pub mod plugin;

pub use host::{Host, HostError, HostOptions, RunSummary};
