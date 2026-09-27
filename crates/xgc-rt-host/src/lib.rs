//! `xgc-rt-host`: loads module plugins by manifest and runs them on
//! Session rounds. It stamps, audits and sends through one transport.

pub mod clock_service;
pub mod clock_source;
pub mod clock_source_abi;
pub mod deployment;
pub mod endpoint;
pub mod host;
pub mod plugin;
pub mod transport_so;

pub use host::{Host, HostError, HostOptions, RunSummary};
