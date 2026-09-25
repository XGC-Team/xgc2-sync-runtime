//! Audit plane: lossless per-message records ([`recorder::FileAudit`]) and
//! the offline multi-node merge ([`merge::merge_run`]). The merge computes
//! `audit-def/1` exactly from sender and receiver logs. The definitions are
//! in docs/audit-definitions.md.

pub mod merge;
pub mod record;
pub mod recorder;

pub use merge::{merge_run, write_report, MergeOptions, Report};
pub use recorder::{FileAudit, NodeMeta};
