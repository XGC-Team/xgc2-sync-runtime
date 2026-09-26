# Audit definitions (`audit-def/1`)

- Every audit artifact carries the version string `audit-def/1`.
- The definitions are implemented once, in `crates/xgc-rt-audit/src/merge.rs`; stamps are taken by the host in `crates/xgc-rt-host/src/endpoint.rs`.
- Record layout: `crates/xgc-rt-audit/src/record.rs`. Report: `xgc-rt-audit merge`.

Decisions, rationale and plans for this runtime live in the academic knowledge base: lxk36/academic, `docs/architecture/xgc2-sync-runtime/audit-definitions.md` (the definitions claims cite).
