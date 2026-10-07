# Audit definitions (`audit-def/1`)

- Every audit artifact carries the version string `audit-def/1`.
- The definitions are implemented once, in `crates/xgc-rt-audit/src/merge.rs`; stamps are taken by the host in `crates/xgc-rt-host/src/endpoint.rs`.
- Record layout: `crates/xgc-rt-audit/src/record.rs`. Report: `xgc-rt-audit merge`.

The September 2026 `audit-def/1` definition record is preserved in lxk36/academic, [memory/archive/sync-runtime/audit-definitions.md](https://github.com/lxk36/academic/blob/main/memory/archive/sync-runtime/audit-definitions.md). The [academic topic](https://github.com/lxk36/academic/blob/main/memory/now/sync-runtime.md) records its source and historical scope. Current implementation paths are listed above.
