//! Shared test support: build the plugin libraries once per test binary
//! and write manifests.

#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;

pub fn workspace_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../..").canonicalize().unwrap()
}

/// Directory holding `libstub_{perception,estimation,planning,control}.so`
/// and `libc_stub.so`. The Rust stubs are built by a nested cargo in a
/// separate target directory, so the nested build never waits on the outer
/// build lock. The C stub is built with the system C compiler against
/// `abi/include/xgc_rt.h`.
pub fn plugin_dir() -> &'static Path {
    static DIR: OnceLock<PathBuf> = OnceLock::new();
    DIR.get_or_init(|| {
        let root = workspace_root();
        let target = root.join("target/plugin-tests");
        let status = Command::new(env!("CARGO"))
            .current_dir(&root)
            .args(["build", "-q", "-p", "stub-perception", "-p", "stub-estimation", "-p", "stub-planning", "-p", "stub-control", "-p", "dmpc-exchange-demo", "--target-dir"])
            .arg(&target)
            .status()
            .expect("run cargo");
        assert!(status.success(), "building the stub plugins failed");
        let out = target.join("debug");
        let cc = std::env::var("CC").unwrap_or_else(|_| "cc".into());
        let status = Command::new(cc)
            .args(["-std=c11", "-Wall", "-Wextra", "-Werror", "-fPIC", "-shared", "-I"])
            .arg(root.join("abi/include"))
            .arg(root.join("plugins/c-stub/c_stub.c"))
            .arg("-o")
            .arg(out.join("libc_stub.so"))
            .status()
            .expect("run cc");
        assert!(status.success(), "building the C stub failed");
        out
    })
}

pub fn scratch(name: &str) -> PathBuf {
    let dir = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

pub fn lib(name: &str) -> String {
    plugin_dir().join(format!("lib{name}.so")).display().to_string()
}

/// Build `libest_hover_thrust.so` and its replay reference with `$CXX`
/// (default `c++`) from the upstream hover_thrust_estimator sources.
pub fn est_hover_thrust() -> &'static (PathBuf, PathBuf) {
    static OUT: OnceLock<(PathBuf, PathBuf)> = OnceLock::new();
    OUT.get_or_init(|| {
        let root = workspace_root();
        let out = root.join("target/plugin-tests/cpp");
        std::fs::create_dir_all(&out).unwrap();
        let (lib, reference) = (out.join("libest_hover_thrust.so"), out.join("hte_reference"));
        let status = Command::new(root.join("scripts/build-est-hover-thrust.sh"))
            .arg(&lib)
            .arg(&reference)
            .status()
            .expect("run build-est-hover-thrust.sh (needs a C++17 compiler in $CXX or c++)");
        assert!(status.success(), "building est-hover-thrust failed");
        (lib, reference)
    })
}

/// Build `libctl_dfbc.so` and its replay reference (needs `$CXX` and
/// `$EIGEN_INCLUDE`, default /usr/include/eigen3).
pub fn ctl_dfbc() -> &'static (PathBuf, PathBuf) {
    static OUT: OnceLock<(PathBuf, PathBuf)> = OnceLock::new();
    OUT.get_or_init(|| {
        let root = workspace_root();
        let out = root.join("target/plugin-tests/cpp");
        std::fs::create_dir_all(&out).unwrap();
        let (lib, reference) = (out.join("libctl_dfbc.so"), out.join("dfbc_reference"));
        let status = Command::new(root.join("scripts/build-ctl-dfbc.sh"))
            .arg(&lib)
            .arg(&reference)
            .status()
            .expect("run build-ctl-dfbc.sh (needs $CXX and Eigen headers in $EIGEN_INCLUDE)");
        assert!(status.success(), "building ctl-dfbc failed");
        (lib, reference)
    })
}
