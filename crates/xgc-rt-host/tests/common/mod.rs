//! Shared test support: build the plugin libraries once per test binary
//! and write manifests.

#![allow(dead_code)]

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;

pub mod px4_plant;

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
            .args(["build", "-q", "-p", "stub-perception", "-p", "stub-estimation", "-p", "stub-planning", "-p", "stub-control", "-p", "dmpc-exchange-demo", "-p", "dmpc-rounds", "-p", "transport-loopback", "-p", "transport-zenoh", "--target-dir"])
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

/// True when every `outputs` file exists and is newer than every file under
/// `inputs` (directories are walked), so a C++ build can be skipped.
pub fn up_to_date(outputs: &[&Path], inputs: &[PathBuf]) -> bool {
    fn newest(p: &Path) -> std::time::SystemTime {
        let meta = std::fs::metadata(p).unwrap();
        if meta.is_dir() {
            std::fs::read_dir(p).unwrap().filter_map(|e| e.ok()).map(|e| newest(&e.path())).max().unwrap_or(std::time::UNIX_EPOCH)
        } else {
            meta.modified().unwrap()
        }
    }
    let outs = outputs.iter().map(|o| std::fs::metadata(o).and_then(|m| m.modified()).ok()).collect::<Option<Vec<_>>>();
    match outs {
        Some(outs) => inputs.iter().map(|i| newest(i)).max() < outs.into_iter().min(),
        None => false,
    }
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

/// Build `libest_rigid_state.so` and its replay reference (needs `$CXX` and
/// Eigen headers in `$EIGEN_INCLUDE`, default /usr/include/eigen3).
pub fn est_rigid_state() -> &'static (PathBuf, PathBuf) {
    static OUT: OnceLock<(PathBuf, PathBuf)> = OnceLock::new();
    OUT.get_or_init(|| {
        let root = workspace_root();
        let out = root.join("target/plugin-tests/cpp");
        std::fs::create_dir_all(&out).unwrap();
        let (lib, reference) = (out.join("libest_rigid_state.so"), out.join("eskf_reference"));
        let products = root.join("../..");
        let inputs = [
            root.join("scripts/build-est-rigid-state.sh"),
            root.join("plugins/est-rigid-state"),
            root.join("plugins/common"),
            root.join("abi/include"),
            products.join("ros1/perception/estimator/rigid-state/estimator_vrpn_px4_rotor_state/src"),
            products.join("ros1/perception/estimator/rigid-state/estimator_vrpn_px4_rotor_state/include"),
        ];
        if up_to_date(&[&lib, &reference], &inputs) {
            return (lib, reference);
        }
        let status = Command::new(root.join("scripts/build-est-rigid-state.sh"))
            .arg(&lib)
            .arg(&reference)
            .status()
            .expect("run build-est-rigid-state.sh (needs $CXX and Eigen headers in $EIGEN_INCLUDE)");
        assert!(status.success(), "building est-rigid-state failed");
        (lib, reference)
    })
}

// --- ROS helpers (tests that need ROS Noetic: set ROS_PREFIX) ---

pub fn ros_prefix() -> Option<PathBuf> {
    std::env::var_os("ROS_PREFIX").map(PathBuf::from).filter(|p| p.join("include/ros/ros.h").is_file())
}

pub fn ros_io_lib(prefix: &std::path::Path) -> &'static PathBuf {
    static LIB: OnceLock<PathBuf> = OnceLock::new();
    LIB.get_or_init(|| {
        let out = workspace_root().join("target/plugin-tests/ros");
        std::fs::create_dir_all(&out).unwrap();
        let lib = out.join("libros_io.so");
        let status = std::process::Command::new(workspace_root().join("scripts/build-ros-io.sh")).arg(&lib).env("ROS_PREFIX", prefix).status().unwrap();
        assert!(status.success(), "building ros_io failed");
        lib
    })
}

/// ctl-px4 built against the ROS-free PX4 controller core
/// (PX4_CORE_LIB_DIR and the other build-ctl-px4.sh variables), with the
/// toolchain that built the core (RoboStack's, when present).
pub fn ctl_px4_lib(prefix: &std::path::Path) -> &'static PathBuf {
    static LIB: OnceLock<PathBuf> = OnceLock::new();
    LIB.get_or_init(|| {
        let out = workspace_root().join("target/plugin-tests/cpp");
        std::fs::create_dir_all(&out).unwrap();
        let lib = out.join("libctl_px4.so");
        let mut c = Command::new(workspace_root().join("scripts/build-ctl-px4.sh"));
        c.arg(&lib);
        let conda_cxx = prefix.join("bin/x86_64-conda-linux-gnu-c++");
        if std::env::var_os("CXX").is_none() && conda_cxx.is_file() {
            c.env("CXX", conda_cxx);
        }
        assert!(c.status().unwrap().success(), "building ctl-px4 failed");
        lib
    })
}

/// ref-trajectory built against the ROS-free reference trajectory core
/// (REF_CORE_LIB_DIR and the other build-ref-trajectory.sh variables), with
/// the toolchain that built the core (RoboStack's, when present).
pub fn ref_trajectory_lib(prefix: &std::path::Path) -> &'static PathBuf {
    static LIB: OnceLock<PathBuf> = OnceLock::new();
    LIB.get_or_init(|| {
        let out = workspace_root().join("target/plugin-tests/cpp");
        std::fs::create_dir_all(&out).unwrap();
        let lib = out.join("libref_trajectory.so");
        let mut c = Command::new(workspace_root().join("scripts/build-ref-trajectory.sh"));
        c.arg(&lib);
        let conda_cxx = prefix.join("bin/x86_64-conda-linux-gnu-c++");
        if std::env::var_os("CXX").is_none() && conda_cxx.is_file() {
            c.env("CXX", conda_cxx);
        }
        assert!(c.status().unwrap().success(), "building ref-trajectory failed");
        lib
    })
}

/// A command run with the ROS environment (`source $ROS_PREFIX/setup.sh`).
pub fn ros_command(prefix: &std::path::Path, program: &str) -> std::process::Command {
    let mut c = std::process::Command::new("bash");
    let path = format!("{}:{}", prefix.join("bin").display(), std::env::var("PATH").unwrap_or_default());
    c.env("PATH", path).arg("-c").arg(format!("source '{}/setup.sh' && exec \"$0\" \"$@\"", prefix.display())).arg(program);
    c
}

/// A child started in its own process group. Dropping it signals the whole
/// group: roscore forks rosmaster and roslaunch forks its nodes, and killing
/// only the parent would leave them running (and rejoining the next test's
/// master on the same port).
pub struct Roscore(pub std::process::Child);

impl Roscore {
    pub fn spawn(mut command: std::process::Command) -> Self {
        use std::os::unix::process::CommandExt;
        command.process_group(0);
        Self(command.spawn().expect("spawn"))
    }
}

impl Drop for Roscore {
    fn drop(&mut self) {
        let pgid = self.0.id() as i32;
        extern "C" {
            fn kill(pid: i32, sig: i32) -> i32;
        }
        // SAFETY: signals only this child's own process group.
        unsafe { kill(-pgid, 2) }; // SIGINT: roslaunch stops its nodes, rosbag closes its bag
        // Wait for the whole group, not just the direct child: the rosbag
        // wrapper exits at once while its recorder is still writing the
        // index. kill(-pgid, 0) fails once no member is left.
        for _ in 0..100 {
            let _ = self.0.try_wait();
            if unsafe { kill(-pgid, 0) } != 0 {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
        unsafe { kill(-pgid, 9) };
        let _ = self.0.wait();
    }
}


// --- transport plugins (xgc_rt_transport_v1) ---

/// A transport plugin built by `plugin_dir`: "loopback" or "zenoh".
pub fn transport_plugin(kind: &str) -> PathBuf {
    plugin_dir().join(format!("libtransport_{kind}.so"))
}

/// The transport plugin `kind`, loaded the way a manifest's [transport]
/// path loads it, with `options` (TOML) as its table.
pub fn so_transport(kind: &str, options: &str) -> Box<dyn xgc_rt_core::transport::Transport> {
    let options: toml::Table = options.parse().unwrap();
    Box::new(xgc_rt_host::transport_so::SoTransport::load(&transport_plugin(kind), None, kind, &options).unwrap())
}

/// The loopback plugin's ground truth for `bus` (its xgc_rt_loopback_truth_json).
pub fn loopback_plugin_truth(bus: &str) -> std::collections::BTreeMap<(u32, u16, u16), xgc_rt_transport_loopback::Truth> {
    let lib = unsafe { libloading::Library::new(transport_plugin("loopback")) }.unwrap();
    let truth: libloading::Symbol<unsafe extern "C" fn(*const std::ffi::c_char, *mut u8, usize) -> isize> =
        unsafe { lib.get(b"xgc_rt_loopback_truth_json\0") }.unwrap();
    let name = std::ffi::CString::new(bus).unwrap();
    let len = unsafe { truth(name.as_ptr(), std::ptr::null_mut(), 0) };
    assert!(len >= 0, "no loopback bus {bus}");
    let mut buf = vec![0u8; len as usize];
    unsafe { truth(name.as_ptr(), buf.as_mut_ptr(), buf.len()) };
    let rows: Vec<[u64; 7]> = serde_json::from_slice(&buf).unwrap();
    rows.into_iter()
        .map(|r| {
            let t = xgc_rt_transport_loopback::Truth { offered: r[3], dropped: r[4], duplicated: r[5], reordered: r[6] };
            ((r[0] as u32, r[1] as u16, r[2] as u16), t)
        })
        .collect()
}

/// Release the samples the loopback plugin's `bus` holds (xgc_rt_loopback_flush).
pub fn loopback_plugin_flush(bus: &str) {
    let lib = unsafe { libloading::Library::new(transport_plugin("loopback")) }.unwrap();
    let flush: libloading::Symbol<unsafe extern "C" fn(*const std::ffi::c_char) -> i32> = unsafe { lib.get(b"xgc_rt_loopback_flush\0") }.unwrap();
    let name = std::ffi::CString::new(bus).unwrap();
    assert_eq!(unsafe { flush(name.as_ptr()) }, 0, "no loopback bus {bus}");
}

/// A localhost port for a Zenoh listener, free for both TCP and UDP when
/// picked. It is below the ephemeral range (32768-60999): a port the OS
/// hands out for `bind(0)` can be taken by an outgoing connection before
/// the listener binds it, which tests running in parallel did.
pub fn listen_port() -> u16 {
    static NEXT: std::sync::atomic::AtomicU16 = std::sync::atomic::AtomicU16::new(0);
    let base = 20_000 + (std::process::id() % 1_000) as u16 * 10;
    loop {
        let port = base + NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed) % 2_000;
        if std::net::TcpListener::bind(("127.0.0.1", port)).is_ok() && std::net::UdpSocket::bind(("127.0.0.1", port)).is_ok() {
            return port;
        }
    }
}
