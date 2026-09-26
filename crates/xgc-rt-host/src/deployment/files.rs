//! Linux descriptor-relative publication. No symlink-following mkdir/write
//! sequence, and no generation overwrite. The advisory lock is inherited by
//! the exec'd host so a second deployment cannot rewrite a live identity.
use super::*;
use serde::Serialize;
use std::ffi::{CString, OsStr};
use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::ffi::OsStrExt;

fn os_error(context: &str) -> String {
    format!("{context}: {}", std::io::Error::last_os_error())
}
fn cstr(value: &OsStr) -> Result<CString> {
    CString::new(value.as_bytes()).map_err(|_| "NUL in path".into())
}
fn checked_fd(fd: i32, context: &str) -> Result<File> {
    if fd < 0 {
        Err(os_error(context))
    } else {
        // SAFETY: successful open/openat returns one newly owned descriptor.
        Ok(unsafe { File::from_raw_fd(fd) })
    }
}
struct Dir(File);
impl Dir {
    fn root(path: &Path, create: bool) -> Result<Self> {
        require(absolute(path), "directory path must be clean and absolute")?;
        // SAFETY: constant NUL-terminated string and valid open flags.
        let mut dir = Self(checked_fd(
            unsafe {
                libc::open(
                    b"/\0".as_ptr().cast(),
                    libc::O_RDONLY | libc::O_DIRECTORY | libc::O_CLOEXEC,
                )
            },
            "open filesystem root",
        )?);
        for component in path.components() {
            match component {
                Component::RootDir => (),
                Component::Normal(name) => dir = dir.child_os(name, create)?,
                _ => {
                    return Err(
                        "directory path must be clean, without dot/parent components".into(),
                    )
                }
            }
        }
        Ok(dir)
    }
    fn child_os(&self, name: &OsStr, create: bool) -> Result<Self> {
        let name = cstr(name)?;
        if create {
            // SAFETY: descriptor and NUL-terminated single path component.
            let result = unsafe { libc::mkdirat(self.0.as_raw_fd(), name.as_ptr(), 0o700) };
            if result != 0 && std::io::Error::last_os_error().raw_os_error() != Some(libc::EEXIST) {
                return Err(os_error("create private directory"));
            }
        }
        // O_NOFOLLOW applies at each component, not only at the final file.
        let fd = unsafe {
            libc::openat(
                self.0.as_raw_fd(),
                name.as_ptr(),
                libc::O_RDONLY | libc::O_DIRECTORY | libc::O_NOFOLLOW | libc::O_CLOEXEC,
            )
        };
        Ok(Self(checked_fd(fd, "open directory without symlinks")?))
    }
    fn child_new(&self, name: &str) -> Result<Self> {
        let value = cstr(OsStr::new(name))?;
        // A generation staging directory must be newly created, never reused.
        if unsafe { libc::mkdirat(self.0.as_raw_fd(), value.as_ptr(), 0o700) } != 0 {
            return Err(os_error("create exclusive directory"));
        }
        self.child(name, false)
    }
    fn child(&self, name: &str, create: bool) -> Result<Self> {
        self.child_os(OsStr::new(name), create)
    }
    fn parent(&self, relative_name: &str) -> Result<(Self, CString)> {
        require(
            relative(relative_name),
            "bundle path must be relative without traversal",
        )?;
        let path = Path::new(relative_name);
        let mut dir = Self(self.0.try_clone().map_err(|e| e.to_string())?);
        if let Some(parent) = path.parent() {
            for component in parent.components() {
                match component {
                    Component::Normal(n) => dir = dir.child_os(n, false)?,
                    _ => return Err("invalid relative component".into()),
                }
            }
        }
        Ok((dir, cstr(path.file_name().ok_or("missing filename")?)?))
    }
    fn open_file(&self, path: &str, flags: i32) -> Result<File> {
        let (parent, name) = self.parent(path)?;
        let fd = unsafe {
            libc::openat(
                parent.0.as_raw_fd(),
                name.as_ptr(),
                flags | libc::O_NOFOLLOW | libc::O_NONBLOCK,
                0o600,
            )
        };
        let file = checked_fd(fd, "open regular file without symlinks")?;
        require(
            file.metadata().map_err(|e| e.to_string())?.is_file(),
            "expected a regular file",
        )?;
        Ok(file)
    }
    fn read(&self, path: &str, maximum: usize) -> Result<Vec<u8>> {
        let file = self.open_file(path, libc::O_RDONLY | libc::O_CLOEXEC)?;
        let mut data = Vec::new();
        file.take(maximum as u64 + 1)
            .read_to_end(&mut data)
            .map_err(|e| e.to_string())?;
        require(data.len() <= maximum, "file exceeds size limit")?;
        Ok(data)
    }
    fn exists(&self, path: &str) -> Result<bool> {
        let name = cstr(OsStr::new(path))?;
        let mut stat = std::mem::MaybeUninit::<libc::stat>::uninit();
        let result = unsafe {
            libc::fstatat(
                self.0.as_raw_fd(),
                name.as_ptr(),
                stat.as_mut_ptr(),
                libc::AT_SYMLINK_NOFOLLOW,
            )
        };
        if result == 0 {
            return Ok(true);
        }
        if std::io::Error::last_os_error().raw_os_error() == Some(libc::ENOENT) {
            Ok(false)
        } else {
            Err(os_error("inspect file"))
        }
    }
    fn write_new(&self, path: &str, bytes: &[u8]) -> Result<()> {
        let mut file = self.open_file(
            path,
            libc::O_WRONLY | libc::O_CREAT | libc::O_EXCL | libc::O_CLOEXEC,
        )?;
        file.write_all(bytes)
            .and_then(|_| file.sync_all())
            .map_err(|e| e.to_string())
    }
    fn publish(&self, from: &str, to: &str) -> Result<()> {
        let from = cstr(OsStr::new(from))?;
        let to = cstr(OsStr::new(to))?;
        // Linux renameat2 gives no-replace atomic publication, including a
        // racing external destination. Both names are single owned entries.
        let result = unsafe {
            libc::syscall(
                libc::SYS_renameat2,
                self.0.as_raw_fd(),
                from.as_ptr(),
                self.0.as_raw_fd(),
                to.as_ptr(),
                libc::RENAME_NOREPLACE,
            )
        };
        if result != 0 {
            return Err(os_error("atomic no-replace publication"));
        }
        self.0.sync_all().map_err(|e| e.to_string())
    }
    fn check_link(&self, link: &BundleLink) -> Result<()> {
        let (parent, name) = self.parent(&link.path)?;
        let mut bytes = [0u8; 4096];
        let size = unsafe {
            libc::readlinkat(
                parent.0.as_raw_fd(),
                name.as_ptr(),
                bytes.as_mut_ptr().cast(),
                bytes.len(),
            )
        };
        if size < 0 {
            return Err(os_error("read declared bundle link"));
        }
        require(
            &bytes[..size as usize] == link.target.as_bytes(),
            "bundle symlink differs from its declared target",
        )
    }
}

fn nonce() -> Result<String> {
    let mut bytes = [0u8; 16];
    File::open("/dev/urandom")
        .and_then(|mut f| f.read_exact(&mut bytes))
        .map_err(|e| e.to_string())?;
    Ok(bytes.iter().map(|b| format!("{b:02x}")).collect())
}

fn verify_elf(dir: &Dir, pin: &BundleFile) -> Result<File> {
    require(digest(&pin.sha256), "noncanonical artifact SHA256")?;
    let mut file = dir.open_file(&pin.path, libc::O_RDONLY | libc::O_CLOEXEC)?;
    let mut header = [0u8; 20];
    file.read_exact(&mut header)
        .map_err(|_| format!("short ELF header: {}", pin.path))?;
    require(
        &header[..4] == b"\x7fELF"
            && header[4] == 2
            && header[5] == 1
            && u16::from_le_bytes([header[18], header[19]]) == 62,
        "artifact is not little-endian ELF64 amd64",
    )?;
    file.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
    let mut hash = Sha256::new();
    let mut buffer = [0u8; 64 * 1024];
    loop {
        let n = file.read(&mut buffer).map_err(|e| e.to_string())?;
        if n == 0 {
            break;
        }
        hash.update(&buffer[..n]);
    }
    require(
        format!("{:x}", hash.finalize()) == pin.sha256,
        &format!("artifact hash mismatch: {}", pin.path),
    )?;
    Ok(file)
}

fn verify_bundle(root: &Path, deployment: &Deployment) -> Result<(Bundle, File)> {
    require(
        cfg!(target_os = "linux") && cfg!(target_arch = "x86_64"),
        "this renderer release supports Linux amd64 only",
    )?;
    let dir = Dir::root(root, false)?;
    let bytes = dir.read(BUNDLE_FILE, 4 * 1024 * 1024)?;
    require(
        sha256(&bytes) == deployment.bundle_sha256,
        "bundle descriptor bytes/digest mismatch",
    )?;
    let bundle: Bundle =
        serde_json::from_slice(&bytes).map_err(|e| format!("bundle schema: {e}"))?;
    require(
        bundle.schema_version == 1
            && bundle.platform == deployment.platform
            && bundle.composition_sha256 == composition(&deployment.composition_id)?.sha256(),
        "bundle release mismatch",
    )?;
    require(
        bundle.host.path == "bin/xgc-rt-host",
        "bundle host path must be bin/xgc-rt-host",
    )?;
    require(
        bundle.libraries.len() <= 256 && bundle.links.len() <= 256,
        "bundle closure is too large",
    )?;
    bundle.plugins.validate_roles(&deployment.composition_id)?;
    let mut paths = BTreeSet::new();
    let mut host = None;
    for file in std::iter::once(&bundle.host)
        .chain(bundle.plugins.entries().iter().map(|(_, f)| *f))
        .chain(bundle.libraries.iter())
    {
        require(
            relative(&file.path) && paths.insert(file.path.clone()),
            "duplicate or unsafe bundle path",
        )?;
        let verified = verify_elf(&dir, file)?;
        if file.path == bundle.host.path {
            host = Some(verified);
        }
    }
    for (_, file) in bundle.plugins.entries() {
        require(
            file.path.starts_with("plugins/"),
            "plugin must be inside plugins/",
        )?;
    }
    for file in &bundle.libraries {
        require(file.path.starts_with("lib/"), "library must be inside lib/")?;
    }
    let regular_paths = paths.clone();
    for link in &bundle.links {
        require(
            relative(&link.path)
                && link.path.starts_with("lib/")
                && relative(&link.target)
                && !link.target.contains('/'),
            "bundle link must remain in its library directory",
        )?;
        let target = Path::new(&link.path)
            .parent()
            .ok_or("missing link parent")?
            .join(&link.target);
        require(
            regular_paths.contains(target.to_str().ok_or("invalid link target")?)
                && paths.insert(link.path.clone()),
            "bundle link must resolve directly to a distinct indexed file",
        )?;
        dir.check_link(link)?;
    }
    let host = host.ok_or("missing host artifact")?;
    Ok((bundle, host))
}

#[derive(Debug, Serialize)]
pub struct Receipt {
    pub schema_version: u32,
    pub session_id: String,
    pub node_id: String,
    pub robot_namespace: String,
    pub platform: String,
    pub deployment_sha256: String,
    pub bundle_sha256: String,
    pub composition_sha256: String,
    pub configuration_sha256: String,
    pub manifest_sha256: String,
    pub generation: String,
    pub manifest_path: PathBuf,
    pub audit_path: PathBuf,
    pub input_time_domain: &'static str,
    pub role: &'static str,
    pub actuator_namespace: Option<String>,
    pub live_readiness: bool,
}

pub struct Prepared {
    pub receipt: Receipt,
    pub host_path: PathBuf,
    pub bundle_root: PathBuf,
    pub configuration: RunConfiguration,
    /// CLOEXEC until this prepared instance deliberately launches its host.
    _lock: File,
    host: File,
}

impl Prepared {
    /// Consume the prepared generation and preserve only its two descriptors
    /// across exec. Ordinary concurrent subprocesses must not inherit locks.
    pub fn exec_host(self) -> Result<()> {
        use std::os::unix::process::CommandExt;
        use std::process::Command;
        let fds = [self._lock.as_raw_fd(), self.host.as_raw_fd()];
        let mut command = Command::new(self.pinned_host_path());
        command
            .args([
                "--manifest",
                self.receipt
                    .manifest_path
                    .to_str()
                    .ok_or("non-UTF8 manifest path")?,
            ])
            .env_remove("ROS_HOSTNAME")
            .env("ROS_MASTER_URI", self.configuration.ros_master_uri())
            .env("ROS_IP", self.configuration.ros_ip())
            .env(
                "ROS_NAMESPACE",
                format!("/{}", self.receipt.robot_namespace),
            )
            .env(
                "LD_LIBRARY_PATH",
                format!("{}/lib:/opt/ros/noetic/lib", self.bundle_root.display()),
            );
        // SAFETY: pre_exec uses only async-signal-safe fcntl, with owned live
        // descriptors. No allocation, locking, or environment access here.
        unsafe {
            command.pre_exec(move || {
                for fd in fds {
                    if libc::fcntl(fd, libc::F_SETFD, 0) != 0 {
                        return Err(std::io::Error::last_os_error());
                    }
                }
                Ok(())
            });
        }
        let error = command.exec();
        Err(format!("exec verified host: {error}"))
    }

    pub fn pinned_host_path(&self) -> PathBuf {
        PathBuf::from(format!("/proc/self/fd/{}", self.host.as_raw_fd()))
    }
}

pub fn prepare(raw: &str, bundle_root: &Path, state_root: &Path) -> Result<Prepared> {
    let (deployment, configuration) = Deployment::parse(raw)?;
    let (bundle, host) = verify_bundle(bundle_root, &deployment)?;
    let root = Dir::root(state_root, true)?;
    let session = root.child(&deployment.session_id, true)?;
    let identity = session.child(&deployment.node_id, true)?;
    let lock = identity.open_file(".lock", libc::O_RDWR | libc::O_CREAT | libc::O_CLOEXEC)?;
    let locked = unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) };
    if locked != 0 {
        return Err(
            "deployment identity is active/locked; cannot start a second generation".into(),
        );
    }
    let input = deployment.identity_bytes()?;
    if identity.exists("deployment.json")? {
        require(
            identity.read("deployment.json", MAX_INPUT * 2)? == input,
            "same Session/node has different frozen configuration; refusing replacement",
        )?;
    } else {
        let temp = format!(".deployment-{}", nonce()?);
        identity.write_new(&temp, &input)?;
        identity.publish(&temp, "deployment.json")?;
    }
    let generations = identity.child("generations", true)?;
    let generation = format!("generation-{}", nonce()?);
    let staged_name = format!(".{generation}");
    require(
        !generations.exists(&staged_name)? && !generations.exists(&generation)?,
        "generation collision",
    )?;
    let staged = generations.child_new(&staged_name)?;
    let audit_dir = staged.child("audit", true)?;
    audit_dir.0.sync_all().map_err(|e| e.to_string())?;
    let final_path = state_root
        .join(&deployment.session_id)
        .join(&deployment.node_id)
        .join("generations")
        .join(&generation);
    let audit_path = final_path.join("audit");
    let manifest = render(&deployment, &bundle, bundle_root, &audit_path)?;
    let receipt = Receipt {
        schema_version: 1,
        session_id: deployment.session_id.clone(),
        node_id: deployment.node_id.clone(),
        robot_namespace: deployment.robot_namespace.clone(),
        platform: deployment.platform.clone(),
        deployment_sha256: sha256(&input),
        bundle_sha256: deployment.bundle_sha256.clone(),
        composition_sha256: deployment.composition_sha256.clone(),
        configuration_sha256: deployment.configuration_sha256.clone(),
        manifest_sha256: sha256(manifest.as_bytes()),
        generation,
        manifest_path: final_path.join("node.toml"),
        audit_path,
        input_time_domain: match configuration.time_domain() {
            TimeDomain::WallUnix => "wall-unix", TimeDomain::Ros1Sim => "ros1-sim",
        },
        role: configuration.role(),
        actuator_namespace: configuration.actuator_namespace(&deployment.robot_namespace).map(str::to_string),
        live_readiness: false,
    };
    staged.write_new("node.toml", manifest.as_bytes())?;
    staged.write_new("deployment.json", &input)?;
    staged.write_new(
        "receipt.json",
        &serde_json::to_vec_pretty(&receipt).map_err(|e| e.to_string())?,
    )?;
    staged.0.sync_all().map_err(|e| e.to_string())?;
    generations.publish(&staged_name, &receipt.generation)?;
    Ok(Prepared {
        receipt,
        host_path: bundle_root.join(&bundle.host.path),
        bundle_root: bundle_root.to_owned(),
        configuration,
        _lock: lock,
        host,
    })
}
