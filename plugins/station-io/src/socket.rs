//! Local stream socket. One request, one reply. Not a bus and not telemetry.

use std::io::{self, Read, Write};
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::ffi::OsStrExt;
use std::os::unix::fs::{FileTypeExt, MetadataExt};
use std::os::unix::net::{UnixListener, UnixStream};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use crate::wire::{self, COMMAND_LEN, TIMELINE_LEN};

pub const KIND_COMMAND: u8 = 1;
pub const KIND_MISSION: u8 = 2;
const QUEUED: u8 = 0;
const REJECTED: u8 = 1;
const TRANSACTION_TIMEOUT: Duration = Duration::from_secs(2);

pub struct Listener {
    path: PathBuf,
    identity: (u64, u64),
    accepted_at: Option<Instant>,
    listener: UnixListener,
    current: Option<UnixStream>,
    buf: Vec<u8>,
    pending: Vec<u8>,
}

pub enum Request {
    Command(String),
    Mission([u8; TIMELINE_LEN]),
}

impl Listener {
    pub fn bind(path: &Path) -> Result<Self, String> {
        if path.as_os_str().len() > 100 {
            return Err("command_socket path is too long".into());
        }
        // Let bind fail on an occupied path. Unlinking here could steal a live
        // authority's command endpoint. Stale paths need explicit owner cleanup.
        let listener = UnixListener::bind(path).map_err(|e| format!("command_socket: {e}"))?;
        listener.set_nonblocking(true).map_err(|e| e.to_string())?;
        let meta = std::fs::symlink_metadata(path).map_err(|e| e.to_string())?;
        Ok(Self {
            path: path.to_path_buf(), identity: (meta.dev(), meta.ino()),
            accepted_at: None, listener, current: None, buf: Vec::new(), pending: Vec::new(),
        })
    }

    fn drop_stream(&mut self) {
        self.current = None;
        self.accepted_at = None;
        self.buf.clear();
        self.pending.clear();
    }

    /// Finish a reply without blocking the step. False means bytes remain.
    fn flush_pending(&mut self) -> Result<bool, String> {
        if self.pending.is_empty() {
            return Ok(true);
        }
        let Some(stream) = self.current.as_mut() else {
            self.pending.clear();
            return Ok(true);
        };
        stream.set_nonblocking(true).map_err(|e| e.to_string())?;
        while !self.pending.is_empty() {
            match stream.write(&self.pending) {
                Ok(0) => {
                    self.drop_stream();
                    return Err("reply peer closed".into());
                }
                Ok(n) => {
                    self.pending.drain(..n);
                }
                Err(e) if e.kind() == std::io::ErrorKind::Interrupted => continue,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(false),
                Err(e) => {
                    self.drop_stream();
                    return Err(e.to_string());
                }
            }
        }
        self.drop_stream();
        Ok(true)
    }

    pub fn poll(&mut self) -> Result<Option<Request>, String> {
        // A partial request or unread reply must not monopolize this authority.
        // This is one absolute deadline, not reset by trickling more bytes.
        if self.accepted_at.is_some_and(|at| at.elapsed() >= TRANSACTION_TIMEOUT) {
            self.drop_stream();
            return Err("command_socket request/reply deadline exceeded".into());
        }
        if !self.flush_pending()? {
            return Ok(None);
        }
        if self.current.is_none() {
            // Even a nonblocking accept allocates an fd before checking the queue.
            // At a file-table boundary that can wait for RCU despite no connection.
            // Readiness with a zero timeout avoids that work on every idle step.
            let mut readiness = libc::pollfd {
                fd: self.listener.as_raw_fd(), events: libc::POLLIN, revents: 0,
            };
            // The listener owns this valid fd for the entire call.
            let ready = unsafe { libc::poll(&mut readiness, 1, 0) };
            if ready < 0 {
                let error = std::io::Error::last_os_error();
                if error.kind() == std::io::ErrorKind::Interrupted { return Ok(None); }
                return Err(error.to_string());
            }
            if ready == 0 { return Ok(None); }
            match self.listener.accept() {
                Ok((stream, _)) => {
                    stream.set_nonblocking(true).map_err(|e| e.to_string())?;
                    self.current = Some(stream);
                    self.accepted_at = Some(Instant::now());
                    self.buf.clear();
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(None),
                Err(e) => return Err(e.to_string()),
            }
        }
        let stream = self.current.as_mut().unwrap();
        let mut tmp = [0u8; 256];
        loop {
            match stream.read(&mut tmp) {
                Ok(0) => {
                    self.drop_stream();
                    return Ok(None);
                }
                Ok(n) => self.buf.extend_from_slice(&tmp[..n]),
                Err(e) if e.kind() == io::ErrorKind::Interrupted => continue,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(None),
                Err(e) => {
                    self.drop_stream();
                    return Err(e.to_string());
                }
            }
            if self.buf.len() >= 5 {
                let len = u32::from_le_bytes(self.buf[1..5].try_into().unwrap()) as usize;
                if len > TIMELINE_LEN {
                    self.reply(false, "payload is longer than 240 bytes")?;
                    return Ok(None);
                }
                if self.buf.len() >= 5 + len {
                    break;
                }
            }
        }
        let kind = self.buf[0];
        let len = u32::from_le_bytes(self.buf[1..5].try_into().unwrap()) as usize;
        let payload = self.buf[5..5 + len].to_vec();
        self.buf.drain(..5 + len);
        match kind {
            KIND_COMMAND => {
                let text = String::from_utf8(payload).map_err(|_| "command is not utf-8".to_string());
                match text {
                    Ok(text) if wire::valid_command_token(&text) => Ok(Some(Request::Command(text))),
                    Ok(_) => {
                        self.reply(false, "command token is not a controller string")?;
                        Ok(None)
                    }
                    Err(e) => {
                        self.reply(false, &e)?;
                        Ok(None)
                    }
                }
            }
            KIND_MISSION => {
                if payload.len() != TIMELINE_LEN || !wire::timeline_schema_ok(&payload) {
                    self.reply(false, "mission_request must be 240 bytes with schema 1")?;
                    return Ok(None);
                }
                let mut bytes = [0u8; TIMELINE_LEN];
                bytes.copy_from_slice(&payload);
                Ok(Some(Request::Mission(bytes)))
            }
            _ => {
                self.reply(false, "unknown request kind")?;
                Ok(None)
            }
        }
    }

    pub fn reply(&mut self, queued: bool, reason: &str) -> Result<(), String> {
        if self.current.is_none() {
            return Ok(());
        }
        let mut end = reason.len().min(200);
        while !reason.is_char_boundary(end) { end -= 1; }
        let reason = &reason.as_bytes()[..end];
        self.pending.clear();
        self.pending.push(if queued { QUEUED } else { REJECTED });
        self.pending.extend_from_slice(&(reason.len() as u32).to_le_bytes());
        self.pending.extend_from_slice(reason);
        self.flush_pending().map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn idle_listener_needs_no_spare_file_descriptor() {
        const CHILD: &str = "XGC_STATION_IDLE_FD_TEST";
        if std::env::var_os(CHILD).is_none() {
            let status = std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact", "socket::tests::idle_listener_needs_no_spare_file_descriptor"])
                .env(CHILD, "1").status().unwrap();
            assert!(status.success());
            return;
        }
        let path = std::env::temp_dir().join(format!("xgc-station-idle-{}.sock", std::process::id()));
        let mut listener = Listener::bind(&path).unwrap();
        let mut previous = libc::rlimit { rlim_cur: 0, rlim_max: 0 };
        // This test runs in a subprocess so its fd limit cannot affect other tests.
        assert_eq!(unsafe { libc::getrlimit(libc::RLIMIT_NOFILE, &mut previous) }, 0);
        // poll's one-element array must itself fit the soft limit.
        let limited = libc::rlimit { rlim_cur: 1, ..previous };
        assert_eq!(unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &limited) }, 0);
        let result = listener.poll();
        assert_eq!(unsafe { libc::setrlimit(libc::RLIMIT_NOFILE, &previous) }, 0);
        assert!(matches!(result, Ok(None)), "idle poll attempted to allocate an fd");

        let mut client = UnixStream::connect(&path).unwrap();
        client.write_all(&[KIND_COMMAND, 7, 0, 0, 0]).unwrap();
        client.write_all(b"prepare").unwrap();
        assert!(matches!(listener.poll(), Ok(Some(Request::Command(token))) if token == "prepare"));
        listener.reply(true, "queued").unwrap();
        let mut reply = [0; 5];
        client.read_exact(&mut reply).unwrap();
        assert_eq!(reply[0], QUEUED);
    }
}

impl Drop for Listener {
    fn drop(&mut self) {
        // Do not unlink a replacement installed at the same pathname.
        if let Ok(meta) = std::fs::symlink_metadata(&self.path) {
            if meta.file_type().is_socket() && (meta.dev(), meta.ino()) == self.identity {
                let _ = std::fs::remove_file(&self.path);
            }
        }
    }
}

fn remaining(deadline: Instant) -> io::Result<Duration> {
    deadline.checked_duration_since(Instant::now())
        .filter(|duration| !duration.is_zero())
        .ok_or_else(|| io::Error::new(io::ErrorKind::TimedOut, "transaction deadline exceeded"))
}

fn connect_once(path: &Path, deadline: Instant) -> io::Result<UnixStream> {
    // Nonblocking connect also bounds a full local listen backlog. No request
    // has been sent yet; EAGAIN is returned as a connect failure, never retried.
    let bytes = path.as_os_str().as_bytes();
    // sockaddr_un is a plain C structure; zeroing also terminates sun_path.
    let mut address: libc::sockaddr_un = unsafe { std::mem::zeroed() };
    if bytes.is_empty() || bytes.len() >= address.sun_path.len() || bytes.contains(&0) {
        return Err(io::Error::new(io::ErrorKind::InvalidInput, "invalid command_socket path"));
    }
    address.sun_family = libc::AF_UNIX as _;
    for (target, byte) in address.sun_path.iter_mut().zip(bytes) {
        *target = *byte as libc::c_char;
    }
    remaining(deadline)?;
    // On supported Linux targets these flags set both properties atomically.
    let fd = unsafe { libc::socket(libc::AF_UNIX, libc::SOCK_STREAM | libc::SOCK_NONBLOCK | libc::SOCK_CLOEXEC, 0) };
    if fd < 0 { return Err(io::Error::last_os_error()); }
    // The new descriptor is exclusively owned by this stream, including errors.
    let stream = unsafe { UnixStream::from_raw_fd(fd) };
    let connected = unsafe {
        libc::connect(fd, (&address as *const libc::sockaddr_un).cast(),
                      std::mem::size_of_val(&address) as libc::socklen_t)
    };
    if connected < 0 {
        let error = io::Error::last_os_error();
        if error.raw_os_error() != Some(libc::EINPROGRESS) { return Err(error); }
        loop {
            let timeout = remaining(deadline)?.as_millis().clamp(1, i32::MAX as u128) as i32;
            let mut ready = libc::pollfd { fd, events: libc::POLLOUT, revents: 0 };
            // The stream owns fd and ready is valid for this call only.
            let result = unsafe { libc::poll(&mut ready, 1, timeout) };
            if result < 0 {
                let error = io::Error::last_os_error();
                if error.kind() == io::ErrorKind::Interrupted { continue; }
                return Err(error);
            }
            if result == 0 { continue; }
            if let Some(error) = stream.take_error()? { return Err(error); }
            stream.peer_addr()?;
            break;
        }
    }
    stream.set_nonblocking(false)?;
    Ok(stream)
}

fn write_before(stream: &mut UnixStream, mut bytes: &[u8], deadline: Instant) -> io::Result<()> {
    while !bytes.is_empty() {
        stream.set_write_timeout(Some(remaining(deadline)?))?;
        match stream.write(bytes) {
            Ok(0) => return Err(io::Error::new(io::ErrorKind::WriteZero, "request peer closed")),
            Ok(count) => bytes = &bytes[count..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

fn read_before(stream: &mut UnixStream, mut bytes: &mut [u8], deadline: Instant) -> io::Result<()> {
    while !bytes.is_empty() {
        stream.set_read_timeout(Some(remaining(deadline)?))?;
        match stream.read(bytes) {
            Ok(0) => return Err(io::Error::new(io::ErrorKind::UnexpectedEof, "reply peer closed")),
            Ok(count) => bytes = &mut bytes[count..],
            Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
            Err(error) => return Err(error),
        }
    }
    Ok(())
}

fn unknown(reason: impl std::fmt::Display) -> String {
    format!("outcome unknown: {reason}; not retried; inspect controller telemetry before another command")
}

pub fn transact(path: &Path, kind: u8, payload: &[u8]) -> Result<(), String> {
    transact_with_timeout(path, kind, payload, TRANSACTION_TIMEOUT)
}

fn transact_with_timeout(path: &Path, kind: u8, payload: &[u8], timeout: Duration) -> Result<(), String> {
    if payload.len() > TIMELINE_LEN || (kind == KIND_COMMAND && payload.len() >= COMMAND_LEN) {
        return Err("payload does not fit the typed frame".into());
    }
    let deadline = Instant::now() + timeout;
    let mut stream = connect_once(path, deadline).map_err(|e| format!("connect: {e}"))?;
    let mut frame = Vec::with_capacity(5 + payload.len());
    frame.push(kind);
    frame.extend_from_slice(&(payload.len() as u32).to_le_bytes());
    frame.extend_from_slice(payload);
    // Even a failed write may have delivered a request. Never infer rejection
    // from transport failure or repeat a flight command after a missing reply.
    write_before(&mut stream, &frame, deadline).map_err(unknown)?;
    let mut header = [0u8; 5];
    read_before(&mut stream, &mut header, deadline).map_err(unknown)?;
    let len = u32::from_le_bytes(header[1..5].try_into().unwrap()) as usize;
    if len > 200 { return Err(unknown("reply is too long")); }
    let mut body = vec![0u8; len];
    read_before(&mut stream, &mut body, deadline).map_err(unknown)?;
    let text = std::str::from_utf8(&body).map_err(unknown)?;
    match header[0] {
        QUEUED => {
            // Keep the existing CLI contract. Queued means local publication,
            // not controller consumption, arming, takeoff, or mission completion.
            println!("queued");
            Ok(())
        }
        REJECTED => Err(format!("rejected: {text}")),
        status => Err(unknown(format!("invalid reply status {status}"))),
    }
}

#[cfg(test)]
#[path = "../tests/support/socket_acceptance.rs"]
mod acceptance_tests;
