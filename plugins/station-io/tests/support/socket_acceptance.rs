//! IPC boundary fixtures only. They do not assert flight or GCS workflow success.
use super::*;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{mpsc, Arc, Barrier};
use std::thread;

static NEXT_PATH: AtomicU64 = AtomicU64::new(0);

struct FixtureDir(PathBuf);
impl FixtureDir {
    fn new() -> Self {
        let path = std::env::temp_dir().join(format!(
            "station-w25-{}-{}", std::process::id(), NEXT_PATH.fetch_add(1, Ordering::Relaxed),
        ));
        std::fs::create_dir(&path).unwrap();
        Self(path)
    }
    fn socket(&self, index: usize) -> PathBuf { self.0.join(format!("{index}.sock")) }
}
impl Drop for FixtureDir {
    fn drop(&mut self) { let _ = std::fs::remove_dir_all(&self.0); }
}

fn send_frame(path: &Path, token: &str) -> UnixStream {
    let mut client = UnixStream::connect(path).unwrap();
    client.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    let mut frame = vec![KIND_COMMAND];
    frame.extend_from_slice(&(token.len() as u32).to_le_bytes());
    frame.extend_from_slice(token.as_bytes());
    client.write_all(&frame).unwrap();
    client
}

#[test]
fn occupied_socket_is_not_stolen() {
    let dir = FixtureDir::new();
    let path = dir.socket(0);
    let mut first = Listener::bind(&path).unwrap();
    assert!(Listener::bind(&path).is_err());
    // The liveness probe closes without sending a request.
    assert!(matches!(first.poll(), Ok(None)));
    let _client = send_frame(&path, "hold");
    assert!(matches!(first.poll(), Ok(Some(Request::Command(token))) if token == "hold"));
    first.reply(true, "queued").unwrap();
}

#[test]
fn stale_path_is_reclaimed_for_restart() {
    let dir = FixtureDir::new();
    let path = dir.socket(0);
    drop(UnixListener::bind(&path).unwrap());
    let mut restarted = Listener::bind(&path).unwrap();
    let _client = send_frame(&path, "hold");
    assert!(matches!(restarted.poll(), Ok(Some(Request::Command(token))) if token == "hold"));
    restarted.reply(true, "queued").unwrap();
}

#[test]
fn drop_preserves_replacement_and_non_socket_paths() {
    let dir = FixtureDir::new();
    let path = dir.socket(0);
    let first = Listener::bind(&path).unwrap();
    std::fs::rename(&path, dir.socket(1)).unwrap();
    let replacement = Listener::bind(&path).unwrap();
    drop(first);
    assert!(path.exists());
    drop(replacement);
    assert!(!path.exists());
    std::fs::write(&path, "keep").unwrap();
    assert!(Listener::bind(&path).is_err());
    assert_eq!(std::fs::read_to_string(&path).unwrap(), "keep");
}

#[test]
fn partial_request_expires_then_next_command_is_received() {
    let dir = FixtureDir::new();
    let path = dir.socket(0);
    let mut listener = Listener::bind(&path).unwrap();
    let mut stalled = UnixStream::connect(&path).unwrap();
    stalled.write_all(&[KIND_COMMAND, 5, 0]).unwrap();
    assert!(matches!(listener.poll(), Ok(None)));
    assert!(listener.accepted_at.is_some());
    // Drive the server clock boundary without sleeping for two seconds.
    listener.accepted_at = Some(Instant::now() - TRANSACTION_TIMEOUT);
    let mut next = send_frame(&path, "stop");
    assert!(listener.poll().is_err());
    assert!(matches!(listener.poll(), Ok(Some(Request::Command(token))) if token == "stop"));
    listener.reply(true, "queued").unwrap();
    let mut header = [0u8; 5];
    next.read_exact(&mut header).unwrap();
    assert_eq!(header[0], QUEUED);
    assert!(matches!(listener.poll(), Ok(None)));
}

#[test]
fn pending_reply_also_expires() {
    let dir = FixtureDir::new();
    let path = dir.socket(0);
    let mut listener = Listener::bind(&path).unwrap();
    let _client = send_frame(&path, "land");
    assert!(matches!(listener.poll(), Ok(Some(Request::Command(_)))));
    listener.pending = vec![QUEUED, 0, 0, 0, 0];
    listener.accepted_at = Some(Instant::now() - TRANSACTION_TIMEOUT);
    assert!(listener.poll().is_err());
    assert!(listener.current.is_none());
    assert!(listener.pending.is_empty());
}

#[test]
fn rejection_reason_is_truncated_at_utf8_boundary() {
    let dir = FixtureDir::new();
    let path = dir.socket(0);
    let mut listener = Listener::bind(&path).unwrap();
    let mut client = send_frame(&path, "start");
    assert!(matches!(listener.poll(), Ok(Some(Request::Command(_)))));
    listener.reply(false, &format!("{}拒绝", "x".repeat(199))).unwrap();
    let mut header = [0u8; 5];
    client.read_exact(&mut header).unwrap();
    assert_eq!(header[0], REJECTED);
    let len = u32::from_le_bytes(header[1..5].try_into().unwrap()) as usize;
    assert_eq!(len, 199);
    let mut body = vec![0; len];
    client.read_exact(&mut body).unwrap();
    assert!(std::str::from_utf8(&body).is_ok());
}

#[test]
fn eight_members_are_received_before_any_reply_with_individual_results() {
    let dir = FixtureDir::new();
    let mut listeners: Vec<_> = (0..8).map(|index| Listener::bind(&dir.socket(index)).unwrap()).collect();
    let barrier = Arc::new(Barrier::new(9));
    let clients: Vec<_> = (0..8).map(|index| {
        let barrier = Arc::clone(&barrier);
        let path = dir.socket(index);
        thread::spawn(move || {
            barrier.wait();
            let token = [b"start".as_slice(), b"stop", b"land"][index % 3];
            transact(&path, KIND_COMMAND, token)
        })
    }).collect();
    barrier.wait();
    let deadline = Instant::now() + TRANSACTION_TIMEOUT;
    let mut received = [false; 8];
    // No listener replies until all eight have a request. Serial dispatch fails.
    while !received.iter().all(|value| *value) && Instant::now() < deadline {
        for (index, listener) in listeners.iter_mut().enumerate() {
            if !received[index] {
                if let Some(Request::Command(token)) = listener.poll().unwrap() {
                    assert_eq!(token, ["start", "stop", "land"][index % 3]);
                    received[index] = true;
                }
            }
        }
        thread::yield_now();
    }
    assert!(received.iter().all(|value| *value), "not all requests arrived before a reply");
    for (index, listener) in listeners.iter_mut().enumerate() {
        match index {
            1 => listener.reply(false, "not the frozen authority").unwrap(),
            2 => listener.drop_stream(), // Received, but acknowledgement was lost.
            3 => {
                listener.current.as_mut().unwrap().write_all(&[9, 0, 0, 0, 0]).unwrap();
                listener.drop_stream();
            }
            _ => listener.reply(true, "queued").unwrap(),
        }
    }
    for (index, client) in clients.into_iter().enumerate() {
        let result = client.join().unwrap();
        match index {
            1 => assert_eq!(result.unwrap_err(), "rejected: not the frozen authority"),
            2 | 3 => assert!(result.unwrap_err().starts_with("outcome unknown:")),
            _ => assert!(result.is_ok()),
        }
        assert!(matches!(listeners[index].poll(), Ok(None)), "unexpected automatic retry for member {index}");
    }
}

#[test]
fn missing_ack_times_out_as_unknown_without_retry() {
    let dir = FixtureDir::new();
    let path = dir.socket(0);
    let listener = UnixListener::bind(&path).unwrap();
    let (release, wait) = mpsc::channel();
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        let mut request = [0u8; 9];
        stream.read_exact(&mut request).unwrap();
        assert_eq!(&request[5..], b"land");
        // Keep the connection open until the caller has actually timed out.
        wait.recv_timeout(Duration::from_secs(3)).unwrap();
        listener.set_nonblocking(true).unwrap();
        assert_eq!(listener.accept().unwrap_err().kind(), io::ErrorKind::WouldBlock);
    });
    let result = transact_with_timeout(&path, KIND_COMMAND, b"land", Duration::from_millis(100));
    release.send(()).unwrap();
    server.join().unwrap();
    let error = result.unwrap_err();
    assert!(error.starts_with("outcome unknown:"), "{error}");
    assert!(error.contains("not retried"));
}

#[test]
fn partial_reply_does_not_reset_the_absolute_deadline() {
    let (mut reader, mut writer) = UnixStream::pair().unwrap();
    writer.write_all(&[QUEUED, 0]).unwrap();
    let mut header = [0u8; 5];
    let result = read_before(&mut reader, &mut header, Instant::now() + Duration::from_millis(50));
    assert!(result.is_err());
    assert_eq!(&header[..2], &[QUEUED, 0]);
    // Expiry is checked even when more bytes have subsequently become readable.
    writer.write_all(&[0, 0, 0]).unwrap();
    assert_eq!(read_before(&mut reader, &mut header[2..], Instant::now() - Duration::from_millis(1))
        .unwrap_err().kind(), io::ErrorKind::TimedOut);
}

#[test]
fn full_listen_backlog_does_not_block_connect() {
    let dir = FixtureDir::new();
    let listener = UnixListener::bind(dir.socket(0)).unwrap();
    // Linux accepts one queued connection at backlog zero. A further
    // nonblocking connect must fail, not wait forever before the I/O timeout.
    assert_eq!(unsafe { libc::listen(listener.as_raw_fd(), 0) }, 0);
    let _first = connect_once(&dir.socket(0), Instant::now() + TRANSACTION_TIMEOUT).unwrap();
    assert!(connect_once(&dir.socket(0), Instant::now() + Duration::from_millis(100)).is_err());
}

#[test]
fn invalid_or_missing_path_is_a_connect_error_not_a_rejection() {
    let dir = FixtureDir::new();
    let error = transact(&dir.socket(0), KIND_COMMAND, b"start").unwrap_err();
    assert!(error.starts_with("connect:"), "{error}");
    assert!(connect_once(Path::new(""), Instant::now() + TRANSACTION_TIMEOUT).is_err());
    assert!(connect_once(Path::new("bad\0path"), Instant::now() + TRANSACTION_TIMEOUT).is_err());
}
