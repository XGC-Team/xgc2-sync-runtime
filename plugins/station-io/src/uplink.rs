//! One Zenoh client for the six GCS keys. No radio listen, no subscription.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use zenoh::pubsub::Publisher;
use zenoh::qos::{CongestionControl, Priority, Reliability};
use zenoh::Wait;

use crate::wire::{
    self, HeartbeatChannel, NativeBattery, NativeFlight, NativeImu, NativePaired, NativePose,
    NativeTwist, Rates,
};

const LEAVES: [&str; 5] = [
    "local_pose",
    "local_velocity",
    "imu",
    "power",
    "flight_state",
];

const FRESH_MS: [i64; 6] = [1000, 1000, 1000, 3000, 3000, 3000];

struct Latest {
    pose: Option<NativePose>,
    twist: Option<NativeTwist>,
    paired: Option<NativePaired>,
    imu: Option<NativeImu>,
    battery: Option<NativeBattery>,
    flight: Option<NativeFlight>,
    controller: Option<String>,
    seen: [Option<Instant>; 6],
    count: [u64; 6],
    session_ms: i64,
    rejected: u64,
    // Native put().wait() results, not transport delivery or receiver receipt.
    put_accepted: u64,
    put_failed: u64,
    // Source-channel generations superseded before selection for encoding.
    // Paired input updates two channels; controller is selected by heartbeat.
    throttled: u64,
    pending: [bool; 6],
    unencodable: [bool; 5],
    next_at: [Instant; 6],
    waiting: bool,
    wait_until: Option<Instant>,
    #[cfg(test)]
    wake_notifications: u64,
    #[cfg(test)]
    snapshots: u64,
}

// Copy only due payloads out of the mutex; unrelated source updates must not
// clone the flight/controller strings.
struct Snapshot {
    pose: Option<NativePose>,
    twist: Option<NativeTwist>,
    paired: Option<NativePaired>,
    imu: Option<NativeImu>,
    battery: Option<NativeBattery>,
    flight: Option<NativeFlight>,
    controller: Option<String>,
    seen: [Option<Instant>; 6],
    count: [u64; 6],
    session_ms: i64,
    rejected: u64,
    put_accepted: u64,
    put_failed: u64,
    throttled: u64,
}

impl Latest {
    fn new(now: Instant) -> Self {
        Self {
            pose: None,
            twist: None,
            paired: None,
            imu: None,
            battery: None,
            flight: None,
            controller: None,
            seen: [None; 6],
            count: [0; 6],
            session_ms: 0,
            rejected: 0,
            put_accepted: 0,
            put_failed: 0,
            throttled: 0,
            pending: [false; 6],
            unencodable: [false; 5],
            next_at: [now; 6],
            waiting: false,
            wait_until: None,
            #[cfg(test)]
            wake_notifications: 0,
            #[cfg(test)]
            snapshots: 0,
        }
    }

    fn snapshot(&mut self, due: [bool; 6]) -> Snapshot {
        for (index, selected) in due.iter().enumerate() {
            if *selected {
                self.pending[index] = false;
            }
        }
        #[cfg(test)]
        {
            self.snapshots += 1;
        }
        Snapshot {
            pose: if due[0] { self.pose } else { None },
            twist: if due[1] { self.twist } else { None },
            paired: if due[0] || due[1] { self.paired } else { None },
            imu: if due[2] { self.imu } else { None },
            battery: if due[3] { self.battery } else { None },
            flight: if due[4] { self.flight.clone() } else { None },
            controller: if due[5] {
                self.controller.clone()
            } else {
                None
            },
            seen: self.seen,
            count: self.count,
            session_ms: self.session_ms,
            rejected: self.rejected,
            put_accepted: self.put_accepted,
            put_failed: self.put_failed,
            throttled: self.throttled,
        }
    }
}

pub struct Uplink {
    shared: Arc<Mutex<Latest>>,
    wake: Arc<Condvar>,
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

pub fn client_config(endpoint: &str) -> Result<zenoh::Config, String> {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("mode", "\"client\"")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]"))
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("connect/exit_on_failure", "false")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .map_err(|e| e.to_string())?;
    Ok(config)
}

#[cfg(test)]
pub fn peer_listen_config(endpoint: &str) -> Result<zenoh::Config, String> {
    let mut config = zenoh::Config::default();
    config
        .insert_json5("mode", "\"peer\"")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]"))
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/multicast/enabled", "false")
        .map_err(|e| e.to_string())?;
    config
        .insert_json5("scouting/gossip/enabled", "false")
        .map_err(|e| e.to_string())?;
    Ok(config)
}

fn mark(latest: &mut Latest, index: usize, stamp_s: f64) {
    if latest.pending[index] {
        latest.throttled = latest.throttled.saturating_add(1);
    }
    latest.pending[index] = true;
    if index < 5 {
        latest.unencodable[index] = false;
    }
    latest.seen[index] = Some(Instant::now());
    latest.count[index] = latest.count[index].saturating_add(1);
    if let Some(ms) = positive_ms(stamp_s) {
        if ms > latest.session_ms {
            latest.session_ms = ms;
        }
    }
}

fn positive_ms(seconds: f64) -> Option<i64> {
    if !seconds.is_finite() || seconds <= 0.0 {
        return None;
    }
    let ms = (seconds * 1000.0).round();
    (ms > 0.0 && ms <= (i64::MAX / 1_000_000) as f64).then_some(ms as i64)
}

impl Uplink {
    pub fn start(
        robot_id: &str,
        endpoint: &str,
        rates: Rates,
        frame_id: &str,
        child_frame_id: &str,
    ) -> Result<Self, String> {
        let config = client_config(endpoint)?;
        let robot_id = robot_id.to_string();
        let frame_id = frame_id.to_string();
        let child_frame_id = child_frame_id.to_string();
        let shared = Arc::new(Mutex::new(Latest::new(Instant::now())));
        let wake = Arc::new(Condvar::new());
        let stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = std::sync::mpsc::channel();
        let thread_shared = Arc::clone(&shared);
        let thread_wake = Arc::clone(&wake);
        let thread_stop = Arc::clone(&stop);
        let join = thread::Builder::new()
            .name("station-io-zenoh".into())
            .spawn(move || {
                let session = match zenoh::open(config).wait() {
                    Ok(session) => session,
                    Err(e) => {
                        let _ = tx.send(Err(format!("zenoh open: {e}")));
                        return;
                    }
                };
                let mut keys = Vec::new();
                for leaf in LEAVES {
                    keys.push((leaf, format!("xgc2/{robot_id}/up/{leaf}")));
                }
                keys.push(("forwarder_hb", format!("xgc2/{robot_id}/up/forwarder_hb")));
                let mut publishers = Vec::new();
                for (leaf, key) in &keys {
                    match session
                        .declare_publisher(key)
                        .reliability(Reliability::BestEffort)
                        .congestion_control(CongestionControl::Drop)
                        .priority(Priority::Data)
                        .wait()
                    {
                        Ok(publisher) => publishers.push((*leaf, publisher)),
                        Err(e) => {
                            let _ = tx.send(Err(format!("zenoh publisher {leaf}: {e}")));
                            return;
                        }
                    }
                }
                let _ = tx.send(Ok(()));
                run_loop(
                    thread_shared,
                    thread_wake,
                    thread_stop,
                    publishers,
                    rates,
                    frame_id,
                    child_frame_id,
                    robot_id,
                    Instant::now(),
                );
            })
            .map_err(|e| e.to_string())?;
        match rx.recv_timeout(Duration::from_secs(8)) {
            Ok(Ok(())) => Ok(Self {
                shared,
                wake,
                stop,
                join: Some(join),
            }),
            Ok(Err(e)) => {
                stop.store(true, Ordering::Relaxed);
                let _ = join.join();
                Err(e)
            }
            Err(_) => {
                stop.store(true, Ordering::Relaxed);
                let _ = join.join();
                Err("zenoh client did not open".into())
            }
        }
    }

    pub fn pose(&self, sample: NativePose) {
        self.update(|latest| {
            if latest.paired.is_some() {
                return;
            }
            mark(latest, 0, sample.stamp_s);
            latest.pose = Some(sample);
        });
    }
    pub fn twist(&self, sample: NativeTwist) {
        self.update(|latest| {
            if latest.paired.is_some() {
                return;
            }
            mark(latest, 1, sample.stamp_s);
            latest.twist = Some(sample);
        });
    }
    pub fn paired(&self, sample: NativePaired) {
        self.update(|latest| {
            mark(latest, 0, sample.pose_stamp_s);
            mark(latest, 1, sample.twist_stamp_s);
            latest.paired = Some(sample);
        });
    }
    pub fn imu(&self, sample: NativeImu) {
        self.update(|latest| {
            mark(latest, 2, sample.stamp_s);
            latest.imu = Some(sample);
        });
    }
    pub fn battery(&self, sample: NativeBattery) {
        self.update(|latest| {
            mark(latest, 3, sample.stamp_s);
            latest.battery = Some(sample);
        });
    }
    pub fn flight(&self, sample: NativeFlight) {
        self.update(|latest| {
            mark(latest, 4, sample.stamp_s);
            latest.flight = Some(sample);
        });
    }
    pub fn controller(&self, stamp_s: f64, name: &str) {
        let name = name.to_string();
        self.update(|latest| {
            mark(latest, 5, stamp_s);
            latest.controller = Some(name);
        });
    }
    pub fn reject(&self) {
        self.update(|latest| latest.rejected = latest.rejected.saturating_add(1));
    }

    fn update(&self, f: impl FnOnce(&mut Latest)) {
        let mut guard = self.shared.lock().unwrap();
        f(&mut guard);
        // A running worker rechecks Latest before parking. Notify a parked
        // worker only if an update advances its actual send deadline.
        if guard.waiting {
            if let Some(deadline) = next_deadline(&guard, Instant::now()) {
                if guard.wait_until.is_none_or(|old| deadline < old) {
                    guard.wait_until = Some(deadline);
                    #[cfg(test)]
                    {
                        guard.wake_notifications += 1;
                    }
                    self.wake.notify_one();
                }
            }
        }
    }
}

impl Drop for Uplink {
    fn drop(&mut self) {
        // Share the predicate mutex with the condvar wait so idle stop cannot
        // be lost between the worker's check and its now-unbounded idle wait.
        {
            let _guard = self.shared.lock().unwrap();
            self.stop.store(true, Ordering::Relaxed);
            self.wake.notify_one();
        }
        if let Some(join) = self.join.take() {
            let _ = join.join();
        }
    }
}

fn run_loop(
    shared: Arc<Mutex<Latest>>,
    wake: Arc<Condvar>,
    stop: Arc<AtomicBool>,
    publishers: Vec<(&'static str, Publisher<'_>)>,
    rates: Rates,
    frame_id: String,
    child_frame_id: String,
    robot_id: String,
    started: Instant,
) {
    let intervals = [
        rates.local_pose_ns,
        rates.local_velocity_ns,
        rates.imu_ns,
        rates.power_ns,
        rates.flight_state_ns,
        rates.forwarder_hb_ns,
    ];
    let mut seq = [0u64; 6];
    loop {
        let (now, due, snap) = {
            let mut guard = shared.lock().unwrap();
            loop {
                guard.waiting = false;
                if stop.load(Ordering::Relaxed) {
                    return;
                }
                let now = Instant::now();
                let due = std::array::from_fn(|index| {
                    guard.next_at[index] <= now && eligible(&guard, index, now)
                });
                if due.iter().any(|selected| *selected) {
                    let snap = guard.snapshot(due);
                    break (now, due, snap);
                }
                guard.waiting = true;
                guard.wait_until = next_deadline(&guard, now);
                guard = if let Some(deadline) = guard.wait_until {
                    wake.wait_timeout(guard, deadline.saturating_duration_since(now))
                        .unwrap()
                        .0
                } else {
                    wake.wait(guard).unwrap()
                };
            }
        };
        let mut jobs: [Option<String>; 6] = std::array::from_fn(|_| None);
        for index in 0..6 {
            if !due[index] {
                continue;
            }
            jobs[index] = if index < 5 {
                encode(
                    &snap,
                    index,
                    seq[index].saturating_add(1),
                    &frame_id,
                    &child_frame_id,
                    now,
                )
            } else {
                heartbeat(&snap, &robot_id, seq[index].saturating_add(1), started, now)
            };
        }
        {
            let mut guard = shared.lock().unwrap();
            for index in 0..6 {
                if jobs[index].is_some() {
                    seq[index] = seq[index].saturating_add(1);
                    guard.next_at[index] = now + Duration::from_nanos(intervals[index]);
                } else if due[index] && index < 5 && !guard.pending[index] {
                    // Do not spin on an unencodable generation. A new source
                    // sample clears this gate and remains eligible immediately.
                    guard.unencodable[index] = true;
                }
            }
        }
        for (index, body) in jobs.into_iter().enumerate() {
            if let Some(body) = body {
                let published = publishers[index].1.put(body.into_bytes()).wait();
                let mut guard = shared.lock().unwrap();
                match published {
                    Ok(()) => guard.put_accepted = guard.put_accepted.saturating_add(1),
                    Err(_) => guard.put_failed = guard.put_failed.saturating_add(1),
                }
            }
        }
    }
}

fn occupied(latest: &Latest, index: usize) -> bool {
    match index {
        0 => latest.paired.is_some() || latest.pose.is_some(),
        1 => latest.paired.is_some() || latest.twist.is_some(),
        2 => latest.imu.is_some(),
        3 => latest.battery.is_some(),
        4 => latest.flight.is_some(),
        _ => false,
    }
}

fn eligible(latest: &Latest, index: usize, at: Instant) -> bool {
    if index == 5 {
        latest.session_ms > 0
    } else {
        occupied(latest, index)
            && !latest.unencodable[index]
            && fresh(&latest.seen, index, FRESH_MS[index], at)
    }
}

fn next_deadline(latest: &Latest, now: Instant) -> Option<Instant> {
    (0..6)
        .filter_map(|index| {
            let at = latest.next_at[index].max(now);
            eligible(latest, index, at).then_some(at)
        })
        .min()
}

fn fresh(seen: &[Option<Instant>; 6], index: usize, window_ms: i64, now: Instant) -> bool {
    match seen[index] {
        Some(at) => now.saturating_duration_since(at).as_millis() as i64 <= window_ms,
        None => false,
    }
}

fn encode(
    latest: &Snapshot,
    index: usize,
    sequence: u64,
    frame_id: &str,
    child: &str,
    now: Instant,
) -> Option<String> {
    match index {
        0 => {
            if !fresh(&latest.seen, 0, 1000, now) {
                return None;
            }
            if let Some(paired) = latest.paired.as_ref() {
                let q_wxyz = [
                    paired.q_xyzw[3],
                    paired.q_xyzw[0],
                    paired.q_xyzw[1],
                    paired.q_xyzw[2],
                ];
                return wire::pose_json(
                    sequence,
                    paired.pose_stamp_s,
                    frame_id,
                    child,
                    &paired.position,
                    &q_wxyz,
                );
            }
            let pose = latest.pose.as_ref()?;
            wire::pose_json(
                sequence,
                pose.stamp_s,
                frame_id,
                child,
                &pose.position,
                &pose.q_wxyz,
            )
        }
        1 => {
            if !fresh(&latest.seen, 1, 1000, now) {
                return None;
            }
            if let Some(paired) = latest.paired.as_ref() {
                return wire::twist_json(
                    sequence,
                    paired.twist_stamp_s,
                    child,
                    &paired.linear,
                    None,
                );
            }
            let twist = latest.twist.as_ref()?;
            wire::twist_json(
                sequence,
                twist.stamp_s,
                child,
                &twist.linear,
                Some(&twist.angular),
            )
        }
        2 => {
            let imu = latest.imu.as_ref()?;
            if !fresh(&latest.seen, 2, 1000, now) {
                return None;
            }
            wire::imu_json(sequence, imu.stamp_s, child, &imu.gyro, &imu.accel)
        }
        3 => {
            let battery = latest.battery.as_ref()?;
            if !fresh(&latest.seen, 3, 3000, now) {
                return None;
            }
            wire::power_json(
                sequence,
                battery.stamp_s,
                battery.percentage,
                battery.voltage,
            )
        }
        4 => {
            let flight = latest.flight.as_ref()?;
            if !fresh(&latest.seen, 4, 3000, now) {
                return None;
            }
            wire::flight_json(
                sequence,
                flight.stamp_s,
                flight.connected,
                flight.armed,
                flight.guided,
                flight.manual_input,
                &flight.mode,
                flight.system_status,
            )
        }
        _ => None,
    }
}

fn heartbeat(
    latest: &Snapshot,
    robot_id: &str,
    sequence: u64,
    started: Instant,
    now: Instant,
) -> Option<String> {
    let names: [&str; 6] = [
        "local_pose",
        "local_velocity",
        "imu",
        "power",
        "flight_state",
        "controller",
    ];
    let controller_text = if fresh(&latest.seen, 5, 3000, now) {
        latest.controller.as_ref()
    } else {
        None
    };
    let channels: Vec<_> = names
        .iter()
        .zip(FRESH_MS)
        .enumerate()
        .map(|(i, (id, window))| {
            let age = latest.seen[i]
                .map(|at| now.saturating_duration_since(at).as_millis() as i64)
                .unwrap_or(-1);
            let ready = age >= 0 && age <= window;
            HeartbeatChannel {
                id: *id,
                source_samples: latest.count[i],
                source_age_ms: age,
                ready,
                text: if *id == "controller" && ready {
                    controller_text.cloned()
                } else {
                    None
                },
            }
        })
        .collect();
    let uptime = now.saturating_duration_since(started).as_millis() as i64;
    wire::heartbeat_json(
        robot_id,
        sequence,
        latest.session_ms,
        uptime,
        &channels,
        latest.put_accepted,
        latest.put_failed,
        latest.throttled,
        latest.rejected,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex as StdMutex;

    fn ephemeral_endpoint() -> String {
        let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);
        format!("tcp/127.0.0.1:{port}")
    }

    fn await_state(uplink: &Uplink, predicate: impl Fn(&Latest) -> bool) {
        let deadline = Instant::now() + Duration::from_secs(3);
        while !predicate(&uplink.shared.lock().unwrap()) {
            assert!(Instant::now() < deadline, "uplink state deadline exceeded");
            thread::sleep(Duration::from_millis(1));
        }
    }

    #[test]
    fn idle_and_rejected_inputs_do_not_snapshot_or_notify() {
        let endpoint = ephemeral_endpoint();
        let _session = zenoh::open(peer_listen_config(&endpoint).unwrap())
            .wait()
            .unwrap();
        let uplink =
            Uplink::start("idle", &endpoint, Rates::contract(), "world", "base_link").unwrap();
        await_state(&uplink, |latest| {
            latest.waiting && latest.wait_until.is_none()
        });
        for _ in 0..100 {
            uplink.reject();
        }
        thread::sleep(Duration::from_millis(100));
        {
            let latest = uplink.shared.lock().unwrap();
            assert_eq!(latest.snapshots, 0);
            assert_eq!(latest.wake_notifications, 0);
            assert_eq!(latest.throttled, 0);
            assert_eq!(latest.rejected, 100);
        }
        let stopped = Instant::now();
        drop(uplink);
        assert!(
            stopped.elapsed() < Duration::from_secs(2),
            "idle worker did not join"
        );
    }

    #[test]
    fn pending_generations_coalesce_without_early_wake_or_recount() {
        let endpoint = ephemeral_endpoint();
        let session = zenoh::open(peer_listen_config(&endpoint).unwrap())
            .wait()
            .unwrap();
        let received = Arc::new(StdMutex::new(Vec::<serde_json::Value>::new()));
        let slot = Arc::clone(&received);
        let _sub = session
            .declare_subscriber("xgc2/coalescing/up/local_pose")
            .callback(move |sample| {
                let body = serde_json::from_slice(&sample.payload().to_bytes()).unwrap();
                slot.lock().unwrap().push(body);
            })
            .wait()
            .unwrap();
        let mut rates = Rates::contract();
        rates.local_pose_ns = 500_000_000;
        let uplink = Uplink::start("coalescing", &endpoint, rates, "world", "base_link").unwrap();
        uplink.pose(NativePose {
            stamp_s: 1.0,
            position: [0.0; 3],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        });
        await_state(&uplink, |latest| latest.put_accepted >= 2 && latest.waiting);
        let (snapshots, notifications) = {
            let latest = uplink.shared.lock().unwrap();
            (latest.snapshots, latest.wake_notifications)
        };
        for n in 1..=100 {
            uplink.pose(NativePose {
                stamp_s: 1.0 + n as f64 * 0.001,
                position: [n as f64, 0.0, 0.0],
                q_wxyz: [1.0, 0.0, 0.0, 0.0],
            });
        }
        {
            let latest = uplink.shared.lock().unwrap();
            assert_eq!(latest.count[0], 101);
            assert_eq!(
                latest.throttled, 99,
                "one count per superseded source generation"
            );
            assert_eq!(latest.snapshots, snapshots);
            assert_eq!(latest.wake_notifications, notifications);
        }
        let deadline = Instant::now() + Duration::from_secs(2);
        while !received
            .lock()
            .unwrap()
            .iter()
            .any(|body| body["position"]["x"] == 100.0)
        {
            assert!(
                Instant::now() < deadline,
                "latest coalesced pose was not received"
            );
            thread::sleep(Duration::from_millis(5));
        }
        thread::sleep(Duration::from_millis(100));
        assert_eq!(
            uplink.shared.lock().unwrap().throttled,
            99,
            "idle/deadline loops must not recount coalescing"
        );
        drop(uplink);
    }

    #[test]
    fn unencodable_generation_parks_until_a_new_sample() {
        let endpoint = ephemeral_endpoint();
        let _session = zenoh::open(peer_listen_config(&endpoint).unwrap())
            .wait()
            .unwrap();
        let uplink = Uplink::start(
            "invalid",
            &endpoint,
            Rates::contract(),
            "world",
            "base_link",
        )
        .unwrap();
        uplink.pose(NativePose {
            stamp_s: f64::NAN,
            position: [0.0; 3],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        });
        await_state(&uplink, |latest| {
            latest.unencodable[0] && latest.waiting && latest.wait_until.is_none()
        });
        let snapshots = uplink.shared.lock().unwrap().snapshots;
        thread::sleep(Duration::from_millis(100));
        assert_eq!(uplink.shared.lock().unwrap().snapshots, snapshots);
        uplink.pose(NativePose {
            stamp_s: 2.0,
            position: [1.0; 3],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        });
        await_state(&uplink, |latest| latest.put_accepted >= 2);
        assert!(!uplink.shared.lock().unwrap().unencodable[0]);
        drop(uplink);
    }

    #[test]
    fn pose_crosses_zenoh_with_session_milliseconds() {
        let endpoint = ephemeral_endpoint();
        let config = peer_listen_config(&endpoint).unwrap();
        let session = zenoh::open(config).wait().unwrap();
        let got = Arc::new(StdMutex::new(Vec::<(String, String)>::new()));
        let slot = Arc::clone(&got);
        let _sub = session
            .declare_subscriber("xgc2/*/up/**")
            .callback(move |sample| {
                let key = sample.key_expr().to_string();
                let body = String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned();
                slot.lock().unwrap().push((key, body));
            })
            .wait()
            .unwrap();
        let uplink = Uplink::start(
            "xgc2e-0123456789abcdef0123",
            &endpoint,
            Rates::contract(),
            "world",
            "base_link",
        )
        .unwrap();
        uplink.pose(NativePose {
            stamp_s: 12.5,
            position: [1.0, 2.0, 3.0],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        });
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut found = None;
        while Instant::now() < deadline {
            if let Some(hit) = got
                .lock()
                .unwrap()
                .iter()
                .find(|(key, _)| key.ends_with("/up/local_pose"))
            {
                found = Some(hit.clone());
                break;
            }
            thread::sleep(Duration::from_millis(20));
        }
        drop(uplink);
        let (key, body) = found.expect("local_pose did not arrive");
        assert_eq!(key, "xgc2/xgc2e-0123456789abcdef0123/up/local_pose");
        assert!(body.contains("\"t_ms\":12500"), "{body}");
        assert!(!body.contains("1970"));
    }

    #[test]
    fn paired_velocity_crosses_zenoh_with_null_angular() {
        let endpoint = {
            let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            let port = listener.local_addr().unwrap().port();
            drop(listener);
            format!("tcp/127.0.0.1:{port}")
        };
        let session = zenoh::open(peer_listen_config(&endpoint).unwrap())
            .wait()
            .unwrap();
        let got = Arc::new(StdMutex::new(Vec::<(String, String)>::new()));
        let slot = Arc::clone(&got);
        let _sub = session
            .declare_subscriber("xgc2/*/up/**")
            .callback(move |sample| {
                let key = sample.key_expr().to_string();
                let body = String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned();
                slot.lock().unwrap().push((key, body));
            })
            .wait()
            .unwrap();
        let uplink = Uplink::start(
            "xgc2e-0123456789abcdef0123",
            &endpoint,
            Rates::contract(),
            "world",
            "base_link",
        )
        .unwrap();
        uplink.paired(NativePaired {
            pose_stamp_s: 2.0,
            twist_stamp_s: 2.0,
            position: [4.0, 0.0, 0.0],
            q_xyzw: [0.0, 0.0, 0.0, 1.0],
            linear: [0.5, 0.0, 0.0],
        });
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut found = None;
        while Instant::now() < deadline {
            if let Some(hit) = got
                .lock()
                .unwrap()
                .iter()
                .find(|(key, _)| key.ends_with("/up/local_velocity"))
            {
                found = Some(hit.1.clone());
                break;
            }
            thread::sleep(Duration::from_millis(20));
        }
        drop(uplink);
        let body = found.expect("paired local_velocity did not arrive");
        let value: serde_json::Value = serde_json::from_str(&body).unwrap();
        assert!(value["angular"].is_null(), "{body}");
        assert_eq!(value["linear"]["x"], 0.5);
        assert!(!body.contains("\"angular\":{\"x\":0.0"));
    }

    #[test]
    fn contract_ceilings_bound_a_flooded_uplink() {
        let endpoint = {
            let listener = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            let port = listener.local_addr().unwrap().port();
            drop(listener);
            format!("tcp/127.0.0.1:{port}")
        };
        let session = zenoh::open(peer_listen_config(&endpoint).unwrap())
            .wait()
            .unwrap();
        let got = Arc::new(StdMutex::new(Vec::<(String, String)>::new()));
        let slot = Arc::clone(&got);
        let _sub = session
            .declare_subscriber("xgc2/*/up/**")
            .callback(move |sample| {
                let key = sample.key_expr().to_string();
                let body = String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned();
                slot.lock().unwrap().push((key, body));
            })
            .wait()
            .unwrap();
        let uplink = Uplink::start(
            "xgc2e-0123456789abcdef0123",
            &endpoint,
            Rates::contract(),
            "world",
            "base_link",
        )
        .unwrap();
        let until = Instant::now() + Duration::from_secs(2);
        let mut n = 0u64;
        while Instant::now() < until {
            let stamp = 1.0 + n as f64 * 0.001;
            uplink.pose(NativePose {
                stamp_s: stamp,
                position: [1.0, 2.0, 3.0],
                q_wxyz: [1.0, 0.0, 0.0, 0.0],
            });
            uplink.twist(NativeTwist {
                stamp_s: stamp,
                linear: [0.2, 0.0, 0.0],
                angular: [0.0, 0.0, 0.1],
            });
            uplink.imu(NativeImu {
                stamp_s: stamp,
                accel: [0.0, 0.0, 9.81],
                gyro: [0.0, 0.0, 0.25],
            });
            uplink.battery(NativeBattery {
                stamp_s: stamp,
                voltage: 16.0,
                percentage: 0.5,
            });
            uplink.flight(NativeFlight {
                stamp_s: stamp,
                connected: true,
                armed: false,
                guided: false,
                manual_input: false,
                system_status: 4,
                mode: "POSCTL".into(),
            });
            uplink.controller(stamp, "Ready");
            n += 1;
            thread::sleep(Duration::from_millis(2));
        }
        thread::sleep(Duration::from_millis(250));
        drop(uplink);
        let samples = got.lock().unwrap();
        let mut counts = std::collections::BTreeMap::<String, usize>::new();
        let mut saw_ready_text = false;
        for (key, body) in samples.iter() {
            let leaf = key.rsplit('/').next().unwrap().to_string();
            if leaf == "imu" {
                assert!(body.contains("\"orientation\":null"), "{body}");
                assert!(
                    !body.contains("[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"),
                    "{body}"
                );
            }
            if leaf == "forwarder_hb" && body.contains("\"text\":\"Ready\"") {
                saw_ready_text = true;
            }
            if leaf == "power" {
                assert!(body.contains("\"charging\":null"), "{body}");
                assert!(!body.contains("\"charging\":false"), "{body}");
            }
            *counts.entry(leaf).or_insert(0) += 1;
        }
        assert!(saw_ready_text, "no heartbeat carried controller text Ready");
        let pose = *counts.get("local_pose").unwrap_or(&0);
        let velocity = *counts.get("local_velocity").unwrap_or(&0);
        let imu = *counts.get("imu").unwrap_or(&0);
        let power = *counts.get("power").unwrap_or(&0);
        let flight = *counts.get("flight_state").unwrap_or(&0);
        let heartbeat = *counts.get("forwarder_hb").unwrap_or(&0);
        assert!(
            (12..=36).contains(&pose),
            "local_pose {pose} outside 15 Hz over 2 s"
        );
        assert!(
            (12..=36).contains(&velocity),
            "local_velocity {velocity} outside 15 Hz over 2 s"
        );
        assert!((8..=26).contains(&imu), "imu {imu} outside 10 Hz over 2 s");
        assert!(
            (2..=6).contains(&power),
            "power {power} outside 2 Hz over 2 s"
        );
        assert!(
            (2..=6).contains(&flight),
            "flight_state {flight} outside 2 Hz over 2 s"
        );
        assert!(
            (1..=4).contains(&heartbeat),
            "forwarder_hb {heartbeat} outside 1 Hz over 2 s"
        );
        assert!(counts.keys().all(|leaf| matches!(
            leaf.as_str(),
            "local_pose" | "local_velocity" | "imu" | "power" | "flight_state" | "forwarder_hb"
        )));
    }
}
