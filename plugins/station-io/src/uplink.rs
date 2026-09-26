//! One Zenoh client for the six GCS keys. No radio listen, no subscription.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use zenoh::pubsub::Publisher;
use zenoh::qos::{CongestionControl, Priority, Reliability};
use zenoh::Wait;

use crate::wire::{self, HeartbeatChannel, NativeBattery, NativeFlight, NativeImu, NativePaired, NativePose, NativeTwist, Rates};

const LEAVES: [&str; 5] = ["local_pose", "local_velocity", "imu", "power", "flight_state"];

#[derive(Clone)]
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
    success: u64,
    failure: u64,
    throttled: u64,
}

pub struct Uplink {
    shared: Arc<Mutex<Latest>>,
    wake: Arc<Condvar>,
    stop: Arc<AtomicBool>,
    join: Option<JoinHandle<()>>,
}

pub fn client_config(endpoint: &str) -> Result<zenoh::Config, String> {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"client\"").map_err(|e| e.to_string())?;
    config.insert_json5("connect/endpoints", &format!("[\"{endpoint}\"]")).map_err(|e| e.to_string())?;
    config.insert_json5("connect/exit_on_failure", "false").map_err(|e| e.to_string())?;
    config.insert_json5("scouting/multicast/enabled", "false").map_err(|e| e.to_string())?;
    config.insert_json5("scouting/gossip/enabled", "false").map_err(|e| e.to_string())?;
    Ok(config)
}

#[cfg(test)]
pub fn peer_listen_config(endpoint: &str) -> Result<zenoh::Config, String> {
    let mut config = zenoh::Config::default();
    config.insert_json5("mode", "\"peer\"").map_err(|e| e.to_string())?;
    config.insert_json5("listen/endpoints", &format!("[\"{endpoint}\"]")).map_err(|e| e.to_string())?;
    config.insert_json5("scouting/multicast/enabled", "false").map_err(|e| e.to_string())?;
    config.insert_json5("scouting/gossip/enabled", "false").map_err(|e| e.to_string())?;
    Ok(config)
}

fn mark(latest: &mut Latest, index: usize, stamp_s: f64) {
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
    pub fn start(robot_id: &str, endpoint: &str, rates: Rates, frame_id: &str, child_frame_id: &str) -> Result<Self, String> {
        let config = client_config(endpoint)?;
        let robot_id = robot_id.to_string();
        let frame_id = frame_id.to_string();
        let child_frame_id = child_frame_id.to_string();
        let shared = Arc::new(Mutex::new(Latest {
            pose: None, twist: None, paired: None, imu: None, battery: None, flight: None, controller: None,
            seen: [None; 6], count: [0; 6], session_ms: 0,
            rejected: 0, success: 0, failure: 0, throttled: 0,
        }));
        let wake = Arc::new(Condvar::new());
        let stop = Arc::new(AtomicBool::new(false));
        let (tx, rx) = std::sync::mpsc::channel();
        let thread_shared = Arc::clone(&shared);
        let thread_wake = Arc::clone(&wake);
        let thread_stop = Arc::clone(&stop);
        let join = thread::Builder::new().name("station-io-zenoh".into()).spawn(move || {
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
                match session.declare_publisher(key)
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
            run_loop(thread_shared, thread_wake, thread_stop, publishers, rates, frame_id, child_frame_id, robot_id, Instant::now());
        }).map_err(|e| e.to_string())?;
        match rx.recv_timeout(Duration::from_secs(8)) {
            Ok(Ok(())) => Ok(Self { shared, wake, stop, join: Some(join) }),
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
        self.update(|latest| { mark(latest, 2, sample.stamp_s); latest.imu = Some(sample); });
    }
    pub fn battery(&self, sample: NativeBattery) {
        self.update(|latest| { mark(latest, 3, sample.stamp_s); latest.battery = Some(sample); });
    }
    pub fn flight(&self, sample: NativeFlight) {
        self.update(|latest| { mark(latest, 4, sample.stamp_s); latest.flight = Some(sample); });
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
        self.wake.notify_one();
    }
}

impl Drop for Uplink {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.wake.notify_one();
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
        rates.local_pose_ns, rates.local_velocity_ns, rates.imu_ns, rates.power_ns, rates.flight_state_ns, rates.forwarder_hb_ns,
    ];
    let mut seq = [0u64; 6];
    let mut next_at = [Instant::now(); 6];
    while !stop.load(Ordering::Relaxed) {
        let now = Instant::now();
        let mut due = Vec::new();
        let snap = {
            let mut guard = shared.lock().unwrap();
            for index in 0..5 {
                if now < next_at[index] {
                    if occupied(&guard, index) {
                        guard.throttled = guard.throttled.saturating_add(1);
                    }
                    continue;
                }
                if occupied(&guard, index) {
                    due.push(index);
                }
            }
            if now >= next_at[5] && guard.session_ms > 0 {
                due.push(5);
            }
            guard.clone()
        };
        let mut jobs: Vec<(usize, String)> = Vec::new();
        for index in due {
            if index < 5 {
                if let Some(body) = encode(&snap, index, seq[index].saturating_add(1), &frame_id, &child_frame_id) {
                    seq[index] = seq[index].saturating_add(1);
                    next_at[index] = now + Duration::from_nanos(intervals[index]);
                    jobs.push((index, body));
                }
            } else if let Some(body) = heartbeat(&snap, &robot_id, seq[5].saturating_add(1), started, now) {
                seq[5] = seq[5].saturating_add(1);
                next_at[5] = now + Duration::from_nanos(intervals[5]);
                jobs.push((5, body));
            }
        }
        for (index, body) in jobs {
            let published = publishers[index].1.put(body.into_bytes()).wait();
            let mut guard = shared.lock().unwrap();
            match published {
                Ok(()) => guard.success = guard.success.saturating_add(1),
                Err(_) => guard.failure = guard.failure.saturating_add(1),
            }
        }
        let guard = shared.lock().unwrap();
        if !stop.load(Ordering::Relaxed) {
            let _parked = wake.wait_timeout(guard, Duration::from_millis(20)).unwrap();
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

fn fresh(latest: &Latest, index: usize, window_ms: i64, now: Instant) -> bool {
    match latest.seen[index] {
        Some(at) => now.saturating_duration_since(at).as_millis() as i64 <= window_ms,
        None => false,
    }
}

fn encode(latest: &Latest, index: usize, sequence: u64, frame_id: &str, child: &str) -> Option<String> {
    let now = Instant::now();
    match index {
        0 => {
            if !fresh(latest, 0, 1000, now) { return None; }
            if let Some(paired) = latest.paired.as_ref() {
                let q_wxyz = [paired.q_xyzw[3], paired.q_xyzw[0], paired.q_xyzw[1], paired.q_xyzw[2]];
                return wire::pose_json(sequence, paired.pose_stamp_s, frame_id, child, &paired.position, &q_wxyz);
            }
            let pose = latest.pose.as_ref()?;
            wire::pose_json(sequence, pose.stamp_s, frame_id, child, &pose.position, &pose.q_wxyz)
        }
        1 => {
            if !fresh(latest, 1, 1000, now) { return None; }
            if let Some(paired) = latest.paired.as_ref() {
                return wire::twist_json(sequence, paired.twist_stamp_s, child, &paired.linear, None);
            }
            let twist = latest.twist.as_ref()?;
            wire::twist_json(sequence, twist.stamp_s, child, &twist.linear, Some(&twist.angular))
        }
        2 => {
            let imu = latest.imu.as_ref()?;
            if !fresh(latest, 2, 1000, now) { return None; }
            wire::imu_json(sequence, imu.stamp_s, child, &imu.gyro, &imu.accel)
        }
        3 => {
            let battery = latest.battery.as_ref()?;
            if !fresh(latest, 3, 3000, now) { return None; }
            wire::power_json(sequence, battery.stamp_s, battery.percentage, battery.voltage)
        }
        4 => {
            let flight = latest.flight.as_ref()?;
            if !fresh(latest, 4, 3000, now) { return None; }
            wire::flight_json(sequence, flight.stamp_s, flight.connected, flight.armed, flight.guided, flight.manual_input, &flight.mode, flight.system_status)
        }
        _ => None,
    }
}

fn heartbeat(latest: &Latest, robot_id: &str, sequence: u64, started: Instant, now: Instant) -> Option<String> {
    let names: [&str; 6] = ["local_pose", "local_velocity", "imu", "power", "flight_state", "controller"];
    let windows = [1000i64, 1000, 1000, 3000, 3000, 3000];
    let controller_text = if fresh(latest, 5, 3000, now) { latest.controller.clone() } else { None };
    let channels: Vec<_> = names.iter().zip(windows).enumerate().map(|(i, (id, window))| {
        let age = latest.seen[i].map(|at| now.saturating_duration_since(at).as_millis() as i64).unwrap_or(-1);
        let ready = age >= 0 && age <= window;
        HeartbeatChannel {
            id: *id,
            source_samples: latest.count[i],
            source_age_ms: age,
            ready,
            text: if *id == "controller" && ready { controller_text.clone() } else { None },
        }
    }).collect();
    let uptime = now.saturating_duration_since(started).as_millis() as i64;
    wire::heartbeat_json(robot_id, sequence, latest.session_ms, uptime, &channels, latest.success, latest.failure, latest.throttled, latest.rejected)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Mutex as StdMutex;

    #[test]
    fn pose_crosses_zenoh_with_session_milliseconds() {
        let endpoint = "tcp/127.0.0.1:17461";
        let config = peer_listen_config(endpoint).unwrap();
        let session = zenoh::open(config).wait().unwrap();
        let got = Arc::new(StdMutex::new(Vec::<(String, String)>::new()));
        let slot = Arc::clone(&got);
        let _sub = session.declare_subscriber("xgc2/*/up/**").callback(move |sample| {
            let key = sample.key_expr().to_string();
            let body = String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned();
            slot.lock().unwrap().push((key, body));
        }).wait().unwrap();
        let uplink = Uplink::start(
            "xgc2e-0123456789abcdef0123",
            endpoint,
            Rates::contract(),
            "world",
            "base_link",
        ).unwrap();
        uplink.pose(NativePose {
            stamp_s: 12.5,
            position: [1.0, 2.0, 3.0],
            q_wxyz: [1.0, 0.0, 0.0, 0.0],
        });
        let deadline = Instant::now() + Duration::from_secs(3);
        let mut found = None;
        while Instant::now() < deadline {
            if let Some(hit) = got.lock().unwrap().iter().find(|(key, _)| key.ends_with("/up/local_pose")) {
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
        let session = zenoh::open(peer_listen_config(&endpoint).unwrap()).wait().unwrap();
        let got = Arc::new(StdMutex::new(Vec::<(String, String)>::new()));
        let slot = Arc::clone(&got);
        let _sub = session.declare_subscriber("xgc2/*/up/**").callback(move |sample| {
            let key = sample.key_expr().to_string();
            let body = String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned();
            slot.lock().unwrap().push((key, body));
        }).wait().unwrap();
        let uplink = Uplink::start("xgc2e-0123456789abcdef0123", &endpoint, Rates::contract(), "world", "base_link").unwrap();
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
            if let Some(hit) = got.lock().unwrap().iter().find(|(key, _)| key.ends_with("/up/local_velocity")) {
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
        let session = zenoh::open(peer_listen_config(&endpoint).unwrap()).wait().unwrap();
        let got = Arc::new(StdMutex::new(Vec::<(String, String)>::new()));
        let slot = Arc::clone(&got);
        let _sub = session.declare_subscriber("xgc2/*/up/**").callback(move |sample| {
            let key = sample.key_expr().to_string();
            let body = String::from_utf8_lossy(&sample.payload().to_bytes()).into_owned();
            slot.lock().unwrap().push((key, body));
        }).wait().unwrap();
        let uplink = Uplink::start(
            "xgc2e-0123456789abcdef0123",
            &endpoint,
            Rates::contract(),
            "world",
            "base_link",
        ).unwrap();
        let until = Instant::now() + Duration::from_secs(2);
        let mut n = 0u64;
        while Instant::now() < until {
            let stamp = 1.0 + n as f64 * 0.001;
            uplink.pose(NativePose { stamp_s: stamp, position: [1.0, 2.0, 3.0], q_wxyz: [1.0, 0.0, 0.0, 0.0] });
            uplink.twist(NativeTwist { stamp_s: stamp, linear: [0.2, 0.0, 0.0], angular: [0.0, 0.0, 0.1] });
            uplink.imu(NativeImu { stamp_s: stamp, accel: [0.0, 0.0, 9.81], gyro: [0.0, 0.0, 0.25] });
            uplink.battery(NativeBattery { stamp_s: stamp, voltage: 16.0, percentage: 0.5 });
            uplink.flight(NativeFlight {
                stamp_s: stamp, connected: true, armed: false, guided: false, manual_input: false,
                system_status: 4, mode: "POSCTL".into(),
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
                assert!(!body.contains("[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"), "{body}");
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
        assert!((12..=36).contains(&pose), "local_pose {pose} outside 15 Hz over 2 s");
        assert!((12..=36).contains(&velocity), "local_velocity {velocity} outside 15 Hz over 2 s");
        assert!((8..=26).contains(&imu), "imu {imu} outside 10 Hz over 2 s");
        assert!((2..=6).contains(&power), "power {power} outside 2 Hz over 2 s");
        assert!((2..=6).contains(&flight), "flight_state {flight} outside 2 Hz over 2 s");
        assert!((1..=4).contains(&heartbeat), "forwarder_hb {heartbeat} outside 1 Hz over 2 s");
        assert!(counts.keys().all(|leaf| matches!(
            leaf.as_str(),
            "local_pose" | "local_velocity" | "imu" | "power" | "flight_state" | "forwarder_hb"
        )));
    }
}
