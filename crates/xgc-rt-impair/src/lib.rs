//! A UDP relay that impairs one direction with a seeded profile and logs
//! per-sample ground truth, in place of `tc netem` where there is no
//! CAP_NET_ADMIN (the sandbox). On the station, netem on the radio veth is
//! the real impairment (D-116 `network-station.sh`).
//!
//! Topology: a client (one node's Zenoh `connect`) sends to `listen`; the
//! relay forwards to `target` (the other node's Zenoh `listen`) and passes
//! replies back. Only the client→target direction is impaired, and only
//! datagrams that carry at least one verified XSE2 envelope: Zenoh session
//! control (handshake, keep-alive) passes untouched, so the session itself
//! is not what is being tested.
//!
//! Per datagram, one action is drawn: drop (Bernoulli or Gilbert–Elliott),
//! duplicate, hold-for-reorder (released after the next forwarded
//! datagram), or forward; forwarded copies are delayed by
//! `delay ± jitter` (uniform; jitter can itself reorder). Every envelope in
//! the datagram is logged with that action and its release times.

use std::collections::BinaryHeap;
use std::cmp::Reverse;
use std::net::{SocketAddr, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde::{Deserialize, Serialize};
use xgc_rt_core::envelope;

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize, Default)]
#[serde(deny_unknown_fields)]
pub struct GilbertElliott {
    /// P(good → bad) per datagram.
    pub p_good_bad: f64,
    /// P(bad → good) per datagram.
    pub p_bad_good: f64,
    pub loss_good: f64,
    pub loss_bad: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Deserialize, Serialize, Default)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    #[serde(default)]
    pub delay_ms: f64,
    #[serde(default)]
    pub jitter_ms: f64,
    /// Bernoulli loss; ignored when `gilbert_elliott` is set.
    #[serde(default)]
    pub loss: f64,
    #[serde(default)]
    pub gilbert_elliott: Option<GilbertElliott>,
    #[serde(default)]
    pub duplicate: f64,
    #[serde(default)]
    pub reorder: f64,
    #[serde(default)]
    pub seed: u64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Action {
    Forward,
    Drop,
    Duplicate,
    Reorder,
}

/// One envelope seen by the relay.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct TruthRecord {
    pub origin: u16,
    pub channel: u32,
    pub seq: u64,
    pub action: Action,
    /// Wall ns when the datagram arrived at the relay.
    pub t_in: i64,
    /// Injected delay per released copy, ns (empty when dropped).
    pub delays_ns: Vec<i64>,
}

fn wall_ns() -> i64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_nanos() as i64).unwrap_or(0)
}

struct Rng(u64);

impl Rng {
    fn unit(&mut self) -> f64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        (x.wrapping_mul(0x2545_F491_4F6C_DD1D) >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// Every verified envelope inside a datagram: (origin, channel, seq).
pub fn envelopes_in(datagram: &[u8]) -> Vec<(u16, u32, u64)> {
    let mut found = Vec::new();
    let mut i = 0;
    while i + envelope::HEADER_LEN <= datagram.len() {
        if datagram[i..i + 4] == envelope::MAGIC {
            let len = u32::from_le_bytes(datagram[i + 52..i + 56].try_into().unwrap()) as usize;
            let end = i + envelope::HEADER_LEN + len;
            if end <= datagram.len() {
                if let Ok((h, _)) = envelope::decode(&datagram[i..end]) {
                    found.push((h.origin, h.channel, h.seq));
                    i = end;
                    continue;
                }
            }
        }
        i += 1;
    }
    found
}

struct Pending {
    due: Instant,
    order: u64,
    bytes: Vec<u8>,
}

impl PartialEq for Pending {
    fn eq(&self, o: &Self) -> bool {
        (self.due, self.order) == (o.due, o.order)
    }
}
impl Eq for Pending {}
impl PartialOrd for Pending {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Pending {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        (self.due, self.order).cmp(&(o.due, o.order))
    }
}

pub struct Relay {
    pub listen: SocketAddr,
    stop: Arc<AtomicBool>,
    threads: Vec<std::thread::JoinHandle<()>>,
    truth: Arc<Mutex<Vec<TruthRecord>>>,
}

impl Relay {
    /// Start relaying `listen` → `target` with `profile` on that direction.
    pub fn start(listen: SocketAddr, target: SocketAddr, profile: Profile) -> std::io::Result<Self> {
        let front = UdpSocket::bind(listen)?;
        let back = UdpSocket::bind(SocketAddr::new(target.ip(), 0))?;
        back.connect(target)?;
        front.set_read_timeout(Some(Duration::from_millis(20)))?;
        back.set_read_timeout(Some(Duration::from_millis(20)))?;
        let listen = front.local_addr()?;
        let stop = Arc::new(AtomicBool::new(false));
        let truth = Arc::new(Mutex::new(Vec::new()));
        let client: Arc<Mutex<Option<SocketAddr>>> = Arc::new(Mutex::new(None));
        let queue: Arc<(Mutex<BinaryHeap<Reverse<Pending>>>, std::sync::Condvar)> =
            Arc::new((Mutex::new(BinaryHeap::new()), std::sync::Condvar::new()));
        let (front, back) = (Arc::new(front), Arc::new(back));
        let mut threads = Vec::new();

        // client → target: impair.
        {
            let (front, stop, truth, client, queue) = (front.clone(), stop.clone(), truth.clone(), client.clone(), queue.clone());
            threads.push(std::thread::Builder::new().name("impair-in".into()).spawn(move || {
                let mut rng = Rng(profile.seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1);
                let mut ge_bad = false;
                let mut held: Option<(Vec<u8>, Vec<usize>)> = None;
                let mut order = 0u64;
                let mut buf = vec![0u8; 65_536];
                let jitter_ns = (profile.jitter_ms * 1e6) as i64;
                let base_ns = (profile.delay_ms * 1e6) as i64;
                let schedule = |bytes: Vec<u8>, rng: &mut Rng, order: &mut u64| -> i64 {
                    let j = if jitter_ns > 0 { ((rng.unit() * 2.0 - 1.0) * jitter_ns as f64) as i64 } else { 0 };
                    let d = (base_ns + j).max(0);
                    *order += 1;
                    let (lock, cv) = &*queue;
                    lock.lock().unwrap().push(Reverse(Pending { due: Instant::now() + Duration::from_nanos(d as u64), order: *order, bytes }));
                    cv.notify_one();
                    d
                };
                while !stop.load(Ordering::Relaxed) {
                    let Ok((n, from)) = front.recv_from(&mut buf) else { continue };
                    *client.lock().unwrap() = Some(from);
                    let t_in = wall_ns();
                    let bytes = buf[..n].to_vec();
                    let samples = envelopes_in(&bytes);
                    if samples.is_empty() {
                        schedule(bytes, &mut rng, &mut order);
                        continue;
                    }
                    let lost = match profile.gilbert_elliott {
                        Some(ge) => {
                            ge_bad = if ge_bad { rng.unit() >= ge.p_bad_good } else { rng.unit() < ge.p_good_bad };
                            rng.unit() < if ge_bad { ge.loss_bad } else { ge.loss_good }
                        }
                        None => rng.unit() < profile.loss,
                    };
                    let roll = rng.unit();
                    let action = if lost {
                        Action::Drop
                    } else if roll < profile.duplicate {
                        Action::Duplicate
                    } else if roll < profile.duplicate + profile.reorder && held.is_none() {
                        Action::Reorder
                    } else {
                        Action::Forward
                    };
                    let mut log = truth.lock().unwrap();
                    let first = log.len();
                    for &(origin, channel, seq) in &samples {
                        log.push(TruthRecord { origin, channel, seq, action, t_in, delays_ns: Vec::new() });
                    }
                    let idx: Vec<usize> = (first..log.len()).collect();
                    match action {
                        Action::Drop => {}
                        Action::Reorder => {
                            held = Some((bytes, idx));
                            continue;
                        }
                        Action::Duplicate => {
                            let d1 = schedule(bytes.clone(), &mut rng, &mut order);
                            let d2 = schedule(bytes, &mut rng, &mut order);
                            for &i in &idx {
                                log[i].delays_ns = vec![d1, d2];
                            }
                        }
                        Action::Forward => {
                            let d = schedule(bytes, &mut rng, &mut order);
                            for &i in &idx {
                                log[i].delays_ns = vec![d];
                            }
                        }
                    }
                    if action != Action::Drop {
                        if let Some((hb, hidx)) = held.take() {
                            // Release strictly after the datagram just scheduled.
                            let d = schedule(hb, &mut rng, &mut order);
                            for i in hidx {
                                log[i].delays_ns = vec![d];
                            }
                        }
                    }
                }
            })?);
        }
        // Delayed sender for the impaired direction.
        {
            let (back, stop, queue) = (back.clone(), stop.clone(), queue.clone());
            threads.push(std::thread::Builder::new().name("impair-out".into()).spawn(move || {
                let (lock, cv) = &*queue;
                let mut q = lock.lock().unwrap();
                while !stop.load(Ordering::Relaxed) {
                    let now = Instant::now();
                    match q.peek() {
                        Some(Reverse(p)) if p.due <= now => {
                            let Reverse(p) = q.pop().unwrap();
                            drop(q);
                            let _ = back.send(&p.bytes);
                            q = lock.lock().unwrap();
                        }
                        Some(Reverse(p)) => {
                            let wait = p.due - now;
                            q = cv.wait_timeout(q, wait).unwrap().0;
                        }
                        None => q = cv.wait_timeout(q, Duration::from_millis(20)).unwrap().0,
                    }
                }
            })?);
        }
        // target → client: pass through.
        {
            let (front, back, stop, client) = (front.clone(), back.clone(), stop.clone(), client.clone());
            threads.push(std::thread::Builder::new().name("impair-back".into()).spawn(move || {
                let mut buf = vec![0u8; 65_536];
                while !stop.load(Ordering::Relaxed) {
                    let Ok(n) = back.recv(&mut buf) else { continue };
                    if let Some(c) = *client.lock().unwrap() {
                        let _ = front.send_to(&buf[..n], c);
                    }
                }
            })?);
        }
        Ok(Self { listen, stop, threads, truth })
    }

    pub fn truth(&self) -> Vec<TruthRecord> {
        self.truth.lock().unwrap().clone()
    }

    pub fn stop(mut self) -> Vec<TruthRecord> {
        self.stop.store(true, Ordering::Relaxed);
        for t in self.threads.drain(..) {
            let _ = t.join();
        }
        self.truth()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use xgc_rt_core::envelope::{encode, Header};

    #[test]
    fn finds_every_envelope_in_a_batched_datagram() {
        let mut d = vec![0xAAu8; 7];
        for seq in 1..=3u64 {
            d.extend(encode(&Header { channel: 2, origin: 5, seq, ..Header::default() }, &[seq as u8; 40]).unwrap());
            d.extend([0x11u8; 3]);
        }
        assert_eq!(envelopes_in(&d), vec![(5, 2, 1), (5, 2, 2), (5, 2, 3)]);
        d[7 + 20] ^= 1; // corrupt the first envelope's seq: CRC rejects it
        assert_eq!(envelopes_in(&d), vec![(5, 2, 2), (5, 2, 3)]);
    }
}
