//! A software plant with a PX4/MAVROS stand-in for closed-loop tests of
//! ctl-px4: px4_standin.py's position mode (tracking backend px4_local).
//! Velocity command 1.5 (p_sp - p) + v_sp, at most 1.5 m/s; acceleration
//! (v_cmd - v) / 0.3 within 3 m/s^2; attitude along the thrust; arming and
//! set_mode answered at once; ground contact. The estimate is the plant
//! truth. Payloads are the xgc schemas ctl-px4 and plan-dmpc read.

use std::path::Path;

pub const G: f64 = 9.8066;

pub fn read_records(path: &Path) -> Vec<(u64, u32, Vec<u8>)> {
    let bytes = std::fs::read(path).unwrap();
    assert_eq!(&bytes[..8], b"XGCDMPC1");
    let mut out = Vec::new();
    let mut i = 8;
    while i < bytes.len() {
        let round = u64::from_le_bytes(bytes[i..i + 8].try_into().unwrap());
        let port = u32::from_le_bytes(bytes[i + 8..i + 12].try_into().unwrap());
        let len = u32::from_le_bytes(bytes[i + 12..i + 16].try_into().unwrap()) as usize;
        out.push((round, port, bytes[i + 16..i + 16 + len].to_vec()));
        i += 16 + len;
    }
    out
}

pub fn f64s(v: &[f64]) -> Vec<u8> {
    v.iter().flat_map(|x| x.to_le_bytes()).collect()
}

pub fn f64_at(d: &[u8], i: usize) -> f64 {
    f64::from_le_bytes(d[8 * i..8 * i + 8].try_into().unwrap())
}

pub fn cstr(d: &[u8]) -> String {
    let end = d.iter().position(|&b| b == 0).unwrap_or(d.len());
    String::from_utf8_lossy(&d[..end]).to_string()
}


pub fn qmul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

pub fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let r = qmul(qmul(q, [0.0, v[0], v[1], v[2]]), [q[0], -q[1], -q[2], -q[3]]);
    [r[1], r[2], r[3]]
}

pub fn tilt_for(acc: [f64; 3]) -> [f64; 4] {
    let f = [acc[0], acc[1], acc[2] + G];
    let n = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
    if n < 1e-6 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let z = [f[0] / n, f[1] / n, f[2] / n];
    let axis = [-z[1], z[0], 0.0];
    let s = (axis[0] * axis[0] + axis[1] * axis[1]).sqrt();
    if s < 1e-9 {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let ang = s.atan2(z[2]);
    let k = (0.5 * ang).sin() / s;
    [(0.5 * ang).cos(), axis[0] * k, axis[1] * k, 0.0]
}

pub fn rates_between(q0: [f64; 4], q1: [f64; 4], dt: f64) -> [f64; 3] {
    let mut d = qmul([q0[0], -q0[1], -q0[2], -q0[3]], q1);
    if d[0] < 0.0 {
        d = [-d[0], -d[1], -d[2], -d[3]];
    }
    [2.0 * d[1] / dt, 2.0 * d[2] / dt, 2.0 * d[3] / dt]
}

pub struct Plant {
    pub p: [f64; 3],
    pub v: [f64; 3],
    pub a: [f64; 3],
    pub q: [f64; 4], // world <- body, w x y z
    pub w: [f64; 3],
    pub armed: bool,
    pub mode: String,
    pub setpoint: Option<Vec<u8>>, // xgc.position_target/1
}

impl Plant {
    pub fn new(p: [f64; 3]) -> Self {
        Self { p, v: [0.0; 3], a: [0.0; 3], q: [1.0, 0.0, 0.0, 0.0], w: [0.0; 3], armed: false, mode: "POSCTL".into(), setpoint: None }
    }

    pub fn step(&mut self, dt: f64) {
        let offboard = self.armed && self.mode == "OFFBOARD";
        let mut a = [0.0; 3];
        match (&self.setpoint, offboard) {
            (Some(m), true) => {
                let mask = u16::from_le_bytes([m[96], m[97]]);
                let use_pos = mask & (1 | 2 | 4) == 0;
                let use_vel = mask & (8 | 16 | 32) == 0;
                let mut vcmd = [0.0; 3];
                for i in 0..3 {
                    let pos = if use_pos { 1.5 * (f64_at(m, 1 + i) - self.p[i]) } else { 0.0 };
                    vcmd[i] = pos + if use_vel { f64_at(m, 4 + i) } else { 0.0 };
                }
                let norm = (vcmd[0] * vcmd[0] + vcmd[1] * vcmd[1] + vcmd[2] * vcmd[2]).sqrt();
                if norm > 1.5 {
                    for c in &mut vcmd {
                        *c *= 1.5 / norm;
                    }
                }
                for i in 0..3 {
                    a[i] = ((vcmd[i] - self.v[i]) / 0.3).clamp(-3.0, 3.0);
                }
            }
            _ => {
                if self.p[2] > 0.0 {
                    a = [-self.v[0] / 0.2, -self.v[1] / 0.2, -self.v[2] / 0.2];
                }
                if !self.armed {
                    a = [0.0, 0.0, if self.p[2] > 0.0 { -G } else { 0.0 }];
                }
            }
        }
        let flying = offboard && self.setpoint.is_some();
        let q_new = if self.armed && self.p[2] > 0.0 { tilt_for(a) } else { [1.0, 0.0, 0.0, 0.0] };
        self.w = rates_between(self.q, q_new, dt);
        self.q = q_new;
        for i in 0..3 {
            self.v[i] += a[i] * dt;
            self.p[i] += self.v[i] * dt;
        }
        if self.p[2] <= 0.0 {
            self.p[2] = 0.0;
            self.v[2] = self.v[2].max(0.0);
            a[2] = a[2].max(0.0);
            if !flying {
                self.v[0] = 0.0;
                self.v[1] = 0.0;
                a = [0.0; 3];
                self.q = [1.0, 0.0, 0.0, 0.0];
                self.w = [0.0; 3];
            }
        }
        self.a = a;
    }

    pub fn fcu_request(&mut self, d: &[u8]) {
        let kind = u32::from_le_bytes(d[8..12].try_into().unwrap());
        if kind == 1 {
            self.armed = u32::from_le_bytes(d[12..16].try_into().unwrap()) != 0;
        } else if kind == 2 {
            self.mode = cstr(&d[16..48]);
        }
    }

    pub fn estimate(&self, t: f64) -> Vec<u8> {
        let mut v = vec![t];
        v.extend(self.p);
        v.extend(self.v);
        v.extend(self.q);
        v.extend(self.w);
        v.extend(self.a);
        v.extend([0.0, 0.0, -G]);
        v.extend([0.0; 3]);
        v.extend([t, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, t, t, t, t]);
        let mut out = f64s(&v);
        out.extend(0u32.to_le_bytes()); // flags
        out.extend(0u32.to_le_bytes());
        out.extend(0u32.to_le_bytes());
        out.extend([3u8, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0]); // estimator_state RUNNING
        assert_eq!(out.len(), 304);
        out
    }

    pub fn imu(&self, t: f64) -> Vec<u8> {
        let qi = [self.q[0], -self.q[1], -self.q[2], -self.q[3]];
        let f = rotate(qi, [self.a[0], self.a[1], self.a[2] + G]);
        f64s(&[t, f[0], f[1], f[2], self.w[0], self.w[1], self.w[2]])
    }

    pub fn pose(&self, t: f64) -> Vec<u8> {
        f64s(&[t, self.p[0], self.p[1], self.p[2], self.q[0], self.q[1], self.q[2], self.q[3]])
    }

    pub fn twist(&self, t: f64) -> Vec<u8> {
        f64s(&[t, self.v[0], self.v[1], self.v[2], 0.0, 0.0, 0.0])
    }

    pub fn fcu_state(&self, t: f64) -> Vec<u8> {
        let mut out = f64s(&[t]);
        out.extend([1u8, self.armed as u8, 1, 0, 0, 0, 0, 0]);
        let mut mode = [0u8; 32];
        mode[..self.mode.len().min(31)].copy_from_slice(&self.mode.as_bytes()[..self.mode.len().min(31)]);
        out.extend(mode);
        out
    }

    pub fn rigid_state(&self, t: f64) -> Vec<u8> {
        let mut v = vec![t];
        v.extend(self.p);
        v.extend(self.v);
        v.extend(self.q);
        v.extend(self.w);
        f64s(&v)
    }
}

pub fn command(text: &str) -> Vec<u8> {
    let mut out = vec![0u8; 64];
    out[..text.len()].copy_from_slice(text.as_bytes());
    out
}

/// A ground robot (Scout) following the DMPC planner's planar setpoint
/// (xgc.planar_pva/1) directly: velocity command 1.5 (p_sp - p) + v_sp in
/// the plane, at most 1.0 m/s, acceleration (v_cmd - v) / 0.3 within
/// 2 m/s^2; heading along the velocity while moving; height fixed.
pub struct GroundPlant {
    pub p: [f64; 3],
    pub v: [f64; 3],
    pub yaw: f64,
    pub setpoint: Option<Vec<u8>>,
}

impl GroundPlant {
    pub fn new(p: [f64; 3], yaw: f64) -> Self {
        Self { p, v: [0.0; 3], yaw, setpoint: None }
    }

    pub fn step(&mut self, dt: f64) {
        let mut vcmd = [0.0; 2];
        if let Some(m) = &self.setpoint {
            // stamp, x, y, yaw, vx, vy, ax, ay
            vcmd = [1.5 * (f64_at(m, 1) - self.p[0]) + f64_at(m, 4), 1.5 * (f64_at(m, 2) - self.p[1]) + f64_at(m, 5)];
            let norm = (vcmd[0] * vcmd[0] + vcmd[1] * vcmd[1]).sqrt();
            if norm > 1.0 {
                vcmd = [vcmd[0] / norm, vcmd[1] / norm];
            }
        }
        for i in 0..2 {
            let a = ((vcmd[i] - self.v[i]) / 0.3).clamp(-2.0, 2.0);
            self.v[i] += a * dt;
            self.p[i] += self.v[i] * dt;
        }
        if (self.v[0] * self.v[0] + self.v[1] * self.v[1]).sqrt() > 0.05 {
            self.yaw = self.v[1].atan2(self.v[0]);
        }
    }

    pub fn rigid_state(&self, t: f64) -> Vec<u8> {
        let (s, c) = (0.5 * self.yaw).sin_cos();
        f64s(&[t, self.p[0], self.p[1], self.p[2], self.v[0], self.v[1], 0.0, c, 0.0, 0.0, s, 0.0, 0.0, 0.0])
    }
}
