//! JSON bytes the existing Mocap Rotor decoder already accepts.
//! Measurement time stays the native sample stamp. Steady age is separate.

use serde_json::{json, Value};

pub const POSE_MAX: usize = 4096;
pub const TWIST_MAX: usize = 4096;
pub const IMU_MAX: usize = 8192;
pub const POWER_MAX: usize = 2048;
pub const FLIGHT_MAX: usize = 4096;
pub const HEARTBEAT_MAX: usize = 8192;
pub const COMMAND_LEN: usize = 64;
pub const TIMELINE_LEN: usize = 240;

pub const POSE_HZ: f64 = 15.0;
pub const VELOCITY_HZ: f64 = 15.0;
pub const IMU_HZ: f64 = 10.0;
pub const POWER_HZ: f64 = 2.0;
pub const FLIGHT_HZ: f64 = 2.0;
pub const HEARTBEAT_HZ: f64 = 1.0;

#[derive(Clone, Copy)]
pub struct Rates {
    pub local_pose_ns: u64,
    pub local_velocity_ns: u64,
    pub imu_ns: u64,
    pub power_ns: u64,
    pub flight_state_ns: u64,
    pub forwarder_hb_ns: u64,
}

impl Rates {
    pub fn contract() -> Self {
        Self {
            local_pose_ns: interval(POSE_HZ),
            local_velocity_ns: interval(VELOCITY_HZ),
            imu_ns: interval(IMU_HZ),
            power_ns: interval(POWER_HZ),
            flight_state_ns: interval(FLIGHT_HZ),
            forwarder_hb_ns: interval(HEARTBEAT_HZ),
        }
    }
}

pub fn interval(hz: f64) -> u64 {
    (1_000_000_000.0 / hz) as u64
}

pub fn parse_rate(value: &toml::Value, ceiling: f64) -> Result<u64, String> {
    let hz = value.as_float().or_else(|| value.as_integer().map(|n| n as f64)).ok_or("rate is not a number")?;
    if !hz.is_finite() || hz <= 0.0 || hz > ceiling {
        return Err(format!("rate {hz} is outside (0, {ceiling}]"));
    }
    Ok(interval(hz))
}

pub fn valid_robot_id(robot_id: &str) -> bool {
    let bytes = robot_id.as_bytes();
    if bytes.len() < 3 || bytes.len() > 128 {
        return false;
    }
    if !bytes[0].is_ascii_lowercase() {
        return false;
    }
    if !bytes[bytes.len() - 1].is_ascii_alphanumeric() {
        return false;
    }
    if !bytes[1..bytes.len() - 1].iter().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || *b == b'-') {
        return false;
    }
    let alias = bytes.len() > 3
        && (robot_id.starts_with("uav") || robot_id.starts_with("ugv"))
        && bytes[3..].iter().all(|b| b.is_ascii_digit());
    !alias
}

pub fn valid_endpoint(endpoint: &str) -> bool {
    let Some(rest) = endpoint.strip_prefix("tcp/") else {
        return false;
    };
    let Some((host, port)) = rest.rsplit_once(':') else {
        return false;
    };
    if host.is_empty() || host.len() > 253 || !host.as_bytes()[0].is_ascii_alphanumeric() {
        return false;
    }
    if !host.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'.' || b == b'-') {
        return false;
    }
    match port.parse::<u16>() {
        Ok(port) => port != 0,
        Err(_) => false,
    }
}

pub fn valid_frame(frame: &str) -> bool {
    let bytes = frame.as_bytes();
    if bytes.len() < 2 || bytes.len() > 128 {
        return false;
    }
    if !bytes[0].is_ascii_alphabetic() || !bytes[bytes.len() - 1].is_ascii_alphanumeric() && bytes[bytes.len() - 1] != b'_' {
        return false;
    }
    if frame.contains("//") || frame.contains("..") {
        return false;
    }
    bytes.iter().all(|b| b.is_ascii_alphanumeric() || matches!(*b, b'_' | b'.' | b'/' | b'-'))
}

/// Control-region name in `xgc_controller_status_v1.state[48]`.
/// Non-empty, at most 48 bytes, no control character (`char::is_control`,
/// which is C0, DEL, and C1).
pub fn valid_controller_state(text: &str) -> bool {
    let bytes = text.as_bytes();
    !bytes.is_empty() && bytes.len() <= 48 && text.chars().all(|c| !c.is_control())
}

const COMMAND_TOKENS: &[&str] = &[
    "prepare", "custom1", "start", "hold", "hover", "stop",
    "takeoff", "Takeoff", "TAKEOFF", "land", "Land", "LAND",
    "Hover", "HOVER", "Custom1", "CUSTOM1", "Start", "START",
    "track", "Track", "TRACK",
];

pub fn valid_command_token(text: &str) -> bool {
    COMMAND_TOKENS.contains(&text)
}

pub fn command_payload(text: &str) -> [u8; COMMAND_LEN] {
    let mut out = [0u8; COMMAND_LEN];
    out[..text.len()].copy_from_slice(text.as_bytes());
    out
}

pub fn timeline_schema_ok(bytes: &[u8]) -> bool {
    bytes.len() == TIMELINE_LEN && u32::from_le_bytes(bytes[0..4].try_into().unwrap()) == 1
}

fn finite3(v: &[f64; 3]) -> bool {
    v.iter().all(|n| n.is_finite())
}

fn quat_ok(q_wxyz: &[f64; 4]) -> bool {
    if q_wxyz.iter().any(|n| !n.is_finite()) {
        return false;
    }
    let n = q_wxyz.iter().map(|c| c * c).sum::<f64>().sqrt();
    n.is_finite() && (0.5..1.5).contains(&n)
}

fn xyzw(q_wxyz: &[f64; 4]) -> Value {
    json!({"x": q_wxyz[1], "y": q_wxyz[2], "z": q_wxyz[3], "w": q_wxyz[0]})
}

fn vec3(v: &[f64; 3]) -> Value {
    json!({"x": v[0], "y": v[1], "z": v[2]})
}

fn stamp_ms(seconds: f64) -> Option<i64> {
    if !seconds.is_finite() || seconds <= 0.0 {
        return None;
    }
    let ms = (seconds * 1000.0).round();
    if ms <= 0.0 || ms > (i64::MAX / 1_000_000) as f64 {
        return None;
    }
    Some(ms as i64)
}

fn dump(value: Value, limit: usize) -> Option<String> {
    let text = serde_json::to_string(&value).ok()?;
    (text.len() <= limit).then_some(text)
}

pub fn pose_json(sequence: u64, stamp_s: f64, frame_id: &str, child_frame_id: &str, position: &[f64; 3], q_wxyz: &[f64; 4]) -> Option<String> {
    let t_ms = stamp_ms(stamp_s)?;
    if sequence == 0 || !finite3(position) || !quat_ok(q_wxyz) {
        return None;
    }
    dump(json!({
        "v": 1, "sequence": sequence, "t_ms": t_ms,
        "frame_id": frame_id, "child_frame_id": child_frame_id,
        "position": vec3(position), "orientation": xyzw(q_wxyz),
    }), POSE_MAX)
}

pub fn twist_json(sequence: u64, stamp_s: f64, frame_id: &str, linear: &[f64; 3], angular: Option<&[f64; 3]>) -> Option<String> {
    let t_ms = stamp_ms(stamp_s)?;
    if sequence == 0 || !finite3(linear) {
        return None;
    }
    let angular = match angular {
        Some(value) if finite3(value) => vec3(value),
        Some(_) => return None,
        None => Value::Null,
    };
    dump(json!({
        "v": 1, "sequence": sequence, "t_ms": t_ms, "frame_id": frame_id,
        "linear": vec3(linear), "angular": angular,
    }), TWIST_MAX)
}

pub fn imu_json(sequence: u64, stamp_s: f64, frame_id: &str, gyro: &[f64; 3], accel: &[f64; 3]) -> Option<String> {
    let t_ms = stamp_ms(stamp_s)?;
    if sequence == 0 || !finite3(gyro) || !finite3(accel) {
        return None;
    }
    dump(json!({
        "v": 1, "sequence": sequence, "t_ms": t_ms, "frame_id": frame_id,
        "orientation": Value::Null,
        "angular_velocity": vec3(gyro),
        "linear_acceleration": vec3(accel),
        "covariance": {"orientation": Value::Null, "angular_velocity": Value::Null, "linear_acceleration": Value::Null},
    }), IMU_MAX)
}

pub fn power_json(sequence: u64, stamp_s: f64, percentage: f64, voltage: f64) -> Option<String> {
    let t_ms = stamp_ms(stamp_s)?;
    if sequence == 0 || !percentage.is_finite() || !(-1.0..=1.0).contains(&percentage) {
        return None;
    }
    let voltage_v = if voltage.is_finite() { json!(voltage) } else { Value::Null };
    dump(json!({
        "v": 1, "sequence": sequence, "t_ms": t_ms,
        "percentage": percentage, "voltage_v": voltage_v,
        "current_a": Value::Null, "temperature_c": Value::Null, "charging": Value::Null,
    }), POWER_MAX)
}

pub fn flight_json(sequence: u64, stamp_s: f64, connected: bool, armed: bool, guided: bool, manual_input: bool, mode: &str, system_status: u32) -> Option<String> {
    let t_ms = stamp_ms(stamp_s)?;
    if sequence == 0 || mode.len() > 64 {
        return None;
    }
    dump(json!({
        "v": 1, "sequence": sequence, "t_ms": t_ms,
        "connected": connected, "armed": armed, "guided": guided, "manual_input": manual_input,
        "mode": mode, "system_status": system_status, "landed_state": Value::Null, "faults": [],
    }), FLIGHT_MAX)
}

pub struct HeartbeatChannel {
    pub id: &'static str,
    pub source_samples: u64,
    pub source_age_ms: i64,
    pub ready: bool,
    pub text: Option<String>,
}

pub fn heartbeat_json(robot_id: &str, sequence: u64, t_ms: i64, uptime_ms: i64, channels: &[HeartbeatChannel], publish_success: u64, publish_failure: u64, throttled: u64, rejected_source: u64) -> Option<String> {
    if sequence == 0 || t_ms <= 0 || uptime_ms < 0 {
        return None;
    }
    let channels: Vec<Value> = channels.iter().map(|c| {
        let mut item = json!({
            "id": c.id, "source_samples": c.source_samples, "source_age_ms": c.source_age_ms, "ready": c.ready,
        });
        if let Some(text) = &c.text {
            item["text"] = json!(text);
        }
        item
    }).collect();
    dump(json!({
        "v": 1, "sequence": sequence, "t_ms": t_ms, "robot_id": robot_id,
        "transport": "zenoh", "uptime_ms": uptime_ms, "channels": channels,
        "stats": {"publish_success": publish_success, "publish_failure": publish_failure, "throttled": throttled, "rejected_source": rejected_source},
    }), HEARTBEAT_MAX)
}

fn f64_at(data: &[u8], offset: usize) -> f64 {
    let mut buf = [0u8; 8];
    buf.copy_from_slice(&data[offset..offset + 8]);
    f64::from_le_bytes(buf)
}

fn f64_3(data: &[u8], offset: usize) -> [f64; 3] {
    [f64_at(data, offset), f64_at(data, offset + 8), f64_at(data, offset + 16)]
}

#[derive(Clone, Copy)]
pub struct NativePose {
    pub stamp_s: f64,
    pub position: [f64; 3],
    pub q_wxyz: [f64; 4],
}

pub fn parse_pose(data: &[u8]) -> Option<NativePose> {
    if data.len() != 64 {
        return None;
    }
    Some(NativePose {
        stamp_s: f64_at(data, 0),
        position: f64_3(data, 8),
        q_wxyz: [f64_at(data, 32), f64_at(data, 40), f64_at(data, 48), f64_at(data, 56)],
    })
}

#[derive(Clone, Copy)]
pub struct NativeTwist {
    pub stamp_s: f64,
    pub linear: [f64; 3],
    pub angular: [f64; 3],
}

pub fn parse_twist(data: &[u8]) -> Option<NativeTwist> {
    if data.len() != 56 {
        return None;
    }
    Some(NativeTwist { stamp_s: f64_at(data, 0), linear: f64_3(data, 8), angular: f64_3(data, 32) })
}

#[derive(Clone, Copy)]
pub struct NativeImu {
    pub stamp_s: f64,
    pub accel: [f64; 3],
    pub gyro: [f64; 3],
}

pub fn parse_imu(data: &[u8]) -> Option<NativeImu> {
    if data.len() != 56 {
        return None;
    }
    Some(NativeImu { stamp_s: f64_at(data, 0), accel: f64_3(data, 8), gyro: f64_3(data, 32) })
}

#[derive(Clone, Copy)]
pub struct NativeBattery {
    pub stamp_s: f64,
    pub voltage: f64,
    pub percentage: f64,
}

pub fn parse_battery(data: &[u8]) -> Option<NativeBattery> {
    if data.len() != 24 {
        return None;
    }
    Some(NativeBattery { stamp_s: f64_at(data, 0), voltage: f64_at(data, 8), percentage: f64_at(data, 16) })
}

#[derive(Clone)]
pub struct NativeFlight {
    pub stamp_s: f64,
    pub connected: bool,
    pub armed: bool,
    pub guided: bool,
    pub manual_input: bool,
    pub system_status: u32,
    pub mode: String,
}

pub fn parse_flight(data: &[u8]) -> Option<NativeFlight> {
    if data.len() != 48 {
        return None;
    }
    let mode = cstr(&data[16..48])?.to_string();
    Some(NativeFlight {
        stamp_s: f64_at(data, 0),
        connected: data[8] != 0,
        armed: data[9] != 0,
        guided: data[10] != 0,
        manual_input: data[11] != 0,
        system_status: u32::from(data[12]),
        mode,
    })
}

#[derive(Clone, Copy)]
pub struct NativePaired {
    pub pose_stamp_s: f64,
    pub twist_stamp_s: f64,
    pub position: [f64; 3],
    pub q_xyzw: [f64; 4],
    pub linear: [f64; 3],
}

pub fn parse_paired(data: &[u8]) -> Option<NativePaired> {
    if data.len() != 96 {
        return None;
    }
    let pose_stamp_s = f64_at(data, 0);
    let twist_stamp_s = f64_at(data, 8);
    if pose_stamp_s != twist_stamp_s {
        return None;
    }
    Some(NativePaired {
        pose_stamp_s,
        twist_stamp_s,
        position: f64_3(data, 16),
        q_xyzw: [f64_at(data, 40), f64_at(data, 48), f64_at(data, 56), f64_at(data, 64)],
        linear: f64_3(data, 72),
    })
}

pub fn parse_status(data: &[u8]) -> Option<(f64, String)> {
    if data.len() != 56 {
        return None;
    }
    let stamp_s = f64_at(data, 0);
    if !stamp_s.is_finite() || stamp_s <= 0.0 {
        return None;
    }
    let name = cstr(&data[8..56])?.to_string();
    valid_controller_state(&name).then_some((stamp_s, name))
}

pub fn cstr(bytes: &[u8]) -> Option<&str> {
    let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
    let text = std::str::from_utf8(&bytes[..end]).ok()?;
    if text.chars().any(|c| c.is_control()) {
        return None;
    }
    Some(text)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn instance_id_is_a_wire_robot_and_uav1_is_not() {
        assert!(valid_robot_id("xgc2e-0123456789abcdef0123"));
        assert!(!valid_robot_id("uav1"));
        assert!(!valid_robot_id("ugv12"));
        assert!(!valid_robot_id("UAV1"));
    }

    #[test]
    fn numeric_vehicle_tokens_pass_and_arm_does_not() {
        for token in ["prepare", "custom1", "start", "hold", "hover", "stop"] {
            assert!(valid_command_token(token), "{token}");
            let packed = command_payload(token);
            assert_eq!(packed.len(), 64);
            assert!(packed[token.len()..].iter().all(|b| *b == 0));
        }
        assert!(!valid_command_token("arm"));
        assert!(!valid_command_token("Arm"));
        assert!(!valid_command_token("disarm"));
        assert!(!valid_command_token("stop-land"));
        for (token, other) in [("prepare", "start"), ("hold", "hover"), ("stop", "land")] {
            let packed = command_payload(token);
            assert_eq!(&packed[..token.len()], token.as_bytes());
            assert_ne!(packed, command_payload(other));
        }
        for token in ["takeoff", "land", "hover", "custom1", "start", "track"] {
            assert!(valid_command_token(token), "{token}");
            assert_eq!(&command_payload(token)[..token.len()], token.as_bytes());
        }
    }

    #[test]
    fn paired_state_is_not_a_pose_struct() {
        assert!(parse_paired(&[0u8; 64]).is_none());
        let mut raw = [0u8; 96];
        raw[0..8].copy_from_slice(&1.25f64.to_le_bytes());
        raw[8..16].copy_from_slice(&1.25f64.to_le_bytes());
        raw[16..24].copy_from_slice(&4.0f64.to_le_bytes());
        raw[64..72].copy_from_slice(&1.0f64.to_le_bytes());
        raw[72..80].copy_from_slice(&0.5f64.to_le_bytes());
        let paired = parse_paired(&raw).unwrap();
        let q_wxyz = [paired.q_xyzw[3], paired.q_xyzw[0], paired.q_xyzw[1], paired.q_xyzw[2]];
        let text = pose_json(1, paired.pose_stamp_s, "world", "base_link", &paired.position, &q_wxyz).unwrap();
        let v: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(v["t_ms"], 1250);
        assert_eq!(v["position"]["x"], 4.0);
        assert_eq!(v["orientation"]["w"], 1.0);
        assert!(parse_pose(&raw).is_none());
    }

    #[test]
    fn pose_stamp_stays_session_milliseconds() {
        let text = pose_json(1, 12.5, "world", "base_link", &[1.0, 2.0, 3.0], &[1.0, 0.0, 0.0, 0.0]).unwrap();
        let v: Value = serde_json::from_str(&text).unwrap();
        assert_eq!(v["t_ms"], 12500);
        assert_eq!(v["orientation"]["w"], 1.0);
        assert!(pose_json(1, 0.0, "world", "base_link", &[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0, 0.0]).is_none());
    }

    #[test]
    fn paired_twist_and_power_leave_unknown_fields_null() {
        let twist = twist_json(1, 1.5, "base_link", &[0.5, 0.0, 0.0], None).unwrap();
        let twist_value: Value = serde_json::from_str(&twist).unwrap();
        assert!(twist_value["angular"].is_null());
        assert_eq!(twist_value["linear"]["x"], 0.5);
        let measured = twist_json(1, 1.5, "base_link", &[0.5, 0.0, 0.0], Some(&[0.0, 0.0, 0.1])).unwrap();
        let measured_value: Value = serde_json::from_str(&measured).unwrap();
        assert_eq!(measured_value["angular"]["z"], 0.1);
        let power = power_json(1, 1.0, 0.4, 15.5).unwrap();
        let power_value: Value = serde_json::from_str(&power).unwrap();
        assert!(power_value["charging"].is_null());
        assert!(power_value["current_a"].is_null());
        assert!(power_value["temperature_c"].is_null());
        assert!(!power.contains("\"charging\":false"));
    }

    #[test]
    fn imu_without_orientation_does_not_invent_covariance() {
        let text = imu_json(1, 2.0, "base_link", &[0.0, 0.0, 0.25], &[0.0, 0.0, 9.81]).unwrap();
        let v: Value = serde_json::from_str(&text).unwrap();
        assert!(v["orientation"].is_null());
        assert!(v["covariance"]["orientation"].is_null());
        assert!(v["covariance"]["angular_velocity"].is_null());
        assert!(v["covariance"]["linear_acceleration"].is_null());
        assert_eq!(v["angular_velocity"]["z"], 0.25);
        assert_eq!(v["linear_acceleration"]["z"], 9.81);
        assert!(!text.contains("[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]"));
    }

    #[test]
    fn controller_status_passes_through_schema_text() {
        let mut raw = [0u8; 56];
        raw[..8].copy_from_slice(&1.5f64.to_le_bytes());
        for name in ["Ready", "SelfCheck", "Hover", "Landing", "Takeoff"] {
            raw[8..56].fill(0);
            raw[8..8 + name.len()].copy_from_slice(name.as_bytes());
            assert_eq!(parse_status(&raw).unwrap().1, name);
        }
        raw[8..56].fill(b'A');
        assert_eq!(parse_status(&raw).unwrap().1.len(), 48);
        assert!(valid_controller_state(&"A".repeat(48)));
        raw[8..56].fill(0);
        assert!(parse_status(&raw).is_none());
        assert!(!valid_controller_state(""));
        raw[8] = 0xC2;
        raw[9] = 0x85;
        assert!(parse_status(&raw).is_none(), "C1 U+0085");
        assert!(!valid_controller_state("\u{0085}"));
        assert!(!valid_controller_state(&"A".repeat(49)));
        let flight = flight_json(1, 1.0, true, false, false, false, "POSCTL", 4).unwrap();
        let body: Value = serde_json::from_str(&flight).unwrap();
        assert!(body["landed_state"].is_null());
    }
}
