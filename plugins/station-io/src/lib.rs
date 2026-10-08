//! Station edge. Telemetry leaves on a private Zenoh client.
//! Command and mission IPC are accepted only when this process is the frozen
//! authority and that capability is explicitly configured.

mod socket;
mod uplink;
mod wire;

use std::path::PathBuf;

use xgc_rt_abi::*;

use socket::{Control, Request};
use uplink::Uplink;
use wire::Rates;

const LOCAL_POSE: u32 = 0;
const LOCAL_VELOCITY: u32 = 1;
const IMU: u32 = 2;
const BATTERY: u32 = 3;
const FCU_STATE: u32 = 4;
const CONTROLLER_STATUS: u32 = 5;
const COMMAND: u32 = 6;
const MISSION_REQUEST: u32 = 7;
const PAIRED_STATE: u32 = 8;

struct Config {
    robot_id: String,
    zenoh_connect: String,
    command_socket: PathBuf,
    authority: bool,
    command: bool,
    mission: bool,
    frame_id: String,
    child_frame_id: String,
    rates: Rates,
}

pub struct StationIo {
    host: Host,
    config: Option<Config>,
    uplink: Option<Uplink>,
    socket: Option<Control>,
    authority: bool,
}

fn required_str(table: &toml::Table, key: &str) -> Result<String, String> {
    table
        .get(key)
        .and_then(|v| v.as_str())
        .map(str::to_owned)
        .ok_or_else(|| format!("{key} is required"))
}

fn required_bool(table: &toml::Table, key: &str) -> Result<bool, String> {
    table
        .get(key)
        .and_then(|v| v.as_bool())
        .ok_or_else(|| format!("{key} capability is required"))
}

fn configure_text(text: &str) -> Result<Config, String> {
    let value: toml::Value = text.parse().map_err(|e| format!("config: {e}"))?;
    let table = value.as_table().ok_or("config must be a table")?;
    for key in table.keys() {
        if !matches!(
            key.as_str(),
            "robot_id"
                | "zenoh_connect"
                | "command_socket"
                | "authority"
                | "command"
                | "mission"
                | "frame_id"
                | "child_frame_id"
                | "rate_hz"
        ) {
            return Err(format!("unknown config key {key}"));
        }
    }
    let robot_id = required_str(table, "robot_id")?;
    if !wire::valid_robot_id(&robot_id) {
        return Err("robot_id must be the experiment instance id, not uavN or ugvN".into());
    }
    let zenoh_connect = required_str(table, "zenoh_connect")?;
    if !wire::valid_endpoint(&zenoh_connect) {
        return Err("zenoh_connect must be one tcp/host:port endpoint".into());
    }
    let command_socket = PathBuf::from(required_str(table, "command_socket")?);
    let authority = table
        .get("authority")
        .and_then(|v| v.as_bool())
        .ok_or("authority is required")?;
    let command = required_bool(table, "command")?;
    let mission = required_bool(table, "mission")?;
    if !authority && (command || mission) {
        return Err("non-authority node cannot enable command or mission".into());
    }
    let frame_id = table
        .get("frame_id")
        .and_then(|v| v.as_str())
        .unwrap_or("world")
        .to_string();
    let child_frame_id = table
        .get("child_frame_id")
        .and_then(|v| v.as_str())
        .unwrap_or("base_link")
        .to_string();
    if !wire::valid_frame(&frame_id) || !wire::valid_frame(&child_frame_id) {
        return Err("frame_id or child_frame_id is not canonical".into());
    }
    let mut rates = Rates::contract();
    if let Some(rate) = table.get("rate_hz") {
        let rate = rate.as_table().ok_or("rate_hz must be a table")?;
        for key in rate.keys() {
            if !matches!(
                key.as_str(),
                "local_pose" | "local_velocity" | "imu" | "power" | "flight_state" | "forwarder_hb"
            ) {
                return Err(format!("unknown rate {key}"));
            }
        }
        let apply = |name: &str, ceiling: f64, slot: &mut u64| -> Result<(), String> {
            if let Some(value) = rate.get(name) {
                *slot = wire::parse_rate(value, ceiling)?;
            }
            Ok(())
        };
        apply("local_pose", wire::POSE_HZ, &mut rates.local_pose_ns)?;
        apply(
            "local_velocity",
            wire::VELOCITY_HZ,
            &mut rates.local_velocity_ns,
        )?;
        apply("imu", wire::IMU_HZ, &mut rates.imu_ns)?;
        apply("power", wire::POWER_HZ, &mut rates.power_ns)?;
        apply("flight_state", wire::FLIGHT_HZ, &mut rates.flight_state_ns)?;
        apply(
            "forwarder_hb",
            wire::HEARTBEAT_HZ,
            &mut rates.forwarder_hb_ns,
        )?;
    }
    Ok(Config {
        robot_id,
        zenoh_connect,
        command_socket,
        authority,
        command,
        mission,
        frame_id,
        child_frame_id,
        rates,
    })
}

fn latest<const N: usize>(host: &mut Host, port: u32) -> Option<Result<[u8; N], ()>> {
    // next() invalidates the previous borrowed sample, including when draining
    // the port reaches its end. Keep one fixed owned buffer while conflating;
    // never allocate a Vec for each sample that will immediately be replaced.
    let mut bytes = [0; N];
    let mut seen = false;
    let mut valid = false;
    while let Some(sample) = host.next(port) {
        seen = true;
        valid = sample.data.len() == N;
        if valid {
            bytes.copy_from_slice(sample.data);
        }
    }
    // An invalid final sample must still reject the batch, rather than reuse
    // an older valid state. No sample and invalid sample are distinct outcomes.
    seen.then_some(if valid { Ok(bytes) } else { Err(()) })
}

impl Plugin for StationIo {
    fn create(host: Host) -> Self {
        Self {
            host,
            config: None,
            uplink: None,
            socket: None,
            authority: false,
        }
    }

    fn configure(&mut self, config: &str) -> Result<(), String> {
        if self.uplink.is_some() {
            return Err("configure after activate".into());
        }
        let config = configure_text(config)?;
        self.authority = config.authority;
        self.host.log(
            XGC_LOG_INFO,
            &format!(
                "station-io robot_id={} authority={} zenoh_connect={}",
                config.robot_id, config.authority, config.zenoh_connect
            ),
        );
        self.config = Some(config);
        Ok(())
    }

    fn activate(&mut self) -> Result<(), String> {
        let config = self.config.as_ref().ok_or("activate without configure")?;
        if self.socket.is_some() || self.uplink.is_some() {
            return Err("station activation still owns resources".into());
        }
        let api = self
            .host
            .rpc_runtime_api()
            .ok_or("station activate requires injected XRPC process owner (Host ABI minor 3)")?;
        // The loader keeps this scoped C factory and the actual module code pin
        // alive during activate. ForeignRuntime retains that origin through its
        // C table; no Rust Runtime or Arc layout crosses the DSO boundary.
        let runtime = unsafe { xgc2_xrpc::ffi::ForeignRuntime::from_api(api.cast()) }
            .map_err(|e| format!("station XRPC owner: {e}"))?;
        self.uplink = Some(Uplink::start(
            &config.robot_id,
            &config.zenoh_connect,
            config.rates,
            &config.frame_id,
            &config.child_frame_id,
        )?);
        self.socket = Some(Control::bind(
            &runtime,
            &config.command_socket,
            &instance_id()?,
            config.authority,
            config.command,
            config.mission,
        )?);
        Ok(())
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        {
            let uplink = self.uplink.as_ref().ok_or("step before activate")?;
            if let Some(bytes) = latest::<64>(&mut self.host, LOCAL_POSE) {
                match bytes.ok().and_then(|bytes| wire::parse_pose(&bytes)) {
                    Some(sample) => uplink.pose(sample),
                    None => uplink.reject(),
                }
            }
            if let Some(bytes) = latest::<56>(&mut self.host, LOCAL_VELOCITY) {
                match bytes.ok().and_then(|bytes| wire::parse_twist(&bytes)) {
                    Some(sample) => uplink.twist(sample),
                    None => uplink.reject(),
                }
            }
            if let Some(bytes) = latest::<56>(&mut self.host, IMU) {
                match bytes.ok().and_then(|bytes| wire::parse_imu(&bytes)) {
                    Some(sample) => uplink.imu(sample),
                    None => uplink.reject(),
                }
            }
            if let Some(bytes) = latest::<24>(&mut self.host, BATTERY) {
                match bytes.ok().and_then(|bytes| wire::parse_battery(&bytes)) {
                    Some(sample) => uplink.battery(sample),
                    None => uplink.reject(),
                }
            }
            if let Some(bytes) = latest::<48>(&mut self.host, FCU_STATE) {
                match bytes.ok().and_then(|bytes| wire::parse_flight(&bytes)) {
                    Some(sample) => uplink.flight(sample),
                    None => uplink.reject(),
                }
            }
            if let Some(bytes) = latest::<96>(&mut self.host, PAIRED_STATE) {
                match bytes.ok().and_then(|bytes| wire::parse_paired(&bytes)) {
                    Some(sample) => uplink.paired(sample),
                    None => uplink.reject(),
                }
            }
            if let Some(bytes) = latest::<56>(&mut self.host, CONTROLLER_STATUS) {
                match bytes.ok().and_then(|bytes| wire::parse_status(&bytes)) {
                    Some((stamp_s, name)) => uplink.controller(stamp_s, &name),
                    None => uplink.reject(),
                }
            }
        }
        let Some(pending) = self.socket.as_mut().and_then(Control::poll) else {
            return Ok(());
        };
        // The copied absolute deadline discards an uncommitted handoff. The
        // foreign factory exposes no caller-abort token. Deadline/cancellation
        // can race this local publication, which cannot be rolled back or replayed.
        if pending.expired() {
            return Ok(());
        }
        let published = match &pending.request {
            Request::Command(payload) => self
                .host
                .publish(COMMAND, ctx.round, payload)
                .map_err(|_| "command publish failed"),
            Request::Mission(payload) => self
                .host
                .publish(MISSION_REQUEST, ctx.round, payload)
                .map_err(|_| "mission_request publish failed"),
        };
        pending.complete(published);
        Ok(())
    }

    fn deactivate(&mut self) -> Result<(), String> {
        if let Some(socket) = self.socket.as_mut() {
            socket.close()?;
        }
        self.socket = None;
        self.uplink = None;
        Ok(())
    }

    fn domain_state(&self) -> &'static std::ffi::CStr {
        if self.authority {
            cstr!("authority")
        } else {
            cstr!("telemetry")
        }
    }
}

fn instance_id() -> Result<String, String> {
    // Each actual station activation owns its incarnation, even when several
    // station modules share one aggregate process. No mutable process env.
    xgc2_xrpc::new_instance_id().map_err(|e| e.to_string())
}

pub fn submit_command(path: &std::path::Path, token: &str) -> Result<(), String> {
    if !wire::valid_command_token(token) {
        return Err("command token is not a controller string".into());
    }
    socket::command(path, "", token)
}

pub fn submit_mission_file(
    socket_path: &std::path::Path,
    file: &std::path::Path,
) -> Result<(), String> {
    use std::{io::Read, os::unix::fs::OpenOptionsExt};
    let input = std::fs::OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NOFOLLOW | libc::O_NONBLOCK)
        .open(file)
        .map_err(|e| e.to_string())?;
    if !input.metadata().map_err(|e| e.to_string())?.is_file() {
        return Err("mission must be a regular file".into());
    }
    let mut bytes = Vec::with_capacity(241);
    input
        .take(241)
        .read_to_end(&mut bytes)
        .map_err(|e| e.to_string())?;
    if !wire::timeline_schema_ok(&bytes) {
        return Err("mission file must be 240 bytes with schema 1".into());
    }
    socket::mission(socket_path, "", &bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::ffi::{c_char, c_void};

    struct InputFixture {
        pending: VecDeque<Vec<u8>>,
        borrowed: [u8; 96],
    }

    unsafe extern "C" fn fixture_next(
        raw: *mut c_void,
        _port: u32,
        view: *mut XgcSampleView,
    ) -> XgcStatus {
        let input = unsafe { &mut *raw.cast::<InputFixture>() };
        // Exercise the public ABI lifetime even on the terminal next() call.
        input.borrowed.fill(0xcd);
        let Some(bytes) = input.pending.pop_front() else {
            return XGC_ERR_AGAIN;
        };
        input.borrowed[..bytes.len()].copy_from_slice(&bytes);
        unsafe {
            (*view).data = input.borrowed.as_ptr();
            (*view).len = bytes.len() as u32;
        }
        XGC_OK
    }

    unsafe extern "C" fn fixture_publish(
        _: *mut c_void,
        _: u32,
        _: u64,
        _: *const u8,
        _: u32,
    ) -> XgcStatus {
        XGC_ERR_INVALID
    }
    unsafe extern "C" fn fixture_now(_: *mut c_void) -> i64 {
        0
    }
    unsafe extern "C" fn fixture_log(_: *mut c_void, _: XgcLogLevel, _: *const c_char) {}
    unsafe extern "C" fn fixture_degrade(_: *mut c_void, _: *const c_char) {}
    unsafe extern "C" fn fixture_recover(_: *mut c_void) {}
    unsafe extern "C" fn fixture_origins(_: *mut c_void, _: u32, _: *mut u16, _: u32) -> u32 {
        0
    }
    unsafe extern "C" fn fixture_node(_: *mut c_void) -> u16 {
        0
    }

    fn drain_fixture<const N: usize>(samples: Vec<Vec<u8>>) -> Option<Result<[u8; N], ()>> {
        let mut input = InputFixture {
            pending: samples.into(),
            borrowed: [0; 96],
        };
        let api = XgcHostApi {
            abi_version: XGC_RT_ABI_VERSION,
            abi_minor: XGC_RT_ABI_MINOR,
            host: (&mut input as *mut InputFixture).cast(),
            publish: fixture_publish,
            next: fixture_next,
            now: fixture_now,
            log: fixture_log,
            request_degrade: fixture_degrade,
            request_recover: fixture_recover,
            port_origins: fixture_origins,
            node_id: fixture_node,
            rpc_runtime: None,
        };
        let mut host = unsafe { Host::from_raw(&api) };
        latest::<N>(&mut host, LOCAL_POSE)
    }

    #[test]
    fn latest_owns_last_sample_before_terminal_next_invalidates_the_borrow() {
        assert_eq!(
            drain_fixture::<64>(vec![vec![1; 64], vec![2; 64]]),
            Some(Ok([2; 64]))
        );
    }

    #[test]
    fn latest_distinguishes_empty_port_and_invalid_final_sample() {
        assert_eq!(drain_fixture::<64>(vec![]), None);
        assert_eq!(
            drain_fixture::<64>(vec![vec![1; 64], vec![2; 63]]),
            Some(Err(()))
        );
        assert_eq!(
            drain_fixture::<64>(vec![vec![1; 63], vec![2; 64]]),
            Some(Ok([2; 64]))
        );
    }

    #[test]
    fn activation_without_injected_process_owner_fails_before_io() {
        let mut input = InputFixture {
            pending: VecDeque::new(),
            borrowed: [0; 96],
        };
        let api = XgcHostApi {
            abi_version: XGC_RT_ABI_VERSION,
            abi_minor: XGC_RT_ABI_MINOR,
            host: (&mut input as *mut InputFixture).cast(),
            publish: fixture_publish,
            next: fixture_next,
            now: fixture_now,
            log: fixture_log,
            request_degrade: fixture_degrade,
            request_recover: fixture_recover,
            port_origins: fixture_origins,
            node_id: fixture_node,
            rpc_runtime: None,
        };
        let mut station = StationIo::create(unsafe { Host::from_raw(&api) });
        station.configure(&base(false, false, false)).unwrap();
        let error = station.activate().unwrap_err();
        assert!(error.contains("injected XRPC process owner"), "{error}");
        assert!(station.socket.is_none());
        assert!(station.uplink.is_none());
    }

    fn base(authority: bool, command: bool, mission: bool) -> String {
        format!(
            "robot_id=\"xgc2e-0123456789abcdef0123\"\nzenoh_connect=\"tcp/127.0.0.1:7457\"\ncommand_socket=\"/tmp/station-io.sock\"\nauthority={authority}\ncommand={command}\nmission={mission}\n"
        )
    }

    #[test]
    fn capability_is_explicit_and_non_authority_cannot_claim_it() {
        assert!(configure_text("robot_id=\"xgc2e-0123456789abcdef0123\"\nzenoh_connect=\"tcp/127.0.0.1:7457\"\ncommand_socket=\"/tmp/station-io.sock\"\nauthority=true\n").is_err());
        let enabled = configure_text(&base(true, true, false)).unwrap();
        assert!(enabled.command && !enabled.mission && enabled.authority);
        match configure_text(&base(false, true, false)) {
            Err(error) => assert!(error.contains("non-authority"), "{error}"),
            Ok(_) => panic!("non-authority command capability was accepted"),
        }
        assert!(!configure_text(&base(false, false, false)).unwrap().command);
    }
}

export_plugin! {
    plugin: StationIo,
    name: "station-io",
    version: "0.1.0",
    ports: [
        ("local_pose", XGC_PORT_IN_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE),
        ("local_velocity", XGC_PORT_IN_OPTIONAL, "xgc.twist/1", XGC_QOS_STATE),
        ("imu", XGC_PORT_IN_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE),
        ("battery", XGC_PORT_IN_OPTIONAL, "xgc.battery/1", XGC_QOS_STATE),
        ("fcu_state", XGC_PORT_IN_OPTIONAL, "xgc.fcu_state/1", XGC_QOS_STATE),
        ("controller_status", XGC_PORT_IN_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE),
        ("command", XGC_PORT_OUT_OPTIONAL, "xgc.command/1", XGC_QOS_EVENT),
        ("mission_request", XGC_PORT_OUT_OPTIONAL, "xgc.dmpc.mission_timeline/1", XGC_QOS_EVENT),
        ("paired_state", XGC_PORT_IN_OPTIONAL, "xgc.dmpc.paired_state/1", XGC_QOS_STATE),
    ],
}
