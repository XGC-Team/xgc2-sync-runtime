//! Station edge. Telemetry leaves on a private Zenoh client.
//! Command and mission IPC are accepted only when this process is the frozen
//! authority and that capability is explicitly configured.

mod socket;
mod uplink;
mod wire;

use std::path::PathBuf;

use xgc_rt_abi::*;

use socket::{Listener, Request};
use uplink::Uplink;
use wire::{Rates, TIMELINE_LEN};

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
    socket: Option<Listener>,
    authority: bool,
}

fn required_str(table: &toml::Table, key: &str) -> Result<String, String> {
    table.get(key).and_then(|v| v.as_str()).map(str::to_owned).ok_or_else(|| format!("{key} is required"))
}

fn required_bool(table: &toml::Table, key: &str) -> Result<bool, String> {
    table.get(key).and_then(|v| v.as_bool()).ok_or_else(|| format!("{key} capability is required"))
}

fn configure_text(text: &str) -> Result<Config, String> {
    let value: toml::Value = text.parse().map_err(|e| format!("config: {e}"))?;
    let table = value.as_table().ok_or("config must be a table")?;
    for key in table.keys() {
        if !matches!(
            key.as_str(),
            "robot_id" | "zenoh_connect" | "command_socket" | "authority" | "command" | "mission" | "frame_id" | "child_frame_id" | "rate_hz"
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
    let authority = table.get("authority").and_then(|v| v.as_bool()).ok_or("authority is required")?;
    let command = required_bool(table, "command")?;
    let mission = required_bool(table, "mission")?;
    if !authority && (command || mission) {
        return Err("non-authority node cannot enable command or mission".into());
    }
    let frame_id = table.get("frame_id").and_then(|v| v.as_str()).unwrap_or("world").to_string();
    let child_frame_id = table.get("child_frame_id").and_then(|v| v.as_str()).unwrap_or("base_link").to_string();
    if !wire::valid_frame(&frame_id) || !wire::valid_frame(&child_frame_id) {
        return Err("frame_id or child_frame_id is not canonical".into());
    }
    let mut rates = Rates::contract();
    if let Some(rate) = table.get("rate_hz") {
        let rate = rate.as_table().ok_or("rate_hz must be a table")?;
        for key in rate.keys() {
            if !matches!(key.as_str(), "local_pose" | "local_velocity" | "imu" | "power" | "flight_state" | "forwarder_hb") {
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
        apply("local_velocity", wire::VELOCITY_HZ, &mut rates.local_velocity_ns)?;
        apply("imu", wire::IMU_HZ, &mut rates.imu_ns)?;
        apply("power", wire::POWER_HZ, &mut rates.power_ns)?;
        apply("flight_state", wire::FLIGHT_HZ, &mut rates.flight_state_ns)?;
        apply("forwarder_hb", wire::HEARTBEAT_HZ, &mut rates.forwarder_hb_ns)?;
    }
    Ok(Config { robot_id, zenoh_connect, command_socket, authority, command, mission, frame_id, child_frame_id, rates })
}

fn latest(host: &mut Host, port: u32) -> Option<Vec<u8>> {
    let mut last = None;
    while let Some(sample) = host.next(port) {
        last = Some(sample.data.to_vec());
    }
    last
}

impl Plugin for StationIo {
    fn create(host: Host) -> Self {
        Self { host, config: None, uplink: None, socket: None, authority: false }
    }

    fn configure(&mut self, config: &str) -> Result<(), String> {
        if self.uplink.is_some() {
            return Err("configure after activate".into());
        }
        let config = configure_text(config)?;
        self.authority = config.authority;
        self.host.log(XGC_LOG_INFO, &format!("station-io robot_id={} authority={} zenoh_connect={}", config.robot_id, config.authority, config.zenoh_connect));
        self.config = Some(config);
        Ok(())
    }

    fn activate(&mut self) -> Result<(), String> {
        let config = self.config.as_ref().ok_or("activate without configure")?;
        let socket = Listener::bind(&config.command_socket)?;
        let uplink = Uplink::start(&config.robot_id, &config.zenoh_connect, config.rates, &config.frame_id, &config.child_frame_id)?;
        self.socket = Some(socket);
        self.uplink = Some(uplink);
        Ok(())
    }

    fn step(&mut self, ctx: &XgcStepCtx) -> Result<(), String> {
        {
        let uplink = self.uplink.as_ref().ok_or("step before activate")?;
        if let Some(bytes) = latest(&mut self.host, LOCAL_POSE) {
            match wire::parse_pose(&bytes) {
                Some(sample) => uplink.pose(sample),
                None => uplink.reject(),
            }
        }
        if let Some(bytes) = latest(&mut self.host, LOCAL_VELOCITY) {
            match wire::parse_twist(&bytes) {
                Some(sample) => uplink.twist(sample),
                None => uplink.reject(),
            }
        }
        if let Some(bytes) = latest(&mut self.host, IMU) {
            match wire::parse_imu(&bytes) {
                Some(sample) => uplink.imu(sample),
                None => uplink.reject(),
            }
        }
        if let Some(bytes) = latest(&mut self.host, BATTERY) {
            match wire::parse_battery(&bytes) {
                Some(sample) => uplink.battery(sample),
                None => uplink.reject(),
            }
        }
        if let Some(bytes) = latest(&mut self.host, FCU_STATE) {
            match wire::parse_flight(&bytes) {
                Some(sample) => uplink.flight(sample),
                None => uplink.reject(),
            }
        }
        if let Some(bytes) = latest(&mut self.host, PAIRED_STATE) {
            match wire::parse_paired(&bytes) {
                Some(sample) => uplink.paired(sample),
                None => uplink.reject(),
            }
        }
        if let Some(bytes) = latest(&mut self.host, CONTROLLER_STATUS) {
            match wire::parse_status(&bytes) {
                Some((stamp_s, name)) => uplink.controller(stamp_s, &name),
                None => uplink.reject(),
            }
        }
        }
        let (command_enabled, mission_enabled) = {
            let config = self.config.as_ref().ok_or("step before configure")?;
            (config.command, config.mission)
        };
        let Some(socket) = self.socket.as_mut() else {
            return Ok(());
        };
        let request = match socket.poll() {
            Ok(request) => request,
            Err(e) => {
                self.host.log(XGC_LOG_WARN, &format!("station-io socket: {e}"));
                return Ok(());
            }
        };
        let Some(request) = request else {
            return Ok(());
        };
        if !self.authority {
            if let Err(e) = socket.reply(false, "not the frozen authority") {
                self.host.log(XGC_LOG_WARN, &format!("station-io reply: {e}"));
            }
            return Ok(());
        }
        let published = if let Request::Command(text) = &request {
            if !command_enabled {
                Err("command capability is not configured".to_string())
            } else {
                let payload = wire::command_payload(text);
                self.host.publish(COMMAND, ctx.round, &payload).map_err(|_| "command publish failed".to_string())
            }
        } else if let Request::Mission(bytes) = &request {
            if !mission_enabled {
                Err("mission capability is not configured".to_string())
            } else if bytes.len() != TIMELINE_LEN {
                Err(format!("mission length {TIMELINE_LEN}"))
            } else {
                self.host.publish(MISSION_REQUEST, ctx.round, bytes).map_err(|_| "mission_request publish failed".to_string())
            }
        } else {
            Err("unknown station request".to_string())
        };
        let reply = match published {
            Ok(()) => socket.reply(true, "queued"),
            Err(e) => socket.reply(false, &e),
        };
        if let Err(e) = reply {
            self.host.log(XGC_LOG_WARN, &format!("station-io reply: {e}"));
        }
        Ok(())
    }

    fn deactivate(&mut self) -> Result<(), String> {
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

pub fn submit_command(path: &std::path::Path, token: &str) -> Result<(), String> {
    if !wire::valid_command_token(token) {
        return Err("command token is not a controller string".into());
    }
    socket::transact(path, socket::KIND_COMMAND, token.as_bytes())
}

pub fn submit_mission_file(socket_path: &std::path::Path, file: &std::path::Path) -> Result<(), String> {
    let bytes = std::fs::read(file).map_err(|e| e.to_string())?;
    if !wire::timeline_schema_ok(&bytes) {
        return Err("mission file must be 240 bytes with schema 1".into());
    }
    socket::transact(socket_path, socket::KIND_MISSION, &bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use xgc_rt_host::deployment::*;

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

    #[test]
    fn rendered_station_config_matches_binds_and_configures() {
        use std::path::Path;

        fn pin() -> BundleFile {
            BundleFile { path: "plugins/mod.so".into(), sha256: sha256(b"station-pin") }
        }
        fn plugins(id: &str) -> Plugins {
            let mut plugins = Plugins {
                ros_io: pin(),
                rigid_state: None,
                hover_thrust: None,
                controller: None,
                reference: None,
                numeric_vehicle: None,
                plan_dmpc: None,
                dmpc_rounds: None,
                station_io: Some(pin()),
            };
            if id != SMC_COMPOSITION_ID {
                plugins.plan_dmpc = Some(pin());
                plugins.dmpc_rounds = Some(pin());
            }
            if id == HIL_COMPOSITION_ID {
                plugins.numeric_vehicle = Some(pin());
            }
            if id == NATIVE_COMPOSITION_ID || id == SMC_COMPOSITION_ID {
                plugins.controller = Some(pin());
            }
            plugins
        }
        fn render_case(raw: &str, id: &str) -> toml::Value {
            let (deployment, _) = Deployment::parse(raw).unwrap();
            let bundle = Bundle {
                schema_version: 1,
                platform: "linux-amd64".into(),
                composition_sha256: composition(id).unwrap().sha256(),
                host: pin(),
                plugins: plugins(id),
                libraries: vec![],
                links: vec![],
            };
            let text = render(&deployment, &bundle, Path::new("/tmp/station-render"), Path::new("/tmp/station-audit")).unwrap();
            text.parse().unwrap()
        }
        fn check(manifest: &toml::Value) {
            let station = manifest["plugin"].as_array().unwrap().iter().find(|p| p["name"].as_str() == Some("station-io")).unwrap();
            let binds = station["bind"].as_table().unwrap();
            let wire = toml::to_string(station.get("config").unwrap()).unwrap();
            let parsed = configure_text(&wire).expect(&wire);
            assert_eq!(parsed.command, binds.contains_key("command"), "{wire}");
            assert_eq!(parsed.mission, binds.contains_key("mission_request"), "{wire}");
            if parsed.authority {
                assert!(parsed.command && parsed.mission, "{wire}");
            } else {
                assert!(!parsed.command && !parsed.mission, "{wire}");
            }
        }
        check(&render_case(&hil_raw(true), HIL_COMPOSITION_ID));
        check(&render_case(&hil_raw(false), HIL_COMPOSITION_ID));
        check(&render_case(&native_raw(true), NATIVE_COMPOSITION_ID));
        check(&render_case(&native_raw(false), NATIVE_COMPOSITION_ID));
        check(&render_case(&planner_raw(true), PLANNER_COMPOSITION_ID));
        check(&render_case(&planner_raw(false), PLANNER_COMPOSITION_ID));
        check(&render_case(&smc_raw(), SMC_COMPOSITION_ID));
    }

    fn deployment_json(id: &str, node: &str, namespace: &str, config: serde_json::Value) -> String {
        let config_json = serde_json::to_string(&config).unwrap();
        serde_json::json!({
            "schema_version": 1,
            "session_id": "station-render",
            "node_id": node,
            "robot_namespace": namespace,
            "platform": "linux-amd64",
            "bundle_sha256": sha256(b"bundle"),
            "composition_id": id,
            "composition_sha256": composition(id).unwrap().sha256(),
            "configuration_sha256": sha256(config_json.as_bytes()),
            "configuration_json": config_json,
        }).to_string()
    }
    fn station_block() -> serde_json::Value {
        serde_json::json!({"robot_id":"xgc2e-12345678901234567890","zenoh_connect":"tcp/127.0.0.1:17457","command_socket":"/tmp/dmpc-board.sock"})
    }
    fn hil_raw(authority_self: bool) -> String {
        deployment_json(HIL_COMPOSITION_ID, "board-b", "uav2", serde_json::json!({
            "input_time_domain":"wall-unix","ros_master_uri":"http://127.0.0.1:11311","ros_ip":"127.0.0.1",
            "epoch_ns":1900000000000000000_i64,
            "members":[
                {"uav_id":1,"robot_namespace":"uav1","planner_node":"board-a","control_node":"board-a"},
                {"uav_id":2,"robot_namespace":"uav2","planner_node":"board-b","control_node":"board-b"}],
            "mission_authority_node": if authority_self {"board-b"} else {"board-a"},
            "radio":{"listen":["tcp/127.0.0.1:17442"],"connect":["tcp/127.0.0.1:17441"]},
            "station": station_block(),
            "scene":{"snapshot_topic":"/experiment/scene/snapshot","state_topic":"/experiment/scene/state","timeline_ack_topic":"/uav2/dmpc/timeline_ack","timeline_status_topic":"/uav2/dmpc/timeline_status"},
            "planner":{"algorithm":"legacy","scene_id":"dmpc-uav8_comprehensive","chain_n":3,"state_dim":9,"horizon":40,"sampling_time":0.1},
            "initial_position":[1.2,-3.4,0.0],"initial_velocity":[0.0,0.0,0.0]
        }))
    }
    fn topics(ns: &str) -> serde_json::Value {
        serde_json::json!({
            "imu_topic": format!("/{ns}/mavros/imu/data_raw"),
            "pose_topic": format!("/mocap/{ns}/pose"),
            "vision_pose_topic": format!("/{ns}/mavros/vision_pose/pose"),
            "fcu_state_topic": format!("/{ns}/mavros/state"),
            "local_pose_topic": format!("/{ns}/mavros/local_position/pose"),
            "local_velocity_topic": format!("/{ns}/mavros/local_position/velocity_local"),
            "fcu_imu_topic": format!("/{ns}/mavros/imu/data"),
            "battery_topic": format!("/{ns}/mavros/battery"),
            "setpoint_topic": format!("/{ns}/mavros/setpoint_raw/local"),
            "status_topic": format!("/{ns}/custom/statustext"),
            "fcu_request_topic": format!("/{ns}/mavros/cmd")
        })
    }
    fn native_raw(authority_self: bool) -> String {
        let (node, ns) = if authority_self { ("board-a", "uav1") } else { ("board-b", "uav2") };
        deployment_json(NATIVE_COMPOSITION_ID, node, ns, serde_json::json!({
            "input_time_domain":"wall-unix","ros_master_uri":"http://127.0.0.1:11311","ros_ip":"127.0.0.1",
            "epoch_ns":1900000000000000000_i64,
            "members":[
                {"uav_id":1,"robot_namespace":"uav1","planner_node":"board-a","control_node":"board-a"},
                {"uav_id":2,"robot_namespace":"uav2","planner_node":"board-b","control_node":"board-b"}],
            "mission_authority_node":"board-a",
            "radio":{"listen":["tcp/127.0.0.1:17442"],"connect":["tcp/127.0.0.1:17441"]},
            "station": station_block(),
            "scene":{"snapshot_topic":"/experiment/scene/snapshot","state_topic":"/experiment/scene/state",
                "timeline_ack_topic": format!("/{ns}/dmpc/timeline_ack"),
                "timeline_status_topic": format!("/{ns}/dmpc/timeline_status")},
            "planner":{"algorithm":"legacy","scene_id":"dmpc-uav8_comprehensive","chain_n":3,"state_dim":9,"horizon":40,"sampling_time":0.1},
            "takeoff_altitude_m":1.5,
            "topics": topics(ns)
        }))
    }
    fn planner_raw(authority_self: bool) -> String {
        let (node, ns) = if authority_self { ("gcs-a", "uav1") } else { ("gcs-b", "uav2") };
        deployment_json(PLANNER_COMPOSITION_ID, node, ns, serde_json::json!({
            "input_time_domain":"wall-unix","ros_master_uri":"http://127.0.0.1:11311","ros_ip":"127.0.0.1",
            "epoch_ns":1900000000000000000_i64,
            "members":[
                {"uav_id":1,"robot_namespace":"uav1","planner_node":"gcs-a","control_node":"board-a"},
                {"uav_id":2,"robot_namespace":"uav2","planner_node":"gcs-b","control_node":"board-b"}],
            "mission_authority_node":"gcs-a",
            "radio":{"listen":["tcp/127.0.0.1:17442"],"connect":["tcp/127.0.0.1:17441"]},
            "station": station_block(),
            "scene":{"snapshot_topic":"/experiment/scene/snapshot","state_topic":"/experiment/scene/state",
                "timeline_ack_topic": format!("/{ns}/dmpc/timeline_ack"),
                "timeline_status_topic": format!("/{ns}/dmpc/timeline_status")},
            "planner":{"algorithm":"legacy","scene_id":"dmpc-uav8_comprehensive","chain_n":3,"state_dim":9,"horizon":40,"sampling_time":0.1}
        }))
    }
    fn smc_raw() -> String {
        deployment_json(SMC_COMPOSITION_ID, "board-b", "uav2", serde_json::json!({
            "input_time_domain":"wall-unix","ros_master_uri":"http://127.0.0.1:11311","ros_ip":"127.0.0.1",
            "epoch_ns":1900000000000000000_i64,
            "members":[
                {"uav_id":1,"robot_namespace":"uav1","planner_node":"gcs-a","control_node":"board-a"},
                {"uav_id":2,"robot_namespace":"uav2","planner_node":"gcs-b","control_node":"board-b"}],
            "mission_authority_node":"gcs-a",
            "radio":{"listen":["tcp/127.0.0.1:17442"],"connect":["tcp/127.0.0.1:17441"]},
            "station": station_block(),
            "takeoff_altitude_m":1.5,
            "topics": topics("uav2")
        }))
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
