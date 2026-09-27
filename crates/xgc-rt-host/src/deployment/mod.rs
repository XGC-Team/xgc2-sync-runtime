//! Release-owned deployment, separate from the software-plant fixture.
//! No live ROS/config discovery: inputs freeze the run and its clock authority.
mod dmpc;
mod files;
mod hil;
#[cfg(test)]
mod target_platform_tests;
pub use hil::{HilConfiguration, RobotMember, NodeRole};
pub(crate) use hil::{Planner, Radio, Scene, Station};

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::net::Ipv4Addr;
use std::path::{Component, Path, PathBuf};
use xgc_rt_core::manifest::Manifest;

pub use files::{prepare, Prepared};
pub type Result<T> = std::result::Result<T, String>;
pub const COMPOSITION_ID: &str = "uav-control-dfbc-native-hover/v1";
pub const COMPOSITION: &str = include_str!("composition.toml");
pub const PX4_LOCAL_COMPOSITION_ID: &str = "uav-control-px4-local-native-hover/v1";
pub const PX4_LOCAL_COMPOSITION: &str = include_str!("composition-px4-local.toml");
pub const HIL_COMPOSITION_ID: &str = "uav-dmpc-numeric-hil/v1";
pub const HIL_COMPOSITION: &str = include_str!("composition-numeric-hil.toml");
pub const NATIVE_COMPOSITION_ID: &str = "uav-dmpc-native/v1";
pub const NATIVE_COMPOSITION: &str = include_str!("composition-dmpc-native.toml");
pub const PLANNER_COMPOSITION_ID: &str = "uav-dmpc-planner/v1";
pub const PLANNER_COMPOSITION: &str = include_str!("composition-dmpc-planner.toml");
pub const SMC_COMPOSITION_ID: &str = "uav-dmpc-smc/v1";
pub const SMC_COMPOSITION: &str = include_str!("composition-dmpc-smc.toml");
pub const BUNDLE_FILE: &str = "DEPLOYMENT-BUNDLE.json";
pub const MAX_INPUT: usize = 64 * 1024;

// Compiled target, never the Core host or an environment override. Keep this
// mapping shared by deployment validation and the target's ELF/describe gates.
fn native_target(os: &str, arch: &str) -> Result<(&'static str, u16)> {
    match (os, arch) {
        ("linux", "x86_64") => Ok(("linux-amd64", 62)),
        ("linux", "aarch64") => Ok(("linux-arm64", 183)),
        _ => Err(format!("unsupported native target {os}/{arch}")),
    }
}

pub fn target_platform() -> Result<&'static str> {
    native_target(std::env::consts::OS, std::env::consts::ARCH).map(|target| target.0)
}

/// e_machine for the same compiled target as target_platform().
pub fn target_elf_machine() -> Result<u16> {
    native_target(std::env::consts::OS, std::env::consts::ARCH).map(|target| target.1)
}

/// Release-owned graphs. The deployment cannot add roles or edit a graph.
pub struct Composition {
    pub id: &'static str,
    pub bytes: &'static str,
}
impl Composition {
    pub fn sha256(&self) -> String {
        sha256(self.bytes.as_bytes())
    }
}
static COMPOSITIONS: [Composition; 6] = [
    Composition {
        id: COMPOSITION_ID,
        bytes: COMPOSITION,
    },
    Composition {
        id: PX4_LOCAL_COMPOSITION_ID,
        bytes: PX4_LOCAL_COMPOSITION,
    },
    Composition { id: HIL_COMPOSITION_ID, bytes: HIL_COMPOSITION },
    Composition { id: NATIVE_COMPOSITION_ID, bytes: NATIVE_COMPOSITION },
    Composition { id: PLANNER_COMPOSITION_ID, bytes: PLANNER_COMPOSITION },
    Composition { id: SMC_COMPOSITION_ID, bytes: SMC_COMPOSITION },
];

pub fn composition(id: &str) -> Result<&'static Composition> {
    COMPOSITIONS
        .iter()
        .find(|c| c.id == id)
        .ok_or_else(|| "unsupported composition identity".into())
}

pub fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}
pub fn composition_sha256() -> String {
    sha256(COMPOSITION.as_bytes())
}
fn require(ok: bool, message: &str) -> Result<()> {
    if ok {
        Ok(())
    } else {
        Err(message.into())
    }
}
fn digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}
fn name(value: &str) -> bool {
    value
        .as_bytes()
        .first()
        .is_some_and(u8::is_ascii_alphanumeric)
        && value.len() <= 64
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}
fn ros_segment(value: &str) -> bool {
    value
        .as_bytes()
        .first()
        .is_some_and(u8::is_ascii_alphabetic)
        && value
            .bytes()
            .all(|b| b.is_ascii_alphanumeric() || b == b'_')
}
fn ros_namespace(value: &str) -> bool {
    value.len() <= 64 && ros_segment(value)
}
fn topic(value: &str) -> bool {
    value.len() <= 128
        && value
            .strip_prefix('/')
            .is_some_and(|s| s.split('/').all(ros_segment))
}
fn identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 256
        && value.trim() == value
        && !value.bytes().any(|b| b < 32 || b == 127)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Deployment {
    pub schema_version: u32,
    pub session_id: String,
    pub node_id: String,
    pub robot_namespace: String,
    pub platform: String,
    pub bundle_sha256: String,
    pub composition_id: String,
    pub composition_sha256: String,
    pub configuration_sha256: String,
    /// SHA binds these exact UTF-8 bytes; never reserialize floats to check it.
    pub configuration_json: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Configuration {
    pub input_time_domain: TimeDomain,
    pub ros_master_uri: String,
    pub ros_ip: String,
    pub takeoff_altitude_m: f64,
    pub topics: Topics,
    pub calibration: Calibration,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub robot_member: Option<RobotMember>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub simulation: Option<Simulation>,
}

/// Configuration shape is selected by the compiled composition, not an untagged fallback.
#[derive(Clone, Debug)]
pub enum RunConfiguration {
    Control(Configuration),
    Hil(HilConfiguration),
    Native(dmpc::NativeConfiguration),
    Planner(dmpc::PlannerConfiguration),
    Smc(dmpc::SmcConfiguration),
}
impl RunConfiguration {
    pub fn time_domain(&self) -> &TimeDomain {
        match self {
            Self::Control(c) => &c.input_time_domain,
            Self::Hil(c) => &c.input_time_domain,
            Self::Native(c) => &c.input_time_domain,
            Self::Planner(c) => &c.input_time_domain,
            Self::Smc(c) => &c.input_time_domain,
        }
    }
    pub fn ros_master_uri(&self) -> &str {
        match self {
            Self::Control(c) => &c.ros_master_uri,
            Self::Hil(c) => &c.ros_master_uri,
            Self::Native(c) => &c.ros_master_uri,
            Self::Planner(c) => &c.ros_master_uri,
            Self::Smc(c) => &c.ros_master_uri,
        }
    }
    pub fn ros_ip(&self) -> &str {
        match self {
            Self::Control(c) => &c.ros_ip,
            Self::Hil(c) => &c.ros_ip,
            Self::Native(c) => &c.ros_ip,
            Self::Planner(c) => &c.ros_ip,
            Self::Smc(c) => &c.ros_ip,
        }
    }
    pub fn role(&self) -> &'static str {
        match self {
            Self::Control(_) => "control",
            Self::Hil(_) => "numeric-hil",
            Self::Native(_) => "dmpc-native",
            Self::Planner(_) => "dmpc-planner",
            Self::Smc(_) => "dmpc-smc",
        }
    }
    pub fn actuator_namespace<'a>(&self, namespace: &'a str) -> Option<&'a str> {
        match self {
            Self::Control(_) | Self::Native(_) | Self::Smc(_) => Some(namespace),
            Self::Hil(_) | Self::Planner(_) => None,
        }
    }
}

// Missing optional blocks preserve existing inputs; explicit null is not a block.
fn present<'de, D, T>(deserializer: D) -> std::result::Result<Option<T>, D::Error>
where D: serde::Deserializer<'de>, T: Deserialize<'de> {
    T::deserialize(deserializer).map(Some)
}

/// The fixed ROS source uses the same authority and limits as Manifest.
/// E0 is supplied by the experiment Run coordinator, never local arrival time.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Simulation {
    pub epoch_ns: i64,
    pub topic: String,
    pub expected_publisher: String,
    pub world_instance_id: String,
    pub startup_timeout_wall_ms: u64,
    pub stale_after_wall_ms: u64,
    pub max_advance_ns: i64,
    pub poll_wall_ms: u64,
    pub queue_capacity: u32,
}

#[derive(Clone, Debug, Serialize)]
pub enum TimeDomain {
    #[serde(rename = "wall-unix")]
    WallUnix,
    #[serde(rename = "ros1-sim")]
    Ros1Sim,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Topics {
    pub imu_topic: String,
    pub pose_topic: String,
    pub vision_pose_topic: String,
    pub rigid_state_estimate_topic: String,
    pub fcu_state_topic: String,
    pub local_pose_topic: String,
    pub local_velocity_topic: String,
    pub fcu_imu_topic: String,
    pub battery_topic: String,
    pub command_topic: String,
    pub alg_setpoint_topic: String,
    pub attitude_target_topic: String,
    pub setpoint_topic: String,
    pub attitude_rate_topic: String,
    pub status_topic: String,
    pub fcu_request_topic: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Calibration {
    pub verified: bool,
    pub field_offset_xyz: [f64; 3],
    pub field_offset_rpy: [f64; 3],
    pub imu_to_vrpn_marker_xyz: [f64; 3],
    pub imu_to_vrpn_marker_rpy: [f64; 3],
    pub provenance: Provenance,
}
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum CalibrationKind {
    Unverified,
    SimulationModel,
    MeasuredCalibration,
}
// String::deserialize is deliberate: serde's derived externally tagged unit
// enums also accept {"variant":null}, which is not this JSON contract.
impl<'de> Deserialize<'de> for TimeDomain {
    fn deserialize<D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<Self, D::Error> {
        let value = String::deserialize(deserializer)?;
        match value.as_str() {
            "wall-unix" => Ok(Self::WallUnix),
            "ros1-sim" => Ok(Self::Ros1Sim),
            _ => Err(serde::de::Error::unknown_variant(&value, &["wall-unix", "ros1-sim"])),
        }
    }
}
impl<'de> Deserialize<'de> for CalibrationKind {
    fn deserialize<D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<Self, D::Error> {
        let value = String::deserialize(deserializer)?;
        match value.as_str() {
            "unverified" => Ok(Self::Unverified),
            "simulation-model" => Ok(Self::SimulationModel),
            "measured-calibration" => Ok(Self::MeasuredCalibration),
            _ => Err(serde::de::Error::unknown_variant(
                &value,
                &["unverified", "simulation-model", "measured-calibration"],
            )),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Provenance {
    pub kind: CalibrationKind,
    pub source_id: String,
    pub source_sha256: String,
    pub robot_asset_id: String,
}

impl Deployment {
    pub fn parse(raw: &str) -> Result<(Self, RunConfiguration)> {
        require(raw.len() <= MAX_INPUT, "deployment exceeds 64 KiB")?;
        let deployment: Self =
            serde_json::from_str(raw).map_err(|e| format!("deployment schema: {e}"))?;
        let config = deployment.validate()?;
        Ok((deployment, config))
    }
    pub fn validate(&self) -> Result<RunConfiguration> {
        require(self.schema_version == 1, "unsupported deployment schema")?;
        require(
            name(&self.session_id) && name(&self.node_id),
            "invalid Session/node identity",
        )?;
        require(
            ros_namespace(&self.robot_namespace),
            "invalid canonical relative robot namespace",
        )?;
        let target = target_platform()?;
        require(
            self.platform == target,
            &format!("deployment platform {} does not match compiled native target {target}", self.platform),
        )?;
        require(
            self.composition_sha256 == composition(&self.composition_id)?.sha256(),
            "composition identity/digest mismatch",
        )?;
        require(
            digest(&self.bundle_sha256) && digest(&self.configuration_sha256),
            "noncanonical SHA256",
        )?;
        require(
            self.configuration_json.len() <= MAX_INPUT
                && sha256(self.configuration_json.as_bytes()) == self.configuration_sha256,
            "configuration bytes/digest mismatch",
        )?;
        if self.composition_id == HIL_COMPOSITION_ID {
            let config: HilConfiguration = serde_json::from_str(&self.configuration_json)
                .map_err(|e| format!("HIL configuration schema: {e}"))?;
            config.validate(self)?;
            return Ok(RunConfiguration::Hil(config));
        }
        if self.composition_id == NATIVE_COMPOSITION_ID {
            let config: dmpc::NativeConfiguration = serde_json::from_str(&self.configuration_json)
                .map_err(|e| format!("native DMPC configuration schema: {e}"))?;
            config.validate(self)?;
            return Ok(RunConfiguration::Native(config));
        }
        if self.composition_id == PLANNER_COMPOSITION_ID {
            let config: dmpc::PlannerConfiguration = serde_json::from_str(&self.configuration_json)
                .map_err(|e| format!("planner configuration schema: {e}"))?;
            config.validate(self)?;
            return Ok(RunConfiguration::Planner(config));
        }
        if self.composition_id == SMC_COMPOSITION_ID {
            let config: dmpc::SmcConfiguration = serde_json::from_str(&self.configuration_json)
                .map_err(|e| format!("SMC configuration schema: {e}"))?;
            config.validate(self)?;
            return Ok(RunConfiguration::Smc(config));
        }
        let config: Configuration = serde_json::from_str(&self.configuration_json)
            .map_err(|e| format!("configuration schema: {e}"))?;
        config.validate(&self.robot_namespace, &self.node_id)?;
        Ok(RunConfiguration::Control(config))
    }
    pub fn identity_bytes(&self) -> Result<Vec<u8>> {
        serde_json::to_vec(self).map_err(|e| e.to_string())
    }
}

impl Configuration {
    fn validate(&self, namespace: &str, node: &str) -> Result<()> {
        if let Some(member) = &self.robot_member {
            require(member.uav_id > 0 && ros_namespace(&member.robot_namespace)
                && name(&member.planner_node) && name(&member.control_node), "invalid robot member")?;
            require(member.robot_namespace == namespace && member.control_node == node,
                "control composition must run on the selected robot control node")?;
        }
        require(
            matches!((&self.input_time_domain, &self.simulation),
                (TimeDomain::WallUnix, None) | (TimeDomain::Ros1Sim, Some(_))),
            "ros1-sim requires simulation; wall-unix forbids simulation",
        )?;
        ros_endpoint(&self.ros_master_uri, &self.ros_ip)?;
        require(
            self.takeoff_altitude_m.is_finite() && self.takeoff_altitude_m > 0.0,
            "takeoff_altitude_m must be finite and positive",
        )?;
        let fields = serde_json::to_value(&self.topics).map_err(|e| e.to_string())?;
        let mut seen = BTreeSet::new();
        for (role, value) in fields.as_object().ok_or("internal topic schema")? {
            let value = value.as_str().ok_or("internal topic type")?;
            require(
                topic(value),
                &format!("invalid absolute ROS name for {role}"),
            )?;
            require(
                seen.insert(value),
                "topic/service roles must have distinct names",
            )?;
            if role != "pose_topic" && role != "command_topic" {
                require(
                    value.starts_with(&format!("/{namespace}/")),
                    &format!("{role} is outside the selected robot namespace"),
                )?;
            }
        }
        validate_calibration(&self.calibration)
    }
}

pub(crate) fn validate_calibration(c: &Calibration) -> Result<()> {
    require(
        c.field_offset_xyz
            .iter()
            .chain(&c.field_offset_rpy)
            .chain(&c.imu_to_vrpn_marker_xyz)
            .chain(&c.imu_to_vrpn_marker_rpy)
            .all(|v| v.is_finite()),
        "nonfinite calibration transform",
    )?;
    require(
        c.verified == (c.provenance.kind != CalibrationKind::Unverified),
        "calibration verification/provenance disagreement",
    )?;
    require(
        identifier(&c.provenance.source_id)
            && identifier(&c.provenance.robot_asset_id)
            && digest(&c.provenance.source_sha256),
        "explicit calibration provenance is required",
    )
}

fn ros_endpoint(ros_master_uri: &str, ros_ip: &str) -> Result<()> {
        let ip: Ipv4Addr = ros_ip
            .parse()
            .map_err(|_| "ros_ip must be a unicast IPv4 literal")?;
        require(
            !ip.is_unspecified() && !ip.is_multicast() && ip != Ipv4Addr::BROADCAST,
            "ros_ip must be a unicast IPv4 literal",
        )?;
        let address = ros_master_uri
            .strip_prefix("http://")
            .ok_or("ROS master must be http://IPv4:port")?;
        let (host, port) = address
            .rsplit_once(':')
            .ok_or("ROS master must be http://IPv4:port")?;
        let host: Ipv4Addr = host
            .parse()
            .map_err(|_| "ROS master must be http://IPv4:port")?;
        require(
            !port.is_empty() && port.bytes().all(|b| b.is_ascii_digit()),
            "invalid ROS master port",
        )?;
        let port: u16 = port.parse().map_err(|_| "invalid ROS master port")?;
        require(
            port > 0
                && !host.is_unspecified()
                && !host.is_multicast()
                && host != Ipv4Addr::BROADCAST,
            "invalid ROS master endpoint",
        )?;
        Ok(())
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BundleFile {
    pub path: String,
    pub sha256: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BundleLink {
    pub path: String,
    pub target: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Plugins {
    pub ros_io: BundleFile,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub rigid_state: Option<BundleFile>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub hover_thrust: Option<BundleFile>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub controller: Option<BundleFile>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub reference: Option<BundleFile>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub numeric_vehicle: Option<BundleFile>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub plan_dmpc: Option<BundleFile>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub dmpc_rounds: Option<BundleFile>,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub station_io: Option<BundleFile>,
}
impl Plugins {
    pub fn entries(&self) -> Vec<(&str, &BundleFile)> {
        let mut entries = vec![("ros_io", &self.ros_io)];
        if let Some(pin) = &self.rigid_state { entries.push(("rigid_state", pin)); }
        if let Some(pin) = &self.hover_thrust { entries.push(("hover_thrust", pin)); }
        if let Some(pin) = &self.controller { entries.push(("controller", pin)); }
        if let Some(pin) = &self.reference { entries.push(("reference", pin)); }
        if let Some(pin) = &self.numeric_vehicle { entries.push(("numeric_vehicle", pin)); }
        if let Some(pin) = &self.plan_dmpc { entries.push(("plan_dmpc", pin)); }
        if let Some(pin) = &self.dmpc_rounds { entries.push(("dmpc_rounds", pin)); }
        if let Some(pin) = &self.station_io { entries.push(("station_io", pin)); }
        entries
    }
    fn validate_roles(&self, id: &str) -> Result<()> {
        let expected: &[&str] = if id == HIL_COMPOSITION_ID {
            &["ros_io", "numeric_vehicle", "plan_dmpc", "dmpc_rounds", "station_io"]
        } else if id == NATIVE_COMPOSITION_ID {
            &["ros_io", "controller", "plan_dmpc", "dmpc_rounds", "station_io"]
        } else if id == PLANNER_COMPOSITION_ID {
            &["ros_io", "plan_dmpc", "dmpc_rounds", "station_io"]
        } else if id == SMC_COMPOSITION_ID {
            &["ros_io", "controller", "station_io"]
        } else {
            &["ros_io", "rigid_state", "hover_thrust", "controller", "reference"]
        };
        let roles: BTreeSet<_> = self.entries().into_iter().map(|(role, _)| role).collect();
        require(roles == expected.iter().copied().collect(), "bundle plugin roles do not match fixed composition")
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Bundle {
    pub schema_version: u32,
    pub platform: String,
    pub composition_sha256: String,
    pub host: BundleFile,
    pub plugins: Plugins,
    pub libraries: Vec<BundleFile>,
    pub links: Vec<BundleLink>,
}
pub(crate) fn relative(path: &str) -> bool {
    !path.is_empty()
        && path.len() <= 1024
        && !path.contains('\0')
        && path
            .split('/')
            .all(|c| !c.is_empty() && c != "." && c != "..")
}

pub(crate) fn absolute(path: &Path) -> bool {
    path.to_str()
        .is_some_and(|s| s == "/" || s.strip_prefix('/').is_some_and(relative))
}

/// Pure construction; artifact verification and atomic publication are separate.
pub fn render(
    deployment: &Deployment,
    bundle: &Bundle,
    bundle_root: &Path,
    audit: &Path,
) -> Result<String> {
    let config = deployment.validate()?;
    let composition = composition(&deployment.composition_id)?;
    bundle.plugins.validate_roles(composition.id)?;
    require(
        bundle.schema_version == 1
            && bundle.platform == deployment.platform
            && bundle.composition_sha256 == composition.sha256(),
        "bundle release does not match deployment",
    )?;
    require(
        absolute(bundle_root) && absolute(audit),
        "render paths must be absolute",
    )?;
    let config = match config {
        RunConfiguration::Hil(config) => return hil::render(deployment, &config, bundle, bundle_root, audit),
        RunConfiguration::Native(config) => return dmpc::render_native(deployment, &config, bundle, bundle_root, audit),
        RunConfiguration::Planner(config) => return dmpc::render_planner(deployment, &config, bundle, bundle_root, audit),
        RunConfiguration::Smc(config) => return dmpc::render_smc(deployment, &config, bundle, bundle_root, audit),
        RunConfiguration::Control(config) => config,
    };
    let mut value: toml::Value = composition
        .bytes
        .parse()
        .map_err(|e| format!("internal composition: {e}"))?;
    value["session"]["id"] = deployment.session_id.clone().into();
    value["session"]["node"] = deployment.node_id.clone().into();
    value["session"]["roster"] = toml::Value::Array(vec![deployment.node_id.clone().into()]);
    value["audit"]["dir"] = audit.to_str().ok_or("non-UTF8 audit path")?.into();
    if let Some(sim) = &config.simulation {
        value["session"].as_table_mut().ok_or("internal session")?
            .insert("epoch_ns".into(), sim.epoch_ns.into());
        let mut source = toml::Value::try_from(sim).map_err(|e| e.to_string())?;
        let fields = source.as_table_mut().ok_or("internal clock source")?;
        fields.remove("epoch_ns");
        fields.insert("kind".into(), "ros1_sim".into());
        fields.insert("plugin".into(), "ros_io".into());
        value.as_table_mut().ok_or("internal composition table")?
            .insert("clock_source".into(), source);
    }
    for plugin in value["plugin"].as_array_mut().ok_or("internal plugins")? {
        let role = plugin["path"].as_str().ok_or("internal role")?.to_owned();
        let pin = bundle
            .plugins
            .entries()
            .into_iter()
            .find(|(k, _)| *k == role)
            .ok_or("internal unbound artifact role")?
            .1;
        require(
            relative(&pin.path) && pin.path.starts_with("plugins/") && digest(&pin.sha256),
            "invalid plugin pin",
        )?;
        plugin["path"] = bundle_root
            .join(&pin.path)
            .to_str()
            .ok_or("non-UTF8 plugin path")?
            .into();
        plugin
            .as_table_mut()
            .ok_or("internal plugin table")?
            .insert("sha256".into(), pin.sha256.clone().into());
        for (_, bind) in plugin["bind"]
            .as_table_mut()
            .ok_or("internal bindings")?
            .iter_mut()
        {
            if let Some(origins) = bind.get_mut("from") {
                require(
                    origins
                        .as_array()
                        .is_some_and(|a| a.len() == 1 && a[0].as_str() == Some("SELF")),
                    "internal composition has non-self origin",
                )?;
                *origins = toml::Value::Array(vec![deployment.node_id.clone().into()]);
            }
        }
        let cfg = plugin["config"]
            .as_table_mut()
            .ok_or("internal plugin configuration")?;
        if role == "ros_io" {
            *cfg = toml::Value::try_from(&config.topics)
                .map_err(|e| e.to_string())?
                .as_table()
                .ok_or("internal topics")?
                .clone();
            cfg.insert(
                "node_name".into(),
                format!(
                    "xgc_rt_ros_{}",
                    &sha256(format!("{}:{}", deployment.session_id, deployment.node_id).as_bytes())
                        [..16]
                )
                .into(),
            );
        } else if role == "controller" {
            cfg.insert("takeoff_altitude".into(), config.takeoff_altitude_m.into());
        } else if role == "rigid_state" {
            cfg.insert(
                "extrinsic_verified".into(),
                config.calibration.verified.into(),
            );
            for (key, values) in [
                ("field_offset_xyz", config.calibration.field_offset_xyz),
                ("field_offset_rpy", config.calibration.field_offset_rpy),
                (
                    "imu_to_vrpn_marker_xyz",
                    config.calibration.imu_to_vrpn_marker_xyz,
                ),
                (
                    "imu_to_vrpn_marker_rpy",
                    config.calibration.imu_to_vrpn_marker_rpy,
                ),
            ] {
                cfg.insert(
                    key.into(),
                    toml::Value::Array(values.iter().map(|v| (*v).into()).collect()),
                );
            }
        }
    }
    let text = toml::to_string_pretty(&value).map_err(|e| e.to_string())?;
    let manifest = Manifest::from_toml_str(&text).map_err(|e| e.to_string())?;
    manifest.resolve().map_err(|e| e.to_string())?;
    Ok(text)
}

/// Managed launch derives its path on the selected target, never in Core's
/// normalized process parameters. There is no home/tmp fallback.
pub fn managed_root(agent: Option<&str>, core: Option<&str>) -> Result<PathBuf> {
    let agent = agent.filter(|s| !s.is_empty());
    let core = core.filter(|s| !s.is_empty());
    let root = match (agent, core) {
        (Some(a), Some(c)) if a == c => a,
        (Some(a), None) => a,
        (None, Some(c)) => c,
        _ => {
            return Err(
                "exactly one target-owned managed root is required (or two equal roots)".into(),
            )
        }
    };
    let root = Path::new(root);
    require(absolute(root), "managed root must be a clean absolute path")?;
    Ok(root.join("sync-runtime"))
}
