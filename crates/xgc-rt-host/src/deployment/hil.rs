//! Fixed numerical HIL graph. Robot identity and execution-node identity are distinct.
use super::*;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RobotMember {
    pub uav_id: u32,
    pub robot_namespace: String,
    pub planner_node: String,
    pub control_node: String,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NodeRole { Planner, Control, CoLocated }
impl RobotMember {
    pub fn role(&self, node: &str) -> Option<NodeRole> {
        match (node == self.planner_node, node == self.control_node) {
            (true, true) => Some(NodeRole::CoLocated), (true, false) => Some(NodeRole::Planner),
            (false, true) => Some(NodeRole::Control), _ => None,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Radio { pub listen: Vec<String>, pub connect: Vec<String> }
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Station {
    pub robot_id: String,
    pub zenoh_connect: String,
    pub command_socket: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Scene {
    pub snapshot_topic: String,
    pub state_topic: String,
    pub timeline_ack_topic: String,
    pub timeline_status_topic: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Planner {
    pub algorithm: String,
    pub scene_id: String,
    pub chain_n: u32,
    pub state_dim: u32,
    pub horizon: u32,
    pub sampling_time: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HilConfiguration {
    pub input_time_domain: TimeDomain,
    pub ros_master_uri: String,
    pub ros_ip: String,
    pub epoch_ns: i64,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "present")]
    pub simulation: Option<Simulation>,
    pub members: Vec<RobotMember>,
    pub mission_authority_node: String,
    pub radio: Radio,
    pub station: Station,
    pub scene: Scene,
    pub planner: Planner,
    pub initial_position: [f64; 3],
    pub initial_velocity: [f64; 3],
}
fn tcp(endpoint: &str) -> bool {
    endpoint.strip_prefix("tcp/").and_then(|s| s.rsplit_once(':')).is_some_and(|(host, port)| {
        !host.is_empty() && host.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'.' || b == b'-')
            && !port.is_empty() && port.bytes().all(|b| b.is_ascii_digit())
            && port.parse::<u16>().is_ok_and(|p| p > 0)
    })
}
impl HilConfiguration {
    pub fn roster(&self) -> Vec<String> {
        let mut nodes = Vec::new();
        for member in &self.members {
            for node in [&member.planner_node, &member.control_node] {
                if !nodes.contains(node) { nodes.push(node.clone()); }
            }
        }
        nodes
    }
    pub fn member_for_node(&self, node: &str) -> Option<&RobotMember> {
        self.members.iter().find(|m| m.role(node).is_some())
    }
    pub(super) fn validate(&self, deployment: &Deployment) -> Result<()> {
        ros_endpoint(&self.ros_master_uri, &self.ros_ip)?;
        require(self.epoch_ns > 0 && self.epoch_ns < i64::MAX - 10_000_000_000, "HIL requires positive shared epoch_ns")?;
        require(matches!((&self.input_time_domain, &self.simulation),
            (TimeDomain::WallUnix, None) | (TimeDomain::Ros1Sim, Some(_))),
            "ros1-sim requires simulation; wall-unix forbids simulation")?;
        if let Some(sim) = &self.simulation { require(sim.epoch_ns == self.epoch_ns, "planner and SimClock epochs differ")?; }
        require(!self.members.is_empty() && self.members.len() <= u16::MAX as usize / 2, "invalid HIL members")?;
        let (mut ids, mut robots, mut nodes) = (BTreeSet::new(), BTreeSet::new(), BTreeSet::new());
        for m in &self.members {
            require(m.uav_id > 0 && ids.insert(m.uav_id) && ros_namespace(&m.robot_namespace)
                && robots.insert(&m.robot_namespace), "invalid or repeated robot identity")?;
            require(name(&m.planner_node) && name(&m.control_node) && nodes.insert(&m.planner_node), "invalid or shared execution node")?;
            if m.control_node != m.planner_node { require(nodes.insert(&m.control_node), "shared execution node")?; }
            // This fixed graph contains both planner and numerical plant on each host.
            // The member model also represents a remote planner/control pair; that pair
            // needs its own actual control composition and cannot select the HIL graph.
            require(m.planner_node == m.control_node, "numeric HIL requires co-located planner and model nodes")?;
        }
        let member = self.member_for_node(&deployment.node_id).ok_or("node is not an experiment member")?;
        require(member.robot_namespace == deployment.robot_namespace, "node does not own selected robot namespace")?;
        require(self.members.iter().any(|m| m.planner_node == self.mission_authority_node), "mission authority is not a planner node")?;
        require(!self.radio.listen.is_empty() && self.radio.listen.iter().chain(&self.radio.connect).all(|e| tcp(e)), "explicit radio TCP endpoints required")?;
        let station = &self.station;
        require(station.robot_id.len() >= 3 && station.robot_id.len() <= 128
            && station.robot_id.as_bytes()[0].is_ascii_lowercase()
            && station.robot_id.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
            && station.robot_id.as_bytes().last().is_some_and(u8::is_ascii_alphanumeric)
            && !["uav", "ugv"].iter().any(|p| station.robot_id.strip_prefix(p).is_some_and(|s| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()))),
            "station robot_id must identify an experiment instance")?;
        require(tcp(&station.zenoh_connect), "station requires its GCS TCP endpoint")?;
        require(absolute(Path::new(&station.command_socket)) && station.command_socket.len() <= 100, "station command_socket must be an absolute path at most 100 bytes")?;
        let mut topics = BTreeSet::new();
        for value in [&self.scene.snapshot_topic, &self.scene.state_topic, &self.scene.timeline_ack_topic, &self.scene.timeline_status_topic] {
            require(topic(value) && topics.insert(value), "invalid or repeated HIL scene/status topic")?;
        }
        for value in [&self.scene.timeline_ack_topic, &self.scene.timeline_status_topic] {
            require(value.starts_with(&format!("/{}/", deployment.robot_namespace)), "HIL status topic outside robot namespace")?;
        }
        let p = &self.planner;
        require(p.algorithm == "legacy" && identifier(&p.scene_id) && p.chain_n > 0 && p.state_dim > 0 && p.horizon > 0
            && p.sampling_time == 0.1, "invalid 100 ms planner shape")?;
        require(self.initial_position.iter().chain(&self.initial_velocity).all(|v| v.is_finite()), "nonfinite scene slot state")
    }
}
fn val<T: Serialize>(value: T) -> Result<toml::Value> { toml::Value::try_from(value).map_err(|e| e.to_string()) }
fn table<T: Serialize>(value: T) -> Result<toml::Table> { val(value)?.as_table().cloned().ok_or("internal HIL configuration table".into()) }

pub(super) fn render(deployment: &Deployment, config: &HilConfiguration, bundle: &Bundle, root: &Path, audit: &Path) -> Result<String> {
    let mut value: toml::Value = HIL_COMPOSITION.parse().map_err(|e| format!("internal HIL composition: {e}"))?;
    let roster = config.roster();
    let origin = |node: &str| roster.iter().position(|n| n == node).expect("validated roster") as i64;
    let member = config.member_for_node(&deployment.node_id).ok_or("missing member")?;
    value["session"]["id"] = deployment.session_id.clone().into();
    value["session"]["node"] = deployment.node_id.clone().into();
    value["session"]["roster"] = val(&roster)?;
    value["session"]["epoch_ns"] = config.epoch_ns.into();
    value["audit"]["dir"] = audit.to_str().ok_or("non-UTF8 audit path")?.into();
    value["transport"]["listen"] = val(&config.radio.listen)?;
    value["transport"]["connect"] = val(&config.radio.connect)?;
    if let Some(sim) = &config.simulation {
        let mut source = table(sim)?;
        source.remove("epoch_ns"); source.insert("kind".into(), "ros1_sim".into()); source.insert("plugin".into(), "ros_io".into());
        value.as_table_mut().unwrap().insert("clock_source".into(), source.into());
    }
    for plugin in value["plugin"].as_array_mut().ok_or("internal HIL plugins")? {
        let role = plugin["path"].as_str().ok_or("internal HIL role")?.to_owned();
        let pin = bundle.plugins.entries().into_iter().find(|(r, _)| *r == role).ok_or("missing HIL artifact role")?.1;
        require(relative(&pin.path) && pin.path.starts_with("plugins/") && digest(&pin.sha256), "invalid plugin pin")?;
        plugin["path"] = root.join(&pin.path).to_str().ok_or("non-UTF8 plugin path")?.into();
        plugin.as_table_mut().unwrap().insert("sha256".into(), pin.sha256.clone().into());
        let binds = plugin["bind"].as_table_mut().ok_or("internal HIL bindings")?;
        for (_, binding) in binds.iter_mut() {
            if let Some(from) = binding.get_mut("from") {
                let selector = from.as_array().and_then(|a| a.first()).and_then(|s| s.as_str()).ok_or("internal origin selector")?;
                *from = val(match selector {
                    "SELF" => vec![deployment.node_id.clone()],
                    "AUTHORITY" => vec![config.mission_authority_node.clone()],
                    "PEERS" => config.members.iter().filter(|m| m.planner_node != deployment.node_id).map(|m| m.planner_node.clone()).collect(),
                    _ => return Err("internal HIL origin selector".into()),
                })?;
            }
        }
        if role == "station_io" && deployment.node_id != config.mission_authority_node {
            binds.remove("command"); binds.remove("mission_request");
        }
        let cfg = match role.as_str() {
            "numeric_vehicle" => table(serde_json::json!({"initial_position":config.initial_position,"initial_velocity":config.initial_velocity}))?,
            "station_io" => {
                let mut cfg = table(&config.station)?;
                cfg.insert("authority".into(), (deployment.node_id == config.mission_authority_node).into());
                cfg.insert("command".into(), binds.contains_key("command").into());
                cfg.insert("mission".into(), binds.contains_key("mission_request").into());
                cfg
            },
            "ros_io" => table(serde_json::json!({
                "node_name":format!("xgc_rt_ros_{}", &sha256(format!("{}:{}",deployment.session_id,deployment.node_id).as_bytes())[..16]),
                "scene_snapshot_topic":config.scene.snapshot_topic,"scene_state_topic":config.scene.state_topic,
                "timeline_ack_topic":config.scene.timeline_ack_topic,"timeline_status_topic":config.scene.timeline_status_topic}))?,
            "plan_dmpc" => {
                let mut cfg = table(&config.planner)?;
                cfg.insert("self_id".into(), (member.uav_id as i64).into()); cfg.insert("fleet_count".into(), (config.members.len() as i64).into());
                cfg.insert("timeline_authority".into(), origin(&config.mission_authority_node).into()); cfg
            },
            "dmpc_rounds" => table(serde_json::json!({
                "uav_id":member.uav_id,"participant_ids":config.members.iter().map(|m|m.uav_id).collect::<Vec<_>>(),
                "origins":config.members.iter().map(|m|origin(&m.planner_node)).collect::<Vec<_>>(),
                "control_origin":origin(&member.control_node),"plan_n":config.planner.state_dim,
                "plan_horizon":config.planner.horizon,"plan_rest_count":0,
                "planner_period_ms":100,"planner_epoch_ns":config.epoch_ns,"timeline_mode":"ordered-timeline-v1",
                "session_id":deployment.session_id,"timeline_authority":origin(&config.mission_authority_node)}))?,
            _ => return Err("internal unsupported HIL role".into()),
        };
        plugin["config"] = cfg.into();
    }
    let text = toml::to_string_pretty(&value).map_err(|e| e.to_string())?;
    Manifest::from_toml_str(&text).map_err(|e| e.to_string())?.resolve().map_err(|e| e.to_string())?;
    Ok(text)
}
