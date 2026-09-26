//! Fixed DMPC graphs. SMC does not use hover-thrust or an attitude-rate channel.
use super::*;
use std::collections::BTreeSet;
use std::path::Path;

#[derive(Clone, Copy)]
enum Place {
    Colocated,
    Planner,
    Control,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleTopics {
    pub imu_topic: String,
    pub pose_topic: String,
    pub vision_pose_topic: String,
    pub fcu_state_topic: String,
    pub local_pose_topic: String,
    pub local_velocity_topic: String,
    pub fcu_imu_topic: String,
    pub battery_topic: String,
    pub setpoint_topic: String,
    pub status_topic: String,
    pub fcu_request_topic: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NativeConfiguration {
    pub input_time_domain: TimeDomain,
    pub ros_master_uri: String,
    pub ros_ip: String,
    pub epoch_ns: i64,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "super::present")]
    pub simulation: Option<Simulation>,
    pub members: Vec<RobotMember>,
    pub mission_authority_node: String,
    pub radio: Radio,
    pub station: Station,
    pub scene: Scene,
    pub planner: Planner,
    pub takeoff_altitude_m: f64,
    pub topics: VehicleTopics,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PlannerConfiguration {
    pub input_time_domain: TimeDomain,
    pub ros_master_uri: String,
    pub ros_ip: String,
    pub epoch_ns: i64,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "super::present")]
    pub simulation: Option<Simulation>,
    pub members: Vec<RobotMember>,
    pub mission_authority_node: String,
    pub radio: Radio,
    pub station: Station,
    pub scene: Scene,
    pub planner: Planner,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SmcConfiguration {
    pub input_time_domain: TimeDomain,
    pub ros_master_uri: String,
    pub ros_ip: String,
    pub epoch_ns: i64,
    #[serde(default, skip_serializing_if = "Option::is_none", deserialize_with = "super::present")]
    pub simulation: Option<Simulation>,
    pub members: Vec<RobotMember>,
    pub mission_authority_node: String,
    pub radio: Radio,
    pub station: Station,
    pub takeoff_altitude_m: f64,
    pub topics: VehicleTopics,
}

fn tcp(endpoint: &str) -> bool {
    endpoint.strip_prefix("tcp/").and_then(|s| s.rsplit_once(':')).is_some_and(|(host, port)| {
        !host.is_empty()
            && host.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'.' || b == b'-')
            && !port.is_empty()
            && port.bytes().all(|b| b.is_ascii_digit())
            && port.parse::<u16>().is_ok_and(|p| p > 0)
    })
}

fn roster(members: &[RobotMember]) -> Vec<String> {
    let mut nodes = Vec::new();
    for member in members {
        for node in [&member.planner_node, &member.control_node] {
            if !nodes.contains(node) {
                nodes.push(node.clone());
            }
        }
    }
    nodes
}

fn validate_clock(domain: &TimeDomain, simulation: &Option<Simulation>, epoch_ns: i64) -> Result<()> {
    require(epoch_ns > 0 && epoch_ns < i64::MAX - 10_000_000_000, "DMPC requires positive shared epoch_ns")?;
    require(
        matches!((domain, simulation), (TimeDomain::WallUnix, None) | (TimeDomain::Ros1Sim, Some(_))),
        "ros1-sim requires simulation; wall-unix forbids simulation",
    )?;
    if let Some(sim) = simulation {
        require(sim.epoch_ns == epoch_ns, "planner and SimClock epochs differ")?;
    }
    Ok(())
}

fn validate_fleet(members: &[RobotMember], deployment: &Deployment, authority: &str, place: Place) -> Result<RobotMember> {
    require(!members.is_empty() && members.len() <= u16::MAX as usize / 2, "invalid DMPC members")?;
    let (mut ids, mut robots, mut nodes) = (BTreeSet::new(), BTreeSet::new(), BTreeSet::new());
    for member in members {
        require(
            member.uav_id > 0 && ids.insert(member.uav_id) && ros_namespace(&member.robot_namespace) && robots.insert(&member.robot_namespace),
            "invalid or repeated robot identity",
        )?;
        require(name(&member.planner_node) && name(&member.control_node) && nodes.insert(&member.planner_node), "invalid or shared execution node")?;
        match place {
            Place::Colocated => require(member.planner_node == member.control_node, "full DMPC requires co-located planner and control nodes")?,
            Place::Planner | Place::Control => {
                require(member.planner_node != member.control_node, "split DMPC requires distinct planner and control nodes")?;
                require(nodes.insert(&member.control_node), "shared execution node")?;
            }
        }
    }
    let member = members.iter().find(|m| m.role(&deployment.node_id).is_some()).cloned().ok_or("node is not an experiment member")?;
    require(member.robot_namespace == deployment.robot_namespace, "node does not own selected robot namespace")?;
    match place {
        Place::Colocated => require(member.role(&deployment.node_id) == Some(NodeRole::CoLocated), "full DMPC must run on the co-located node")?,
        Place::Planner => require(member.role(&deployment.node_id) == Some(NodeRole::Planner), "planner composition must run on the planner node")?,
        Place::Control => require(member.role(&deployment.node_id) == Some(NodeRole::Control), "SMC composition must run on the control node")?,
    }
    require(members.iter().any(|m| m.planner_node == authority), "mission authority is not a planner node")?;
    Ok(member)
}

fn validate_radio_station(radio: &Radio, station: &Station) -> Result<()> {
    require(!radio.listen.is_empty() && radio.listen.iter().chain(&radio.connect).all(|e| tcp(e)), "explicit radio TCP endpoints required")?;
    require(
        station.robot_id.len() >= 3
            && station.robot_id.len() <= 128
            && station.robot_id.as_bytes()[0].is_ascii_lowercase()
            && station.robot_id.bytes().all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b == b'-')
            && station.robot_id.as_bytes().last().is_some_and(u8::is_ascii_alphanumeric)
            && !["uav", "ugv"].iter().any(|p| station.robot_id.strip_prefix(p).is_some_and(|s| !s.is_empty() && s.bytes().all(|b| b.is_ascii_digit()))),
        "station robot_id must identify an experiment instance",
    )?;
    require(tcp(&station.zenoh_connect), "station requires its GCS TCP endpoint")?;
    require(absolute(Path::new(&station.command_socket)) && station.command_socket.len() <= 100, "station command_socket must be an absolute path at most 100 bytes")
}

fn validate_scene(scene: &Scene, namespace: &str) -> Result<()> {
    let mut topics = BTreeSet::new();
    for value in [&scene.snapshot_topic, &scene.state_topic, &scene.timeline_ack_topic, &scene.timeline_status_topic] {
        require(topic(value) && topics.insert(value.as_str()), "invalid or repeated DMPC scene/status topic")?;
    }
    for value in [&scene.timeline_ack_topic, &scene.timeline_status_topic] {
        require(value.starts_with(&format!("/{namespace}/")), "DMPC status topic outside robot namespace")?;
    }
    Ok(())
}

fn validate_planner(planner: &Planner) -> Result<()> {
    planner.require_configured()
}

fn validate_vehicle(topics: &VehicleTopics, takeoff: f64, namespace: &str) -> Result<()> {
    require(takeoff.is_finite() && takeoff > 0.0, "takeoff_altitude_m must be finite and positive")?;
    let fields = serde_json::to_value(topics).map_err(|e| e.to_string())?;
    let mut seen = BTreeSet::new();
    for (role, value) in fields.as_object().ok_or("internal vehicle topic schema")? {
        let value = value.as_str().ok_or("internal vehicle topic type")?;
        require(topic(value), &format!("invalid absolute ROS name for {role}"))?;
        require(seen.insert(value), "topic/service roles must have distinct names")?;
        if role != "pose_topic" {
            require(value.starts_with(&format!("/{namespace}/")), &format!("{role} is outside the selected robot namespace"))?;
        }
    }
    Ok(())
}

impl NativeConfiguration {
    pub(super) fn validate(&self, deployment: &Deployment) -> Result<()> {
        ros_endpoint(&self.ros_master_uri, &self.ros_ip)?;
        validate_clock(&self.input_time_domain, &self.simulation, self.epoch_ns)?;
        validate_fleet(&self.members, deployment, &self.mission_authority_node, Place::Colocated)?;
        validate_radio_station(&self.radio, &self.station)?;
        validate_scene(&self.scene, &deployment.robot_namespace)?;
        validate_planner(&self.planner)?;
        validate_vehicle(&self.topics, self.takeoff_altitude_m, &deployment.robot_namespace)
    }
}

impl PlannerConfiguration {
    pub(super) fn validate(&self, deployment: &Deployment) -> Result<()> {
        ros_endpoint(&self.ros_master_uri, &self.ros_ip)?;
        validate_clock(&self.input_time_domain, &self.simulation, self.epoch_ns)?;
        validate_fleet(&self.members, deployment, &self.mission_authority_node, Place::Planner)?;
        validate_radio_station(&self.radio, &self.station)?;
        validate_scene(&self.scene, &deployment.robot_namespace)?;
        validate_planner(&self.planner)
    }
}

impl SmcConfiguration {
    pub(super) fn validate(&self, deployment: &Deployment) -> Result<()> {
        ros_endpoint(&self.ros_master_uri, &self.ros_ip)?;
        validate_clock(&self.input_time_domain, &self.simulation, self.epoch_ns)?;
        validate_fleet(&self.members, deployment, &self.mission_authority_node, Place::Control)?;
        validate_radio_station(&self.radio, &self.station)?;
        validate_vehicle(&self.topics, self.takeoff_altitude_m, &deployment.robot_namespace)
    }
}

fn val<T: Serialize>(value: T) -> Result<toml::Value> {
    toml::Value::try_from(value).map_err(|e| e.to_string())
}
fn table<T: Serialize>(value: T) -> Result<toml::Table> {
    val(value)?.as_table().cloned().ok_or_else(|| "internal DMPC configuration table".into())
}

struct Graph<'a> {
    composition: &'static str,
    members: &'a [RobotMember],
    authority: &'a str,
    epoch_ns: i64,
    simulation: Option<&'a Simulation>,
    radio: &'a Radio,
}

fn render_graph(deployment: &Deployment, bundle: &Bundle, root: &Path, audit: &Path, graph: Graph<'_>, mut fill: impl FnMut(&str, &mut toml::Table) -> Result<()>) -> Result<String> {
    let mut value: toml::Value = graph.composition.parse().map_err(|e| format!("internal DMPC composition: {e}"))?;
    let nodes = roster(graph.members);
    let member = graph.members.iter().find(|m| m.role(&deployment.node_id).is_some()).ok_or("missing member")?;
    value["session"]["id"] = deployment.session_id.clone().into();
    value["session"]["node"] = deployment.node_id.clone().into();
    value["session"]["roster"] = val(&nodes)?;
    value["session"]["epoch_ns"] = graph.epoch_ns.into();
    value["audit"]["dir"] = audit.to_str().ok_or("non-UTF8 audit path")?.into();
    value["transport"]["listen"] = val(&graph.radio.listen)?;
    value["transport"]["connect"] = val(&graph.radio.connect)?;
    if let Some(sim) = graph.simulation {
        let mut source = table(sim)?;
        source.remove("epoch_ns");
        source.insert("kind".into(), "ros1_sim".into());
        source.insert("plugin".into(), "ros_io".into());
        value.as_table_mut().unwrap().insert("clock_source".into(), source.into());
    }
    for plugin in value["plugin"].as_array_mut().ok_or("internal DMPC plugins")? {
        let role = plugin["path"].as_str().ok_or("internal DMPC role")?.to_owned();
        let pin = bundle.plugins.entries().into_iter().find(|(r, _)| *r == role).ok_or("missing DMPC artifact role")?.1;
        require(relative(&pin.path) && pin.path.starts_with("plugins/") && digest(&pin.sha256), "invalid plugin pin")?;
        plugin["path"] = root.join(&pin.path).to_str().ok_or("non-UTF8 plugin path")?.into();
        plugin.as_table_mut().unwrap().insert("sha256".into(), pin.sha256.clone().into());
        let (command_bound, mission_bound) = {
        let binds = plugin["bind"].as_table_mut().ok_or("internal DMPC bindings")?;
        for (_, binding) in binds.iter_mut() {
            if let Some(from) = binding.get_mut("from") {
                let selector = from.as_array().and_then(|a| a.first()).and_then(|s| s.as_str()).ok_or("internal origin selector")?;
                *from = val(match selector {
                    "SELF" => vec![deployment.node_id.clone()],
                    "AUTHORITY" => vec![graph.authority.to_string()],
                    "PEERS" => graph.members.iter().filter(|m| m.planner_node != deployment.node_id).map(|m| m.planner_node.clone()).collect(),
                    "CONTROL" => vec![member.control_node.clone()],
                    "PLANNER" => vec![member.planner_node.clone()],
                    _ => return Err("internal DMPC origin selector".into()),
                })?;
            }
        }
        if role == "station_io" && deployment.node_id != graph.authority {
            binds.remove("command");
            binds.remove("mission_request");
        }
        (binds.contains_key("command"), binds.contains_key("mission_request"))
        };
        let mut cfg = plugin["config"].as_table().cloned().unwrap_or_default();
        fill(&role, &mut cfg)?;
        if role == "station_io" {
            cfg.insert("command".into(), command_bound.into());
            cfg.insert("mission".into(), mission_bound.into());
        }
        plugin["config"] = cfg.into();
    }
    let text = toml::to_string_pretty(&value).map_err(|e| e.to_string())?;
    Manifest::from_toml_str(&text).map_err(|e| e.to_string())?.resolve().map_err(|e| e.to_string())?;
    Ok(text)
}

fn node_name(deployment: &Deployment) -> String {
    format!("xgc_rt_ros_{}", &sha256(format!("{}:{}", deployment.session_id, deployment.node_id).as_bytes())[..16])
}

fn insert_vehicle(cfg: &mut toml::Table, topics: &VehicleTopics, deployment: &Deployment) -> Result<()> {
    let mut ros = table(topics)?;
    ros.insert("node_name".into(), node_name(deployment).into());
    *cfg = ros;
    Ok(())
}

fn insert_scene(cfg: &mut toml::Table, scene: &Scene, deployment: &Deployment) -> Result<()> {
    cfg.insert("node_name".into(), node_name(deployment).into());
    cfg.insert("scene_snapshot_topic".into(), scene.snapshot_topic.clone().into());
    cfg.insert("scene_state_topic".into(), scene.state_topic.clone().into());
    cfg.insert("timeline_ack_topic".into(), scene.timeline_ack_topic.clone().into());
    cfg.insert("timeline_status_topic".into(), scene.timeline_status_topic.clone().into());
    Ok(())
}

fn insert_rounds(cfg: &mut toml::Table, deployment: &Deployment, members: &[RobotMember], planner: &Planner, epoch_ns: i64, authority: &str) -> Result<()> {
    let nodes = roster(members);
    let origin = |node: &str| nodes.iter().position(|n| n == node).expect("validated roster") as i64;
    let member = members.iter().find(|m| m.role(&deployment.node_id).is_some()).ok_or("missing member")?;
    *cfg = table(serde_json::json!({
        "uav_id": member.uav_id,
        "participant_ids": members.iter().map(|m| m.uav_id).collect::<Vec<_>>(),
        "origins": members.iter().map(|m| origin(&m.planner_node)).collect::<Vec<_>>(),
        "control_origin": origin(&member.control_node),
        "plan_n": planner.state_dim,
        "plan_horizon": planner.horizon,
        "plan_rest_count": 0,
        "planner_period_ms": 100,
        "planner_epoch_ns": epoch_ns,
        "timeline_mode": "ordered-timeline-v1",
        "session_id": deployment.session_id,
        "timeline_authority": origin(authority),
    }))?;
    Ok(())
}

fn insert_plan(cfg: &mut toml::Table, deployment: &Deployment, members: &[RobotMember], planner: &Planner, authority: &str) -> Result<()> {
    let nodes = roster(members);
    let origin = |node: &str| nodes.iter().position(|n| n == node).expect("validated roster") as i64;
    let member = members.iter().find(|m| m.role(&deployment.node_id).is_some()).ok_or("missing member")?;
    *cfg = super::hil::plan_plugin_config(planner, member.uav_id as i64, origin(authority))?;
    Ok(())
}

pub(super) fn render_native(deployment: &Deployment, config: &NativeConfiguration, bundle: &Bundle, root: &Path, audit: &Path) -> Result<String> {
    let graph = Graph {
        composition: NATIVE_COMPOSITION,
        members: &config.members,
        authority: &config.mission_authority_node,
        epoch_ns: config.epoch_ns,
        simulation: config.simulation.as_ref(),
        radio: &config.radio,
    };
    render_graph(deployment, bundle, root, audit, graph, |role, cfg| match role {
        "ros_io" => {
            insert_vehicle(cfg, &config.topics, deployment)?;
            insert_scene(cfg, &config.scene, deployment)
        }
        "controller" => {
            cfg.insert("takeoff_altitude".into(), config.takeoff_altitude_m.into());
            Ok(())
        }
        "plan_dmpc" => insert_plan(cfg, deployment, &config.members, &config.planner, &config.mission_authority_node),
        "dmpc_rounds" => insert_rounds(cfg, deployment, &config.members, &config.planner, config.epoch_ns, &config.mission_authority_node),
        "station_io" => {
            *cfg = table(&config.station)?;
            cfg.insert("authority".into(), (deployment.node_id == config.mission_authority_node).into());
            Ok(())
        }
        _ => Err("internal unsupported native DMPC role".into()),
    })
}

pub(super) fn render_planner(deployment: &Deployment, config: &PlannerConfiguration, bundle: &Bundle, root: &Path, audit: &Path) -> Result<String> {
    let graph = Graph {
        composition: PLANNER_COMPOSITION,
        members: &config.members,
        authority: &config.mission_authority_node,
        epoch_ns: config.epoch_ns,
        simulation: config.simulation.as_ref(),
        radio: &config.radio,
    };
    render_graph(deployment, bundle, root, audit, graph, |role, cfg| match role {
        "ros_io" => insert_scene(cfg, &config.scene, deployment),
        "plan_dmpc" => insert_plan(cfg, deployment, &config.members, &config.planner, &config.mission_authority_node),
        "dmpc_rounds" => insert_rounds(cfg, deployment, &config.members, &config.planner, config.epoch_ns, &config.mission_authority_node),
        "station_io" => {
            *cfg = table(&config.station)?;
            cfg.insert("authority".into(), (deployment.node_id == config.mission_authority_node).into());
            Ok(())
        }
        _ => Err("internal unsupported planner role".into()),
    })
}

pub(super) fn render_smc(deployment: &Deployment, config: &SmcConfiguration, bundle: &Bundle, root: &Path, audit: &Path) -> Result<String> {
    let graph = Graph {
        composition: SMC_COMPOSITION,
        members: &config.members,
        authority: &config.mission_authority_node,
        epoch_ns: config.epoch_ns,
        simulation: config.simulation.as_ref(),
        radio: &config.radio,
    };
    render_graph(deployment, bundle, root, audit, graph, |role, cfg| match role {
        "ros_io" => insert_vehicle(cfg, &config.topics, deployment),
        "controller" => {
            cfg.insert("takeoff_altitude".into(), config.takeoff_altitude_m.into());
            Ok(())
        }
        "station_io" => {
            *cfg = table(&config.station)?;
            cfg.insert("authority".into(), (deployment.node_id == config.mission_authority_node).into());
            Ok(())
        }
        _ => Err("internal unsupported SMC role".into()),
    })
}
