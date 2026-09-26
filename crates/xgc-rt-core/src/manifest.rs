//! Host manifest (TOML). It declares the Session identity, roster, rounds,
//! transport, audit, channels and plugins with their port bindings. A
//! manifest names everything, and `resolve` interns names to ids.
//!
//! ```toml
//! [session]
//! id = "demo"
//! node = "uav1"
//! roster = ["uav1", "uav2"]
//! period_ms = 50
//!
//! [transport]
//! kind = "loopback"
//!
//! [audit]
//! dir = "out/audit"
//!
//! [[channel]]
//! name = "dmpc/plan"
//! qos = "control"
//!
//! [[plugin]]
//! name = "planner"
//! path = "libplanner.so"
//! trigger = "on_round"
//! [plugin.bind]
//! plan_out = { channel = "dmpc/plan" }
//! plan_in = { channel = "dmpc/plan", from = ["uav2"] }
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;

use serde::Deserialize;

use crate::transport::{ChannelSpec, Qos};
use crate::{ChannelId, OriginId};

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    pub session: SessionSpec,
    pub transport: TransportSpec,
    pub audit: AuditSpec,
    /// Clock-bound probe (docs/time-model.md). Absent: the bound stays
    /// whatever the clock reports (0 on a single-host loopback run).
    pub clock: Option<ClockSpec>,
    #[serde(rename = "channel", default)]
    pub channels: Vec<ChannelDecl>,
    #[serde(rename = "plugin", default)]
    pub plugins: Vec<PluginDecl>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SessionSpec {
    pub id: String,
    pub node: String,
    pub roster: Vec<String>,
    pub period_ms: f64,
    /// Publish deadline within a round; defaults to the full period.
    pub publish_deadline_ms: Option<f64>,
    /// Absolute epoch E0 in Session ns. From Z3 the Session sets it; until
    /// then `start_delay_ms` after host start is used.
    pub epoch_ns: Option<i64>,
    #[serde(default = "default_start_delay_ms")]
    pub start_delay_ms: u64,
    /// Stop after this long past E0. None means run until signalled.
    pub run_for_ms: Option<u64>,
    /// How long startup waits for remote subscribers on every out-channel.
    #[serde(default = "default_peer_timeout_ms")]
    pub peer_timeout_ms: u64,
    /// The aggregator stops after this many hung modules were abandoned, so
    /// the Agent restarts the whole process.
    #[serde(default = "default_max_abandoned")]
    pub max_abandoned: u32,
}

fn default_peer_timeout_ms() -> u64 {
    5_000
}

fn default_max_abandoned() -> u32 {
    2
}

fn default_start_delay_ms() -> u64 {
    500
}

#[derive(Debug, Clone, Deserialize)]
pub struct TransportSpec {
    pub kind: String,
    #[serde(flatten)]
    pub options: toml::Table,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AuditSpec {
    pub dir: PathBuf,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClockRole {
    /// The reference (the station): answers probes, bound 0.
    Server,
    /// Probes the server, gates activation on the bound.
    Client,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClockSpec {
    pub role: ClockRole,
    /// Roster name of the server.
    pub server: String,
    #[serde(default = "default_probe_interval_ms")]
    pub interval_ms: u64,
    /// Activation gate: the bound must be at or under this.
    #[serde(default = "default_gate_ms")]
    pub gate_ms: f64,
    #[serde(default = "default_gate_timeout_ms")]
    pub gate_timeout_ms: u64,
    #[serde(default = "default_probe_window")]
    pub window: usize,
}

fn default_probe_interval_ms() -> u64 {
    1_000
}
fn default_gate_ms() -> f64 {
    2.0
}
fn default_gate_timeout_ms() -> u64 {
    5_000
}
fn default_probe_window() -> usize {
    8
}

/// Reserved channels the host appends when `[clock]` is set.
pub const CLOCK_REQ_CHANNEL: &str = "xgc/clock/req";
pub const CLOCK_REP_CHANNEL: &str = "xgc/clock/rep";

#[derive(Debug, Clone, PartialEq)]
pub struct ResolvedClock {
    pub role: ClockRole,
    pub server: OriginId,
    pub req: ChannelId,
    pub rep: ChannelId,
    pub interval_ns: i64,
    pub gate_ns: i64,
    pub gate_timeout_ns: i64,
    pub window: usize,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ChannelDecl {
    pub name: String,
    pub qos: Qos,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Trigger {
    /// Step once per round.
    OnRound,
    /// Step only when an in-port has unread samples.
    OnDirty,
    /// Both.
    Both,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RestartKind {
    Never,
    OnError,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RestartPolicy {
    pub policy: RestartKind,
    #[serde(default)]
    pub max: u32,
    #[serde(default)]
    pub backoff_ms: u64,
}

impl Default for RestartPolicy {
    fn default() -> Self {
        Self { policy: RestartKind::Never, max: 0, backoff_ms: 0 }
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PluginDecl {
    pub name: String,
    pub path: PathBuf,
    /// Hex sha256 of the library. When set, loading refuses any other bytes.
    pub sha256: Option<String>,
    pub trigger: Trigger,
    #[serde(default)]
    pub restart: RestartPolicy,
    /// Longest normal step. Longer marks the module Degraded; 10× longer is
    /// a hang. Default: one period.
    pub step_budget_ms: Option<f64>,
    #[serde(default)]
    pub config: toml::Table,
    #[serde(default)]
    pub bind: BTreeMap<String, Binding>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Binding {
    pub channel: String,
    /// For an in-port: the origins to receive from. Default: every other
    /// roster node. This node's own name means the module in this process
    /// that writes the channel; the handoff is then in memory.
    pub from: Option<Vec<String>>,
    /// For an in-port: keep only the newest unread sample instead of
    /// queueing every sample.
    #[serde(default)]
    pub latest: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ManifestError(pub String);

impl std::fmt::Display for ManifestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ManifestError {}

/// Names interned to ids and cross-checked.
#[derive(Debug, Clone)]
pub struct Resolved {
    pub node_id: OriginId,
    pub channels: Vec<ChannelSpec>,
    pub period_ns: i64,
    pub publish_deadline_ns: i64,
    /// Per plugin: port name → (channel id, origins; empty for out-ports).
    pub bindings: Vec<BTreeMap<String, (ChannelId, Vec<OriginId>)>>,
    pub clock: Option<ResolvedClock>,
}

impl Manifest {
    pub fn from_toml_str(text: &str) -> Result<Self, ManifestError> {
        toml::from_str(text).map_err(|e| ManifestError(format!("manifest: {e}")))
    }

    pub fn resolve(&self) -> Result<Resolved, ManifestError> {
        let err = |m: String| Err(ManifestError(m));
        let s = &self.session;
        if s.roster.is_empty() || s.roster.len() > OriginId::MAX as usize {
            return err(format!("roster must have 1..={} nodes", OriginId::MAX));
        }
        let mut seen = BTreeSet::new();
        for name in &s.roster {
            if !valid_name(name) || !seen.insert(name) {
                return err(format!("roster node {name:?} is invalid or repeated"));
            }
        }
        let Some(node_id) = s.roster.iter().position(|n| *n == s.node) else {
            return err(format!("node {:?} is not in the roster", s.node));
        };
        if !(s.period_ms.is_finite() && s.period_ms > 0.0) {
            return err("period_ms must be positive".into());
        }
        let period_ns = (s.period_ms * 1e6).round() as i64;
        let publish_deadline_ns = match s.publish_deadline_ms {
            None => period_ns,
            Some(ms) if ms.is_finite() && ms > 0.0 && ms * 1e6 <= period_ns as f64 => (ms * 1e6).round() as i64,
            Some(_) => return err("publish_deadline_ms must be in (0, period_ms]".into()),
        };

        let mut channels = Vec::new();
        let mut channel_ids = BTreeMap::new();
        for (i, c) in self.channels.iter().enumerate() {
            if !valid_channel(&c.name) || channel_ids.insert(c.name.clone(), i as ChannelId).is_some() {
                return err(format!("channel {:?} is invalid or repeated", c.name));
            }
            channels.push(ChannelSpec { id: i as ChannelId, name: c.name.clone(), qos: c.qos });
        }

        let clock = match &self.clock {
            None => None,
            Some(c) => {
                let Some(server) = s.roster.iter().position(|n| *n == c.server) else {
                    return err(format!("clock server {:?} is not in the roster", c.server));
                };
                if (c.role == ClockRole::Server) != (server == node_id) {
                    return err("clock role must be server exactly on the server node".into());
                }
                if c.interval_ms == 0 || !(c.gate_ms.is_finite() && c.gate_ms > 0.0) || c.window == 0 {
                    return err("clock interval_ms, gate_ms and window must be positive".into());
                }
                let mut add = |name: &str| -> Result<ChannelId, ManifestError> {
                    if channel_ids.contains_key(name) {
                        return Err(ManifestError(format!("channel {name} is reserved for the clock probe")));
                    }
                    let id = channels.len() as ChannelId;
                    channel_ids.insert(name.to_string(), id);
                    channels.push(ChannelSpec { id, name: name.to_string(), qos: Qos::Control });
                    Ok(id)
                };
                let req = add(CLOCK_REQ_CHANNEL)?;
                let rep = add(CLOCK_REP_CHANNEL)?;
                Some(ResolvedClock {
                    role: c.role,
                    server: server as OriginId,
                    req,
                    rep,
                    interval_ns: c.interval_ms as i64 * 1_000_000,
                    gate_ns: (c.gate_ms * 1e6) as i64,
                    gate_timeout_ns: c.gate_timeout_ms as i64 * 1_000_000,
                    window: c.window,
                })
            }
        };

        let mut plugin_names = BTreeSet::new();
        let mut bindings = Vec::new();
        for p in &self.plugins {
            if !valid_name(&p.name) || !plugin_names.insert(&p.name) {
                return err(format!("plugin {:?} is invalid or repeated", p.name));
            }
            if p.step_budget_ms.is_some_and(|ms| !(ms.is_finite() && ms > 0.0)) {
                return err(format!("plugin {}: step_budget_ms must be positive", p.name));
            }
            let mut ports = BTreeMap::new();
            for (port, b) in &p.bind {
                let Some(&channel) = channel_ids.get(&b.channel) else {
                    return err(format!("plugin {} port {port}: unknown channel {:?}", p.name, b.channel));
                };
                let origins = match &b.from {
                    None => (0..s.roster.len()).filter(|&i| i != node_id).map(|i| i as OriginId).collect(),
                    Some(from) => {
                        let mut ids = Vec::new();
                        for name in from {
                            match s.roster.iter().position(|n| n == name) {
                                Some(i) if !ids.contains(&(i as OriginId)) => ids.push(i as OriginId),
                                _ => return err(format!("plugin {} port {port}: bad origin {name:?}", p.name)),
                            }
                        }
                        ids
                    }
                };
                ports.insert(port.clone(), (channel, origins));
            }
            bindings.push(ports);
        }
        Ok(Resolved { node_id: node_id as OriginId, channels, period_ns, publish_deadline_ns, bindings, clock })
    }
}

fn valid_name(s: &str) -> bool {
    !s.is_empty() && s.len() <= 64 && s.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'-' || b == b'_')
}

/// Channel names may use `/` between segments, and each segment is a valid
/// name. They become Zenoh key segments, so `*`, `$`, `#`, `?` and empty
/// segments are excluded.
fn valid_channel(s: &str) -> bool {
    s.len() <= 128 && s.split('/').all(valid_name)
}

#[cfg(test)]
mod tests {
    use super::*;

    const BASE: &str = r#"
[session]
id = "s1"
node = "uav1"
roster = ["uav1", "uav2", "uav3"]
period_ms = 50

[transport]
kind = "loopback"

[audit]
dir = "out"

[[channel]]
name = "dmpc/plan"
qos = "control"

[[plugin]]
name = "planner"
path = "libplanner.so"
trigger = "on_round"
[plugin.bind]
plan_out = { channel = "dmpc/plan" }
plan_in = { channel = "dmpc/plan" }
only_uav3 = { channel = "dmpc/plan", from = ["uav3"] }
"#;

    #[test]
    fn resolves_ids_and_default_origins() {
        let r = Manifest::from_toml_str(BASE).unwrap().resolve().unwrap();
        assert_eq!(r.node_id, 0);
        assert_eq!(r.period_ns, 50_000_000);
        assert_eq!(r.publish_deadline_ns, 50_000_000);
        assert_eq!(r.bindings[0]["plan_in"], (0, vec![1, 2]));
        assert_eq!(r.bindings[0]["only_uav3"], (0, vec![2]));
    }

    #[test]
    fn rejects_unknown_fields_bad_names_and_references() {
        assert!(Manifest::from_toml_str(&BASE.replace("period_ms = 50", "period_ms = 50\nbogus = 1")).is_err());
        for bad in [
            BASE.replace("node = \"uav1\"", "node = \"uav9\""),
            BASE.replace("name = \"dmpc/plan\"", "name = \"dmpc/*\""),
            BASE.replace("from = [\"uav3\"]", "from = [\"uav7\"]"),
            BASE.replace("period_ms = 50", "period_ms = 50\npublish_deadline_ms = 60"),
        ] {
            assert!(Manifest::from_toml_str(&bad).unwrap().resolve().is_err(), "accepted:\n{bad}");
        }
    }
}
