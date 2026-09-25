//! Zenoh transport (Z2). It moves envelope v2 frames on explicit keys:
//!
//! ```text
//! xgc/{session}/d/{channel}/{origin}
//! ```
//!
//! - One publisher per declared out-channel, and one subscriber per
//!   `(channel, origin)` the host asked for. Control loops never use
//!   wildcard subscriptions.
//! - The session runs in peer mode with multicast scouting and gossip off. It listens
//!   and connects only on the endpoints the manifest gives, which in
//!   deployment are the radio-network addresses. So this transport can
//!   never ride the physics network by discovery.
//! - QoS classes map as in docs/qos.md. The transport never stamps, audits
//!   or decodes: the host's sink does that.
//!
//! Manifest:
//! ```toml
//! [transport]
//! kind = "zenoh"
//! listen = ["tcp/172.30.251.101:7447"]
//! connect = ["tcp/172.30.251.102:7447", "tcp/172.30.251.103:7447"]
//! ```

use std::collections::HashMap;

use xgc_rt_core::transport::{Qos, RxSink, Transport, TransportContext, TransportError};
use xgc_rt_core::{ChannelId, OriginId};
use zenoh::pubsub::{Publisher, Subscriber};
use zenoh::qos::{CongestionControl, Priority, Reliability};
use zenoh::{Session, Wait};

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ZenohOptions {
    pub listen: Vec<String>,
    pub connect: Vec<String>,
}

impl ZenohOptions {
    /// Read `listen` and `connect` from the manifest's `[transport]` table.
    pub fn from_table(table: &toml::Table) -> Result<Self, TransportError> {
        let list = |key: &str| -> Result<Vec<String>, TransportError> {
            match table.get(key) {
                None => Ok(Vec::new()),
                Some(toml::Value::Array(items)) => items
                    .iter()
                    .map(|v| v.as_str().map(str::to_owned).ok_or_else(|| TransportError(format!("transport.{key}: strings only"))))
                    .collect(),
                Some(_) => Err(TransportError(format!("transport.{key} must be an array of endpoints"))),
            }
        };
        if let Some(extra) = table.keys().find(|k| !["listen", "connect"].contains(&k.as_str())) {
            return Err(TransportError(format!("transport.{extra} is not a zenoh option")));
        }
        Ok(Self { listen: list("listen")?, connect: list("connect")? })
    }
}

/// QoS class → Zenoh publisher settings.
pub fn qos_settings(qos: Qos) -> (Reliability, CongestionControl, Priority, bool) {
    match qos {
        Qos::Control => (Reliability::BestEffort, CongestionControl::Drop, Priority::RealTime, true),
        Qos::State => (Reliability::BestEffort, CongestionControl::Drop, Priority::InteractiveHigh, false),
        Qos::Event => (Reliability::Reliable, CongestionControl::Block, Priority::InteractiveLow, false),
        Qos::Bulk => (Reliability::Reliable, CongestionControl::Block, Priority::DataLow, false),
    }
}

pub fn data_key(session: &str, channel: &str, origin: &str) -> String {
    format!("xgc/{session}/d/{channel}/{origin}")
}

pub struct ZenohTransport {
    options: ZenohOptions,
    session: Option<Session>,
    ctx: Option<TransportContext>,
    sink: Option<RxSink>,
    publishers: HashMap<ChannelId, Publisher<'static>>,
    subscribers: Vec<Subscriber<()>>,
}

fn terr(what: &str, e: impl std::fmt::Display) -> TransportError {
    TransportError(format!("zenoh {what}: {e}"))
}

impl ZenohTransport {
    pub fn new(options: ZenohOptions) -> Self {
        Self { options, session: None, ctx: None, sink: None, publishers: HashMap::new(), subscribers: Vec::new() }
    }

    fn config(&self) -> Result<zenoh::Config, TransportError> {
        let mut config = zenoh::Config::default();
        let json = |v: &Vec<String>| serde_json_like(v);
        config.insert_json5("mode", "\"peer\"").map_err(|e| terr("config mode", e))?;
        config.insert_json5("scouting/multicast/enabled", "false").map_err(|e| terr("config scouting", e))?;
        // No gossip either: peers are exactly the manifest's endpoints, so a
        // session can never be rerouted around the radio path (or a relay).
        config.insert_json5("scouting/gossip/enabled", "false").map_err(|e| terr("config gossip", e))?;
        config.insert_json5("listen/endpoints", &json(&self.options.listen)).map_err(|e| terr("config listen", e))?;
        config.insert_json5("connect/endpoints", &json(&self.options.connect)).map_err(|e| terr("config connect", e))?;
        // Peers come and go at different times: keep retrying connects rather
        // than failing the host at startup.
        config.insert_json5("connect/exit_on_failure", "false").map_err(|e| terr("config connect retry", e))?;
        Ok(config)
    }

    fn parts(&self) -> Result<(&Session, &TransportContext), TransportError> {
        match (&self.session, &self.ctx) {
            (Some(s), Some(c)) => Ok((s, c)),
            _ => Err(TransportError("zenoh transport used before open".into())),
        }
    }
}

fn serde_json_like(items: &[String]) -> String {
    let quoted: Vec<String> = items.iter().map(|s| format!("{:?}", s)).collect();
    format!("[{}]", quoted.join(","))
}

impl Transport for ZenohTransport {
    fn kind(&self) -> &'static str {
        "zenoh"
    }

    fn open(&mut self, ctx: &TransportContext, sink: RxSink) -> Result<(), TransportError> {
        if self.options.listen.is_empty() && self.options.connect.is_empty() {
            return Err(TransportError("zenoh transport needs listen and/or connect endpoints (radio addresses)".into()));
        }
        let session = zenoh::open(self.config()?).wait().map_err(|e| terr("open", e))?;
        self.session = Some(session);
        self.ctx = Some(ctx.clone());
        self.sink = Some(sink);
        Ok(())
    }

    fn declare_out(&mut self, channel: ChannelId) -> Result<(), TransportError> {
        let (session, ctx) = self.parts()?;
        let spec = ctx.channel(channel).ok_or_else(|| TransportError(format!("unknown channel {channel}")))?;
        let key = data_key(&ctx.session, &spec.name, &ctx.node);
        let (reliability, congestion, priority, express) = qos_settings(spec.qos);
        let publisher = session
            .declare_publisher(key)
            .reliability(reliability)
            .congestion_control(congestion)
            .priority(priority)
            .express(express)
            .wait()
            .map_err(|e| terr("declare publisher", e))?;
        self.publishers.insert(channel, publisher);
        Ok(())
    }

    fn declare_in(&mut self, channel: ChannelId, origins: &[OriginId]) -> Result<(), TransportError> {
        let (session, ctx) = self.parts()?;
        let spec = ctx.channel(channel).ok_or_else(|| TransportError(format!("unknown channel {channel}")))?;
        let sink = self.sink.clone().ok_or_else(|| TransportError("no sink".into()))?;
        let mut declared = Vec::new();
        for &origin in origins {
            let origin_name = ctx.roster.get(origin as usize).ok_or_else(|| TransportError(format!("unknown origin {origin}")))?;
            let key = data_key(&ctx.session, &spec.name, origin_name);
            let sink = sink.clone();
            let subscriber = session
                .declare_subscriber(key)
                .callback(move |sample| {
                    let bytes = sample.payload().to_bytes();
                    sink(&bytes);
                })
                .wait()
                .map_err(|e| terr("declare subscriber", e))?;
            declared.push(subscriber);
        }
        self.subscribers.extend(declared);
        Ok(())
    }

    fn send(&mut self, channel: ChannelId, frame: &[u8]) -> Result<(), TransportError> {
        let publisher = self
            .publishers
            .get(&channel)
            .ok_or_else(|| TransportError(format!("channel {channel} was not declared for output")))?;
        publisher.put(frame.to_vec()).wait().map_err(|e| terr("put", e))
    }

    fn wait_ready(&mut self, timeout: std::time::Duration) -> bool {
        let deadline = std::time::Instant::now() + timeout;
        loop {
            let all = self
                .publishers
                .values()
                .all(|p| p.matching_status().wait().map(|m| m.matching()).unwrap_or(false));
            if all {
                return true;
            }
            if std::time::Instant::now() >= deadline {
                return false;
            }
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    fn close(&mut self) {
        self.subscribers.clear();
        self.publishers.clear();
        if let Some(session) = self.session.take() {
            let _ = session.close().wait();
        }
    }
}

impl Drop for ZenohTransport {
    fn drop(&mut self) {
        self.close();
    }
}
