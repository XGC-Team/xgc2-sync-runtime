//! Channel planning: which ports may share a channel.
//!
//! The planner works on plain data (port specs and channel specs), so the same code validates
//! a manifest before anything is loaded or started, validates a hot-plug request against the
//! live topology, and answers `--check`. Channels are created by their first port; every later
//! port must match kind, schema id, size and align exactly. A state channel has one writer, an
//! event channel any number. Reader and queue capacity are fixed when the channel is created.

use crate::channel::{Kind, PayloadSpec};
use crate::loader::{Dir, PortSpec};
use std::collections::{BTreeMap, HashMap};

/// Simultaneous readers a channel supports unless the manifest says otherwise.
pub const DEFAULT_MAX_READERS: u32 = 8;
pub const MAX_READERS_LIMIT: u32 = 64;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ChannelSpec {
    pub kind: Kind,
    pub payload: PayloadSpec,
    /// Event queue length; 0 for state channels.
    pub depth: u32,
    pub max_readers: u32,
    pub writers: u32,
    pub readers: u32,
    /// Survives having no bound port (the clock channel).
    pub keep: bool,
}

/// Manifest `[[channel]]` overrides, applied when the channel is created.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Hint {
    pub depth: Option<u32>,
    pub max_readers: Option<u32>,
}

/// Channel name for every port that gets one: explicit bindings, and a private
/// `<instance>.<port>` channel for outputs without one. Inputs without a binding stay
/// unconnected. Bindings naming a port the module does not have are an error.
pub fn port_channels(
    instance: &str,
    ports: &[PortSpec],
    bind: &BTreeMap<String, String>,
    module: &str,
) -> Result<Vec<(usize, String)>, String> {
    for (port, channel) in bind {
        if !ports.iter().any(|spec| &spec.name == port) {
            return Err(format!("module {module} has no port {port}"));
        }
        if !crate::names::valid_name(channel) {
            return Err(format!("channel name {channel:?} is not a valid name"));
        }
    }
    Ok(ports
        .iter()
        .enumerate()
        .filter_map(|(index, port)| match (bind.get(&port.name), port.dir) {
            (Some(channel), _) => Some((index, channel.clone())),
            (None, Dir::Out) => Some((index, format!("{instance}.{}", port.name))),
            (None, Dir::In) => None,
        })
        .collect())
}

pub struct Planner {
    specs: BTreeMap<String, ChannelSpec>,
    hints: HashMap<String, Hint>,
    /// Queue length the ports of the whole manifest ask for, so the first port that creates
    /// an event channel does not fix a depth that a later port cannot use.
    expected_depth: HashMap<String, u32>,
}

fn describe(port: &PortSpec) -> String {
    format!("{} {} {} ({} bytes, align {})", port.kind.as_str(), port.payload.schema, port.name, port.payload.size, port.payload.align)
}

impl Planner {
    pub fn new(existing: BTreeMap<String, ChannelSpec>, hints: HashMap<String, Hint>) -> Self {
        Planner { specs: existing, hints, expected_depth: HashMap::new() }
    }

    /// Start with the queue lengths a pre-pass over the whole manifest found.
    pub fn with_expected_depths(mut self, depths: HashMap<String, u32>) -> Self {
        self.expected_depth = depths;
        self
    }

    pub fn specs(&self) -> &BTreeMap<String, ChannelSpec> {
        &self.specs
    }

    pub fn into_specs(self) -> BTreeMap<String, ChannelSpec> {
        self.specs
    }

    /// Pre-pass: remember the queue length `port` needs on `channel`.
    pub fn expect_depth(&mut self, channel: &str, port: &PortSpec) {
        if port.kind == Kind::Event {
            let depth = self.expected_depth.entry(channel.to_owned()).or_insert(0);
            *depth = (*depth).max(port.queue_depth);
        }
    }

    /// Create the clock channel up front.
    pub fn add_clock_channel(&mut self, name: &str) {
        self.specs.insert(
            name.to_owned(),
            ChannelSpec {
                kind: Kind::State,
                payload: PayloadSpec {
                    schema: crate::clock::CLOCK_SCHEMA.to_owned(),
                    size: crate::clock::CLOCK_PAYLOAD_SIZE,
                    align: crate::clock::CLOCK_PAYLOAD_ALIGN,
                },
                depth: 0,
                max_readers: self.hints.get(name).and_then(|hint| hint.max_readers).unwrap_or(DEFAULT_MAX_READERS),
                writers: 0,
                readers: 0,
                keep: true,
            },
        );
    }

    /// Bind `port` of `instance` to `channel`, creating the channel if it does not exist.
    pub fn bind(&mut self, instance: &str, port: &PortSpec, channel: &str) -> Result<(), String> {
        let who = format!("instance {instance}: port {} -> channel {channel}", port.name);
        if !self.specs.contains_key(channel) {
            let hint = self.hints.get(channel).cloned().unwrap_or_default();
            let depth = match port.kind {
                Kind::State => 0,
                Kind::Event => {
                    let wanted = self.expected_depth.get(channel).copied().unwrap_or(0).max(port.queue_depth);
                    let depth = hint.depth.unwrap_or(wanted);
                    if depth < wanted {
                        return Err(format!("{who}: manifest depth {depth} is below the {wanted} the ports ask for"));
                    }
                    depth
                }
            };
            if hint.depth.is_some() && port.kind == Kind::State {
                return Err(format!("{who}: depth applies to event channels only"));
            }
            self.specs.insert(
                channel.to_owned(),
                ChannelSpec {
                    kind: port.kind,
                    payload: port.payload.clone(),
                    depth,
                    max_readers: hint.max_readers.unwrap_or(DEFAULT_MAX_READERS),
                    writers: 0,
                    readers: 0,
                    keep: false,
                },
            );
        }
        let spec = self.specs.get_mut(channel).expect("channel exists");
        if spec.kind != port.kind || spec.payload != port.payload {
            return Err(format!(
                "{who}: port is {} but the channel carries {} {} ({} bytes, align {})",
                describe(port),
                spec.kind.as_str(),
                spec.payload.schema,
                spec.payload.size,
                spec.payload.align
            ));
        }
        if port.kind == Kind::Event && port.queue_depth > spec.depth {
            return Err(format!("{who}: port needs queue depth {} but the channel was created with {}", port.queue_depth, spec.depth));
        }
        match port.dir {
            Dir::Out => {
                if spec.kind == Kind::State && spec.writers >= 1 {
                    return Err(format!("{who}: state channel already has a writer"));
                }
                spec.writers += 1;
            }
            Dir::In => {
                if spec.readers >= spec.max_readers {
                    return Err(format!("{who}: channel already has its {} readers", spec.max_readers));
                }
                spec.readers += 1;
            }
        }
        Ok(())
    }

    /// Remove a binding made by [`Planner::bind`]; a channel without ports disappears.
    pub fn unbind(&mut self, port: &PortSpec, channel: &str) {
        let Some(spec) = self.specs.get_mut(channel) else { return };
        match port.dir {
            Dir::Out => spec.writers = spec.writers.saturating_sub(1),
            Dir::In => spec.readers = spec.readers.saturating_sub(1),
        }
        if spec.writers == 0 && spec.readers == 0 && !spec.keep {
            self.specs.remove(channel);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn port(name: &str, dir: Dir, kind: Kind, schema: &str, size: u32, depth: u32) -> PortSpec {
        PortSpec {
            name: name.into(),
            dir,
            kind,
            payload: PayloadSpec { schema: schema.into(), size, align: 8 },
            queue_depth: depth,
            required: false,
            async_writer: false,
        }
    }

    fn planner(hints: &HashMap<String, Hint>) -> Planner {
        Planner::new(BTreeMap::new(), hints.clone())
    }

    #[test]
    fn matching_ports_share_a_channel() {
        let hints = HashMap::new();
        let mut plan = planner(&hints);
        plan.bind("a", &port("out", Dir::Out, Kind::State, "s.v1", 16, 0), "c").unwrap();
        plan.bind("b", &port("in", Dir::In, Kind::State, "s.v1", 16, 0), "c").unwrap();
        plan.bind("c", &port("in", Dir::In, Kind::State, "s.v1", 16, 0), "c").unwrap();
        let spec = &plan.specs()["c"];
        assert_eq!((spec.writers, spec.readers, spec.depth), (1, 2, 0));
    }

    #[test]
    fn mismatches_are_rejected_with_the_offending_ports() {
        let hints = HashMap::new();
        let mut plan = planner(&hints);
        plan.bind("a", &port("out", Dir::Out, Kind::State, "s.v1", 16, 0), "c").unwrap();
        for (bad, expect) in [
            (port("in", Dir::In, Kind::State, "other.v1", 16, 0), "other.v1"),
            (port("in", Dir::In, Kind::State, "s.v1", 24, 0), "24 bytes"),
            (port("in", Dir::In, Kind::Event, "s.v1", 16, 4), "event"),
        ] {
            let error = plan.bind("b", &bad, "c").unwrap_err();
            assert!(error.contains(expect) && error.contains("instance b") && error.contains("channel c"), "{error}");
        }
        let mut other_align = port("in", Dir::In, Kind::State, "s.v1", 16, 0);
        other_align.payload.align = 16;
        assert!(plan.bind("b", &other_align, "c").is_err());
        assert_eq!(plan.specs()["c"].readers, 0, "failed binds change nothing");
    }

    #[test]
    fn state_channels_have_one_writer_and_events_many() {
        let hints = HashMap::new();
        let mut plan = planner(&hints);
        plan.bind("a", &port("out", Dir::Out, Kind::State, "s.v1", 8, 0), "s").unwrap();
        assert!(plan.bind("b", &port("out", Dir::Out, Kind::State, "s.v1", 8, 0), "s").unwrap_err().contains("already has a writer"));
        plan.bind("a", &port("ev", Dir::Out, Kind::Event, "e.v1", 8, 4), "e").unwrap();
        plan.bind("b", &port("ev", Dir::Out, Kind::Event, "e.v1", 8, 4), "e").unwrap();
        assert_eq!(plan.specs()["e"].writers, 2);
    }

    #[test]
    fn event_depth_follows_the_manifest_and_cannot_grow_later() {
        let hints = HashMap::new();
        let mut plan = planner(&hints);
        let small = port("ev", Dir::Out, Kind::Event, "e.v1", 8, 4);
        let large = port("ev", Dir::In, Kind::Event, "e.v1", 8, 32);
        plan.expect_depth("e", &small);
        plan.expect_depth("e", &large);
        plan.bind("a", &small, "e").unwrap();
        plan.bind("b", &large, "e").unwrap();
        assert_eq!(plan.specs()["e"].depth, 32);

        let mut late = planner(&hints);
        late.bind("a", &small, "e").unwrap();
        assert!(late.bind("b", &large, "e").unwrap_err().contains("queue depth 32"));
    }

    #[test]
    fn hints_override_depth_and_reader_capacity() {
        let mut hints = HashMap::new();
        hints.insert("e".to_owned(), Hint { depth: Some(64), max_readers: Some(1) });
        hints.insert("low".to_owned(), Hint { depth: Some(2), max_readers: None });
        hints.insert("s".to_owned(), Hint { depth: Some(2), max_readers: None });
        let mut plan = planner(&hints);
        let out = port("ev", Dir::Out, Kind::Event, "e.v1", 8, 4);
        plan.bind("a", &out, "e").unwrap();
        assert_eq!(plan.specs()["e"].depth, 64);
        plan.bind("b", &port("ev", Dir::In, Kind::Event, "e.v1", 8, 4), "e").unwrap();
        assert!(plan.bind("c", &port("ev", Dir::In, Kind::Event, "e.v1", 8, 4), "e").unwrap_err().contains("readers"));
        assert!(plan.bind("a", &out, "low").unwrap_err().contains("below the 4"));
        assert!(plan.bind("a", &port("o", Dir::Out, Kind::State, "s.v1", 8, 0), "s").unwrap_err().contains("event channels only"));
    }

    #[test]
    fn unbinding_the_last_port_removes_the_channel_but_not_the_clock() {
        let hints = HashMap::new();
        let mut plan = planner(&hints);
        let out = port("out", Dir::Out, Kind::State, "s.v1", 8, 0);
        plan.bind("a", &out, "c").unwrap();
        plan.unbind(&out, "c");
        assert!(plan.specs().is_empty());
        plan.add_clock_channel("clock");
        let clock_out = port("t", Dir::Out, Kind::State, crate::clock::CLOCK_SCHEMA, 8, 0);
        plan.bind("sim", &clock_out, "clock").unwrap();
        plan.unbind(&clock_out, "clock");
        assert!(plan.specs().contains_key("clock"));
        let wrong = port("t", Dir::Out, Kind::State, "other.v1", 8, 0);
        assert!(plan.bind("sim", &wrong, "clock").is_err());
    }
}
