//! Starting a host from a manifest, and checking a manifest without starting anything.

use crate::channel::Kind;
use crate::host::{HostError, ModuleHost};
use crate::loader::{self, Dir, Module};
use crate::manifest::{Manifest, ManifestError};
use crate::plan::{port_channels, Planner};
use serde_json::{json, Value};
use std::collections::{BTreeMap, HashMap};

/// Start a host, load the manifest's libraries and add its instances in document order.
/// Instances start in that order: list the consumers of an event channel before its
/// producers if events published during start-up must not be missed.
pub fn launch(manifest: &Manifest) -> Result<ModuleHost, HostError> {
    let host = ModuleHost::start(manifest.host_options())?;
    match populate(&host, manifest) {
        Ok(()) => Ok(host),
        Err(error) => {
            host.shutdown();
            Err(error)
        }
    }
}

fn populate(host: &ModuleHost, manifest: &Manifest) -> Result<(), HostError> {
    for module in &manifest.modules {
        host.load_module(Some(&module.name), &module.path, module.sha256.as_deref())?;
    }
    let specs = manifest.instance_specs();
    // The first port that creates an event channel must not fix a depth that a later port
    // of the manifest cannot use.
    let mut depths: HashMap<String, u32> = HashMap::new();
    for spec in &specs {
        let module = host.loaded_module(&spec.module)?;
        for (port, channel) in port_channels(&spec.name, &module.ports, &spec.bind, &module.name).map_err(HostError::Invalid)? {
            let port = &module.ports[port];
            if port.kind == Kind::Event {
                let depth = depths.entry(channel).or_insert(0);
                *depth = (*depth).max(port.queue_depth);
            }
        }
    }
    host.declare_channels(manifest.channel_hints(), depths);
    for spec in specs {
        host.add_instance(spec)?;
    }
    Ok(())
}

/// Load every library and plan every channel without creating an instance. Returns a
/// summary, or all problems found.
pub fn check(manifest: &Manifest) -> Result<Value, ManifestError> {
    let mut problems = Vec::new();
    let mut modules: BTreeMap<String, Module> = BTreeMap::new();
    for section in &manifest.modules {
        match loader::load(&section.path, section.sha256.as_deref()) {
            Ok(module) => {
                modules.insert(section.name.clone(), module);
            }
            Err(error) => problems.push(format!("module {}: {error}", section.name)),
        }
    }
    let specs = manifest.instance_specs();
    let mut planner = Planner::new(BTreeMap::new(), manifest.channel_hints());
    for spec in &specs {
        let Some(module) = modules.get(&spec.module) else { continue };
        match port_channels(&spec.name, &module.ports, &spec.bind, &module.name) {
            Ok(bindings) => {
                for (port, channel) in bindings {
                    planner.expect_depth(&channel, &module.ports[port]);
                }
            }
            Err(error) => problems.push(format!("instance {}: {error}", spec.name)),
        }
    }
    if let Some(channel) = &manifest.clock.channel {
        planner.add_clock_channel(channel);
    }
    for spec in &specs {
        let Some(module) = modules.get(&spec.module) else { continue };
        let Ok(bindings) = port_channels(&spec.name, &module.ports, &spec.bind, &module.name) else { continue };
        for (port, channel) in bindings {
            if let Err(error) = planner.bind(&spec.name, &module.ports[port], &channel) {
                problems.push(error);
            }
        }
    }
    let channels = planner.into_specs();
    let mut warnings = Vec::new();
    for spec in &specs {
        let Some(module) = modules.get(&spec.module) else { continue };
        for port in module.ports.iter().filter(|p| p.dir == Dir::In && p.required) {
            let produced = spec.bind.get(&port.name).is_some_and(|channel| channels.get(channel).is_some_and(|c| c.writers > 0));
            if !produced {
                warnings.push(format!("required input {}.{} has no producer in this manifest", spec.name, port.name));
            }
        }
    }
    if !problems.is_empty() {
        return Err(ManifestError { problems });
    }
    Ok(json!({
        "entity": manifest.entity,
        "modules": modules.iter().map(|(handle, m)| json!({"module": handle, "name": m.name, "version": m.version, "sha256": m.sha256})).collect::<Vec<_>>(),
        "instances": specs.iter().map(|s| &s.name).collect::<Vec<_>>(),
        "channels": channels.iter().map(|(name, c)| json!({
            "name": name, "kind": c.kind.as_str(), "schema": c.payload.schema, "depth": c.depth,
            "writers": c.writers, "readers": c.readers})).collect::<Vec<_>>(),
        "warnings": warnings,
    }))
}
