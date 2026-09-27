//! The Zenoh transport as a transport plugin (`xgc_rt_transport_v1`). The
//! options are the built-in transport's: `listen` and `connect` endpoint
//! lists (xgc-rt-transport-zenoh).

use xgc_rt_core::transport::{Transport, TransportContext, TransportError};
use xgc_rt_transport_zenoh::{ZenohOptions, ZenohTransport};

fn factory(_ctx: &TransportContext, options: &toml::Table) -> Result<Box<dyn Transport>, TransportError> {
    Ok(Box::new(ZenohTransport::new(ZenohOptions::from_table(options)?)))
}

xgc_rt_core::export_transport!("zenoh", factory);
