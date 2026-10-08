/*
 * xgc_rt.h — XGC2 Sync Runtime plugin ABI, version 1.
 *
 * The one contract between an aggregator process (xgc-rt-host) and every module
 * plugin, whatever its domain (perception, estimation, planning, control,
 * DMPC neighbor exchange, simulation adapters) or language. A plugin is a
 * shared library that exports `xgc_rt_plugin_v1`.
 *
 * Rules:
 *  - The host owns threads, the clock, transports and audit. A plugin never
 *    opens sockets or spawns threads for IPC; it publishes and reads through
 *    `xgc_host_api`.
 *  - Every vtable call happens on the aggregator's thread for that plugin
 *    (one thread per plugin), never concurrently. `step` is called only on a
 *    round boundary or when an in-port is dirty, as the manifest trigger
 *    says. `next` reads a snapshot of the inputs taken when the step began.
 *  - "Ports" are module inputs and outputs in memory, not network ports.
 *    Between plugins of one aggregator, `publish` hands the sample over in
 *    memory; it is sent over the link (Zenoh) only to other processes.
 *  - ROS edge rule: a domain plugin (estimation, control, planning, DMPC)
 *    never calls ROS: no ros::init/rospy, no publish/subscribe, no ROS
 *    libraries. Only the aggregator's `ros_io` plugin talks ROS, with ordinary
 *    subscribe and publish: inbound topics become input samples, output
 *    samples become outbound topics. It is not the ros1_bridge package.
 *    VRPN, simulators and third-party ROS stacks stay ROS nodes.
 *  - Times are Session nanoseconds (see docs/time-model.md).
 *  - Strings are UTF-8, NUL-terminated, and owned by whoever returned them;
 *    descriptor strings must stay valid for the lifetime of the library.
 *  - No function may unwind across this boundary.
 */
#ifndef XGC_RT_H
#define XGC_RT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XGC_RT_ABI_VERSION 1u
#define XGC_RT_MAX_PORTS 64u

typedef enum xgc_status {
  XGC_OK = 0,
  XGC_ERR = 1,          /* plugin fault: the host moves it to Error */
  XGC_ERR_INVALID = 2,  /* bad argument (unknown port, wrong direction) */
  XGC_ERR_AGAIN = 3     /* nothing available now (in-port empty) */
} xgc_status;

/* An OPTIONAL port (minor 2) may be left unbound in the manifest: an unbound
 * optional in-port never has samples (`next` returns XGC_ERR_AGAIN) and a
 * `publish` on an unbound optional out-port is dropped and returns XGC_OK.
 * Every other port must be bound. */
typedef enum xgc_port_dir {
  XGC_PORT_IN = 0,
  XGC_PORT_OUT = 1,
  XGC_PORT_IN_OPTIONAL = 2,
  XGC_PORT_OUT_OPTIONAL = 3
} xgc_port_dir;

/* QoS class; the host maps it onto the transport (docs/qos.md). */
typedef enum xgc_qos {
  XGC_QOS_CONTROL = 0,  /* best-effort, drop on congestion, real-time priority */
  XGC_QOS_STATE = 1,    /* best-effort, drop, high priority */
  XGC_QOS_EVENT = 2,    /* reliable, bounded block */
  XGC_QOS_BULK = 3      /* reliable, block, low priority */
} xgc_qos;

typedef enum xgc_log_level {
  XGC_LOG_DEBUG = 0, XGC_LOG_INFO = 1, XGC_LOG_WARN = 2, XGC_LOG_ERROR = 3
} xgc_log_level;

typedef struct xgc_port_decl {
  const char* name;       /* unique within the plugin */
  xgc_port_dir dir;
  const char* schema_id;  /* payload schema; opaque to the runtime */
  xgc_qos qos;
} xgc_port_decl;

/* One received sample. `data` is valid until the next `next` call on any
 * port or the end of the current `step`, whichever comes first. */
typedef struct xgc_sample_view {
  uint16_t origin;        /* roster index of the sending node */
  uint16_t reserved;
  uint32_t len;
  uint64_t seq;           /* per (origin, channel), from 1 */
  uint64_t round;         /* round the sender produced it for */
  int64_t t_produce;
  int64_t t_tx;
  int64_t t_rx;
  const uint8_t* data;
} xgc_sample_view;

typedef struct xgc_step_ctx {
  uint64_t round;         /* current round k */
  int64_t now;            /* Session time at step start */
  int64_t round_start;    /* E0 + k*P */
  int64_t deadline;       /* publish deadline of round k */
  uint64_t dirty_ports;   /* bit i set: in-port i has unread samples */
  uint32_t round_advanced;/* 1 when this step is the first in round k */
  uint32_t reserved;
} xgc_step_ctx;

/* Minor revisions only append functions to xgc_host_api. A plugin checks
 * `abi_minor` before calling a function added in that minor. */
#define XGC_RT_ABI_MINOR 3u

typedef struct xgc_host_api {
  uint32_t abi_version;
  uint32_t abi_minor;     /* 0: through request_recover; 1: + port_origins, node_id;
                             2: unchanged table; 3: + rpc_runtime */
  void* host;
  /* Publish on an out-port for `round`. The host stamps, audits and sends. */
  xgc_status (*publish)(void* host, uint32_t port, uint64_t round,
                        const uint8_t* data, uint32_t len);
  /* Pop the next unread sample of an in-port; XGC_ERR_AGAIN when empty. */
  xgc_status (*next)(void* host, uint32_t port, xgc_sample_view* out);
  int64_t (*now)(void* host);
  void (*log)(void* host, xgc_log_level level, const char* message);
  /* Ask the host to move this plugin Active -> Degraded / back. */
  void (*request_degrade)(void* host, const char* reason);
  void (*request_recover)(void* host);
  /* abi_minor >= 1 */
  /* Roster ids an in-port receives from (the manifest `from`). Writes up to
   * `cap` ids and returns the total count, or 0 for an out-port. */
  uint32_t (*port_origins)(void* host, uint32_t port, uint16_t* out, uint32_t cap);
  /* This node's roster id. */
  uint16_t (*node_id)(void* host);
  /* abi_minor >= 3; nullable. Only call on this module's vtable thread from
   * create or activate. Returns a borrowed official xgc2_xrpc_runtime_api_v1
   * table (see xgc2/xrpc.h), valid only during that vtable call. Copy and retain
   * it there before keeping it; Rust modules use ForeignRuntime::from_api.
   * The retained context pins this module's actual dynamic library and uses
   * the process owner's existing runtime. Never cast it to a Rust Runtime or
   * invoke thread-affine host callbacks from an RPC handler. Close/release
   * all module endpoints before destroy; retaining does not authorize unload
   * while callbacks or their release are still executing. */
  const void* (*rpc_runtime)(void* host);
} xgc_host_api;

typedef struct xgc_plugin_vtbl {
  /* Allocate the instance. `host` stays valid until `destroy`. */
  void* (*create)(const xgc_host_api* host);
  /* `config` is the plugin's manifest table as TOML text (may be empty). */
  xgc_status (*configure)(void* self, const char* config);
  xgc_status (*activate)(void* self);
  xgc_status (*step)(void* self, const xgc_step_ctx* ctx);
  xgc_status (*deactivate)(void* self);
  void (*destroy)(void* self);
  /* Current domain FSM state name, for health; may return a static string. */
  const char* (*domain_state)(void* self);
} xgc_plugin_vtbl;

typedef struct xgc_plugin_descriptor {
  uint32_t abi_version;   /* must equal XGC_RT_ABI_VERSION */
  uint32_t port_count;    /* <= XGC_RT_MAX_PORTS */
  const char* name;
  const char* version;
  const xgc_port_decl* ports;
  const xgc_plugin_vtbl* vtbl;
} xgc_plugin_descriptor;

/* The single exported symbol. */
typedef const xgc_plugin_descriptor* (*xgc_rt_plugin_v1_fn)(void);
const xgc_plugin_descriptor* xgc_rt_plugin_v1(void);

/* ---------------------------------------------------------------------------
 * Transport plugin ABI, version 1: how a host moves envelope frames between
 * nodes (processes). Loopback and Zenoh are transport plugins; the manifest
 * names one with [transport] path = "...", exactly as [[plugin]] path.
 *
 * A transport moves opaque frames. It never stamps, audits or decodes them:
 * the host does that beside every send and receive, so every transport is
 * audited identically. The host makes every call except the sink from one
 * thread at a time; the transport calls the sink from its own IO threads,
 * and the sink copies the frame and returns without blocking. The sink is
 * never called after `close` returns.
 * ------------------------------------------------------------------------- */

#define XGC_RT_TRANSPORT_ABI_VERSION 1u

typedef struct xgc_transport_channel {
  uint32_t id;            /* the channel id frames carry */
  xgc_qos qos;
  const char* name;
} xgc_transport_channel;

/* Valid for the duration of `open`; the transport copies what it keeps. */
typedef struct xgc_transport_context {
  const char* session;
  const char* node;       /* this node's roster name */
  uint16_t node_id;       /* its roster index */
  uint16_t reserved;
  uint32_t roster_count;
  const char* const* roster;
  uint32_t channel_count;
  const xgc_transport_channel* channels;
  const char* options;    /* the manifest's [transport] table as TOML text,
                             without kind, path and sha256 */
} xgc_transport_context;

/* One received frame; `frame` is valid only during the call. */
typedef void (*xgc_transport_sink)(void* sink_ctx, const uint8_t* frame, uint32_t len);

typedef struct xgc_transport_vtbl {
  void* (*create)(void);
  xgc_status (*open)(void* self, const xgc_transport_context* ctx, xgc_transport_sink sink, void* sink_ctx);
  /* This node publishes `channel`. */
  xgc_status (*declare_out)(void* self, uint32_t channel);
  /* Deliver `channel` from exactly these origins. */
  xgc_status (*declare_in)(void* self, uint32_t channel, const uint16_t* origins, uint32_t count);
  xgc_status (*send)(void* self, uint32_t channel, const uint8_t* frame, uint32_t len);
  /* Block up to `timeout_ns` until every declared out-channel has a matching
   * remote subscriber: 1 ready, 0 timed out. */
  int32_t (*wait_ready)(void* self, uint64_t timeout_ns);
  void (*close)(void* self);
  void (*destroy)(void* self);
  /* The reason for the last XGC_ERR* return, or "" (owned by the transport,
   * valid until its next call). */
  const char* (*last_error)(void* self);
} xgc_transport_vtbl;

typedef struct xgc_transport_descriptor {
  uint32_t abi_version;   /* must equal XGC_RT_TRANSPORT_ABI_VERSION */
  uint32_t reserved;
  const char* kind;       /* e.g. "loopback", "zenoh" */
  const xgc_transport_vtbl* vtbl;
} xgc_transport_descriptor;

/* The single exported symbol of a transport plugin. */
typedef const xgc_transport_descriptor* (*xgc_rt_transport_v1_fn)(void);
const xgc_transport_descriptor* xgc_rt_transport_v1(void);

#ifdef __cplusplus
}
#endif

#endif /* XGC_RT_H */
