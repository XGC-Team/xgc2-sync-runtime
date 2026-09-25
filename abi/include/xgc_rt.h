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

typedef enum xgc_port_dir { XGC_PORT_IN = 0, XGC_PORT_OUT = 1 } xgc_port_dir;

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
#define XGC_RT_ABI_MINOR 1u

typedef struct xgc_host_api {
  uint32_t abi_version;
  uint32_t abi_minor;     /* 0: through request_recover; 1: + port_origins, node_id */
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

#ifdef __cplusplus
}
#endif

#endif /* XGC_RT_H */
