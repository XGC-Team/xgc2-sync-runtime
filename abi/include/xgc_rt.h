/*
 * xgc_rt.h — XGC2 Sync Runtime plugin ABI, version 1.
 *
 * The one contract between an aggregator process (xgc-rt-host) and every module
 * plugin, whatever its domain (perception, estimation, planning, control,
 * DMPC neighbor exchange, simulation adapters) or language. A plugin is a
 * shared library that exports `xgc_rt_plugin_v1`.
 *
 * Rules:
 *  - The host owns scheduling, the Session clock, transports and audit.
 *    Ordinary domain plugins use `xgc_host_api` for sample exchange. Declared
 *    ROS I/O owners (including a simulation World owning its ROS boundary)
 *    may own ingress/service/egress threads; this does not permit arbitrary
 *    domain threads to call ordinary Host API functions.
 *  - Every vtable call happens on the aggregator's thread for that plugin
 *    (one thread per plugin), never concurrently. `step` is called only on a
 *    round boundary or when an in-port is dirty, as the manifest trigger
 *    says. `next` reads a snapshot of the inputs taken when the step began.
 *  - "Ports" are module inputs and outputs in memory, not network ports.
 *    Between plugins of one aggregator, `publish` hands the sample over in
 *    memory; it is sent over the link (Zenoh) only to other processes.
 *  - ROS edge rule: ordinary domain mathematics (estimation, control,
 *    planning, DMPC) never calls ROS. Only declared ROS I/O owning modules
 *    provide subscribe/publish/service boundaries; a World may own its
 *    simulation ROS boundary without forwarding it through another plugin.
 *    This is not ros1_bridge and does not move external controller/estimator
 *    mathematics into the World. Third-party ROS stacks stay ROS nodes.
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

/* Minor revisions only append functions to xgc_host_api. Before touching an
 * appended field, check abi_version/abi_minor through the existing 8-byte
 * prefix. An old table may physically end before that field. The descriptor
 * stays ABI v1: its major-only loader check is not a Host minor capability gate.
 * The Rust SDK create shim rejects an old prefix before user create code can
 * borrow the current full table, including plugins that do not need a reader.
 * Ordinary Host API functions are module-thread services inside vtable calls.
 * Only an acquired owned reader's now operation is an asynchronous pure read.
 */
#define XGC_RT_ABI_MINOR 3u
#define XGC_RT_CLOCK_READER_ABI_MINOR 3u

/* One owned reference to the SAME live Session Clock used by Host::now.
 * Acquire once during cold create; do not acquire/retain per body or read.
 * now is thread-safe and only reads that Clock: no Slot/ctx/input queue, clock
 * source poll, allocation, Arc clone or alternate time policy. It preserves
 * the Clock's existing locking and accepted-time/initial-zero semantics; it
 * is not promised lock-free. On error now leaves *out_time_ns unchanged.
 * The I/O owner fences readers and joins ALL reading threads before release.
 * release consumes opaque exactly once, then this whole handle is invalid.
 * A handle is unique ownership, not a memcpy-copyable second owner. All calls
 * and release must finish before plugin destroy / Host library unload.
 */
typedef struct xgc_clock_reader_v1 {
  void* opaque;
  xgc_status (*now)(void* opaque, int64_t* out_time_ns);
  void (*release)(void* opaque);
} xgc_clock_reader_v1;

typedef struct xgc_host_api {
  uint32_t abi_version;
  uint32_t abi_minor;     /* 0: base; 1: roster; 2: optional ports; 3: owned reader */
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
  /* abi_minor >= 3; cold module-thread call only. reader_size must equal
   * sizeof(xgc_clock_reader_v1), and out must be a fresh zeroed handle.
   * OK returns all three non-NULL fields; failure acquires no reference.
   * Of the reader operations, only this call touches the module Slot. Later
   * now/release calls use opaque independently of the API table / Slot lifetime.
   */
  xgc_status (*acquire_clock_reader)(void* host, uint32_t reader_size,
                                    xgc_clock_reader_v1* out);
} xgc_host_api;

/* Strict consumer gate. No Host::now / ROS time / cached-clock fallback.
 * Do not inspect acquire_clock_reader until the old prefix admits minor 3.
 * World cold create must reject an error instead of starting ingress with an
 * absent reader. The output must be zero-initialized and not already owned.
 */
static inline xgc_status xgc_acquire_clock_reader_v1(
    const xgc_host_api* api, xgc_clock_reader_v1* out) {
  xgc_status status;
  if (api == 0 || out == 0 || out->opaque != 0 || out->now != 0 || out->release != 0)
    return XGC_ERR_INVALID;
  if (api->abi_version != XGC_RT_ABI_VERSION ||
      api->abi_minor < XGC_RT_CLOCK_READER_ABI_MINOR)
    return XGC_ERR_INVALID;
  if (api->acquire_clock_reader == 0) return XGC_ERR_INVALID;
  status = api->acquire_clock_reader(api->host, (uint32_t)sizeof(*out), out);
  if (status != XGC_OK) return status;
  if (out->opaque == 0 || out->now == 0 || out->release == 0) {
    if (out->opaque != 0 && out->release != 0) out->release(out->opaque);
    out->opaque = 0; out->now = 0; out->release = 0;
    return XGC_ERR_INVALID;
  }
  return XGC_OK;
}

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
