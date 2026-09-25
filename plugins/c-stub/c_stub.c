/*
 * C stub plugin: a pure-C module on xgc_rt.h. It proves the ABI is
 * language-neutral with no Rust in the plugin.
 *
 * Consumes `cmd` (event) and counts commands; domain FSM `waiting` ->
 * `counting`. Config `fail_after = N` returns XGC_ERR from the Nth step, to
 * exercise the host's restart policy.
 *
 * Build: cc -std=c11 -Wall -Wextra -Werror -fPIC -shared \
 *          -I ../../abi/include c_stub.c -o libc_stub.so
 */
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "xgc_rt.h"

/* The C side of the layout check in crates/xgc-rt-abi/tests/layout.rs. */
_Static_assert(sizeof(xgc_sample_view) == 56, "xgc_sample_view layout");
_Static_assert(offsetof(xgc_sample_view, data) == 48, "xgc_sample_view.data");
_Static_assert(sizeof(xgc_step_ctx) == 48, "xgc_step_ctx layout");
_Static_assert(offsetof(xgc_step_ctx, dirty_ports) == 32, "xgc_step_ctx.dirty_ports");
_Static_assert(sizeof(xgc_port_decl) == 32, "xgc_port_decl layout");
_Static_assert(sizeof(xgc_plugin_descriptor) == 40, "xgc_plugin_descriptor layout");

enum { PORT_CMD = 0 };

typedef struct c_stub {
  const xgc_host_api* host;
  unsigned long long commands;
  unsigned long long steps;
  long fail_after; /* 0 = never */
} c_stub;

static void* create(const xgc_host_api* host) {
  c_stub* self = calloc(1, sizeof *self);
  if (self) self->host = host;
  return self;
}

static xgc_status configure(void* p, const char* config) {
  c_stub* self = p;
  const char* key = config ? strstr(config, "fail_after") : NULL;
  if (key) {
    const char* eq = strchr(key, '=');
    if (!eq) return XGC_ERR_INVALID;
    self->fail_after = strtol(eq + 1, NULL, 10);
    if (self->fail_after < 0) return XGC_ERR_INVALID;
  }
  return XGC_OK;
}

static xgc_status activate(void* p) { (void)p; return XGC_OK; }

static xgc_status step(void* p, const xgc_step_ctx* ctx) {
  c_stub* self = p;
  (void)ctx;
  self->steps++;
  if (self->fail_after > 0 && self->steps == (unsigned long long)self->fail_after) {
    self->host->log(self->host->host, XGC_LOG_WARN, "c-stub: configured failure");
    return XGC_ERR;
  }
  xgc_sample_view view;
  while (self->host->next(self->host->host, PORT_CMD, &view) == XGC_OK) {
    if (view.len < 16) return XGC_ERR;
    self->commands++;
  }
  return XGC_OK;
}

static xgc_status deactivate(void* p) { (void)p; return XGC_OK; }

static void destroy(void* p) { free(p); }

static const char* domain_state(void* p) {
  const c_stub* self = p;
  return self->commands ? "counting" : "waiting";
}

static const xgc_port_decl PORTS[] = {
  { "cmd", XGC_PORT_IN, "xgc.stub.cmd/1", XGC_QOS_EVENT },
};

static const xgc_plugin_vtbl VTBL = {
  create, configure, activate, step, deactivate, destroy, domain_state,
};

static const xgc_plugin_descriptor DESCRIPTOR = {
  XGC_RT_ABI_VERSION, 1u, "c-stub", "0.1.0", PORTS, &VTBL,
};

const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) { return &DESCRIPTOR; }
