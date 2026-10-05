/*
 * C stub plugin: a pure-C module on xgc_rt.h. It proves the ABI is
 * language-neutral with no Rust in the plugin.
 *
 * Consumes `cmd` (event) and counts commands; domain FSM `waiting` ->
 * `counting`. Config `fail_after = N` returns XGC_ERR from the Nth step, to
 * exercise the restart policy. `step_sleep_ms = N` makes every step take N ms
 * and `hang_after = N` blocks the Nth step for 60 s, to exercise the
 * watchdog.
 *
 * Build: cc -std=c11 -Wall -Wextra -Werror -fPIC -shared \
 *          -I ../../abi/include c_stub.c -o libc_stub.so
 */
#define _POSIX_C_SOURCE 199309L
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "xgc_rt.h"

/* The C side of the layout check in crates/xgc-rt-abi/tests/layout.rs. */
_Static_assert(sizeof(xgc_sample_view) == 56, "xgc_sample_view layout");
_Static_assert(offsetof(xgc_sample_view, data) == 48, "xgc_sample_view.data");
_Static_assert(sizeof(xgc_step_ctx) == 48, "xgc_step_ctx layout");
_Static_assert(offsetof(xgc_step_ctx, dirty_ports) == 32, "xgc_step_ctx.dirty_ports");
_Static_assert(sizeof(xgc_port_decl) == 32, "xgc_port_decl layout");
_Static_assert(sizeof(xgc_plugin_descriptor) == 40, "xgc_plugin_descriptor layout");
_Static_assert(sizeof(xgc_host_api) == 88, "minor-3 host API layout");
_Static_assert(offsetof(xgc_host_api, acquire_clock_reader) == 80, "minor-3 append offset");
_Static_assert(sizeof(xgc_clock_reader_v1) == 24, "owned clock reader layout");

enum { PORT_CMD = 0 };

typedef struct c_stub {
  const xgc_host_api* host;
  unsigned long long commands;
  unsigned long long steps;
  long fail_after; /* 0 = never */
  long step_sleep_ms;
  long hang_after; /* 0 = never */
} c_stub;

static void sleep_ms(long ms) {
  struct timespec t = { ms / 1000, (ms % 1000) * 1000000L };
  while (nanosleep(&t, &t) != 0) {}
}

/* Read `key = N` from the TOML config; 0 when absent, -1 when malformed. */
static long config_int(const char* config, const char* key) {
  const char* at = config ? strstr(config, key) : NULL;
  if (!at) return 0;
  const char* eq = strchr(at, '=');
  if (!eq) return -1;
  long value = strtol(eq + 1, NULL, 10);
  return value < 0 ? -1 : value;
}

static void* create(const xgc_host_api* host) {
  c_stub* self = calloc(1, sizeof *self);
  if (self) self->host = host;
  return self;
}

static xgc_status configure(void* p, const char* config) {
  c_stub* self = p;
  self->fail_after = config_int(config, "fail_after");
  self->step_sleep_ms = config_int(config, "step_sleep_ms");
  self->hang_after = config_int(config, "hang_after");
  if (self->fail_after < 0 || self->step_sleep_ms < 0 || self->hang_after < 0) return XGC_ERR_INVALID;
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
  if (self->hang_after > 0 && self->steps == (unsigned long long)self->hang_after) sleep_ms(60000);
  if (self->step_sleep_ms > 0) sleep_ms(self->step_sleep_ms);
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
