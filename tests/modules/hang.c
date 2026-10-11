/* A module whose step stops returning after `hang_after` steps: it sleeps `hang_ms`
 * milliseconds (forever when 0). It publishes a sample per step on `out`, so the host has
 * something to mark stale. */
#include "common.h"

typedef struct instance {
  const xgc2_host_api* host;
  void* ctx;
  uint64_t hang_after, hang_ms, steps;
} instance;

static const xgc2_port_desc PORTS[] = {
    {"in", XGC2_PORT_IN, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, 0},
    {"out", XGC2_PORT_OUT, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, 0},
};

static xgc2_status create(const xgc2_host_api* host, void* ctx, const xgc2_config* config, xgc2_instance** out) {
  instance* self = (instance*)calloc(1, sizeof *self);
  if (!self) return XGC2_ERR_INTERNAL;
  self->host = host;
  self->ctx = ctx;
  self->hang_after = (uint64_t)cfg_number(config, "hang_after", 3);
  self->hang_ms = (uint64_t)cfg_number(config, "hang_ms", 0);
  *out = (xgc2_instance*)self;
  return XGC2_OK;
}

static xgc2_status configure(xgc2_instance* handle, const xgc2_config* config) {
  (void)handle;
  (void)config;
  return XGC2_OK;
}

static xgc2_status start(xgc2_instance* handle) {
  (void)handle;
  return XGC2_OK;
}

static xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* step_ctx) {
  instance* self = (instance*)handle;
  (void)step_ctx;
  xgc2_sample_view view;
  self->host->read_latest(self->ctx, 0, &view);
  test_sample* sample = (test_sample*)self->host->write_begin(self->ctx, 1);
  if (sample) {
    memset(sample, 0, sizeof *sample);
    sample->counter = ++self->steps;
    self->host->write_commit(self->ctx, 1, self->host->now_ns(self->ctx));
  }
  if (self->steps >= self->hang_after) {
    if (self->hang_ms == 0) {
      for (;;) pause_us(100000);
    }
    pause_us((long)self->hang_ms * 1000);
  }
  return XGC2_OK;
}

static xgc2_status stop(xgc2_instance* handle) {
  (void)handle;
  return XGC2_OK;
}

static void destroy(xgc2_instance* handle) { free(handle); }

static const xgc2_module_desc DESC = {XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "test_hang", "1.0.0",
                                      PORTS, 2, create, configure, start, step, stop, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_entry(void) { return &DESC; }
