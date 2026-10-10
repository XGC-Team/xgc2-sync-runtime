/* A module whose step takes `sleep_us` microseconds. The time can be changed live with
 * configure. Reads its optional state input, so a producer can outrun it. */
#include "common.h"

typedef struct instance {
  const xgc2_host_api* host;
  void* ctx;
  long sleep_us;
  uint64_t steps, reads, fail_at, report_at;
  int report_health;
} instance;

static const xgc2_port_desc PORTS[] = {
    {"in", XGC2_PORT_IN, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, 0},
};

static void apply(instance* self, const xgc2_config* config) {
  self->sleep_us = (long)cfg_number(config, "sleep_us", 0);
  self->fail_at = (uint64_t)cfg_number(config, "fail_at", 0);
  self->report_at = (uint64_t)cfg_number(config, "report_at", 0);
  self->report_health = (int)cfg_number(config, "report_health", XGC2_OK);
}

static xgc2_status create(const xgc2_host_api* host, void* ctx, const xgc2_config* config, xgc2_instance** out) {
  instance* self = (instance*)calloc(1, sizeof *self);
  if (!self) return XGC2_ERR_INTERNAL;
  self->host = host;
  self->ctx = ctx;
  apply(self, config);
  *out = (xgc2_instance*)self;
  return XGC2_OK;
}

static xgc2_status configure(xgc2_instance* handle, const xgc2_config* config) {
  if (cfg_number(config, "reject", 0) != 0) return XGC2_ERR_INVALID;
  apply((instance*)handle, config);
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
  if (self->host->read_latest(self->ctx, 0, &view) == XGC2_OK) self->reads++;
  self->steps++;
  if (self->sleep_us > 0) pause_us(self->sleep_us);
  char detail[96];
  snprintf(detail, sizeof detail, "{\"steps\":%llu,\"reads\":%llu,\"sleep_us\":%ld}", (unsigned long long)self->steps,
           (unsigned long long)self->reads, self->sleep_us);
  self->host->report(self->ctx, self->report_at && self->steps >= self->report_at ? self->report_health : XGC2_OK, detail);
  if (self->fail_at && self->steps == self->fail_at) return XGC2_ERR_INTERNAL;
  return XGC2_OK;
}

static xgc2_status stop(xgc2_instance* handle) {
  (void)handle;
  return XGC2_OK;
}

static void destroy(xgc2_instance* handle) { free(handle); }

static const xgc2_module_desc DESC = {XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "test_slow", "1.0.0",
                                      PORTS, 1, create, configure, start, step, stop, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_v2(void) { return &DESC; }
