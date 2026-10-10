/* State producer: `burst` commits per step. Stamps every sample with the host clock and the
 * address of the slot it wrote, so a consumer can prove the data was not copied. */
#include "common.h"

typedef struct instance {
  const xgc2_host_api* host;
  void* ctx;
  uint64_t producer;
  uint64_t burst;
  uint64_t counter;
  uint64_t steps;
  uint64_t fail_at;
  uint64_t report_every;
} instance;

static const xgc2_port_desc PORTS[] = {
    {"out", XGC2_PORT_OUT, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, 0},
};

static void apply(instance* self, const xgc2_config* config) {
  self->producer = (uint64_t)cfg_number(config, "id", 1);
  self->burst = (uint64_t)cfg_number(config, "burst", 1);
  self->fail_at = (uint64_t)cfg_number(config, "fail_at", 0);
  self->report_every = (uint64_t)cfg_number(config, "report_every", 1);
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
  for (uint64_t i = 0; i < self->burst; i++) {
    test_sample* sample = (test_sample*)self->host->write_begin(self->ctx, 0);
    if (!sample) return XGC2_ERR_FULL;
    memset(sample, 0, sizeof *sample);
    sample->counter = ++self->counter;
    sample->addr = (uint64_t)(uintptr_t)sample;
    sample->producer = self->producer;
    sample->committed_ns = self->host->now_ns(self->ctx);
    self->host->write_commit(self->ctx, 0, sample->committed_ns);
  }
  self->steps++;
  if (self->report_every && self->steps % self->report_every == 0) {
    char detail[96];
    snprintf(detail, sizeof detail, "{\"steps\":%llu,\"commits\":%llu}", (unsigned long long)self->steps,
             (unsigned long long)self->counter);
    self->host->report(self->ctx, XGC2_OK, detail);
  }
  return self->fail_at && self->steps == self->fail_at ? XGC2_ERR_INTERNAL : XGC2_OK;
}

static xgc2_status stop(xgc2_instance* handle) {
  (void)handle;
  return XGC2_OK;
}

static void destroy(xgc2_instance* handle) { free(handle); }

static const xgc2_module_desc DESC = {XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "test_state_producer", "1.0.0",
                                      PORTS, 1, create, configure, start, step, stop, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_v2(void) { return &DESC; }
