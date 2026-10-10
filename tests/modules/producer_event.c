/* Event producer: `burst` events per step with a per-producer sequence number. A full queue
 * (write_begin returns NULL) is counted, not retried. */
#include "common.h"

typedef struct instance {
  const xgc2_host_api* host;
  void* ctx;
  uint64_t producer;
  uint64_t burst;
  uint64_t seq;
  uint64_t sent;
  uint64_t dropped;
} instance;

static const xgc2_port_desc PORTS[] = {
    {"out", XGC2_PORT_OUT, XGC2_PORT_EVENT, EVENT_SCHEMA, sizeof(test_event), 8, 16, 0},
};

static void apply(instance* self, const xgc2_config* config) {
  self->producer = (uint64_t)cfg_number(config, "id", 1);
  self->burst = (uint64_t)cfg_number(config, "burst", 1);
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
    /* The sequence number is taken per attempt, so a refused write shows up as a gap. */
    uint64_t seq = ++self->seq;
    test_event* event = (test_event*)self->host->write_begin(self->ctx, 0);
    if (!event) {
      self->dropped++;
      continue;
    }
    event->producer = self->producer;
    event->seq = seq;
    self->host->write_commit(self->ctx, 0, self->host->now_ns(self->ctx));
    self->sent++;
  }
  char detail[128];
  snprintf(detail, sizeof detail, "{\"sent\":%llu,\"dropped\":%llu,\"last_seq\":%llu}", (unsigned long long)self->sent,
           (unsigned long long)self->dropped, (unsigned long long)self->seq);
  self->host->report(self->ctx, XGC2_OK, detail);
  return XGC2_OK;
}

static xgc2_status stop(xgc2_instance* handle) {
  (void)handle;
  return XGC2_OK;
}

static void destroy(xgc2_instance* handle) { free(handle); }

static const xgc2_module_desc DESC = {XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "test_event_producer", "1.0.0",
                                      PORTS, 1, create, configure, start, step, stop, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_v2(void) { return &DESC; }
