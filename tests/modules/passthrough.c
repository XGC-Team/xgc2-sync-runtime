/* One stage of a chain: reads the newest sample of `in` and republishes it on `out` with its
 * own commit stamp; the payload keeps the origin's `committed_ns`, so a sink can measure both
 * the last hop and the whole chain. With `latency_file` the stage records, for every sample,
 * the nanoseconds between the commit of the sample it read and the start of its own step. */
#include "common.h"

#define MAX_LATENCIES 400000

typedef struct instance {
  const xgc2_host_api* host;
  void* ctx;
  uint64_t steps, forwarded;
  int version;
  int fail_start;
  uint64_t report_every;
  char latency_file[256];
  int64_t* latencies;
  uint64_t latency_count;
} instance;

#ifndef VERSION
#define VERSION 1
#endif

static const xgc2_port_desc PORTS[] = {
    {"in", XGC2_PORT_IN, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, XGC2_PORT_REQUIRED},
    {"out", XGC2_PORT_OUT, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, 0},
};

static xgc2_status create(const xgc2_host_api* host, void* ctx, const xgc2_config* config, xgc2_instance** out) {
  if (cfg_number(config, "fail_create", 0) != 0) return XGC2_ERR_INVALID;
  instance* self = (instance*)calloc(1, sizeof *self);
  if (!self) return XGC2_ERR_INTERNAL;
  self->host = host;
  self->ctx = ctx;
  self->version = VERSION;
  self->fail_start = cfg_number(config, "fail_start", 0) != 0;
  self->report_every = (uint64_t)cfg_number(config, "report_every", 1);
  cfg_string(config, "latency_file", self->latency_file, sizeof self->latency_file);
  if (self->latency_file[0]) self->latencies = (int64_t*)malloc(MAX_LATENCIES * sizeof(int64_t));
  *out = (xgc2_instance*)self;
  return XGC2_OK;
}

static xgc2_status configure(xgc2_instance* handle, const xgc2_config* config) {
  (void)handle;
  (void)config;
  return XGC2_OK;
}

static xgc2_status start(xgc2_instance* handle) {
  return ((instance*)handle)->fail_start ? XGC2_ERR_INTERNAL : XGC2_OK;
}

static xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* step_ctx) {
  instance* self = (instance*)handle;
  self->steps++;
  xgc2_sample_view view;
  if (self->host->read_latest(self->ctx, 0, &view) == XGC2_OK) {
    if (self->latencies && self->latency_count < MAX_LATENCIES)
      self->latencies[self->latency_count++] = step_ctx->now_ns - view.stamp_ns;
    test_sample* out = (test_sample*)self->host->write_begin(self->ctx, 1);
    if (out) {
      memcpy(out, view.data, sizeof *out);
      out->addr = (uint64_t)(uintptr_t)out;
      self->host->write_commit(self->ctx, 1, self->host->now_ns(self->ctx));
      self->forwarded++;
    }
  }
  if (self->report_every && self->steps % self->report_every == 0) {
    char detail[96];
    snprintf(detail, sizeof detail, "{\"version\":%d,\"steps\":%llu,\"forwarded\":%llu}", self->version,
             (unsigned long long)self->steps, (unsigned long long)self->forwarded);
    self->host->report(self->ctx, XGC2_OK, detail);
  }
  return XGC2_OK;
}

static xgc2_status stop(xgc2_instance* handle) {
  instance* self = (instance*)handle;
  if (self->latencies && self->latency_file[0]) {
    FILE* file = fopen(self->latency_file, "wb");
    if (file) {
      fwrite(self->latencies, sizeof(int64_t), self->latency_count, file);
      fclose(file);
    }
    self->latency_count = 0;
  }
  return XGC2_OK;
}

static void destroy(xgc2_instance* handle) {
  free(((instance*)handle)->latencies);
  free(handle);
}

static const xgc2_module_desc DESC = {XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "test_passthrough", "1.0.0",
                                      PORTS, 2, create, configure, start, step, stop, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_v2(void) { return &DESC; }
