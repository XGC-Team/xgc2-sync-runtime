/* Simulated time source for the external clock mode: a thread publishes xgc2.clock.v1
 * samples `step_ms` of simulated time apart, one every `interval_us` of wall time, `count`
 * times, then stops. It is what the ROS edge does with /clock. */
#include <pthread.h>

#include "common.h"

typedef struct instance {
  const xgc2_host_api* host;
  void* ctx;
  long step_ms, interval_us, count;
  volatile int running;
  pthread_t thread;
  int started;
  volatile long published;
} instance;

static const xgc2_port_desc PORTS[] = {
    {"time", XGC2_PORT_OUT, XGC2_PORT_STATE, CLOCK_SCHEMA, sizeof(test_clock), 8, 0, XGC2_PORT_ASYNC_WRITER},
};

static void* run(void* argument) {
  instance* self = (instance*)argument;
  for (long k = 1; k <= self->count && self->running; k++) {
    pause_us(self->interval_us);
    test_clock* clock = (test_clock*)self->host->write_begin(self->ctx, 0);
    if (!clock) continue;
    clock->time_ns = k * self->step_ms * 1000000L;
    self->host->write_commit(self->ctx, 0, clock->time_ns);
    self->published = k;
  }
  return NULL;
}

static xgc2_status create(const xgc2_host_api* host, void* ctx, const xgc2_config* config, xgc2_instance** out) {
  instance* self = (instance*)calloc(1, sizeof *self);
  if (!self) return XGC2_ERR_INTERNAL;
  self->host = host;
  self->ctx = ctx;
  self->step_ms = (long)cfg_number(config, "step_ms", 10);
  self->interval_us = (long)cfg_number(config, "interval_us", 1000);
  self->count = (long)cfg_number(config, "count", 100);
  *out = (xgc2_instance*)self;
  return XGC2_OK;
}

static xgc2_status configure(xgc2_instance* handle, const xgc2_config* config) {
  (void)handle;
  (void)config;
  return XGC2_OK;
}

static xgc2_status start(xgc2_instance* handle) {
  instance* self = (instance*)handle;
  self->running = 1;
  if (pthread_create(&self->thread, NULL, run, self) != 0) return XGC2_ERR_INTERNAL;
  self->started = 1;
  return XGC2_OK;
}

static xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* step_ctx) {
  instance* self = (instance*)handle;
  (void)step_ctx;
  char detail[64];
  snprintf(detail, sizeof detail, "{\"published\":%ld}", self->published);
  self->host->report(self->ctx, XGC2_OK, detail);
  return XGC2_OK;
}

static xgc2_status stop(xgc2_instance* handle) {
  instance* self = (instance*)handle;
  self->running = 0;
  if (self->started) pthread_join(self->thread, NULL);
  self->started = 0;
  return XGC2_OK;
}

static void destroy(xgc2_instance* handle) { free(handle); }

static const xgc2_module_desc DESC = {XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "test_sim_clock", "1.0.0",
                                      PORTS, 1, create, configure, start, step, stop, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_entry(void) { return &DESC; }
