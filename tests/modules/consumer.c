/* Consumer of one state input and one event input. It checks what the host promises:
 *  - the state sequence never goes backwards and a sample is delivered at most once per change;
 *  - the sample is the very memory the producer wrote (zero-copy);
 *  - events of each producer arrive in order;
 *  - the same step reads the same state sample twice.
 * `work_us` simulates a slow consumer. `latency_file` receives the nanoseconds between each
 * sample's commit stamp and the start of the step that read it; `origin_file` the nanoseconds
 * since the payload's `committed_ns`, which a pass-through stage preserves (state input only). */
#include "common.h"

#define MAX_PRODUCERS 16
#define MAX_LATENCIES 400000

typedef struct instance {
  const xgc2_host_api* host;
  void* ctx;
  long work_us;
  uint64_t report_every;
  char latency_file[256], origin_file[256];
  uint64_t steps, state_updates, bad_seq, zero_copy_bad, repeat_bad, wrong_size;
  uint64_t events, event_disorder, event_gaps, no_input_steps;
  uint64_t last_state_seq, last_producer;
  uint64_t last_event[MAX_PRODUCERS];
  uint64_t config_steps, wake_steps, timer_steps;
  int64_t *latencies, *origins;
  uint64_t latency_count;
} instance;

enum { IN_STATE = 0, IN_EVENT = 1 };

static const xgc2_port_desc PORTS[] = {
    {"state_in", XGC2_PORT_IN, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, 0},
    {"event_in", XGC2_PORT_IN, XGC2_PORT_EVENT, EVENT_SCHEMA, sizeof(test_event), 8, 16, 0},
};

static void apply(instance* self, const xgc2_config* config) {
  self->work_us = (long)cfg_number(config, "work_us", 0);
  self->report_every = (uint64_t)cfg_number(config, "report_every", 1);
}

static xgc2_status create(const xgc2_host_api* host, void* ctx, const xgc2_config* config, xgc2_instance** out) {
  instance* self = (instance*)calloc(1, sizeof *self);
  if (!self) return XGC2_ERR_INTERNAL;
  self->host = host;
  self->ctx = ctx;
  apply(self, config);
  cfg_string(config, "latency_file", self->latency_file, sizeof self->latency_file);
  cfg_string(config, "origin_file", self->origin_file, sizeof self->origin_file);
  if (self->latency_file[0]) self->latencies = (int64_t*)malloc(MAX_LATENCIES * sizeof(int64_t));
  if (self->origin_file[0]) self->origins = (int64_t*)malloc(MAX_LATENCIES * sizeof(int64_t));
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

static void publish(instance* self) {
  char detail[700];
  snprintf(detail, sizeof detail,
           "{\"steps\":%llu,\"state_updates\":%llu,\"bad_seq\":%llu,\"zero_copy_bad\":%llu,\"repeat_bad\":%llu,"
           "\"wrong_size\":%llu,\"events\":%llu,\"event_disorder\":%llu,\"event_gaps\":%llu,\"last_state_seq\":%llu,"
           "\"config_steps\":%llu,\"wake_steps\":%llu,\"timer_steps\":%llu,\"work_us\":%ld,\"last_producer\":%llu}",
           (unsigned long long)self->steps, (unsigned long long)self->state_updates, (unsigned long long)self->bad_seq,
           (unsigned long long)self->zero_copy_bad, (unsigned long long)self->repeat_bad,
           (unsigned long long)self->wrong_size, (unsigned long long)self->events,
           (unsigned long long)self->event_disorder, (unsigned long long)self->event_gaps,
           (unsigned long long)self->last_state_seq, (unsigned long long)self->config_steps,
           (unsigned long long)self->wake_steps, (unsigned long long)self->timer_steps, self->work_us,
           (unsigned long long)self->last_producer);
  self->host->report(self->ctx, XGC2_OK, detail);
}

static xgc2_status step(xgc2_instance* handle, const xgc2_step_ctx* step_ctx) {
  instance* self = (instance*)handle;
  self->steps++;
  if (step_ctx->reasons & XGC2_STEP_CONFIG) self->config_steps++;
  if (step_ctx->reasons & XGC2_STEP_WAKE) self->wake_steps++;
  if (step_ctx->reasons & XGC2_STEP_TIMER) self->timer_steps++;
  /* changed_inputs numbers the inputs: state_in is bit 0, event_in is bit 1. */
  int state_changed = (step_ctx->changed_inputs & 1) != 0;
  int event_changed = (step_ctx->changed_inputs & 2) != 0;
  if (state_changed != self->host->changed(self->ctx, IN_STATE)) self->bad_seq++;
  if (event_changed != self->host->changed(self->ctx, IN_EVENT)) self->bad_seq++;

  xgc2_sample_view view;
  if (self->host->read_latest(self->ctx, IN_STATE, &view) == XGC2_OK) {
    const test_sample* sample = (const test_sample*)view.data;
    if (view.size != sizeof(test_sample)) self->wrong_size++;
    if (view.seq < self->last_state_seq) self->bad_seq++;
    if (view.seq > self->last_state_seq) {
      self->state_updates++;
      if ((uint64_t)(uintptr_t)view.data != sample->addr) self->zero_copy_bad++;
      if (self->latency_count < MAX_LATENCIES) {
        if (self->latencies) self->latencies[self->latency_count] = step_ctx->now_ns - view.stamp_ns;
        if (self->origins) self->origins[self->latency_count] = step_ctx->now_ns - sample->committed_ns;
        if (self->latencies || self->origins) self->latency_count++;
      }
    }
    self->last_state_seq = view.seq;
    self->last_producer = sample->producer;
    /* A second read in the same step returns the same sample. */
    xgc2_sample_view again;
    if (self->host->read_latest(self->ctx, IN_STATE, &again) != XGC2_OK || again.data != view.data ||
        again.seq != view.seq)
      self->repeat_bad++;
  }

  while (self->host->read_next(self->ctx, IN_EVENT, &view) == XGC2_OK) {
    const test_event* event = (const test_event*)view.data;
    self->events++;
    if (event->producer < MAX_PRODUCERS) {
      uint64_t last = self->last_event[event->producer];
      if (event->seq <= last) self->event_disorder++;
      else if (event->seq != last + 1) self->event_gaps++;
      self->last_event[event->producer] = event->seq;
    }
  }
  if (self->report_every && self->steps % self->report_every == 0) publish(self);
  if (self->work_us > 0) pause_us(self->work_us);
  return XGC2_OK;
}

static void dump(const char* path, const int64_t* values, uint64_t count) {
  FILE* file = path[0] && values ? fopen(path, "wb") : NULL;
  if (!file) return;
  fwrite(values, sizeof(int64_t), count, file);
  fclose(file);
}

static xgc2_status stop(xgc2_instance* handle) {
  instance* self = (instance*)handle;
  dump(self->latency_file, self->latencies, self->latency_count);
  dump(self->origin_file, self->origins, self->latency_count);
  self->latency_count = 0;
  return XGC2_OK;
}

static void destroy(xgc2_instance* handle) {
  instance* self = (instance*)handle;
  free(self->latencies);
  free(self->origins);
  free(self);
}

static const xgc2_module_desc DESC = {XGC2_MODULE_ABI_MAJOR, XGC2_MODULE_ABI_MINOR, "test_consumer", "1.0.0",
                                      PORTS, 2, create, configure, start, step, stop, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_v2(void) { return &DESC; }
