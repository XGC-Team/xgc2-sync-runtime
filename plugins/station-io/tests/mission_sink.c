#include "xgc_rt.h"
#include <stdint.h>
#include <stdio.h>

static const xgc_host_api *host_api;
static FILE *out;

static void *create(const xgc_host_api *api) {
  host_api = api;
  return &host_api;
}

static xgc_status configure(void *self, const char *text) {
  (void)self;
  (void)text;
  out = fopen(OUTPUT, "w");
  return out ? XGC_OK : XGC_ERR;
}

static xgc_status life(void *self) {
  (void)self;
  return XGC_OK;
}

static xgc_status step(void *self, const xgc_step_ctx *ctx) {
  (void)self;
  (void)ctx;
  xgc_sample_view sample;
  while (host_api->next(host_api->host, 0, &sample) == XGC_OK) {
    if (sample.len != 240 || sample.data == NULL)
      return XGC_ERR;
    uint32_t schema = (uint32_t)sample.data[0] | ((uint32_t)sample.data[1] << 8) |
                      ((uint32_t)sample.data[2] << 16) | ((uint32_t)sample.data[3] << 24);
    if (schema != 1)
      return XGC_ERR;
    fprintf(out, "mission 240\n");
    fflush(out);
  }
  return XGC_OK;
}

static void destroy(void *self) {
  (void)self;
  if (out) {
    fclose(out);
    out = NULL;
  }
}

static const char *state(void *self) {
  (void)self;
  return "mission-sink";
}

static const xgc_port_decl ports[] = {
    {"mission_request", XGC_PORT_IN, "xgc.dmpc.mission_timeline/1", XGC_QOS_EVENT},
};

static const xgc_plugin_vtbl vtbl = {create, configure, life, step, life, destroy, state};
static const xgc_plugin_descriptor descriptor = {1, 1, "mission-sink", "1", ports, &vtbl};

const xgc_plugin_descriptor *xgc_rt_plugin_v1(void) { return &descriptor; }
