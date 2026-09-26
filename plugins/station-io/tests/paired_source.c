#include "xgc_rt.h"
#include <string.h>

static const xgc_host_api *host_api;

static void *create(const xgc_host_api *api) {
  host_api = api;
  return &host_api;
}

static xgc_status configure(void *self, const char *text) {
  (void)self;
  (void)text;
  return XGC_OK;
}

static xgc_status life(void *self) {
  (void)self;
  return XGC_OK;
}

static xgc_status step(void *self, const xgc_step_ctx *ctx) {
  (void)self;
  double stamp = (double)ctx->now * 1e-9;
  if (stamp <= 0.0)
    return XGC_OK;
  double values[12] = {stamp, stamp, 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, 0.25, 0.0, 0.0};
  uint8_t bytes[96];
  memcpy(bytes, values, sizeof bytes);
  return host_api->publish(host_api->host, 0, ctx->round, bytes, sizeof bytes);
}

static void destroy(void *self) { (void)self; }

static const char *state(void *self) {
  (void)self;
  return "paired-source";
}

static const xgc_port_decl ports[] = {
    {"paired_state", XGC_PORT_OUT, "xgc.dmpc.paired_state/1", XGC_QOS_STATE},
};

static const xgc_plugin_vtbl vtbl = {create, configure, life, step, life, destroy, state};
static const xgc_plugin_descriptor descriptor = {1, 1, "paired-source", "1", ports, &vtbl};

const xgc_plugin_descriptor *xgc_rt_plugin_v1(void) { return &descriptor; }
