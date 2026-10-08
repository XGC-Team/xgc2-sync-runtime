/* Independently compiled C module: no Rust/Arc/Runtime layout crosses a DSO. */
#include <xgc_rt.h>
#include <xgc2/xrpc.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

struct instance {
  const xgc_host_api *host_api;
  xgc2_xrpc_runtime_api_v1 rpc;
  void *endpoint;
  char path[108];
  _Atomic unsigned refs;
};
static _Atomic unsigned created, activated, destroyed, callbacks, blocked;
static _Atomic unsigned bad_getters, close_timeouts, caps_seen, destroy_calls;
static pthread_mutex_t gate_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_changed = PTHREAD_COND_INITIALIZER;
static int gate_open;

unsigned bridge_count(unsigned kind) {
  switch (kind) {
    case 0: return atomic_load(&created);
    case 1: return atomic_load(&activated);
    case 2: return atomic_load(&destroyed);
    case 3: return atomic_load(&callbacks);
    case 4: return atomic_load(&blocked);
    case 5: return atomic_load(&bad_getters);
    case 6: return atomic_load(&close_timeouts);
    case 7: return atomic_load(&caps_seen);
    case 8: return atomic_load(&destroy_calls);
    default: return 0;
  }
}
void bridge_open_gate(void) {
  pthread_mutex_lock(&gate_lock);
  gate_open = 1;
  pthread_cond_broadcast(&gate_changed);
  pthread_mutex_unlock(&gate_lock);
}
static int32_t retain_instance(void *context) {
  atomic_fetch_add(&((struct instance *)context)->refs, 1);
  return XGC2_XRPC_OK;
}
static void release_instance(void *context) {
  struct instance *self = context;
  if (atomic_fetch_sub(&self->refs, 1) == 1) {
    atomic_fetch_add(&destroyed, 1);
    free(self);
  }
}
static int32_t callback(void *context, const xgc2_xrpc_request_v1 *request,
                        uint8_t *output, size_t capacity, size_t *written) {
  (void)context;
  atomic_fetch_add(&callbacks, 1);
  if (request->path.len == 6 && memcmp(request->path.data, "/block", 6) == 0) {
    atomic_fetch_add(&blocked, 1);
    pthread_mutex_lock(&gate_lock);
    while (!gate_open) pthread_cond_wait(&gate_changed, &gate_lock);
    pthread_mutex_unlock(&gate_lock);
  }
  const char result[] = "{\"ok\":true}";
  if (capacity < sizeof(result) - 1) return XGC2_XRPC_RESOURCE_EXHAUSTED;
  memcpy(output, result, sizeof(result) - 1);
  *written = sizeof(result) - 1;
  return XGC2_XRPC_OK;
}
static void *create(const xgc_host_api *api) {
  if (api->abi_minor < 3 || !api->rpc_runtime) return NULL;
  const xgc2_xrpc_runtime_api_v1 *rpc = api->rpc_runtime(api->host);
  if (!rpc || rpc->abi_version != 1 || rpc->struct_size < sizeof(*rpc)) return NULL;
  struct instance *self = calloc(1, sizeof(*self));
  if (!self) return NULL;
  self->host_api = api;
  self->rpc = *rpc;
  atomic_init(&self->refs, 1);
  if (self->rpc.retain(self->rpc.context) != XGC2_XRPC_OK) { free(self); return NULL; }
  if (rpc->baseline_caps.max_connections == 2 &&
      rpc->baseline_caps.max_in_flight == 4 &&
      rpc->baseline_caps.max_response_bytes == 256 &&
      rpc->baseline_caps.shutdown_timeout_ms == 30) atomic_fetch_add(&caps_seen, 1);
  atomic_fetch_add(&created, 1);
  return self;
}
static xgc_status configure(void *context, const char *config) {
  struct instance *self = context;
  if (self->host_api->rpc_runtime(self->host_api->host) != NULL) atomic_fetch_add(&bad_getters, 1);
  const char *start = strchr(config, '"');
  if (!start) return XGC_ERR_INVALID;
  const char *end = strchr(++start, '"');
  if (!end || (size_t)(end - start) >= sizeof(self->path)) return XGC_ERR_INVALID;
  memcpy(self->path, start, (size_t)(end - start));
  return XGC_OK;
}
static xgc_status activate(void *context) {
  struct instance *self = context;
  const xgc2_xrpc_runtime_api_v1 *current = self->host_api->rpc_runtime(self->host_api->host);
  if (!current || current->context != self->rpc.context) return XGC_ERR_INVALID;
  const char incarnation[] = "bridge:fixture";
  xgc2_xrpc_bind_v1 config = {
    .abi_version = 1, .struct_size = sizeof(config),
    .path = { (const uint8_t *)self->path, strlen(self->path) },
    .instance_id = { (const uint8_t *)incarnation, sizeof(incarnation) - 1 },
    .reclaim_unreachable = 1,
  };
  xgc2_xrpc_handler_v1 handler = {
    .abi_version = 1, .struct_size = sizeof(handler), .context = self,
    .retain = retain_instance, .release = release_instance, .call = callback,
  };
  if (self->rpc.bind_http(self->rpc.context, &config, &handler, &self->endpoint) != XGC2_XRPC_OK) return XGC_ERR;
  atomic_fetch_add(&activated, 1);
  return XGC_OK;
}
static xgc_status step(void *context, const xgc_step_ctx *ctx) {
  (void)ctx;
  struct instance *self = context;
  if (self->host_api->rpc_runtime(self->host_api->host) != NULL) atomic_fetch_add(&bad_getters, 1);
  return XGC_OK;
}
static xgc_status deactivate(void *context) {
  struct instance *self = context;
  if (!self->endpoint) return XGC_OK;
  self->rpc.host_stop(self->rpc.context, self->endpoint);
  int32_t result = self->rpc.host_close(self->rpc.context, self->endpoint);
  if (result != XGC2_XRPC_OK) atomic_fetch_add(&close_timeouts, 1);
  return result == XGC2_XRPC_OK ? XGC_OK : XGC_ERR;
}
static void destroy(void *context) {
  struct instance *self = context;
  atomic_fetch_add(&destroy_calls, 1);
  if (self->endpoint) {
    self->rpc.host_release(self->rpc.context, self->endpoint);
    self->endpoint = NULL;
  }
  self->rpc.release(self->rpc.context);
  release_instance(self);
}
static const char *domain_state(void *context) { (void)context; return "bridge"; }
static const xgc_plugin_vtbl vtable = { create, configure, activate, step, deactivate, destroy, domain_state };
static const xgc_plugin_descriptor descriptor = { 1, 0, "rpc-bridge-fixture", "1", NULL, &vtable };
const xgc_plugin_descriptor *xgc_rt_plugin_v1(void) { return &descriptor; }
