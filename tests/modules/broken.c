/* Descriptors the loader must refuse. Compiled once per -DVARIANT_<name>. */
#include "common.h"

static xgc2_status create(const xgc2_host_api* host, void* ctx, const xgc2_config* config, xgc2_instance** out) {
  (void)host; (void)ctx; (void)config; (void)out;
  return XGC2_OK;
}
static xgc2_status configure(xgc2_instance* i, const xgc2_config* c) { (void)i; (void)c; return XGC2_OK; }
static xgc2_status lifecycle(xgc2_instance* i) { (void)i; return XGC2_OK; }
static void destroy(xgc2_instance* i) { (void)i; }

#define STATE_OUT(align) {"p", XGC2_PORT_OUT, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), align, 0, 0}

#if defined(VARIANT_ABI1) || defined(VARIANT_NEWER_MINOR) || defined(VARIANT_NO_STEP)
static const xgc2_port_desc PORTS[] = {STATE_OUT(8)};
#elif defined(VARIANT_DUPLICATE_PORT)
static const xgc2_port_desc PORTS[] = {STATE_OUT(8), {"p", XGC2_PORT_IN, XGC2_PORT_STATE, SAMPLE_SCHEMA, sizeof(test_sample), 8, 0, 0}};
#elif defined(VARIANT_BAD_ALIGN)
static const xgc2_port_desc PORTS[] = {STATE_OUT(3)};
#elif defined(VARIANT_EVENT_WITHOUT_DEPTH)
static const xgc2_port_desc PORTS[] = {{"p", XGC2_PORT_OUT, XGC2_PORT_EVENT, EVENT_SCHEMA, sizeof(test_event), 8, 0, 0}};
#else
#error "define a VARIANT_*"
#endif

#if defined(VARIANT_ABI1)
#define MAJOR 1u
#else
#define MAJOR XGC2_MODULE_ABI_MAJOR
#endif
#if defined(VARIANT_NEWER_MINOR)
#define MINOR 7u
#else
#define MINOR XGC2_MODULE_ABI_MINOR
#endif

#if defined(VARIANT_NO_STEP)
#define STEP NULL
#else
static xgc2_status step(xgc2_instance* i, const xgc2_step_ctx* c) { (void)i; (void)c; return XGC2_OK; }
#define STEP step
#endif

static const xgc2_module_desc DESC = {MAJOR, MINOR, "test_broken", "1.0.0", PORTS, sizeof PORTS / sizeof PORTS[0],
                                      create, configure, lifecycle, STEP, lifecycle, destroy};

TEST_EXPORT const xgc2_module_desc* xgc2_module_v2(void) { return &DESC; }
