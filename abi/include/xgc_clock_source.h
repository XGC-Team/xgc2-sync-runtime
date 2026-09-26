/* Host-owned clock source service, separate from the domain plugin ABI.
 * Only the pinned ROS edge exports this entry. No exception crosses it.
 * The host owns the calling thread and keeps the library loaded throughout.
 */
#ifndef XGC_CLOCK_SOURCE_H
#define XGC_CLOCK_SOURCE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define XGC_CLOCK_SOURCE_ABI_VERSION 1u
#define XGC_CLOCK_SOURCE_ENTRY "xgc_rt_clock_source_v1"
#define XGC_CLOCK_TEXT_CAP 256u
#define XGC_CLOCK_OK 0
#define XGC_CLOCK_AGAIN 1
#define XGC_CLOCK_ERROR 2
#define XGC_CLOCK_GATE_CLOSED 0u
#define XGC_CLOCK_GATE_OPEN 1u
#define XGC_CLOCK_GATE_FAULT 2u

typedef struct xgc_clock_observation_v1 {
  int64_t time_ns;                /* exact ROS sec*1e9+nsec; zero is valid */
  uint64_t sequence;              /* strictly increasing for each OK poll */
  uint32_t coalesced;             /* extra valid stamps coalesced this poll */
  uint32_t publisher_count;       /* exact currently connected publishers */
  uint64_t dropped;               /* cumulative queue drops; must remain 0 */
  char publisher[XGC_CLOCK_TEXT_CAP]; /* actual caller ID, UTF-8 + NUL */
  char error[XGC_CLOCK_TEXT_CAP]; /* error detail, UTF-8 + NUL */
} xgc_clock_observation_v1;

typedef struct xgc_clock_source_vtbl_v1 {
  void* (*create)(void);
  /* Strict TOML keys: node_name, topic, expected_publisher, queue_capacity.
   * Initialize/reuse ROS once, require /use_sim_time=true without changing it.
   * Use a private callback queue. Begin with the ROS output gate CLOSED.
   * Error text uses out->error; out is zero-initialized by the host.
   */
  int32_t (*start)(void*, const char* config_toml, xgc_clock_observation_v1* out);
  /* Bounded wall-time poll; never wait using ROS/Session time.
   * OK: one sample (possibly a validated/coalesced batch); AGAIN: no sample.
   * All returns report publisher_count/dropped and errors when applicable.
   * Validate EVERY intermediate stamp before coalescing; backward time,
   * malformed stamp, unexpected/multiple publisher or drops returns ERROR.
   * An AGAIN result's time_ns/sequence/publisher are not clock samples.
   */
  int32_t (*poll)(void*, uint64_t timeout_wall_ns, xgc_clock_observation_v1* out);
  /* Same source thread as poll. Shared atomic gate used by ordinary ros_io:
   * CLOSED/FAULT suppress and discard pending actuator/service output; OPEN
   * permits current output. Input clock reception is never gated. The host
   * independently gates domain steps/publication; this gates the ROS edge.
   */
  int32_t (*set_gate)(void*, uint32_t gate);
  void (*stop)(void*);             /* close gate and release clock subscription */
  void (*destroy)(void*);
} xgc_clock_source_vtbl_v1;

typedef struct xgc_clock_source_descriptor_v1 {
  uint32_t abi_version;
  uint32_t reserved;              /* must be zero */
  const xgc_clock_source_vtbl_v1* vtbl;
} xgc_clock_source_descriptor_v1;

typedef const xgc_clock_source_descriptor_v1* (*xgc_clock_source_entry_v1)(void);
/* Export with default visibility:
 * const xgc_clock_source_descriptor_v1* xgc_rt_clock_source_v1(void);
 */
#ifdef __cplusplus
}
#endif
#endif
