/* NeighborExchange for C and C++ planners: the same contract as the Rust
 * `xgc_rt_abi::neighbor::NeighborExchange` (crates/xgc-rt-abi/src/neighbor.rs),
 * header-only, on the plugin host API (xgc_rt.h, abi_minor >= 1).
 *
 * A distributed planner publishes its own plan for round k and, at round k,
 * solves with each neighbor's plan produced for round k - 1.
 *
 *   xgc_nx nx;
 *   xgc_nx_open(&nx, host, PLAN_IN, PLAN_OUT, s_max);   // neighbors = plan_in's `from`
 *   // each step:   xgc_nx_absorb(&nx, host, k);        // or xgc_nx_offer per sample
 *   // at round k:  xgc_nx_view(&nx, i, k, now, &view) for i < nx.count
 *   xgc_nx_close(&nx);
 *
 * Admission: a plan is admitted only when `origin` is a neighbor and its
 * round is <= the planner round passed to `offer`; a refused plan is not
 * cached. Per neighbor the newest admitted (round, seq) is kept; an older or
 * duplicate plan does not replace it.
 *
 * Status of neighbor j in the snapshot of round k (latest admitted plan):
 *   XGC_NX_FRESH    produced for round k - 1 <= round <= k;
 *   XGC_NX_STALE    produced for round k - 1 - n, 1 <= n <= s_max (n in
 *                   `stale_rounds`);
 *   XGC_NX_MISSING  nothing admitted, older than the stale window, or a
 *                   cached round in the future of this snapshot.
 * Nothing blocks. A snapshot is also a wire record, published on a planner's
 * optional `neighbors` port (schema "xgc.dmpc.neighbor_snapshot/1"):
 * xgc_nx_snapshot_head_v1 then `count` xgc_nx_snapshot_entry_v1, little-endian.
 */
#ifndef XGC_RT_NX_H
#define XGC_RT_NX_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "xgc_rt.h"

#ifdef __cplusplus
extern "C" {
#endif

#define XGC_NX_FRESH 0u
#define XGC_NX_STALE 1u
#define XGC_NX_MISSING 2u
/* `round` of a neighbor never admitted. */
#define XGC_NX_NO_ROUND UINT64_MAX
#define XGC_NX_MAX_NEIGHBORS 256u

typedef struct xgc_nx_latest {
  uint16_t origin;
  uint8_t has;
  uint64_t round;
  uint64_t seq;
  int64_t t_produce;
  uint8_t* data;
  uint32_t len;
  uint32_t cap;
} xgc_nx_latest;

typedef struct xgc_nx {
  uint32_t plan_in;
  uint32_t plan_out;
  uint64_t s_max;
  uint32_t count; /* neighbors */
  xgc_nx_latest* n;
} xgc_nx;

typedef struct xgc_nx_view {
  uint16_t origin;
  uint8_t status;         /* XGC_NX_* */
  uint64_t stale_rounds;  /* n when XGC_NX_STALE, else 0 */
  uint64_t round;         /* XGC_NX_NO_ROUND when never admitted */
  int64_t age_ns;         /* now - t_produce; 0 when never admitted */
  const uint8_t* data;    /* NULL when Missing */
  uint32_t len;
} xgc_nx_view;

typedef struct xgc_nx_snapshot_head_v1 {
  uint64_t round;
  uint32_t count;
  uint32_t reserved;
} xgc_nx_snapshot_head_v1;

typedef struct xgc_nx_snapshot_entry_v1 {
  uint16_t origin;
  uint8_t status;
  uint8_t reserved0;
  uint32_t reserved1;
  uint64_t stale_rounds;
  uint64_t round;
  int64_t age_ns;
} xgc_nx_snapshot_entry_v1;

/* Neighbors given explicitly. */
static inline xgc_status xgc_nx_init(xgc_nx* nx, const uint16_t* neighbors, uint32_t count, uint32_t plan_in,
                                     uint32_t plan_out, uint64_t s_max) {
  uint32_t i;
  memset(nx, 0, sizeof *nx);
  if (count > XGC_NX_MAX_NEIGHBORS) return XGC_ERR_INVALID;
  nx->plan_in = plan_in;
  nx->plan_out = plan_out;
  nx->s_max = s_max;
  if (count == 0) return XGC_OK;
  nx->n = (xgc_nx_latest*)calloc(count, sizeof *nx->n);
  if (!nx->n) return XGC_ERR;
  nx->count = count;
  for (i = 0; i < count; ++i) nx->n[i].origin = neighbors[i];
  return XGC_OK;
}

/* Neighbors are plan_in's bound origins (the manifest `from`). */
static inline xgc_status xgc_nx_open(xgc_nx* nx, const xgc_host_api* host, uint32_t plan_in, uint32_t plan_out,
                                     uint64_t s_max) {
  uint16_t ids[XGC_NX_MAX_NEIGHBORS];
  uint32_t count;
  if (host->abi_minor < 1 || !host->port_origins) return XGC_ERR_INVALID;
  count = host->port_origins(host->host, plan_in, ids, XGC_NX_MAX_NEIGHBORS);
  if (count > XGC_NX_MAX_NEIGHBORS) return XGC_ERR_INVALID;
  return xgc_nx_init(nx, ids, count, plan_in, plan_out, s_max);
}

static inline void xgc_nx_close(xgc_nx* nx) {
  uint32_t i;
  for (i = 0; i < nx->count; ++i) free(nx->n[i].data);
  free(nx->n);
  memset(nx, 0, sizeof *nx);
}

/* Admit one received plan at planner round `planner_k`: 1 admitted (cached
 * when newer), 0 refused (not a neighbor, or a future round), -1 out of
 * memory. */
static inline int xgc_nx_offer(xgc_nx* nx, uint64_t planner_k, uint16_t origin, uint64_t round, uint64_t seq,
                               int64_t t_produce, const uint8_t* data, uint32_t len) {
  uint32_t i;
  xgc_nx_latest* l = NULL;
  for (i = 0; i < nx->count; ++i) {
    if (nx->n[i].origin == origin) l = &nx->n[i];
  }
  if (!l || round > planner_k) return 0;
  if (l->has && !(round > l->round || (round == l->round && seq > l->seq))) return 1;
  if (len > l->cap) {
    uint8_t* grown = (uint8_t*)realloc(l->data, len);
    if (!grown) return -1;
    l->data = grown;
    l->cap = len;
  }
  if (len) memcpy(l->data, data, len);
  l->len = len;
  l->has = 1;
  l->round = round;
  l->seq = seq;
  l->t_produce = t_produce;
  return 1;
}

/* Drain plan_in, offering every sample at `planner_k`. Returns samples read. */
static inline uint32_t xgc_nx_absorb(xgc_nx* nx, const xgc_host_api* host, uint64_t planner_k) {
  xgc_sample_view s;
  uint32_t read = 0;
  while (host->next(host->host, nx->plan_in, &s) == XGC_OK) {
    ++read;
    (void)xgc_nx_offer(nx, planner_k, s.origin, s.round, s.seq, s.t_produce, s.data, s.len);
  }
  return read;
}

/* Neighbor i (< nx->count) in the snapshot of round k at `now`. */
static inline void xgc_nx_view_of(const xgc_nx* nx, uint32_t i, uint64_t k, int64_t now, xgc_nx_view* out) {
  const xgc_nx_latest* l = &nx->n[i];
  const uint64_t expected = k > 0 ? k - 1 : 0;
  memset(out, 0, sizeof *out);
  out->origin = l->origin;
  out->round = XGC_NX_NO_ROUND;
  out->status = XGC_NX_MISSING;
  if (!l->has) return;
  out->round = l->round;
  out->age_ns = now - l->t_produce;
  if (l->round > k) {
    out->status = XGC_NX_MISSING;
  } else if (l->round >= expected) {
    out->status = XGC_NX_FRESH;
  } else if (expected - l->round <= nx->s_max) {
    out->status = XGC_NX_STALE;
    out->stale_rounds = expected - l->round;
  }
  if (out->status != XGC_NX_MISSING) {
    out->data = l->data;
    out->len = l->len;
  }
}

/* Bytes of the snapshot record for this exchange. */
static inline uint32_t xgc_nx_snapshot_size(const xgc_nx* nx) {
  return (uint32_t)(sizeof(xgc_nx_snapshot_head_v1) + nx->count * sizeof(xgc_nx_snapshot_entry_v1));
}

/* Write the snapshot of round k into `out` (at least xgc_nx_snapshot_size
 * bytes). Returns the bytes written, or 0 when `cap` is too small. */
static inline uint32_t xgc_nx_snapshot_encode(const xgc_nx* nx, uint64_t k, int64_t now, uint8_t* out, uint32_t cap) {
  xgc_nx_snapshot_head_v1 head;
  uint32_t i;
  const uint32_t size = xgc_nx_snapshot_size(nx);
  if (cap < size) return 0;
  memset(&head, 0, sizeof head);
  head.round = k;
  head.count = nx->count;
  memcpy(out, &head, sizeof head);
  for (i = 0; i < nx->count; ++i) {
    xgc_nx_view v;
    xgc_nx_snapshot_entry_v1 e;
    xgc_nx_view_of(nx, i, k, now, &v);
    memset(&e, 0, sizeof e);
    e.origin = v.origin;
    e.status = v.status;
    e.stale_rounds = v.stale_rounds;
    e.round = v.round;
    e.age_ns = v.age_ns;
    memcpy(out + sizeof head + i * sizeof e, &e, sizeof e);
  }
  return size;
}

static inline xgc_status xgc_nx_publish(const xgc_nx* nx, const xgc_host_api* host, uint64_t round,
                                        const uint8_t* plan, uint32_t len) {
  return host->publish(host->host, nx->plan_out, round, plan, len);
}

#ifdef __cplusplus
}
#endif

#endif /* XGC_RT_NX_H */
