/* Drives abi/include/xgc_rt_nx.h from a line script (neighbor_exchange_c.rs):
 *   i COUNT ORIGIN... S_MAX                 init
 *   o K ORIGIN ROUND SEQ T_PRODUCE LEN       offer (payload: LEN bytes of SEQ & 0xff) -> "o RET"
 *   s K NOW                                  snapshot -> "v ORIGIN STATUS STALE ROUND AGE LEN FIRST" per neighbor
 *   e K NOW                                  encoded snapshot record -> "e HEX"
 * Built as C11 and as C++17. */
#include <inttypes.h>
#include <stdio.h>

#include "xgc_rt_nx.h"

int main(void) {
  char op[4];
  xgc_nx nx;
  uint8_t payload[4096];
  uint8_t record[65536];
  memset(&nx, 0, sizeof nx);
  while (scanf("%3s", op) == 1) {
    if (op[0] == 'i') {
      uint32_t count, i;
      uint16_t ids[XGC_NX_MAX_NEIGHBORS];
      uint64_t s_max;
      if (scanf("%" SCNu32, &count) != 1 || count > XGC_NX_MAX_NEIGHBORS) return 2;
      for (i = 0; i < count; ++i) {
        if (scanf("%" SCNu16, &ids[i]) != 1) return 2;
      }
      if (scanf("%" SCNu64, &s_max) != 1) return 2;
      xgc_nx_close(&nx);
      if (xgc_nx_init(&nx, ids, count, 0, 1, s_max) != XGC_OK) return 3;
    } else if (op[0] == 'o') {
      uint64_t k, round, seq;
      uint16_t origin;
      int64_t t;
      uint32_t len;
      if (scanf("%" SCNu64 " %" SCNu16 " %" SCNu64 " %" SCNu64 " %" SCNd64 " %" SCNu32, &k, &origin, &round, &seq, &t,
                &len) != 6 ||
          len > sizeof payload)
        return 2;
      memset(payload, (int)(seq & 0xff), len);
      printf("o %d\n", xgc_nx_offer(&nx, k, origin, round, seq, t, payload, len));
    } else if (op[0] == 's' || op[0] == 'e') {
      uint64_t k;
      int64_t now;
      uint32_t i;
      if (scanf("%" SCNu64 " %" SCNd64, &k, &now) != 2) return 2;
      if (op[0] == 'e') {
        const uint32_t n = xgc_nx_snapshot_encode(&nx, k, now, record, sizeof record);
        printf("e ");
        for (i = 0; i < n; ++i) printf("%02x", record[i]);
        printf("\n");
        continue;
      }
      for (i = 0; i < nx.count; ++i) {
        xgc_nx_view v;
        xgc_nx_view_of(&nx, i, k, now, &v);
        printf("v %" PRIu16 " %u %" PRIu64 " %" PRIu64 " %" PRId64 " %" PRIu32 " %d\n", v.origin, (unsigned)v.status,
               v.stale_rounds, v.round, v.age_ns, v.len, v.len ? (int)v.data[0] : -1);
      }
    } else {
      return 2;
    }
  }
  xgc_nx_close(&nx);
  return 0;
}
