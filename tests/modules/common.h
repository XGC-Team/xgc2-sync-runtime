/* Shared helpers of the test modules. Each module is one small file that includes this. */
#ifndef XGC2_TEST_COMMON_H
#define XGC2_TEST_COMMON_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <xgc2/module.h>

#ifdef __cplusplus
#define TEST_EXPORT extern "C"
#else
#define TEST_EXPORT
#endif

/* Payloads. Sizes and alignments are part of the port descriptors. */
typedef struct test_sample {
  uint64_t counter;       /* per producer, starts at 1 */
  uint64_t addr;          /* address the producer got from write_begin: proves zero-copy */
  int64_t committed_ns;   /* host clock right before write_commit */
  uint64_t producer;
  uint8_t pad[32];
} test_sample;            /* 64 bytes */

typedef struct test_event {
  uint64_t producer;
  uint64_t seq;           /* per producer, starts at 1 */
} test_event;             /* 16 bytes */

typedef struct test_clock {
  int64_t time_ns;
} test_clock;             /* schema xgc2.clock.v1 */

#define SAMPLE_SCHEMA "test.sample.v1"
#define EVENT_SCHEMA "test.event.v1"
#define CLOCK_SCHEMA "xgc2.clock.v1"

/* Number after "key": in the flat JSON configuration, or the fallback. Test configurations
 * have no nesting and no escapes. */
static inline double cfg_number(const xgc2_config* config, const char* key, double fallback) {
  char pattern[80];
  snprintf(pattern, sizeof pattern, "\"%s\"", key);
  const char* at = config && config->json ? strstr(config->json, pattern) : NULL;
  if (!at) return fallback;
  at += strlen(pattern);
  while (*at == ' ' || *at == ':') at++;
  if (strncmp(at, "true", 4) == 0) return 1;
  if (strncmp(at, "false", 5) == 0) return 0;
  char* end = NULL;
  double value = strtod(at, &end);
  return end == at ? fallback : value;
}

/* String after "key": "..."; empty when absent. */
static inline void cfg_string(const xgc2_config* config, const char* key, char* out, size_t size) {
  char pattern[80];
  snprintf(pattern, sizeof pattern, "\"%s\"", key);
  out[0] = 0;
  const char* at = config && config->json ? strstr(config->json, pattern) : NULL;
  if (!at) return;
  at = strchr(at + strlen(pattern), '"');
  if (!at) return;
  const char* end = strchr(at + 1, '"');
  if (!end) return;
  size_t length = (size_t)(end - at - 1);
  if (length >= size) length = size - 1;
  memcpy(out, at + 1, length);
  out[length] = 0;
}

static inline void pause_us(long microseconds) {
  struct timespec ts = {microseconds / 1000000, (microseconds % 1000000) * 1000};
  nanosleep(&ts, NULL);
}

#endif
