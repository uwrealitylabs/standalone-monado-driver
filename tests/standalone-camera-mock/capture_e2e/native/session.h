#pragma once
#include "mock_capture.h"
#include "matrix.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Error codes: 0 valid; 1 format; 2 stride; 3 payload; 4 dimensions. */
typedef int (*E2ESourceCallback)(void *, const struct MatrixView *, uint32_t error_code);
struct E2EStats {
    struct wrl_stats sources[4]; /* Snapshot after source stop, before graph teardown. */
    uint64_t guard_seen[4], guard_rejected[4];
    uint64_t source_timed_out, finished;
};
void *e2e_create(const struct wrl_config *, uint32_t count, uint32_t converter,
                 uint32_t pairing, MatrixCallback, E2ESourceCallback, void *);
int e2e_start(void *, uint64_t epoch_ns);
void e2e_stop(void *);
int e2e_finish(void *, uint32_t timeout_ms);
void e2e_stats(void *, struct E2EStats *);
void e2e_pipeline_stats(void *, struct MatrixStats *);
void e2e_close(void *);
uint32_t e2e_stats_size(void);
uint32_t e2e_format(uint32_t);
#ifdef __cplusplus
}
#endif
