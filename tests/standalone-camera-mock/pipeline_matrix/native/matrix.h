#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
struct MatrixView {
    uint32_t route, format, width, height;
    uint64_t stride, size, sequence, source_id;
    int64_t timestamp, source_timestamp;
    uint64_t arrival_ns;
    const uint8_t *data;
};
struct MatrixStats {
    uint64_t sent, converted, received, processed, rejected, live, peak, retained_valid, errors, warnings;
};
typedef int (*MatrixCallback)(void *, const struct MatrixView *);
void *matrix_create(uint32_t,uint32_t,uint32_t,uint32_t,MatrixCallback,void *);
int matrix_push(void *,uint32_t,uint32_t,uint32_t,uint32_t,uint64_t,const uint8_t *,uint64_t,
                uint64_t,int64_t,int64_t);
int matrix_finish(void *,uint32_t,uint32_t);
void matrix_stats(void *,struct MatrixStats *);
struct xrt_frame;
int matrix_accept_frame(void *, uint32_t route, struct xrt_frame *);
void matrix_close(void *);
uint32_t matrix_format(uint32_t);
uint32_t matrix_view_size(void);
uint32_t matrix_stats_size(void);
#ifdef __cplusplus
}
#endif
