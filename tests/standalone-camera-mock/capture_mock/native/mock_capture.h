// Local prototype ABI. The production boundary remains Monado's xrt_fs.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include "xrt/xrt_frameserver.h"
#ifdef __cplusplus
extern "C" {
#endif
struct wrl_config {
    uint32_t width, height, padding, fps_num, fps_den, frames;
    uint32_t camera_id, seed, queue_capacity, interrupt_after;
    uint32_t malformed_sequence, malformed_kind, hold_first, session_id;
};
struct wrl_frame_view {
    uint32_t camera_id, session_id, width, height, stride, format, payload_crc, reserved;
    uint64_t sequence, source_id, simulated_capture_ns, published_timestamp_ns;
    uint64_t ready_ns, arrival_ns, payload_size;
    const uint8_t *data;
};
struct wrl_stats {
    uint64_t generated, pushed, received, processed, rejected;
    uint64_t live_frames, peak_live_frames, retained_valid, producer_done;
    uint64_t interrupted, started_ns, ended_ns, native_error;
};
typedef int (*wrl_receive_callback)(void *, const struct wrl_frame_view *);
/* Real native factory: caller supplies its normal sink to xrt_fs_stream_start.
 * Frame-context destruction stops/deletes this source. No tracker is created. */
struct xrt_fs *wrl_mock_capture_create(struct xrt_frame_context *, const struct wrl_config *);
/* Image-only bundle: config[i] routes to sinks.cams[i]. All configured routes
 * required and distinct. IMU/pose/mask requests return NOT_IMPLEMENTED.
 * Source/frame contexts own the group and its children. Recreate to restart. */
struct xrt_fs *wrl_mock_capture_group_create(
    struct xrt_frame_context *, const struct wrl_config *configs, uint32_t count);
/* Explicit format variants: generated grayscale bytes, R8 or L8 only.
 * Existing factories keep the R8 default; no physical RAW conversion implied. */
struct xrt_fs *wrl_mock_capture_create_format(struct xrt_frame_context *, const struct wrl_config *, uint32_t format);
struct xrt_fs *wrl_mock_capture_group_create_format(struct xrt_frame_context *, const struct wrl_config *, uint32_t count, uint32_t format);
int wrl_mock_capture_group_schedule(struct xrt_fs *, uint64_t epoch_ns);
int wrl_mock_capture_group_stats(struct xrt_fs *, uint32_t camera_index, struct wrl_stats *);
/* Validation harness: real upstream queue + explicitly NEW observation sink. */
void *wrl_harness_create(const struct wrl_config *, wrl_receive_callback, void *);
int wrl_harness_start(void *, uint64_t start_ns);
int wrl_harness_group_start(void **handles, uint32_t count, uint64_t start_ns);
int wrl_harness_finish(void *, uint32_t timeout_ms);
void wrl_harness_stop(void *);
void wrl_harness_stats(void *, struct wrl_stats *);
void wrl_harness_close(void *);
const char *wrl_last_error(void);
uint32_t wrl_stats_size(void);
uint32_t wrl_config_size(void);
uint32_t wrl_frame_view_size(void);
uint32_t wrl_format_r8(void);
#ifdef __cplusplus
}
#endif
