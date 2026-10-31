// Independent native lifecycle/retained-buffer test, using the public xrt_fs boundary.
#include "mock_capture.h"
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <thread>
static std::atomic<unsigned> received{0};
static xrt_frame *retained = nullptr;
static void observe(xrt_frame_sink *, xrt_frame *frame)
{
    if (!retained) xrt_frame_reference(&retained, frame);
    if (frame->format == XRT_FORMAT_R8 && frame->width == 64 && frame->size == 64 * 64 &&
        frame->data[0] == 87 && frame->data[1] == 82 && frame->data[2] == 76) received++;
}
static int validation_callback(void *, const wrl_frame_view *frame)
{
    return frame->format == wrl_format_r8() ? 0 : 1;
}
int main()
{
    wrl_config config{64,64,0,1000,1,8,0,1,16,8,0xffffffff,0,0,1};
    xrt_frame_context context{};
    xrt_fs *source = wrl_mock_capture_create(&context, &config);
    if (!source) return 1;
    xrt_fs_mode *modes = nullptr;
    uint32_t count = 0;
    if (xrt_fs_enumerate_modes(source, &modes, &count) != XRT_SUCCESS ||
        count != 1 || modes[0].width != 64 || modes[0].format != XRT_FORMAT_R8) return 2;
    std::free(modes);
    xrt_frame_sink sink{observe};
    if (xrt_fs_stream_start(source, nullptr, XRT_FS_CAPTURE_TYPE_TRACKING, 0) == XRT_SUCCESS) return 3;
    if (xrt_fs_stream_start(source, &sink, XRT_FS_CAPTURE_TYPE_TRACKING, 2) == XRT_SUCCESS) return 4;
    if (xrt_fs_stream_start(source, &sink, XRT_FS_CAPTURE_TYPE_TRACKING, 0) != XRT_SUCCESS) return 5;
    if (xrt_fs_stream_start(source, &sink, XRT_FS_CAPTURE_TYPE_TRACKING, 0) == XRT_SUCCESS) return 6;
    bool running = true;
    for (int i=0; i<1000 && running; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        xrt_fs_is_running(source, &running);
    }
    xrt_fs_stream_stop(source);
    xrt_frame_context_destroy_nodes(&context);
    // Retention survives later captures and even source destruction.
    bool valid = received == 8 && retained && retained->source_sequence == 0 &&
                 retained->data[0] == 87 && retained->data[5] == 0;
    xrt_frame_reference(&retained, nullptr);
    config.fps_num = 120;
    config.malformed_sequence = 2;
    config.malformed_kind = 1;
    config.hold_first = 1;
    void *harness = wrl_harness_create(&config, validation_callback, nullptr);
    if (!harness || wrl_harness_start(harness, 0)) return 9;
    if (wrl_harness_finish(harness, 2000)) return 10;
    wrl_stats stats{};
    wrl_harness_stats(harness, &stats);
    bool harness_valid = stats.generated == 8 && stats.processed == 7 &&
                         stats.rejected == 1 && stats.live_frames == 0 && stats.retained_valid == 1;
    wrl_harness_close(harness);
    if (!harness_valid) return 11;
    config.width = 0;
    if (wrl_mock_capture_create(&context, &config)) return 7;
    return valid && !running ? 0 : 8;
}
