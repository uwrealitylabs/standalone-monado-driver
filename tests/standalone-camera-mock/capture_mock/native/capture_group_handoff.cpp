// Independent receiver test: public xrt frames only, no mock-private frame owner.
#include "mock_capture.h"
#include "xrt/xrt_tracking.h"
#include "util/u_sink.h"
#include <atomic>
#include <chrono>
#include <cstddef>
#include <thread>
struct Endpoint {
    xrt_frame_sink sink{};
    unsigned expected = 0;
    std::atomic<unsigned> seen{0}, errors{0};
    xrt_frame *retained = nullptr;
    uint64_t first_timestamp = 0;
    static void push(xrt_frame_sink *sink, xrt_frame *frame)
    {
        auto *self = reinterpret_cast<Endpoint *>(sink);
        if (frame->format != XRT_FORMAT_R8 || frame->width != 64 || frame->height != 64 ||
            frame->stride != 64 || frame->size != 4096 || frame->source_id != 0x57524C47ULL ||
            frame->data[0] != 87 || frame->data[4] != self->expected ||
            frame->data[5] != frame->source_sequence) self->errors++;
        if (!self->retained) {
            xrt_frame_reference(&self->retained, frame);
            self->first_timestamp = uint64_t(frame->timestamp);
        }
        self->seen++;
    }
};
struct Graph {
    xrt_frame_context ctx{};
    ~Graph() { xrt_frame_context_destroy_nodes(&ctx); }
};
int main()
{
    Endpoint endpoints[4]; // Must outlive the graph, including early failure cleanup.
    Graph graph;
    wrl_config configs[4];
    unsigned ids[4] = {2, 0, 3, 1}; // Routing is array order, not an assumed ID/index match.
    for (unsigned i = 0; i < 4; i++)
        configs[i] = {64,64,0,120,1,8,ids[i],1,32,8,0xffffffff,0,0,1};
    if (wrl_mock_capture_group_create(&graph.ctx, configs, 0)) return 1;
    configs[1].camera_id = 2;
    if (wrl_mock_capture_group_create(&graph.ctx, configs, 4)) return 2;
    configs[1].camera_id = 0;
    xrt_slam_sinks bundle{};
    bundle.cam_count = 4;
    for (unsigned i = 0; i < 4; i++) {
        endpoints[i].expected = ids[i];
        endpoints[i].sink.push_frame = Endpoint::push;
        if (!u_sink_queue_create(&graph.ctx, 32, &endpoints[i].sink, &bundle.cams[i])) return 3;
    }
    xrt_fs *source = wrl_mock_capture_group_create(&graph.ctx, configs, 4);
    if (!source) return 4;
    if (xrt_fs_slam_stream_start(source, nullptr) != XRT_ERROR_INVALID_ARGUMENT) return 5;
    auto invalid = bundle;
    invalid.cam_count = 3;
    if (xrt_fs_slam_stream_start(source, &invalid) != XRT_ERROR_INVALID_ARGUMENT) return 6;
    invalid = bundle; invalid.cams[1] = nullptr;
    if (xrt_fs_slam_stream_start(source, &invalid) != XRT_ERROR_INVALID_ARGUMENT) return 7;
    invalid = bundle; invalid.cams[1] = invalid.cams[0];
    if (xrt_fs_slam_stream_start(source, &invalid) != XRT_ERROR_INVALID_ARGUMENT) return 8;
    invalid = bundle; invalid.imu_count = 1;
    if (xrt_fs_slam_stream_start(source, &invalid) != XRT_ERROR_NOT_IMPLEMENTED) return 9;
    xrt_imu_sink imu{};
    invalid = bundle; invalid.imus[0] = &imu;
    if (xrt_fs_slam_stream_start(source, &invalid) != XRT_ERROR_NOT_IMPLEMENTED) return 10;
    xrt_pose_sink pose{};
    invalid = bundle; invalid.gt = &pose;
    if (xrt_fs_slam_stream_start(source, &invalid) != XRT_ERROR_NOT_IMPLEMENTED) return 11;
    xrt_device_masks_sink mask{};
    invalid = bundle; invalid.masks[0] = &mask;
    if (xrt_fs_slam_stream_start(source, &invalid) != XRT_ERROR_NOT_IMPLEMENTED) return 12;
    if (xrt_fs_stream_start(source, bundle.cams[0], XRT_FS_CAPTURE_TYPE_TRACKING, 0) !=
        XRT_ERROR_NOT_IMPLEMENTED) return 13;
    bool running = true;
    xrt_fs_is_running(source, &running);
    if (running) return 14; // Rejected requests must not start any camera.
    if (xrt_fs_slam_stream_start(source, &bundle) != XRT_SUCCESS) return 15;
    if (xrt_fs_slam_stream_start(source, &bundle) != XRT_ERROR_INVALID_ARGUMENT) return 16;
    bool received_all = false;
    for (unsigned attempt = 0; attempt < 2000 && !received_all; attempt++) {
        received_all = true;
        for (auto &endpoint : endpoints) received_all = received_all && endpoint.seen == 8;
        if (!received_all) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    xrt_fs_stream_stop(source);
    xrt_fs_stream_stop(source);
    xrt_frame_context_destroy_nodes(&graph.ctx);
    bool valid = received_all;
    for (auto &endpoint : endpoints) {
        valid = valid && endpoint.errors == 0 && endpoint.retained &&
                endpoint.retained->data[4] == endpoint.expected &&
                endpoint.first_timestamp == endpoints[0].first_timestamp;
        xrt_frame_reference(&endpoint.retained, nullptr);
    }
    return valid ? 0 : 17;
}
