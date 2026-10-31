// New mock Capture and validation harness. Upstream Monado files are unchanged.
#include "mock_capture.h"
#include "xrt/xrt_tracking.h"
#include "util/u_sink.h"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>
#include <time.h>
#include <zlib.h>

namespace {
thread_local std::string last_error;
uint64_t now_ns()
{
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ULL + uint64_t(ts.tv_nsec);
}
struct Counters {
    std::atomic<uint64_t> generated{0}, pushed{0}, received{0}, processed{0}, rejected{0};
    std::atomic<uint64_t> live{0}, peak{0}, active{0}, held_count{0};
    std::atomic<bool> done{false}, interrupted{false};
    std::atomic<uint64_t> started{0}, ended{0}, error{0};
    std::mutex mutex;
    std::condition_variable condition;
};
struct OwnedFrame {
    xrt_frame frame{};
    std::shared_ptr<Counters> counters;
    std::vector<uint8_t> pixels;
    uint64_t ready_ns = 0, simulated_ns = 0;
    uint32_t crc = 0, camera_id = 0, session_id = 0;
};
void destroy_frame(xrt_frame *frame)
{
    auto *owned = static_cast<OwnedFrame *>(frame->owner);
    auto counters = owned->counters;
    delete owned;
    counters->live--;
    counters->condition.notify_all();
}
bool valid_config(const wrl_config &c)
{
    return c.width >= 64 && c.width <= 4096 && c.height >= 64 && c.height <= 4096 &&
           c.padding <= 256 && c.fps_num > 0 && c.fps_num <= 1000 &&
           c.fps_den > 0 && c.fps_den <= 1000 && c.frames > 0 && c.frames <= 100000 &&
           c.camera_id < 4 && c.queue_capacity > 0 && c.queue_capacity <= 1024 &&
           c.malformed_kind <= 3 && c.hold_first <= 1 && c.session_id > 0 &&
           uint64_t(c.width + c.padding) * c.height <= 64 * 1024 * 1024;
}
class Capture;
struct SourceBase { xrt_fs fs{}; Capture *owner = nullptr; };
struct SourceNode { xrt_frame_node node{}; Capture *owner = nullptr; };

class Capture {
public:
    SourceBase base;
    SourceNode node;
    wrl_config config;
    xrt_format output_format;
    std::shared_ptr<Counters> counts;
    xrt_frame_sink *sink = nullptr;
    std::thread worker;
    std::atomic<bool> running{false}, stopping{false};
    bool started_once = false;
    uint64_t scheduled_start = 0;
    std::mutex sleep_mutex;
    std::condition_variable sleep_condition;

    static Capture *get(xrt_fs *fs) { return reinterpret_cast<SourceBase *>(fs)->owner; }
    static Capture *get(xrt_frame_node *n) { return reinterpret_cast<SourceNode *>(n)->owner; }
    Capture(const wrl_config &c, std::shared_ptr<Counters> counters, xrt_format format = XRT_FORMAT_R8)
        : config(c), output_format(format), counts(counters)
    {
        base.owner = this;
        node.owner = this;
        base.fs.source_id = 0x57524C00ULL + c.camera_id;
        std::snprintf(base.fs.name, sizeof(base.fs.name), "WRL mock capture camera %u", c.camera_id);
        base.fs.enumerate_modes = enumerate;
        // Synthetic controls are best-effort no-ops, not physical exposure controls.
        base.fs.configure_capture = [](xrt_fs *, xrt_fs_capture_parameters *) { return XRT_SUCCESS; };
        base.fs.stream_start = [](xrt_fs *fs, xrt_frame_sink *out, xrt_fs_capture_type type, uint32_t mode) {
            auto *self = get(fs);
            if (!out || mode != 0 || self->started_once ||
                (type != XRT_FS_CAPTURE_TYPE_TRACKING && type != XRT_FS_CAPTURE_TYPE_CALIBRATION)) return XRT_ERROR_INVALID_ARGUMENT;
            self->sink = out;
            self->started_once = true;
            self->running = true;
            self->counts->started = now_ns();
            if (self->scheduled_start == 0) self->scheduled_start = self->counts->started.load();
            try {
                self->worker = std::thread([self] { self->produce(); });
            } catch (...) {
                self->running = false;
                self->counts->error = 2;
                self->counts->done = true;
                return XRT_ERROR_INVALID_ARGUMENT;
            }
            return XRT_SUCCESS;
        };
        base.fs.slam_stream_start = [](xrt_fs *, xrt_slam_sinks *) { return XRT_ERROR_NOT_IMPLEMENTED; };
        base.fs.stream_stop = [](xrt_fs *fs) { get(fs)->stop(); return XRT_SUCCESS; };
        base.fs.is_running = [](xrt_fs *fs, bool *out) {
            if (!out) return XRT_ERROR_INVALID_ARGUMENT;
            *out = get(fs)->running.load();
            return XRT_SUCCESS;
        };
        node.node.break_apart = [](xrt_frame_node *n) { get(n)->stop(); };
        node.node.destroy = [](xrt_frame_node *n) { delete get(n); };
    }
    ~Capture() { stop(); }
    static xrt_result_t enumerate(xrt_fs *fs, xrt_fs_mode **modes, uint32_t *count)
    {
        if (!modes || !count) return XRT_ERROR_INVALID_ARGUMENT;
        *modes = static_cast<xrt_fs_mode *>(std::calloc(1, sizeof(xrt_fs_mode)));
        if (!*modes) return XRT_ERROR_INVALID_ARGUMENT;
        auto *self = get(fs);
        **modes = {self->config.width, self->config.height, self->output_format, XRT_STEREO_FORMAT_NONE};
        *count = 1;
        return XRT_SUCCESS;
    }
    void stop()
    {
        stopping = true;
        sleep_condition.notify_all();
        if (worker.joinable()) worker.join();
        running = false;
    }
    bool wait_for_deadline(uint64_t deadline)
    {
        std::unique_lock<std::mutex> lock(sleep_mutex);
        while (!stopping) {
            uint64_t current = now_ns();
            if (current >= deadline) break;
            sleep_condition.wait_for(lock, std::chrono::nanoseconds(deadline - current),
                                     [this] { return stopping.load(); });
        }
        return !stopping;
    }
    xrt_frame *make_frame(uint32_t sequence)
    {
        auto owned = std::make_unique<OwnedFrame>();
        owned->counters = counts;
        owned->camera_id = config.camera_id;
        owned->session_id = config.session_id;
        uint32_t stride = config.width + config.padding;
        owned->pixels.assign(size_t(stride) * config.height, 0xA5);
        uint32_t left = 32 + ((sequence * 3 + config.camera_id * 7 + config.seed) %
                             std::max(1U, config.width - 48));
        for (uint32_t y = 0; y < config.height; y++) {
            for (uint32_t x = 0; x < config.width; x++) {
                uint8_t value = ((x / 16 + y / 16) % 2 ? 200 : 40) + config.camera_id * 5;
                if (x >= left && x < left + 12 && y >= config.height / 2 &&
                    y < config.height / 2 + 12) value = 250;
                owned->pixels[size_t(y) * stride + x] = value;
            }
        }
        // Independent receiver decodes this payload identity, not just frame metadata.
        uint8_t header[16] = {0x57, 0x52, 0x4c, 1, uint8_t(config.camera_id),
            uint8_t(sequence), uint8_t(sequence >> 8), uint8_t(sequence >> 16),
            uint8_t(sequence >> 24), uint8_t(config.seed), uint8_t(config.seed >> 8),
            uint8_t(config.session_id), 0, 0, 0, 0};
        std::memcpy(owned->pixels.data(), header, sizeof(header));
        owned->simulated_ns = uint64_t(sequence) * 1000000000ULL * config.fps_den / config.fps_num;
        owned->crc = uint32_t(crc32(0, owned->pixels.data(), uInt(owned->pixels.size())));
        xrt_frame &f = owned->frame;
        f.reference.count = 1;
        f.destroy = destroy_frame;
        f.owner = owned.get();
        f.width = config.width; f.height = config.height;
        f.stride = stride; f.size = owned->pixels.size(); f.data = owned->pixels.data();
        f.format = output_format; f.stereo_format = XRT_STEREO_FORMAT_NONE;
        f.timestamp = int64_t(scheduled_start + owned->simulated_ns);
        f.source_timestamp = int64_t(owned->simulated_ns);
        f.source_sequence = sequence; f.source_id = base.fs.source_id;
        if (sequence == config.malformed_sequence) {
            if (config.malformed_kind == 1) f.format = XRT_FORMAT_R8G8B8;
            if (config.malformed_kind == 2) f.size--;
            if (config.malformed_kind == 3) f.stride = config.width - 1;
        }
        owned->ready_ns = now_ns();
        uint64_t live = ++counts->live, old = counts->peak;
        while (old < live && !counts->peak.compare_exchange_weak(old, live)) {}
        return &owned.release()->frame;
    }
    void produce()
    {
        try {
            for (uint32_t seq = 0; seq < config.frames; seq++) {
                if (seq >= config.interrupt_after) { counts->interrupted = true; break; }
                uint64_t deadline = scheduled_start + uint64_t(seq) * 1000000000ULL *
                                    config.fps_den / config.fps_num;
                if (!wait_for_deadline(deadline)) { counts->interrupted = true; break; }
                xrt_frame *frame = make_frame(seq);
                counts->generated++;
                xrt_sink_push_frame(sink, frame);
                counts->pushed++;
                xrt_frame_reference(&frame, nullptr);
            }
        } catch (...) { counts->error = 1; }
        counts->ended = now_ns();
        running = false;
        counts->done = true;
        counts->condition.notify_all();
    }
};

// Image-only adapter for Monado's existing multi-camera sink bundle. This does
// not construct a tracker or provide IMU/pose/mask samples.
class CaptureGroup;
struct GroupBase { xrt_fs fs{}; CaptureGroup *owner = nullptr; };
struct GroupNode { xrt_frame_node node{}; CaptureGroup *owner = nullptr; };
class CaptureGroup {
public:
    GroupBase base;
    GroupNode node;
    std::vector<Capture *> cameras;
    uint64_t epoch = 0;
    bool started_once = false;
    explicit CaptureGroup(std::vector<Capture *> sources) : cameras(std::move(sources))
    {
        base.owner = this;
        node.owner = this;
        base.fs.source_id = 0x57524C47ULL;
        for (auto *camera : cameras) camera->base.fs.source_id = base.fs.source_id;
        std::snprintf(base.fs.name, sizeof(base.fs.name), "WRL mock image-only camera group");
        base.fs.enumerate_modes = [](xrt_fs *, xrt_fs_mode **modes, uint32_t *count) {
            if (!modes || !count) return XRT_ERROR_INVALID_ARGUMENT;
            *modes = nullptr; *count = 0;
            return XRT_ERROR_NOT_IMPLEMENTED; // Per-camera layouts come from the explicit configs.
        };
        base.fs.configure_capture = [](xrt_fs *fs, xrt_fs_capture_parameters *params) {
            if (!params) return XRT_ERROR_INVALID_ARGUMENT;
            for (auto *camera : get(fs)->cameras) {
                auto result = xrt_fs_configure_capture(&camera->base.fs, params);
                if (result != XRT_SUCCESS) return result;
            }
            return XRT_SUCCESS;
        };
        base.fs.stream_start = [](xrt_fs *fs, xrt_frame_sink *sink, xrt_fs_capture_type type, uint32_t mode) {
            if (get(fs)->cameras.size() != 1) return XRT_ERROR_NOT_IMPLEMENTED;
            if (type != XRT_FS_CAPTURE_TYPE_TRACKING || mode != 0) return XRT_ERROR_INVALID_ARGUMENT;
            xrt_slam_sinks bundle{};
            bundle.cam_count = 1; bundle.cams[0] = sink;
            return get(fs)->start(&bundle);
        };
        base.fs.slam_stream_start = [](xrt_fs *fs, xrt_slam_sinks *bundle) {
            return get(fs)->start(bundle);
        };
        base.fs.stream_stop = [](xrt_fs *fs) { get(fs)->stop(); return XRT_SUCCESS; };
        base.fs.is_running = [](xrt_fs *fs, bool *out) {
            if (!out) return XRT_ERROR_INVALID_ARGUMENT;
            *out = false;
            for (auto *camera : get(fs)->cameras) *out = *out || camera->running.load();
            return XRT_SUCCESS;
        };
        node.node.break_apart = [](xrt_frame_node *n) {
            reinterpret_cast<GroupNode *>(n)->owner->stop();
        };
        // Children have separate frame-context nodes; never delete them twice.
        node.node.destroy = [](xrt_frame_node *n) {
            delete reinterpret_cast<GroupNode *>(n)->owner;
        };
    }
    static CaptureGroup *get(xrt_fs *fs) { return reinterpret_cast<GroupBase *>(fs)->owner; }
    xrt_result_t start(xrt_slam_sinks *bundle)
    {
        if (!bundle || started_once || bundle->cam_count != int(cameras.size()) ||
            bundle->imu_count < 0) return XRT_ERROR_INVALID_ARGUMENT;
        if (bundle->imu_count != 0 || bundle->gt) return XRT_ERROR_NOT_IMPLEMENTED;
        for (auto *imu : bundle->imus) if (imu) return XRT_ERROR_NOT_IMPLEMENTED;
        for (auto *mask : bundle->masks) if (mask) return XRT_ERROR_NOT_IMPLEMENTED;
        // This prototype deliberately requires every configured camera route.
        // Reject duplicate sinks so camera routing cannot silently collapse.
        for (size_t i = 0; i < cameras.size(); i++) {
            if (!bundle->cams[i] || !bundle->cams[i]->push_frame || cameras[i]->started_once)
                return XRT_ERROR_INVALID_ARGUMENT;
            for (size_t j = 0; j < i; j++)
                if (bundle->cams[i] == bundle->cams[j]) return XRT_ERROR_INVALID_ARGUMENT;
        }
        for (size_t i = cameras.size(); i < XRT_TRACKING_MAX_CAMS; i++)
            if (bundle->cams[i]) return XRT_ERROR_INVALID_ARGUMENT;
        started_once = true;
        const uint64_t shared_start = epoch ? epoch : now_ns() + 10000000ULL;
        for (size_t i = 0; i < cameras.size(); i++) {
            cameras[i]->scheduled_start = shared_start;
            auto result = xrt_fs_stream_start(&cameras[i]->base.fs, bundle->cams[i],
                                              XRT_FS_CAPTURE_TYPE_TRACKING, 0);
            if (result != XRT_SUCCESS) { stop(); return result; }
        }
        return XRT_SUCCESS;
    }
    void stop() { for (auto *camera : cameras) camera->stop(); }
};

struct Harness;
struct Receiver {
    xrt_frame_sink sink{};
    xrt_frame_node node{};
    Harness *owner = nullptr;
};
struct Harness {
    xrt_frame_context context{};
    Capture *capture = nullptr;
    std::shared_ptr<Counters> counts = std::make_shared<Counters>();
    Receiver receiver;
    xrt_frame_sink *queue = nullptr;
    xrt_frame *held = nullptr;
    uint32_t held_crc = 0;
    wrl_receive_callback callback;
    void *userdata;
    bool started = false, finished = false, retained_valid = true;
    uint32_t hold_first;
    wrl_config config;

    Harness(const wrl_config &cfg, wrl_receive_callback cb, void *ud)
        : callback(cb), userdata(ud), hold_first(cfg.hold_first), config(cfg)
    {
        receiver.owner = this;
        receiver.sink.push_frame = receive;
        receiver.node.break_apart = [](xrt_frame_node *) {};
        receiver.node.destroy = [](xrt_frame_node *) {};
        xrt_frame_context_add(&context, &receiver.node);
        if (!u_sink_queue_create(&context, cfg.queue_capacity, &receiver.sink, &queue))
            throw std::runtime_error("Monado queue creation failed");
        try {
            capture = new Capture(cfg, counts);
            xrt_frame_context_add(&context, &capture->node.node);
        } catch (...) {
            xrt_frame_context_destroy_nodes(&context);
            throw;
        }
    }
    static void receive(xrt_frame_sink *sink, xrt_frame *frame)
    {
        auto *h = reinterpret_cast<Receiver *>(sink)->owner;
        h->counts->active++;
        h->counts->received++;
        auto *owned = static_cast<OwnedFrame *>(frame->owner);
        if (h->hold_first && !h->held) {
            xrt_frame_reference(&h->held, frame);
            h->held_crc = owned->crc;
            h->counts->held_count = 1;
        }
        wrl_frame_view v{};
        // Route identity comes from the receiving endpoint, not the producer.
        v.camera_id = h->config.camera_id; v.session_id = h->config.session_id;
        v.width = frame->width; v.height = frame->height;
        v.stride = uint32_t(frame->stride); v.format = uint32_t(frame->format);
        v.payload_crc = owned->crc; v.sequence = frame->source_sequence;
        v.source_id = frame->source_id; v.simulated_capture_ns = owned->simulated_ns;
        v.published_timestamp_ns = uint64_t(frame->timestamp);
        v.ready_ns = owned->ready_ns; v.arrival_ns = now_ns();
        v.payload_size = frame->size; v.data = frame->data;
        int result = h->callback(h->userdata, &v);
        if (result == 0) h->counts->processed++; else h->counts->rejected++;
        h->counts->active--;
        h->counts->condition.notify_all();
    }
    bool finish(uint32_t timeout_ms)
    {
        if (finished) return true;
        if (!started) {
            xrt_frame_context_destroy_nodes(&context);
            capture = nullptr;
            finished = true;
            return true;
        }
        std::unique_lock<std::mutex> lock(counts->mutex);
        auto drained_now = [this] {
            return counts->done.load() && counts->active.load() == 0 &&
                   counts->live.load() == counts->held_count.load();
        };
        // Atomics are updated by producers/queue callbacks without this mutex.
        // A short bounded wait avoids a lost final notification stalling cleanup.
        uint64_t deadline = now_ns() + uint64_t(timeout_ms) * 1000000;
        while (!drained_now() && now_ns() < deadline)
            counts->condition.wait_for(lock, std::chrono::milliseconds(2));
        bool drained = drained_now();
        lock.unlock();
        // Stop joins producers first. Native graph teardown then joins the queue.
        capture->stop();
        xrt_frame_context_destroy_nodes(&context);
        capture = nullptr;
        if (held) {
            auto *owned = static_cast<OwnedFrame *>(held->owner);
            retained_valid = uint32_t(crc32(0, owned->pixels.data(), uInt(owned->pixels.size()))) == held_crc;
            xrt_frame_reference(&held, nullptr);
            counts->held_count = 0;
        }
        finished = true;
        return drained;
    }
    ~Harness() { if (!finished) finish(2000); }
};
} // namespace

extern "C" {
xrt_fs *wrl_mock_capture_create(xrt_frame_context *ctx, const wrl_config *cfg)
{
    return wrl_mock_capture_create_format(ctx, cfg, XRT_FORMAT_R8);
}
xrt_fs *wrl_mock_capture_create_format(xrt_frame_context *ctx, const wrl_config *cfg, uint32_t format)
{
    if (!ctx || !cfg || !valid_config(*cfg) ||
        (format != XRT_FORMAT_R8 && format != XRT_FORMAT_L8)) return nullptr;
    try {
        auto *capture = new Capture(*cfg, std::make_shared<Counters>(), xrt_format(format));
        xrt_frame_context_add(ctx, &capture->node.node);
        return &capture->base.fs;
    } catch (...) { return nullptr; }
}

xrt_fs *wrl_mock_capture_group_create(xrt_frame_context *ctx, const wrl_config *configs, uint32_t count)
{
    return wrl_mock_capture_group_create_format(ctx, configs, count, XRT_FORMAT_R8);
}
xrt_fs *wrl_mock_capture_group_create_format(xrt_frame_context *ctx, const wrl_config *configs, uint32_t count, uint32_t format)
{
    if (!ctx || !configs || count == 0 || count > 4 ||
        (format != XRT_FORMAT_R8 && format != XRT_FORMAT_L8)) return nullptr;
    for (uint32_t i = 0; i < count; i++) {
        if (!valid_config(configs[i])) return nullptr;
        for (uint32_t j = 0; j < i; j++)
            if (configs[i].camera_id == configs[j].camera_id) return nullptr;
    }
    try {
        // Allocate everything before registering nodes in the caller's graph.
        std::vector<std::unique_ptr<Capture>> owned;
        std::vector<Capture *> cameras;
        for (uint32_t i = 0; i < count; i++) {
            owned.push_back(std::make_unique<Capture>(configs[i], std::make_shared<Counters>(), xrt_format(format)));
            cameras.push_back(owned.back().get());
        }
        auto group = std::make_unique<CaptureGroup>(cameras);
        for (auto &camera : owned) {
            xrt_frame_context_add(ctx, &camera->node.node);
            camera.release();
        }
        xrt_frame_context_add(ctx, &group->node.node);
        return &group.release()->base.fs;
    } catch (...) { return nullptr; }
}
int wrl_mock_capture_group_schedule(xrt_fs *fs, uint64_t epoch)
{
    if (!fs || !epoch) return -1;
    auto *group = CaptureGroup::get(fs);
    if (group->started_once) return -1;
    group->epoch = epoch;
    return 0;
}
int wrl_mock_capture_group_stats(xrt_fs *fs, uint32_t index, wrl_stats *out)
{
    if (!fs || !out) return -1;
    auto *group = CaptureGroup::get(fs);
    if (index >= group->cameras.size()) return -1;
    auto &c = *group->cameras[index]->counts;
    *out = {c.generated.load(), c.pushed.load(), 0, 0, 0, c.live.load(), c.peak.load(),
            0, uint64_t(c.done.load()), uint64_t(c.interrupted.load()),
            c.started.load(), c.ended.load(), c.error.load()};
    return 0;
}
int wrl_harness_group_start(void **handles, uint32_t count, uint64_t start)
{
    if (!handles || count == 0 || count > 4) return -1;
    std::vector<Capture *> cameras;
    xrt_slam_sinks bundle{};
    bundle.cam_count = int(count);
    for (uint32_t i = 0; i < count; i++) {
        auto *h = static_cast<Harness *>(handles[i]);
        if (!h || h->started || h->finished || !h->capture) return -1;
        for (auto *camera : cameras)
            if (camera->config.camera_id == h->capture->config.camera_id) return -1;
        cameras.push_back(h->capture);
        bundle.cams[i] = h->queue;
    }
    CaptureGroup group(cameras);
    group.epoch = start;
    auto result = xrt_fs_slam_stream_start(&group.base.fs, &bundle);
    for (uint32_t i = 0; i < count; i++)
        static_cast<Harness *>(handles[i])->started = cameras[i]->started_once;
    return result == XRT_SUCCESS ? 0 : -1;
}

void *wrl_harness_create(const wrl_config *cfg, wrl_receive_callback cb, void *ud)
{
    last_error.clear();
    if (!cfg || !cb || !valid_config(*cfg)) { last_error = "Invalid mock configuration"; return nullptr; }
    try { return new Harness(*cfg, cb, ud); }
    catch (const std::exception &e) { last_error = e.what(); return nullptr; }
}
int wrl_harness_start(void *ptr, uint64_t start)
{
    auto *h = static_cast<Harness *>(ptr);
    if (!h || h->started || h->finished) return -1;
    h->capture->scheduled_start = start;
    xrt_fs_capture_parameters controls{};
    xrt_fs_configure_capture(&h->capture->base.fs, &controls);
    auto result = xrt_fs_stream_start(&h->capture->base.fs, h->queue, XRT_FS_CAPTURE_TYPE_TRACKING, 0);
    h->started = result == XRT_SUCCESS;
    return h->started ? 0 : -1;
}
int wrl_harness_finish(void *ptr, uint32_t timeout)
{
    if (!ptr) return -1;
    return static_cast<Harness *>(ptr)->finish(timeout) ? 0 : 1;
}
void wrl_harness_stop(void *ptr)
{
    auto *h = static_cast<Harness *>(ptr);
    if (h && h->capture) h->capture->stop();
}
void wrl_harness_stats(void *ptr, wrl_stats *out)
{
    auto *h = static_cast<Harness *>(ptr);
    if (!h || !out) return;
    auto &c = *h->counts;
    *out = {c.generated.load(), c.pushed.load(), c.received.load(), c.processed.load(),
            c.rejected.load(), c.live.load(), c.peak.load(), uint64_t(h->retained_valid),
            uint64_t(c.done.load()), uint64_t(c.interrupted.load()), c.started.load(),
            c.ended.load(), c.error.load()};
}
void wrl_harness_close(void *ptr) { delete static_cast<Harness *>(ptr); }
const char *wrl_last_error() { return last_error.c_str(); }
uint32_t wrl_stats_size() { return sizeof(wrl_stats); }
uint32_t wrl_config_size() { return sizeof(wrl_config); }
uint32_t wrl_frame_view_size() { return sizeof(wrl_frame_view); }
uint32_t wrl_format_r8() { return XRT_FORMAT_R8; }
}
