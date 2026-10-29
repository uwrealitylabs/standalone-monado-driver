// Task 1 baseline check: use Monado's real queue without changing it.
#include "xrt/xrt_frame.h"
#include "util/u_sink.h"
#include <atomic>
#include <chrono>
#include <thread>
static std::atomic<int> received{0};
static void receive_frame(xrt_frame_sink *, xrt_frame *frame)
{
    if (frame->width == 64 && frame->source_sequence == 7) received++;
}
static void do_not_delete_stack_frame(xrt_frame *) {}
int main()
{
    xrt_frame_context context{};
    xrt_frame_sink receiver{receive_frame};
    xrt_frame_sink *queue = nullptr;
    if (!u_sink_queue_create(&context, 4, &receiver, &queue)) return 1;
    xrt_frame frame{};
    frame.reference.count = 1;
    frame.destroy = do_not_delete_stack_frame;
    frame.width = 64;
    frame.source_sequence = 7;
    xrt_sink_push_frame(queue, &frame);
    for (int i = 0; i < 200 && received.load() == 0; i++)
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    xrt_frame_context_destroy_nodes(&context);
    return received.load() == 1 && frame.reference.count == 1 ? 0 : 2;
}
