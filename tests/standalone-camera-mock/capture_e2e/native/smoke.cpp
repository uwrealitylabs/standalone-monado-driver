#include "session.h"
#include <cstdlib>
#include <time.h>
static unsigned received=0,rejected=0;
static int source_event(void *,const MatrixView *,uint32_t error){if(error)rejected++;return 0;}
static int cv_event(void *,const MatrixView *frame) {
    if(frame->format!=e2e_format(2)||frame->data[12]!=frame->route)return 1;
    received++;return 0;
}
int main() {
    wrl_config configs[2];
    for(unsigned i=0;i<2;i++)configs[i]={64,64,16,120,1,8,i,1,32,8,0xffffffff,0,0,1};
    xrt_frame_context context{};
    if(wrl_mock_capture_create_format(&context,configs,XRT_FORMAT_R8G8B8))return 1;
    auto *single=wrl_mock_capture_create_format(&context,configs,XRT_FORMAT_L8);
    if(!single)return 2;
    xrt_fs_mode *modes=nullptr;uint32_t count=0;
    if(xrt_fs_enumerate_modes(single,&modes,&count)!=XRT_SUCCESS||count!=1||modes[0].format!=XRT_FORMAT_L8)return 3;
    std::free(modes);xrt_frame_context_destroy_nodes(&context);
    void *session=e2e_create(configs,2,2,1,cv_event,source_event,nullptr);
    if(!session)return 4;
    timespec t{};clock_gettime(CLOCK_MONOTONIC,&t);
    uint64_t epoch=uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec+10000000ULL;
    if(e2e_start(session,epoch)){e2e_close(session);return 5;}
    int finish=e2e_finish(session,3000);
    E2EStats src{};MatrixStats stats{};
    e2e_stats(session,&src);e2e_pipeline_stats(session,&stats);e2e_close(session);
    return !finish&&received==16&&!rejected&&stats.live==0&&stats.retained_valid&&
           src.sources[0].generated==8&&src.sources[1].producer_done?0:6;
}
