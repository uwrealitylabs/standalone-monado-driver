// Minimal source-selection wiring. Only public native frames cross the boundary.
#include "session.h"
#include "xrt/xrt_tracking.h"
#include <atomic>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <time.h>
static uint64_t now_ns() {
    timespec ts{};clock_gettime(CLOCK_MONOTONIC,&ts);
    return uint64_t(ts.tv_sec)*1000000000ULL+ts.tv_nsec;
}
struct Session;
struct Guard {
    xrt_frame_sink sink{};
    xrt_frame_node node{};
    Session *owner=nullptr;
    wrl_config config{};
    unsigned route=0;
    std::atomic<uint64_t> seen{0},rejected{0};
};
struct Session {
    xrt_frame_context source_context{};
    xrt_fs *source=nullptr;
    void *pipeline=nullptr;
    Guard guards[4];
    unsigned count;
    E2ESourceCallback observe_source;
    void *userdata;
    E2EStats snapshot{};
    bool started=false,finished=false;
    int finish_result=0;
    Session(const wrl_config *configs,unsigned n,unsigned converter,bool pairing,
            MatrixCallback cb,E2ESourceCallback source_cb,void *ud)
        :count(n),observe_source(source_cb),userdata(ud) {
        try {
            pipeline=matrix_create(n,converter,configs[0].queue_capacity,pairing,cb,ud);
            if(!pipeline)throw std::runtime_error("Could not create processing graph");
            for(unsigned i=0;i<n;i++) {
                auto &g=guards[i];g.owner=this;g.route=i;g.config=configs[i];
                g.node.break_apart=[](xrt_frame_node*){};g.node.destroy=[](xrt_frame_node*){};
                g.sink.push_frame=[](xrt_frame_sink *sink,xrt_frame *f) {
                    auto *g=reinterpret_cast<Guard *>(sink);auto *s=g->owner;g->seen++;
                    unsigned error=0;
                    if(f->format!=XRT_FORMAT_L8)error=1;
                    else if(f->width!=g->config.width||f->height!=g->config.height)error=4;
                    else if(f->stride!=g->config.width+g->config.padding)error=2;
                    else if(!f->data||f->size!=f->stride*f->height)error=3;
                    MatrixView v{g->route,uint32_t(f->format),f->width,f->height,f->stride,f->size,
                        f->source_sequence,f->source_id,f->timestamp,f->source_timestamp,now_ns(),f->data};
                    int observer=s->observe_source(s->userdata,&v,error);
                    if(error||observer){g->rejected++;return;}
                    if(matrix_accept_frame(s->pipeline,g->route,f))g->rejected++;
                };
                xrt_frame_context_add(&source_context,&g.node);
            }
            source=wrl_mock_capture_group_create_format(&source_context,configs,n,XRT_FORMAT_L8);
            if(!source)throw std::runtime_error("Invalid Capture configuration");
        } catch(...) {
            xrt_frame_context_destroy_nodes(&source_context);
            if(pipeline)matrix_close(pipeline);
            throw;
        }
    }
    int start(uint64_t epoch) {
        if(started||finished||!epoch)return -1;
        if(wrl_mock_capture_group_schedule(source,epoch))return -1;
        xrt_fs_capture_parameters controls{};
        if(xrt_fs_configure_capture(source,&controls)!=XRT_SUCCESS)return -1;
        xrt_slam_sinks sinks{};sinks.cam_count=int(count);
        for(unsigned i=0;i<count;i++)sinks.cams[i]=&guards[i].sink;
        auto result=xrt_fs_slam_stream_start(source,&sinks);
        started=true; // Even partial start must be stopped and joined.
        return result==XRT_SUCCESS?0:-1;
    }
    void stop(){if(source)xrt_fs_stream_stop(source);}
    int finish(unsigned timeout) {
        if(finished)return finish_result;
        const uint64_t deadline=now_ns()+uint64_t(timeout)*1000000;
        bool running=false;
        if(started)do {
            xrt_fs_is_running(source,&running);
            if(!running)break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }while(now_ns()<deadline);
        snapshot.source_timed_out=running;
        stop();
        uint64_t accepted=0;
        for(unsigned i=0;i<count;i++) {
            wrl_mock_capture_group_stats(source,i,&snapshot.sources[i]);
            snapshot.guard_seen[i]=guards[i].seen;
            snapshot.guard_rejected[i]=guards[i].rejected;
            accepted+=snapshot.guard_seen[i]-snapshot.guard_rejected[i];
        }
        // Outstanding native image references must survive source destruction.
        xrt_frame_context_destroy_nodes(&source_context);source=nullptr;
        uint64_t current=now_ns();
        unsigned left=current<deadline?unsigned((deadline-current)/1000000)+1:1;
        int drained=matrix_finish(pipeline,unsigned(accepted),left);
        finished=true;snapshot.finished=1;
        finish_result=snapshot.source_timed_out||drained?1:0;
        return finish_result;
    }
    ~Session(){if(!finished)finish(2000);if(pipeline)matrix_close(pipeline);}
};
extern "C" {
void *e2e_create(const wrl_config *configs,uint32_t count,uint32_t converter,uint32_t pairing,
                 MatrixCallback cb,E2ESourceCallback source_cb,void *userdata) {
    if(!configs||(count!=1&&count!=2&&count!=4)||converter>2||pairing>1||
       (pairing&&count==1)||!cb||!source_cb)return nullptr;
    for(unsigned i=0;i<count;i++)
        if(configs[i].camera_id!=i||configs[i].queue_capacity!=configs[0].queue_capacity)return nullptr;
    try{return new Session(configs,count,converter,pairing!=0,cb,source_cb,userdata);}catch(...){return nullptr;}
}
int e2e_start(void *p,uint64_t epoch){return p?static_cast<Session*>(p)->start(epoch):-1;}
void e2e_stop(void *p){if(p)static_cast<Session*>(p)->stop();}
int e2e_finish(void *p,uint32_t timeout){return p?static_cast<Session*>(p)->finish(timeout):-1;}
void e2e_stats(void *p,E2EStats *out){if(p&&out)*out=static_cast<Session*>(p)->snapshot;}
void e2e_pipeline_stats(void *p,MatrixStats *out){if(p)matrix_stats(static_cast<Session*>(p)->pipeline,out);}
void e2e_close(void *p){delete static_cast<Session*>(p);}
uint32_t e2e_stats_size(){return sizeof(E2EStats);}
uint32_t e2e_format(uint32_t index){return matrix_format(index);}
}
