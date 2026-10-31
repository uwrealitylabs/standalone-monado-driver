// Diagnostic component harness: public frames only. No tracker is replaced.
#include "xrt/xrt_frame.h"
#include "util/u_frame.h"
#include "util/u_sink.h"
#include <atomic>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <memory>
#include <thread>
#include <vector>
#include <time.h>
#include "matrix.h"
extern "C" {
void matrix_log_reset();
uint64_t matrix_log_errors();
uint64_t matrix_log_warnings();
}
static uint64_t clock_ns() {
    timespec t{}; clock_gettime(CLOCK_MONOTONIC, &t);
    return uint64_t(t.tv_sec)*1000000000ULL+t.tv_nsec;
}
static uint64_t checksum(xrt_frame *f) {
    uint64_t hash=1469598103934665603ULL;
    for(size_t i=0;i<f->size;i++) hash=(hash^f->data[i])*1099511628211ULL;
    return hash;
}
struct Counts {
    std::atomic<uint64_t> sent{0},converted{0},received{0},processed{0},rejected{0};
    std::atomic<uint64_t> live{0},peak{0},active{0},held{0};
};
struct Lease {
    xrt_frame frame{};
    xrt_frame *original=nullptr;
    std::shared_ptr<Counts> counts;
    static void destroy(xrt_frame *frame) {
        auto *self=static_cast<Lease *>(frame->owner);
        auto counts=self->counts;
        xrt_frame_reference(&self->original,nullptr);
        delete self;
        counts->live--;
    }
};
struct Matrix;
struct Endpoint {
    xrt_frame_sink sink{};
    xrt_frame_node node{};
    Matrix *owner=nullptr;
    unsigned route=0;
    xrt_frame *held=nullptr;
    uint64_t held_checksum=0;
};
struct Gateway {
    xrt_frame_sink sink{};
    xrt_frame_node node{};
    xrt_frame_sink *queue=nullptr;
    std::shared_ptr<Counts> counts;
};
struct Matrix {
    xrt_frame_context context{};
    Endpoint receivers[4];
    Gateway gateways[4];
    xrt_frame_sink *entries[4]{};
    std::shared_ptr<Counts> counts=std::make_shared<Counts>();
    unsigned cameras;
    MatrixCallback callback;
    void *userdata;
    bool closed=false;
    bool retained_valid=true;
    Matrix(unsigned count,unsigned converter,unsigned capacity,bool genlock,MatrixCallback cb,void *ud)
        : cameras(count),callback(cb),userdata(ud) {
        matrix_log_reset();
        for(unsigned i=0;i<cameras;i++) {
            auto &r=receivers[i];r.owner=this;r.route=i;
            r.sink.push_frame=[](xrt_frame_sink *sink,xrt_frame *f) {
                auto *r=reinterpret_cast<Endpoint *>(sink);auto *m=r->owner;
                m->counts->active++;m->counts->received++;
                if(!r->held) {
                    xrt_frame_reference(&r->held,f);r->held_checksum=checksum(f);m->counts->held++;
                }
                MatrixView v{r->route,uint32_t(f->format),f->width,f->height,f->stride,f->size,
                    f->source_sequence,f->source_id,f->timestamp,f->source_timestamp,clock_ns(),f->data};
                if(m->callback(m->userdata,&v)==0)m->counts->processed++;else m->counts->rejected++;
                m->counts->active--;
            };
            r.node.break_apart=[](xrt_frame_node*){};r.node.destroy=[](xrt_frame_node*){};
            xrt_frame_context_add(&context,&r.node);
        }
        xrt_frame_sink *outputs[4]{};
        for(unsigned i=0;i<cameras;i++) outputs[i]=&receivers[i].sink;
        if(genlock) for(unsigned i=0;i<cameras;i+=2) {
            if(!u_sink_force_genlock_create(&context,outputs[i],outputs[i+1],&outputs[i],&outputs[i+1]))
                throw "Could not create native genlock";
        }
        for(unsigned i=0;i<cameras;i++) {
            auto &g=gateways[i];g.counts=counts;
            if(!u_sink_queue_create(&context,capacity,outputs[i],&g.queue)) throw "Could not create queue";
            g.sink.push_frame=[](xrt_frame_sink *sink,xrt_frame *f) {
                auto *g=reinterpret_cast<Gateway *>(sink);
                auto *w=new Lease();w->counts=g->counts;w->frame=*f;
                w->frame.reference.count=1;w->frame.owner=w;w->frame.destroy=Lease::destroy;
                xrt_frame_reference(&w->original,f);
                uint64_t live=++g->counts->live,old=g->counts->peak;
                while(old<live&&!g->counts->peak.compare_exchange_weak(old,live)){}
                g->counts->converted++;
                xrt_frame *frame=&w->frame;
                xrt_sink_push_frame(g->queue,frame);
                xrt_frame_reference(&frame,nullptr);
            };
            g.node.break_apart=[](xrt_frame_node*){};g.node.destroy=[](xrt_frame_node*){};
            xrt_frame_context_add(&context,&g.node);
            entries[i]=&g.sink;
            if(converter) u_sink_create_format_converter(&context,converter==1?XRT_FORMAT_L8:XRT_FORMAT_R8G8B8,
                                                          entries[i],&entries[i]);
        }
    }
    int finish(unsigned expected,unsigned timeout) {
        if(closed)return 0;
        uint64_t deadline=clock_ns()+uint64_t(timeout)*1000000;
        bool drained=false;
        do {
            drained=counts->converted==expected&&counts->active==0&&counts->live==counts->held;
            if(drained)break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        } while(clock_ns()<deadline);
        xrt_frame_context_destroy_nodes(&context);
        for(unsigned i=0;i<cameras;i++) if(receivers[i].held) {
            retained_valid=retained_valid&&checksum(receivers[i].held)==receivers[i].held_checksum;
            xrt_frame_reference(&receivers[i].held,nullptr);counts->held--;
        }
        closed=true;return drained?0:1;
    }
    ~Matrix(){if(!closed)finish(unsigned(counts->converted.load()),1000);}
};
extern "C" {
void *matrix_create(uint32_t cameras,uint32_t converter,uint32_t capacity,uint32_t genlock,
                    MatrixCallback callback,void *userdata) {
    if((cameras!=1&&cameras!=2&&cameras!=4)||converter>2||!capacity||capacity>1024||
       !callback||(genlock&&cameras==1))return nullptr;
    try{return new Matrix(cameras,converter,capacity,genlock!=0,callback,userdata);}catch(...){return nullptr;}
}
int matrix_accept_frame(void *ptr,uint32_t route,xrt_frame *frame) {
    auto *m=static_cast<Matrix *>(ptr);
    if(!m||m->closed||route>=m->cameras||!frame)return -1;
    m->counts->sent++;
    xrt_sink_push_frame(m->entries[route],frame);
    return 0;
}
int matrix_push(void *ptr,uint32_t route,uint32_t format,uint32_t width,uint32_t height,
                uint64_t stride,const uint8_t *data,uint64_t size,uint64_t sequence,
                int64_t timestamp,int64_t source_timestamp) {
    auto *m=static_cast<Matrix *>(ptr);
    if(!m||m->closed||route>=m->cameras||!data||width<16||height<16||width>4096||height>4096)return -1;
    if(format!=XRT_FORMAT_R8&&format!=XRT_FORMAT_L8&&format!=XRT_FORMAT_R8G8B8)return -1;
    uint64_t row=width*(format==XRT_FORMAT_R8G8B8?3:1);
    if(stride<row||stride>row+256||size!=stride*height)return -1;
    xrt_frame *frame=nullptr;
    u_frame_create_one_off(xrt_format(format),width,height,&frame);
    if(!frame||!frame->data){xrt_frame_reference(&frame,nullptr);return -1;}
    void *resized=std::realloc(frame->data,size);
    if(!resized){xrt_frame_reference(&frame,nullptr);return -1;}
    frame->data=static_cast<uint8_t*>(resized);frame->size=size;frame->stride=stride;
    std::memcpy(frame->data,data,size);
    frame->source_id=0x50495045ULL;frame->source_sequence=sequence;
    frame->timestamp=timestamp;frame->source_timestamp=source_timestamp;
    int status=matrix_accept_frame(ptr,route,frame);
    xrt_frame_reference(&frame,nullptr);return status;
}
int matrix_finish(void *ptr,uint32_t expected,uint32_t timeout) {
    return ptr?static_cast<Matrix*>(ptr)->finish(expected,timeout):-1;
}
void matrix_stats(void *ptr,MatrixStats *out) {
    auto *m=static_cast<Matrix*>(ptr);if(!m||!out)return;auto &c=*m->counts;
    *out={c.sent.load(),c.converted.load(),c.received.load(),c.processed.load(),c.rejected.load(),
          c.live.load(),c.peak.load(),uint64_t(m->retained_valid),matrix_log_errors(),matrix_log_warnings()};
}
void matrix_close(void *ptr){delete static_cast<Matrix*>(ptr);}
uint32_t matrix_format(uint32_t id){return id==0?XRT_FORMAT_R8:id==1?XRT_FORMAT_L8:XRT_FORMAT_R8G8B8;}
uint32_t matrix_view_size(){return sizeof(MatrixView);}
uint32_t matrix_stats_size(){return sizeof(MatrixStats);}
}
