#include "matrix.h"
#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
static std::atomic<unsigned> seen{0},errors{0};
static int receive(void *,const MatrixView *v) {
    if(v->format!=matrix_format(2)||v->width!=64||v->height!=64||v->stride!=192||
       v->size!=12288||v->source_id!=0x50495045ULL||
       v->data[0]!=42+v->route||v->data[1]!=42+v->route||
       v->source_timestamp!=int64_t(v->sequence*33000000ULL+v->route*500000ULL)||
       v->timestamp!=int64_t(1000000000ULL+v->sequence*33000000ULL+250000ULL))errors++;
    seen++;return 0;
}
int main() {
    void *matrix=matrix_create(2,2,32,1,receive,nullptr);
    if(!matrix)return 1;
    for(unsigned seq=0;seq<4;seq++) {
        for(int cam=1;cam>=0;cam--) {
            std::vector<uint8_t> pixels(80*64,165);
            for(unsigned y=0;y<64;y++)for(unsigned x=0;x<64;x++)pixels[y*80+x]=uint8_t(42+cam);
            int64_t source=seq*33000000ULL+cam*500000ULL;
            if(matrix_push(matrix,cam,matrix_format(1),64,64,80,pixels.data(),pixels.size(),seq,
                           1000000000ULL+source,source)){matrix_close(matrix);return 2;}
        }
        for(unsigned attempt=0;attempt<2000&&seen<2*(seq+1);attempt++)
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    int finish=matrix_finish(matrix,8,3000);
    MatrixStats stats{};matrix_stats(matrix,&stats);matrix_close(matrix);
    return !finish&&seen==8&&errors==0&&stats.processed==8&&stats.live==0&&stats.retained_valid?0:3;
}
