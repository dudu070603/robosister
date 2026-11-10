#include "CAN_bsp.h"
#include <cstring>
extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if(!(x)) return __LINE__; } while(0)
extern "C" void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan);
namespace {
can_tx_frame saved{}, dequeued{};
osMessageQueueId_t saved_queue;
unsigned reset_count[2]{}, abort_count[2]{}, released[2]{};
unsigned critical_depth=0, add_count=0, acquire_count=0;
bool mailbox_pending[2]{}, add_outside_critical=false, acquire_inside_critical=false;
bool service_running=false, queue_ready=false, cancel_on_tick=false, deferred_cancel=false;
bool release_inside_service_critical=false, abort_callback=false;
uint32_t tick_ms=1234;
osStatus_t acquire_result=osOK;
HAL_StatusTypeDef add_result=HAL_OK;
CAN_TxHeaderTypeDef added_header{};
uint8_t added_data[8]{};
// GCC nonlocal-jump fixture avoids pulling hosted unwind/syscall support into the RAM test image.
void *service_return[5];
void run_once(const can_tx_frame &frame,uint32_t now,osStatus_t acquire,HAL_StatusTypeDef add,bool inject_cancel=false) {
    dequeued=frame; queue_ready=true; service_running=true; tick_ms=now;
    acquire_result=acquire; add_result=add; cancel_on_tick=inject_cancel;
    if(__builtin_setjmp(service_return)==0) can1_sending_service(nullptr);
    service_running=false; cancel_on_tick=false; tick_ms=1234;
}
}
osMessageQueueId_t can1_send_fifoHandle = reinterpret_cast<void *>(1);
osMessageQueueId_t can2_send_fifoHandle = reinterpret_cast<void *>(2);
osSemaphoreId_t CAN1CountingSemHandle = reinterpret_cast<void *>(3);
osSemaphoreId_t CAN2CountingSemHandle = reinterpret_cast<void *>(4);
extern "C" {
CAN_HandleTypeDef hcan1{}, hcan2{};
uint32_t HAL_GetTick() {
    if(service_running && cancel_on_tick) {
        cancel_on_tick=false;
        // Simulate a higher priority task requesting cancellation exactly after the
        // sender's generation read, during its age check. Critical sections defer it.
        if(critical_depth) deferred_cancel=true;
        else can_cancel_pending(1);
    }
    return tick_ms;
}
osStatus_t osMessageQueuePut(osMessageQueueId_t queue, const void *data, uint8_t, uint32_t) {
    saved = *static_cast<const can_tx_frame *>(data); saved_queue = queue; return osOK;
}
osStatus_t osMessageQueueGet(osMessageQueueId_t,void *data,uint8_t *,uint32_t) {
    if(service_running && queue_ready) { *static_cast<can_tx_frame *>(data)=dequeued; queue_ready=false; return osOK; }
    // One real production service iteration, then stop at its next blocking get.
    if(service_running) __builtin_longjmp(service_return,1);
    return osErrorResource;
}
osStatus_t osMessageQueueReset(osMessageQueueId_t queue) {
    ++reset_count[queue == can1_send_fifoHandle ? 0 : 1]; return osOK;
}
osStatus_t osSemaphoreAcquire(osSemaphoreId_t,uint32_t) {
    ++acquire_count; if(critical_depth) acquire_inside_critical=true; return acquire_result;
}
osStatus_t osSemaphoreRelease(osSemaphoreId_t sem) {
    if(service_running && critical_depth && !abort_callback) release_inside_service_critical=true;
    ++released[sem == CAN1CountingSemHandle ? 0 : 1]; return osOK;
}
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h,const CAN_TxHeaderTypeDef *header,
                                     const uint8_t data[],uint32_t *mailbox) {
    ++add_count; if(!critical_depth) add_outside_critical=true;
    added_header=*header; std::memcpy(added_data,data,8); *mailbox=CAN_TX_MAILBOX0;
    if(add_result==HAL_OK) mailbox_pending[h==&hcan1 ? 0 : 1]=true;
    return add_result;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t) {
    const unsigned bus=h==&hcan1 ? 0 : 1; ++abort_count[bus];
    if(mailbox_pending[bus]) {
        mailbox_pending[bus]=false; abort_callback=true;
        HAL_CAN_TxMailbox0AbortCallback(h); abort_callback=false;
    }
    return HAL_OK;
}
uint32_t HAL_CAN_GetError(const CAN_HandleTypeDef *h) { return h->ErrorCode; }
HAL_StatusTypeDef HAL_CAN_ResetError(CAN_HandleTypeDef *h) { h->ErrorCode = 0; return HAL_OK; }
void vPortEnterCritical() { ++critical_depth; }
void vPortExitCritical() {
    if(critical_depth) --critical_depth;
    if(!critical_depth && deferred_cancel) { deferred_cancel=false; can_cancel_pending(1); }
}
}

int test_can_tx() {
    uint8_t d[8]={1,2,3,4,5,6,7,8};
    can_device_transmit tx(&hcan1,0x200,8,d);
    CHECK(tx.can_send_msg()==osOK && saved_queue==can1_send_fifoHandle);
    std::memset(d,0xff,8); // 源缓冲区变更不得污染已经入队的帧。
    CHECK(saved.data[0]==1 && saved.data[7]==8 && saved.queued_ms==1234 && saved.id==0x200);
    tx.member.hcan=&hcan2;
    CHECK(tx.can_send_msg()==osOK && saved_queue==can2_send_fifoHandle);
    tx.member.len=9; CHECK(tx.can_send_msg()==osErrorParameter);
    tx.member.len=8; tx.member.buf_data=nullptr; CHECK(tx.can_send_msg()==osErrorParameter);
    // CAN1机械臂停机不能清空/中止CAN2底盘；旧帧代号只在CAN1变化。
    can_cancel_pending(1);
    CHECK(reset_count[0]==1 && reset_count[1]==0 && abort_count[0]==1 && abort_count[1]==0);
    tx.member.buf_data=d;
    CHECK(tx.can_send_msg()==osOK && saved.generation==0); // 仍是CAN2。
    tx.member.hcan=&hcan1; CHECK(tx.can_send_msg()==osOK && saved.generation==1);
    // 两个发送邮箱出错时恰好归还两份令牌，且清除已处理错误位。
    hcan1.ErrorCode=HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR2;
    HAL_CAN_ErrorCallback(&hcan1);
    CHECK(released[0]==2 && released[1]==0 && !hcan1.ErrorCode && can_tx_error_count[0]==1);

    { // 正常帧4ms边界可发送；HAL提交必须在临界区，等信号量必须在外。
        CHECK(tx.can_send_msg()==osOK); const auto frame=saved;
        const unsigned tokens=released[0], adds=add_count, waits=acquire_count;
        run_once(frame,1238,osOK,HAL_OK);
        CHECK(add_count==adds+1 && acquire_count==waits+1 && mailbox_pending[0]);
        CHECK(released[0]==tokens && critical_depth==0);
        CHECK(added_header.StdId==0x200 && added_header.DLC==8 && added_header.IDE==CAN_ID_STD && added_header.RTR==CAN_RTR_DATA);
        CHECK(added_data[0]==0xff && added_data[7]==0xff);
        mailbox_pending[0]=false; HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
        CHECK(released[0]==tokens+1);
    }
    { // 已出队但随后取消的帧：代号不符，不得提交邮箱，令牌恰好归还。
        CHECK(tx.can_send_msg()==osOK); const auto old=saved; can_cancel_pending(1);
        const unsigned tokens=released[0], adds=add_count, errors=can_tx_error_count[0];
        run_once(old,1234,osOK,HAL_OK);
        CHECK(add_count==adds && released[0]==tokens+1 && !mailbox_pending[0]);
        CHECK(can_tx_error_count[0]==errors && critical_depth==0);
    }
    { // 超过4ms的帧不得发送；HAL拒绝发送也应归还邮箱令牌。
        for(unsigned scenario=0;scenario<2;++scenario) {
            CHECK(tx.can_send_msg()==osOK); const auto frame=saved;
            const unsigned tokens=released[0], adds=add_count, errors=can_tx_error_count[0];
            run_once(frame,scenario ? 1234 : 1239,osOK,scenario ? HAL_ERROR : HAL_OK);
            CHECK(add_count==adds+scenario && released[0]==tokens+1 && !mailbox_pending[0]);
            CHECK(can_tx_error_count[0]==errors+1 && critical_depth==0);
        }
    }
    { // 获取令牌超时：没有取得的令牌不能归还，也不能调用HAL发送。
        CHECK(tx.can_send_msg()==osOK); const auto frame=saved;
        const unsigned tokens=released[0], adds=add_count, errors=can_tx_error_count[0];
        run_once(frame,1234,osErrorTimeout,HAL_OK);
        CHECK(add_count==adds && released[0]==tokens && can_tx_error_count[0]==errors+1);
    }
    { // 在generation检查与HAL提交之间请求关控：发送与取消必须有原子顺序。
        CHECK(tx.can_send_msg()==osOK); const auto frame=saved;
        const unsigned aborts=abort_count[0], tokens=released[0], adds=add_count;
        run_once(frame,1234,osOK,HAL_OK,true);
        CHECK(add_count==adds+1 && abort_count[0]==aborts+1);
        CHECK(!mailbox_pending[0] && !deferred_cancel && released[0]==tokens+1);
        CHECK(critical_depth==0 && reset_count[1]==0 && abort_count[1]==0);
    }
    CHECK(!add_outside_critical && !acquire_inside_critical && !release_inside_service_critical);
    return 0;
}
