#pragma once
#include "stm32f4xx_hal.h"

// RTOS队列按值保存报文，禁止把可变缓冲区指针作为报文快照。
typedef struct {
    CAN_HandleTypeDef *hcan;
    uint32_t id;
    uint32_t queued_ms;
    uint32_t generation;
    uint8_t len;
    uint8_t data[8];
} can_tx_frame;

#ifdef __cplusplus
extern "C" {
#endif
extern volatile uint32_t can_tx_error_count[2];
// bit0=CAN1,bit1=CAN2，只取消指定总线的旧帧/邮箱；共享同总线时须统一所有权。
void can_cancel_pending(uint8_t bus_mask);
#ifdef __cplusplus
}
#endif
