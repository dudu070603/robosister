#pragma once
#include "dr16_protocol.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    DR16Frame frame;
    uint32_t received_ms,good_frames,rejected_frames,uart_errors,restart_errors;
    uint8_t valid,consecutive_frames;
} DR16Status;
// 原接收快照的稳定观察符号，外部只读；任务间用DR16_GetStatus复制。
extern volatile DR16Status g_dr16_status;
void DR16_Start(void);
void DR16_Service(void);
int DR16_GetStatus(DR16Status *status);
#ifdef __cplusplus
}
#endif
