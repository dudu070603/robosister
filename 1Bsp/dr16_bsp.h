#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { DR16_FRAME_SIZE = 18, DR16_SWITCH_UP = 1, DR16_SWITCH_DOWN = 2, DR16_SWITCH_MID = 3 };

typedef struct {
    int16_t channel[5];  // 原码减1024，四摇杆轴通常在[-660,660]
    uint8_t switches[2]; // 物理左右归属由OperatorConfig配置，实机验证
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_left, mouse_right;
    uint16_t keyboard;
} DR16Frame;
// 无CRC的DBUS报文只接受完整18字节及合理的摇杆/拨杆值，失败不修改out。
int DR16_Decode(const uint8_t *bytes, uint16_t length, DR16Frame *out);

typedef struct {
    DR16Frame frame;
    uint32_t received_ms, good_frames, rejected_frames, uart_errors, restart_errors;
    uint8_t valid, consecutive_frames;
    // Read-only reception diagnostics: distinguish task start, UART/DMA bytes,
    // rejected framing and a stalled receiver without enabling any motor.
    uint32_t start_calls, service_ms, rx_events, rx_bytes, last_event_ms, last_size;
    uint32_t last_uart_error, last_start_result, uart_rx_state, dma_remaining;
    uint8_t rx_running;
} DR16Status;
// 原接收快照的稳定观察符号，外部只读；任务间用DR16_GetStatus复制。
extern volatile DR16Status g_dr16_status;
void DR16_Start(void);
void DR16_Service(void);
int DR16_GetStatus(DR16Status *status);
#ifdef __cplusplus
}
#endif
