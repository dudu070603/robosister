#include "dr16_bsp.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

// DBUS解包与校验；只处理报文，模式和摇杆映射由ArmOperator负责。
static uint16_t le16(const uint8_t *p) {
    return (uint16_t)(p[0] | (uint16_t)p[1] << 8);
}
int DR16_Decode(const uint8_t *b, uint16_t n, DR16Frame *out) {
    if (!b || !out || n != DR16_FRAME_SIZE)
        return 0;
    DR16Frame f = {0};
    f.channel[0] = (int16_t)((b[0] | (uint16_t)b[1] << 8) & 0x7ff) - 1024;
    f.channel[1] = (int16_t)((b[1] >> 3 | (uint16_t)b[2] << 5) & 0x7ff) - 1024;
    f.channel[2] =
        (int16_t)((b[2] >> 6 | (uint16_t)b[3] << 2 | (uint16_t)b[4] << 10) & 0x7ff) - 1024;
    f.channel[3] = (int16_t)((b[4] >> 1 | (uint16_t)b[5] << 7) & 0x7ff) - 1024;
    f.channel[4] = (int16_t)(le16(b + 16) & 0x7ff) - 1024;
    f.switches[0] = (b[5] >> 4) & 3;
    f.switches[1] = (b[5] >> 6) & 3;
    for (unsigned i = 0; i < 4; ++i)
        if (f.channel[i] < -700 || f.channel[i] > 700)
            return 0;
    for (unsigned i = 0; i < 2; ++i)
        if (!f.switches[i] || f.switches[i] > 3)
            return 0;
    if (b[12] > 1 || b[13] > 1)
        return 0;
    f.mouse_x = (int16_t)le16(b + 6);
    f.mouse_y = (int16_t)le16(b + 8);
    f.mouse_z = (int16_t)le16(b + 10);
    f.mouse_left = b[12];
    f.mouse_right = b[13];
    f.keyboard = le16(b + 14);
    *out = f;
    return 1;
}

// USART3 DMA接收、状态快照与异常恢复。
static uint8_t buffers[2][72];
static unsigned active_buffer;
static volatile uint8_t restart_needed;
volatile DR16Status g_dr16_status;
static void sample_receiver(void) {
    g_dr16_status.uart_rx_state = huart3.RxState;
    const int configured = huart3.Instance && huart3.hdmarx && huart3.hdmarx->Instance;
    g_dr16_status.dma_remaining = configured ? __HAL_DMA_GET_COUNTER(huart3.hdmarx) : 0;
    g_dr16_status.rx_running = configured && !restart_needed &&
        huart3.RxState == HAL_UART_STATE_BUSY_RX &&
        (huart3.Instance->CR3 & USART_CR3_DMAR) &&
        (huart3.hdmarx->Instance->CR & DMA_SxCR_EN);
}
static int begin_receive(void) {
    const HAL_StatusTypeDef result = huart3.Instance && huart3.hdmarx && huart3.hdmarx->Instance
        ? HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buffers[active_buffer], sizeof(buffers[0]))
        : HAL_ERROR;
    g_dr16_status.last_start_result = result;
    if (result != HAL_OK) {
        g_dr16_status.valid = 0;
        g_dr16_status.consecutive_frames = 0;
        ++g_dr16_status.restart_errors;
        restart_needed = 1;
        sample_receiver();
        return 0;
    }
    // 通过UART句柄取DMA，避免依赖CubeMX生成文件中DMA句柄的可见性。
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    restart_needed = 0;
    sample_receiver();
    return 1;
}
void DR16_Start(void) {
    ++g_dr16_status.start_calls;
    active_buffer = 0;
    begin_receive();
}
void DR16_Service(void) {
    taskENTER_CRITICAL();
    g_dr16_status.service_ms = HAL_GetTick();
    if (restart_needed) {
        HAL_UART_AbortReceive(&huart3);
        active_buffer ^= 1;
        begin_receive();
    }
    sample_receiver();
    taskEXIT_CRITICAL();
}
int DR16_GetStatus(DR16Status *s) {
    if (!s)
        return 0;
    taskENTER_CRITICAL();
    *s = g_dr16_status;
    taskEXIT_CRITICAL();
    return s->valid;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size) {
    if (h != &huart3)
        return;
    const uint32_t now = HAL_GetTick();
    ++g_dr16_status.rx_events;
    g_dr16_status.rx_bytes += size;
    g_dr16_status.last_event_ms = now;
    g_dr16_status.last_size = size;
    uint8_t *completed = buffers[active_buffer];
    active_buffer ^= 1;
    const int rearmed = begin_receive(); // 换缓冲后先重启DMA，再解析完整帧
    DR16Frame f;
    // IDLE can report several complete DBUS frames after a delayed interrupt.
    // Preserve frame boundaries: accept only aligned batches, validate every
    // frame, then publish the newest one. Never scan noise for a plausible frame.
    int decoded = size && size <= sizeof(buffers[0]) && size % DR16_FRAME_SIZE == 0;
    for (uint16_t offset = 0; decoded && offset < size; offset += DR16_FRAME_SIZE)
        decoded = DR16_Decode(completed + offset, DR16_FRAME_SIZE, &f);
    if (!rearmed || !decoded) {
        g_dr16_status.valid = 0;
        g_dr16_status.consecutive_frames = 0;
        ++g_dr16_status.rejected_frames;
        return;
    }
    if (g_dr16_status.good_frames && (uint32_t)(now - g_dr16_status.received_ms) > 100)
        g_dr16_status.consecutive_frames = 0;
    g_dr16_status.frame = f;
    g_dr16_status.received_ms = now;
    const uint16_t count = size / DR16_FRAME_SIZE;
    g_dr16_status.good_frames += count;
    const uint16_t consecutive = g_dr16_status.consecutive_frames + count;
    g_dr16_status.consecutive_frames = consecutive > 255 ? 255 : (uint8_t)consecutive;
    g_dr16_status.valid = 1;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
    if (h != &huart3)
        return;
    g_dr16_status.valid = 0;
    g_dr16_status.consecutive_frames = 0;
    ++g_dr16_status.uart_errors;
    g_dr16_status.last_uart_error = h->ErrorCode;
    restart_needed = 1;
    sample_receiver();
}
