#include "dr16_bsp.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
static uint8_t buffers[2][72];
static unsigned active_buffer;
static volatile uint8_t restart_needed;
volatile DR16Status g_dr16_status;
static int begin_receive(void) {
    if(HAL_UARTEx_ReceiveToIdle_DMA(&huart3,buffers[active_buffer],sizeof(buffers[0]))!=HAL_OK) {
        g_dr16_status.valid=0; g_dr16_status.consecutive_frames=0; ++g_dr16_status.restart_errors;
        restart_needed=1; return 0;
    }
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx,DMA_IT_HT);
    restart_needed=0; return 1;
}
void DR16_Start(void) { active_buffer=0; begin_receive(); }
void DR16_Service(void) {
    if(!restart_needed) return;
    taskENTER_CRITICAL();
    HAL_UART_AbortReceive(&huart3); active_buffer^=1; begin_receive();
    taskEXIT_CRITICAL();
}
int DR16_GetStatus(DR16Status *s) {
    if(!s) return 0;
    taskENTER_CRITICAL(); *s=g_dr16_status; taskEXIT_CRITICAL(); return s->valid;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h,uint16_t size) {
    if(h!=&huart3) return;
    uint8_t *completed=buffers[active_buffer]; active_buffer^=1;
    const int rearmed=begin_receive(); // 换缓冲后先重启DMA，再解析完整帧
    DR16Frame f;
    if(!rearmed || !DR16_Decode(completed,size,&f)) {
        g_dr16_status.valid=0; g_dr16_status.consecutive_frames=0; ++g_dr16_status.rejected_frames; return;
    }
    const uint32_t now=HAL_GetTick();
    if(g_dr16_status.good_frames && (uint32_t)(now-g_dr16_status.received_ms)>100) g_dr16_status.consecutive_frames=0;
    g_dr16_status.frame=f; g_dr16_status.received_ms=now; ++g_dr16_status.good_frames;
    if(g_dr16_status.consecutive_frames<255) ++g_dr16_status.consecutive_frames;
    g_dr16_status.valid=1;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
    if(h!=&huart3) return;
    g_dr16_status.valid=0; g_dr16_status.consecutive_frames=0; ++g_dr16_status.uart_errors;
    restart_needed=1;
}
