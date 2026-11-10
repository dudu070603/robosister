#pragma once
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
void MX_USART3_UART_Init(void);
#ifdef __cplusplus
}
#endif
