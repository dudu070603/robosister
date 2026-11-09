#include "usart.h"
#include "main.h"
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
void MX_USART3_UART_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    huart3.Instance=USART3;
    huart3.Init.BaudRate=100000;
    // STM32F4字长含校验位：9B+EVEN对应线上8数据位+偶校验。
    huart3.Init.WordLength=UART_WORDLENGTH_9B;
    huart3.Init.StopBits=UART_STOPBITS_1;
    huart3.Init.Parity=UART_PARITY_EVEN;
    huart3.Init.Mode=UART_MODE_RX;
    huart3.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    huart3.Init.OverSampling=UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart3)!=HAL_OK) Error_Handler();
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn,6,0); HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_SetPriority(USART3_IRQn,6,0); HAL_NVIC_EnableIRQ(USART3_IRQn);
}
void HAL_UART_MspInit(UART_HandleTypeDef *h) {
    if(h->Instance!=USART3) return;
    __HAL_RCC_USART3_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio={0};
    gpio.Pin=GPIO_PIN_11; gpio.Mode=GPIO_MODE_AF_PP;
    gpio.Pull=GPIO_PULLUP; gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate=GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC,&gpio); // C板DBUS反相电路输出接PC11
    hdma_usart3_rx.Instance=DMA1_Stream1;
    hdma_usart3_rx.Init.Channel=DMA_CHANNEL_4;
    hdma_usart3_rx.Init.Direction=DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc=DMA_PINC_DISABLE; hdma_usart3_rx.Init.MemInc=DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode=DMA_NORMAL; hdma_usart3_rx.Init.Priority=DMA_PRIORITY_HIGH;
    hdma_usart3_rx.Init.FIFOMode=DMA_FIFOMODE_DISABLE;
    if(HAL_DMA_Init(&hdma_usart3_rx)!=HAL_OK) Error_Handler();
    __HAL_LINKDMA(h,hdmarx,hdma_usart3_rx);
}
