#include "dr16_bsp.h"
#include "usart.h"
#include <cstring>

extern "C" {
volatile unsigned test_checks = 0, test_failed_line = 0, test_done = 0;
UART_HandleTypeDef huart3{};
DMA_HandleTypeDef hdma_usart3_rx{};
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *, uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
}

#define CHECK(x) do { ++test_checks; if (!(x)) return __LINE__; } while (0)

namespace {
USART_TypeDef uart_registers{};
DMA_Stream_TypeDef dma_registers{};
uint8_t *receiving_buffer = nullptr;
uint16_t receiving_capacity = 0;
unsigned starts = 0, aborts = 0, critical_depth = 0;
uint32_t now_ms = 100;
HAL_StatusTypeDef next_start_result = HAL_OK;
bool start_was_in_critical = false;

void encode(uint8_t *bytes, int a = 0, int b = 0, int c = 0, int d = 0,
            unsigned left = DR16_SWITCH_UP, unsigned right = DR16_SWITCH_MID) {
    std::memset(bytes, 0, DR16_FRAME_SIZE);
    const unsigned values[] = {unsigned(a + 1024), unsigned(b + 1024),
                               unsigned(c + 1024), unsigned(d + 1024)};
    for (unsigned channel = 0; channel < 4; ++channel)
        for (unsigned bit = 0; bit < 11; ++bit)
            if (values[channel] & (1u << bit))
                bytes[(channel * 11 + bit) / 8] |= uint8_t(1u << ((channel * 11 + bit) % 8));
    bytes[5] |= uint8_t((left << 4) | (right << 6));
    bytes[16] = 0;
    bytes[17] = 4;
}

void complete(uint16_t size, int first_channel = 0, bool corrupt_first_frame = false) {
    // Model the HAL normal-mode completion before it calls the application.
    // HAL already aborts/stops DMA and restores READY before IDLE/TC callback.
    huart3.RxState = HAL_UART_STATE_READY;
    huart3.ReceptionType = HAL_UART_RECEPTION_STANDARD;
    dma_registers.CR &= ~DMA_SxCR_EN;
    dma_registers.NDTR = size <= receiving_capacity ? receiving_capacity - size : 0;
    std::memset(receiving_buffer, 0, receiving_capacity);
    for (unsigned offset = 0; offset + DR16_FRAME_SIZE <= receiving_capacity &&
                             (offset == 0 || offset + DR16_FRAME_SIZE <= size);
         offset += DR16_FRAME_SIZE)
        encode(receiving_buffer + offset, first_channel + offset / DR16_FRAME_SIZE, -321, 222, -111);
    if (corrupt_first_frame)
        receiving_buffer[5] &= 0x0f; // Both switch codes zero: invalid even if next frame is good.
    HAL_UARTEx_RxEventCallback(&huart3, size);
}

int run() {
    huart3.Instance = &uart_registers;
    huart3.hdmarx = &hdma_usart3_rx;
    huart3.RxState = HAL_UART_STATE_READY;
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    hdma_usart3_rx.Instance = &dma_registers;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.State = HAL_DMA_STATE_READY;
    CHECK(!DR16_GetStatus(nullptr));
    DR16_Start();
    CHECK(starts == 1 && receiving_buffer && receiving_capacity == 72);
    CHECK(g_dr16_status.start_calls == 1 && g_dr16_status.last_start_result == HAL_OK);
    CHECK(g_dr16_status.rx_running && !g_dr16_status.valid);
    CHECK(!(dma_registers.CR & DMA_IT_HT));
    uint8_t *first_buffer = receiving_buffer;
    const auto initial_events = g_dr16_status.rx_events;
    UART_HandleTypeDef other{};
    HAL_UARTEx_RxEventCallback(&other, 18);
    HAL_UART_ErrorCallback(&other);
    CHECK(starts == 1 && g_dr16_status.rx_events == initial_events && !g_dr16_status.uart_errors);

    complete(18, 123);
    CHECK(starts == 2 && receiving_buffer != first_buffer);
    CHECK(g_dr16_status.valid && g_dr16_status.good_frames == 1 &&
          g_dr16_status.consecutive_frames == 1);
    CHECK(g_dr16_status.frame.channel[0] == 123 && g_dr16_status.frame.channel[1] == -321 &&
          g_dr16_status.frame.channel[2] == 222 && g_dr16_status.frame.channel[3] == -111 &&
          g_dr16_status.frame.channel[4] == 0);
    CHECK(g_dr16_status.frame.switches[0] == DR16_SWITCH_UP &&
          g_dr16_status.frame.switches[1] == DR16_SWITCH_MID);
    CHECK(g_dr16_status.rx_events == 1 && g_dr16_status.rx_bytes == 18 &&
          g_dr16_status.last_size == 18 && g_dr16_status.last_event_ms == now_ms &&
          g_dr16_status.received_ms == now_ms);
    DR16Status copy{};
    CHECK(DR16_GetStatus(&copy) && copy.frame.channel[0] == 123 && copy.rx_events == 1);
    now_ms += 7;
    complete(18, 456);
    CHECK(receiving_buffer == first_buffer && g_dr16_status.good_frames == 2 &&
          g_dr16_status.consecutive_frames == 2 && g_dr16_status.rx_bytes == 36);
    CHECK(g_dr16_status.frame.channel[0] == 456 && g_dr16_status.received_ms == now_ms);

    const unsigned batch_sizes[] = {36, 54, 72};
    for (unsigned size : batch_sizes) {
        const auto good = g_dr16_status.good_frames;
        const auto consecutive = g_dr16_status.consecutive_frames;
        const auto bytes = g_dr16_status.rx_bytes, events = g_dr16_status.rx_events;
        now_ms += 7;
        complete(size, -100);
        CHECK(g_dr16_status.valid && g_dr16_status.good_frames == good + size / 18 &&
              g_dr16_status.consecutive_frames == consecutive + size / 18);
        CHECK(g_dr16_status.frame.channel[0] == -100 + int(size / 18) - 1 &&
              g_dr16_status.received_ms == now_ms);
        CHECK(g_dr16_status.rx_events == events + 1 && g_dr16_status.rx_bytes == bytes + size &&
              g_dr16_status.last_size == size);
    }
    const auto last_accepted_channel = g_dr16_status.frame.channel[0];
    const auto batches_good = g_dr16_status.good_frames;
    const auto batches_rejected = g_dr16_status.rejected_frames;
    complete(36, 222, true);
    CHECK(!g_dr16_status.valid && g_dr16_status.consecutive_frames == 0 &&
          g_dr16_status.good_frames == batches_good &&
          g_dr16_status.rejected_frames == batches_rejected + 1);
    CHECK(g_dr16_status.frame.channel[0] == last_accepted_channel); // Whole batch rejected atomically.

    const unsigned invalid_sizes[] = {0, 17, 19, 73};
    for (unsigned size : invalid_sizes) {
        const auto good = g_dr16_status.good_frames, rejected = g_dr16_status.rejected_frames;
        const auto previous_received_ms = g_dr16_status.received_ms;
        now_ms += 7;
        complete(size, -12);
        CHECK(!g_dr16_status.valid && g_dr16_status.consecutive_frames == 0);
        CHECK(g_dr16_status.good_frames == good && g_dr16_status.rejected_frames == rejected + 1);
        CHECK(g_dr16_status.last_size == size && g_dr16_status.last_event_ms == now_ms);
        CHECK(g_dr16_status.frame.channel[0] == last_accepted_channel &&
              g_dr16_status.received_ms == previous_received_ms); // No stale data made fresh.
    }
    CHECK(g_dr16_status.rx_events == 10 && g_dr16_status.rx_bytes == 343);
    CHECK(g_dr16_status.rx_running && starts == 11);
    now_ms += 7;
    complete(18, 77);
    CHECK(g_dr16_status.valid && g_dr16_status.consecutive_frames == 1 &&
          g_dr16_status.frame.channel[0] == 77);
    now_ms += 101;
    complete(18, 88);
    CHECK(g_dr16_status.valid && g_dr16_status.consecutive_frames == 1 &&
          g_dr16_status.frame.channel[0] == 88); // Reconnect needs new consecutive frames.

    // Rearm failure invalidates even an otherwise valid completed frame. Service
    // recovers DMA, but it cannot synthesize a new valid remote input.
    next_start_result = HAL_BUSY;
    const auto good = g_dr16_status.good_frames;
    const auto restart_errors = g_dr16_status.restart_errors;
    const auto old_received_ms = g_dr16_status.received_ms;
    now_ms += 7;
    complete(18, 99);
    CHECK(!g_dr16_status.valid && !g_dr16_status.rx_running &&
          g_dr16_status.consecutive_frames == 0);
    CHECK(g_dr16_status.last_start_result == HAL_BUSY &&
          g_dr16_status.restart_errors == restart_errors + 1);
    CHECK(g_dr16_status.good_frames == good && g_dr16_status.frame.channel[0] == 88 &&
          g_dr16_status.received_ms == old_received_ms);
    next_start_result = HAL_OK;
    now_ms += 4;
    DR16_Service();
    CHECK(aborts == 1 && g_dr16_status.rx_running && !g_dr16_status.valid);
    CHECK(g_dr16_status.service_ms == now_ms && g_dr16_status.last_start_result == HAL_OK);
    CHECK(critical_depth == 0 && start_was_in_critical);
    const auto recovered_starts = starts;
    dma_registers.NDTR = 55;
    now_ms += 4;
    DR16_Service();
    CHECK(starts == recovered_starts && g_dr16_status.service_ms == now_ms &&
          g_dr16_status.dma_remaining == 55 &&
          g_dr16_status.uart_rx_state == HAL_UART_STATE_BUSY_RX);
    complete(18, 111);
    CHECK(g_dr16_status.valid && g_dr16_status.frame.channel[0] == 111);

    // Hardware errors remain visible after the HAL clears its transient code.
    const auto errors = g_dr16_status.uart_errors;
    huart3.ErrorCode = HAL_UART_ERROR_PE | HAL_UART_ERROR_ORE;
    HAL_UART_ErrorCallback(&huart3);
    CHECK(g_dr16_status.uart_errors == errors + 1 && !g_dr16_status.valid &&
          !g_dr16_status.rx_running && g_dr16_status.consecutive_frames == 0);
    CHECK(g_dr16_status.last_uart_error == (HAL_UART_ERROR_PE | HAL_UART_ERROR_ORE) &&
          g_dr16_status.frame.channel[0] == 111);
    huart3.ErrorCode = HAL_UART_ERROR_NONE;
    next_start_result = HAL_ERROR;
    now_ms += 4;
    DR16_Service();
    CHECK(aborts == 2 && !g_dr16_status.rx_running && !g_dr16_status.valid &&
          g_dr16_status.last_start_result == HAL_ERROR);
    next_start_result = HAL_OK;
    now_ms += 4;
    DR16_Service();
    CHECK(aborts == 3 && g_dr16_status.rx_running && !g_dr16_status.valid && critical_depth == 0);
    CHECK(g_dr16_status.start_calls == 1); // Public start calls, not every DMA rearm.
    complete(18, -222);
    CHECK(g_dr16_status.valid && g_dr16_status.frame.channel[0] == -222);
    CHECK(g_dr16_status.last_uart_error == (HAL_UART_ERROR_PE | HAL_UART_ERROR_ORE));
    return 0;
}
} // namespace

extern "C" uint32_t HAL_GetTick() { return now_ms; }
extern "C" void vPortEnterCritical() { ++critical_depth; }
extern "C" void vPortExitCritical() { --critical_depth; }
extern "C" HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h, uint8_t *data,
                                                         uint16_t size) {
    ++starts;
    start_was_in_critical |= critical_depth > 0;
    if (next_start_result != HAL_OK)
        return next_start_result;
    receiving_buffer = data;
    receiving_capacity = size;
    h->RxState = HAL_UART_STATE_BUSY_RX;
    h->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    h->Instance->CR3 |= USART_CR3_DMAR;
    h->hdmarx->State = HAL_DMA_STATE_BUSY;
    h->hdmarx->Instance->CR = DMA_SxCR_EN | DMA_IT_HT | DMA_IT_TC;
    h->hdmarx->Instance->NDTR = size;
    return HAL_OK;
}
extern "C" HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *h) {
    ++aborts;
    h->RxState = HAL_UART_STATE_READY;
    h->hdmarx->State = HAL_DMA_STATE_READY;
    h->hdmarx->Instance->CR &= ~DMA_SxCR_EN;
    return HAL_OK;
}
extern "C" void test_entry() {
    test_failed_line = run();
    test_done = 1;
    __asm volatile("bkpt #0");
    for (;;) {}
}
