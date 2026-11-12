#include "dm_id_config_task.h"

#if DM_ID_CONFIG_MODE
#include "CAN_bsp.h"
#include <cstring>

extern "C" {
volatile DmIdConfigRequest g_dm_config_request{1, 1, 0};
dmconfig::Status g_dm_config_status{};
}

namespace {
struct Received {
    uint16_t id;
    uint8_t len, data[8];
    uint32_t ms;
};
osMessageQueueId_t feedback = nullptr;
volatile uint32_t dropped_rx = 0;

void cancel_pending() {
    can_cancel_pending(1);
    osMessageQueueReset(feedback);
}
void receive_config_frame(uint16_t id, const uint8_t *data, uint8_t len, uint32_t ms) {
    if (!feedback || len > 8)
        return;
    Received f{};
    f.id = id;
    f.len = len;
    f.ms = ms;
    std::memcpy(f.data, data, len);
    if (osMessageQueuePut(feedback, &f, 0, 0) != osOK)
        ++dropped_rx;
}
} // namespace

extern "C" void dm_id_config_task(void *) {
    feedback = osMessageQueueNew(32, sizeof(Received), nullptr);
    if (!feedback || osKernelGetTickFreq() != 1000)
        Error_Handler();
    // Application selects its receiver; the BSP has no ID-configuration branch.
    if (!can_set_bus_rx_callback(&hcan1, receive_config_frame))
        Error_Handler();
    // 未知MST_ID（可能为0）；只在独立固件中打开CAN1标准数据帧过滤器。
    CAN_FilterTypeDef filter{};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
        Error_Handler();

    dmconfig::Configurator config;
    uint32_t previous_errors = can_tx_error_count[0], previous_drops = dropped_rx;
    for (;;) {
        const bool was_busy = config.status().busy;
        uint32_t command, source_id, target_axis;
        taskENTER_CRITICAL();
        command = g_dm_config_request.command;
        source_id = g_dm_config_request.source_id;
        target_axis = g_dm_config_request.target_axis;
        g_dm_config_request.command = 0;
        taskEXIT_CRITICAL();
        if (command) {
            // 清掉旧命令和旧反馈；source/target必须先写，command最后写。
            cancel_pending();
            config.command(command, source_id, target_axis, HAL_GetTick());
        }
        Received f;
        for (unsigned n = 0; n < 32 && osMessageQueueGet(feedback, &f, nullptr, 0) == osOK; ++n)
            config.receive(f.id, f.data, f.len, f.ms);
        const uint32_t errors = can_tx_error_count[0], drops = dropped_rx;
        if (errors != previous_errors || drops != previous_drops ||
            (hcan1.Instance->ESR & CAN_ESR_BOFF)) {
            config.transport_fault();
            cancel_pending();
        }
        previous_errors = errors;
        previous_drops = drops;
        dmconfig::Frame frame{};
        if (config.step(HAL_GetTick(), frame)) {
            can_device_transmit tx(&hcan1, frame.id, frame.len, frame.data);
            if (tx.can_send_msg() != osOK) {
                config.transport_fault();
                cancel_pending();
            }
        }
        if (was_busy && !config.status().busy)
            cancel_pending();
        taskENTER_CRITICAL();
        g_dm_config_status = config.status();
        taskEXIT_CRITICAL();
        osDelay(2);
    }
}
#endif
