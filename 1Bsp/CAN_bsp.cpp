//
// Created by Tessia on 2023/12/10.
//

#include "CAN_bsp.h"
#include <cstring>

#define MAX_CAN_DEV_NUM                                                                            \
    (14 *                                                                                          \
     4) // 因为过滤器一共28组，can1
        // can2均分，而每组过滤器有两个32位寄存器，16位list模式每个寄存器存2个id，所以max 14 * 4
can_device_receive *can_device_rx_list[2][MAX_CAN_DEV_NUM] = {0};

uint32_t can1_filter[14][4] = {0};
uint32_t can1_device_num = 0;

uint32_t can2_filter[14][4] = {0};
uint32_t can2_device_num = 0;

// can1和can2消息队列
extern osMessageQueueId_t can1_send_fifoHandle;
extern osMessageQueueId_t can2_send_fifoHandle;

// can1和can2信号量
extern osSemaphoreId_t CAN1CountingSemHandle; // 信号量，初始值为3，最大值为3
extern osSemaphoreId_t CAN2CountingSemHandle; // 信号量，初始值为3，最大值为3
static volatile uint32_t tx_generation[2] = {0, 0};
static can_bus_rx_callback bus_rx_callback[2] = {nullptr, nullptr};

bool can_set_bus_rx_callback(CAN_HandleTypeDef *handle, can_bus_rx_callback callback) {
    if (handle != &hcan1 && handle != &hcan2)
        return false;
    taskENTER_CRITICAL();
    bus_rx_callback[handle == &hcan1 ? 0 : 1] = callback;
    taskEXIT_CRITICAL();
    return true;
}

/**************************以下为can发送**********************************/

can_device_transmit::can_device_transmit(CAN_HandleTypeDef *hcan_, uint32_t id_, uint8_t len_,
                                         uint8_t *buf_data_)
    : member{hcan_, id_, len_, buf_data_} {}

can_device_transmit::can_device_transmit() : member{&hcan1, 0x000, 8, nullptr} {}

bool can_device_transmit::set_buf_address(uint8_t *buf_address) {
    if (buf_address != nullptr) {
        member.buf_data = buf_address;
        return true;
    } else {
        return false;
    }
}

/// 只判断标准帧
bool can_device_transmit::set_id(uint32_t id_) {
    if (id_ > 0x7FF) {
        return false;
    } else {
        member.id = id_;
        return true;
    }
}

osStatus_t can_device_transmit::can_send_msg() { // 实际上是发送进队列中
    if (!member.buf_data || member.len > 8 || member.id > 0x7ff ||
        (member.hcan != &hcan1 && member.hcan != &hcan2))
        return osErrorParameter;
    can_tx_frame frame{};
    frame.hcan = member.hcan;
    frame.id = member.id;
    frame.len = member.len;
    frame.queued_ms = HAL_GetTick();
    frame.generation = tx_generation[member.hcan == &hcan1 ? 0 : 1];
    memcpy(frame.data, member.buf_data, member.len);
    osStatus_t state;
    if (member.hcan == &hcan1) {
        state = osMessageQueuePut(can1_send_fifoHandle, &frame, 0, 0);
    } else {
        state = osMessageQueuePut(can2_send_fifoHandle, &frame, 0, 0);
    }
    return state;
}

volatile uint32_t can_tx_error_count[2] = {0, 0};
static void sending_service(CAN_HandleTypeDef *handle, osMessageQueueId_t queue,
                            osSemaphoreId_t sem, unsigned bus) {
    for (;;) {
        can_tx_frame msg{};
        if (osMessageQueueGet(queue, &msg, nullptr, osWaitForever) != osOK)
            continue;
        if (osSemaphoreAcquire(sem, 5) != osOK) {
            ++can_tx_error_count[bus];
            continue;
        }
        CAN_TxHeaderTypeDef header{};
        header.RTR = CAN_RTR_DATA;
        header.IDE = CAN_ID_STD;
        header.StdId = msg.id;
        header.DLC = msg.len;
        uint32_t mailbox;
        // 与can_cancel_pending使用同一临界区：禁止代号检查通过后被关控抢占，
        // 又把已取消的旧运动帧送进邮箱。获取信号量和等待始终在临界区外。
        taskENTER_CRITICAL();
        const bool cancelled = msg.generation != tx_generation[bus];
        const bool expired = uint32_t(HAL_GetTick() - msg.queued_ms) > 4;
        const HAL_StatusTypeDef sent =
            cancelled || expired ? HAL_ERROR
                                 : HAL_CAN_AddTxMessage(handle, &header, msg.data, &mailbox);
        taskEXIT_CRITICAL();
        if (cancelled) {
            osSemaphoreRelease(sem);
            continue;
        }
        if (sent != HAL_OK) {
            osSemaphoreRelease(sem);
            ++can_tx_error_count[bus];
        }
    }
}
void can1_sending_service(void *argument) {
    sending_service(&hcan1, can1_send_fifoHandle, CAN1CountingSemHandle, 0);
}
void can2_sending_service(void *argument) {
    sending_service(&hcan2, can2_send_fifoHandle, CAN2CountingSemHandle, 1);
}

void can_tx_complete_callback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        osSemaphoreRelease(CAN1CountingSemHandle); // 释放信号量
    } else {
        osSemaphoreRelease(CAN2CountingSemHandle);
    }
}

// 三个week函数
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}
void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}
void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}
void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan) {
    can_tx_complete_callback(hcan);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    const uint32_t error = HAL_CAN_GetError(hcan);
    const unsigned bus = hcan == &hcan1 ? 0 : 1;
    // 发送失败没有成功回调，同样要归还邮箱令牌，否则三次失败后永久堵塞。
    if (error & (HAL_CAN_ERROR_TX_ALST0 | HAL_CAN_ERROR_TX_TERR0))
        can_tx_complete_callback(hcan);
    if (error & (HAL_CAN_ERROR_TX_ALST1 | HAL_CAN_ERROR_TX_TERR1))
        can_tx_complete_callback(hcan);
    if (error & (HAL_CAN_ERROR_TX_ALST2 | HAL_CAN_ERROR_TX_TERR2))
        can_tx_complete_callback(hcan);
    ++can_tx_error_count[bus];
    HAL_CAN_ResetError(hcan);
}

void can_cancel_pending(uint8_t bus_mask) {
    taskENTER_CRITICAL();
    if (bus_mask & 1) {
        ++tx_generation[0];
        osMessageQueueReset(can1_send_fifoHandle);
        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    if (bus_mask & 2) {
        ++tx_generation[1];
        osMessageQueueReset(can2_send_fifoHandle);
        HAL_CAN_AbortTxRequest(&hcan2, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    taskEXIT_CRITICAL();
}

/**************************以下为can接收**********************************/

// Caller holds the registration critical section. Both buses use the same
// four-ID bank layout; the peripheral/bank offset is the only difference.
static void configure_device_filter(CAN_HandleTypeDef *handle, uint32_t index, uint32_t id) {
    const bool first_bus = handle == &hcan1;
    auto *banks = first_bus ? can1_filter : can2_filter;
    banks[index / 4][index % 4] = id;

    CAN_FilterTypeDef filter{};
    filter.FilterMode = CAN_FILTERMODE_IDLIST;
    filter.FilterScale = CAN_FILTERSCALE_16BIT;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    filter.FilterFIFOAssignment = first_bus ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
    filter.FilterBank = index / 4 + (first_bus ? 0 : 14);
    filter.FilterIdLow = banks[index / 4][0] << 5;
    filter.FilterMaskIdLow = banks[index / 4][1] << 5;
    filter.FilterIdHigh = banks[index / 4][2] << 5;
    filter.FilterMaskIdHigh = banks[index / 4][3] << 5;
    HAL_CAN_ConfigFilter(handle, &filter);
}

can_device_receive::can_device_receive(CAN_HandleTypeDef *hcan_, uint32_t id_,
                                       can_rx_callback *callback_)
    : hcan(hcan_), id(id_), index(0), rx_data{}, callback(callback_) {
    is_callback_none = 0;

    taskENTER_CRITICAL();
    if (hcan == &hcan1 && can1_device_num < MAX_CAN_DEV_NUM) {
        this->index = can1_device_num++;
        configure_device_filter(hcan, index, id);
        can_device_rx_list[0][this->index] = this;
    } else if (hcan == &hcan2 && can2_device_num < MAX_CAN_DEV_NUM) {
        this->index = can2_device_num++;
        configure_device_filter(hcan, index, id);
        can_device_rx_list[1][this->index] = this;
    }
    taskEXIT_CRITICAL();
}

bool can_device_receive::can_set_callback(can_rx_callback *callback_) {
    if (callback_ != nullptr) {
        callback = callback_;
        return true;
    } else {
        return false;
    }
}

void can_device_receive::can_modify_id(uint32_t id_) {
    id = id_;
    taskENTER_CRITICAL();
    if (hcan == &hcan1) {
        configure_device_filter(hcan, index, id);
        can_device_rx_list[0][this->index] = this;
    } else if (hcan == &hcan2) {
        configure_device_filter(hcan, index, id);
        can_device_rx_list[1][this->index] = this;
    }
    taskEXIT_CRITICAL();
}

static void receive_fifo(CAN_HandleTypeDef *hcan, uint32_t fifo) {
    if (hcan != &hcan1 && hcan != &hcan2)
        return;
    const unsigned bus = hcan == &hcan1 ? 0 : 1;
    const unsigned count = bus == 0 ? can1_device_num : can2_device_num;
    // FIFO最多3帧；限次处理，避免高流量使ISR长期占用CPU。
    for (unsigned n = 0; n < 3 && HAL_CAN_GetRxFifoFillLevel(hcan, fifo); ++n) {
        CAN_RxHeaderTypeDef header{};
        uint8_t data[8]{};
        if (HAL_CAN_GetRxMessage(hcan, fifo, &header, data) != HAL_OK)
            break;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8)
            continue;
        const auto receiver = bus_rx_callback[bus];
        if (receiver) {
            receiver(header.StdId, data, header.DLC, HAL_GetTick());
            continue;
        }
        // 按实际帧ID查表，避免CAN2过滤器FMI基址/空槽造成越界和空指针。
        for (unsigned i = 0; i < count && i < MAX_CAN_DEV_NUM; ++i) {
            auto *dev = can_device_rx_list[bus][i];
            if (!dev || !dev->matches(hcan, header.StdId))
                continue;
            memset(dev->rx_data, 0, 8);
            memcpy(dev->rx_data, data, header.DLC);
            dev->rx_dlc = header.DLC;
            if (dev->callback)
                dev->callback(dev, data);
            else
                dev->is_callback_none = 1;
            break;
        }
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    receive_fifo(hcan, CAN_RX_FIFO0);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    receive_fifo(hcan, CAN_RX_FIFO1);
}
