//
// Created by Tessia on 2023/12/10.
//

#ifndef PEPPER_PICKER_GIMBAL_CAN_BSP_H
#define PEPPER_PICKER_GIMBAL_CAN_BSP_H

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "can_frame.h"

class can_device_receive;

typedef void(can_rx_callback)(can_device_receive *can_receive, uint8_t *data);

// Optional per-bus standard-data-frame receiver, called in the CAN ISR.
// A registered receiver owns dispatch on that bus; nullptr restores device-ID
// dispatch. Configure filters in the owning application before enabling them.
// The callback must copy data before returning and must never block.
using can_bus_rx_callback = void (*)(uint16_t id, const uint8_t *data, uint8_t length,
                                     uint32_t received_ms);
bool can_set_bus_rx_callback(CAN_HandleTypeDef *handle, can_bus_rx_callback callback);

struct can_device_transmit_member {
    CAN_HandleTypeDef *hcan;
    uint32_t id;
    uint8_t len;
    uint8_t *buf_data;
};

// Board transport only: register an ID receiver or a bus receiver. Applications
// own decoding/control; this layer only copies frames and dispatches callbacks.
class can_device_receive {

  public:
    can_device_receive(CAN_HandleTypeDef *hcan_ = &hcan1, uint32_t id_ = 0,
                       can_rx_callback *callback_ = nullptr);
    bool can_set_callback(can_rx_callback *callback_);
    void can_modify_id(uint32_t id_);
    bool matches(CAN_HandleTypeDef *handle, uint32_t std_id) const {
        return hcan == handle && id == std_id;
    }

  private:
    CAN_HandleTypeDef *hcan; // 通过函数传递的参数
    uint32_t id;             // 通过函数传递的参数
    uint32_t index;          // 计算出的索引

  public:
    uint8_t rx_data[8]; // 原始数据
    uint8_t rx_dlc = 0;
    can_rx_callback *callback; // 需要直接赋值
    uint8_t is_callback_none;  // 未定义回执函数
};

class can_device_transmit {

  public:
    can_device_transmit(CAN_HandleTypeDef *hcan_, uint32_t id_ = 0, uint8_t len_ = 8,
                        uint8_t *buf_data_ = nullptr); // init
    can_device_transmit();
    osStatus_t can_send_msg(); // 发送
    bool set_buf_address(uint8_t *buf_address);
    bool set_id(uint32_t id);

  public:
    can_device_transmit_member member;
};

#ifdef __cplusplus
extern "C" {
#endif

void can1_sending_service(void *argument);
void can2_sending_service(void *argument);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // PEPPER_PICKER_GIMBAL_CAN_BSP_H
