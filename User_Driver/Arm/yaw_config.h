#pragma once
#include "arm_control.h"

namespace engineer {
// 大yaw负责人维护；仍由ArmControl/arm_task统一闭环并在CAN1发送。
// 禁止另建同一6020的驱动对象或第二个0x2FE发送者。
// 以下方向、零位范围、PID和300mA上限仍是台架占位，需负责人实测。
inline constexpr JointConfig kYawConfig = {
    Motor::Gm6020Current, 1, 5, 0, // 电流模式；CAN1；ID5，反馈0x209，指令0x2FE第一槽
    1.0f, 1,                      // 37T:37T同步带比例；反馈正向待确认
    -90, 90, 10, 30,              // 角范围deg、参考速度deg/s、加速度deg/s²
    300.0f, 2.0f, 20.0f, 5.0f,   // 电流mA、位置P、速度PI
    0, 0, 0, 70                  // 无DM协议量程；温度门槛°C
};
}
