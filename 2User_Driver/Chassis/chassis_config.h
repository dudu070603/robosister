#pragma once
#include <cstdint>

namespace engineer {
struct ChassisConfig {
    // 轮子ID、方向和几何已由用户实测确认；正常底盘输出不再被整车确认门卡住。
    bool hardware_confirmed = true;
    uint8_t motor_id[2] = {1, 2}; // Physical test: old index 0 (ID2) was right; ID1 is left.
    int8_t direction[2] = {1, -1}; // Preserve each motor's validated sign: ID1 +1, ID2 -1.
    float diameter_m = .145f, track_m = .330f, gear_ratio = 19.0f;
    float max_speed_m_s = .80f,
    max_turn_rad_s = 5.0f;//最大转向角速度
    float acceleration_m_s2 = 1.0f,//推杆后加速的快慢
    deceleration_m_s2 = 1.0f,/*松杆后减速的快慢*/
    angular_accel_rad_s2 = 5.0f,//转向指令变大的快慢
    angular_decel_rad_s2 = 8.0f;//转向指令回零的快慢
    // 9000 is the rotor limit, not a recommended test speed. It also lets
    // tune_speed_rpm reach the full rotor range instead of stopping at 1500.
    float max_rotor_rpm = 9000, max_current_ma = 10000;
    float velocity_kp = 6.2f, velocity_ki = 1.0f; // User wheel tuning result; mA/rpm, mA/(rpm*s).
    float stationary_rpm = 15, max_temperature_c = 70;
    uint32_t feedback_timeout_ms = 50, command_timeout_ms = 40, stationary_ms = 200;
    uint32_t period_ms = 4;
};
// 0档底盘直驱：只用共用遥控解析值和两轮反馈，不依赖机械臂/Yaw/IMU/功率。
// 以后整车联调时改成 false，恢复 ChassisCommand 的整车许可路径。
inline bool kChassisIndependent = true;
inline constexpr ChassisConfig kChassisConfig{};
} // namespace engineer
