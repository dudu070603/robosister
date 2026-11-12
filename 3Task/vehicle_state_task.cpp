#include "vehicle_state_task.h"
#include "arm_task.h"
#include "chassis_task.h"
#include "task_ins.h"
#include "robot_control.h"
#include "Arm/arm_control.h"
#include "Chassis/chassis_config.h"
#include "Imu/attitude_estimator.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include <cmath>

extern "C" {
VehicleHealth g_vehicle_health{};
}

extern "C" int Vehicle_GetHealth(VehicleHealth *out) {
    if (!out) {
        return 0;
    }
    taskENTER_CRITICAL();
    *out = g_vehicle_health;
    taskEXIT_CRITICAL();
    return out->ready;
}

extern "C" uint32_t Vehicle_GetProtectionReasons(uint32_t now_ms) {
    VehicleHealth health{};
    Vehicle_GetHealth(&health);
    return Vehicle_ProtectionFromHealth(now_ms, &health,
                                        engineer::kVehicleConfig.status_timeout_ms);
}

#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
namespace {

// Translate publication protocols into only the values needed by domain policy.
engineer::VehicleInput adapt_inputs(const ArmStatus &arm, const ChassisStatus &chassis,
                                    const InsStatus &imu, float yaw_half_range_deg) {
    engineer::VehicleInput input{};
    auto &ch = input.chassis;
    ch.updated_ms = chassis.updated_ms;
    ch.faults = chassis.faults;
    ch.ready = chassis.ready;
    ch.configuration_valid = chassis.configuration_valid;
    ch.hardware_confirmed = chassis.hardware_confirmed;
    ch.power_configuration_valid = chassis.power_configuration_valid;
    ch.power_confirmed = chassis.power_confirmed;
    ch.power_session_allowed = chassis.power_session_allowed;
    ch.power_stop_requested = chassis.power_stop_requested;
    ch.wheels_stationary = chassis.wheels_stationary;
    for (unsigned i = 0; i < 2; ++i) {
        ch.feedback_received[i] = chassis.feedback_frames[i] != 0;
        ch.feedback_ms[i] = chassis.feedback_ms[i];
    }

    auto &a = input.arm;
    a.updated_ms = arm.updated_ms;
    a.faults = arm.faults;
    a.ready = arm.ready;
    a.configuration_valid = arm.configuration_valid;
    a.all_feedback_received = true;
    a.all_online = true;
    a.all_calibrated = true;
    a.all_positions_finite = true;
    for (const auto &joint : arm.joint) {
        a.all_feedback_received &= joint.received != 0;
        a.all_online &= joint.online != 0;
        a.all_calibrated &= joint.calibrated != 0;
        a.all_positions_finite &= std::isfinite(joint.position_deg);
        if (joint.feedback_age_ms > a.oldest_feedback_age_ms) {
            a.oldest_feedback_age_ms = joint.feedback_age_ms;
        }
    }
    a.yaw_deg = arm.joint[ARM_YAW].position_deg;
    a.big_pitch_deg = arm.joint[ARM_BIG_PITCH].position_deg;
    a.small_pitch_deg = arm.joint[ARM_SMALL_PITCH].position_deg;
    a.roll_deg = arm.joint[ARM_ROLL].position_deg;

    auto &sensor = input.imu;
    sensor.updated_ms = imu.updated_ms;
    sensor.received_ms = imu.received_ms;
    sensor.faults = imu.faults;
    sensor.ready = imu.ready;
    sensor.configuration_valid = imu.configuration_valid;
    sensor.valid = imu.valid;
    sensor.confirmed = imu.confirmed;
    sensor.calibrated = imu.calibrated;
    sensor.tipped = imu.tipped;
    input.yaw_half_range_deg = yaw_half_range_deg;
    return input;
}

// Calibration does not depend on vehicle health: disabled publishers must be
// able to establish a valid IMU before motion can be armed.
void publish_calibration_permission(uint32_t now, const RobotControlStatus &robot,
                                    const ChassisStatus &chassis, const ArmStatus &arm) {
    const uint32_t robot_age = now - robot.updated_ms;
    const uint32_t chassis_age = now - chassis.updated_ms;
    const uint32_t arm_age = now - arm.updated_ms;
    const bool all_disabled = robot.ready && !robot.active && chassis.ready && !chassis.enabled &&
                              arm.ready && !arm.enabled_mask;
    const bool all_fresh = robot_age <= INS_CALIBRATION_SOURCE_TIMEOUT_MS &&
                           chassis_age <= INS_CALIBRATION_SOURCE_TIMEOUT_MS &&
                           arm_age <= INS_CALIBRATION_SOURCE_TIMEOUT_MS;
    InsCalibrationPermission permission{};
    permission.published_ms = now;
    permission.ready = 1;
    permission.allowed = all_disabled && all_fresh;
    if (permission.allowed) {
        uint32_t oldest_age = robot_age;
        if (chassis_age > oldest_age) {
            oldest_age = chassis_age;
        }
        if (arm_age > oldest_age) {
            oldest_age = arm_age;
        }
        permission.oldest_source_updated_ms = now - oldest_age;
        permission.valid_for_ms = INS_CALIBRATION_SOURCE_TIMEOUT_MS - oldest_age;
    }
    INS_SetCalibrationPermission(&permission);
}

RobotInterlocks adapt_interlocks(const engineer::VehicleInterlocks &input) {
    RobotInterlocks output{};
    output.chassis_stopped = input.chassis_stopped;
    output.arm_allows_drive = input.arm_allows_drive;
    output.arm_allows_fast = input.arm_allows_fast;
    output.drive_yaw_allowed = input.drive_yaw_allowed;
    output.drive_yaw_min_deg = input.drive_yaw_min_deg;
    output.drive_yaw_max_deg = input.drive_yaw_max_deg;
    output.drive_yaw_max_speed_deg_s = input.drive_yaw_max_speed_deg_s;
    return output;
}

} // namespace
#endif

extern "C" void vehicle_state_task(void *) {
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
    const engineer::VehicleTiming timing{engineer::kChassisConfig.feedback_timeout_ms,
                                         engineer::kArmConfig.feedback_timeout_ms,
                                         engineer::kAttitudeConfig.max_sample_gap_ms};
    uint32_t tick = osKernelGetTickCount();
    for (;;) {
        ArmStatus arm{};
        ChassisStatus chassis{};
        InsStatus imu{};
        RobotControlStatus robot{};
        YawLimitStatus yaw{};
        Arm_GetStatus(&arm);
        Chassis_GetStatus(&chassis);
        INS_GetStatus(&imu);
        Robot_GetControlStatus(&robot);
        Yaw_GetLimitStatus(&yaw);
        const uint32_t now = HAL_GetTick();
        publish_calibration_permission(now, robot, chassis, arm);

        const auto input = adapt_inputs(arm, chassis, imu, yaw.active_half_range_deg);
        const auto state = engineer::evaluate_vehicle(now, engineer::kVehicleConfig, input, timing);
        const auto interlocks = adapt_interlocks(state.locks);
        Robot_SetInterlocks(&interlocks);
        taskENTER_CRITICAL();
        g_vehicle_health = {now, state.reasons, 1, uint8_t(state.healthy)};
        taskEXIT_CRITICAL();

        const uint32_t current = osKernelGetTickCount();
        tick = int32_t(current - tick) > 0 ? current + 20 : tick + 20;
        osDelayUntil(tick);
    }
#else
    for (;;) {
        osDelay(100);
    }
#endif
}
