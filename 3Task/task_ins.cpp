#include "task_ins.h"
#include "main.h"
#include "bsp_dwt.h"
#include "bsp_imu.h"
#include "Imu/attitude_estimator.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"
#include <cstring>

namespace {
InsCalibrationPermission calibration_permission{};
uint32_t calibration_revoked_ms = 0;
bool calibration_revoked = false;

[[maybe_unused]] bool calibration_allowed(uint32_t now_ms) {
    taskENTER_CRITICAL();
    const auto permission = calibration_permission;
    const bool revoked = calibration_revoked;
    const uint32_t revoked_ms = calibration_revoked_ms;
    taskEXIT_CRITICAL();
    return INS_CalibrationPermissionValid(&permission, now_ms, revoked, revoked_ms);
}
} // namespace

extern "C" {
InsStatus g_ins_status{};
volatile uint32_t g_ins_debug_command = 0;
}
extern "C" void INS_SetCalibrationPermission(const InsCalibrationPermission *permission) {
    taskENTER_CRITICAL();
    if (INS_CalibrationPermissionValid(permission, permission ? permission->published_ms : 0,
                                       calibration_revoked, calibration_revoked_ms)) {
        calibration_permission = *permission;
    } else {
        calibration_permission = {};
    }
    taskEXIT_CRITICAL();
}

extern "C" void INS_RevokeCalibrationPermission(uint32_t now_ms) {
    taskENTER_CRITICAL();
    calibration_permission = {};
    calibration_revoked = true;
    calibration_revoked_ms = now_ms;
    taskEXIT_CRITICAL();
}

extern "C" int INS_GetStatus(InsStatus *out) {
    if (!out)
        return 0;
    taskENTER_CRITICAL();
    *out = g_ins_status;
    taskEXIT_CRITICAL();
    return out->ready;
}
extern "C" void ins_task(void *) {
#if defined(HAL_SPI_MODULE_ENABLED) && VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
    using namespace engineer;
    constexpr uint32_t period = 4;
    if (osKernelGetTickFreq() != 1000)
        Error_Handler();
    Bmi088 device;
    AttitudeEstimator attitude;
    const bool timing_ready = BSP_DWT_Init(HAL_RCC_GetHCLKFreq()) != 0;
    device.begin(HAL_GetTick());
    uint32_t tick = osKernelGetTickCount(), previous_cycles = 0;
    bool have_cycles = false;
    for (;;) {
        const uint32_t now = HAL_GetTick();
        uint32_t command;
        taskENTER_CRITICAL();
        command = g_ins_debug_command;
        g_ins_debug_command = 0;
        taskEXIT_CRITICAL();
        // A refused calibration request is not retained for a later surprise.
        if (command == 1 && calibration_allowed(now)) {
            attitude.restart_calibration();
            have_cycles = false;
        }
        Bmi088Sample sample{};
        if (device.poll(now, sample)) {
            AttitudeSample input{};
            std::memcpy(input.accel_m_s2, sample.accel_m_s2, sizeof(input.accel_m_s2));
            std::memcpy(input.gyro_rad_s, sample.gyro_rad_s, sizeof(input.gyro_rad_s));
            input.temperature_c = sample.temperature_c;
            input.received_ms = sample.received_ms;
            input.frame = sample.frames;
            const float dt = have_cycles
                                 ? BSP_DWT_ElapsedSeconds(sample.received_cycles, previous_cycles)
                                 : period * .001f;
            previous_cycles = sample.received_cycles;
            have_cycles = true;
            // Re-read after SPI: an enable may revoke while polling was preempted.
            const bool was_calibrated = attitude.status().calibrated;
            attitude.step(input, dt, calibration_allowed(HAL_GetTick()));
            // A higher-priority owner may also revoke during the pure estimate.
            // Discard a calibration completed across that edge before publishing.
            if (!was_calibrated && !calibration_allowed(HAL_GetTick())) {
                attitude.restart_calibration();
            }
        }
        const auto &driver = device.status();
        if (!timing_ready || driver.phase != Bmi088Phase::Ready || !attitude.status().frames ||
            uint32_t(HAL_GetTick() - attitude.status().received_ms) >
                kAttitudeConfig.max_sample_gap_ms)
            attitude.invalidate(HAL_GetTick(),
                                !timing_ready ? AttitudeTiming : AttitudeSampleInvalid);
        const auto &s = attitude.status();
        InsStatus out{};
        out.ready = 1;
        out.updated_ms = HAL_GetTick();
        out.received_ms = s.received_ms;
        out.frames = s.frames;
        out.faults = s.faults;
        out.calibration_count = s.calibration_count;
        out.driver_errors = driver.errors;
        out.driver_attempts = driver.attempts;
        out.driver_read_failures = driver.read_failures;
        out.driver_phase = uint8_t(driver.phase);
        out.accel_id = driver.accel_id;
        out.gyro_id = driver.gyro_id;
        out.configuration_valid = s.configuration_valid;
        out.confirmed = s.confirmed;
        out.calibrated = s.calibrated;
        out.valid = s.valid;
        out.accel_trusted = s.accel_trusted;
        out.tipped = s.tipped;
        out.calibrating = s.calibrating;
        out.tilt_deg = s.tilt_deg;
        out.accel_norm_m_s2 = s.accel_norm_m_s2;
        out.temperature_c = s.temperature_c;
        std::memcpy(out.accel_body_m_s2, s.accel_body_m_s2, sizeof(out.accel_body_m_s2));
        std::memcpy(out.gyro_body_rad_s, s.gyro_body_rad_s, sizeof(out.gyro_body_rad_s));
        std::memcpy(out.gyro_bias_rad_s, s.gyro_bias_rad_s, sizeof(out.gyro_bias_rad_s));
        std::memcpy(out.up_body, s.up_body, sizeof(out.up_body));
        taskENTER_CRITICAL();
        g_ins_status = out;
        taskEXIT_CRITICAL();
        const uint32_t current = osKernelGetTickCount();
        tick = int32_t(current - tick) > 0 ? current + period : tick + period;
        osDelayUntil(tick);
    }
#else
    // Four-axis bench, chassis/Yaw bench and ID firmware never consume IMU
    // as a fabricated healthy vehicle state or initialize its peripheral here.
    for (;;)
        osDelay(100);
#endif
}
