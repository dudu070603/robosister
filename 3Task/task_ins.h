#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t updated_ms, received_ms, frames, faults, calibration_count;
    uint32_t driver_errors, driver_attempts, driver_read_failures;
    float accel_body_m_s2[3], gyro_body_rad_s[3], gyro_bias_rad_s[3], up_body[3];
    float tilt_deg, accel_norm_m_s2, temperature_c;
    uint8_t ready, configuration_valid, confirmed, calibrated, valid, accel_trusted;
    uint8_t tipped, calibrating, driver_phase, accel_id, gyro_id;
} InsStatus;
extern InsStatus g_ins_status; // Read-only snapshot; use getter across tasks.
// 0 idle, 1 request static bias calibration. Consumed by ins_task; never enables.
extern volatile uint32_t g_ins_debug_command;

enum { INS_CALIBRATION_SOURCE_TIMEOUT_MS = 40 };

typedef struct {
    uint32_t published_ms;
    uint32_t oldest_source_updated_ms;
    uint32_t valid_for_ms;
    uint8_t ready;
    uint8_t allowed;
} InsCalibrationPermission;

// Pure permission validation, shared by the task and logic tests. The source
// budget is consumed before publication; publishing never extends old inputs.
static inline int INS_CalibrationPermissionValid(const InsCalibrationPermission *permission,
                                                 uint32_t now_ms, uint8_t revoked,
                                                 uint32_t revoked_at_ms) {
    if (!permission || permission->ready != 1 || permission->allowed != 1) {
        return 0;
    }
    const uint32_t source_age_ms = permission->published_ms - permission->oldest_source_updated_ms;
    if (source_age_ms > INS_CALIBRATION_SOURCE_TIMEOUT_MS ||
        permission->valid_for_ms > INS_CALIBRATION_SOURCE_TIMEOUT_MS - source_age_ms ||
        (uint32_t)(now_ms - permission->published_ms) > permission->valid_for_ms) {
        return 0;
    }
    return !revoked || (int32_t)(permission->oldest_source_updated_ms - revoked_at_ms) > 0;
}

// Vehicle aggregation publishes continuously. Every output owner revokes before
// attempting its first enable; snapshots from before that edge cannot regrant.
void INS_SetCalibrationPermission(const InsCalibrationPermission *permission);
void INS_RevokeCalibrationPermission(uint32_t now_ms);
int INS_GetStatus(InsStatus *out);
void ins_task(void *argument);
#ifdef __cplusplus
}
#endif
