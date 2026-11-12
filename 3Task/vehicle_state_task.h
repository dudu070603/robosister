#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
    uint32_t updated_ms, reasons;
    uint8_t ready, healthy;
} VehicleHealth;
extern VehicleHealth g_vehicle_health;
int Vehicle_GetHealth(VehicleHealth *out);
uint32_t Vehicle_GetProtectionReasons(uint32_t now_ms);
void vehicle_state_task(void *argument);
#ifdef __cplusplus
}
#include "Vehicle/vehicle_state.h"
#include "task_protect.h"

// Task-layer protocol validation and fault mapping. Kept pure so tests exercise
// the actual bridge without starting RTOS tasks or fabricating healthy sensors.
inline uint32_t Vehicle_ProtectionFromHealth(uint32_t now_ms, const VehicleHealth *health,
                                             uint32_t timeout_ms) {
    if (!health || health->ready != 1 || health->healthy > 1 || timeout_ms == 0 ||
        uint32_t(now_ms - health->updated_ms) > timeout_ms) {
        return PROTECT_VEHICLE_STATE;
    }
    constexpr uint32_t known = engineer::VehicleConfiguration | engineer::VehicleChassisStale |
                               engineer::VehicleArmStale | engineer::VehicleImuInvalid |
                               engineer::VehicleTipped | engineer::VehicleWheelFault |
                               engineer::VehicleEnergyInhibited | engineer::VehicleArmFault;
    if ((health->reasons & ~known) != 0 || (!health->healthy && health->reasons == 0)) {
        return PROTECT_VEHICLE_STATE;
    }
    uint32_t reasons = 0;
    if (health->reasons &
        (engineer::VehicleConfiguration | engineer::VehicleArmStale | engineer::VehicleArmFault)) {
        reasons |= PROTECT_VEHICLE_STATE;
    }
    if (health->reasons & (engineer::VehicleChassisStale | engineer::VehicleWheelFault)) {
        reasons |= PROTECT_CHASSIS;
    }
    if (health->reasons & engineer::VehicleImuInvalid) {
        reasons |= PROTECT_IMU;
    }
    if (health->reasons & engineer::VehicleTipped) {
        reasons |= PROTECT_TIPPED;
    }
    if (health->reasons & engineer::VehicleEnergyInhibited) {
        reasons |= PROTECT_POWER;
    }
    return reasons;
}
#endif
