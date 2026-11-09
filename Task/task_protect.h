#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ProtectReason {
    PROTECT_BOOT = 1u << 0,
    PROTECT_CONFIG = 1u << 1,
    PROTECT_REMOTE = 1u << 2,
    PROTECT_INTERLOCK = 1u << 3,
    PROTECT_STOP_REQUEST = 1u << 4,
    PROTECT_TIMING = 1u << 5,
    PROTECT_ARM_FAULT = 1u << 6,
    PROTECT_ENABLE_REJECTED = 1u << 7
};

typedef struct {
    uint8_t configuration_confirmed;
    uint8_t remote_online;
    uint8_t interlock_online;
    uint8_t stop_requested;
    uint8_t timing_ok;
    uint8_t arm_fault;
} TaskProtectInput;

typedef struct {
    uint8_t disabled;
    uint32_t current_reasons;
    uint32_t latched_reasons;
    uint32_t last_trip_ms;
} TaskProtectStatus;

// Only the arm/control task owns this state. Publish copies through Robot status;
// do not call these functions concurrently from another task or an ISR.
// This module never sends CAN frames, drives motors, or treats Standby as a fault.
void Task_Protect_Init(void);
void Task_Protect_Tick(const TaskProtectInput *input, uint32_t now_ms);
// Caller must additionally require an explicit operator request and neutral input.
// Healthy input alone never reenables control. Returns 1 only after a healthy Tick.
int Task_Protect_TryArm(void);
// Immediate software failure (e.g. rejected enable) latches protection until rearm.
void Task_Protect_Trip(uint32_t reason, uint32_t now_ms);
TaskProtectStatus Task_Protect_GetStatus(void);

#ifdef __cplusplus
}
#endif
