#include "task_protect.h"

namespace {
// Defaults are safe even before the task explicitly calls Init.
TaskProtectStatus state{1u, PROTECT_BOOT, PROTECT_BOOT, 0u};
bool have_tick = false;
constexpr uint32_t known_reasons = PROTECT_BOOT | PROTECT_CONFIG | PROTECT_REMOTE |
    PROTECT_INTERLOCK | PROTECT_STOP_REQUEST | PROTECT_TIMING | PROTECT_ARM_FAULT |
    PROTECT_ENABLE_REJECTED;

void update_reasons(uint32_t reasons, uint32_t now_ms) {
    if (reasons != 0u) {
        // Record the arrival of a new cause or the actual enabled -> disabled edge.
        // An unchanged fault must not keep moving the original trip timestamp.
        if (!state.disabled || (reasons & ~state.current_reasons) != 0u)
            state.last_trip_ms = now_ms;
        state.disabled = 1u;
        state.latched_reasons |= reasons;
    }
    state.current_reasons = reasons;
}
}

void Task_Protect_Init(void) {
    state = {1u, PROTECT_BOOT, PROTECT_BOOT, 0u};
    have_tick = false;
}

void Task_Protect_Tick(const TaskProtectInput *input, uint32_t now_ms) {
    have_tick = true;
    if (!input || input->configuration_confirmed > 1u || input->remote_online > 1u ||
        input->interlock_online > 1u || input->stop_requested > 1u ||
        input->timing_ok > 1u || input->arm_fault > 1u) {
        update_reasons(PROTECT_CONFIG, now_ms);
        return;
    }
    uint32_t reasons = 0u;
    if (!input->configuration_confirmed) reasons |= PROTECT_CONFIG;
    if (!input->remote_online) reasons |= PROTECT_REMOTE;
    if (!input->interlock_online) reasons |= PROTECT_INTERLOCK;
    if (input->stop_requested) reasons |= PROTECT_STOP_REQUEST;
    if (!input->timing_ok) reasons |= PROTECT_TIMING;
    if (input->arm_fault) reasons |= PROTECT_ARM_FAULT;
    update_reasons(reasons, now_ms);
}

int Task_Protect_TryArm(void) {
    if (!have_tick || state.current_reasons != 0u) return 0;
    state.disabled = 0u;
    state.latched_reasons = 0u;
    return 1;
}

void Task_Protect_Trip(uint32_t reason, uint32_t now_ms) {
    // A malformed caller cannot accidentally use Trip(0) to clear a stop.
    if (reason == 0u || (reason & ~known_reasons) != 0u) reason = PROTECT_CONFIG;
    update_reasons(state.current_reasons | reason, now_ms);
}

TaskProtectStatus Task_Protect_GetStatus(void) {
    return state;
}
