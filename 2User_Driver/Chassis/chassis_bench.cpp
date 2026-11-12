#include "Chassis/chassis_bench.h"
#include <cmath>

namespace engineer {
bool chassis_bench_remote_ready(uint32_t now, const ChassisBenchRemoteInput &input) {
    bool ready = input.mapping_confirmed && input.valid && input.consecutive_frames >= 3 &&
                 uint32_t(now - input.received_ms) <= 100;
    for (int channel : input.channel) {
        ready &= std::abs(channel) <= 30;
    }
    const bool arm_safe = (input.switches_in_test_position && input.owner_idle) ||
                          (input.switches_in_hold_position && input.owner_holding);
    ready &= arm_safe && input.robot_ready && uint32_t(now - input.robot_updated_ms) <= 40;
    return ready;
}

bool chassis_bench_arm_holding(uint32_t now, const ChassisBenchArmInput &input) {
    const uint32_t age = now - input.updated_ms;
    if (!input.ready || !input.configuration_valid || !input.robot_ready || !input.robot_active ||
        !input.robot_standby || input.enabled_mask != 0x1e || input.faults || age > 40 ||
        uint32_t(now - input.robot_updated_ms) > 40 || !input.feedback_timeout_ms)
        return false;
    for (unsigned i = 0; i < 4; ++i) {
        if (!input.online[i] || !input.calibrated[i] ||
            uint64_t(input.feedback_age_ms[i]) + age > input.feedback_timeout_ms ||
            !std::isfinite(input.drive_velocity_deg_s[i]) ||
            !std::isfinite(input.stationary_speed_deg_s[i]) || input.stationary_speed_deg_s[i] <= 0 ||
            std::fabs(input.drive_velocity_deg_s[i]) > input.stationary_speed_deg_s[i])
            return false;
    }
    return true;
}

ChassisBenchFeedbackHealth chassis_bench_feedback_health(uint32_t now, const WheelFeedback wheel[2],
                                                         const ChassisConfig &config) {
    ChassisBenchFeedbackHealth health{};
    for (unsigned i = 0; i < 2; ++i) {
        health.fresh &=
            wheel[i].frames && uint32_t(now - wheel[i].received_ms) <= config.feedback_timeout_ms;
        health.temperature_ok &= wheel[i].temperature_c < config.max_temperature_c;
        health.stationary &= std::fabs(wheel[i].rotor_rpm) < config.stationary_rpm;
    }
    return health;
}

void ChassisBenchGate::cancel(ChassisBenchResult result) {
    state_.active = false;
    state_.value = 0;
    state_.mode = uint32_t(ChassisBenchMode::Stop);
    state_.result = result;
}

const ChassisBenchState &ChassisBenchGate::step(uint32_t now, const ChassisBenchRequest &request,
                                                const ChassisBenchHealth &health) {
    state_.clear_requested = false;
    const bool stop_changed = initialized_ && health.stop_epoch != previous_stop_epoch_;
    previous_stop_epoch_ = health.stop_epoch;
    initialized_ = true;
    const bool was_active = state_.active;
    const bool healthy = health.hardware_confirmed && health.mapping_confirmed &&
                         health.remote_ok && health.owner_safe && health.feedback_fresh &&
                         health.temperature_ok && health.transport_ok && health.timing_ok &&
                         !health.global_stop && !stop_changed;
    if (state_.active) {
        if (!healthy || health.controller_faulted || !health.controller_enabled) {
            cancel(ChassisBenchResult::Cancelled);
        } else if (int32_t(now - state_.until_ms) >= 0) {
            cancel(ChassisBenchResult::Expired);
        }
    }
    if (request.request_seq == state_.consumed_seq) {
        return state_;
    }
    state_.consumed_seq = request.request_seq;

    // Every new command first withdraws the previous pulse. A stream of new
    // sequence numbers cannot extend an existing pulse indefinitely.
    cancel(ChassisBenchResult::InvalidRequest);
    if (request.mode == uint32_t(ChassisBenchMode::Stop)) {
        state_.result = ChassisBenchResult::Stopped;
        return state_;
    }
    if (request.mode == uint32_t(ChassisBenchMode::Clear)) {
        if (was_active || health.controller_enabled) {
            state_.result = ChassisBenchResult::Busy;
        } else if (!healthy || !health.wheels_stationary) {
            state_.result = ChassisBenchResult::NotReady;
        } else {
            state_.clear_requested = true;
            state_.result = ChassisBenchResult::ClearRequested;
        }
        return state_;
    }
    const bool valid = (request.mode == uint32_t(ChassisBenchMode::Current) ||
                        request.mode == uint32_t(ChassisBenchMode::Speed)) &&
                       request.wheel < 2 && request.duration_ms > 0 && request.duration_ms <= 500 &&
                       std::isfinite(request.value) && std::fabs(request.value) <= 300 &&
                       request.suspended_confirm == 0x53555350u;
    if (!valid) {
        return state_;
    }
    if (was_active || health.controller_enabled) {
        state_.result = ChassisBenchResult::Busy;
        return state_;
    }
    if (!healthy || !health.wheels_stationary || health.controller_faulted) {
        state_.result = ChassisBenchResult::NotReady;
        return state_;
    }
    state_.active = true;
    state_.mode = request.mode;
    state_.wheel = request.wheel;
    state_.value = request.value;
    state_.until_ms = now + request.duration_ms;
    state_.result = ChassisBenchResult::Accepted;
    return state_;
}
} // namespace engineer
