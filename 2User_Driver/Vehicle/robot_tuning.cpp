#include "Vehicle/robot_tuning.h"
#include <cmath>

namespace engineer {
void RobotTuning::block(uint32_t reason) {
    state_.reason = reason;
    state_.blocked = true;
    state_.ready = false;
    state_.current_active = false;
    state_.value = 0;
}

const TuneState &RobotTuning::step(const TuneInput &input) {
    state_.changed = !initialized_ || input.mode != state_.mode;
    initialized_ = true;
    if (state_.changed) {
        ++state_.epoch;
        state_.mode = input.mode;
        state_.ready = false;
        state_.current_active = false;
        state_.value = 0;
        enabled_seen_ = false;
    }

    // Mode 0 is an explicit reset of the tuning session, not a motor-enable
    // request. Normal control still owns its own rearm and neutral checks.
    if (input.mode == TuneNormal) {
        state_.ready = false;
        state_.blocked = false;
        state_.current_active = false;
        state_.value = 0;
        state_.reason = TuneNormalOperation;
        enabled_seen_ = false;
        return state_;
    }
    // blocked describes this cycle, not a latched zero-target handshake.
    // Owner clears targets on failure/timeout; recovery needs a fresh target.
    state_.blocked = false;
    if (input.mode > TuneYawPosition) {
        block(TuneInvalid);
        return state_;
    }
    if (input.stop) {
        block(TuneStopped);
        return state_;
    }
    // Selecting an object is harmless while output is disabled. The owner
    // clears enable on every selection; neither readiness nor stationary
    // checks are consumed until the operator explicitly enables the test.
    if (state_.changed) {
        state_.reason = TuneOutputDisabled;
        if (input.mode == TuneYawPosition && std::isfinite(input.yaw_position_deg))
            state_.value = input.yaw_position_deg;
        return state_;
    }
    if (!input.enable) {
        enabled_seen_ = false;
        state_.ready = false;
        state_.current_active = false;
        state_.value = 0;
        state_.reason = TuneOutputDisabled;
        return state_;
    }
    if (!input.common_ready || !input.selected_ready) {
        block(TuneNotReady);
        return state_;
    }
    if (!enabled_seen_) {
        if (input.mode == TuneYawPosition) {
            if (!std::isfinite(input.yaw_position_deg))
                block(TuneInvalid);
            else
                state_.value = input.yaw_position_deg;
        }
        if (!state_.blocked)
            state_.reason = TuneReady;
        enabled_seen_ = !state_.blocked;
        return state_;
    }

    float value = 0;
    float limit = 0;
    if (input.mode == TuneLeftCurrent || input.mode == TuneRightCurrent) {
        value = input.current_ma;
        limit = 10000;
    } else if (input.mode == TuneLeftSpeed || input.mode == TuneRightSpeed) {
        value = input.speed_rpm;
        limit = 9000;
    } else if (input.mode == TuneYawSpeed) {
        value = input.yaw_speed_deg_s;
        limit = 5;
    } else {
        value = input.yaw_target_deg;
        if (!std::isfinite(input.yaw_min_deg) || !std::isfinite(input.yaw_max_deg) ||
            input.yaw_min_deg >= input.yaw_max_deg || !std::isfinite(value) ||
            value < input.yaw_min_deg || value > input.yaw_max_deg) {
            block(TuneInvalid);
            return state_;
        }
    }
    if (!std::isfinite(value) || (input.mode != TuneYawPosition && std::fabs(value) > limit)) {
        block(TuneInvalid);
        return state_;
    }

    if (input.mode == TuneLeftCurrent || input.mode == TuneRightCurrent) {
        // Once a nonzero current begins, changing its magnitude or sign cannot
        // postpone the deadline. Unsigned subtraction also handles tick wrap.
        if (state_.current_active && uint32_t(input.now_ms - current_started_ms_) >= 500) {
            block(TuneCurrentTimeout);
            return state_;
        }
        if (value == 0) {
            state_.current_active = false;
        } else if (!state_.current_active) {
            current_started_ms_ = input.now_ms;
            state_.current_active = true;
        }
    }
    state_.value = value;
    state_.ready = true;
    state_.reason = TuneReady;
    return state_;
}
} // namespace engineer
