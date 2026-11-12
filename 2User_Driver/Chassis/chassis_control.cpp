#include "Chassis/chassis_control.h"
#include <cmath>
namespace engineer {
namespace {
float clamp(float x, float lo, float hi) {
    return std::fmax(lo, std::fmin(x, hi));
}
float approach(float x, float goal, float step) {
    return x + clamp(goal - x, -step, step);
}
} // namespace
void ChassisControl::set_motion_tuning(float accel, float decel, float angular_accel,
                                       float angular_decel, float max_turn) {
    // Invalid writes leave the corresponding active setting unchanged.
    if (std::isfinite(accel) && accel > 0) c_.acceleration_m_s2 = accel;
    if (std::isfinite(decel) && decel > 0) c_.deceleration_m_s2 = decel;
    if (std::isfinite(angular_accel) && angular_accel > 0) c_.angular_accel_rad_s2 = angular_accel;
    if (std::isfinite(angular_decel) && angular_decel > 0)
        c_.angular_decel_rad_s2 = angular_decel;
    if (std::isfinite(max_turn) && max_turn > 0) c_.max_turn_rad_s = max_turn;
}
ChassisControl::ChassisControl(const ChassisConfig &c)
    : c_(c), startup_current_limit_ma_(c.max_current_ma) {
    const float values[] = {
        c.diameter_m,       c.track_m,           c.gear_ratio,        c.max_speed_m_s,
        c.max_turn_rad_s,   c.acceleration_m_s2, c.deceleration_m_s2, c.angular_accel_rad_s2,
        c.angular_decel_rad_s2, c.max_rotor_rpm, c.max_current_ma,   c.velocity_kp,
        c.stationary_rpm,   c.max_temperature_c};
    bool good = true;
    for (float x : values)
        good &= std::isfinite(x) && x > 0;
    good &= c.motor_id[0] >= 1 && c.motor_id[0] <= 4 && c.motor_id[1] >= 1 && c.motor_id[1] <= 4 &&
            c.motor_id[0] != c.motor_id[1];
    for (int d : c.direction)
        good &= d == 1 || d == -1;
    good &= std::isfinite(c.velocity_ki) && c.velocity_ki >= 0;
    good &= c.period_ms > 0 && c.feedback_timeout_ms > c.period_ms &&
            c.command_timeout_ms > c.period_ms && c.stationary_ms > 0 && c.max_current_ma <= 20000;
    s_.configuration_valid = good;
    if (!good)
        trip(ChassisConfigFault);
}
void ChassisControl::differential(float v, float w, float d, float b, float ratio, float rpm[2]) {
    const float scale = 60.0f * ratio / (3.14159265358979323846f * d);
    rpm[0] = (v - w * b * .5f) * scale;
    rpm[1] = (v + w * b * .5f) * scale;
}
void ChassisControl::trip(uint32_t reason) {
    if (s_.enabled || reason)
        rearm_required_ = true;
    s_.faults |= reason;
    s_.enabled = false;
    s_.reference_v_m_s = s_.reference_omega_rad_s = 0;
    for (unsigned i = 0; i < 2; ++i) {
        integral_[i] = 0;
        s_.candidate_ma[i] = s_.applied_ma[i] = s_.target_rpm[i] = 0;
    }
}
bool ChassisControl::clear_faults() {
    if (s_.enabled || !s_.configuration_valid)
        return false;
    s_.faults = 0;
    have_epoch_ = false;
    return true;
}
ChassisTuneResult ChassisControl::set_velocity_tuning(float kp, float ki, float current_ma) {
    // Validate everything before changing the live copy. The startup current
    // ceiling cannot be raised through a sequence of online requests.
    if (!std::isfinite(kp) || kp <= 0 || kp > 1000 || !std::isfinite(ki) || ki < 0 ||
        ki > 1000 || !std::isfinite(current_ma) || current_ma <= 0 ||
        current_ma > startup_current_limit_ma_)
        return ChassisTuneResult::Invalid;
    if (s_.enabled)
        return ChassisTuneResult::Busy;
    if (!s_.configuration_valid || s_.faults)
        return ChassisTuneResult::NotReady;
    c_.velocity_kp = kp;
    c_.velocity_ki = ki;
    c_.max_current_ma = current_ma;
    for (unsigned i = 0; i < 2; ++i) {
        integral_[i] = error_[i] = 0;
        s_.candidate_ma[i] = s_.applied_ma[i] = s_.target_rpm[i] = 0;
    }
    return ChassisTuneResult::Applied;
}
const ChassisState &ChassisControl::step(uint32_t now, const WheelFeedback wheel[2],
                                         const ChassisCommand &cmd) {
    if (!cmd.tune_mode || cmd.tune_mode >= 5) {
        have_tune_epoch_ = false;
        s_.tune_result = ChassisTuneResult::Idle;
        s_.tune_current_limit_ma = 0;
    }
    const bool reset_edge = cmd.reset && !previous_reset_;
    previous_reset_ = cmd.reset;
    if (!cmd.enabled)
        rearm_required_ = false; // Must observe explicit disabled input before rearm.
    const uint32_t elapsed = have_step_ ? uint32_t(now - previous_ms_) : c_.period_ms;
    previous_ms_ = now;
    have_step_ = true;
    s_.updated_ms = now;
    const bool timing = elapsed > 0 && elapsed <= c_.period_ms + 2;
    dt_ = timing ? elapsed * .001f : 0;
    bool feedback = true, temp = true;
    for (unsigned i = 0; i < 2; ++i) {
        feedback &= wheel[i].frames &&
                    uint32_t(now - wheel[i].received_ms) <= c_.feedback_timeout_ms &&
                    std::isfinite(wheel[i].rotor_rpm);
        temp &=
            std::isfinite(wheel[i].temperature_c) && wheel[i].temperature_c < c_.max_temperature_c;
        s_.measured_rpm[i] = wheel[i].rotor_rpm * c_.direction[i];
    }
    const float scale = 3.14159265358979323846f * c_.diameter_m / (60 * c_.gear_ratio);
    const float vl = s_.measured_rpm[0] * scale, vr = s_.measured_rpm[1] * scale;
    s_.v_m_s = (vl + vr) * .5f;
    s_.omega_rad_s = (vr - vl) / c_.track_m;
    const bool still = feedback && std::fabs(s_.measured_rpm[0]) < c_.stationary_rpm &&
                       std::fabs(s_.measured_rpm[1]) < c_.stationary_rpm;
    if (!still)
        tracking_stationary_ = false;
    else if (!tracking_stationary_) {
        tracking_stationary_ = true;
        stationary_since_ = now;
    }
    s_.wheels_stationary = still && uint32_t(now - stationary_since_) >= c_.stationary_ms;
    if (s_.enabled) {
        if (!timing)
            trip(ChassisTimingFault);
        if (!feedback)
            trip(ChassisFeedbackFault);
        if (!temp)
            trip(ChassisTemperatureFault);
    }
    const bool fresh = cmd.valid && uint32_t(now - cmd.updated_ms) <= c_.command_timeout_ms;
    const bool finite = std::isfinite(cmd.speed_request) && std::isfinite(cmd.turn_request) &&
                        std::fabs(cmd.speed_request) <= 1 && std::fabs(cmd.turn_request) <= 1;
    if (s_.enabled && (!fresh || !finite))
        trip(ChassisCommandFault);
    if ((!c_.hardware_confirmed && !(cmd.tune_mode >= 1 && cmd.tune_mode <= 4)) ||
        !s_.configuration_valid || !fresh || !finite || !cmd.enabled ||
        !feedback || !temp || !timing || s_.faults || rearm_required_) {
        trip(0);
        if (cmd.enabled)
            rearm_required_ = true; // Rejected enable requests cannot become delayed starts.
        else
            rearm_required_ = false;
        return s_;
    }
    if (!s_.enabled || !have_epoch_ || cmd.epoch != epoch_ || reset_edge) {
        s_.reference_v_m_s = s_.v_m_s;
        s_.reference_omega_rad_s = s_.omega_rad_s;
        for (float &i : integral_)
            i = 0;
    }
    epoch_ = cmd.epoch;
    have_epoch_ = true;
    s_.enabled = true;
    const float v = (cmd.stop || cmd.brake) ? 0 : cmd.speed_request * c_.max_speed_m_s;
    const float w = cmd.stop ? 0 : cmd.turn_request * c_.max_turn_rad_s;
    // With no normal drive request, drop both integrators every cycle. Otherwise
    // a wound-up integral can freeze against stiction and keep one wheel driven
    // long after the reference has reached zero. Single-wheel tuning keeps its
    // own PI state through step_tuning().
    const bool no_drive =
        !cmd.tune_mode && (cmd.stop || (cmd.brake && cmd.turn_request == 0));
    if (no_drive)
        integral_[0] = integral_[1] = 0;
    // Reverse through zero first, use the measured state on mode transitions.
    const float goal = s_.reference_v_m_s * v < 0 ? 0 : v;
    const float accel = std::fabs(goal) > std::fabs(s_.reference_v_m_s) ? c_.acceleration_m_s2
                                                                        : c_.deceleration_m_s2;
    s_.reference_v_m_s = approach(s_.reference_v_m_s, goal, accel * dt_);
    const bool slowing = std::fabs(w) < std::fabs(s_.reference_omega_rad_s);
    s_.reference_omega_rad_s =
        approach(s_.reference_omega_rad_s, w,
                 (slowing ? c_.angular_decel_rad_s2 : c_.angular_accel_rad_s2) * dt_);
    differential(s_.reference_v_m_s, s_.reference_omega_rad_s, c_.diameter_m, c_.track_m,
                 c_.gear_ratio, s_.target_rpm);
    const float peak = std::fmax(std::fabs(s_.target_rpm[0]), std::fabs(s_.target_rpm[1]));
    if (peak > c_.max_rotor_rpm)
        for (float &r : s_.target_rpm)
            r *= c_.max_rotor_rpm / peak;
    for (unsigned i = 0; i < 2; ++i) {
        error_[i] = s_.target_rpm[i] - s_.measured_rpm[i];
        s_.candidate_ma[i] =
            clamp(c_.velocity_kp * error_[i] + integral_[i], -c_.max_current_ma, c_.max_current_ma);
    }
    return s_;
}
const ChassisState &ChassisControl::step_tuning(uint32_t now, const WheelFeedback wheel[2],
                                               const ChassisCommand &cmd) {
    ChassisCommand guarded = cmd;
    guarded.speed_request = guarded.turn_request = 0;
    guarded.stop = guarded.brake = true;
    guarded.reset = false;
    guarded.epoch = cmd.tune_epoch;
    const bool valid = cmd.tune_mode >= 1 && cmd.tune_mode <= 4 &&
                       std::isfinite(cmd.tune_value) &&
                       std::fabs(cmd.tune_value) <= (cmd.tune_mode <= 2 ? startup_current_limit_ma_ : c_.max_rotor_rpm) &&
                       std::isfinite(cmd.tune_velocity_kp) && cmd.tune_velocity_kp > 0 &&
                       cmd.tune_velocity_kp <= 1000 && std::isfinite(cmd.tune_velocity_ki) &&
                       cmd.tune_velocity_ki >= 0 && cmd.tune_velocity_ki <= 1000 &&
                       std::isfinite(cmd.tune_current_limit_ma) &&
                       cmd.tune_current_limit_ma > 0 &&
                       cmd.tune_current_limit_ma <= startup_current_limit_ma_;
    const bool changed = !have_tune_epoch_ || previous_tune_mode_ != cmd.tune_mode ||
                         previous_tune_epoch_ != cmd.tune_epoch;
    if (!valid || changed) {
        guarded.enabled = false;
        step(now, wheel, guarded);
        if (!valid) {
            trip(ChassisCommandFault);
            s_.tune_result = ChassisTuneResult::Invalid;
        } else {
            have_tune_epoch_ = true;
            previous_tune_mode_ = cmd.tune_mode;
            previous_tune_epoch_ = cmd.tune_epoch;
            s_.tune_result = ChassisTuneResult::NotReady;
        }
        return s_;
    }
    const bool new_values = cmd.tune_velocity_kp != c_.velocity_kp ||
                            cmd.tune_velocity_ki != c_.velocity_ki ||
                            cmd.tune_current_limit_ma != tune_current_limit_ma_;
    if (new_values) {
        // Apply validated gains without stopping the wheel or requiring zero target.
        // Preserve the integrator and constrain it to the updated output bound.
        c_.velocity_kp = cmd.tune_velocity_kp;
        c_.velocity_ki = cmd.tune_velocity_ki;
        tune_current_limit_ma_ = cmd.tune_current_limit_ma;
        s_.tune_result = ChassisTuneResult::Applied;
        const float bound = std::fmin(startup_current_limit_ma_, std::fmin(c_.max_current_ma,
                                                        tune_current_limit_ma_));
        for (unsigned i = 0; i < 2; ++i)
            integral_[i] = clamp(integral_[i], -bound, bound);
    }
    step(now, wheel, guarded);
    s_.tune_current_limit_ma = std::fmin(startup_current_limit_ma_, std::fmin(c_.max_current_ma,
                                                     tune_current_limit_ma_));
    // Keep successful parameter acceptance visible while the target is zero; otherwise
    // a one-period success would disappear before FreeMASTER can sample it.
    s_.tune_result = s_.enabled || (!s_.faults && s_.tune_result == ChassisTuneResult::Applied)
                         ? ChassisTuneResult::Applied : ChassisTuneResult::NotReady;
    if (!s_.enabled)
        return s_;
    const unsigned selected = (cmd.tune_mode == 2 || cmd.tune_mode == 4) ? 1 : 0;
    if (cmd.tune_mode <= 2) {
        direct_output();
        s_.candidate_ma[0] = s_.candidate_ma[1] = 0;
        s_.candidate_ma[selected] = clamp(cmd.tune_value, -s_.tune_current_limit_ma,
                                          s_.tune_current_limit_ma);
    } else {
        step_debug_speed(selected, cmd.tune_value);
        s_.candidate_ma[selected] = clamp(s_.candidate_ma[selected], -s_.tune_current_limit_ma,
                                          s_.tune_current_limit_ma);
    }
    return s_;
}
void ChassisControl::apply(const float ma[2], bool submitted) {
    if (!submitted) {
        trip(ChassisTransportFault);
        return;
    }
    if (!s_.enabled)
        return;
    if (!std::isfinite(ma[0]) || !std::isfinite(ma[1])) {
        trip(ChassisCommandFault);
        return;
    }
    for (unsigned i = 0; i < 2; ++i) {
        s_.applied_ma[i] = clamp(ma[i], -c_.max_current_ma, c_.max_current_ma);
        const float raw = c_.velocity_kp * error_[i] + integral_[i];
        // The final submission is quantized to a C620 integer slot. Truncation
        // toward zero alone must not freeze PI integration every cycle.
        constexpr float quantization_tolerance_ma = 20000.0f / 16384.0f + 0.001f;
        const float residual = raw - s_.applied_ma[i];
        if (std::fabs(residual) <= quantization_tolerance_ma || residual * error_[i] <= 0)
            integral_[i] = clamp(integral_[i] + c_.velocity_ki * error_[i] * dt_,
                                 -c_.max_current_ma, c_.max_current_ma);
    }
}
void ChassisControl::direct_output() {
    for (unsigned i = 0; i < 2; ++i) {
        integral_[i] = 0;
        error_[i] = 0;
        s_.target_rpm[i] = 0;
    }
}
void ChassisControl::step_debug_speed(unsigned wheel, float rpm) {
    if (!s_.enabled || wheel > 1 || !std::isfinite(rpm) ||
        std::fabs(rpm) > c_.max_rotor_rpm)
        return;
    for (unsigned i = 0; i < 2; ++i) {
        s_.target_rpm[i] = i == wheel ? rpm : 0;
        error_[i] = s_.target_rpm[i] - s_.measured_rpm[i];
        s_.candidate_ma[i] = i == wheel ? clamp(c_.velocity_kp * error_[i] + integral_[i],
                                                -c_.max_current_ma,
                                                c_.max_current_ma)
                                        : 0;
        if (i != wheel) {
            error_[i] = 0;
            integral_[i] = 0;
        }
    }
}
} // namespace engineer
