#include "Power/chassis_power.h"
#include <cmath>
namespace engineer {
namespace {
float clamp(float x, float l, float h) {
    return std::fmax(l, std::fmin(h, x));
}
} // namespace
ChassisPower::ChassisPower(const ChassisPowerConfig &c) : c_(c) {
    const float values[] = {c.budget_w,         c.warning_energy_j, c.stop_energy_j,
                            c.restart_energy_j, c.drive_current_ma, c.braking_current_ma};
    bool ok = true;
    for (float v : values)
        ok &= std::isfinite(v) && v > 0;
    ok &= c.budget_w < 50 && c.warning_energy_j > c.stop_energy_j &&
          c.restart_energy_j >= c.warning_energy_j && c.restart_energy_j <= 500 &&
          c.drive_current_ma <= 20000 && c.braking_current_ma <= c.drive_current_ma &&
          c.timeout_ms > 0 && c.stopping_timeout_ms > 0 && c.stopping_timeout_ms < 10000;
    s_.configuration_valid = ok;
    s_.confirmed = c.confirmed;
}
bool ChassisPower::fresh(uint32_t now, const PowerMeterStatus &m) const {
    return m.frames && uint32_t(now - m.received_ms) <= c_.timeout_ms && std::isfinite(m.power_w) &&
           std::isfinite(m.energy_j);
}
const ChassisPowerState &ChassisPower::step(uint32_t now, const PowerMeterStatus &m, bool enabled,
                                            bool stationary) {
    const bool had_permission = s_.session_allowed;
    s_.updated_ms = now;
    s_.meter_online = fresh(now, m);
    s_.stop_requested = false;
    s_.speed_scale = s_.drive_scale = 0;
    s_.session_allowed = false;
    if (!s_.configuration_valid || !c_.confirmed) {
        s_.phase = PowerPhase::Disabled;
        s_.reason = s_.configuration_valid ? PowerBlock::Unconfirmed : PowerBlock::InvalidConfig;
        return s_;
    }
    if (s_.inhibited) {
        s_.phase = PowerPhase::Inhibited;
        return s_;
    }
    const bool stopping =
        s_.phase == PowerPhase::StopLowEnergy || s_.phase == PowerPhase::StopFeedback;
    if (stopping) {
        if (!enabled || stationary ||
            uint32_t(now - s_.stopping_since_ms) >= c_.stopping_timeout_ms) {
            s_.inhibited = true;
            s_.phase = PowerPhase::Inhibited;
            s_.reason = stationary
                            ? PowerBlock::Stopped
                            : (!enabled ? PowerBlock::ExternalStop : PowerBlock::StopTimeout);
        } else {
            s_.session_allowed = true;
            s_.stop_requested = true;
        }
        return s_;
    }
    if (!s_.meter_online || m.energy_j <= c_.stop_energy_j) {
        // enabled is the actual wheel-controller state from the previous cycle.
        // A meter loss directly after its first enabled cycle still needs the
        // bounded braking phase if a healthy Ready state granted that cycle.
        if (enabled && had_permission) {
            s_.phase = s_.meter_online ? PowerPhase::StopLowEnergy : PowerPhase::StopFeedback;
            s_.reason = s_.meter_online ? PowerBlock::LowEnergy : PowerBlock::FeedbackLost;
            s_.stopping_since_ms = now;
            s_.session_allowed = true;
            s_.stop_requested = true;
        } else {
            s_.phase = PowerPhase::Disabled;
            s_.reason = s_.meter_online ? PowerBlock::LowEnergy : PowerBlock::NoMeter;
        }
        return s_;
    }
    s_.phase = enabled ? PowerPhase::Driving : PowerPhase::Ready;
    s_.reason = PowerBlock::None;
    s_.session_allowed = true;
    s_.speed_scale =
        clamp((m.energy_j - c_.stop_energy_j) / (c_.warning_energy_j - c_.stop_energy_j), 0, 1);
    s_.drive_scale = m.power_w > c_.budget_w ? clamp(c_.budget_w / m.power_w, 0, 1) : 1;
    return s_;
}
bool ChassisPower::clear(uint32_t now, const PowerMeterStatus &m, bool disabled) {
    if (!disabled || !s_.configuration_valid || !c_.confirmed || !fresh(now, m) ||
        m.energy_j < c_.restart_energy_j)
        return false;
    s_.inhibited = false;
    s_.phase = PowerPhase::Disabled;
    s_.reason = PowerBlock::None;
    s_.session_allowed = false;
    s_.stop_requested = false;
    s_.speed_scale = s_.drive_scale = 0;
    s_.stopping_since_ms = 0;
    return true;
}
void ChassisPower::limit(const float candidate[2], const float rpm[2], float out[2]) const {
    out[0] = out[1] = 0;
    if (!s_.session_allowed)
        return;
    for (unsigned i = 0; i < 2; ++i) {
        if (!std::isfinite(candidate[i]) || !std::isfinite(rpm[i])) {
            out[0] = out[1] = 0;
            return;
        }
        const bool braking = std::fabs(rpm[i]) > 5 && candidate[i] * rpm[i] < 0;
        if (s_.stop_requested) {
            // Never propel during a power-stop. Near zero a conservative hold
            // current remains bounded; the measured stationary timer finishes it.
            if (braking || std::fabs(rpm[i]) <= 5)
                out[i] = clamp(candidate[i], -c_.braking_current_ma, c_.braking_current_ma);
        } else {
            const float bound = braking ? c_.braking_current_ma
                                        : c_.drive_current_ma * s_.drive_scale * s_.speed_scale;
            out[i] = clamp(candidate[i], -bound, bound);
        }
    }
}
} // namespace engineer
