#pragma once
#include "Power/power_meter.h"
#include <cstdint>
namespace engineer {
struct ChassisPowerConfig {
    bool confirmed = false;
    float budget_w = 40, warning_energy_j = 200, stop_energy_j = 100, restart_energy_j = 250;
    float drive_current_ma = 1000, braking_current_ma = 500;
    uint32_t timeout_ms = 200, stopping_timeout_ms = 2000;
};
inline constexpr ChassisPowerConfig kChassisPowerConfig{};
enum class PowerPhase : uint8_t {
    Disabled,
    Ready,
    Driving,
    StopLowEnergy,
    StopFeedback,
    Inhibited
};
enum class PowerBlock : uint8_t {
    None,
    Unconfirmed,
    InvalidConfig,
    NoMeter,
    LowEnergy,
    FeedbackLost,
    StopTimeout,
    Stopped,
    ExternalStop
};
struct ChassisPowerState {
    PowerPhase phase = PowerPhase::Disabled;
    PowerBlock reason = PowerBlock::Unconfirmed;
    bool configuration_valid = false, confirmed = false, meter_online = false,
         session_allowed = false;
    bool stop_requested = false, inhibited = false;
    float speed_scale = 0, drive_scale = 0;
    uint32_t updated_ms = 0, stopping_since_ms = 0;
};
class ChassisPower {
  public:
    explicit ChassisPower(const ChassisPowerConfig &config = kChassisPowerConfig);
    const ChassisPowerState &step(uint32_t now, const PowerMeterStatus &meter, bool enabled,
                                  bool wheels_stationary);
    bool clear(uint32_t now, const PowerMeterStatus &meter, bool disabled);
    // Signed mechanical direction after wheel direction conversion. Braking is
    // explicitly bounded too; this conservative feedback limiter is not a fitted model.
    void limit(const float candidate_ma[2], const float rotor_rpm[2], float out_ma[2]) const;
    const ChassisPowerState &status() const {
        return s_;
    }

  private:
    ChassisPowerConfig c_;
    ChassisPowerState s_{};
    bool fresh(uint32_t now, const PowerMeterStatus &m) const;
};
} // namespace engineer
