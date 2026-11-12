#pragma once
#include <cstdint>

namespace engineer {
enum TuneMode : uint32_t {
    TuneNormal = 0,
    TuneLeftCurrent = 1,
    TuneRightCurrent = 2,
    TuneLeftSpeed = 3,
    TuneRightSpeed = 4,
    TuneYawSpeed = 5,
    TuneYawPosition = 6
};
enum TuneReason : uint32_t {
    TuneNormalOperation = 0,
    TuneReady = 1,
    TuneNotReady = 2,
    TuneStopped = 3,
    TuneInvalid = 4,
    TuneCurrentTimeout = 5,
    TuneWaitingStill = 6,
    TuneOutputDisabled = 7
};
struct TuneInput {
    uint32_t mode = TuneNormal, now_ms = 0;
    bool common_ready = false, selected_ready = false, stationary = false, stop = false;
    float current_ma = 0, speed_rpm = 0, yaw_speed_deg_s = 0, yaw_target_deg = 0;
    float yaw_position_deg = 0, yaw_min_deg = 0, yaw_max_deg = 0;
    bool enable = false;
};
struct TuneState {
    uint32_t mode = TuneNormal, epoch = 0, reason = TuneNormalOperation;
    bool changed = false, ready = false, blocked = false, current_active = false;
    float value = 0;
};

// One control owner calls step(). A mode change always returns ready=false so
// the owner can clear public targets and the old object's history first.
// Failed entry, health loss and STOP never become delayed starts: visit mode 0
// before explicitly selecting a new tuning mode. This class does not enable or
// disable motors and does not replace the device owner's local protection.
class RobotTuning {
  public:
    const TuneState &step(const TuneInput &input);
    const TuneState &status() const { return state_; }

  private:
    TuneState state_{};
    uint32_t current_started_ms_ = 0;
    bool initialized_ = false, enabled_seen_ = false;
    void block(uint32_t reason);
};
} // namespace engineer
