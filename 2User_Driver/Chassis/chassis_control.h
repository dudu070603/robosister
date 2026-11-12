#pragma once
#include "Chassis/chassis_config.h"
#include "Chassis/chassis_motor.h"
namespace engineer {
enum ChassisFault : uint32_t {
    ChassisNoFault = 0,
    ChassisConfigFault = 1,
    ChassisFeedbackFault = 2,
    ChassisTransportFault = 4,
    ChassisTimingFault = 8,
    ChassisCommandFault = 16,
    ChassisTemperatureFault = 32,
    ChassisEnergyFault = 64
};
struct ChassisCommand {
    bool valid = false, enabled = false, stop = true, brake = true, reset = true;
    bool independent = false; // 0档底盘直驱：不参与整车互锁/功率强制门控。
    float speed_request = 0, turn_request = 0;
    uint32_t updated_ms = 0, epoch = 0;
    uint32_t stop_epoch = 0; // Explicit stop cancels pulses even when already disabled.
    // Runtime tuning shares the normal owner and CAN sender. 1/2=current L/R,
    // 3/4=rotor-speed L/R; 5/6 reserve both wheels disabled for Yaw tuning.
    uint8_t tune_mode = 0;
    uint32_t tune_epoch = 0;
    float tune_value = 0, tune_current_limit_ma = 100;
    float tune_velocity_kp = 1, tune_velocity_ki = 0;
};
enum class ChassisTuneResult : uint32_t { Idle, Applied, Invalid, Busy, NotReady };
struct ChassisState {
    uint32_t updated_ms = 0, faults = 0;
    bool configuration_valid = false, enabled = false, wheels_stationary = false;
    float v_m_s = 0, omega_rad_s = 0, reference_v_m_s = 0, reference_omega_rad_s = 0;
    float target_rpm[2]{}, measured_rpm[2]{}, candidate_ma[2]{}, applied_ma[2]{};
    ChassisTuneResult tune_result = ChassisTuneResult::Idle;
    float tune_current_limit_ma = 0;
};
class ChassisControl {
  public:
    explicit ChassisControl(const ChassisConfig &c);
    const ChassisState &step(uint32_t now, const WheelFeedback wheel[2], const ChassisCommand &cmd);
    const ChassisState &step_tuning(uint32_t now, const WheelFeedback wheel[2],
                                   const ChassisCommand &cmd);
    void apply(const float ma[2], bool submitted);          // All final limits, or transport fault.
    void step_debug_speed(unsigned wheel, float rotor_rpm); // Same PI, owner-only suspended pulse.
    void direct_output(); // Open-loop pulse: discard speed-loop state before final submission.
    void trip(uint32_t reason);
    bool clear_faults(); // Disabled only; never reenables.
    ChassisTuneResult set_velocity_tuning(float kp, float ki, float current_ma);
    void set_motion_tuning(float accel, float decel, float angular_accel, float angular_decel,
                           float max_turn);
    const ChassisConfig &config() const {
        return c_;
    }
    const ChassisState &status() const {
        return s_;
    }
    static void differential(float v, float omega, float diameter, float track, float ratio,
                             float rpm[2]);

  private:
    ChassisConfig c_;
    const float startup_current_limit_ma_;
    ChassisState s_{};
    float integral_[2]{}, error_[2]{}, dt_ = 0;
    uint32_t previous_ms_ = 0, epoch_ = 0, stationary_since_ = 0;
    bool have_step_ = false, tracking_stationary_ = false, have_epoch_ = false;
    bool previous_reset_ = false, rearm_required_ = false;
    bool have_tune_epoch_ = false;
    uint8_t previous_tune_mode_ = 0;
    uint32_t previous_tune_epoch_ = 0;
    float tune_current_limit_ma_ = 100;
};
} // namespace engineer
