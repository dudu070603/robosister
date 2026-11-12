#pragma once
#include <cstdint>
namespace engineer {
struct AttitudeConfig {
    bool installation_confirmed = false, thresholds_confirmed = false;
    // Body +X forward, +Y left, +Z up. body[i]=sign[i]*sensor[axis[i]].
    // Must be a proper right-handed rotation, not a reflection.
    uint8_t sensor_axis[3] = {0, 1, 2};
    int8_t sensor_sign[3] = {1, 1, 1};
    float gravity_m_s2 = 9.80665f, correction_tau_s = .25f;
    float accel_tolerance_m_s2 = 1.5f, calibration_accel_tolerance_m_s2 = .5f;
    float calibration_max_gyro_rad_s = .08f, calibration_gyro_span_rad_s = .025f;
    float calibration_accel_span_m_s2 = .25f;
    float max_gyro_rad_s = 30, max_temperature_c = 75, min_temperature_c = -20;
    float trip_tilt_deg = 55, release_tilt_deg = 40;
    uint32_t calibration_samples = 500, calibration_min_ms = 1500;
    uint32_t max_sample_gap_ms = 20, max_accel_untrusted_ms = 500;
    uint32_t trip_ms = 150, release_ms = 500;
};
inline constexpr AttitudeConfig kAttitudeConfig{};
struct AttitudeSample {
    float accel_m_s2[3]{}, gyro_rad_s[3]{}, temperature_c = 0;
    uint32_t received_ms = 0, frame = 0;
};
enum AttitudeFault : uint32_t {
    AttitudeNoFault = 0,
    AttitudeConfiguration = 1,
    AttitudeUnconfirmed = 2,
    AttitudeNoCalibration = 4,
    AttitudeSampleInvalid = 8,
    AttitudeTiming = 16,
    AttitudeAccelUntrusted = 32,
    AttitudeCalibrationMoving = 64
};
struct AttitudeStatus {
    uint32_t updated_ms = 0, received_ms = 0, frames = 0, faults = 0, calibration_count = 0;
    bool configuration_valid = false, confirmed = false, calibrated = false, valid = false;
    bool accel_trusted = false, tipped = false, calibrating = false;
    float accel_body_m_s2[3]{}, gyro_body_rad_s[3]{}, gyro_bias_rad_s[3]{};
    float up_body[3] = {0, 0, 1}, accel_norm_m_s2 = 0, tilt_deg = 0;
    float temperature_c = 0;
};
// Gravity-vector complementary filter. No world yaw, quaternion, EKF or HAL.
// Sample time comes from acquisition; dt_s may use DWT unsigned cycle delta.
// calibration_allowed must mean all outputs disabled, not merely zero targets.
class AttitudeEstimator {
  public:
    explicit AttitudeEstimator(const AttitudeConfig &config = kAttitudeConfig);
    void restart_calibration();
    const AttitudeStatus &step(const AttitudeSample &sample, float dt_s, bool calibration_allowed);
    void invalidate(uint32_t now_ms, uint32_t reason = AttitudeSampleInvalid);
    const AttitudeStatus &status() const {
        return s_;
    }

  private:
    AttitudeConfig c_;
    AttitudeStatus s_{};
    uint32_t previous_frame_ = 0, previous_ms_ = 0, calibration_started_ms_ = 0;
    uint32_t accel_untrusted_since_ = 0, trip_since_ = 0, release_since_ = 0;
    bool have_sample_ = false, have_attitude_ = false, untrusted_tracking_ = false;
    bool trip_tracking_ = false, release_tracking_ = false;
    float gyro_sum_[3]{}, gyro_min_[3]{}, gyro_max_[3]{}, accel_min_[3]{}, accel_max_[3]{};
    void reset_calibration_window();
    bool calibration_sample(const float accel[3], const float gyro[3], uint32_t now);
};
} // namespace engineer
