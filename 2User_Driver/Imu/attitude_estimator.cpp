#include "Imu/attitude_estimator.h"
#include <cmath>
namespace engineer {
namespace {
float norm(const float x[3]) {
    return std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}
float clamp(float x, float lo, float hi) {
    return std::fmax(lo, std::fmin(x, hi));
}
bool positive(float x) {
    return std::isfinite(x) && x > 0;
}
} // namespace
AttitudeEstimator::AttitudeEstimator(const AttitudeConfig &c) : c_(c) {
    bool good = c.sensor_axis[0] < 3 && c.sensor_axis[1] < 3 && c.sensor_axis[2] < 3 &&
                c.sensor_axis[0] != c.sensor_axis[1] && c.sensor_axis[0] != c.sensor_axis[2] &&
                c.sensor_axis[1] != c.sensor_axis[2];
    for (int sign : c.sensor_sign)
        good &= sign == 1 || sign == -1;
    if (good) {
        const unsigned inversions = (c.sensor_axis[0] > c.sensor_axis[1]) +
                                    (c.sensor_axis[0] > c.sensor_axis[2]) +
                                    (c.sensor_axis[1] > c.sensor_axis[2]);
        const int parity = (inversions & 1) ? -1 : 1;
        good = parity * c.sensor_sign[0] * c.sensor_sign[1] * c.sensor_sign[2] == 1;
    }
    const float values[] = {c.gravity_m_s2,
                            c.correction_tau_s,
                            c.accel_tolerance_m_s2,
                            c.calibration_accel_tolerance_m_s2,
                            c.calibration_max_gyro_rad_s,
                            c.calibration_gyro_span_rad_s,
                            c.calibration_accel_span_m_s2,
                            c.max_gyro_rad_s,
                            c.trip_tilt_deg,
                            c.release_tilt_deg};
    for (float x : values)
        good &= positive(x);
    good &= std::isfinite(c.min_temperature_c) && std::isfinite(c.max_temperature_c) &&
            c.min_temperature_c < c.max_temperature_c;
    good &= c.release_tilt_deg < c.trip_tilt_deg && c.trip_tilt_deg < 90 &&
            c.calibration_samples >= 10 && c.calibration_min_ms > 0 && c.max_sample_gap_ms > 0 &&
            c.max_sample_gap_ms <= 100 && c.max_accel_untrusted_ms >= c.max_sample_gap_ms &&
            c.trip_ms > 0 && c.release_ms > 0;
    good &= c.calibration_accel_tolerance_m_s2 < c.accel_tolerance_m_s2 &&
            c.accel_tolerance_m_s2 < c.gravity_m_s2 * .5f;
    good &= c.calibration_max_gyro_rad_s < c.max_gyro_rad_s;
    s_.configuration_valid = good;
    s_.confirmed = c.installation_confirmed && c.thresholds_confirmed;
    s_.faults = (!good ? AttitudeConfiguration : AttitudeNoFault) |
                (!s_.confirmed ? AttitudeUnconfirmed : AttitudeNoFault) | AttitudeNoCalibration;
    s_.calibrating = true;
}
void AttitudeEstimator::reset_calibration_window() {
    s_.calibration_count = 0;
    for (float &x : gyro_sum_)
        x = 0;
}
void AttitudeEstimator::restart_calibration() {
    s_.calibrated = false;
    s_.valid = false;
    s_.calibrating = true;
    s_.faults |= AttitudeNoCalibration;
    have_attitude_ = false;
    reset_calibration_window();
}
bool AttitudeEstimator::calibration_sample(const float a[3], const float g[3], uint32_t now) {
    if (std::fabs(norm(a) - c_.gravity_m_s2) > c_.calibration_accel_tolerance_m_s2 ||
        norm(g) > c_.calibration_max_gyro_rad_s) {
        reset_calibration_window();
        return false;
    }
    if (!s_.calibration_count) {
        calibration_started_ms_ = now;
        for (unsigned i = 0; i < 3; ++i) {
            gyro_min_[i] = gyro_max_[i] = g[i];
            accel_min_[i] = accel_max_[i] = a[i];
        }
    }
    for (unsigned i = 0; i < 3; ++i) {
        gyro_min_[i] = std::fmin(gyro_min_[i], g[i]);
        gyro_max_[i] = std::fmax(gyro_max_[i], g[i]);
        accel_min_[i] = std::fmin(accel_min_[i], a[i]);
        accel_max_[i] = std::fmax(accel_max_[i], a[i]);
        if (gyro_max_[i] - gyro_min_[i] > c_.calibration_gyro_span_rad_s ||
            accel_max_[i] - accel_min_[i] > c_.calibration_accel_span_m_s2) {
            reset_calibration_window();
            return false;
        }
    }
    for (unsigned i = 0; i < 3; ++i)
        gyro_sum_[i] += g[i];
    ++s_.calibration_count;
    if (s_.calibration_count < c_.calibration_samples ||
        uint32_t(now - calibration_started_ms_) < c_.calibration_min_ms)
        return true;
    for (unsigned i = 0; i < 3; ++i)
        s_.gyro_bias_rad_s[i] = gyro_sum_[i] / float(s_.calibration_count);
    s_.calibrated = true;
    s_.calibrating = false;
    have_attitude_ = false;
    return true;
}
void AttitudeEstimator::invalidate(uint32_t now, uint32_t reason) {
    s_.updated_ms = now;
    s_.valid = false;
    s_.faults |= reason;
    // Calibration requires one uninterrupted static window. Invalid/duplicate
    // samples and driver gaps cannot count toward the same window.
    if (!s_.calibrated)
        reset_calibration_window();
    trip_tracking_ = release_tracking_ = false;
}
const AttitudeStatus &AttitudeEstimator::step(const AttitudeSample &sample, float dt,
                                              bool calibration_allowed) {
    s_.updated_ms = sample.received_ms;
    uint32_t faults = (!s_.configuration_valid ? AttitudeConfiguration : AttitudeNoFault) |
                      (!s_.confirmed ? AttitudeUnconfirmed : AttitudeNoFault);
    if (!s_.configuration_valid) {
        invalidate(sample.received_ms, faults);
        return s_;
    }
    bool finite = std::isfinite(sample.temperature_c) &&
                  sample.temperature_c >= c_.min_temperature_c &&
                  sample.temperature_c < c_.max_temperature_c;
    for (unsigned i = 0; i < 3; ++i)
        finite &= std::isfinite(sample.accel_m_s2[i]) && std::isfinite(sample.gyro_rad_s[i]);
    if (!finite || !sample.frame || (have_sample_ && sample.frame == previous_frame_)) {
        invalidate(sample.received_ms, AttitudeSampleInvalid);
        return s_;
    }
    const uint32_t gap = have_sample_ ? uint32_t(sample.received_ms - previous_ms_) : 0;
    const bool timing = std::isfinite(dt) && dt > 0 && dt <= float(c_.max_sample_gap_ms) * .001f &&
                        (!have_sample_ || (gap > 0 && gap <= c_.max_sample_gap_ms));
    previous_ms_ = sample.received_ms;
    previous_frame_ = sample.frame;
    have_sample_ = true;
    s_.received_ms = sample.received_ms;
    s_.frames = sample.frame;
    s_.temperature_c = sample.temperature_c;
    for (unsigned i = 0; i < 3; ++i) {
        s_.accel_body_m_s2[i] = c_.sensor_sign[i] * sample.accel_m_s2[c_.sensor_axis[i]];
        s_.gyro_body_rad_s[i] = c_.sensor_sign[i] * sample.gyro_rad_s[c_.sensor_axis[i]];
    }
    s_.accel_norm_m_s2 = norm(s_.accel_body_m_s2);
    if (!std::isfinite(s_.accel_norm_m_s2) || s_.accel_norm_m_s2 < .01f ||
        norm(s_.gyro_body_rad_s) > c_.max_gyro_rad_s) {
        reset_calibration_window();
        invalidate(sample.received_ms, AttitudeSampleInvalid);
        return s_;
    }
    if (!timing) {
        have_attitude_ = false;
        reset_calibration_window();
        faults |= AttitudeTiming;
    }
    if (!s_.calibrated) {
        if (!calibration_allowed || !timing ||
            !calibration_sample(s_.accel_body_m_s2, s_.gyro_body_rad_s, sample.received_ms)) {
            reset_calibration_window();
            faults |= AttitudeCalibrationMoving;
        }
        if (!s_.calibrated)
            faults |= AttitudeNoCalibration;
    }
    for (unsigned i = 0; i < 3; ++i)
        s_.gyro_body_rad_s[i] -= s_.gyro_bias_rad_s[i];
    s_.accel_trusted = std::fabs(s_.accel_norm_m_s2 - c_.gravity_m_s2) <= c_.accel_tolerance_m_s2;
    if (!s_.accel_trusted) {
        if (!untrusted_tracking_) {
            untrusted_tracking_ = true;
            accel_untrusted_since_ = sample.received_ms;
        }
        if (uint32_t(sample.received_ms - accel_untrusted_since_) >= c_.max_accel_untrusted_ms)
            faults |= AttitudeAccelUntrusted;
    } else
        untrusted_tracking_ = false;
    if (!have_attitude_ && s_.accel_trusted) {
        for (unsigned i = 0; i < 3; ++i)
            s_.up_body[i] = s_.accel_body_m_s2[i] / s_.accel_norm_m_s2;
        have_attitude_ = true;
    } else if (have_attitude_ && timing) {
        const auto &w = s_.gyro_body_rad_s;
        const float x = s_.up_body[0], y = s_.up_body[1], z = s_.up_body[2];
        s_.up_body[0] += (w[2] * y - w[1] * z) * dt;
        s_.up_body[1] += (w[0] * z - w[2] * x) * dt;
        s_.up_body[2] += (w[1] * x - w[0] * y) * dt;
        if (s_.accel_trusted) {
            const float gain = dt / (c_.correction_tau_s + dt);
            const float dot =
                (s_.accel_body_m_s2[0] * s_.up_body[0] + s_.accel_body_m_s2[1] * s_.up_body[1] +
                 s_.accel_body_m_s2[2] * s_.up_body[2]) /
                s_.accel_norm_m_s2;
            // Normalized linear blending cannot escape exactly antipodal vectors.
            // A plausible-gravity inversion therefore reanchors conservatively;
            // the separate persistence timer still guards the rollover output.
            if (dot < -.95f)
                for (unsigned i = 0; i < 3; ++i)
                    s_.up_body[i] = s_.accel_body_m_s2[i] / s_.accel_norm_m_s2;
            else
                for (unsigned i = 0; i < 3; ++i)
                    s_.up_body[i] +=
                        (s_.accel_body_m_s2[i] / s_.accel_norm_m_s2 - s_.up_body[i]) * gain;
        }
        const float length = norm(s_.up_body);
        if (!std::isfinite(length) || length < .01f) {
            have_attitude_ = false;
            faults |= AttitudeSampleInvalid;
        } else
            for (float &x : s_.up_body)
                x /= length;
    }
    s_.tilt_deg = std::acos(clamp(s_.up_body[2], -1, 1)) * 57.29577951308232f;
    s_.faults = faults;
    s_.valid = faults == 0 && s_.calibrated && have_attitude_;
    if (!s_.valid) {
        trip_tracking_ = release_tracking_ = false;
        return s_;
    }
    if (!s_.tipped) {
        if (s_.tilt_deg >= c_.trip_tilt_deg) {
            if (!trip_tracking_) {
                trip_tracking_ = true;
                trip_since_ = sample.received_ms;
            }
            if (uint32_t(sample.received_ms - trip_since_) >= c_.trip_ms) {
                s_.tipped = true;
                release_tracking_ = false;
            }
        } else
            trip_tracking_ = false;
    } else {
        if (s_.tilt_deg <= c_.release_tilt_deg) {
            if (!release_tracking_) {
                release_tracking_ = true;
                release_since_ = sample.received_ms;
            }
            if (uint32_t(sample.received_ms - release_since_) >= c_.release_ms) {
                s_.tipped = false;
                trip_tracking_ = false;
            }
        } else
            release_tracking_ = false;
    }
    return s_;
}
} // namespace engineer
