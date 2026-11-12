#include "Vehicle/vehicle_state.h"
#include "Power/chassis_power.h"
#include "Imu/attitude_estimator.h"
#include "Arm/arm_control.h"
#include "Chassis/chassis_config.h"
#include "vehicle_state_task.h"
#include "task_ins.h"
#include <cmath>
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x))                                                                                  \
            return __LINE__;                                                                       \
    } while (0)
namespace {
bool near(float x, float y, float error = .001f) {
    return std::fabs(x - y) < error;
}
PowerMeterStatus meter(uint32_t now, float energy = 400, float power = 10) {
    PowerMeterStatus m{};
    m.frames = 1;
    m.received_ms = now;
    m.energy_j = energy;
    m.power_w = power;
    return m;
}
AttitudeSample sample(uint32_t ms, uint32_t frame, float ax = 0, float ay = 0,
                      float az = 9.80665f) {
    AttitudeSample s{};
    s.received_ms = ms;
    s.frame = frame;
    s.temperature_c = 25;
    s.accel_m_s2[0] = ax;
    s.accel_m_s2[1] = ay;
    s.accel_m_s2[2] = az;
    return s;
}
AttitudeConfig attitude_config() {
    AttitudeConfig c{};
    c.installation_confirmed = c.thresholds_confirmed = true;
    c.calibration_samples = 10;
    c.calibration_min_ms = 36;
    c.correction_tau_s = .02f;
    return c;
}
void sources(uint32_t now, VehicleChassisInput &ch, VehicleArmInput &arm, VehicleImuInput &imu) {
    ch = {};
    arm = {};
    imu = {};
    ch.ready = ch.configuration_valid = ch.wheels_stationary = ch.power_session_allowed = true;
    ch.hardware_confirmed = ch.power_configuration_valid = ch.power_confirmed = true;
    ch.updated_ms = now;
    for (unsigned i = 0; i < 2; ++i) {
        ch.feedback_received[i] = true;
        ch.feedback_ms[i] = now;
    }
    arm.ready = arm.configuration_valid = true;
    arm.updated_ms = now;
    arm.all_feedback_received = arm.all_online = arm.all_calibrated = arm.all_positions_finite =
        true;
    imu.ready = imu.configuration_valid = imu.valid = imu.calibrated = imu.confirmed = true;
    imu.updated_ms = imu.received_ms = now;
}
VehicleState evaluate(uint32_t now, const VehicleConfig &cfg, const VehicleChassisInput &ch,
                      const VehicleArmInput &arm, const VehicleImuInput &imu, float yaw_half) {
    const VehicleTiming timing{kChassisConfig.feedback_timeout_ms, kArmConfig.feedback_timeout_ms,
                               kAttitudeConfig.max_sample_gap_ms};
    return evaluate_vehicle(now, cfg, VehicleInput{ch, arm, imu, yaw_half}, timing);
}
} // namespace
int test_vehicle() {
    { // No first meter frame and unconfirmed parameters cannot permit a session.
        ChassisPower defaults;
        CHECK(!defaults.step(0, meter(0), false, true).session_allowed);
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        ChassisPower p(cfg);
        PowerMeterStatus empty{};
        CHECK(!p.step(0, empty, false, true).session_allowed &&
              p.status().reason == PowerBlock::NoMeter);
        CHECK(p.step(4, meter(4), false, true).session_allowed &&
              p.status().phase == PowerPhase::Ready);
        CHECK(p.step(8, meter(8), true, false).phase == PowerPhase::Driving);
        const float candidate[2] = {2000, -2000}, rpm[2] = {100, 100};
        float out[2];
        p.limit(candidate, rpm, out);
        CHECK(out[0] <= cfg.drive_current_ma && out[1] >= -cfg.braking_current_ma);
        CHECK(out[0] > 0 && out[1] < 0);
        p.step(12, meter(12, 400, 80), true, false);
        p.limit(candidate, rpm, out);
        CHECK(near(out[0], cfg.drive_current_ma * .5f) && near(out[1], -cfg.braking_current_ma));
        p.step(16, meter(16, 150), true, false);
        CHECK(near(p.status().speed_scale, .5f));
        const float nan[2] = {100, std::numeric_limits<float>::quiet_NaN()};
        p.limit(nan, rpm, out);
        CHECK(out[0] == 0 && out[1] == 0);
    }
    { // Low energy starts bounded controlled braking, and recovery never cancels the stop.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        ChassisPower p(cfg);
        p.step(0, meter(0), false, true);
        p.step(4, meter(4), true, false);
        CHECK(p.step(8, meter(8, 90), true, false).phase == PowerPhase::StopLowEnergy);
        CHECK(p.status().stop_requested && p.status().session_allowed);
        const float candidate[2] = {1000, -1000}, rpm[2] = {100, 100};
        float out[2];
        p.limit(candidate, rpm, out);
        CHECK(out[0] == 0 && out[1] == -cfg.braking_current_ma);
        const float reverse[2] = {-100, -100};
        p.limit(candidate, reverse, out);
        CHECK(out[0] == cfg.braking_current_ma && out[1] == 0);
        CHECK(p.step(12, meter(12), true, false).phase == PowerPhase::StopLowEnergy);
        CHECK(p.step(16, meter(16), true, true).phase == PowerPhase::Inhibited);
        CHECK(p.status().inhibited && !p.status().session_allowed);
        CHECK(!p.clear(20, meter(20), false));
        CHECK(!p.clear(20, meter(20, 200), true));
        CHECK(!p.clear(221, meter(0), true));
        CHECK(p.clear(224, meter(224), true));
        CHECK(!p.status().session_allowed && !p.status().inhibited);
        CHECK(!p.status().stop_requested && p.status().speed_scale == 0 &&
              p.status().drive_scale == 0);
        CHECK(p.step(228, meter(228), false, true).phase == PowerPhase::Ready);
    }
    { // Meter loss has a finite stopping window, with no infinite brake exception.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        cfg.timeout_ms = 20;
        cfg.stopping_timeout_ms = 100;
        ChassisPower p(cfg);
        p.step(0, meter(0), false, true);
        p.step(4, meter(4), true, false);
        auto stale = meter(4);
        CHECK(p.step(28, stale, true, false).phase == PowerPhase::StopFeedback);
        CHECK(p.status().session_allowed && !p.status().meter_online && p.status().stop_requested);
        CHECK(p.step(124, stale, true, false).session_allowed);
        CHECK(p.step(128, stale, true, false).phase == PowerPhase::Inhibited);
        CHECK(p.status().reason == PowerBlock::StopTimeout && !p.status().session_allowed);
        float out[2];
        const float c[2] = {-100, -100}, rpm[2] = {100, 100};
        p.limit(c, rpm, out);
        CHECK(out[0] == 0 && out[1] == 0);
        CHECK(!p.step(132, meter(132), true, false).session_allowed);
        auto invalid = cfg;
        invalid.budget_w = 50;
        ChassisPower unsafe(invalid);
        CHECK(!unsafe.status().configuration_valid);
        ChassisPower wrap(cfg);
        wrap.step(0xfffffff0u, meter(0xfffffff0u), false, true);
        CHECK(wrap.step(0, meter(0), true, false).session_allowed);
    }
    { // Meter loss immediately after the first granted cycle still permits bounded braking.
        auto cfg = kChassisPowerConfig;
        cfg.confirmed = true;
        ChassisPower p(cfg);
        CHECK(p.step(0, meter(0), false, true).session_allowed);
        PowerMeterStatus absent{};
        CHECK(p.step(4, absent, true, false).phase == PowerPhase::StopFeedback);
        CHECK(p.status().stop_requested && p.status().session_allowed);
        CHECK(p.step(8, absent, false, false).phase == PowerPhase::Inhibited);
        CHECK(p.status().reason == PowerBlock::ExternalStop && !p.status().session_allowed);
        CHECK(p.clear(12, meter(12), true));
        CHECK(!p.status().session_allowed && !p.status().stop_requested && !p.status().inhibited);
        CHECK(p.status().speed_scale == 0 && p.status().drive_scale == 0);
        float out[2];
        const float candidate[2] = {-500, -500}, rpm[2] = {100, 100};
        p.limit(candidate, rpm, out);
        CHECK(out[0] == 0 && out[1] == 0); // Clearing never emits a deferred brake/start.
    }
    { // Real healthy snapshots permit slow drive before active; no startup deadlock.
        VehicleConfig cfg{};
        cfg.posture_confirmed = true;
        VehicleChassisInput ch{};
        VehicleArmInput arm{};
        VehicleImuInput imu{};
        sources(100, ch, arm, imu);
        auto s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK(s.healthy && s.locks.chassis_stopped && s.locks.arm_allows_drive);
        CHECK(!s.locks.arm_allows_fast && s.locks.drive_yaw_allowed);
        CHECK(s.locks.drive_yaw_min_deg == -10 && s.locks.drive_yaw_max_deg == 10);
        CHECK(s.locks.drive_yaw_max_speed_deg_s == cfg.drive_yaw_speed_deg_s);
        s = evaluate(104, cfg, ch, arm, imu, 50);
        CHECK(s.locks.drive_yaw_max_deg == cfg.drive_yaw_deg);
        for (unsigned bad = 0; bad < 13; ++bad) {
            sources(100, ch, arm, imu);
            auto conf = cfg;
            if (bad == 0)
                ch.updated_ms = 0;
            if (bad == 1)
                arm.updated_ms = 0;
            if (bad == 2)
                imu.updated_ms = 0;
            if (bad == 3)
                imu.valid = 0;
            if (bad == 4)
                imu.tipped = 1;
            if (bad == 5)
                ch.feedback_received[0] = false;
            if (bad == 6)
                ch.feedback_ms[1] = 0;
            if (bad == 7)
                ch.power_session_allowed = 0;
            if (bad == 8)
                conf.posture_confirmed = false;
            if (bad == 9)
                ch.hardware_confirmed = 0;
            if (bad == 10)
                ch.power_configuration_valid = 0;
            if (bad == 11)
                ch.power_confirmed = 0;
            if (bad == 12)
                imu.configuration_valid = 0;
            s = evaluate(100, conf, ch, arm, imu, 10);
            CHECK(!s.healthy && s.reasons && !s.locks.arm_allows_drive && !s.locks.arm_allows_fast);
        }
        sources(100, ch, arm, imu);
        arm.big_pitch_deg = cfg.drive_pitch_deg + 1;
        CHECK(!evaluate(100, cfg, ch, arm, imu, 10).locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        arm.all_positions_finite = false;
        CHECK(!evaluate(100, cfg, ch, arm, imu, 10).locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        arm.all_calibrated = false;
        CHECK(!evaluate(100, cfg, ch, arm, imu, 10).locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        ch.power_stop_requested = 1;
        s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK(s.healthy && !s.locks.arm_allows_drive && s.locks.drive_yaw_allowed);
        CHECK(s.locks.drive_yaw_min_deg == -10 && s.locks.drive_yaw_max_deg == 10);
        sources(100, ch, arm, imu);
        arm.yaw_deg = 11;
        s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK(s.locks.drive_yaw_allowed && !s.locks.arm_allows_drive);
        CHECK(s.locks.drive_yaw_min_deg == -10 && s.locks.drive_yaw_max_deg == 10);
        sources(100, ch, arm, imu);
        arm.yaw_deg = 21;
        s = evaluate(100, cfg, ch, arm, imu, 50);
        CHECK(s.healthy && s.locks.drive_yaw_allowed && !s.locks.arm_allows_drive);
        CHECK(s.locks.drive_yaw_min_deg == -20 && s.locks.drive_yaw_max_deg == 20);
        sources(100, ch, arm, imu);
        arm.faults = 1;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10).reasons & VehicleArmFault);
        sources(100, ch, arm, imu);
        arm.updated_ms = 80;
        arm.oldest_feedback_age_ms = 40;
        s = evaluate(100, cfg, ch, arm, imu, 10);
        CHECK((s.reasons & VehicleArmStale) && !s.healthy && !s.locks.arm_allows_drive);
        sources(100, ch, arm, imu);
        arm.updated_ms = 80;
        arm.oldest_feedback_age_ms = 30;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10)
                  .locks.arm_allows_drive); // Exact combined source-age boundary.
        sources(100, ch, arm, imu);
        arm.updated_ms = 80;
        arm.oldest_feedback_age_ms = 0xfffffff0u;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10).reasons &
              VehicleArmStale); // No uint32_t addition wrap.
        sources(100, ch, arm, imu);
        arm.all_feedback_received = false;
        CHECK(evaluate(100, cfg, ch, arm, imu, 10).reasons & VehicleArmStale);
        sources(100, ch, arm, imu);
        CHECK(!evaluate(100, cfg, ch, arm, imu, std::numeric_limits<float>::quiet_NaN()).healthy);
        // Publishing a fresh snapshot does not make stale wheel/sensor timestamps fresh.
        ch.updated_ms = 120;
        arm.updated_ms = 120;
        imu.updated_ms = 120;
        imu.received_ms = 90;
        CHECK(evaluate(120, cfg, ch, arm, imu, 10).reasons & VehicleImuInvalid);
    }
    { // The task bridge retains each actual cause and rejects malformed health.
        VehicleHealth health{100, 0, 1, 1};
        const uint32_t timeout = kVehicleConfig.status_timeout_ms;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == 0);
        CHECK(Vehicle_ProtectionFromHealth(100, nullptr, timeout) == PROTECT_VEHICLE_STATE);
        CHECK(Vehicle_ProtectionFromHealth(160, &health, timeout) == 0);
        CHECK(Vehicle_ProtectionFromHealth(161, &health, timeout) == PROTECT_VEHICLE_STATE);
        const uint32_t faults[] = {VehicleConfiguration,   VehicleChassisStale, VehicleArmStale,
                                   VehicleImuInvalid,      VehicleTipped,       VehicleWheelFault,
                                   VehicleEnergyInhibited, VehicleArmFault};
        const uint32_t expected[] = {
            PROTECT_VEHICLE_STATE, PROTECT_CHASSIS, PROTECT_VEHICLE_STATE, PROTECT_IMU,
            PROTECT_TIPPED,        PROTECT_CHASSIS, PROTECT_POWER,         PROTECT_VEHICLE_STATE};
        for (unsigned i = 0; i < 8; ++i) {
            health.reasons = faults[i];
            health.healthy = 0;
            CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == expected[i]);
        }
        health.reasons = 0xff;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) ==
              (PROTECT_VEHICLE_STATE | PROTECT_CHASSIS | PROTECT_IMU | PROTECT_TIPPED |
               PROTECT_POWER));
        health.reasons = 0x100;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health.reasons = 0;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health.healthy = 2;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health.healthy = 1;
        health.ready = 0;
        CHECK(Vehicle_ProtectionFromHealth(100, &health, timeout) == PROTECT_VEHICLE_STATE);
        health = {0xfffffff0u, 0, 1, 1};
        CHECK(Vehicle_ProtectionFromHealth(16, &health, timeout) == 0);
    }
    { // Permission expires at the original source deadline, including after a revoke.
        InsCalibrationPermission permission{};
        CHECK(!INS_CalibrationPermissionValid(&permission, 0, 0, 0));
        CHECK(!INS_CalibrationPermissionValid(nullptr, 0, 0, 0));
        permission = {100, 90, 30, 1, 1};
        CHECK(INS_CalibrationPermissionValid(&permission, 130, 0, 0));
        CHECK(!INS_CalibrationPermissionValid(&permission, 131, 0, 0));
        CHECK(!INS_CalibrationPermissionValid(&permission, 99, 0, 0));
        permission.valid_for_ms = 31;
        CHECK(!INS_CalibrationPermissionValid(&permission, 100, 0, 0));
        permission = {100, 50, 0, 1, 1};
        CHECK(!INS_CalibrationPermissionValid(&permission, 100, 0, 0));
        permission = {120, 100, 20, 1, 1};
        CHECK(!INS_CalibrationPermissionValid(&permission, 120, 1, 100));
        permission.oldest_source_updated_ms = 101;
        permission.valid_for_ms = 21;
        CHECK(INS_CalibrationPermissionValid(&permission, 120, 1, 100));
        permission.allowed = 0;
        CHECK(!INS_CalibrationPermissionValid(&permission, 120, 1, 100));
        permission = {0xfffffff0u, 0xfffffff0u, 40, 1, 1};
        CHECK(INS_CalibrationPermissionValid(&permission, 16, 1, 0xffffffe0u));
        CHECK(!INS_CalibrationPermissionValid(&permission, 25, 0, 0));

        auto cfg = attitude_config();
        AttitudeEstimator attitude(cfg);
        permission = {0, 0, 32, 1, 1};
        for (unsigned n = 0; n < 9; ++n) {
            const uint32_t now = n * 4;
            attitude.step(sample(now, n + 1), .004f,
                          INS_CalibrationPermissionValid(&permission, now, 0, 0));
        }
        CHECK(attitude.status().calibration_count == 9 && !attitude.status().calibrated);
        attitude.step(sample(36, 10), .004f, INS_CalibrationPermissionValid(&permission, 36, 0, 0));
        CHECK(attitude.status().calibration_count == 0 && !attitude.status().calibrated);
        permission = {40, 40, 40, 1, 1};
        attitude.step(sample(40, 11), .004f,
                      INS_CalibrationPermissionValid(&permission, 40, 1, 36));
        CHECK(attitude.status().calibration_count == 1 && !attitude.status().calibrated);
    }
    { // Static calibration is gated by all outputs disabled and enough continuous samples.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 20; ++n)
            a.step(sample(n * 4, n + 1), .004f, false);
        CHECK(!a.status().calibrated && !a.status().valid);
        a.restart_calibration();
        for (unsigned n = 20; n < 29; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        CHECK(!a.status().calibrated);
        a.step(sample(116, 30), .004f, true);
        CHECK(a.status().calibrated && a.status().valid && a.status().tilt_deg < .01f);
        AttitudeEstimator defaults;
        for (unsigned n = 0; n < 600; ++n)
            defaults.step(sample(n * 4, n + 1), .004f, true);
        CHECK(defaults.status().configuration_valid && !defaults.status().confirmed &&
              !defaults.status().valid);
        auto reflected = cfg;
        reflected.sensor_sign[0] = -1;
        AttitudeEstimator bad(reflected);
        CHECK(!bad.status().configuration_valid);
        reflected.sensor_axis[0] = 1;
        reflected.sensor_axis[1] = 0;
        AttitudeEstimator right_handed(reflected);
        CHECK(right_handed.status().configuration_valid);
    }
    { // Flip/release persistence and hysteresis; a brief tilt is not a latched tip.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 10; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        unsigned n = 10;
        for (; n < 20; ++n)
            a.step(sample(n * 4, n + 1, 0, 0, -9.80665f), .004f, false);
        CHECK(!a.status().tipped); // less than trip_ms
        for (; n < 80; ++n)
            a.step(sample(n * 4, n + 1, 0, 0, -9.80665f), .004f, false);
        CHECK(a.status().tipped && a.status().tilt_deg > 179);
        for (; n < 140; ++n)
            a.step(sample(n * 4, n + 1), .004f, false);
        CHECK(a.status().tipped); // release requires 500 ms, not one flat sample.
        for (; n < 260; ++n)
            a.step(sample(n * 4, n + 1), .004f, false);
        CHECK(a.status().valid && !a.status().tipped && a.status().tilt_deg < .1f);
        for (; n < 410; ++n)
            a.step(sample(n * 4, n + 1, 0, 0, 20), .004f, false);
        CHECK(!a.status().valid && (a.status().faults & AttitudeAccelUntrusted));
        a.step(sample(n * 4, n + 1), .004f, false);
        ++n;
        CHECK(a.status().valid);
        auto bad = sample(n * 4, n + 1);
        bad.accel_m_s2[0] = std::numeric_limits<float>::quiet_NaN();
        a.step(bad, .004f, false);
        ++n;
        CHECK(!a.status().valid && (a.status().faults & AttitudeSampleInvalid));
        a.step(sample(n * 4, n + 1), .004f, false);
        ++n;
        a.step(sample(n * 4 + 100, n + 1), .1f, false);
        CHECK(!a.status().valid && (a.status().faults & AttitudeTiming));
    }
    { // Any invalid sample interrupts a continuous static-calibration window.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 9; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        CHECK(a.status().calibration_count == 9 && !a.status().calibrated);
        auto invalid = sample(36, 10);
        invalid.gyro_rad_s[0] = std::numeric_limits<float>::quiet_NaN();
        a.step(invalid, .004f, true);
        CHECK(!a.status().valid && a.status().calibration_count == 0);
        a.step(sample(40, 11), .004f, true);
        CHECK(!a.status().calibrated && a.status().calibration_count == 1);
        a.step(sample(44, 12), .004f, false);
        CHECK(!a.status().calibrated && a.status().calibration_count == 0);
        for (unsigned n = 12; n < 21; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        a.invalidate(84);
        CHECK(a.status().calibration_count == 0);
        a.step(sample(84, 22), .004f, true);
        CHECK(!a.status().calibrated && a.status().calibration_count == 1);
    }
    { // Bias subtraction, duplicate frame rejection and unsigned timestamp rollover.
        auto cfg = attitude_config();
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 10; ++n) {
            auto s = sample(n * 4, n + 1);
            s.gyro_rad_s[0] = .01f;
            a.step(s, .004f, true);
        }
        CHECK(a.status().valid && near(a.status().gyro_bias_rad_s[0], .01f, .00001f));
        a.step(sample(40, 10), .004f, false);
        CHECK(!a.status().valid && (a.status().faults & AttitudeSampleInvalid));
        AttitudeEstimator wrap(cfg);
        for (unsigned n = 0; n < 10; ++n)
            wrap.step(sample(0xfffffff0u + n * 4, n + 1), .004f, true);
        CHECK(wrap.status().valid && !(wrap.status().faults & AttitudeTiming));
        wrap.invalidate(100);
        CHECK(!wrap.status().valid && wrap.status().updated_ms == 100);
    }
    { // The installation map rotates accel and gyro into the same body frame.
        auto cfg = attitude_config();
        cfg.sensor_axis[0] = 1;
        cfg.sensor_axis[1] = 0;
        cfg.sensor_sign[0] = 1;
        cfg.sensor_sign[1] = -1;
        AttitudeEstimator a(cfg);
        for (unsigned n = 0; n < 10; ++n)
            a.step(sample(n * 4, n + 1), .004f, true);
        CHECK(a.status().valid && a.status().configuration_valid);
        auto moving = sample(40, 11);
        moving.gyro_rad_s[0] = 1;
        a.step(moving, .004f, false);
        CHECK(a.status().valid && near(a.status().gyro_body_rad_s[0], 0));
        CHECK(near(a.status().gyro_body_rad_s[1], -1) && near(a.status().gyro_body_rad_s[2], 0));
        CHECK(near(a.status().accel_body_m_s2[2], 9.80665f));
        CHECK(a.status().up_body[0] > 0 && near(a.status().up_body[1], 0));
    }
    return 0;
}
