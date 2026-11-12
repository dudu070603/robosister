#include "Vehicle/vehicle_state.h"
#include <cmath>

namespace engineer {

VehicleState evaluate_vehicle(uint32_t now, const VehicleConfig &cfg, const VehicleInput &input,
                              const VehicleTiming &timing) {
    const auto &ch = input.chassis;
    const auto &arm = input.arm;
    const auto &imu = input.imu;
    VehicleState state{};
    state.updated_ms = now;

    const bool configuration_valid =
        cfg.posture_confirmed && cfg.status_timeout_ms > 0 &&
        timing.wheel_feedback_timeout_ms > 0 && timing.arm_feedback_timeout_ms > 0 &&
        timing.imu_sample_timeout_ms > 0 && std::isfinite(cfg.drive_pitch_deg) &&
        cfg.drive_pitch_deg > 0 && std::isfinite(cfg.drive_roll_deg) && cfg.drive_roll_deg > 0 &&
        std::isfinite(cfg.drive_yaw_deg) && cfg.drive_yaw_deg > 0 &&
        std::isfinite(cfg.drive_yaw_speed_deg_s) && cfg.drive_yaw_speed_deg_s > 0 &&
        std::isfinite(input.yaw_half_range_deg) && input.yaw_half_range_deg > 0;
    if (!configuration_valid || !ch.hardware_confirmed || !ch.power_configuration_valid ||
        !ch.power_confirmed) {
        state.reasons |= VehicleConfiguration;
    }

    const bool chassis_fresh = ch.ready && uint32_t(now - ch.updated_ms) <= cfg.status_timeout_ms;
    if (!chassis_fresh) {
        state.reasons |= VehicleChassisStale;
    }
    if (!ch.configuration_valid || ch.faults) {
        state.reasons |= VehicleWheelFault;
    }
    for (unsigned i = 0; i < 2; ++i) {
        if (!ch.feedback_received[i] ||
            uint32_t(now - ch.feedback_ms[i]) > timing.wheel_feedback_timeout_ms) {
            state.reasons |= VehicleWheelFault;
        }
    }

    const uint32_t arm_publication_age = now - arm.updated_ms;
    const bool arm_fresh = arm.ready && arm_publication_age <= cfg.status_timeout_ms;
    const bool arm_feedback_fresh =
        arm.all_feedback_received && uint64_t(arm.oldest_feedback_age_ms) + arm_publication_age <=
                                         timing.arm_feedback_timeout_ms;
    if (!arm_fresh || !arm_feedback_fresh) {
        state.reasons |= VehicleArmStale;
    }
    if (arm.faults || !arm.configuration_valid) {
        state.reasons |= VehicleArmFault;
    }

    if (!imu.ready || !imu.configuration_valid || !imu.valid || !imu.confirmed || !imu.calibrated ||
        imu.faults || uint32_t(now - imu.updated_ms) > cfg.status_timeout_ms ||
        uint32_t(now - imu.received_ms) > timing.imu_sample_timeout_ms) {
        state.reasons |= VehicleImuInvalid;
    }
    if (imu.tipped) {
        state.reasons |= VehicleTipped;
    }
    if (!ch.power_session_allowed) {
        state.reasons |= VehicleEnergyInhibited;
    }

    state.healthy = state.reasons == 0;
    state.locks.chassis_stopped =
        chassis_fresh && ch.wheels_stationary && !(state.reasons & VehicleWheelFault);
    const bool posture = arm_fresh && arm_feedback_fresh && !arm.faults &&
                         arm.configuration_valid && arm.all_online && arm.all_calibrated &&
                         arm.all_positions_finite && std::isfinite(arm.yaw_deg) &&
                         std::isfinite(arm.big_pitch_deg) && std::isfinite(arm.small_pitch_deg) &&
                         std::isfinite(arm.roll_deg) &&
                         std::fabs(arm.big_pitch_deg) <= cfg.drive_pitch_deg &&
                         std::fabs(arm.small_pitch_deg) <= cfg.drive_pitch_deg &&
                         std::fabs(arm.roll_deg) <= cfg.drive_roll_deg;
    const float half = std::fmin(input.yaw_half_range_deg, cfg.drive_yaw_deg);
    state.locks.arm_allows_drive =
        state.healthy && posture && !ch.power_stop_requested && std::fabs(arm.yaw_deg) <= half;
    state.locks.arm_allows_fast = false;

    // Preserve the known interval through power stopping and actual-Yaw
    // violations. The application owner retains its boundary while braking.
    state.locks.drive_yaw_allowed = state.healthy && posture;
    if (state.locks.drive_yaw_allowed) {
        state.locks.drive_yaw_min_deg = -half;
        state.locks.drive_yaw_max_deg = half;
        state.locks.drive_yaw_max_speed_deg_s = cfg.drive_yaw_speed_deg_s;
    }
    return state;
}

} // namespace engineer
