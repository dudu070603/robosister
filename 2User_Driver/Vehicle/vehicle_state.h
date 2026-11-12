#pragma once
#include <cstdint>

namespace engineer {

struct VehicleConfig {
    bool posture_confirmed = false;
    float drive_pitch_deg = 15;
    float drive_roll_deg = 15;
    float drive_yaw_deg = 20;
    float drive_yaw_speed_deg_s = 3;
    uint32_t status_timeout_ms = 60;
};
inline constexpr VehicleConfig kVehicleConfig{};

// The task adapter supplies these from the owning modules' actual configs.
struct VehicleTiming {
    uint32_t wheel_feedback_timeout_ms = 0;
    uint32_t arm_feedback_timeout_ms = 0;
    uint32_t imu_sample_timeout_ms = 0;
};

struct VehicleChassisInput {
    uint32_t updated_ms = 0;
    uint32_t faults = 0;
    uint32_t feedback_ms[2]{};
    bool feedback_received[2]{};
    bool ready = false;
    bool configuration_valid = false;
    bool hardware_confirmed = false;
    bool power_configuration_valid = false;
    bool power_confirmed = false;
    bool power_session_allowed = false;
    bool power_stop_requested = false;
    bool wheels_stationary = false;
};

struct VehicleArmInput {
    uint32_t updated_ms = 0;
    uint32_t faults = 0;
    // The evaluator adds snapshot age using a wide sum.
    uint32_t oldest_feedback_age_ms = 0;
    float yaw_deg = 0;
    float big_pitch_deg = 0;
    float small_pitch_deg = 0;
    float roll_deg = 0;
    bool ready = false;
    bool configuration_valid = false;
    bool all_feedback_received = false;
    bool all_online = false;
    bool all_calibrated = false;
    bool all_positions_finite = false;
};

struct VehicleImuInput {
    uint32_t updated_ms = 0;
    uint32_t received_ms = 0;
    uint32_t faults = 0;
    bool ready = false;
    bool configuration_valid = false;
    bool valid = false;
    bool confirmed = false;
    bool calibrated = false;
    bool tipped = false;
};

struct VehicleInput {
    VehicleChassisInput chassis;
    VehicleArmInput arm;
    VehicleImuInput imu;
    float yaw_half_range_deg = 0;
};

enum VehicleFault : uint32_t {
    VehicleNoFault = 0,
    VehicleConfiguration = 1,
    VehicleChassisStale = 2,
    VehicleArmStale = 4,
    VehicleImuInvalid = 8,
    VehicleTipped = 16,
    VehicleWheelFault = 32,
    VehicleEnergyInhibited = 64,
    VehicleArmFault = 128
};

struct VehicleInterlocks {
    bool chassis_stopped = false;
    bool arm_allows_drive = false;
    bool arm_allows_fast = false;
    bool drive_yaw_allowed = false;
    float drive_yaw_min_deg = 0;
    float drive_yaw_max_deg = 0;
    float drive_yaw_max_speed_deg_s = 0;
};

struct VehicleState {
    uint32_t updated_ms = 0;
    uint32_t reasons = 0;
    bool healthy = false;
    VehicleInterlocks locks;
};

// Pure safety/posture policy. Acquisition and task protocol adaptation belong
// to the caller; no application snapshot or task global is read here.
VehicleState evaluate_vehicle(uint32_t now, const VehicleConfig &config, const VehicleInput &input,
                              const VehicleTiming &timing);

} // namespace engineer
