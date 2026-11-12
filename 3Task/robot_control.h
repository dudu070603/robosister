#pragma once
#include <stdint.h>
#include "yaw_service.h" // Compatibility: existing callers keep the Yaw request API.
#ifdef __cplusplus
extern "C" {
#endif
enum RobotModeCode { ROBOT_SAFE, ROBOT_DRIVE, ROBOT_GRAB, ROBOT_STANDBY };
enum RobotDriveGear { ROBOT_GEAR_SLOW, ROBOT_GEAR_FAST };
typedef struct {
    uint8_t chassis_stopped;
    uint8_t arm_allows_drive, arm_allows_fast;
    uint8_t drive_yaw_allowed;
    // 来自已核定的姿态包络，单位deg/deg/s；allowed=0时不使用三个数值。
    float drive_yaw_min_deg, drive_yaw_max_deg, drive_yaw_max_speed_deg_s;
} RobotInterlocks;
typedef struct {
    uint8_t ready, mode, block, active, wait_neutral, remote_online;
    uint8_t emergency_stop, stop_chassis, longitudinal_brake, reset_chassis_motion;
    uint8_t fast_requested, drive_gear, drive_yaw_limited, protection_disabled;
    uint8_t independent_stop_latched; // 0档直驱 STOP 锁存；命令4清除。
    uint8_t joint_mask;  // 0x1f整车 / 0x1e四轴；台架始终发布底盘停控
    uint32_t stop_epoch; // Explicit global stop event, including an already-disabled robot.
    uint32_t updated_ms, motion_epoch, remote_age_ms, interlock_age_ms;
    uint32_t protection_current_reasons, protection_latched_reasons, protection_last_trip_ms;
    // 均为[-1,1]速度请求：纵向正=前进，turn正=逆时针，非m/s或rad/s。
    float longitudinal_speed_request, turn_rate_request, yaw_relative_deg;
    float target_deg[5], gripper_effort_limit;
    // Read-only remote observations, also refreshed while disabled/tuning.
    // left Y, left X, right Y, right X: mapped direction + deadband, [-1,1].
    float remote_axis[4];
    uint8_t remote_switch[2]; // Configured left/right switch: UP=1, MID=3, DOWN=2.
} RobotControlStatus;
// 整车诊断快照；底盘控制不读取本结构。原调试命令每轮取走并清零；
// 有序号的Yaw请求由同一owner一次消费，不允许直接写状态控制电机。
extern RobotControlStatus g_robot_status;
enum RobotDebugCommand {
    ROBOT_DEBUG_IDLE,
    ROBOT_DEBUG_STOP,
    ROBOT_DEBUG_CAPTURE_ZERO,
    ROBOT_DEBUG_CLEAR_FAULTS,
    ROBOT_DEBUG_REQUEST_CONTROL
};
extern volatile uint32_t g_robot_debug_command;
// One daily firmware. Remote Drive/Grab/Standby remains independent of these
// owner-consumed FreeMASTER inputs. Switching mode clears motion targets.
enum RobotTuneMode {
    ROBOT_TUNE_NORMAL, ROBOT_TUNE_LEFT_CURRENT, ROBOT_TUNE_RIGHT_CURRENT,
    ROBOT_TUNE_LEFT_SPEED, ROBOT_TUNE_RIGHT_SPEED, ROBOT_TUNE_YAW_SPEED,
    ROBOT_TUNE_YAW_POSITION
};
extern volatile uint32_t robot_tune_mode;
extern volatile float tune_current_ma, tune_speed_rpm, tune_yaw_speed_deg_s, tune_yaw_target_deg;
extern volatile float tune_wheel_kp, tune_wheel_ki, tune_wheel_current_limit_ma;
extern volatile float tune_yaw_position_kp, tune_yaw_velocity_kp, tune_yaw_velocity_ki,
    tune_yaw_current_limit_ma, tune_yaw_half_range_deg;
typedef struct {
    uint32_t updated_ms, mode, epoch, reason;
    uint8_t ready, blocked, held_mask;
    uint32_t yaw_parameter_result;
    float yaw_position_kp, yaw_velocity_kp, yaw_velocity_ki, yaw_current_limit_ma;
} RobotTuneStatus;
extern RobotTuneStatus g_robot_tune_status; // Read-only accepted mode / limits.
// ZERO/CLEAR同时请求关控，已使能时核心返回Busy；绝不自动校零/自动开控。
// 单C板：底盘/安全管理约每20ms更新真实状态；100ms过期禁止运动。
// 非法输入立即使互锁无效，不继续沿用旧许可。无底盘任务时不要伪造全部为1。
int Robot_SetInterlocks(const RobotInterlocks *state);
// 回中/停车、反馈和校零就绪后显式请求一次；待机中忽略、不排队等待以后开控。
int Robot_RequestControl(void);
void Robot_StopControl(void);
int Robot_GetControlStatus(RobotControlStatus *status); // Diagnostics and calibration state only.
#ifdef __cplusplus
}

namespace engineer {
struct ChassisCommand;
}
// One atomic copy of the existing control command; no diagnostic fields or conversion in chassis.
// valid means the producer has published; enabled also includes the full-vehicle build gate.
// The chassis checks updated_ms locally and retains its feedback/power/fault vetoes.
bool Robot_GetChassisCommand(engineer::ChassisCommand *command);
#endif
