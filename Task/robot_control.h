#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
#define ROBOT_CONTROL_API_VERSION 0x0202u
// V2.1接口不兼容旧Climb字段；底盘需按新结构接入。
enum RobotModeCode { ROBOT_SAFE,ROBOT_DRIVE,ROBOT_GRAB,ROBOT_STANDBY };
enum RobotDriveGear { ROBOT_GEAR_SLOW,ROBOT_GEAR_FAST };
typedef struct {
    uint8_t chassis_stopped;
    uint8_t arm_allows_drive,arm_allows_fast;
    uint8_t drive_yaw_allowed;
    // 来自已核定的姿态包络，单位deg/deg/s；allowed=0时不使用三个数值。
    float drive_yaw_min_deg,drive_yaw_max_deg,drive_yaw_max_speed_deg_s;
} RobotInterlocks;
typedef struct {
    uint8_t ready,mode,block,active,wait_neutral,remote_online;
    uint8_t emergency_stop,stop_chassis,longitudinal_brake,reset_chassis_motion;
    uint8_t yaw_follow; // V2.1固定0，底盘不得根据yaw_relative_deg自行跟随。
    uint8_t fast_requested,drive_gear,drive_yaw_limited,protection_disabled;
    uint8_t joint_mask; // 0x1f整车 / 0x1e四轴；台架始终发布底盘停控
    uint32_t updated_ms,motion_epoch,remote_age_ms,interlock_age_ms;
    uint32_t protection_current_reasons,protection_latched_reasons,protection_last_trip_ms;
    // 均为[-1,1]请求：纵向正=前进加速，turn正=逆时针，非m/s或rad/s。
    float longitudinal_accel_request,turn_rate_request,yaw_relative_deg;
    float target_deg[5],gripper_effort_limit;
} RobotControlStatus;
// 状态符号只读；任务间仍使用Get接口。唯一可写调试变量每轮取走并清零。
extern RobotControlStatus g_robot_status;
enum RobotDebugCommand {
    ROBOT_DEBUG_IDLE, ROBOT_DEBUG_STOP, ROBOT_DEBUG_CAPTURE_ZERO,
    ROBOT_DEBUG_CLEAR_FAULTS, ROBOT_DEBUG_REQUEST_CONTROL
};
extern volatile uint32_t g_robot_debug_command;
// ZERO/CLEAR同时请求关控，已使能时核心返回Busy；绝不自动校零/自动开控。
// 单C板：底盘/安全管理约每20ms更新真实状态；100ms过期禁止运动。
// 非法输入立即使互锁无效，不继续沿用旧许可。无底盘任务时不要伪造全部为1。
int Robot_SetInterlocks(const RobotInterlocks *state);
// 回中/停车、反馈和校零就绪后显式请求一次；待机中忽略、不排队等待以后开控。
int Robot_RequestControl(void);
void Robot_StopControl(void);
int Robot_GetControlStatus(RobotControlStatus *status);
// 消费规则：ready/active失效、emergency_stop或(now-updated_ms)过期 -> 底盘自行执行安全停机。
// stop_chassis -> 受控停车；longitudinal_brake -> 即使原地转向也将纵向速度制动到0。
// reset_chassis_motion为1或motion_epoch变化 -> 清旧积分目标/历史，再按实测斜坡停车。
// DriveGear为许可后的档位，并不实现速度限制；底盘负责轮速/功率/加速度及停机输出。
#ifdef __cplusplus
}
#endif
