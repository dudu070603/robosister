#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum ArmJoint { ARM_YAW, ARM_BIG_PITCH, ARM_SMALL_PITCH, ARM_ROLL, ARM_GRIPPER, ARM_JOINT_COUNT };
#define ARM_JOINT_MASK(joint) ((uint8_t)(1u << (joint)))
#define ARM_ALL_JOINTS ((uint8_t)0x1f)
#define ARM_FOUR_AXIS_JOINTS ((uint8_t)0x1e)
#define ARM_PITCH_PAIR ((uint8_t)0x06)
typedef struct {
    // effort: 6020电流模式/C610为mA，DM为Nm；feedback_effort仍为DJI反馈原码/DM Nm。
    // BigPitch=肩角增量，SmallPitch=肘相对角增量；不是第二DM的驱动角。
    float position_deg, velocity_deg_s, target_deg, reference_deg, effort;
    float drive_position_deg, drive_velocity_deg_s, drive_target_deg, drive_reference_deg;
    float drive_reference_velocity_deg_s;
    float feedback_effort, temperature_c;
    float velocity_target_deg_s;
    uint32_t received, feedback_age_ms, faults;
    uint8_t online, calibrated, enabled, motor_state;
} ArmJointStatus;
typedef struct {
    ArmJointStatus joint[ARM_JOINT_COUNT];
    uint32_t faults, tx_errors[2], rx_dropped;
    uint32_t updated_ms, yaw_limit_version;
    uint8_t enabled_mask, configuration_valid, ready;
    uint8_t last_result; // engineer::Result数值；提交成功只表示入队成功
} ArmStatus;

// 原共享快照的稳定调试符号，外部只读；任务间仍用Arm_GetStatus原子复制。
extern ArmStatus g_arm_status;

// Write the fields first and request_seq last. Rejected requests are consumed once.
// PID/effort changes are RAM-only, disabled-only and never alter hardware/zero/limits.
typedef struct {
    uint32_t joint;
    float position_kp, velocity_kp, velocity_ki, effort_limit;
    uint32_t request_seq;
} ArmTuneRequest;
typedef struct {
    uint32_t consumed_seq, active_seq, result, active_joint;
    float position_kp, velocity_kp, velocity_ki, effort_limit;
} ArmTuneStatus;
extern volatile ArmTuneRequest g_arm_tune_request;
extern ArmTuneStatus g_arm_tune_status; // Read-only; result = engineer::Result.

// 仅供任务上下文调用；返回1=成功入队，实际执行结果看Arm_GetStatus().last_result。
// 含pitch的校零/使能必须成对使用ARM_PITCH_PAIR；手动摆好的联合姿态定义两关节的0增量。
int Arm_CaptureZero(uint8_t mask);
int Arm_Enable(uint8_t mask);
int Arm_Disable(void); // 独立停机标志，不受命令队列满影响
int Arm_ClearFaults(void);
// 单pitch目标可更新；会保留另一关节目标并原子验证a2=q肩+q肘范围。
int Arm_SetTargets(uint8_t mask, const float joint_deg[ARM_JOINT_COUNT]);
int Arm_GetStatus(ArmStatus *status);
void arm_control_task(void *argument);
#ifdef __cplusplus
}
#endif
