#pragma once
#include "Arm/arm_control.h"
#include "dr16_bsp.h"

namespace engineer {
enum class RobotMode : uint8_t { Safe, Drive, Grab, Standby };
enum class DriveGear : uint8_t { Slow, Fast };
enum class OperatorBlock : uint8_t {
    None,
    RemoteOffline,
    InterlockOffline,
    WaitNeutral,
    NeedArm,
    ArmUnavailable,
    WaitChassisStop,
    PostureDenied,
    Standby,
    InvalidConfig,
    Timing,
    DriveYawDenied,
    InvalidInterlock
};
struct OperatorConfig {
    bool mapping_confirmed; // 实机确认摇杆通道/拨杆左右/正向后设true
    uint8_t left_switch_index, right_switch_index;
    uint8_t left_y_channel, left_x_channel, right_y_channel, right_x_channel;
    int8_t axis_direction[4]; // 左Y/左X/右Y/右X的符号
    float deadband;           // 归一化[-1,1]上的死区
    uint32_t remote_timeout_ms, interlock_timeout_ms, neutral_ms;
    float gripper_open_deg, gripper_closed_deg;
    float gripper_hold_current; // C610原码，mA
    uint32_t gripper_close_timeout_ms;
};
extern const OperatorConfig kOperatorConfig;
// Shared by normal control and read-only remote diagnostics. Invalid parameters
// return zero; direction and deadband use the same normalized control convention.
float remote_axis_value(const DR16Frame &, unsigned channel, int direction, float deadband);
struct RemoteInput {
    DR16Frame frame;
    uint32_t received_ms;
    bool valid;
};
struct Interlocks {
    bool valid = false;
    uint32_t received_ms = 0;
    bool chassis_stopped = false;
    bool arm_allows_drive = false, arm_allows_fast = false;
    // 安全管理按实测姿态提供行驶yaw包络；未提供时只禁止行驶中的yaw操作。
    bool drive_yaw_allowed = false;
    float drive_yaw_min_deg = 0, drive_yaw_max_deg = 0, drive_yaw_max_speed_deg_s = 0;
};
struct OperatorOutput {
    RobotMode mode = RobotMode::Safe;
    OperatorBlock block = OperatorBlock::RemoteOffline;
    bool active = false, wait_neutral = true, request_enable = false, disable_arm = true;
    bool stop_chassis = true, emergency_stop = true;
    bool longitudinal_brake = true, reset_chassis_motion = true, fast_requested = false;
    bool drive_yaw_limited = false;
    DriveGear drive_gear = DriveGear::Slow;
    uint32_t motion_epoch = 0;            // 底盘比较代号，不能漏掉两次读取之间发生的模式切换/关控。
    float longitudinal_speed_request = 0; // 归一化速度请求，底盘执行速度映射/斜坡/回中制动
    float turn_rate_request = 0;          // [-1,1]角速度意图，逆时针为正；不是rad/s，不随倒车翻转。
    float yaw_relative_deg = 0;
    float target_deg[kJointCount]{};
    float gripper_effort_limit = 0;
};

// 与HAL/RTOS无关。唯一的模式解释器；底盘读取输出，不自行重解同一拨杆。
class ArmOperator {
  public:
    ArmOperator(const Config &arm, const OperatorConfig &config);
    void set_yaw_limits(const YawRuntimeLimits &limits);
    // Owner-only explicit return from tuning, after neutral/vehicle health checks.
    // Captures targets and resets timing; step still validates all normal guards.
    // This does not enable any drive: partially enabled arms require arm_request.
    void resume_from_tuning(const Status &, const RemoteInput &, uint32_t now_ms);
    OperatorOutput step(const RemoteInput &, const Interlocks &, const Status &, uint32_t now_ms,
                        bool arm_request, bool stop_request, bool parked_tuning = false);
    uint32_t enable_result(bool ok) {
        if (!ok) {
            active_ = false;
            waiting_ = true;
            neutral_tracking_ = false;
            last_right_ = 0;
            bench_up_seen_ = false;
            bench_arm_pending_ = false;
            yaw_was_allowed_ = false;
            ++motion_epoch_;
        }
        return motion_epoch_;
    }

  private:
    Config arm_;
    YawRuntimeLimits yaw_limits_{};
    OperatorConfig config_;
    bool valid_ = false, active_ = false, waiting_ = true, neutral_tracking_ = false;
    bool have_step_ = false, safe_latched_ = true, permission_blocked_ = false;
    bool yaw_was_allowed_ = false, close_tracking_ = false, close_limited_ = false;
    bool bench_up_seen_ = false, bench_arm_pending_ = false;
    RobotMode mode_ = RobotMode::Safe;
    uint8_t last_right_ = 0;
    uint32_t neutral_since_ = 0, last_step_ = 0, motion_epoch_ = 0, close_since_ = 0;
    float targets_[kJointCount]{};
    void freeze(const Status &);
    void wait_for_neutral(const Status &);
};
} // namespace engineer
