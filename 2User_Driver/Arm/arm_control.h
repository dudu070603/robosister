#pragma once
#include <cstddef>
#include <cstdint>

// 上层使用关节端deg/deg/s：BigPitch=肩角增量，SmallPitch=肘相对角增量。
// 两DM的驱动坐标为a1=q肩、a2=q肩+q肘；电机闭环和effort仍属于驱动坐标。
namespace engineer {
constexpr unsigned kJointCount = 5;
constexpr uint8_t kAllJoints = 0x1f;
constexpr uint8_t kFourAxisJoints = 0x1e; // 大/小pitch、roll、夹爪；无yaw
enum Joint : uint8_t { Yaw, BigPitch, SmallPitch, Roll, Gripper };
constexpr uint8_t kPitchPair = (1u << BigPitch) | (1u << SmallPitch);
constexpr int32_t kJogCapturePitchZero = 5, kJogClearFaults = 8, kJogStop = 9;
enum class Motor : uint8_t { Gm6020Voltage, M2006, Dm4310Mit, Gm6020Current };
enum Fault : uint32_t {
    NoFault = 0,
    FeedbackTimeout = 1u << 0,
    CommandTimeout = 1u << 1,
    OverTemperature = 1u << 2,
    MotorError = 1u << 3,
    PositionLimit = 1u << 4,
    PositionLost = 1u << 5,
    TransportError = 1u << 6,
    TimingError = 1u << 7,
    EnableTimeout = 1u << 8
};
enum class Result : uint8_t {
    Ok,
    InvalidArgument,
    ConfigUnconfirmed,
    Busy,
    NotOnline,
    NotCalibrated,
    FaultLatched,
    Moving,
    OutOfRange,
    InterlockOpen
};

struct JointConfig {
    Motor motor;
    uint8_t bus;              // 1=CAN1, 2=CAN2
    uint16_t motor_id;        // DJI: 拨码/电调ID; DM: ESC_ID
    uint16_t feedback_id;     // DM: MST_ID; DJI由型号和motor_id推导
    float feedback_per_joint; // 反馈轴角/驱动坐标角；pitch为绝对连杆驱动角，夹爪为32T轮角
    int8_t direction;         // 驱动坐标正向，+1/-1同时作用于测量和输出
    float min_deg, max_deg;   // 公共关节q范围；第二DM驱动范围另见Config.pitch
    float max_speed_deg_s;
    float max_accel_deg_s2;
    float max_effort;                    // 6020电流模式/C610: mA; DM: Nm; 6020电压模式: 原码
    float position_kp;                   // 驱动坐标位置误差deg -> 驱动速度deg/s
    float velocity_kp, velocity_ki;      // 驱动速度PI -> max_effort对应单位，积分乘实际dt
    float p_max, v_max, t_max;           // DM协议映射范围，必须与上位机读数一致
    float max_temperature_c;             // C610不提供温度，此项不适用于M2006
    float limit_margin_deg = 2.0f;       // 反馈越界锁存余量，同样按此轴角度单位
    float stationary_speed_deg_s = 2.0f; // 手动校零允许的驱动速度阈值
    float initial_effort_limit = -1.0f;  // -1 uses max_effort; same units as max_effort.
};
struct PitchConfig {
    bool mapping_confirmed; // 两DM实例、正向、联合零位及连杆分支实测确认后才设true
    float second_drive_min_deg, second_drive_max_deg; // a2=q肩+q肘的独立保护范围
    float second_drive_max_speed_deg_s, second_drive_max_accel_deg_s2;
    uint32_t max_feedback_skew_ms; // 两反馈时间差门槛；不代表硬件同步采样
};
struct Config {
    JointConfig joint[kJointCount];
    bool hardware_confirmed; // 接线、传动、零位、限位确认后才改true
    uint32_t control_period_ms;
    uint32_t feedback_timeout_ms;
    uint32_t command_timeout_ms;
    uint32_t dji_unwrap_gap_ms;
    PitchConfig pitch;
    uint8_t joint_mask = kAllJoints; // 编译配置只选整车或四轴台架
};
extern const Config kArmConfig;

struct Frame {
    uint8_t bus;
    uint16_t id;
    uint8_t data[8];
};
struct YawRuntimeLimits {
    float min_deg = 0, max_deg = 0, max_speed_deg_s = 0;
    float braking_accel_deg_s2 = 0, braking_margin_deg = 0, max_target_lead_deg = 0;
    uint32_t version = 0;
};
struct JointStatus {
    // effort: 电流模式6020/C610为mA，DM为Nm；6020电压模式为原码。
    float position_deg, velocity_deg_s, target_deg, reference_deg, effort;
    float drive_position_deg, drive_velocity_deg_s, drive_target_deg, drive_reference_deg;
    float drive_reference_velocity_deg_s;
    float motor_effort_feedback; // DJI反馈原码 / DM Nm；不是夹持力
    float temperature_c;
    float velocity_target_deg_s;
    uint32_t received, last_feedback_ms, faults;
    bool online, calibrated, enabled;
    uint8_t motor_state; // DM: 0=失能,1=使能,8~F=故障
};
struct Status {
    JointStatus joint[kJointCount];
    uint8_t enabled_mask;
    uint32_t faults;
    Result last_result;
    bool configuration_valid;
};

struct JointTuning {
    float position_kp, velocity_kp, velocity_ki, effort_limit;
};

// 纯控制逻辑，无HAL依赖；由一个任务拥有，禁止从ISR直接调用。
class ArmControl {
  public:
    explicit ArmControl(const Config &config);
    bool receive(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t len, uint32_t now_ms);
    Result capture_zero(uint8_t mask, uint32_t now_ms);
    // Explicitly add the requested axes; already-enabled axes keep their state.
    Result enable(uint8_t mask, uint32_t now_ms);
    // Owner-only four-axis gain tuning with the chassis stopped.
    Result enable_parked_tuning(uint32_t now_ms);
    void disable();
    // Owner-only mode changes. Neither operation enables an axis; pitch stays paired.
    Result hold(uint8_t mask, uint32_t now_ms);
    Result disable_joints(uint8_t mask);
    Result clear_faults();
    Result set_targets(uint8_t mask, const float positions[kJointCount], uint32_t now_ms);
    void transport_fault();
    Result set_effort_limit(unsigned joint, float limit);
    // Disabled-only RAM tuning. Limits never exceed the startup output cap.
    Result set_tuning(unsigned joint, const JointTuning &tuning, uint32_t now_ms);
    // Runtime tuning requires the affected drive(s) stationary and their targets
    // and references within 0.5 deg of feedback. Other holding axes stay enabled.
    Result set_tuning_runtime(unsigned joint, const JointTuning &tuning, uint32_t now_ms);
    bool get_tuning(unsigned joint, JointTuning &tuning) const;
    Result set_yaw_limits(const YawRuntimeLimits &limits, uint32_t now_ms);
    Result set_yaw_envelope(float min_deg, float max_deg, float speed_deg_s);
    Result set_yaw_test_speed(float speed_deg_s, uint32_t now_ms);
    void clear_yaw_test();
    const YawRuntimeLimits &yaw_limits() const {
        return yaw_limits_;
    }
    // 输出最多5帧：DJI按bus+组ID合帧，DM每台一帧。调用者提供>=5个槽位。
    unsigned step(uint32_t now_ms, Frame out[kJointCount]);
    // Independent four-axis jog owner. Use in place of step(), never alongside it.
    unsigned step_jog(uint32_t now_ms, int32_t request, bool permit, Frame out[kJointCount]);
    const Status &status() const {
        return status_;
    }
    uint16_t feedback_id(unsigned joint) const;

  private:
    struct Runtime {
        uint16_t last_encoder;
        int64_t encoder_sum;
        float motor_position_deg, zero_motor_deg, integral, reference_velocity, effort_limit;
        uint32_t last_command_ms, enable_ms;
        bool have_feedback, dm_was_enabled;
    } runtime_[kJointCount]{};
    Config config_;
    const float startup_effort_cap_[kJointCount];
    float tuning_effort_limit_[kJointCount]{};
    Status status_{};
    uint32_t last_step_ms_ = 0;
    bool have_step_ = false;
    float pitch_direction_[2]{}, pitch_end_[2]{};
    float pitch_speed_ = 0, pitch_speed_limit_ = 0, pitch_accel_limit_ = 0;
    bool pitch_path_active_ = false;
    YawRuntimeLimits yaw_limits_{};
    float yaw_motion_min_ = 0, yaw_motion_max_ = 0, yaw_motion_speed_ = 0, yaw_test_speed_ = 0;
    bool yaw_test_active_ = false;
    int8_t jog_direction_ = 0;
    unsigned jog_axis_ = BigPitch;
    float jog_start_deg_[kJointCount]{};
    Result enable_checked(uint8_t mask, uint32_t now_ms, bool require_confirmation);
    Result start_jog(unsigned joint, int8_t direction, uint32_t now_ms);
    void run_jog(uint32_t now_ms);
    unsigned output_frames(Frame out[kJointCount]) const;
    float limit_yaw_speed(float requested, float dt) const;
    void refresh(uint32_t now_ms);
    void update_joint_feedback();
    void update_drive_references();
    void step_pitch_reference(float dt);
    bool pitch_feedback_ready(uint32_t now_ms) const;
    bool targets_in_range(const float q[kJointCount]) const;
    float drive_min(unsigned joint) const;
    float drive_max(unsigned joint) const;
    float drive_speed_limit(unsigned joint) const;
    void trip(uint32_t fault, unsigned joint);
    Result set_tuning_checked(unsigned joint, const JointTuning &tuning, uint32_t now_ms,
                             bool runtime);
    bool valid_config() const;
    bool selected(unsigned joint) const {
        return (config_.joint_mask & (1u << joint)) != 0;
    }
    uint16_t command_id(unsigned joint) const;
};
// MIT纯力矩模式: kp=kd=0，通过外部位置/速度闭环限幅后给t_ff。
void pack_dm_torque(float torque, float p_max, float v_max, float t_max, uint8_t data[8]);
} // namespace engineer
