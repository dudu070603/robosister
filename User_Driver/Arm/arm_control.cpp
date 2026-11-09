#include "arm_control.h"
#include <cmath>
#include <cstring>

namespace engineer {
namespace {
constexpr float kRadToDeg = 57.29577951308232f;
float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
bool finite(float x) { return std::isfinite(x); }
bool mask_valid(uint8_t m, uint8_t allowed) { return m != 0 && (m & ~allowed) == 0; }
bool whole_pitch_pair(uint8_t m) { return !(m & kPitchPair) || (m & kPitchPair) == kPitchPair; }
uint16_t be16(const uint8_t *d) { return uint16_t(d[0]) << 8 | d[1]; }
float decode(unsigned x, float max, unsigned bits) {
    return float(x) * (2.0f * max) / float((1u << bits) - 1u) - max;
}
unsigned encode(float x, float max, unsigned bits) {
    return unsigned((clamp(x, -max, max) + max) * float((1u << bits) - 1u) / (2 * max));
}
}

void pack_dm_torque(float t, float pm, float vm, float tm, uint8_t d[8]) {
    const unsigned p = encode(0, pm, 16), v = encode(0, vm, 12), torque = encode(t, tm, 12);
    d[0] = p >> 8; d[1] = p;
    d[2] = v >> 4; d[3] = (v & 0xf) << 4; // kp=0
    d[4] = 0; d[5] = 0;                    // kd=0
    d[6] = torque >> 8; d[7] = torque;
}

ArmControl::ArmControl(const Config &c) : config_(c) {
    status_.configuration_valid = valid_config();
    for(unsigned i=0;i<kJointCount;++i) runtime_[i].effort_limit=c.joint[i].max_effort;
}

float ArmControl::drive_min(unsigned i) const {
    return i == SmallPitch ? config_.pitch.second_drive_min_deg : config_.joint[i].min_deg;
}
float ArmControl::drive_max(unsigned i) const {
    return i == SmallPitch ? config_.pitch.second_drive_max_deg : config_.joint[i].max_deg;
}
float ArmControl::drive_speed_limit(unsigned i) const {
    return i == SmallPitch ? config_.pitch.second_drive_max_speed_deg_s : config_.joint[i].max_speed_deg_s;
}
bool ArmControl::targets_in_range(const float q[kJointCount]) const {
    for (unsigned i=0;i<kJointCount;++i) if (selected(i))
        if (!finite(q[i]) || q[i]<config_.joint[i].min_deg || q[i]>config_.joint[i].max_deg) return false;
    const float a2=q[BigPitch]+q[SmallPitch];
    return finite(a2) && a2>=drive_min(SmallPitch) && a2<=drive_max(SmallPitch);
}
bool ArmControl::pitch_feedback_ready(uint32_t now) const {
    const auto &b=status_.joint[BigPitch], &s=status_.joint[SmallPitch];
    if (!runtime_[BigPitch].have_feedback || !runtime_[SmallPitch].have_feedback ||
        uint32_t(now-b.last_feedback_ms)>config_.feedback_timeout_ms ||
        uint32_t(now-s.last_feedback_ms)>config_.feedback_timeout_ms) return false;
    // 两时间戳都已限制在最近feedback_timeout范围内，支持HAL tick绕回。
    const uint32_t ab=b.last_feedback_ms-s.last_feedback_ms;
    const uint32_t ba=s.last_feedback_ms-b.last_feedback_ms;
    return (ab<ba ? ab : ba)<=config_.pitch.max_feedback_skew_ms;
}
void ArmControl::update_joint_feedback() {
    for (unsigned i=0;i<kJointCount;++i) {
        if (!selected(i)) continue;
        auto &s=status_.joint[i]; const auto &c=config_.joint[i];
        s.drive_position_deg=c.direction*(runtime_[i].motor_position_deg-runtime_[i].zero_motor_deg)/c.feedback_per_joint;
        s.position_deg=s.drive_position_deg;
        s.velocity_deg_s=s.drive_velocity_deg_s;
    }
    auto &s=status_.joint[SmallPitch]; const auto &b=status_.joint[BigPitch];
    s.position_deg=s.drive_position_deg-b.drive_position_deg;
    s.velocity_deg_s=s.drive_velocity_deg_s-b.drive_velocity_deg_s;
}
void ArmControl::update_drive_references() {
    for (unsigned i=0;i<kJointCount;++i) {
        if (!selected(i)) continue;
        auto &s=status_.joint[i];
        s.drive_target_deg=s.target_deg;
        s.drive_reference_deg=s.reference_deg;
        s.drive_reference_velocity_deg_s=runtime_[i].reference_velocity;
    }
    auto &s=status_.joint[SmallPitch]; const auto &b=status_.joint[BigPitch];
    s.drive_target_deg=b.target_deg+s.target_deg;
    s.drive_reference_deg=b.reference_deg+s.reference_deg;
    // runtime_.reference_velocity在pitch规划中已保存驱动坐标速度。
}

void ArmControl::step_pitch_reference(float dt) {
    auto &b=status_.joint[BigPitch]; auto &s=status_.joint[SmallPitch];
    const float error[2]={b.target_deg-b.reference_deg,s.target_deg-s.reference_deg};
    const float distance=std::sqrt(error[0]*error[0]+error[1]*error[1]);
    if (!pitch_path_active_ || pitch_speed_<=0) {
        pitch_speed_=0; pitch_path_active_=false;
        if (distance>0.00001f) {
            pitch_direction_[0]=error[0]/distance; pitch_direction_[1]=error[1]/distance;
            pitch_end_[0]=b.target_deg; pitch_end_[1]=s.target_deg;
            pitch_speed_limit_=1.0e9f; pitch_accel_limit_=1.0e9f;
            const float components[3]={pitch_direction_[0],pitch_direction_[1],pitch_direction_[0]+pitch_direction_[1]};
            const float speeds[3]={config_.joint[BigPitch].max_speed_deg_s,config_.joint[SmallPitch].max_speed_deg_s,
                                   config_.pitch.second_drive_max_speed_deg_s};
            const float accels[3]={config_.joint[BigPitch].max_accel_deg_s2,config_.joint[SmallPitch].max_accel_deg_s2,
                                   config_.pitch.second_drive_max_accel_deg_s2};
            for (unsigned i=0;i<3;++i) if (std::fabs(components[i])>0.000001f) {
                pitch_speed_limit_=std::fmin(pitch_speed_limit_,speeds[i]/std::fabs(components[i]));
                pitch_accel_limit_=std::fmin(pitch_accel_limit_,accels[i]/std::fabs(components[i]));
            }
            pitch_path_active_=true;
        }
    }
    if (pitch_path_active_) {
        const float along=error[0]*pitch_direction_[0]+error[1]*pitch_direction_[1];
        const float across=error[0]*pitch_direction_[1]-error[1]*pitch_direction_[0];
        const float stop_distance=pitch_speed_*pitch_speed_/(2*pitch_accel_limit_);
        // 连续同方向遥控目标可延长线段；转向/退回时先沿旧可行线段减速再重规划。
        // q1/q2范围及a2=q1+q2范围都是凸区间，合法端点间直线不会穿越限位。
        const bool following=along>=stop_distance && std::fabs(across)<=0.00001f;
        float desired_speed=0;
        if (following) {
            const float adt=pitch_accel_limit_*dt;
            // 比连续sqrt(2*a*d)提前一个采样周期制动，避免离散积分越过端点。
            desired_speed=std::fmin(pitch_speed_limit_,
                std::sqrt(adt*adt+2*pitch_accel_limit_*along)-adt);
            pitch_end_[0]=b.reference_deg+pitch_direction_[0]*along;
            pitch_end_[1]=s.reference_deg+pitch_direction_[1]*along;
        }
        pitch_speed_+=clamp(desired_speed-pitch_speed_,-pitch_accel_limit_*dt,pitch_accel_limit_*dt);
        if (pitch_speed_<0.0000001f) { pitch_speed_=0; pitch_path_active_=false; }
        const float remaining=(pitch_end_[0]-b.reference_deg)*pitch_direction_[0]+
                              (pitch_end_[1]-s.reference_deg)*pitch_direction_[1];
        if (pitch_speed_*dt>remaining+0.0001f) { trip(PositionLimit,SmallPitch); return; }
        b.reference_deg+=pitch_direction_[0]*pitch_speed_*dt;
        s.reference_deg+=pitch_direction_[1]*pitch_speed_*dt;
    }
    runtime_[BigPitch].reference_velocity=pitch_direction_[0]*pitch_speed_;
    runtime_[SmallPitch].reference_velocity=(pitch_direction_[0]+pitch_direction_[1])*pitch_speed_;
}

uint16_t ArmControl::feedback_id(unsigned i) const {
    const auto &c = config_.joint[i];
    return c.motor == Motor::Dm4310Mit ? c.feedback_id :
        uint16_t((c.motor == Motor::M2006 ? 0x200 : 0x204) + c.motor_id);
}
uint16_t ArmControl::command_id(unsigned i) const {
    const auto &c = config_.joint[i];
    if (c.motor == Motor::Dm4310Mit) return c.motor_id;
    if (c.motor == Motor::M2006) return c.motor_id <= 4 ? 0x200 : 0x1ff;
    if (c.motor == Motor::Gm6020Current) return c.motor_id <= 4 ? 0x1fe : 0x2fe;
    return c.motor_id <= 4 ? 0x1ff : 0x2ff; // 仅供显式选择的6020电压模式
}
bool ArmControl::valid_config() const {
    if (config_.joint_mask != kAllJoints && config_.joint_mask != kFourAxisJoints) return false;
    if (!config_.feedback_timeout_ms || !config_.command_timeout_ms || !config_.dji_unwrap_gap_ms ||
        config_.control_period_ms < 1 || config_.control_period_ms > 10)
        return false;
    const auto &p=config_.pitch;
    if (config_.joint[BigPitch].motor!=Motor::Dm4310Mit || config_.joint[SmallPitch].motor!=Motor::Dm4310Mit ||
        !finite(p.second_drive_min_deg) || !finite(p.second_drive_max_deg) ||
        p.second_drive_min_deg>=p.second_drive_max_deg || p.second_drive_min_deg>0 || p.second_drive_max_deg<0 ||
        !finite(p.second_drive_max_speed_deg_s) || p.second_drive_max_speed_deg_s<=0 ||
        !finite(p.second_drive_max_accel_deg_s2) || p.second_drive_max_accel_deg_s2<=0 ||
        !p.max_feedback_skew_ms || p.max_feedback_skew_ms>config_.feedback_timeout_ms) return false;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i)) continue;
        const auto &c = config_.joint[i];
        if ((c.motor != Motor::Gm6020Voltage && c.motor != Motor::Gm6020Current &&
             c.motor != Motor::M2006 && c.motor != Motor::Dm4310Mit) ||
            (c.bus != 1 && c.bus != 2) || (c.direction != 1 && c.direction != -1) ||
            !finite(c.feedback_per_joint) || c.feedback_per_joint <= 0 ||
            !finite(c.min_deg) || !finite(c.max_deg) || c.min_deg >= c.max_deg ||
            c.min_deg > 0 || c.max_deg < 0 ||
            !finite(c.max_speed_deg_s) || c.max_speed_deg_s <= 0 ||
            !finite(c.max_accel_deg_s2) || c.max_accel_deg_s2 <= 0 ||
            !finite(c.max_effort) || c.max_effort <= 0 ||
            !finite(c.position_kp) || c.position_kp <= 0 ||
            !finite(c.velocity_kp) || c.velocity_kp < 0 ||
            !finite(c.velocity_ki) || c.velocity_ki < 0 ||
            !finite(c.max_temperature_c) || !c.motor_id ||
            !finite(c.limit_margin_deg) || c.limit_margin_deg<0 ||
            !finite(c.stationary_speed_deg_s) || c.stationary_speed_deg_s<=0) return false;
        if (c.motor == Motor::Dm4310Mit) {
            // 反馈数据低4位携带电机ID，此版要求ID为1~15且MST_ID各自独立。
            if (c.motor_id > 15 || !c.feedback_id || c.feedback_id > 0x7ff ||
                !finite(c.p_max) || c.p_max <= 0 || !finite(c.v_max) || c.v_max <= 0 ||
                !finite(c.t_max) || c.t_max <= 0 || c.max_effort > c.t_max ||
                c.max_temperature_c <= 0 || c.motor_id == c.feedback_id) return false;
        } else if (c.motor_id > (c.motor == Motor::M2006 ? 8 : 7) ||
                   c.max_effort > (c.motor == Motor::M2006 ? 10000 :
                                   c.motor == Motor::Gm6020Current ? 3000 : 25000)) return false;
        for (unsigned j = 0; j < i; ++j) {
            if (!selected(j)) continue;
            if (c.bus != config_.joint[j].bus) continue;
            if (feedback_id(i) == feedback_id(j)) return false;
            const bool dm = c.motor == Motor::Dm4310Mit;
            const bool other_dm = config_.joint[j].motor == Motor::Dm4310Mit;
            // DM指令与其他指令/反馈不可同ID。
            if ((dm || other_dm) && command_id(i) == command_id(j)) return false;
            if (command_id(i) == feedback_id(j) || command_id(j) == feedback_id(i)) return false;
        }
    }
    return true;
}

bool ArmControl::receive(uint8_t bus, uint16_t id, const uint8_t *d, uint8_t len, uint32_t now) {
    if (!d || len != 8 || !status_.configuration_valid) return false;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i)) continue;
        const auto &c = config_.joint[i];
        if (bus != c.bus || id != feedback_id(i)) continue;
        auto &r = runtime_[i]; auto &s = status_.joint[i];
        if (c.motor == Motor::Dm4310Mit) {
            if ((d[0] & 0xf) != c.motor_id) return false;
            r.motor_position_deg = decode(be16(d + 1), c.p_max, 16) * kRadToDeg;
            s.drive_velocity_deg_s = c.direction * decode((unsigned(d[3]) << 4) | (d[4] >> 4), c.v_max, 12)
                * kRadToDeg / c.feedback_per_joint;
            s.motor_effort_feedback = decode((unsigned(d[4] & 0xf) << 8) | d[5], c.t_max, 12);
            s.motor_state = d[0] >> 4;
            s.temperature_c = d[6] > d[7] ? d[6] : d[7];
        } else {
            const auto encoder = be16(d);
            if (encoder >= 8192) return false;
            if (!r.have_feedback) r.encoder_sum = encoder;
            else if (uint32_t(now - s.last_feedback_ms) > config_.dji_unwrap_gap_ms) {
                r.encoder_sum = encoder;
                s.calibrated = false;
                if (s.enabled) trip(PositionLost, i);
            } else {
                int delta = int(encoder) - int(r.last_encoder);
                if (delta > 4096) delta -= 8192;
                if (delta < -4096) delta += 8192;
                r.encoder_sum += delta;
            }
            r.last_encoder = encoder;
            r.motor_position_deg = float(r.encoder_sum) * (360.0f / 8192.0f);
            s.drive_velocity_deg_s = c.direction * float(int16_t(be16(d + 2))) * 6.0f / c.feedback_per_joint;
            s.motor_effort_feedback = int16_t(be16(d + 4));
            // C610手册的DATA[6:7]保留，不当成温度反馈。
            s.temperature_c = c.motor != Motor::M2006 ? float(d[6]) : 0;
        }
        s.last_feedback_ms = now; ++s.received; r.have_feedback = true; s.online = true;
        update_joint_feedback(); // step在输出前统一refresh并检查时差；enable也会重新检查。
        return true;
    }
    return false;
}

void ArmControl::trip(uint32_t fault, unsigned i) {
    status_.faults |= fault;
    status_.joint[i].faults |= fault;
    if (i==BigPitch || i==SmallPitch || fault==TransportError) {
        status_.joint[BigPitch].calibrated=false;
        status_.joint[SmallPitch].calibrated=false;
    }
    disable();                    // 所选关节作为整体停机，禁止自动恢复
}
void ArmControl::disable() {
    status_.enabled_mask = 0;
    for (unsigned i = 0; i < kJointCount; ++i) {
        auto &s = status_.joint[i]; auto &r = runtime_[i];
        s.enabled = false; s.effort = 0; r.integral = 0;
        r.reference_velocity = 0; r.dm_was_enabled = false;
        s.drive_reference_velocity_deg_s=0;
    }
    pitch_speed_=0; pitch_path_active_=false;
}
void ArmControl::transport_fault() { trip(TransportError, selected(Yaw) ? Yaw : BigPitch); }
Result ArmControl::set_effort_limit(unsigned i,float limit) {
    if(i>=kJointCount || !selected(i) || !finite(limit) || limit<0 || limit>config_.joint[i].max_effort)
        return status_.last_result=Result::InvalidArgument;
    runtime_[i].effort_limit=limit;
    runtime_[i].integral=clamp(runtime_[i].integral,-limit,limit);
    return status_.last_result=Result::Ok;
}
Result ArmControl::clear_faults() {
    if (status_.enabled_mask) return status_.last_result = Result::Busy;
    status_.faults = 0;
    for (auto &s : status_.joint) s.faults = 0;
    return status_.last_result = Result::Ok;
}
void ArmControl::refresh(uint32_t now) {
    update_joint_feedback();
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!selected(i)) continue;
        auto &s = status_.joint[i]; const auto &c = config_.joint[i];
        s.online = runtime_[i].have_feedback && uint32_t(now - s.last_feedback_ms) <= config_.feedback_timeout_ms;
        if (!s.online) {
            s.calibrated = false;
            if (s.enabled) trip(FeedbackTimeout, i);
        }
        if (runtime_[i].have_feedback && c.motor != Motor::Dm4310Mit &&
            uint32_t(now - s.last_feedback_ms) > config_.dji_unwrap_gap_ms) {
            s.calibrated = false;
            if (s.enabled) trip(PositionLost, i);
        }
    }
    const bool pitch_ready=pitch_feedback_ready(now);
    status_.joint[BigPitch].online=pitch_ready;
    status_.joint[SmallPitch].online=pitch_ready;
    if (!pitch_ready) {
        status_.joint[BigPitch].calibrated=false;
        status_.joint[SmallPitch].calibrated=false;
        if (status_.enabled_mask & kPitchPair) trip(FeedbackTimeout,SmallPitch);
    }
    for (unsigned i=0;i<kJointCount;++i) {
        auto &s=status_.joint[i]; const auto &c=config_.joint[i];
        if (!s.enabled) continue;
        if (uint32_t(now - runtime_[i].last_command_ms) > config_.command_timeout_ms) trip(CommandTimeout, i);
        else if (c.motor != Motor::M2006 && s.temperature_c >= c.max_temperature_c) trip(OverTemperature, i);
        else if (c.motor == Motor::Dm4310Mit && s.motor_state >= 8) trip(MotorError, i);
        else if (s.position_deg < c.min_deg-c.limit_margin_deg || s.position_deg > c.max_deg+c.limit_margin_deg) trip(PositionLimit, i);
        else if (s.drive_position_deg<drive_min(i)-c.limit_margin_deg || s.drive_position_deg>drive_max(i)+c.limit_margin_deg) trip(PositionLimit,i);
    }
}
Result ArmControl::capture_zero(uint8_t mask, uint32_t now) {
    if (!mask_valid(mask, config_.joint_mask) || !whole_pitch_pair(mask)) return status_.last_result = Result::InvalidArgument;
    if (status_.enabled_mask) return status_.last_result = Result::Busy;
    refresh(now);
    for (unsigned i = 0; i < kJointCount; ++i) if (mask & (1u << i)) {
        if (!status_.joint[i].online) return status_.last_result = Result::NotOnline;
        // 两DM一起同向运动时q肘速度可能为0，校零仍必须检查每个驱动轴静止。
        if (std::fabs(status_.joint[i].drive_velocity_deg_s) > config_.joint[i].stationary_speed_deg_s)
            return status_.last_result = Result::Moving;
    }
    for (unsigned i = 0; i < kJointCount; ++i) if (mask & (1u << i)) {
        runtime_[i].zero_motor_deg = runtime_[i].motor_position_deg;
        auto &s = status_.joint[i];
        s.calibrated = true; s.position_deg = 0; s.target_deg = 0; s.reference_deg = 0;
    }
    update_joint_feedback(); update_drive_references();
    return status_.last_result = Result::Ok;
}
Result ArmControl::enable(uint8_t mask, uint32_t now) {
    if (!mask_valid(mask, config_.joint_mask) || !whole_pitch_pair(mask)) return status_.last_result = Result::InvalidArgument;
    if (status_.enabled_mask) return status_.last_result = Result::Busy;
    if (!status_.configuration_valid || !config_.hardware_confirmed ||
        ((mask & kPitchPair) && !config_.pitch.mapping_confirmed))
        return status_.last_result = Result::ConfigUnconfirmed;
    refresh(now);
    if (status_.faults) return status_.last_result = Result::FaultLatched;
    for (unsigned i = 0; i < kJointCount; ++i) if (mask & (1u << i)) {
        const auto &s = status_.joint[i]; const auto &c = config_.joint[i];
        if (!s.online) return status_.last_result = Result::NotOnline;
        if (!s.calibrated) return status_.last_result = Result::NotCalibrated;
        if (s.position_deg < c.min_deg || s.position_deg > c.max_deg) return status_.last_result = Result::OutOfRange;
        if (s.drive_position_deg<drive_min(i) || s.drive_position_deg>drive_max(i)) return status_.last_result=Result::OutOfRange;
        if ((c.motor != Motor::M2006 && s.temperature_c >= c.max_temperature_c) ||
            (c.motor == Motor::Dm4310Mit && s.motor_state >= 8)) return status_.last_result = Result::FaultLatched;
    }
    status_.enabled_mask = mask;
    for (unsigned i = 0; i < kJointCount; ++i) if (mask & (1u << i)) {
        auto &s = status_.joint[i]; auto &r = runtime_[i];
        s.enabled = true; s.target_deg = s.position_deg; s.reference_deg = s.position_deg;
        r.integral = 0; r.reference_velocity = 0;
        r.last_command_ms = now; r.enable_ms = now;
    }
    pitch_speed_=0; pitch_path_active_=false;
    update_drive_references();
    return status_.last_result = Result::Ok;
}
Result ArmControl::set_targets(uint8_t mask, const float q[kJointCount], uint32_t now) {
    if (!mask_valid(mask, config_.joint_mask) || !q) return status_.last_result = Result::InvalidArgument;
    float next[kJointCount];
    for (unsigned i=0;i<kJointCount;++i) next[i]=status_.joint[i].target_deg;
    for (unsigned i = 0; i < kJointCount; ++i) if (mask & (1u << i)) {
        const auto &c = config_.joint[i];
        if (!finite(q[i])) return status_.last_result = Result::InvalidArgument;
        if (q[i] < c.min_deg || q[i] > c.max_deg) return status_.last_result = Result::OutOfRange;
        next[i]=q[i];
    }
    // 单个pitch更新保留另一关节目标，完整验证a2后才整包提交，禁止单独clamp a2。
    if (!targets_in_range(next)) return status_.last_result=Result::OutOfRange;
    for (unsigned i = 0; i < kJointCount; ++i) if (mask & (1u << i)) {
        status_.joint[i].target_deg = q[i]; runtime_[i].last_command_ms = now;
    }
    if (mask & kPitchPair) {
        runtime_[BigPitch].last_command_ms=now;
        runtime_[SmallPitch].last_command_ms=now;
    }
    update_drive_references();
    return status_.last_result = Result::Ok;
}

unsigned ArmControl::step(uint32_t now, Frame out[kJointCount]) {
    if (!status_.configuration_valid || !out) return 0;
    const uint32_t elapsed = have_step_ ? uint32_t(now - last_step_ms_) : config_.control_period_ms;
    if (have_step_ && (elapsed == 0 || elapsed > config_.control_period_ms + 2) && status_.enabled_mask)
        trip(TimingError, selected(Yaw) ? Yaw : BigPitch);
    last_step_ms_ = now; have_step_ = true;
    const float dt = float(elapsed ? elapsed : 1) * 0.001f;
    refresh(now);
    // 先统一检查DM状态，任何故障都保证本轮全部输出归零。
    for (unsigned i = 0; i < kJointCount; ++i) {
        const auto &c = config_.joint[i]; auto &r = runtime_[i]; auto &s = status_.joint[i];
        if (c.motor != Motor::Dm4310Mit || !s.enabled) continue;
        if (s.motor_state == 1) r.dm_was_enabled = true;
        else if (r.dm_was_enabled) trip(MotorError, i);
        else if (uint32_t(now - r.enable_ms) > 200) trip(EnableTimeout, i);
    }
    // 先规划全部参考、检查全部边界，再计算任何输出；避免本轮后轴故障时前轴仍非零。
    const bool pitch_running=(status_.enabled_mask & kPitchPair)==kPitchPair &&
        status_.joint[BigPitch].motor_state==1 && status_.joint[SmallPitch].motor_state==1;
    if (pitch_running) step_pitch_reference(dt);
    else { runtime_[BigPitch].reference_velocity=0; runtime_[SmallPitch].reference_velocity=0; }
    for (unsigned i = 0; i < kJointCount; ++i) {
        const auto &c = config_.joint[i]; auto &r = runtime_[i]; auto &s = status_.joint[i];
        if (i==BigPitch || i==SmallPitch) continue;
        if (s.enabled && (c.motor != Motor::Dm4310Mit || s.motor_state == 1)) {
            const float error = s.target_deg - s.reference_deg;
            // 到目标前按制动距离减速；每周期约束参考速度和加速度。
            const float braking = std::sqrt(2 * c.max_accel_deg_s2 * std::fabs(error));
            float desired = clamp(error / dt, -c.max_speed_deg_s, c.max_speed_deg_s);
            desired = clamp(desired, -braking, braking);
            r.reference_velocity += clamp(desired - r.reference_velocity,
                -c.max_accel_deg_s2 * dt, c.max_accel_deg_s2 * dt);
            s.reference_deg = clamp(s.reference_deg + r.reference_velocity * dt, c.min_deg, c.max_deg);
        }
    }
    update_drive_references();
    for (unsigned i=0;i<kJointCount;++i) {
        const auto &s=status_.joint[i]; const auto &c=config_.joint[i];
        if (s.enabled && (!finite(s.reference_deg) || !finite(s.drive_reference_deg) ||
            s.reference_deg<c.min_deg-0.0001f || s.reference_deg>c.max_deg+0.0001f ||
            s.drive_reference_deg<drive_min(i)-0.0001f || s.drive_reference_deg>drive_max(i)+0.0001f))
            trip(PositionLimit,i);
    }
    unsigned count = 0;
    for (unsigned i=0;i<kJointCount;++i) {
        if (!selected(i)) continue;
        const auto &c=config_.joint[i]; auto &r=runtime_[i]; auto &s=status_.joint[i];
        const bool ready=(c.motor!=Motor::Dm4310Mit || s.motor_state==1) &&
                         ((i!=BigPitch && i!=SmallPitch) || pitch_running);
        if (s.enabled && ready) {
            const float speed_limit=drive_speed_limit(i);
            const float speed_target=clamp(r.reference_velocity+c.position_kp*(s.drive_reference_deg-s.drive_position_deg),
                                           -speed_limit,speed_limit);
            // 这里必须使用a2的驱动速度；不能把q肘速度送入第二DM的独立PI。
            const float e = speed_target - s.drive_velocity_deg_s;
            const float limit=r.effort_limit;
            const float next_i = clamp(r.integral + c.velocity_ki * e * dt, -limit, limit);
            const float u = c.velocity_kp * e + next_i;
            if (std::fabs(u) <= limit || u * e < 0) r.integral = next_i;
            s.effort = clamp(c.velocity_kp * e + r.integral, -limit, limit);
            if ((s.drive_position_deg <= drive_min(i) && s.effort < 0) ||
                (s.drive_position_deg >= drive_max(i) && s.effort > 0)) { s.effort = 0; r.integral = 0; }
        } else { s.effort = 0; r.integral = 0; }

        if (c.motor == Motor::Dm4310Mit) {
            auto &f = out[count++]; f = {}; f.bus = c.bus; f.id = command_id(i);
            if (!s.enabled || s.motor_state != 1) {
                std::memset(f.data, 0xff, 8); f.data[7] = s.enabled ? 0xfc : 0xfd;
            } else pack_dm_torque(c.direction * s.effort, c.p_max, c.v_max, c.t_max, f.data);
        } else {
            unsigned slot = 0;
            while (slot < count && (out[slot].bus != c.bus || out[slot].id != command_id(i))) ++slot;
            if (slot == count) { out[count] = {}; out[count].bus = c.bus; out[count++].id = command_id(i); }
            const unsigned byte = ((c.motor_id - 1) % 4) * 2;
            float wire_command=c.direction*s.effort;
            if(c.motor==Motor::Gm6020Current) {
                // GM6020电流协议: ±16384对应±3000mA。不得套用C610的1mA/原码。
                wire_command=clamp(wire_command*(16384.0f/3000.0f),-16384.0f,16384.0f);
            }
            const auto raw = int16_t(wire_command);
            out[slot].data[byte] = uint16_t(raw) >> 8; out[slot].data[byte + 1] = uint16_t(raw);
        }
    }
    return count;
}
}
