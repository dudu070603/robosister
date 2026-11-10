#include "operator_control.h"
#include <cmath>

namespace engineer {
namespace {
float clamp(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
RobotMode mode_for(uint8_t value) {
    return value == DR16_SWITCH_UP ? RobotMode::Drive :
        value == DR16_SWITCH_MID ? RobotMode::Grab : RobotMode::Standby;
}
bool finite(float x) { return std::isfinite(x); }
}
ArmOperator::ArmOperator(const Config &a, const OperatorConfig &c) : arm_(a), config_(c) {
    valid_ = c.mapping_confirmed && c.left_switch_index < 2 && c.right_switch_index < 2 &&
        c.left_switch_index != c.right_switch_index && finite(c.deadband) && c.deadband > 0 && c.deadband < 0.5f &&
        c.remote_timeout_ms && (a.joint_mask==kFourAxisJoints || c.interlock_timeout_ms) && c.neutral_ms &&
        finite(c.gripper_open_deg) && finite(c.gripper_closed_deg) &&
        c.gripper_open_deg != c.gripper_closed_deg &&
        finite(c.gripper_hold_current) && c.gripper_hold_current > 0 &&
        c.gripper_hold_current <= a.joint[Gripper].max_effort && c.gripper_close_timeout_ms &&
        finite(a.pitch.second_drive_min_deg) && finite(a.pitch.second_drive_max_deg) &&
        a.pitch.second_drive_min_deg<=0 && a.pitch.second_drive_max_deg>=0 &&
        a.pitch.second_drive_min_deg<a.pitch.second_drive_max_deg;
    const unsigned channels[] = {c.left_y_channel,c.left_x_channel,c.right_y_channel,c.right_x_channel};
    for (unsigned i=0;i<4;++i) {
        if (channels[i]>=4 || (c.axis_direction[i]!=1 && c.axis_direction[i]!=-1)) valid_=false;
        for (unsigned j=0;j<i;++j) if(channels[i]==channels[j]) valid_=false;
    }
    const auto &g=a.joint[Gripper];
    if (c.gripper_open_deg < g.min_deg || c.gripper_open_deg > g.max_deg ||
        c.gripper_closed_deg < g.min_deg || c.gripper_closed_deg > g.max_deg) valid_=false;
}
void ArmOperator::freeze(const Status &s) {
    for (unsigned i=0;i<kJointCount;++i)
        targets_[i]=(arm_.joint_mask & (1u<<i)) ?
            clamp(s.joint[i].position_deg,arm_.joint[i].min_deg,arm_.joint[i].max_deg) : 0;
    // 反馈接近保护边界时，逐关节clamp后仍须保留a2的可行范围。
    const auto &p=arm_.pitch;
    targets_[BigPitch]=clamp(targets_[BigPitch],
        std::fmax(arm_.joint[BigPitch].min_deg,p.second_drive_min_deg-arm_.joint[SmallPitch].max_deg),
        std::fmin(arm_.joint[BigPitch].max_deg,p.second_drive_max_deg-arm_.joint[SmallPitch].min_deg));
    targets_[SmallPitch]=clamp(targets_[SmallPitch],
        std::fmax(arm_.joint[SmallPitch].min_deg,p.second_drive_min_deg-targets_[BigPitch]),
        std::fmin(arm_.joint[SmallPitch].max_deg,p.second_drive_max_deg-targets_[BigPitch]));
    close_tracking_=false; close_limited_=false;
}
void ArmOperator::wait_for_neutral(const Status &s) {
    waiting_=true; neutral_tracking_=false; last_right_=0;
    ++motion_epoch_; yaw_was_allowed_=false;
    freeze(s);
}
float ArmOperator::axis(const DR16Frame &f, unsigned ch, int sign) const {
    const float x=clamp(float(f.channel[ch])/660.0f,-1,1)*sign;
    if(std::fabs(x)<=config_.deadband) return 0;
    return (x>0 ? 1.0f : -1.0f)*(std::fabs(x)-config_.deadband)/(1-config_.deadband);
}

OperatorOutput ArmOperator::step(const RemoteInput &r, const Interlocks &in, const Status &s,
                               uint32_t now, bool arm_request, bool stop_request) {
    OperatorOutput o;
    const bool bench=arm_.joint_mask==kFourAxisJoints;
    o.gripper_effort_limit=config_.gripper_hold_current;
    const uint32_t elapsed=have_step_ ? uint32_t(now-last_step_) : arm_.control_period_ms;
    last_step_=now; have_step_=true;
    const bool timing_ok=elapsed>0 && elapsed<=arm_.control_period_ms+2;
    const bool remote_ok=r.valid && uint32_t(now-r.received_ms)<=config_.remote_timeout_ms;
    const bool link_ok=bench || (in.valid && uint32_t(now-in.received_ms)<=config_.interlock_timeout_ms);
    auto finish=[&]() {
        o.mode=mode_; o.active=active_; o.wait_neutral=waiting_;
        o.motion_epoch=motion_epoch_; o.yaw_relative_deg=bench ? 0 : s.joint[Yaw].position_deg;
        if(bench) { o.stop_chassis=true; o.emergency_stop=true; }
        for(unsigned i=0;i<kJointCount;++i) o.target_deg[i]=targets_[i];
        return o;
    };
    auto fail=[&](OperatorBlock reason) {
        if(!safe_latched_) wait_for_neutral(s);
        safe_latched_=true; active_=false; mode_=RobotMode::Safe;
        bench_up_seen_=false; bench_arm_pending_=false;
        permission_blocked_=false; o.block=reason;
        return finish(); // 关控/失联保持此前确认的全臂失能策略。
    };
    if(!valid_) return fail(OperatorBlock::InvalidConfig);
    if(!remote_ok) return fail(OperatorBlock::RemoteOffline);
    if(!link_ok) return fail(OperatorBlock::InterlockOffline);
    if(stop_request) return fail(OperatorBlock::NeedArm);
    if(!timing_ok) return fail(OperatorBlock::Timing);
    for(unsigned i=0;i<4;++i)
        if(r.frame.channel[i] < -700 || r.frame.channel[i]>700) return fail(OperatorBlock::RemoteOffline);
    const uint8_t left=r.frame.switches[config_.left_switch_index];
    const uint8_t right=r.frame.switches[config_.right_switch_index];
    if(left<1 || left>3 || right<1 || right>3) return fail(OperatorBlock::RemoteOffline);
    const auto &yaw=arm_.joint[Yaw];
    if(!bench && in.drive_yaw_allowed &&
       (!finite(in.drive_yaw_min_deg) || !finite(in.drive_yaw_max_deg) ||
        !finite(in.drive_yaw_max_speed_deg_s) || in.drive_yaw_min_deg>=in.drive_yaw_max_deg ||
        in.drive_yaw_min_deg<yaw.min_deg || in.drive_yaw_max_deg>yaw.max_deg ||
        in.drive_yaw_max_speed_deg_s<=0 || in.drive_yaw_max_speed_deg_s>yaw.max_speed_deg_s))
        return fail(OperatorBlock::InvalidInterlock);
    if(bench && left==DR16_SWITCH_UP) {
        // UP是台架关控位；只有本次健康连接中实际见过UP，MID才能产生开控事件。
        const auto stopped=fail(OperatorBlock::NeedArm);
        bench_up_seen_=s.configuration_valid && !s.faults;
        return stopped;
    }
    safe_latched_=false;
    if(active_ && (s.enabled_mask!=arm_.joint_mask || s.faults)) return fail(OperatorBlock::ArmUnavailable);
    if(bench_up_seen_) {
        bench_arm_pending_=left==DR16_SWITCH_MID;
        bench_up_seen_=false;
    }
    const RobotMode wanted=mode_for(left);
    if(wanted!=mode_) {
        mode_=wanted; wait_for_neutral(s); permission_blocked_=false;
    }
    const float ly=axis(r.frame,config_.left_y_channel,config_.axis_direction[0]);
    const float lx=axis(r.frame,config_.left_x_channel,config_.axis_direction[1]);
    const float ry=axis(r.frame,config_.right_y_channel,config_.axis_direction[2]);
    const float rx=axis(r.frame,config_.right_x_channel,config_.axis_direction[3]);
    const bool neutral=ly==0 && lx==0 && ry==0 && rx==0 && right==DR16_SWITCH_MID;
    bool joints_ready=true;
    for(unsigned i=0;i<kJointCount;++i)
        if((arm_.joint_mask & (1u<<i)) && (!s.joint[i].online || !s.joint[i].calibrated)) joints_ready=false;
    if(bench && (!neutral || mode_!=RobotMode::Grab || !joints_ready || !s.configuration_valid || s.faults))
        bench_arm_pending_=false;
    // 正常阻塞/待机可保持已使能关节；异常路径始终保持默认emergency_stop/disable_arm。
    auto hold=[&](OperatorBlock reason) {
        o.block=reason; o.disable_arm=!active_; o.emergency_stop=!active_;
        return finish();
    };
    if(mode_==RobotMode::Standby) {
        // 进入时已抓取一次反馈为保持目标。后续不能跟着下垂反馈不断重抓目标。
        // 即使arm_request为真也不在待机中开控；退出时重新经过停车和回中。
        return hold(OperatorBlock::Standby);
    }
    bool permitted=bench || (mode_==RobotMode::Grab ? in.chassis_stopped : in.arm_allows_drive);
    OperatorBlock denial=mode_==RobotMode::Grab ? OperatorBlock::WaitChassisStop : OperatorBlock::PostureDenied;
    if(mode_==RobotMode::Drive && in.drive_yaw_allowed &&
       (s.joint[Yaw].position_deg<in.drive_yaw_min_deg || s.joint[Yaw].position_deg>in.drive_yaw_max_deg ||
        targets_[Yaw]<in.drive_yaw_min_deg || targets_[Yaw]>in.drive_yaw_max_deg ||
        s.joint[Yaw].reference_deg<in.drive_yaw_min_deg || s.joint[Yaw].reference_deg>in.drive_yaw_max_deg)) {
        permitted=false; denial=OperatorBlock::DriveYawDenied;
    }
    if(!permitted) {
        if(!permission_blocked_) wait_for_neutral(s);
        permission_blocked_=true; neutral_tracking_=false;
        return hold(denial);
    }
    permission_blocked_=false;
    if(waiting_) {
        // 只在切换/重新许可阶段要求停稳；正常行驶不能永久要求chassis_stopped。
        if(!bench && !in.chassis_stopped) { neutral_tracking_=false; return hold(OperatorBlock::WaitChassisStop); }
        if(!neutral) neutral_tracking_=false;
        else if(!neutral_tracking_) { neutral_tracking_=true; neutral_since_=now; }
        else if(uint32_t(now-neutral_since_)>=config_.neutral_ms) { waiting_=false; last_right_=DR16_SWITCH_MID; }
        if(waiting_) return hold(OperatorBlock::WaitNeutral);
    }
    if(!active_) {
        const bool requested=arm_request || bench_arm_pending_;
        bench_arm_pending_=false; // 只消费一次；失败后必须重新UP→MID或显式请求。
        if(!requested || !neutral || !s.configuration_valid || s.faults) return hold(OperatorBlock::NeedArm);
        if(!joints_ready) return hold(OperatorBlock::ArmUnavailable);
        freeze(s); active_=true; o.request_enable=true;
    }
    o.disable_arm=false; o.emergency_stop=false; o.block=OperatorBlock::None;
    const float dt=float(elapsed)*0.001f;
    auto integrate=[&](unsigned i,float input) {
        const auto &c=arm_.joint[i];
        targets_[i]=clamp(targets_[i]+input*c.max_speed_deg_s*dt,c.min_deg,c.max_deg);
    };
    if(mode_==RobotMode::Drive) {
        o.reset_chassis_motion=false;
        o.fast_requested=right==DR16_SWITCH_DOWN;
        o.drive_gear=o.fast_requested && in.arm_allows_fast ? DriveGear::Fast : DriveGear::Slow;
        o.longitudinal_accel_request=ly;
        o.longitudinal_brake=ly==0; // 转弯时也必须独立将纵向速度制动到0。
        o.turn_rate_request=-rx; // 右推=顺时针；CCW正，不乘纵向输入或倒车符号。
        o.stop_chassis=ly==0 && rx==0;
        if(in.drive_yaw_allowed) {
            targets_[Yaw]=clamp(targets_[Yaw]+lx*in.drive_yaw_max_speed_deg_s*dt,
                                in.drive_yaw_min_deg,in.drive_yaw_max_deg);
            yaw_was_allowed_=true;
        } else {
            if(yaw_was_allowed_) targets_[Yaw]=clamp(s.joint[Yaw].position_deg,yaw.min_deg,yaw.max_deg);
            yaw_was_allowed_=false; o.drive_yaw_limited=true;
        }
        // yaw_follow始终false；其余机械臂轴不接受行驶摇杆新目标。
    } else if(mode_==RobotMode::Grab) {
        const auto &b=arm_.joint[BigPitch]; const auto &elbow_config=arm_.joint[SmallPitch];
        float db=clamp(targets_[BigPitch]+ly*b.max_speed_deg_s*dt,b.min_deg,b.max_deg)-targets_[BigPitch];
        float ds=clamp(targets_[SmallPitch]+ry*elbow_config.max_speed_deg_s*dt,elbow_config.min_deg,elbow_config.max_deg)-targets_[SmallPitch];
        const float sum=targets_[BigPitch]+targets_[SmallPitch], delta=db+ds;
        float scale=1;
        if(delta>0) scale=std::fmin(scale,(arm_.pitch.second_drive_max_deg-sum)/delta);
        else if(delta<0) scale=std::fmin(scale,(arm_.pitch.second_drive_min_deg-sum)/delta);
        scale=clamp(scale,0,1);
        targets_[BigPitch]+=db*scale; targets_[SmallPitch]+=ds*scale;
        integrate(Roll,lx);
        if(!bench) integrate(Yaw,rx);
        if(right!=last_right_) {
            targets_[Gripper]=clamp(s.joint[Gripper].position_deg,arm_.joint[Gripper].min_deg,arm_.joint[Gripper].max_deg);
            close_tracking_=false; close_limited_=false;
        }
        if(right==DR16_SWITCH_DOWN) {
            if(!close_tracking_) { close_tracking_=true; close_since_=now; }
            if(uint32_t(now-close_since_)>=config_.gripper_close_timeout_ms) close_limited_=true;
        }
        if(right!=DR16_SWITCH_MID && !close_limited_) {
            const float goal=right==DR16_SWITCH_UP ? config_.gripper_open_deg : config_.gripper_closed_deg;
            const float max_step=arm_.joint[Gripper].max_speed_deg_s*dt;
            targets_[Gripper]+=clamp(goal-targets_[Gripper],-max_step,max_step);
            o.gripper_effort_limit=arm_.joint[Gripper].max_effort;
        }
        if(close_limited_) {
            targets_[Gripper]=clamp(s.joint[Gripper].position_deg,arm_.joint[Gripper].min_deg,arm_.joint[Gripper].max_deg);
            o.gripper_effort_limit=config_.gripper_hold_current;
        }
    }
    last_right_=right;
    return finish();
}
}
