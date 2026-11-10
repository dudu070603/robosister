#include "arm_control.h"
#include "operator_control.h"
#include <cmath>
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if(!(x)) return __LINE__; } while(0)
namespace {
Config four_axis(bool confirmed=true) {
    auto c=kArmConfig; c.joint_mask=kFourAxisJoints;
    c.hardware_confirmed=confirmed; c.pitch.mapping_confirmed=confirmed;
    return c;
}
OperatorConfig controls() {
    auto c=kOperatorConfig; c.mapping_confirmed=true; // Test fixture only; production remains false.
    c.neutral_ms=300; return c;
}
void feed_four(ArmControl &a,uint32_t ms) {
    const uint8_t dji[8]={3,232,0,0,0,0,25,0};
    a.receive(1,a.feedback_id(Roll),dji,8,ms); a.receive(1,a.feedback_id(Gripper),dji,8,ms);
    for(unsigned j=BigPitch;j<=SmallPitch;++j) {
        const uint8_t dm[8]={uint8_t(0x10+j),0x80,0,0x80,0x08,0,25,25};
        a.receive(1,a.feedback_id(j),dm,8,ms);
    }
}
bool no_yaw_frames(const Frame *f,unsigned n) {
    for(unsigned i=0;i<n;++i)
        if(f[i].bus!=1 || (f[i].id!=1 && f[i].id!=2 && f[i].id!=0x200)) return false;
    return true;
}
struct Rig {
    Config config=four_axis(); OperatorConfig operator_config=controls();
    ArmOperator op{config,operator_config}; Status status{}; RemoteInput rc{}; Interlocks locks{};
    uint32_t now=0;
    Rig() {
        status.configuration_valid=true;
        for(unsigned j=BigPitch;j<kJointCount;++j) { status.joint[j].online=true; status.joint[j].calibrated=true; }
        status.joint[Yaw].position_deg=80; // Physically absent and uncalibrated; must never gate four-axis control.
        status.joint[Gripper].position_deg=22.5f;
        rc.valid=true; rc.frame.switches[0]=DR16_SWITCH_MID; rc.frame.switches[1]=DR16_SWITCH_MID;
        locks.valid=false; locks.drive_yaw_allowed=true;
        locks.drive_yaw_min_deg=std::numeric_limits<float>::quiet_NaN(); // Bench ignores the chassis envelope.
    }
    OperatorOutput tick(bool explicit_request=false,bool stop=false) {
        rc.received_ms=now;
        const auto o=op.step(rc,locks,status,now,explicit_request,stop);
        if(o.request_enable) { status.enabled_mask=kFourAxisJoints; op.enable_result(true); }
        if(o.disable_arm) status.enabled_mask=0;
        now+=4; return o;
    }
    OperatorOutput start_explicit() {
        for(unsigned n=0;n<75;++n) tick();
        return tick(true);
    }
    void neutral() { for(auto &v:rc.frame.channel) v=0; rc.frame.switches[1]=DR16_SWITCH_MID; }
};
}
int test_four_axis() {
    CHECK(kFourAxisJoints==0x1e && !(kFourAxisJoints & (1u<<Yaw)));
#if ARM_FOUR_AXIS_BENCH
    CHECK(kArmConfig.joint_mask==kFourAxisJoints);
#else
    CHECK(kArmConfig.joint_mask==kAllJoints);
#endif
    CHECK(!kArmConfig.hardware_confirmed && !kArmConfig.pitch.mapping_confirmed && !kOperatorConfig.mapping_confirmed);
    Frame frames[kJointCount]{};
    { // Four-axis default gates remain closed, and even zero/disable output must never address yaw.
        ArmControl a(four_axis(false)); CHECK(a.status().configuration_valid);
        CHECK(a.enable(kFourAxisJoints,0)==Result::ConfigUnconfirmed);
        const unsigned count=a.step(0,frames); CHECK(count==3 && no_yaw_frames(frames,count));
        for(unsigned i=0;i<count;++i) {
            if(frames[i].id==0x200) { for(auto b:frames[i].data) CHECK(b==0); }
            else CHECK(frames[i].data[7]==0xfd);
        }
    }
    { // Disabled-axis API requests are rejected atomically. A yaw CAN frame is not accepted as feedback.
        ArmControl a(four_axis()); feed_four(a,0);
        const uint8_t yaw[8]={0x10,0,0,0,0,0,90,0};
        CHECK(!a.receive(1,0x209,yaw,8,0)); CHECK(a.status().joint[Yaw].received==0);
        CHECK(a.capture_zero(1u<<Yaw,0)==Result::InvalidArgument);
        CHECK(a.capture_zero(kAllJoints,0)==Result::InvalidArgument);
        CHECK(a.enable(1u<<Yaw,0)==Result::InvalidArgument);
        CHECK(a.enable(kAllJoints,0)==Result::InvalidArgument);
        CHECK(a.set_effort_limit(Yaw,1)==Result::InvalidArgument);
        float q[kJointCount]={1,1,1,1,1};
        CHECK(a.set_targets(1u<<Yaw,q,0)==Result::InvalidArgument);
        CHECK(a.set_targets(kAllJoints,q,0)==Result::InvalidArgument);
        for(const auto &j:a.status().joint) CHECK(j.target_deg==0);
        CHECK(a.capture_zero(kPitchPair,0)==Result::Ok);
        CHECK(a.capture_zero(1u<<BigPitch,0)==Result::InvalidArgument);
        CHECK(a.capture_zero(kFourAxisJoints,0)==Result::Ok);
        CHECK(!a.status().joint[Yaw].calibrated && !a.status().joint[Yaw].online);
        CHECK(a.enable(kFourAxisJoints,0)==Result::Ok);
        q[Yaw]=std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_targets(kFourAxisJoints,q,0)==Result::Ok); // Unselected absent yaw value is irrelevant.
        CHECK(a.status().joint[Yaw].target_deg==0);
        CHECK(a.set_effort_limit(Gripper,150)==Result::Ok);
        for(uint32_t ms=0;ms<=400;ms+=4) {
            feed_four(a,ms); CHECK(a.set_targets(kFourAxisJoints,q,ms)==Result::Ok);
            const auto n=a.step(ms,frames); CHECK(n==3 && no_yaw_frames(frames,n));
            CHECK(a.status().enabled_mask==kFourAxisJoints && !a.status().faults);
        }
        a.disable(); const auto n=a.step(404,frames); CHECK(n==3 && no_yaw_frames(frames,n));
    }
    { // Missing one real axis still blocks enable; yaw absence is the only removed dependency.
        ArmControl a(four_axis());
        CHECK(a.feedback_id(Roll)==0x203 && a.feedback_id(Gripper)==0x204);
        const uint8_t dji[8]={3,232,0,0,0,0,0,0};
        CHECK(!a.receive(1,0x201,dji,8,0) && !a.receive(1,0x202,dji,8,0));
        CHECK(a.receive(1,0x203,dji,8,0));
        CHECK(a.status().joint[Roll].received==1 && a.status().joint[Gripper].received==0);
        CHECK(a.capture_zero(kFourAxisJoints,0)==Result::NotOnline);
        CHECK(a.enable(kFourAxisJoints,0)==Result::NotOnline);
        CHECK(a.receive(1,0x204,dji,8,0));
        CHECK(a.status().joint[Roll].received==1 && a.status().joint[Gripper].received==1);
    }
    { // Starting in MID never auto-arms, including long neutral input; explicit request is still allowed.
        Rig r;
        for(unsigned n=0;n<100;++n) {
            const auto o=r.tick(); CHECK(!o.active && !o.request_enable && o.disable_arm);
            CHECK(o.stop_chassis && o.emergency_stop && o.turn_rate_request==0 && o.longitudinal_accel_request==0);
        }
        const auto o=r.tick(true); CHECK(o.request_enable && o.active && !o.disable_arm);
        CHECK(o.emergency_stop && o.stop_chassis && o.target_deg[Yaw]==0);
    }
    { // Healthy UP->MID requires the complete 300ms neutral interval and produces one enable request.
        Rig r; r.rc.frame.switches[0]=DR16_SWITCH_UP;
        auto o=r.tick(true); CHECK(o.disable_arm && !o.request_enable && !o.active);
        r.rc.frame.switches[0]=DR16_SWITCH_MID;
        for(unsigned n=0;n<75;++n) { o=r.tick(); CHECK(!o.request_enable && !o.active); }
        o=r.tick(); CHECK(o.request_enable && o.active && !o.disable_arm);
        o=r.tick(); CHECK(!o.request_enable && o.active && o.stop_chassis && o.emergency_stop);
        for(unsigned j=0;j<4;++j) r.rc.frame.channel[j]=660;
        o=r.tick(); CHECK(o.target_deg[BigPitch]>0 && o.target_deg[SmallPitch]>0 && o.target_deg[Roll]>0);
        CHECK(o.target_deg[Yaw]==0 && o.turn_rate_request==0 && o.longitudinal_accel_request==0);
        r.rc.frame.switches[0]=DR16_SWITCH_UP; o=r.tick(); CHECK(!o.active && o.disable_arm && o.stop_chassis);
    }
    { // MID non-neutral cancels the entire auto candidate, not merely the neutral timer.
        for(unsigned input=0;input<5;++input) {
            Rig r; r.rc.frame.switches[0]=DR16_SWITCH_UP; r.tick(); r.rc.frame.switches[0]=DR16_SWITCH_MID;
            for(unsigned n=0;n<10;++n) r.tick();
            if(input<4) r.rc.frame.channel[input]=660; else r.rc.frame.switches[1]=DR16_SWITCH_DOWN;
            auto o=r.tick(); CHECK(!o.active && !o.request_enable);
            r.neutral();
            for(unsigned n=0;n<100;++n) { o=r.tick(); CHECK(!o.active && !o.request_enable); }
            r.rc.frame.switches[0]=DR16_SWITCH_UP; r.tick(); r.rc.frame.switches[0]=DR16_SWITCH_MID;
            for(unsigned n=0;n<75;++n) r.tick(); o=r.tick(); CHECK(o.active && o.request_enable);
        }
    }
    { // Starting MID non-neutral or passing through standby cannot retain UP permission for later use.
        for(unsigned path=0;path<2;++path) {
            Rig r; r.rc.frame.switches[0]=DR16_SWITCH_UP; r.tick();
            if(path) { r.rc.frame.switches[0]=DR16_SWITCH_DOWN; r.tick(); }
            r.rc.frame.switches[0]=DR16_SWITCH_MID;
            if(!path) r.rc.frame.channel[3]=660;
            r.tick(); r.neutral();
            for(unsigned n=0;n<100;++n) { const auto o=r.tick(); CHECK(!o.active && !o.request_enable); }
        }
    }
    { // Missing feedback/calibration, faults, or a rejected enable consume the candidate permanently.
        for(unsigned reason=0;reason<5;++reason) {
            Rig r; r.rc.frame.switches[0]=DR16_SWITCH_UP; r.tick(); r.rc.frame.switches[0]=DR16_SWITCH_MID;
            if(reason==0) r.status.joint[Roll].online=false;
            if(reason==1) r.status.joint[Gripper].calibrated=false;
            if(reason==2) r.status.faults=FeedbackTimeout;
            if(reason==3) r.status.configuration_valid=false;
            auto o=r.tick();
            if(reason==4) {
                for(unsigned n=0;n<74;++n) r.tick(); o=r.tick(); CHECK(o.request_enable);
                r.op.enable_result(false); r.status.enabled_mask=0;
            }
            r.status.joint[Roll].online=true; r.status.joint[Gripper].calibrated=true;
            r.status.faults=0; r.status.configuration_valid=true;
            for(unsigned n=0;n<100;++n) { o=r.tick(); CHECK(!o.active && !o.request_enable); }
        }
    }
    { // Reconnection in MID, explicit stop, invalid timing or switch cannot revive a prior UP candidate.
        for(unsigned reason=0;reason<4;++reason) {
            Rig r; r.rc.frame.switches[0]=DR16_SWITCH_UP; r.tick(); r.rc.frame.switches[0]=DR16_SWITCH_MID;
            for(unsigned n=0;n<10;++n) r.tick();
            if(reason==0) r.rc.valid=false;
            if(reason==2) r.now+=20;
            if(reason==3) r.rc.frame.switches[0]=0;
            auto o=r.tick(false,reason==1); CHECK(!o.active && !o.request_enable && o.disable_arm);
            r.rc.valid=true; r.rc.frame.switches[0]=DR16_SWITCH_MID;
            for(unsigned n=0;n<100;++n) { o=r.tick(); CHECK(!o.active && !o.request_enable); }
        }
    }
    { // DOWN keeps existing four-axis hold, never enables an inactive arm; all chassis intent stays inhibited.
        Rig r; CHECK(r.start_explicit().active);
        r.status.joint[BigPitch].position_deg=3; r.status.joint[SmallPitch].position_deg=-2;
        r.rc.frame.switches[0]=DR16_SWITCH_DOWN; auto o=r.tick(); const auto held=o;
        CHECK(o.active && !o.disable_arm && o.stop_chassis && o.emergency_stop);
        r.status.joint[BigPitch].position_deg=4;
        for(unsigned n=0;n<100;++n) {
            for(unsigned j=0;j<4;++j) r.rc.frame.channel[j]=660;
            r.rc.frame.switches[1]=1+n%3; o=r.tick(true);
            CHECK(o.active && !o.request_enable && !o.disable_arm && o.emergency_stop && o.stop_chassis);
            for(unsigned j=0;j<kJointCount;++j) CHECK(o.target_deg[j]==held.target_deg[j]);
        }
        Rig disabled; disabled.rc.frame.switches[0]=DR16_SWITCH_DOWN;
        for(unsigned n=0;n<100;++n) { o=disabled.tick(true); CHECK(!o.active && o.disable_arm && !o.request_enable); }
    }
    { // Real control core + bench interpreter: auto-enable only four axes with no yaw frames or chassis task.
        const auto config=four_axis(); ArmControl arm(config); ArmOperator op(config,controls());
        Rig input; feed_four(arm,0); CHECK(arm.capture_zero(kFourAxisJoints,0)==Result::Ok);
        for(uint32_t t=0;t<800;t+=4) {
            feed_four(arm,t); input.rc.received_ms=t;
            input.rc.frame.switches[0]=t==0 ? DR16_SWITCH_UP : (t<500 ? DR16_SWITCH_MID : DR16_SWITCH_DOWN);
            const auto o=op.step(input.rc,input.locks,arm.status(),t,false,false);
            if(o.request_enable) { const bool ok=arm.enable(kFourAxisJoints,t)==Result::Ok; op.enable_result(ok); CHECK(ok); }
            if(o.disable_arm) arm.disable();
            else CHECK(arm.set_targets(kFourAxisJoints,o.target_deg,t)==Result::Ok);
            const unsigned n=arm.step(t,frames); CHECK(n==3 && no_yaw_frames(frames,n));
            CHECK(o.stop_chassis && o.emergency_stop && o.target_deg[Yaw]==0);
            if(t>=304) CHECK(o.active && arm.status().enabled_mask==kFourAxisJoints && !arm.status().faults);
        }
    }
    return 0;
}
