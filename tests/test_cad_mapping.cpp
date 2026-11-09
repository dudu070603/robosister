#include "arm_control.h"
#include "operator_control.h"
#include <cmath>
#include <limits>
using namespace engineer;

extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if (!(x)) return __LINE__; } while (0)
namespace {
constexpr float kRadToDeg = 57.29577951308232f;
constexpr uint8_t kGrip = 1u << Gripper;
Config bench() {
    auto c=kArmConfig;
    c.joint_mask=kAllJoints;
    c.hardware_confirmed=true;
    c.pitch.mapping_confirmed=true;
    return c;
}
bool near(float a,float b,float tolerance=0.0001f) { return std::fabs(a-b)<=tolerance; }
void dm_raw(ArmControl &arm,const Config &cfg,unsigned joint,unsigned position,
            unsigned velocity,uint32_t now,unsigned state=1,unsigned temperature=25) {
    // Fixed wire fields, not the production packer: position16 / velocity12 / torque12.
    const unsigned torque=2048;
    uint8_t d[8]={uint8_t((state<<4)|cfg.joint[joint].motor_id),uint8_t(position>>8),uint8_t(position),
                  uint8_t(velocity>>4),uint8_t((velocity&15)<<4 | torque>>8),uint8_t(torque),
                  uint8_t(temperature),uint8_t(temperature)};
    arm.receive(cfg.joint[joint].bus,arm.feedback_id(joint),d,8,now);
}
void pitch(ArmControl &arm,const Config &cfg,uint32_t now,unsigned velocity=2048,unsigned state=1) {
    dm_raw(arm,cfg,BigPitch,32768,velocity,now,state);
    dm_raw(arm,cfg,SmallPitch,32768,velocity,now,state);
}
void dji(ArmControl &arm,unsigned joint,unsigned encoder,uint32_t now,int rpm=0) {
    uint8_t d[8]={uint8_t(encoder>>8),uint8_t(encoder),uint8_t(uint16_t(rpm)>>8),uint8_t(rpm),0,0,25,0};
    arm.receive(1,arm.feedback_id(joint),d,8,now);
}
const Frame *find(const Frame *f,unsigned n,unsigned id) {
    for(unsigned i=0;i<n;++i) if(f[i].bus==1 && f[i].id==id) return f+i;
    return nullptr;
}
float torque(const Frame &f,float maximum) {
    const unsigned raw=(unsigned(f.data[6]&15)<<8)|f.data[7];
    return float(raw)*(2*maximum)/4095-maximum;
}
int current(const Frame &f,unsigned slot) {
    return int16_t(uint16_t(f.data[slot*2])<<8 | f.data[slot*2+1]);
}
}

int test_cad_mapping() {
    Frame frames[kJointCount]{};
    { // Default firmware keeps both hardware and newly identified pitch mapping gates closed.
        CHECK(!kArmConfig.hardware_confirmed);
        CHECK(!kArmConfig.pitch.mapping_confirmed);
        CHECK(!kOperatorConfig.mapping_confirmed);
        auto cfg=bench(); cfg.pitch.mapping_confirmed=false;
        ArmControl a(cfg); pitch(a,cfg,0);
        CHECK(a.capture_zero(kPitchPair,0)==Result::Ok);
        CHECK(a.enable(kPitchPair,0)==Result::ConfigUnconfirmed);
        const unsigned n=a.step(0,frames);
        CHECK(find(frames,n,1)!=nullptr && find(frames,n,2)!=nullptr);
        CHECK(find(frames,n,1)->data[7]==0xfd && find(frames,n,2)->data[7]==0xfd);
    }
    { // Different nonzero raw zeros, reduction ratios and opposite encoder signs.
        auto cfg=bench();
        cfg.joint[BigPitch].direction=-1; cfg.joint[BigPitch].feedback_per_joint=2;
        cfg.joint[SmallPitch].direction=1; cfg.joint[SmallPitch].feedback_per_joint=4;
        ArmControl a(cfg);
        dm_raw(a,cfg,BigPitch,30000,2048,0);
        dm_raw(a,cfg,SmallPitch,35000,2048,0);
        CHECK(a.capture_zero(1u<<BigPitch,0)==Result::InvalidArgument);
        CHECK(!a.status().joint[BigPitch].calibrated && !a.status().joint[SmallPitch].calibrated);
        CHECK(a.capture_zero(kPitchPair,0)==Result::Ok);
        dm_raw(a,cfg,BigPitch,30100,2050,4);
        dm_raw(a,cfg,SmallPitch,35400,2054,4);
        const float p_quantum=25.0f*kRadToDeg/65535;
        const float a1=-100*p_quantum/2, a2=400*p_quantum/4;
        const float v1=-((2050.0f*60/4095)-30)*kRadToDeg/2;
        const float v2=((2054.0f*60/4095)-30)*kRadToDeg/4;
        CHECK(near(a.status().joint[BigPitch].position_deg,a1,0.0003f));
        CHECK(near(a.status().joint[SmallPitch].position_deg,a2-a1,0.0003f));
        CHECK(near(a.status().joint[SmallPitch].drive_position_deg,a2,0.0003f));
        CHECK(near(a.status().joint[BigPitch].velocity_deg_s,v1,0.0003f));
        CHECK(near(a.status().joint[SmallPitch].velocity_deg_s,v2-v1,0.0003f));
        CHECK(near(a.status().joint[SmallPitch].drive_velocity_deg_s,v2,0.0003f));
        CHECK(a.enable(1u<<SmallPitch,4)==Result::InvalidArgument);
        CHECK(a.enable(kPitchPair,4)==Result::Ok);
        const auto &s=a.status();
        CHECK(near(s.joint[BigPitch].target_deg,a1,0.0003f));
        CHECK(near(s.joint[SmallPitch].target_deg,a2-a1,0.0003f));
        CHECK(near(s.joint[SmallPitch].drive_target_deg,a2,0.0003f));
        CHECK(near(s.joint[SmallPitch].drive_reference_deg,a2,0.0003f));
        float q[5]={0,10,5,0,0};
        CHECK(a.set_targets(kPitchPair,q,4)==Result::Ok);
        CHECK(a.status().joint[BigPitch].drive_target_deg==10);
        CHECK(a.status().joint[SmallPitch].drive_target_deg==15);
        q[BigPitch]=12; q[SmallPitch]=999; // An unmasked element must not overwrite the held elbow target.
        CHECK(a.set_targets(1u<<BigPitch,q,4)==Result::Ok);
        CHECK(a.status().joint[SmallPitch].target_deg==5);
        CHECK(a.status().joint[SmallPitch].drive_target_deg==17);
        q[Yaw]=20; q[BigPitch]=20; q[SmallPitch]=20;
        CHECK(a.set_targets((1u<<Yaw)|kPitchPair,q,4)==Result::OutOfRange);
        CHECK(a.status().joint[Yaw].target_deg==0);
        CHECK(a.status().joint[BigPitch].target_deg==12 && a.status().joint[SmallPitch].target_deg==5);
        q[BigPitch]=std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_targets(kPitchPair,q,4)==Result::InvalidArgument);
        CHECK(a.status().joint[SmallPitch].drive_target_deg==17);
    }
    { // A single shoulder update also keeps the paired drive's command watchdog alive.
        auto cfg=bench(); ArmControl a(cfg); pitch(a,cfg,0);
        CHECK(a.capture_zero(kPitchPair,0)==Result::Ok);
        CHECK(a.enable(kPitchPair,0)==Result::Ok);
        float q[5]={0,10,5,0,0}; CHECK(a.set_targets(kPitchPair,q,0)==Result::Ok);
        for(unsigned t=0;t<=320;t+=4) {
            pitch(a,cfg,t); q[BigPitch]=10;
            CHECK(a.set_targets(1u<<BigPitch,q,t)==Result::Ok);
            a.step(t,frames);
            CHECK(a.status().faults==0 && a.status().enabled_mask==kPitchPair);
            CHECK(a.status().joint[SmallPitch].target_deg==5);
        }
    }
    { // Equal drive velocities cancel in q_elbow, but the second PI must still brake its motor.
        auto cfg=bench(); ArmControl a(cfg); pitch(a,cfg,0);
        CHECK(a.capture_zero(kPitchPair,0)==Result::Ok);
        CHECK(a.enable(kPitchPair,0)==Result::Ok); a.step(0,frames);
        pitch(a,cfg,4,2054); const unsigned n=a.step(4,frames);
        CHECK(near(a.status().joint[SmallPitch].velocity_deg_s,0));
        CHECK(a.status().joint[SmallPitch].drive_velocity_deg_s>5);
        CHECK(a.status().joint[SmallPitch].effort < -0.1f);
        CHECK(find(frames,n,2)!=nullptr);
        CHECK(torque(*find(frames,n,2),cfg.joint[SmallPitch].t_max)<-0.1f);
        CHECK(a.status().faults==0);
    }
    { // Feedback skew is bounded independently of the ordinary 50ms feedback timeout.
        auto cfg=bench(); ArmControl a(cfg);
        dm_raw(a,cfg,BigPitch,32768,2048,10);
        dm_raw(a,cfg,SmallPitch,32768,2048,0);
        CHECK(a.capture_zero(kPitchPair,10)==Result::NotOnline);
        pitch(a,cfg,12); CHECK(a.capture_zero(kPitchPair,12)==Result::Ok);
        CHECK(a.enable(kPitchPair,12)==Result::Ok); a.step(12,frames);
        dm_raw(a,cfg,BigPitch,32768,2048,16); a.step(16,frames);
        dm_raw(a,cfg,BigPitch,32768,2048,20); a.step(20,frames);
        CHECK(a.status().faults==0); // Exactly 8ms is accepted.
        dm_raw(a,cfg,BigPitch,32768,2048,24); const unsigned n=a.step(24,frames);
        CHECK(a.status().faults & FeedbackTimeout);
        CHECK(a.status().enabled_mask==0);
        CHECK(!a.status().joint[BigPitch].calibrated && !a.status().joint[SmallPitch].calibrated);
        CHECK(find(frames,n,1)->data[7]==0xfd && find(frames,n,2)->data[7]==0xfd);
        CHECK(a.clear_faults()==Result::Ok); pitch(a,cfg,28);
        CHECK(a.enable(kPitchPair,28)==Result::NotCalibrated);
        CHECK(a.capture_zero(kPitchPair,28)==Result::Ok);
        CHECK(a.enable(kPitchPair,28)==Result::Ok);
    }
    { // A motor fault invalidates the paired calibration; MCU tick rollover is not skew.
        auto cfg=bench(); ArmControl a(cfg); const uint32_t t=0xfffffffcu;
        pitch(a,cfg,t); a.capture_zero(kPitchPair,t); a.enable(kPitchPair,t); a.step(t,frames);
        dm_raw(a,cfg,BigPitch,32768,2048,0); a.step(0,frames);
        CHECK(a.status().faults==0);
        pitch(a,cfg,4); dm_raw(a,cfg,SmallPitch,32768,2048,4,8); a.step(4,frames);
        CHECK(a.status().faults & MotorError);
        CHECK(!a.status().joint[BigPitch].calibrated && !a.status().joint[SmallPitch].calibrated);
    }
    { // Retargeting a moving pitch pair stays inside both q and mapped-drive envelopes.
        auto cfg=bench(); ArmControl a(cfg); pitch(a,cfg,0);
        a.capture_zero(kPitchPair,0); a.enable(kPitchPair,0);
        float q[5]={0,10,5,0,0};
        float old_v1=0,old_v2=0,old_q1_v=0,old_q2_v=0;
        float old_q1=0,old_q2=0;
        for(unsigned i=0;i<1400;++i) {
            const unsigned t=i*4;
            if(i==30) {q[BigPitch]=-12; q[SmallPitch]=10;}
            if(i==80) {q[BigPitch]=25; q[SmallPitch]=-20;}
            if(i==150) {q[BigPitch]=-25; q[SmallPitch]=0;}
            if(i==250) {q[BigPitch]=10; q[SmallPitch]=5;}
            pitch(a,cfg,t);
            CHECK(a.set_targets(kPitchPair,q,t)==Result::Ok);
            a.step(t,frames);
            const auto &p1=a.status().joint[BigPitch]; const auto &p2=a.status().joint[SmallPitch];
            CHECK(a.status().faults==0 && a.status().enabled_mask==kPitchPair);
            CHECK(p1.reference_deg>=-30.0001f && p1.reference_deg<=30.0001f);
            CHECK(p2.reference_deg>=-30.0001f && p2.reference_deg<=30.0001f);
            CHECK(p2.drive_reference_deg>=-30.0001f && p2.drive_reference_deg<=30.0001f);
            CHECK(near(p2.drive_reference_deg,p1.reference_deg+p2.reference_deg,0.0001f));
            const float v1=p1.drive_reference_velocity_deg_s,v2=p2.drive_reference_velocity_deg_s;
            const float q1_v=v1,q2_v=v2-v1;
            CHECK(std::fabs(v1)<=cfg.joint[BigPitch].max_speed_deg_s+0.0001f);
            CHECK(std::fabs(v2)<=cfg.pitch.second_drive_max_speed_deg_s+0.0001f);
            CHECK(std::fabs(q2_v)<=cfg.joint[SmallPitch].max_speed_deg_s+0.0001f);
            CHECK(std::fabs(v1-old_v1)<=cfg.joint[BigPitch].max_accel_deg_s2*0.004f+0.0002f);
            CHECK(std::fabs(v2-old_v2)<=cfg.pitch.second_drive_max_accel_deg_s2*0.004f+0.0002f);
            CHECK(std::fabs(q1_v-old_q1_v)<=cfg.joint[BigPitch].max_accel_deg_s2*0.004f+0.0002f);
            CHECK(std::fabs(q2_v-old_q2_v)<=cfg.joint[SmallPitch].max_accel_deg_s2*0.004f+0.0002f);
            CHECK(near((p1.reference_deg-old_q1)/0.004f,q1_v,0.002f));
            CHECK(near((p2.reference_deg-old_q2)/0.004f,q2_v,0.002f));
            old_v1=v1; old_v2=v2; old_q1_v=q1_v; old_q2_v=q2_v;
            old_q1=p1.reference_deg; old_q2=p2.reference_deg;
        }
        CHECK(near(a.status().joint[BigPitch].reference_deg,10,0.02f));
        CHECK(near(a.status().joint[SmallPitch].reference_deg,5,0.02f));
    }
    { // The 32T output angle reduces the old shaft-side envelope by exactly 3/4.
        const auto &g=kArmConfig.joint[Gripper];
        CHECK(g.feedback_per_joint==48 && g.min_deg==0 && g.max_deg==45);
        CHECK(g.max_speed_deg_s==3.75f && g.max_accel_deg_s2==11.25f);
        CHECK(g.limit_margin_deg==1.5f && g.stationary_speed_deg_s==1.5f);
        CHECK(near(g.velocity_kp*0.75f,20) && near(g.velocity_ki*0.75f,5));
        CHECK(g.max_deg*g.feedback_per_joint==60*36);
        CHECK(g.max_speed_deg_s*g.feedback_per_joint==5*36);
        CHECK(g.max_accel_deg_s2*g.feedback_per_joint==15*36);
        CHECK(kOperatorConfig.gripper_closed_deg==45 && kOperatorConfig.gripper_close_timeout_ms==2000);
        auto cfg=bench(); ArmControl a(cfg); dji(a,Gripper,1000,0);
        CHECK(a.capture_zero(kGrip,0)==Result::Ok);
        // Six rotor revolutions = 45deg on the 32T output. Every sample moves less than half a turn.
        const unsigned total=6*8192;
        unsigned accumulated=0,now=0;
        while(accumulated<total) {
            const unsigned step=(total-accumulated)>3000?3000:(total-accumulated);
            accumulated+=step; ++now; dji(a,Gripper,(1000+accumulated)%8192,now);
        }
        CHECK(near(a.status().joint[Gripper].position_deg,45));
        float q[5]={0,0,0,0,60};
        CHECK(a.set_targets(kGrip,q,now)==Result::OutOfRange);
    }
    { // Same rotor feedback and physical trajectory yield the same current before/after unit conversion.
        auto cfg=bench(),legacy=cfg;
        auto &old=legacy.joint[Gripper]; old.feedback_per_joint=36; old.max_deg=60;
        old.max_speed_deg_s=5; old.max_accel_deg_s2=15; old.velocity_kp=20; old.velocity_ki=5;
        old.limit_margin_deg=2; old.stationary_speed_deg_s=2;
        ArmControl a(cfg),b(legacy);
        dji(a,Gripper,1000,0); dji(b,Gripper,1000,0);
        a.capture_zero(kGrip,0); b.capture_zero(kGrip,0); a.enable(kGrip,0); b.enable(kGrip,0);
        float q[5]={0,0,0,0,45},old_q[5]={0,0,0,0,60};
        Frame old_frames[kJointCount]{};
        for(unsigned i=0;i<150;++i) {
            const unsigned t=i*4;
            dji(a,Gripper,1000,t); dji(b,Gripper,1000,t);
            CHECK(a.set_targets(kGrip,q,t)==Result::Ok && b.set_targets(kGrip,old_q,t)==Result::Ok);
            const unsigned n=a.step(t,frames),old_n=b.step(t,old_frames);
            CHECK(a.status().faults==0 && b.status().faults==0);
            CHECK(near(a.status().joint[Gripper].reference_deg*48,b.status().joint[Gripper].reference_deg*36,0.002f));
            CHECK(near(a.status().joint[Gripper].effort,b.status().joint[Gripper].effort,0.01f));
            CHECK(find(frames,n,0x200)!=nullptr && find(old_frames,old_n,0x200)!=nullptr);
            CHECK(std::abs(current(*find(frames,n,0x200),1)-current(*find(old_frames,old_n,0x200),1))<=1);
        }
    }
    return 0;
}
