#include "robot_control.h"
#include "robot_control_internal.h"
#include "task_protect.h"
#include "operator_control.h"
#include "dr16_bsp.h"
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if(!(x)) return __LINE__; } while(0)
namespace { DR16Status receiver{}; }
extern "C" int DR16_GetStatus(DR16Status *out) {
    if(!out) return 0;
    *out=receiver; return out->valid;
}
int test_robot_control() {
    static_assert(ROBOT_CONTROL_API_VERSION==0x0202u);
    static_assert(unsigned(RobotMode::Standby)==ROBOT_STANDBY);
    static_assert(unsigned(RobotMode::Drive)==ROBOT_DRIVE);
    static_assert(unsigned(DriveGear::Fast)==ROBOT_GEAR_FAST);
    RobotControlStatus s{};
    CHECK(!Robot_GetControlStatus(nullptr)); CHECK(!Robot_GetControlStatus(&s));
    CHECK(!Robot_SetInterlocks(nullptr));
    RobotInterlocks locks{};
    locks.chassis_stopped=1; locks.arm_allows_drive=1; locks.arm_allows_fast=1;
    locks.drive_yaw_allowed=1; locks.drive_yaw_min_deg=-20; locks.drive_yaw_max_deg=20;
    locks.drive_yaw_max_speed_deg_s=2;
    CHECK(Robot_SetInterlocks(&locks));
    receiver.valid=1; receiver.consecutive_frames=3; receiver.received_ms=1234;
    receiver.frame.switches[0]=DR16_SWITCH_UP; receiver.frame.switches[1]=DR16_SWITCH_MID;
    CHECK(Robot_RequestControl());
    ArmControl arm(kArmConfig); Robot_Execute(arm,1234,false);
    CHECK(Robot_GetControlStatus(&s));
    CHECK(s.updated_ms==1234 && s.ready && s.remote_online);
    CHECK(s.joint_mask==kArmConfig.joint_mask && g_robot_status.joint_mask==s.joint_mask);
    CHECK(!s.active && s.protection_disabled && s.emergency_stop && s.stop_chassis);
    CHECK(s.protection_current_reasons & PROTECT_CONFIG);
    CHECK(!(s.protection_current_reasons & PROTECT_REMOTE));
    CHECK(!(s.protection_current_reasons & PROTECT_INTERLOCK));
    CHECK(s.longitudinal_brake && s.reset_chassis_motion && !s.yaw_follow);
    CHECK(!s.fast_requested && s.drive_gear==ROBOT_GEAR_SLOW);
    CHECK(s.longitudinal_accel_request==0 && s.turn_rate_request==0 && arm.status().enabled_mask==0);
    for(unsigned bad=0;bad<8;++bad) {
        auto invalid=locks;
        if(bad==0) invalid.chassis_stopped=2;
        if(bad==1) invalid.arm_allows_drive=2;
        if(bad==2) invalid.arm_allows_fast=2;
        if(bad==3) invalid.drive_yaw_allowed=2;
        if(bad==4) invalid.drive_yaw_min_deg=std::numeric_limits<float>::quiet_NaN();
        if(bad==5) invalid.drive_yaw_max_deg=100;
        if(bad==6) invalid.drive_yaw_max_speed_deg_s=0;
        if(bad==7) invalid.drive_yaw_min_deg=21;
        CHECK(!Robot_SetInterlocks(&invalid));
    }
    Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
    if(kArmConfig.joint_mask==kFourAxisJoints) CHECK(!(s.protection_current_reasons & PROTECT_INTERLOCK));
    else CHECK(s.protection_current_reasons & PROTECT_INTERLOCK); // 整车配置无效输入必须立刻撤掉旧许可。
    locks.drive_yaw_allowed=0; locks.drive_yaw_min_deg=0; locks.drive_yaw_max_deg=0;
    locks.drive_yaw_max_speed_deg_s=0; CHECK(Robot_SetInterlocks(&locks));
    Robot_StopControl(); Robot_Execute(arm,1234,false); CHECK(Robot_GetControlStatus(&s));
    CHECK(s.protection_current_reasons & PROTECT_STOP_REQUEST);
    CHECK(!s.active && s.emergency_stop && s.turn_rate_request==0);
    Status fault{}; fault.faults=FeedbackTimeout; Robot_ReportArmState(fault);
    CHECK(Robot_GetControlStatus(&s));
    CHECK((s.protection_latched_reasons & PROTECT_ARM_FAULT) && s.protection_disabled);
    CHECK(s.stop_chassis && s.emergency_stop && s.reset_chassis_motion);
    // Stable debugger command symbol consumes each command once and preserves default disable.
    for(unsigned j=0;j<kJointCount;++j) if(kArmConfig.joint_mask & (1u<<j)) {
        if(j==BigPitch || j==SmallPitch) {
            const uint8_t d[8]={uint8_t(0x10+j),0x80,0,0x80,0x08,0,25,25};
            CHECK(arm.receive(1,arm.feedback_id(j),d,8,1234));
        } else {
            const uint8_t d[8]={3,232,0,0,0,0,25,0};
            CHECK(arm.receive(1,arm.feedback_id(j),d,8,1234));
        }
    }
    g_robot_debug_command=ROBOT_DEBUG_CAPTURE_ZERO;
    Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE && arm.status().last_result==Result::Ok);
    CHECK(arm.status().enabled_mask==0);
    for(unsigned j=0;j<kJointCount;++j)
        CHECK(arm.status().joint[j].calibrated==bool(kArmConfig.joint_mask & (1u<<j)));
    arm.transport_fault(); CHECK(arm.status().faults & TransportError);
    g_robot_debug_command=ROBOT_DEBUG_CLEAR_FAULTS; Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE && arm.status().last_result==Result::Ok);
    CHECK(arm.status().faults==0 && arm.status().enabled_mask==0);
    g_robot_debug_command=ROBOT_DEBUG_REQUEST_CONTROL; Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE && arm.status().enabled_mask==0);
    g_robot_debug_command=ROBOT_DEBUG_STOP; Robot_Execute(arm,1234,false);
    CHECK(g_robot_debug_command==ROBOT_DEBUG_IDLE); CHECK(Robot_GetControlStatus(&s));
    CHECK(s.protection_current_reasons & PROTECT_STOP_REQUEST);
    return 0;
}
