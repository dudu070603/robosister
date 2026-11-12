#include "Remote/operator_control.h"
#include "dr16_bsp.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <initializer_list>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x))                                                                                  \
            return __LINE__;                                                                       \
    } while (0)
namespace {
bool near(float a, float b, float eps = 0.0001f) {
    return std::fabs(a - b) <= eps;
}
Config vehicle() {
    auto c = kArmConfig;
    c.joint_mask = kAllJoints;
    return c;
}
OperatorConfig settings() {
    auto c = kOperatorConfig;
    c.mapping_confirmed = true;
    c.neutral_ms = 8;
    c.gripper_close_timeout_ms = 12;
    return c;
}
struct Rig {
    OperatorConfig config = settings();
    Config arm_config;
    ArmOperator op;
    Status arm{};
    RemoteInput rc{};
    Interlocks locks;
    uint32_t now = 0;
    explicit Rig(const Config &a = vehicle()) : arm_config(a), op(arm_config, config) {
        arm.configuration_valid = true;
        for (auto &j : arm.joint) {
            j.online = true;
            j.calibrated = true;
        }
        arm.joint[Gripper].position_deg = 22.5f;
        rc.valid = true;
        rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
        rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
        locks.valid = true;
        locks.chassis_stopped = true;
        locks.arm_allows_drive = true;
        locks.arm_allows_fast = true;
        locks.drive_yaw_allowed = true;
        locks.drive_yaw_min_deg = -20;
        locks.drive_yaw_max_deg = 20;
        locks.drive_yaw_max_speed_deg_s = 2;
    }
    OperatorOutput tick(bool request = false, bool stop = false, bool fresh = true) {
        if (fresh) {
            rc.received_ms = now;
            locks.received_ms = now;
        }
        auto out = op.step(rc, locks, arm, now, request, stop);
        if (out.request_enable) {
            arm.enabled_mask = kAllJoints;
            op.enable_result(true);
        }
        if (out.disable_arm)
            arm.enabled_mask = 0;
        now += 4;
        return out;
    }
    OperatorOutput start() {
        tick();
        tick();
        tick();
        return tick(true);
    }
    void neutral() {
        for (auto &v : rc.frame.channel)
            v = 0;
        rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
    }
};
Config bench() {
    auto c = vehicle();
    c.hardware_confirmed = true;
    c.pitch.mapping_confirmed = true;
    return c;
}
void feed(ArmControl &a, uint32_t ms) {
    // Fixed independent wire vector: stable DJI encoders, near-zero DM velocity.
    const uint8_t dji[8] = {3, 232, 0, 0, 0, 0, 25, 0};
    for (unsigned j : {unsigned(Yaw), unsigned(Roll), unsigned(Gripper)})
        a.receive(1, a.feedback_id(j), dji, 8, ms);
    for (unsigned j = BigPitch; j <= SmallPitch; ++j) {
        const uint8_t dm[8] = {uint8_t(0x10 + j), 0x80, 0, 0x80, 0x08, 0, 25, 25};
        a.receive(1, a.feedback_id(j), dm, 8, ms);
    }
}
} // namespace
int test_operator() {
    { // Control and diagnostic views share one validated channel/sign/deadband mapping.
        DR16Frame f{};
        f.channel[0] = 660;
        f.channel[1] = -660;
        f.channel[2] = 330;
        f.channel[3] = -330;
        CHECK(remote_axis_value(f, 0, 1, .06f) == 1);
        CHECK(remote_axis_value(f, 1, 1, .06f) == -1);
        CHECK(remote_axis_value(f, 0, -1, .06f) == -1);
        CHECK(remote_axis_value(f, 1, -1, .06f) == 1);
        CHECK(near(remote_axis_value(f, 2, 1, .06f), (.5f - .06f) / .94f));
        CHECK(near(remote_axis_value(f, 3, -1, .06f), (.5f - .06f) / .94f));
        f.channel[0] = 39;
        f.channel[1] = -39;
        CHECK(remote_axis_value(f, 0, 1, .06f) == 0);
        CHECK(remote_axis_value(f, 1, 1, .06f) == 0);
        f.channel[0] = 40;
        CHECK(remote_axis_value(f, 0, 1, .06f) > 0);
        f.channel[0] = 32767;
        CHECK(remote_axis_value(f, 0, 1, .06f) == 1);
        CHECK(remote_axis_value(f, 4, 1, .06f) == 0);
        CHECK(remote_axis_value(f, ~0u, 1, .06f) == 0);
        CHECK(remote_axis_value(f, 0, 0, .06f) == 0);
        CHECK(remote_axis_value(f, 0, 2, .06f) == 0);
        CHECK(remote_axis_value(f, 0, 1, 0) == 0);
        CHECK(remote_axis_value(f, 0, 1, .5f) == 0);
        CHECK(remote_axis_value(f, 0, 1, -.1f) == 0);
        CHECK(remote_axis_value(f, 0, 1, std::numeric_limits<float>::quiet_NaN()) == 0);
        CHECK(remote_axis_value(f, 0, 1, std::numeric_limits<float>::infinity()) == 0);
    }
    { // 固定DBUS中位向量与高位校验，避免只测打包/解包互相镜像。
        uint8_t raw[18] = {0x00, 0x04, 0x20, 0x00, 0x01, 0xf8, 0xff, 0xff, 0,
                           0,    0,    0,    1,    0,    0x34, 0x12, 0,    4};
        DR16Frame d{};
        CHECK(DR16_Decode(raw, 18, &d));
        for (unsigned i = 0; i < 5; ++i)
            CHECK(d.channel[i] == 0);
        CHECK(d.switches[0] == 3 && d.switches[1] == 3);
        CHECK(d.mouse_x == -1 && d.mouse_left == 1 && d.keyboard == 0x1234);
        const auto saved = d;
        raw[5] = 0;
        CHECK(!DR16_Decode(raw, 18, &d));
        CHECK(std::memcmp(&saved, &d, sizeof(d)) == 0);
        raw[5] = 0xf8;
        CHECK(!DR16_Decode(raw, 17, &d));
        raw[0] = 0xff;
        raw[1] = 0x07;
        CHECK(!DR16_Decode(raw, 18, &d));
    }
    { // 夹取映射保留：左竖肩、左横roll、右竖肘、右横yaw。
        Rig r;
        auto o = r.start();
        CHECK(o.active && o.request_enable && !o.disable_arm);
        r.rc.frame.channel[3] = 660;
        r.rc.frame.channel[2] = 660;
        r.rc.frame.channel[1] = -660;
        r.rc.frame.channel[0] = -660;
        o = r.tick();
        CHECK(o.mode == RobotMode::Grab && o.stop_chassis);
        CHECK(o.target_deg[BigPitch] > 0 && o.target_deg[Roll] > 0);
        CHECK(o.target_deg[SmallPitch] < 0 && o.target_deg[Yaw] < 0);
        CHECK(o.turn_rate_request == 0 && o.longitudinal_speed_request == 0);
        const float pitch = o.target_deg[BigPitch];
        r.neutral();
        o = r.tick();
        CHECK(o.target_deg[BigPitch] == pitch && pitch != 0);
    }
    { // 行驶左横只动yaw；右横只动底盘。纵向和独立yaw可同时操作。
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        CHECK(r.start().active);
        r.rc.frame.channel[2] = 660;
        auto o = r.tick();
        CHECK(near(o.target_deg[Yaw], 0.008f));
        CHECK(o.turn_rate_request == 0 && o.longitudinal_speed_request == 0);
        CHECK(o.target_deg[Roll] == 0 && o.target_deg[BigPitch] == 0 &&
              o.target_deg[SmallPitch] == 0);
        const float yaw = o.target_deg[Yaw];
        r.rc.frame.channel[2] = 0;
        r.rc.frame.channel[0] = 660;
        o = r.tick();
        CHECK(o.target_deg[Yaw] == yaw && o.turn_rate_request == -1);
        CHECK(o.longitudinal_brake && !o.stop_chassis && !o.emergency_stop);
        r.rc.frame.channel[3] = 660;
        r.rc.frame.channel[1] = 660;
        r.rc.frame.channel[2] = 660;
        o = r.tick();
        CHECK(o.longitudinal_speed_request == 1 && o.turn_rate_request == -1 &&
              !o.longitudinal_brake);
        CHECK(o.target_deg[Yaw] > yaw && o.target_deg[SmallPitch] == 0);
        CHECK(!o.reset_chassis_motion);
        r.rc.frame.channel[3] = -660;
        o = r.tick();
        CHECK(o.longitudinal_speed_request == -1 &&
              o.turn_rate_request == -1); // 倒车仍右转，不反号。
        r.rc.frame.channel[0] = -660;
        o = r.tick();
        CHECK(o.turn_rate_request == 1);
        r.neutral();
        o = r.tick();
        CHECK(o.longitudinal_brake && o.stop_chassis && o.turn_rate_request == 0 &&
              o.longitudinal_speed_request == 0);
    }
    { // 上、中慢，下快；拒绝快速只降档，不能清除人工请求或缩放归一化意图。
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        CHECK(r.start().active);
        r.rc.frame.channel[3] = 660;
        auto o = r.tick();
        CHECK(o.drive_gear == DriveGear::Slow && !o.fast_requested &&
              o.longitudinal_speed_request == 1);
        r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_UP;
        o = r.tick();
        CHECK(o.drive_gear == DriveGear::Slow && !o.fast_requested);
        r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_DOWN;
        o = r.tick();
        CHECK(o.drive_gear == DriveGear::Fast && o.fast_requested &&
              o.longitudinal_speed_request == 1);
        r.locks.arm_allows_fast = false;
        o = r.tick();
        CHECK(o.drive_gear == DriveGear::Slow && o.fast_requested && o.active);
        r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
        o = r.tick();
        CHECK(!o.fast_requested && o.drive_gear == DriveGear::Slow);
    }
    { // 行驶->夹取：未停稳或未回中均不能放行。
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        CHECK(r.start().active);
        r.rc.frame.channel[3] = 660;
        r.locks.chassis_stopped = false;
        auto o = r.tick();
        CHECK(o.active && !o.stop_chassis); // 正常行驶不永久要求stopped。
        const auto epoch = o.motion_epoch;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
        o = r.tick();
        CHECK(o.block == OperatorBlock::WaitChassisStop && o.stop_chassis &&
              o.reset_chassis_motion);
        CHECK(o.motion_epoch != epoch && !o.disable_arm);
        r.neutral();
        for (unsigned n = 0; n < 5; ++n) {
            o = r.tick();
            CHECK(o.wait_neutral && o.stop_chassis);
        }
        r.locks.chassis_stopped = true;
        r.tick();
        r.tick();
        o = r.tick();
        CHECK(!o.wait_neutral);
        r.rc.frame.channel[3] = 660;
        o = r.tick();
        CHECK(o.target_deg[BigPitch] > 0);
    }
    { // 夹取->行驶：旧夹紧下档不能成为快速；双向切换都确认实际停车。
        Rig r;
        CHECK(r.start().active);
        r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_DOWN;
        r.tick();
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        r.locks.chassis_stopped = false;
        auto o = r.tick();
        CHECK(o.stop_chassis && o.reset_chassis_motion && !o.fast_requested);
        CHECK(o.block == OperatorBlock::WaitChassisStop);
        r.locks.chassis_stopped = true;
        for (unsigned n = 0; n < 6; ++n) {
            o = r.tick();
            CHECK(o.wait_neutral && o.drive_gear == DriveGear::Slow);
        }
        r.neutral();
        r.tick();
        r.tick();
        o = r.tick();
        CHECK(!o.wait_neutral && o.drive_gear == DriveGear::Slow);
        r.rc.frame.channel[3] = 660;
        r.locks.chassis_stopped = false;
        o = r.tick();
        CHECK(!o.stop_chassis && o.active);
    }
    { // 待机进入时固定反馈目标一次；任意摇杆和右拨杆都不能再产生运动意图。
        Rig r;
        CHECK(r.start().active);
        r.arm.joint[Yaw].position_deg = 7;
        r.arm.joint[BigPitch].position_deg = 3;
        r.arm.joint[SmallPitch].position_deg = -2;
        r.arm.joint[Roll].position_deg = 4;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
        for (auto &v : r.rc.frame.channel)
            v = 660;
        auto o = r.tick();
        CHECK(o.mode == RobotMode::Standby && o.active && !o.disable_arm);
        CHECK(o.stop_chassis && o.reset_chassis_motion && !o.emergency_stop && !o.request_enable);
        const auto entered = o;
        r.arm.joint[Yaw].position_deg = 8;
        r.arm.joint[Gripper].position_deg = 23;
        for (unsigned n = 0; n < 300; ++n) {
            r.rc.frame.switches[kOperatorConfig.right_switch_index] = 1 + n % 3;
            for (unsigned j = 0; j < 4; ++j)
                r.rc.frame.channel[j] = (n + j) % 2 ? -660 : 660;
            o = r.tick(true);
            CHECK(o.active && !o.disable_arm && !o.request_enable && o.stop_chassis);
            CHECK(o.longitudinal_speed_request == 0 && o.turn_rate_request == 0 &&
                  o.longitudinal_brake);
            CHECK(!o.fast_requested && o.drive_gear == DriveGear::Slow);
            CHECK(o.motion_epoch == entered.motion_epoch);
            for (unsigned j = 0; j < kJointCount; ++j)
                CHECK(o.target_deg[j] == entered.target_deg[j]);
            CHECK(o.gripper_effort_limit == r.config.gripper_hold_current);
        }
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        o = r.tick();
        CHECK(o.wait_neutral && o.stop_chassis);
        r.neutral();
        r.tick();
        r.tick();
        o = r.tick();
        CHECK(o.active && !o.wait_neutral);
    }
    { // 初始失能或关控后进入待机，包含显式开控请求，也不能自动使能。
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
        for (unsigned n = 0; n < 10; ++n) {
            const auto o = r.tick(true);
            CHECK(!o.active && o.disable_arm && !o.request_enable && o.stop_chassis);
        }
        Rig a;
        CHECK(a.start().active);
        a.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
        a.tick();
        auto o = a.tick(false, true);
        CHECK(!o.active && o.disable_arm && o.emergency_stop);
        for (unsigned n = 0; n < 5; ++n) {
            o = a.tick(true);
            CHECK(!o.active && o.disable_arm && !o.request_enable);
        }
    }
    { // 长时间待机与真实控制核心接合：保持目标仍刷新，不能触发250ms命令超时。
        const auto cfg = bench();
        ArmControl arm(cfg);
        ArmOperator op(cfg, settings());
        Rig input(cfg);
        Frame frames[kJointCount]{};
        feed(arm, 0);
        CHECK(arm.capture_zero(kAllJoints, 0) == Result::Ok);
        for (uint32_t t = 0; t < 1400; t += 4) {
            feed(arm, t);
            input.rc.received_ms = t;
            input.locks.received_ms = t;
            if (t >= 20)
                input.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
            const auto o = op.step(input.rc, input.locks, arm.status(), t, t == 12, false);
            if (o.request_enable) {
                const bool ok = arm.enable(kAllJoints, t) == Result::Ok;
                op.enable_result(ok);
                CHECK(ok);
            }
            if (o.disable_arm)
                arm.disable();
            else {
                CHECK(arm.set_targets(kAllJoints, o.target_deg, t) == Result::Ok);
            }
            arm.step(t, frames);
            if (t >= 20) {
                CHECK(o.mode == RobotMode::Standby && o.active && !o.disable_arm);
                CHECK(arm.status().enabled_mask == kAllJoints && arm.status().faults == 0);
            }
        }
    }
    { // 行驶yaw只按经许可范围/速度调整，禁用后不追着反馈重设保持目标。
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        r.arm.joint[Yaw].position_deg = 19.999f;
        CHECK(r.start().active);
        r.rc.frame.channel[2] = 660;
        auto o = r.tick();
        CHECK(near(o.target_deg[Yaw], 20));
        o = r.tick();
        CHECK(near(o.target_deg[Yaw], 20));
        r.rc.frame.channel[2] = -660;
        o = r.tick();
        CHECK(near(o.target_deg[Yaw], 19.992f));
        r.locks.drive_yaw_allowed = false;
        r.arm.joint[Yaw].position_deg = 18;
        r.rc.frame.channel[0] = 660;
        o = r.tick();
        CHECK(o.drive_yaw_limited && o.target_deg[Yaw] == 18);
        CHECK(o.turn_rate_request == -1 && o.active);
        r.arm.joint[Yaw].position_deg = 18.5f;
        o = r.tick();
        CHECK(o.target_deg[Yaw] == 18);
    }
    { // 许可收紧不能强拉yaw进入新范围，需停车等待处理。
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        r.arm.joint[Yaw].position_deg = 15;
        CHECK(r.start().active);
        r.locks.drive_yaw_max_deg = 10;
        r.rc.frame.channel[3] = 660;
        const auto o = r.tick();
        CHECK(o.stop_chassis && o.reset_chassis_motion && o.longitudinal_speed_request == 0);
        CHECK(o.target_deg[Yaw] >= 15); // 不能凭修改限位命令yaw自动转到10度。
    }
    { // 关控、遥控丢失、互锁丢失优先于任何模式；恢复报文不自动重新开控。
        for (auto mode : {DR16_SWITCH_UP, DR16_SWITCH_MID, DR16_SWITCH_DOWN}) {
            Rig r;
            CHECK(r.start().active);
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = mode;
            r.tick();
            r.rc.valid = false;
            auto o = r.tick();
            CHECK(o.disable_arm && !o.active && o.stop_chassis && o.emergency_stop);
            CHECK(o.reset_chassis_motion && o.turn_rate_request == 0 &&
                  o.longitudinal_speed_request == 0);
            r.rc.valid = true;
            r.neutral();
            r.tick();
            r.tick();
            o = r.tick();
            CHECK(!o.active && o.disable_arm);
        }
        Rig r;
        CHECK(r.start().active);
        r.rc.valid = false;
        r.tick();
        r.rc.valid = true;
        r.tick();
        r.tick();
        r.tick();
        CHECK(r.tick(true).active);
        r.now += 101;
        auto o = r.tick(false, false, false);
        CHECK(o.disable_arm && o.block == OperatorBlock::RemoteOffline);
        Rig a;
        CHECK(a.start().active);
        a.locks.valid = false;
        o = a.tick();
        CHECK(o.disable_arm && o.block == OperatorBlock::InterlockOffline);
        Rig b;
        CHECK(b.start().active);
        o = b.tick(false, true);
        CHECK(o.disable_arm && !o.active);
    }
    { // 姿态许可丢失后停车，需停稳并回中；已经使能的关节仍保持。
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        CHECK(r.start().active);
        r.rc.frame.channel[3] = 660;
        r.locks.chassis_stopped = false;
        const auto epoch = r.tick().motion_epoch;
        r.locks.arm_allows_drive = false;
        auto o = r.tick();
        CHECK(o.stop_chassis && o.reset_chassis_motion && o.active && !o.disable_arm &&
              o.motion_epoch != epoch);
        r.locks.arm_allows_drive = true;
        for (unsigned n = 0; n < 4; ++n) {
            o = r.tick();
            CHECK(o.stop_chassis);
        }
        r.neutral();
        r.locks.chassis_stopped = true;
        r.tick();
        r.tick();
        o = r.tick();
        CHECK(!o.wait_neutral && o.active);
    }
    { // 使能/目标提交被下层拒绝后，新的显式请求也不能绕过完整回中时间。
        for (unsigned failure = 0; failure < 2; ++failure) {
            Rig r;
            auto o = r.start();
            CHECK(o.active);
            if (failure) {
                r.rc.frame.channel[3] = 660;
                o = r.tick();
                CHECK(o.target_deg[BigPitch] > 0);
            }
            const uint32_t before = o.motion_epoch;
            const uint32_t rejected_epoch = r.op.enable_result(false);
            r.arm.enabled_mask = 0;
            CHECK(rejected_epoch != before);
            if (failure) {
                for (unsigned n = 0; n < 5; ++n) {
                    o = r.tick(true);
                    CHECK(o.wait_neutral && !o.request_enable && !o.active);
                }
            }
            r.neutral();
            o = r.tick(true);
            CHECK(o.wait_neutral && !o.request_enable && !o.active &&
                  o.motion_epoch == rejected_epoch);
            CHECK(o.reset_chassis_motion && o.stop_chassis);
            o = r.tick(true);
            CHECK(o.wait_neutral && !o.request_enable && !o.active);
            o = r.tick(true);
            CHECK(!o.wait_neutral && o.request_enable && o.active);
            CHECK(o.target_deg[BigPitch] == r.arm.joint[BigPitch].position_deg);
        }
    }
    { // 未经核定或畸形yaw包络不能作为允许运动的依据。
        for (unsigned bad = 0; bad < 6; ++bad) {
            Rig r;
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            CHECK(r.start().active);
            if (bad == 0)
                r.locks.drive_yaw_min_deg = std::numeric_limits<float>::quiet_NaN();
            if (bad == 1)
                r.locks.drive_yaw_max_deg = std::numeric_limits<float>::infinity();
            if (bad == 2)
                r.locks.drive_yaw_max_speed_deg_s = 0;
            if (bad == 3)
                r.locks.drive_yaw_max_speed_deg_s = 11;
            if (bad == 4)
                r.locks.drive_yaw_min_deg = 25;
            if (bad == 5)
                r.locks.drive_yaw_max_deg = 91;
            const auto o = r.tick();
            CHECK(o.disable_arm && o.emergency_stop && o.stop_chassis && !o.active);
            CHECK(o.block == OperatorBlock::InvalidInterlock && o.turn_rate_request == 0);
        }
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        r.locks.drive_yaw_allowed = false;
        r.locks.drive_yaw_min_deg = 0;
        r.locks.drive_yaw_max_deg = 0;
        r.locks.drive_yaw_max_speed_deg_s = 0;
        CHECK(r.start().active);
        r.rc.frame.channel[2] = 660;
        r.rc.frame.channel[0] = 660;
        const auto o = r.tick();
        CHECK(o.target_deg[Yaw] == 0 && o.drive_yaw_limited && o.turn_rate_request == -1);
    }
    { // Tick回绕不会跳过回中检查，也不能把新报文判为失联。
        Rig r;
        r.now = 0xfffffff8u;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        CHECK(r.start().active);
        CHECK(r.now < 32);
        r.rc.frame.channel[0] = 660;
        const auto o = r.tick();
        CHECK(o.active && o.turn_rate_request == -1);
    }
    { // 夹爪上/中/下及原有闭合超时、电流限制保持。
        Rig r;
        CHECK(r.start().active);
        r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_UP;
        auto o = r.tick();
        CHECK(o.target_deg[Gripper] < 22.5f);
        r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
        o = r.tick();
        CHECK(o.target_deg[Gripper] == 22.5f);
        r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_DOWN;
        o = r.tick();
        CHECK(o.target_deg[Gripper] > 22.5f && o.gripper_effort_limit == 500);
        r.tick();
        r.tick();
        o = r.tick();
        CHECK(o.gripper_effort_limit == 150 && o.target_deg[Gripper] == 22.5f);
        o = r.tick();
        CHECK(o.gripper_effort_limit == 150 && o.target_deg[Gripper] == 22.5f);
    }
    { // 肩与肘同时向组合边界推进，不能因a2=q1+q2超范围意外关控。
        Rig r;
        ArmControl validator(r.arm_config);
        const float second_drive_max_deg = r.arm_config.pitch.second_drive_max_deg;
        const float near_boundary_deg = second_drive_max_deg * 0.5f - 0.01f;
        r.arm.joint[BigPitch].position_deg = near_boundary_deg;
        r.arm.joint[SmallPitch].position_deg = near_boundary_deg;
        CHECK(r.start().active);
        r.rc.frame.channel[3] = 660;
        r.rc.frame.channel[1] = 660;
        for (unsigned i = 0; i < 250; ++i) {
            const auto o = r.tick();
            CHECK(o.active && !o.disable_arm);
            CHECK(o.target_deg[BigPitch] <= kArmConfig.joint[BigPitch].max_deg + 0.0001f);
            CHECK(o.target_deg[SmallPitch] <= kArmConfig.joint[SmallPitch].max_deg + 0.0001f);
            CHECK(o.target_deg[BigPitch] + o.target_deg[SmallPitch] <=
                  kArmConfig.pitch.second_drive_max_deg + 0.0001f);
            CHECK(validator.set_targets(kAllJoints, o.target_deg, r.now) == Result::Ok);
        }
        r.rc.frame.channel[3] = -660;
        r.rc.frame.channel[1] = -660;
        const auto o = r.tick();
        CHECK(o.active && !o.disable_arm);
        CHECK(o.target_deg[BigPitch] + o.target_deg[SmallPitch] < second_drive_max_deg);
        CHECK(validator.set_targets(kAllJoints, o.target_deg, r.now) == Result::Ok);
    }
    { // 非默认范围下冻结反馈：肩80deg不能给肘构造[-10,-50]这类倒置区间。
        auto cfg = vehicle();
        cfg.joint[BigPitch].min_deg = -90;
        cfg.joint[BigPitch].max_deg = 90;
        cfg.joint[SmallPitch].min_deg = -10;
        cfg.joint[SmallPitch].max_deg = 10;
        cfg.pitch.second_drive_min_deg = -30;
        cfg.pitch.second_drive_max_deg = 30;
        Rig r(cfg);
        ArmControl validator(cfg);
        r.arm.joint[BigPitch].position_deg = 80;
        auto o = r.tick();
        CHECK(!o.active && o.disable_arm);
        CHECK(std::isfinite(o.target_deg[BigPitch]) && std::isfinite(o.target_deg[SmallPitch]));
        CHECK(o.target_deg[BigPitch] >= -40 && o.target_deg[BigPitch] <= 40);
        CHECK(o.target_deg[SmallPitch] >= -10 && o.target_deg[SmallPitch] <= 10);
        CHECK(validator.set_targets(kAllJoints, o.target_deg, r.now) == Result::Ok);
        r.rc.valid = false;
        r.arm.joint[BigPitch].position_deg = -80;
        o = r.tick();
        CHECK(!o.active && o.disable_arm);
        CHECK(std::isfinite(o.target_deg[BigPitch]) && std::isfinite(o.target_deg[SmallPitch]));
        CHECK(o.target_deg[BigPitch] >= -40 && o.target_deg[BigPitch] <= 40);
        CHECK(o.target_deg[SmallPitch] >= -10 && o.target_deg[SmallPitch] <= 10);
        CHECK(validator.set_targets(kAllJoints, o.target_deg, r.now) == Result::Ok);
    }
    { // 不等输入与反向切换反复触及组合边界，输出必须通过底层严格浮点检查。
        Rig r;
        ArmControl validator(r.arm_config);
        const float half_drive_max_deg = r.arm_config.pitch.second_drive_max_deg * 0.5f;
        r.arm.joint[BigPitch].position_deg = half_drive_max_deg - 0.87655f;
        r.arm.joint[SmallPitch].position_deg = half_drive_max_deg + 0.87622f;
        CHECK(r.start().active);
        const int inputs[][2] = {{660, 330},  {330, -660}, {-231, 660}, {-660, -330},
                                 {660, -231}, {-330, 660}, {660, 330},  {-231, -660}};
        for (const auto &input : inputs) {
            r.rc.frame.channel[3] = input[0];
            r.rc.frame.channel[1] = input[1];
            for (unsigned i = 0; i < 200; ++i) {
                const auto o = r.tick();
                CHECK(o.active && !o.disable_arm);
                CHECK(std::isfinite(o.target_deg[BigPitch]) &&
                      std::isfinite(o.target_deg[SmallPitch]));
                CHECK(validator.set_targets(kAllJoints, o.target_deg, r.now) == Result::Ok);
            }
        }
    }
    { // 未确认映射、非法拨杆、控制周期异常均拒绝开控。
        OperatorConfig c = settings();
        c.mapping_confirmed = false;
        ArmOperator a(vehicle(), c);
        Rig r;
        auto o = a.step(r.rc, r.locks, r.arm, 0, true, false);
        CHECK(o.disable_arm && o.block == OperatorBlock::InvalidConfig);
        r.start();
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = 0;
        o = r.tick();
        CHECK(o.disable_arm);
        Rig t;
        CHECK(t.start().active);
        t.now += 20;
        o = t.tick();
        CHECK(o.disable_arm && o.block == OperatorBlock::Timing);
    }
    { // An explicit tuning exit resumes the existing arm without a stale-period stop or re-enable.
        for (uint8_t left : {uint8_t(DR16_SWITCH_UP), uint8_t(DR16_SWITCH_MID)}) {
            Rig r;
            CHECK(r.start().active);
            r.rc.frame.channel[3] = 660;
            r.tick(); // Leave an old normal-operation target that must not be reused.
            r.neutral();
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = left;
            r.arm.joint[Yaw].position_deg = 2;
            r.arm.joint[BigPitch].position_deg = 6;
            r.arm.joint[SmallPitch].position_deg = -2;
            r.arm.joint[Roll].position_deg = 5;
            r.now += 100000;
            const auto original = r.arm;
            r.op.resume_from_tuning(r.arm, r.rc, r.now);
            CHECK(std::memcmp(&original, &r.arm, sizeof(original)) == 0);
            const auto o = r.tick(true);
            CHECK(o.active && !o.request_enable && !o.disable_arm && !o.emergency_stop);
            CHECK(!o.wait_neutral && o.block == OperatorBlock::None);
            CHECK(o.mode == (left == DR16_SWITCH_UP ? RobotMode::Drive : RobotMode::Grab));
            CHECK(r.arm.enabled_mask == kAllJoints);
            CHECK(o.stop_chassis && o.longitudinal_speed_request == 0 && o.turn_rate_request == 0);
            for (unsigned i = 0; i < kJointCount; ++i)
                CHECK(o.target_deg[i] == r.arm.joint[i].position_deg);
        }
    }
    { // Resuming a partial hold never enables it implicitly; only the following request can add axes.
        for (bool request : {false, true}) {
            Rig r;
            CHECK(r.start().active);
            r.arm.enabled_mask = kFourAxisJoints;
            r.now += 100000;
            r.op.resume_from_tuning(r.arm, r.rc, r.now);
            CHECK(r.arm.enabled_mask == kFourAxisJoints);
            r.rc.received_ms = r.locks.received_ms = r.now;
            const auto o = r.op.step(r.rc, r.locks, r.arm, r.now, request, false);
            CHECK(r.arm.enabled_mask == kFourAxisJoints);
            CHECK(o.request_enable == request && o.active == request &&
                  o.disable_arm == !request);
            CHECK(o.block == (request ? OperatorBlock::None : OperatorBlock::NeedArm));
        }
    }
    { // Resume resets timing/state only: normal remote, interlock, axis health and stop checks still run.
        for (unsigned bad = 0; bad < 7; ++bad) {
            Rig r;
            CHECK(r.start().active);
            r.now += 100000;
            r.op.resume_from_tuning(r.arm, r.rc, r.now);
            if (bad == 0)
                r.rc.valid = false;
            if (bad == 1)
                r.locks.valid = false;
            if (bad == 2)
                r.arm.configuration_valid = false;
            if (bad == 3)
                r.arm.faults = FeedbackTimeout;
            if (bad == 4)
                r.arm.joint[Roll].online = false;
            if (bad == 5)
                r.arm.joint[Yaw].calibrated = false;
            const auto o = r.tick(true, bad == 6);
            CHECK(!o.active && o.disable_arm && o.emergency_stop && !o.request_enable);
            CHECK(o.stop_chassis && o.longitudinal_speed_request == 0 && o.turn_rate_request == 0);
            if (bad >= 2 && bad <= 5)
                CHECK(o.block == OperatorBlock::ArmUnavailable);
        }
        Rig r;
        CHECK(r.start().active);
        r.now += 100000;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        r.locks.arm_allows_drive = false;
        r.op.resume_from_tuning(r.arm, r.rc, r.now);
        const auto o = r.tick(true);
        CHECK(o.active && !o.disable_arm && !o.request_enable && o.stop_chassis);
        CHECK(o.block == OperatorBlock::PostureDenied && o.wait_neutral);
    }
    return 0;
}
