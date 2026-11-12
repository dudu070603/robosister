#include "Arm/arm_control.h"
#include "Remote/operator_control.h"
#include <cmath>
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x))                                                                                  \
            return __LINE__;                                                                       \
    } while (0)
namespace {
Config four_axis(bool confirmed = true) {
    auto c = kArmConfig;
    c.joint_mask = kFourAxisJoints;
    c.hardware_confirmed = confirmed;
    c.pitch.mapping_confirmed = confirmed;
    return c;
}
OperatorConfig controls() {
    auto c = kOperatorConfig;
    c.mapping_confirmed = true; // Test fixture only; production remains false.
    c.neutral_ms = 300;
    return c;
}
void feed_four(ArmControl &a, uint32_t ms) {
    const uint8_t dji[8] = {3, 232, 0, 0, 0, 0, 25, 0};
    a.receive(1, a.feedback_id(Roll), dji, 8, ms);
    a.receive(1, a.feedback_id(Gripper), dji, 8, ms);
    for (unsigned j = BigPitch; j <= SmallPitch; ++j) {
        const uint8_t dm[8] = {uint8_t(0x10 + j), 0x80, 0, 0x80, 0x08, 0, 25, 25};
        a.receive(1, a.feedback_id(j), dm, 8, ms);
    }
}
bool no_yaw_frames(const Frame *f, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        if (f[i].bus != 1 || (f[i].id != 1 && f[i].id != 2 && f[i].id != 0x200))
            return false;
    return true;
}
constexpr uint16_t kDmEncoderMidpoint = 0x8000;
void feed_jog_dm(ArmControl &arm, uint32_t ms, unsigned joint, bool enabled,
                 uint16_t position = kDmEncoderMidpoint, uint16_t velocity = 0x800) {
    const unsigned state = enabled ? 0x10 : 0;
    const uint8_t dm[8] = {uint8_t(state | joint), uint8_t(position >> 8), uint8_t(position),
                           uint8_t(velocity >> 4), uint8_t(((velocity & 0xf) << 4) | 0x08),
                           0, 25, 25};
    arm.receive(1, arm.feedback_id(joint), dm, 8, ms);
}
void feed_jog(ArmControl &arm, uint32_t ms, unsigned dji_mask = 0, unsigned enabled_dm = Yaw,
              uint16_t big_position = kDmEncoderMidpoint,
              uint16_t small_position = kDmEncoderMidpoint) {
    feed_jog_dm(arm, ms, BigPitch, enabled_dm == BigPitch, big_position);
    feed_jog_dm(arm, ms, SmallPitch, enabled_dm == SmallPitch, small_position);
    const uint8_t dji[8] = {3, 232, 0, 0, 0, 0, 25, 0};
    for (unsigned joint = Roll; joint <= Gripper; ++joint)
        if (dji_mask & (1u << joint))
            arm.receive(1, arm.feedback_id(joint), dji, 8, ms);
}
const Frame *jog_frame(const Frame *frames, unsigned count, unsigned id) {
    for (unsigned index = 0; index < count; ++index)
        if (frames[index].id == id)
            return &frames[index];
    return nullptr;
}
bool dm_command(const Frame *frames, unsigned count, unsigned id, uint8_t command) {
    const auto *frame = jog_frame(frames, count, id);
    if (!frame || frame->data[7] != command)
        return false;
    for (unsigned index = 0; index < 7; ++index)
        if (frame->data[index] != 0xff)
            return false;
    return true;
}
int16_t jog_current(const Frame &frame, unsigned slot) {
    const auto *data = &frame.data[slot * 2];
    return static_cast<int16_t>((unsigned(data[0]) << 8) | data[1]);
}
bool jog_stopped(const Frame *frames, unsigned count) {
    const auto *dji = jog_frame(frames, count, 0x200);
    return count == 3 && no_yaw_frames(frames, count) && dji &&
           !jog_current(*dji, 0) && !jog_current(*dji, 1) && !jog_current(*dji, 2) &&
           !jog_current(*dji, 3) && dm_command(frames, count, 1, 0xfd) &&
           dm_command(frames, count, 2, 0xfd);
}
struct Rig {
    Config config = four_axis();
    OperatorConfig operator_config = controls();
    ArmOperator op{config, operator_config};
    Status status{};
    RemoteInput rc{};
    Interlocks locks{};
    uint32_t now = 0;
    Rig() {
        status.configuration_valid = true;
        for (unsigned j = BigPitch; j < kJointCount; ++j) {
            status.joint[j].online = true;
            status.joint[j].calibrated = true;
        }
        status.joint[Yaw].position_deg =
            80; // Physically absent and uncalibrated; must never gate four-axis control.
        status.joint[Gripper].position_deg = 22.5f;
        rc.valid = true;
        rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
        rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
        locks.valid = false;
        locks.drive_yaw_allowed = true;
        locks.drive_yaw_min_deg =
            std::numeric_limits<float>::quiet_NaN(); // Bench ignores the chassis envelope.
    }
    OperatorOutput tick(bool explicit_request = false, bool stop = false) {
        rc.received_ms = now;
        const auto o = op.step(rc, locks, status, now, explicit_request, stop);
        if (o.request_enable) {
            status.enabled_mask = kFourAxisJoints;
            op.enable_result(true);
        }
        if (o.disable_arm)
            status.enabled_mask = 0;
        now += 4;
        return o;
    }
    OperatorOutput start_explicit() {
        for (unsigned n = 0; n < 75; ++n)
            tick();
        return tick(true);
    }
    void neutral() {
        for (auto &v : rc.frame.channel)
            v = 0;
        rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_MID;
    }
};
} // namespace
int test_four_axis() {
    CHECK(kFourAxisJoints == 0x1e && !(kFourAxisJoints & (1u << Yaw)));
#if ARM_FOUR_AXIS_BENCH
    CHECK(kArmConfig.joint_mask == kFourAxisJoints);
#else
    CHECK(kArmConfig.joint_mask == kAllJoints);
#endif
    CHECK(!kArmConfig.hardware_confirmed && !kArmConfig.pitch.mapping_confirmed &&
          !kOperatorConfig.mapping_confirmed);
    Frame frames[kJointCount]{};
    { // Four-axis default gates remain closed, and even zero/disable output must never address yaw.
        ArmControl a(four_axis(false));
        CHECK(a.status().configuration_valid);
        CHECK(a.enable(kFourAxisJoints, 0) == Result::ConfigUnconfirmed);
        const unsigned count = a.step(0, frames);
        CHECK(count == 3 && no_yaw_frames(frames, count));
        for (unsigned i = 0; i < count; ++i) {
            if (frames[i].id == 0x200) {
                for (auto b : frames[i].data)
                    CHECK(b == 0);
            } else
                CHECK(frames[i].data[7] == 0xfd);
        }
    }
    { // Disabled-axis API requests are rejected atomically. A yaw CAN frame is not accepted as
      // feedback.
        ArmControl a(four_axis());
        feed_four(a, 0);
        const uint8_t yaw[8] = {0x10, 0, 0, 0, 0, 0, 90, 0};
        CHECK(!a.receive(1, 0x205, yaw, 8, 0));
        CHECK(a.status().joint[Yaw].received == 0);
        CHECK(a.capture_zero(1u << Yaw, 0) == Result::InvalidArgument);
        CHECK(a.capture_zero(kAllJoints, 0) == Result::InvalidArgument);
        CHECK(a.enable(1u << Yaw, 0) == Result::InvalidArgument);
        CHECK(a.enable(kAllJoints, 0) == Result::InvalidArgument);
        CHECK(a.set_effort_limit(Yaw, 1) == Result::InvalidArgument);
        float q[kJointCount] = {1, 1, 1, 1, 1};
        CHECK(a.set_targets(1u << Yaw, q, 0) == Result::InvalidArgument);
        CHECK(a.set_targets(kAllJoints, q, 0) == Result::InvalidArgument);
        for (const auto &j : a.status().joint)
            CHECK(j.target_deg == 0);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        CHECK(a.capture_zero(1u << BigPitch, 0) == Result::InvalidArgument);
        CHECK(a.capture_zero(kFourAxisJoints, 0) == Result::Ok);
        CHECK(!a.status().joint[Yaw].calibrated && !a.status().joint[Yaw].online);
        CHECK(a.enable(kFourAxisJoints, 0) == Result::Ok);
        q[Yaw] = std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_targets(kFourAxisJoints, q, 0) ==
              Result::Ok); // Unselected absent yaw value is irrelevant.
        CHECK(a.status().joint[Yaw].target_deg == 0);
        CHECK(a.set_effort_limit(Gripper, 150) == Result::Ok);
        for (uint32_t ms = 0; ms <= 400; ms += 4) {
            feed_four(a, ms);
            CHECK(a.set_targets(kFourAxisJoints, q, ms) == Result::Ok);
            const auto n = a.step(ms, frames);
            CHECK(n == 3 && no_yaw_frames(frames, n));
            CHECK(a.status().enabled_mask == kFourAxisJoints && !a.status().faults);
        }
        a.disable();
        const auto n = a.step(404, frames);
        CHECK(n == 3 && no_yaw_frames(frames, n));
    }
    { // Missing one real axis still blocks enable; yaw absence is the only removed dependency.
        ArmControl a(four_axis());
        const uint8_t dji[8] = {3, 232, 0, 0, 0, 0, 0, 0};
        CHECK(a.feedback_id(Roll) == 0x203 && a.feedback_id(Gripper) == 0x204);
        CHECK(!a.receive(1, 0x201, dji, 8, 0));
        CHECK(!a.receive(1, 0x202, dji, 8, 0));
        CHECK(a.receive(1, a.feedback_id(Roll), dji, 8, 0));
        CHECK(a.capture_zero(kFourAxisJoints, 0) == Result::NotOnline);
        CHECK(a.enable(kFourAxisJoints, 0) == Result::NotOnline);
    }
    { // Starting in MID never auto-arms, including long neutral input; explicit request is still
      // allowed.
        Rig r;
        for (unsigned n = 0; n < 100; ++n) {
            const auto o = r.tick();
            CHECK(!o.active && !o.request_enable && o.disable_arm);
            CHECK(o.stop_chassis && o.emergency_stop && o.turn_rate_request == 0 &&
                  o.longitudinal_speed_request == 0);
        }
        const auto o = r.tick(true);
        CHECK(o.request_enable && o.active && !o.disable_arm);
        CHECK(o.emergency_stop && o.stop_chassis && o.target_deg[Yaw] == 0);
    }
    { // Healthy UP->MID requires the complete 300ms neutral interval and produces one enable
      // request.
        Rig r;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        auto o = r.tick(true);
        CHECK(o.disable_arm && !o.request_enable && !o.active);
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
        for (unsigned n = 0; n < 75; ++n) {
            o = r.tick();
            CHECK(!o.request_enable && !o.active);
        }
        o = r.tick();
        CHECK(o.request_enable && o.active && !o.disable_arm);
        o = r.tick();
        CHECK(!o.request_enable && o.active && o.stop_chassis && o.emergency_stop);
        for (unsigned j = 0; j < 4; ++j)
            r.rc.frame.channel[j] = 660;
        o = r.tick();
        CHECK(o.target_deg[BigPitch] > 0 && o.target_deg[SmallPitch] > 0 && o.target_deg[Roll] > 0);
        CHECK(o.target_deg[Yaw] == 0 && o.turn_rate_request == 0 &&
              o.longitudinal_speed_request == 0);
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
        o = r.tick();
        CHECK(!o.active && o.disable_arm && o.stop_chassis);
    }
    { // MID non-neutral cancels the entire auto candidate, not merely the neutral timer.
        for (unsigned input = 0; input < 5; ++input) {
            Rig r;
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            r.tick();
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
            for (unsigned n = 0; n < 10; ++n)
                r.tick();
            if (input < 4)
                r.rc.frame.channel[input] = 660;
            else
                r.rc.frame.switches[kOperatorConfig.right_switch_index] = DR16_SWITCH_DOWN;
            auto o = r.tick();
            CHECK(!o.active && !o.request_enable);
            r.neutral();
            for (unsigned n = 0; n < 100; ++n) {
                o = r.tick();
                CHECK(!o.active && !o.request_enable);
            }
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            r.tick();
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
            for (unsigned n = 0; n < 75; ++n)
                r.tick();
            o = r.tick();
            CHECK(o.active && o.request_enable);
        }
    }
    { // Starting MID non-neutral or passing through standby cannot retain UP permission for later
      // use.
        for (unsigned path = 0; path < 2; ++path) {
            Rig r;
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            r.tick();
            if (path) {
                r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
                r.tick();
            }
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
            if (!path)
                r.rc.frame.channel[3] = 660;
            r.tick();
            r.neutral();
            for (unsigned n = 0; n < 100; ++n) {
                const auto o = r.tick();
                CHECK(!o.active && !o.request_enable);
            }
        }
    }
    { // Missing feedback/calibration, faults, or a rejected enable consume the candidate
      // permanently.
        for (unsigned reason = 0; reason < 5; ++reason) {
            Rig r;
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            r.tick();
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
            if (reason == 0)
                r.status.joint[Roll].online = false;
            if (reason == 1)
                r.status.joint[Gripper].calibrated = false;
            if (reason == 2)
                r.status.faults = FeedbackTimeout;
            if (reason == 3)
                r.status.configuration_valid = false;
            auto o = r.tick();
            if (reason == 4) {
                for (unsigned n = 0; n < 74; ++n)
                    r.tick();
                o = r.tick();
                CHECK(o.request_enable);
                r.op.enable_result(false);
                r.status.enabled_mask = 0;
            }
            r.status.joint[Roll].online = true;
            r.status.joint[Gripper].calibrated = true;
            r.status.faults = 0;
            r.status.configuration_valid = true;
            for (unsigned n = 0; n < 100; ++n) {
                o = r.tick();
                CHECK(!o.active && !o.request_enable);
            }
        }
    }
    { // Reconnection in MID, explicit stop, invalid timing or switch cannot revive a prior UP
      // candidate.
        for (unsigned reason = 0; reason < 4; ++reason) {
            Rig r;
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_UP;
            r.tick();
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
            for (unsigned n = 0; n < 10; ++n)
                r.tick();
            if (reason == 0)
                r.rc.valid = false;
            if (reason == 2)
                r.now += 20;
            if (reason == 3)
                r.rc.frame.switches[kOperatorConfig.left_switch_index] = 0;
            auto o = r.tick(false, reason == 1);
            CHECK(!o.active && !o.request_enable && o.disable_arm);
            r.rc.valid = true;
            r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_MID;
            for (unsigned n = 0; n < 100; ++n) {
                o = r.tick();
                CHECK(!o.active && !o.request_enable);
            }
        }
    }
    { // DOWN keeps existing four-axis hold, never enables an inactive arm; all chassis intent stays
      // inhibited.
        Rig r;
        CHECK(r.start_explicit().active);
        r.status.joint[BigPitch].position_deg = 3;
        r.status.joint[SmallPitch].position_deg = -2;
        r.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
        auto o = r.tick();
        const auto held = o;
        CHECK(o.active && !o.disable_arm && o.stop_chassis && o.emergency_stop);
        r.status.joint[BigPitch].position_deg = 4;
        for (unsigned n = 0; n < 100; ++n) {
            for (unsigned j = 0; j < 4; ++j)
                r.rc.frame.channel[j] = 660;
            r.rc.frame.switches[kOperatorConfig.right_switch_index] = 1 + n % 3;
            o = r.tick(true);
            CHECK(o.active && !o.request_enable && !o.disable_arm && o.emergency_stop &&
                  o.stop_chassis);
            for (unsigned j = 0; j < kJointCount; ++j)
                CHECK(o.target_deg[j] == held.target_deg[j]);
        }
        Rig disabled;
        disabled.rc.frame.switches[kOperatorConfig.left_switch_index] = DR16_SWITCH_DOWN;
        for (unsigned n = 0; n < 100; ++n) {
            o = disabled.tick(true);
            CHECK(!o.active && o.disable_arm && !o.request_enable);
        }
    }
    { // Real control core + bench interpreter: auto-enable only four axes with no yaw frames or
      // chassis task.
        const auto config = four_axis();
        ArmControl arm(config);
        ArmOperator op(config, controls());
        Rig input;
        feed_four(arm, 0);
        CHECK(arm.capture_zero(kFourAxisJoints, 0) == Result::Ok);
        for (uint32_t t = 0; t < 800; t += 4) {
            feed_four(arm, t);
            input.rc.received_ms = t;
            input.rc.frame.switches[kOperatorConfig.left_switch_index] =
                t == 0 ? DR16_SWITCH_UP : (t < 500 ? DR16_SWITCH_MID : DR16_SWITCH_DOWN);
            const auto o = op.step(input.rc, input.locks, arm.status(), t, false, false);
            if (o.request_enable) {
                const bool ok = arm.enable(kFourAxisJoints, t) == Result::Ok;
                op.enable_result(ok);
                CHECK(ok);
            }
            if (o.disable_arm)
                arm.disable();
            else
                CHECK(arm.set_targets(kFourAxisJoints, o.target_deg, t) == Result::Ok);
            const unsigned n = arm.step(t, frames);
            CHECK(n == 3 && no_yaw_frames(frames, n));
            CHECK(o.stop_chassis && o.emergency_stop && o.target_deg[Yaw] == 0);
            if (t >= 304)
                CHECK(o.active && arm.status().enabled_mask == kFourAxisJoints &&
                      !arm.status().faults);
        }
    }
    { // Pitch origins can be captured while stopped without any DJI or yaw feedback.
        ArmControl arm(four_axis(false));
        auto n = arm.step_jog(0, 0, false, frames);
        CHECK(jog_stopped(frames, n) && !arm.status().enabled_mask);
        feed_jog(arm, 4);
        n = arm.step_jog(4, 5, false, frames);
        CHECK(jog_stopped(frames, n) && arm.status().last_result == Result::Ok);
        CHECK(arm.status().joint[BigPitch].calibrated && arm.status().joint[SmallPitch].calibrated);
        CHECK(!arm.status().joint[Yaw].calibrated && !arm.status().joint[Roll].calibrated &&
              !arm.status().joint[Gripper].calibrated);
        feed_jog(arm, 8);
        n = arm.step_jog(8, 1, false, frames);
        CHECK(jog_stopped(frames, n) && !arm.status().enabled_mask);
    }
    { // Signed DJI pulses address only the selected CAN slot. Renewing a command stops it.
        ArmControl arm(four_axis(false));
        feed_jog(arm, 0, 1u << Roll);
        arm.step_jog(0, 5, false, frames);
        feed_jog(arm, 4, 1u << Roll);
        auto n = arm.step_jog(4, 3, true, frames);
        CHECK(arm.status().enabled_mask == (1u << Roll));
        feed_jog(arm, 8, 1u << Roll);
        n = arm.step_jog(8, 0, true, frames);
        const auto *dji = jog_frame(frames, n, 0x200);
        CHECK(n == 3 && no_yaw_frames(frames, n) && dji);
        CHECK(!jog_current(*dji, 0) && !jog_current(*dji, 1) && jog_current(*dji, 2) > 0 &&
              !jog_current(*dji, 3));
        CHECK(dm_command(frames, n, 1, 0xfd) && dm_command(frames, n, 2, 0xfd));
        feed_jog(arm, 12, 1u << Roll);
        n = arm.step_jog(12, 3, true, frames);
        CHECK(jog_stopped(frames, n) && arm.status().last_result == Result::Busy);
        feed_jog(arm, 16, 1u << Gripper);
        arm.step_jog(16, -4, true, frames);
        CHECK(arm.status().enabled_mask == (1u << Gripper));
        feed_jog(arm, 20, 1u << Gripper);
        n = arm.step_jog(20, 0, true, frames);
        dji = jog_frame(frames, n, 0x200);
        CHECK(dji && !jog_current(*dji, 0) && !jog_current(*dji, 1) &&
              !jog_current(*dji, 2) && jog_current(*dji, 3) < 0);
        feed_jog(arm, 24, 1u << Gripper);
        n = arm.step_jog(24, 0, false, frames);
        CHECK(jog_stopped(frames, n) && !arm.status().enabled_mask);
    }
    { // The maximum pulse duration expires even with live feedback and the RC interlock held.
        ArmControl arm(four_axis(false));
        feed_jog(arm, 0, 1u << Roll);
        arm.step_jog(0, 5, false, frames);
        feed_jog(arm, 4, 1u << Roll);
        arm.step_jog(4, 3, true, frames);
        for (uint32_t ms = 8; ms < 704; ms += 4) {
            feed_jog(arm, ms, 1u << Roll);
            const auto n = arm.step_jog(ms, 0, true, frames);
            const auto *dji = jog_frame(frames, n, 0x200);
            CHECK(dji && jog_current(*dji, 2) > 0);
        }
        feed_jog(arm, 704, 1u << Roll);
        auto n = arm.step_jog(704, 0, true, frames);
        CHECK(jog_stopped(frames, n));
        feed_jog(arm, 708, 1u << Roll);
        arm.step_jog(708, 3, true, frames);
        CHECK(arm.status().enabled_mask == (1u << Roll));
        feed_jog(arm, 712, 1u << Roll);
        n = arm.step_jog(712, 9, true, frames);
        CHECK(jog_stopped(frames, n));
    }
    { // A DM pulse enables only the selected drive and stops after 1 degree of travel.
        ArmControl arm(four_axis(false));
        feed_jog(arm, 0);
        auto n = arm.step_jog(0, 1, true, frames);
        CHECK(jog_stopped(frames, n) && arm.status().last_result == Result::NotCalibrated);
        feed_jog(arm, 4);
        arm.step_jog(4, 5, false, frames);
        feed_jog(arm, 8);
        n = arm.step_jog(8, 1, true, frames);
        CHECK(dm_command(frames, n, 1, 0xfc) && dm_command(frames, n, 2, 0xfd));
        feed_jog(arm, 12, 0, BigPitch);
        n = arm.step_jog(12, 0, true, frames);
        CHECK(!dm_command(frames, n, 1, 0xfc) && !dm_command(frames, n, 1, 0xfd) &&
              dm_command(frames, n, 2, 0xfd));
        // 200 DM counts out of 65535 represent 1.10 degrees at the configured position scale.
        feed_jog(arm, 16, 0, BigPitch, kDmEncoderMidpoint + 200);
        n = arm.step_jog(16, 0, true, frames);
        CHECK(jog_stopped(frames, n) && !arm.status().faults);
    }
    for (unsigned axis = BigPitch; axis <= SmallPitch; ++axis) {
        // Motion above 5 deg/s is allowed only on the selected pitch during its pulse.
        ArmControl arm(four_axis(false));
        const unsigned other = axis == BigPitch ? SmallPitch : BigPitch;
        constexpr uint16_t kFastDmVelocity = 0x808; // About 7.1 deg/s at the configured VMAX.
        constexpr uint16_t kSmallMovePosition = kDmEncoderMidpoint + 5;
        feed_jog(arm, 0);
        arm.step_jog(0, 5, false, frames);
        feed_jog_dm(arm, 4, axis, false, kDmEncoderMidpoint, kFastDmVelocity);
        feed_jog_dm(arm, 4, other, false);
        auto n = arm.step_jog(4, int32_t(axis), true, frames);
        CHECK(jog_stopped(frames, n) && arm.status().last_result == Result::Moving);

        feed_jog(arm, 8);
        n = arm.step_jog(8, int32_t(axis), true, frames);
        CHECK(dm_command(frames, n, axis, 0xfc) && dm_command(frames, n, other, 0xfd));
        feed_jog_dm(arm, 12, axis, true, kSmallMovePosition, kFastDmVelocity);
        feed_jog_dm(arm, 12, other, false);
        n = arm.step_jog(12, 0, true, frames);
        const auto &selected = arm.status().joint[axis];
        CHECK(selected.drive_velocity_deg_s > 5 && std::fabs(selected.drive_position_deg) < 1);
        CHECK(arm.status().enabled_mask == (1u << axis) && !arm.status().faults);
        CHECK(!dm_command(frames, n, axis, 0xfd) && dm_command(frames, n, other, 0xfd));

        feed_jog_dm(arm, 16, axis, true, kSmallMovePosition);
        feed_jog_dm(arm, 16, other, false, kDmEncoderMidpoint, kFastDmVelocity);
        n = arm.step_jog(16, 0, true, frames);
        CHECK(jog_stopped(frames, n) && (arm.status().joint[other].faults & PositionLimit));
    }
    for (int direction = 1; direction >= -1; direction -= 2) {
        // Both pitch encoders cross the raw position seam in opposite directions.
        ArmControl arm(four_axis(false));
        constexpr uint16_t kEncoderMax = 65535;
        const uint16_t start = direction > 0 ? kEncoderMax - 2 : 2;
        const uint16_t across = direction > 0 ? 4 : kEncoderMax - 4;
        const uint16_t farther = direction > 0 ? 200 : kEncoderMax - 200;
        feed_jog(arm, 0, 0, Yaw, start, kEncoderMax - start);
        const auto &big = arm.status().joint[BigPitch];
        const auto &small = arm.status().joint[SmallPitch];
        CHECK(!big.calibrated && !small.calibrated);
        CHECK(direction * big.drive_position_deg > 179.0f);
        CHECK(-direction * small.drive_position_deg > 179.0f);
        auto n = arm.step_jog(0, 5, false, frames);
        CHECK(jog_stopped(frames, n) && arm.status().last_result == Result::Ok);
        CHECK(big.drive_position_deg == 0 && small.drive_position_deg == 0);

        feed_jog(arm, 4, 0, Yaw, start, kEncoderMax - start);
        n = arm.step_jog(4, direction, true, frames);
        CHECK(dm_command(frames, n, 1, 0xfc) && dm_command(frames, n, 2, 0xfd));
        feed_jog(arm, 8, 0, BigPitch, across, kEncoderMax - across);
        n = arm.step_jog(8, 0, true, frames);
        const float travel = direction * (6.0f * 360.0f / kEncoderMax);
        CHECK(std::fabs(big.drive_position_deg - travel) < 0.002f);
        CHECK(std::fabs(small.drive_position_deg + travel) < 0.002f);
        CHECK(std::fabs(big.position_deg - travel) < 0.002f);
        CHECK(std::fabs(small.position_deg + 2 * travel) < 0.002f);
        CHECK(arm.status().enabled_mask == (1u << BigPitch) && !arm.status().faults);
        CHECK(!dm_command(frames, n, 1, 0xfd) && dm_command(frames, n, 2, 0xfd));

        // The selected drive still stops normally after about 1.1 degrees of actual travel.
        feed_jog(arm, 12, 0, BigPitch, farther, kEncoderMax - across);
        n = arm.step_jog(12, 0, true, frames);
        CHECK(jog_stopped(frames, n) && !arm.status().faults);
    }
    { // A wider DM protocol mapping keeps its original linear position interpretation.
        auto config = four_axis(false);
        config.joint[BigPitch].p_max = 12.5f;
        ArmControl arm(config);
        constexpr uint16_t kOffsetCounts = 5000;
        feed_jog(arm, 0, 0, Yaw, kDmEncoderMidpoint - kOffsetCounts);
        CHECK(arm.capture_zero(kPitchPair, 0) == Result::Ok);
        feed_jog(arm, 4, 0, Yaw, kDmEncoderMidpoint + kOffsetCounts);
        const auto &big = arm.status().joint[BigPitch];
        CHECK(std::fabs(big.drive_position_deg - 218.57f) < 0.02f);
    }
    { // Unexpected displacement of a pitch also stops a selected C610 pulse.
        ArmControl arm(four_axis(false));
        feed_jog(arm, 0, 1u << Gripper);
        arm.step_jog(0, 5, false, frames);
        feed_jog(arm, 4, 1u << Gripper);
        arm.step_jog(4, 4, true, frames);
        CHECK(arm.status().enabled_mask == (1u << Gripper));
        // 432 DM counts represent 2.37 degrees, beyond the joint movement guard.
        feed_jog(arm, 8, 1u << Gripper, Yaw, kDmEncoderMidpoint, kDmEncoderMidpoint + 432);
        const auto n = arm.step_jog(8, 0, true, frames);
        CHECK(jog_stopped(frames, n) && (arm.status().faults & PositionLimit));
    }
    { // The selected C610 must also stop if the unpowered pitch pair loses feedback.
        ArmControl arm(four_axis(false));
        feed_jog(arm, 0, 1u << Roll);
        arm.step_jog(0, 5, false, frames);
        feed_jog(arm, 4, 1u << Roll);
        arm.step_jog(4, 3, true, frames);
        const uint8_t dm[8] = {BigPitch, 0x80, 0, 0x80, 0x08, 0, 25, 25};
        const uint8_t dji[8] = {3, 232, 0, 0, 0, 0, 25, 0};
        for (uint32_t ms = 8; ms <= 16; ms += 4) {
            arm.receive(1, arm.feedback_id(BigPitch), dm, 8, ms);
            arm.receive(1, arm.feedback_id(Roll), dji, 8, ms);
            const auto n = arm.step_jog(ms, 0, true, frames);
            if (ms == 16)
                CHECK(jog_stopped(frames, n) && (arm.status().faults & FeedbackTimeout));
            else
                CHECK(arm.status().enabled_mask == (1u << Roll));
        }
    }
    return 0;
}
