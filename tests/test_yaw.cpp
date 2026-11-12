#include "Arm/arm_control.h"
#include "arm_task.h"
#include "Remote/operator_control.h"
#include "Arm/yaw_bench.h"
#include "yaw_service.h"
#include <cmath>
#include <initializer_list>
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
Config config() {
    Config c = kArmConfig;
    c.joint_mask = kAllJoints;
    c.hardware_confirmed = true;
    c.pitch.mapping_confirmed = true;
    return c;
}
bool feed_dji(ArmControl &arm, unsigned joint, uint32_t ms, unsigned encoder = 1000,
              int rpm = 0) {
    const uint8_t d[8] = {uint8_t(encoder >> 8),
                          uint8_t(encoder),
                          uint8_t(uint16_t(rpm) >> 8),
                          uint8_t(rpm),
                          0,
                          0,
                          25,
                          0};
    return arm.receive(1, arm.feedback_id(joint), d, 8, ms);
}
bool feed(ArmControl &arm, uint32_t ms, unsigned encoder = 1000, int rpm = 0) {
    return feed_dji(arm, Yaw, ms, encoder, rpm);
}
bool feed_all(ArmControl &arm, uint32_t ms, unsigned yaw_encoder = 1000, int yaw_rpm = 0) {
    bool ok = feed(arm, ms, yaw_encoder, yaw_rpm) && feed_dji(arm, Roll, ms) &&
              feed_dji(arm, Gripper, ms);
    for (unsigned joint : {unsigned(BigPitch), unsigned(SmallPitch)}) {
        const uint8_t d[8] = {uint8_t(0x10 | (joint == BigPitch ? 1 : 2)),
                              0x80, 0, 0x80, 0x08, 0, 25, 25};
        ok = arm.receive(1, arm.feedback_id(joint), d, 8, ms) && ok;
    }
    return ok;
}
bool same_hold(const JointStatus &a, const JointStatus &b) {
    return a.target_deg == b.target_deg && a.reference_deg == b.reference_deg &&
           a.drive_target_deg == b.drive_target_deg &&
           a.drive_reference_deg == b.drive_reference_deg &&
           a.drive_reference_velocity_deg_s == b.drive_reference_velocity_deg_s &&
           a.velocity_target_deg_s == b.velocity_target_deg_s && a.effort == b.effort &&
           a.enabled == b.enabled;
}
YawRuntimeLimits limits(uint32_t version = 1) {
    return {-10, 10, 10, 15, 2, 3, version};
}
bool yaw_zero(const Frame *f, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        if (f[i].id == 0x1fe || f[i].id == 0x2fe)
            for (uint8_t b : f[i].data)
                if (b)
                    return false;
    return true;
}
} // namespace
int test_yaw() {
    { // Task adaptation preserves the public debug ABI and one-shot requests.
        static_assert(sizeof(YawLimitRequest) == 8);
        static_assert(sizeof(YawLimitStatus) == 32);
        CHECK(!Yaw_GetLimitStatus(nullptr));
        YawLimitStatus published{};
        CHECK(!Yaw_GetLimitStatus(&published) && !published.ready);
        const auto &j = kArmConfig.joint[Yaw];
        CHECK(Yaw_EnvelopeAllowed(j.min_deg, j.max_deg, j.max_speed_deg_s));
        CHECK(!Yaw_EnvelopeAllowed(j.max_deg, j.min_deg, 1));
        CHECK(!Yaw_EnvelopeAllowed(j.min_deg - 1, j.max_deg, 1));
        CHECK(!Yaw_EnvelopeAllowed(j.min_deg, j.max_deg, 0));
        CHECK(!Yaw_EnvelopeAllowed(j.min_deg, j.max_deg, j.max_speed_deg_s + 1));
        CHECK(!Yaw_EnvelopeAllowed(std::numeric_limits<float>::quiet_NaN(), j.max_deg, 1));
        ArmControl arm(kArmConfig);
        ArmOperator controller(kArmConfig, kOperatorConfig);
        Interlocks locks;
        locks.valid = true;
        CHECK(!Yaw_ConfigurationReady(arm));
        g_yaw_limit_request.half_range_deg = 10;
        g_yaw_limit_request.request_seq = 17;
        Yaw_ApplyLimitRequest(arm, controller, locks, locks, 0);
        CHECK(Yaw_GetLimitStatus(&published) == 0 && published.consumed_seq == 17);
        CHECK(published.result == uint32_t(kArmConfig.joint_mask == kFourAxisJoints
                                               ? Result::InvalidArgument
                                               : Result::ConfigUnconfirmed));
        CHECK(!published.active_seq && !published.ready && locks.valid);
        const auto first_result = published.result;
        g_yaw_limit_request.half_range_deg = std::numeric_limits<float>::quiet_NaN();
        Yaw_ApplyLimitRequest(arm, controller, locks, locks, 4);
        CHECK(!Yaw_GetLimitStatus(&published) && published.result == first_result &&
              published.consumed_seq == 17);
        CHECK(arm.yaw_limits().version == 0 && !arm.status().enabled_mask);
        g_yaw_limit_request.half_range_deg = 0;
        g_yaw_limit_request.request_seq = 0;
        g_yaw_limit_status = {};
    }
    static_assert(sizeof(ArmStatus::updated_ms) == 4);
    Frame frames[kJointCount]{};
    { // No first frame, no calibration, stale/moving/enabled input: atomic refusal.
        ArmControl a(config());
        auto l = limits();
        CHECK(a.set_yaw_limits(l, 0) == Result::NotOnline);
        CHECK(a.yaw_limits().version == 0);
        CHECK(feed(a, 0));
        CHECK(a.set_yaw_limits(l, 0) == Result::NotCalibrated);
        CHECK(a.capture_zero(1, 0) == Result::Ok);
        CHECK(a.set_yaw_limits(l, 0) == Result::Ok);
        CHECK(a.yaw_limits().version == 1 && a.yaw_limits().max_deg == 10);
        l.max_deg = 100;
        CHECK(a.set_yaw_limits(l, 0) == Result::InvalidArgument);
        l = limits();
        l.braking_margin_deg = 10;
        CHECK(a.set_yaw_limits(l, 0) == Result::InvalidArgument);
        l = limits();
        l.max_speed_deg_s = std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_yaw_limits(l, 0) == Result::InvalidArgument);
        l = limits();
        l.braking_accel_deg_s2 = 0;
        CHECK(a.set_yaw_limits(l, 0) == Result::InvalidArgument);
        l = limits();
        l.version = 0;
        CHECK(a.set_yaw_limits(l, 0) == Result::InvalidArgument);
        CHECK(a.yaw_limits().version == 1);
        CHECK(a.enable(1, 0) == Result::Ok);
        CHECK(a.set_yaw_limits(limits(2), 0) == Result::Busy);
        a.disable();
        CHECK(feed(a, 4, 1000, 1));
        CHECK(a.set_yaw_limits(limits(2), 4) == Result::Moving);
        CHECK(feed(a, 8));
        CHECK(a.set_yaw_limits(limits(2), 8) == Result::Ok);
        CHECK(a.set_yaw_limits(limits(3), 100) == Result::NotOnline);
        CHECK(a.yaw_limits().version == 2);
    }
    { // Shrinking outside actual/target/reference is rejected, old boundary remains.
        ArmControl a(config());
        feed(a, 0);
        a.capture_zero(1, 0);
        CHECK(a.set_yaw_limits(limits(), 0) == Result::Ok);
        CHECK(feed(a, 4, 1137)); // About +6 degrees, zero-speed feedback.
        auto small = limits(2);
        small.min_deg = -4;
        small.max_deg = 4;
        small.braking_margin_deg = 1;
        CHECK(a.set_yaw_limits(small, 4) == Result::OutOfRange);
        CHECK(a.yaw_limits().max_deg == 10 && a.yaw_limits().version == 1);
        ArmControl b(config());
        feed(b, 0);
        b.capture_zero(1, 0);
        b.set_yaw_limits(limits(), 0);
        float q[kJointCount] = {3, 0, 0, 0, 0};
        CHECK(b.set_targets(1, q, 0) == Result::Ok);
        small.min_deg = -3;
        small.max_deg = 3;
        small.braking_margin_deg = 1;
        CHECK(b.set_yaw_limits(small, 0) ==
              Result::OutOfRange); // Target would be excluded by margin.
    }
    { // Lead bound is enforced in the kernel, including direct target API callers.
        ArmControl a(config());
        feed(a, 0);
        a.capture_zero(1, 0);
        a.set_yaw_limits(limits(), 0);
        CHECK(a.enable(1, 0) == Result::Ok);
        float q[kJointCount] = {80, 0, 0, 0, 0};
        CHECK(a.set_targets(1, q, 0) == Result::Ok);
        CHECK(std::fabs(a.status().joint[Yaw].target_deg - 3) < .001f);
        CHECK(a.set_yaw_envelope(-4, 4, 1) == Result::Ok);
        for (uint32_t t = 0; t < 120; t += 4) {
            CHECK(feed(a, t));
            CHECK(a.set_targets(1, q, t) == Result::Ok);
            a.step(t, frames);
            CHECK(std::fabs(a.status().joint[Yaw].velocity_target_deg_s) <= 1.0001f);
            CHECK(std::fabs(a.status().joint[Yaw].reference_deg) <= 3.0001f);
        }
        CHECK(a.set_yaw_envelope(-2, 2, 1) ==
              Result::OutOfRange); // Existing target outside new envelope.
        CHECK(a.set_yaw_envelope(-11, 11, 1) == Result::InvalidArgument);
    }
    { // Mode changes keep the driving envelope until wheel-stop confirmation.
        ArmControl a(config());
        feed(a, 0);
        a.capture_zero(1, 0);
        auto bound = limits();
        bound.max_target_lead_deg = 8;
        CHECK(a.set_yaw_limits(bound, 0) == Result::Ok);
        Interlocks locks;
        locks.valid = true;
        locks.drive_yaw_allowed = true;
        locks.drive_yaw_min_deg = -2;
        locks.drive_yaw_max_deg = 2;
        locks.drive_yaw_max_speed_deg_s = 1;
        OperatorOutput out;
        out.mode = RobotMode::Drive;
        out.active = true;
        Yaw_ApplyOperatingEnvelope(a, locks, out);
        float q[kJointCount] = {8, 0, 0, 0, 0};
        CHECK(a.set_targets(1, q, 0) == Result::Ok && a.status().joint[Yaw].target_deg == 2);
        out.mode = RobotMode::Grab;
        locks.chassis_stopped = false;
        Yaw_ApplyOperatingEnvelope(a, locks, out);
        CHECK(a.set_targets(1, q, 0) == Result::Ok && a.status().joint[Yaw].target_deg == 2);
        out.mode = RobotMode::Standby;
        Yaw_ApplyOperatingEnvelope(a, locks, out);
        CHECK(a.set_targets(1, q, 0) == Result::Ok && a.status().joint[Yaw].target_deg == 2);
        out.mode = RobotMode::Drive;
        locks.drive_yaw_allowed = false;
        locks.chassis_stopped = true;
        Yaw_ApplyOperatingEnvelope(a, locks, out);
        CHECK(a.set_targets(1, q, 0) == Result::Ok && a.status().joint[Yaw].target_deg == 2);
        out.mode = RobotMode::Grab;
        Yaw_ApplyOperatingEnvelope(a, locks, out);
        CHECK(a.set_targets(1, q, 0) == Result::Ok && a.status().joint[Yaw].target_deg == 8);
        out.mode = RobotMode::Drive;
        locks.drive_yaw_allowed = true;
        out.stop_chassis = out.longitudinal_brake = out.reset_chassis_motion = false;
        out.longitudinal_speed_request = .5f;
        out.turn_rate_request = .3f;
        Yaw_ApplyOperatingEnvelope(a, locks,
                                   out); // Current target is outside the proposed narrow range.
        CHECK(out.stop_chassis && out.longitudinal_brake && out.reset_chassis_motion);
        CHECK(out.longitudinal_speed_request == 0 && out.turn_rate_request == 0 &&
              out.drive_yaw_limited);
        CHECK(out.block == OperatorBlock::DriveYawDenied && out.target_deg[Yaw] == 8);
        CHECK(a.set_targets(1, q, 0) == Result::Ok && a.status().joint[Yaw].target_deg == 8);
    }
    { // Actual outward velocity near the wire boundary commands inward braking.
        ArmControl a(config());
        feed(a, 0);
        a.capture_zero(1, 0);
        a.set_yaw_limits(limits(), 0);
        a.enable(1, 0);
        CHECK(feed(a, 4, 1205,
                   1)); // +9deg and +6deg/s; within the old range, inside stopping margin.
        float q[kJointCount] = {10, 0, 0, 0, 0};
        a.set_targets(1, q, 4);
        a.step(4, frames);
        CHECK(a.status().joint[Yaw].velocity_target_deg_s <= 0.0001f);
        CHECK(a.status().joint[Yaw].effort < 0); // Brake is retained, not blanket-zeroed.
        CHECK(!yaw_zero(frames, 4));
        a.disable();
        ArmControl b(config());
        feed(b, 0);
        b.capture_zero(1, 0);
        b.set_yaw_limits(limits(), 0);
        b.enable(1, 0);
        CHECK(feed(b, 4, 795, -1));
        q[0] = -10;
        b.set_targets(1, q, 4);
        b.step(4, frames);
        CHECK(b.status().joint[Yaw].velocity_target_deg_s >= -.0001f);
        CHECK(b.status().joint[Yaw].effort > 0);
    }
    { // The owner-only tuning path still uses the same velocity PI and bounds.
        ArmControl a(config());
        feed(a, 0);
        a.capture_zero(1, 0);
        a.set_yaw_limits(limits(), 0);
        CHECK(a.set_yaw_test_speed(1, 0) == Result::InvalidArgument);
        CHECK(a.enable(1, 0) == Result::Ok);
        CHECK(a.set_effort_limit(Yaw, 150) == Result::Ok);
        CHECK(a.set_yaw_test_speed(6, 0) == Result::InvalidArgument);
        CHECK(a.set_yaw_test_speed(2, 0) == Result::Ok);
        a.step(0, frames);
        CHECK(a.status().joint[Yaw].velocity_target_deg_s == 2);
        CHECK(a.status().joint[Yaw].effort > 0 && a.status().joint[Yaw].effort <= 150);
        CHECK(feed(a, 4, 1205, 1));
        CHECK(a.set_yaw_test_speed(2, 4) == Result::Ok);
        a.step(4, frames);
        CHECK(a.status().joint[Yaw].velocity_target_deg_s <= 0);
        CHECK(a.status().joint[Yaw].effort < 0);
        a.disable();
        CHECK(a.set_yaw_test_speed(1, 4) == Result::InvalidArgument);
    }
    { // Runtime selection adds only the selected axis; existing arm holders keep their history.
        ArmControl a(config());
        CHECK(feed_all(a, 0));
        CHECK(a.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(a.set_yaw_limits(limits(), 0) == Result::Ok);
        float q[kJointCount] = {2, 3, -1, 4, 1};
        CHECK(a.set_targets(1u << Yaw, q, 0) == Result::Ok);
        CHECK(a.hold(1u << Yaw, 0) == Result::Ok);
        CHECK(!a.status().enabled_mask && a.status().joint[Yaw].target_deg == 2);
        CHECK(a.enable(kFourAxisJoints, 0) == Result::Ok);
        CHECK(a.set_targets(kFourAxisJoints, q, 0) == Result::Ok);
        a.step(0, frames);
        auto before = a.status();
        CHECK(a.enable(1u << Yaw, 0) == Result::Ok);
        CHECK(a.status().enabled_mask == kAllJoints);
        for (unsigned i = BigPitch; i < kJointCount; ++i)
            CHECK(same_hold(before.joint[i], a.status().joint[i]));
        CHECK(a.status().joint[Yaw].target_deg == a.status().joint[Yaw].position_deg);
        CHECK(a.enable(1u << Yaw, 0) == Result::Busy);
        CHECK(a.set_yaw_test_speed(2, 0) == Result::Ok);
        CHECK(feed_all(a, 4));
        a.step(4, frames);
        CHECK(a.status().enabled_mask == kAllJoints && a.status().joint[Yaw].effort > 0);
        CHECK(a.status().joint[Yaw].velocity_target_deg_s == 2);
        for (unsigned i = BigPitch; i < kJointCount; ++i)
            CHECK(a.status().joint[i].target_deg == q[i]);
        CHECK(feed_all(a, 8, 1023));
        before = a.status();
        a.clear_yaw_test();
        const auto &yaw = a.status().joint[Yaw];
        CHECK(yaw.position_deg > .9f && yaw.target_deg == yaw.position_deg &&
              yaw.reference_deg == yaw.position_deg && yaw.effort == 0 &&
              yaw.velocity_target_deg_s == 0);
        for (unsigned i = BigPitch; i < kJointCount; ++i)
            CHECK(same_hold(before.joint[i], a.status().joint[i]));
        a.step(8, frames);
        CHECK(a.status().joint[Yaw].effort == 0 && a.status().joint[Yaw].velocity_target_deg_s == 0);
        before = a.status();
        CHECK(a.disable_joints(1u << BigPitch) == Result::InvalidArgument);
        CHECK(a.hold(1u << SmallPitch, 8) == Result::InvalidArgument);
        CHECK(a.status().enabled_mask == kAllJoints);
        CHECK(a.disable_joints(1u << Yaw) == Result::Ok);
        CHECK(a.status().enabled_mask == kFourAxisJoints && !a.status().joint[Yaw].enabled);
        CHECK(a.hold(1u << Yaw, 8) == Result::Ok);
        CHECK(a.status().enabled_mask == kFourAxisJoints);
        for (unsigned i = BigPitch; i < kJointCount; ++i)
            CHECK(same_hold(before.joint[i], a.status().joint[i]));
        CHECK(a.enable(1u << Yaw, 8) == Result::Ok);
        CHECK(a.hold(kPitchPair, 8) == Result::Ok);
        CHECK(a.status().enabled_mask == kAllJoints);
        for (unsigned i : {unsigned(BigPitch), unsigned(SmallPitch)}) {
            const auto &s = a.status().joint[i];
            CHECK(s.target_deg == s.position_deg && s.reference_deg == s.position_deg &&
                  s.velocity_target_deg_s == 0 && s.drive_reference_velocity_deg_s == 0);
        }
        CHECK(same_hold(before.joint[Roll], a.status().joint[Roll]));
        CHECK(same_hold(before.joint[Gripper], a.status().joint[Gripper]));
        CHECK(a.disable_joints(kPitchPair) == Result::Ok);
        CHECK(a.status().enabled_mask == ((1u << Yaw) | (1u << Roll) | (1u << Gripper)));
        CHECK(a.hold(kAllJoints, 8) == Result::Ok);
        CHECK(a.status().enabled_mask == ((1u << Yaw) | (1u << Roll) | (1u << Gripper)));
    }
    { // Online tuning affects only the selected stationary drive and honors the immutable cap.
        ArmControl a(config());
        CHECK(feed_all(a, 0));
        CHECK(a.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(a.set_yaw_limits(limits(), 0) == Result::Ok);
        CHECK(a.enable(kAllJoints, 0) == Result::Ok);
        float q[kJointCount] = {0, 3, -1, 4, 1};
        CHECK(a.set_targets(kFourAxisJoints, q, 0) == Result::Ok);
        a.step(0, frames);
        JointTuning original{}, tuned{};
        CHECK(a.get_tuning(Yaw, original));
        tuned = original;
        tuned.velocity_kp += 1;
        tuned.effort_limit *= .5f;
        const auto before = a.status();
        CHECK(a.set_tuning(Yaw, tuned, 0) == Result::Busy);
        CHECK(a.set_tuning_runtime(Yaw, tuned, 0) == Result::Ok);
        CHECK(a.status().enabled_mask == kAllJoints);
        for (unsigned i = BigPitch; i < kJointCount; ++i)
            CHECK(same_hold(before.joint[i], a.status().joint[i]));
        JointTuning read{};
        CHECK(a.get_tuning(Yaw, read) && read.velocity_kp == tuned.velocity_kp &&
              read.effort_limit == tuned.effort_limit);
        CHECK(a.set_yaw_test_speed(2, 0) == Result::Ok);
        CHECK(a.set_tuning_runtime(Yaw, original, 0) == Result::Busy);
        CHECK(a.set_yaw_test_speed(0, 0) == Result::Ok);
        CHECK(a.set_tuning_runtime(Yaw, original, 0) == Result::Ok);
        CHECK(a.get_tuning(Yaw, read) && read.effort_limit == original.effort_limit);
        tuned.effort_limit = original.effort_limit + 1;
        CHECK(a.set_tuning_runtime(Yaw, tuned, 0) == Result::InvalidArgument);
        tuned = original;
        tuned.velocity_ki = std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_tuning_runtime(Yaw, tuned, 0) == Result::InvalidArgument);
        CHECK(a.get_tuning(Yaw, read) && read.velocity_ki == original.velocity_ki &&
              read.effort_limit == original.effort_limit);
        a.clear_yaw_test();
        q[Yaw] = 1;
        CHECK(a.set_targets(1u << Yaw, q, 0) == Result::Ok);
        CHECK(a.set_tuning_runtime(Yaw, original, 0) == Result::Busy);
        CHECK(a.hold(1u << Yaw, 0) == Result::Ok);
        CHECK(a.set_tuning_runtime(Yaw, original, 0) == Result::Ok);
        CHECK(feed_all(a, 4, 1000, 1));
        CHECK(a.set_tuning_runtime(Yaw, original, 4) == Result::Moving);
        CHECK(a.status().enabled_mask == kAllJoints);
        CHECK(feed_all(a, 8));
        JointTuning pitch{};
        CHECK(a.get_tuning(BigPitch, pitch));
        CHECK(a.set_tuning_runtime(BigPitch, pitch, 8) == Result::Busy);
        CHECK(a.hold(kPitchPair, 8) == Result::Ok);
        CHECK(a.set_tuning_runtime(BigPitch, pitch, 8) == Result::Ok);
        CHECK(a.status().enabled_mask == kAllJoints);
        CHECK(a.set_tuning_runtime(kJointCount, original, 8) == Result::InvalidArgument);
        CHECK(a.disable_joints(0) == Result::InvalidArgument);
        CHECK(a.hold(0, 8) == Result::InvalidArgument);
    }
    { // Four-axis bench never registers/enables yaw through the new core API.
        auto c = config();
        c.joint_mask = kFourAxisJoints;
        ArmControl a(c);
        CHECK(a.set_yaw_limits(limits(), 0) == Result::InvalidArgument);
        CHECK(a.set_yaw_envelope(-1, 1, 1) == Result::InvalidArgument);
        const unsigned n = a.step(0, frames);
        for (unsigned i = 0; i < n; ++i)
            CHECK(frames[i].id != 0x1fe && frames[i].id != 0x2fe);
    }
    { // Operator and inner loop consume the same effective limit version.
        auto c = config();
        OperatorConfig rc = kOperatorConfig;
        rc.mapping_confirmed = true;
        ArmOperator op(c, rc);
        op.set_yaw_limits(limits(7));
        Status s{};
        s.configuration_valid = true;
        for (auto &j : s.joint) {
            j.online = true;
            j.calibrated = true;
        }
        RemoteInput r{};
        r.valid = true;
        r.frame.switches[rc.left_switch_index] = DR16_SWITCH_MID;
        r.frame.switches[rc.right_switch_index] = DR16_SWITCH_MID;
        Interlocks in;
        in.valid = true;
        in.chassis_stopped = true;
        in.arm_allows_drive = true;
        OperatorOutput o;
        for (uint32_t t = 0; t <= 400; t += 4) {
            r.received_ms = t;
            in.received_ms = t;
            o = op.step(r, in, s, t, t == 400, false);
        }
        CHECK(o.active);
        s.enabled_mask = kAllJoints;
        r.frame.channel[rc.right_x_channel] = 660;
        for (uint32_t t = 404; t <= 2000; t += 4) {
            r.received_ms = t;
            in.received_ms = t;
            o = op.step(r, in, s, t, false, false);
            CHECK(o.target_deg[Yaw] <= 3.0001f && o.target_deg[Yaw] >= -3.0001f);
        }
    }
    { // Bounded pulse: no first-frame delayed start, refresh extension, or automatic recovery.
        YawBenchGate gate;
        YawBenchRequest r{1, 2, 500, 0x59415753, 1};
        YawBenchHealth h;
        h.stationary = true;
        CHECK(!gate.step(0, r, h).active);
        h.ready = true;
        CHECK(!gate.step(4, r, h).active); // Rejected sequence already consumed.
        r.request_seq = 2;
        CHECK(gate.step(8, r, h).active);
        h.enabled = true;
        h.stationary = false;
        CHECK(gate.step(12, r, h).active);
        r.request_seq = 3;
        CHECK(!gate.step(16, r, h).active); // New request cancels, cannot extend.
        h.enabled = false;
        h.stationary = true;
        r.request_seq = 4;
        CHECK(gate.step(20, r, h).active);
        h.enabled = true;
        h.stop_epoch = 1;
        CHECK(!gate.step(24, r, h).active);
        h.enabled = false;
        CHECK(!gate.step(28, r, h).active);
        r.request_seq = 5;
        h.stop = true;
        CHECK(!gate.step(32, r, h).active);
        h.stop = false;
        CHECK(!gate.step(36, r, h).active);
        r.request_seq = 6;
        r.speed_deg_s = std::numeric_limits<float>::quiet_NaN();
        CHECK(!gate.step(40, r, h).active);
        r.request_seq = 7;
        r.speed_deg_s = 2;
        r.duration_ms = 501;
        CHECK(!gate.step(44, r, h).active);
        r.request_seq = 8;
        r.duration_ms = 20;
        CHECK(gate.step(0xfffffff0u, r, h).active);
        h.enabled = true;
        CHECK(gate.step(0xfffffff4u, r, h).active);
        CHECK(!gate.step(4, r, h).active);
        h.enabled = false;
        CHECK(!gate.step(8, r, h).active);
    }
    return 0;
}
