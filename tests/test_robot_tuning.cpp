#include "Vehicle/robot_tuning.h"
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x)                                                                                    \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x))                                                                                  \
            return __LINE__;                                                                       \
    } while (0)

namespace {
TuneInput healthy(uint32_t mode, uint32_t now = 100) {
    TuneInput input{};
    input.mode = mode;
    input.now_ms = now;
    input.common_ready = input.selected_ready = input.stationary = true;
    input.yaw_min_deg = -10;
    input.yaw_max_deg = 10;
    input.enable = true;
    return input;
}
} // namespace

int test_robot_tuning() {
    { // Selection is a zero-output transition, never an old target replay.
        RobotTuning gate;
        auto in = healthy(TuneLeftCurrent);
        in.current_ma = 250;
        auto out = gate.step(in);
        CHECK(out.changed && !out.ready && !out.blocked && !out.current_active && out.value == 0);
        const uint32_t old_epoch = out.epoch;
        in.current_ma = 0; // Owner clears the public target on changed.
        out = gate.step(in);
        CHECK(!out.ready && !out.blocked && out.value == 0); // Enable edge prepares once.
        out = gate.step(in);
        CHECK(!out.changed && out.ready && out.value == 0 && !out.current_active);
        in.current_ma = 50;
        out = gate.step(in);
        CHECK(out.ready && out.current_active && out.value == 50);
        in.mode = TuneRightSpeed;
        in.speed_rpm = 300;
        out = gate.step(in);
        CHECK(out.changed && out.epoch != old_epoch && !out.ready && out.value == 0);
        in.speed_rpm = 0;
        out = gate.step(in);
        CHECK(!out.ready && out.value == 0);
        out = gate.step(in);
        CHECK(out.ready && !out.current_active && out.value == 0);
        in.speed_rpm = -100;
        in.stationary = false; // Motion after entry must not itself trip the gate.
        out = gate.step(in);
        CHECK(out.ready && !out.blocked && out.value == -100);
    }
    { // Current has a fixed deadline; target edits cannot prolong the pulse.
        RobotTuning gate;
        auto in = healthy(TuneRightCurrent, 1000);
        gate.step(in);
        gate.step(in);
        in.current_ma = 40;
        CHECK(gate.step(in).current_active);
        in.now_ms = 1499;
        in.current_ma = -60;
        CHECK(gate.step(in).ready && gate.status().value == -60);
        in.now_ms = 1500;
        auto out = gate.step(in);
        CHECK(out.blocked && !out.ready && !out.current_active && out.value == 0 &&
              out.reason == TuneCurrentTimeout);
        in.current_ma = 0;
        CHECK(!gate.step(in).blocked && gate.status().ready); // per-cycle failure; zero target is ready.
        in.mode = TuneLeftSpeed;
        CHECK(gate.step(in).changed && !gate.status().blocked);
        in.mode = TuneNormal;
        CHECK(!gate.step(in).blocked);
        in.mode = TuneLeftSpeed;
        CHECK(gate.step(in).changed && !gate.status().ready);
        CHECK(!gate.step(in).ready);
        CHECK(gate.step(in).ready);
    }
    { // Releasing current before its deadline permits a fresh nonzero edge.
        RobotTuning gate;
        auto in = healthy(TuneLeftCurrent, 100);
        gate.step(in);
        gate.step(in);
        in.current_ma = 50;
        gate.step(in);
        in.now_ms = 200;
        in.current_ma = 0;
        CHECK(!gate.step(in).current_active && !gate.status().blocked);
        in.now_ms = 1000;
        in.current_ma = -50;
        CHECK(gate.step(in).current_active);
        in.now_ms = 1499;
        CHECK(gate.step(in).ready);
        in.now_ms = 1500;
        CHECK(gate.step(in).reason == TuneCurrentTimeout);
    }
    { // Failing entry is per-cycle; a fresh healthy input can start.
        for (unsigned cause = 0; cause < 2; ++cause) {
            RobotTuning gate;
            auto in = healthy(TuneLeftSpeed);
            if (cause == 0) in.common_ready = false;
            if (cause == 1) in.selected_ready = false;
            CHECK(!gate.step(in).blocked); // Selection alone is not an enable request.
            auto out = gate.step(in);
            CHECK(out.blocked && !out.ready && out.value == 0);
            CHECK(out.reason == TuneNotReady);
            in.common_ready = in.selected_ready = in.stationary = true;
            CHECK(!gate.step(in).blocked && !gate.status().ready);
            CHECK(gate.step(in).ready);
            in.mode = TuneYawSpeed;
            CHECK(gate.step(in).changed && !gate.status().blocked);
            in.mode = TuneNormal;
            CHECK(!gate.step(in).blocked && gate.status().value == 0);
            in.mode = TuneYawSpeed;
            CHECK(!gate.step(in).ready);
            CHECK(!gate.step(in).ready);
            CHECK(gate.step(in).ready);
        }
    }
    { // STOP and health loss are per-cycle; a fresh healthy step can resume.
        for (unsigned cause = 0; cause < 3; ++cause) {
            RobotTuning gate;
            auto in = healthy(TuneYawSpeed);
            gate.step(in);
            gate.step(in);
            in.yaw_speed_deg_s = 1;
            CHECK(gate.step(in).ready && gate.status().value == 1);
            if (cause == 0) in.stop = true;
            if (cause == 1) in.common_ready = false;
            if (cause == 2) in.selected_ready = false;
            auto out = gate.step(in);
            CHECK(out.blocked && !out.ready && out.value == 0);
            CHECK(out.reason == (cause == 0 ? TuneStopped : TuneNotReady));
            in.stop = false;
            in.common_ready = in.selected_ready = true;
            CHECK(gate.step(in).ready);
            in.mode = TuneNormal;
            CHECK(!gate.step(in).blocked);
        }
    }
    { // Finite, hard numeric limits are checked before output in every mode.
        const float bad[] = {std::numeric_limits<float>::quiet_NaN(),
                             std::numeric_limits<float>::infinity(),
                             -std::numeric_limits<float>::infinity(), 10001};
        for (uint32_t mode = TuneLeftCurrent; mode <= TuneYawSpeed; ++mode) {
            for (float value : bad) {
                RobotTuning gate;
                auto in = healthy(mode);
                gate.step(in);
                gate.step(in);
                in.current_ma = in.speed_rpm = in.yaw_speed_deg_s = value;
                auto out = gate.step(in);
                CHECK(out.blocked && out.reason == TuneInvalid && out.value == 0);
            }
        }
        RobotTuning gate;
        auto in = healthy(7);
        CHECK(gate.step(in).blocked && gate.status().reason == TuneInvalid);
        in.mode = TuneYawPosition;
        CHECK(gate.step(in).changed && !gate.status().blocked);
        in.mode = TuneNormal;
        CHECK(!gate.step(in).blocked);
    }
    { // Position begins at measured angle, then checks the requested interval.
        RobotTuning gate;
        auto in = healthy(TuneYawPosition);
        in.yaw_position_deg = 3;
        in.yaw_target_deg = -8; // Old public target must not be used on transition.
        auto out = gate.step(in);
        CHECK(out.changed && !out.ready && out.value == 3);
        in.yaw_target_deg = 3;
        CHECK(!gate.step(in).ready && gate.status().value == 3);
        CHECK(gate.step(in).ready && gate.status().value == 3);
        in.yaw_target_deg = 10;
        CHECK(gate.step(in).ready && gate.status().value == 10);
        in.yaw_target_deg = 10.1f;
        CHECK(gate.step(in).blocked && gate.status().reason == TuneInvalid);
        in.yaw_target_deg = 0;
        CHECK(gate.step(in).ready);
        in.mode = TuneNormal;
        gate.step(in);
        in.mode = TuneYawPosition;
        gate.step(in);
        gate.step(in);
        in.yaw_max_deg = in.yaw_min_deg;
        CHECK(gate.step(in).blocked && gate.status().reason == TuneInvalid);
    }
    { // Timer subtraction remains correct across uint32 tick wrap.
        RobotTuning gate;
        auto in = healthy(TuneLeftCurrent, 0xffffff00u);
        gate.step(in);
        gate.step(in);
        in.current_ma = 20;
        CHECK(gate.step(in).current_active);
        in.now_ms = 0xf3u; // 499ms later.
        CHECK(gate.step(in).ready);
        in.now_ms = 0xf4u; // 500ms later.
        CHECK(gate.step(in).reason == TuneCurrentTimeout);
    }
    { // Output is opt-in; an unready selection can wait safely while disabled.
        RobotTuning gate;
        auto in = healthy(TuneLeftSpeed);
        in.enable = false;
        in.common_ready = in.selected_ready = in.stationary = false;
        in.speed_rpm = 50;
        CHECK(!gate.step(in).blocked && !gate.status().ready);
        CHECK(!gate.step(in).blocked && gate.status().reason == TuneOutputDisabled);
        in.common_ready = in.selected_ready = in.stationary = true;
        CHECK(!gate.step(in).ready && !gate.status().blocked);
        in.enable = true;
        CHECK(!gate.step(in).ready && gate.status().value == 0);
        in.speed_rpm = 0; // Owner clears the target on explicit enable edge.
        CHECK(gate.step(in).ready && gate.status().value == 0);
        in.speed_rpm = 80;
        in.stationary = false;
        CHECK(gate.step(in).ready && gate.status().value == 80);
        in.enable = false;
        CHECK(!gate.step(in).ready && !gate.status().blocked && gate.status().value == 0);
        in.common_ready = in.selected_ready = false;
        CHECK(!gate.step(in).blocked); // Disabled test does not wait for a delayed start.
        in.enable = true;
        CHECK(gate.step(in).blocked && gate.status().reason == TuneNotReady);
        in.enable = false;
        in.common_ready = in.selected_ready = in.stationary = true;
        CHECK(!gate.step(in).blocked); // enable=0 acknowledges the failed test.
        in.mode = TuneNormal;
        CHECK(!gate.step(in).blocked);
    }
    { // Re-enable no longer waits for stillness; STOP remains effective while disabled.
        RobotTuning gate;
        auto in = healthy(TuneYawSpeed);
        in.enable = false;
        gate.step(in);
        CHECK(!gate.step(in).ready);
        in.enable = true;
        CHECK(!gate.step(in).ready);
        CHECK(gate.step(in).ready);
        in.enable = false;
        CHECK(!gate.step(in).ready);
        in.stationary = false;
        in.enable = true;
        CHECK(!gate.step(in).blocked && !gate.status().ready);
        CHECK(gate.step(in).ready); // stationary is advisory, not a hard gate.
        in.mode = TuneNormal;
        gate.step(in);
        in.mode = TuneYawSpeed;
        in.enable = false;
        gate.step(in);
        in.stop = true;
        CHECK(gate.step(in).blocked && gate.status().reason == TuneStopped);
    }
    { // Disable cancels a short pulse; enable=0 acknowledges the timeout.
        RobotTuning gate;
        auto in = healthy(TuneLeftCurrent, 100);
        gate.step(in);
        gate.step(in);
        in.current_ma = 40;
        CHECK(gate.step(in).current_active);
        in.enable = false;
        in.now_ms = 200;
        CHECK(!gate.step(in).current_active && !gate.status().blocked);
        in.now_ms = 1000;
        in.enable = true;
        CHECK(!gate.step(in).ready && !gate.status().current_active);
        in.current_ma = 0;
        CHECK(gate.step(in).ready);
        in.current_ma = 40;
        CHECK(gate.step(in).current_active);
        in.now_ms = 1500;
        CHECK(gate.step(in).reason == TuneCurrentTimeout);
        in.enable = false;
        CHECK(!gate.step(in).blocked); // enable=0 acknowledges the timeout.
        in.enable = true;
        CHECK(!gate.step(in).blocked && !gate.status().ready);
        CHECK(gate.step(in).ready);
    }
    return 0;
}
