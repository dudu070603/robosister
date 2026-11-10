#include "task_protect.h"

extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if (!(x)) return __LINE__; } while (0)

namespace {
TaskProtectInput healthy() { return {1, 1, 1, 0, 1, 0}; }
}

int test_protect() {
    {
        Task_Protect_Init();
        auto s = Task_Protect_GetStatus();
        CHECK(s.disabled && s.current_reasons == PROTECT_BOOT);
        CHECK(s.latched_reasons == PROTECT_BOOT && s.last_trip_ms == 0);
        CHECK(!Task_Protect_TryArm());
        auto in = healthy();
        Task_Protect_Tick(&in, 100);
        s = Task_Protect_GetStatus();
        CHECK(s.disabled && s.current_reasons == 0);
        CHECK(s.latched_reasons == PROTECT_BOOT);
        for (unsigned t = 104; t < 1000; t += 4) Task_Protect_Tick(&in, t);
        CHECK(Task_Protect_GetStatus().disabled);
        CHECK(Task_Protect_TryArm());
        s = Task_Protect_GetStatus();
        CHECK(!s.disabled && s.current_reasons == 0 && s.latched_reasons == 0);
        Task_Protect_Tick(&in, 1000);
        CHECK(!Task_Protect_GetStatus().disabled);
    }
    {
        // Every individual input cause must disable and reject rearm.
        const unsigned reasons[] = {PROTECT_CONFIG, PROTECT_REMOTE, PROTECT_INTERLOCK,
            PROTECT_STOP_REQUEST, PROTECT_TIMING, PROTECT_ARM_FAULT};
        for (unsigned i = 0; i < 6; ++i) {
            Task_Protect_Init();
            auto in = healthy();
            Task_Protect_Tick(&in, 10);
            CHECK(Task_Protect_TryArm());
            switch (i) {
            case 0: in.configuration_confirmed = 0; break;
            case 1: in.remote_online = 0; break;
            case 2: in.interlock_online = 0; break;
            case 3: in.stop_requested = 1; break;
            case 4: in.timing_ok = 0; break;
            default: in.arm_fault = 1; break;
            }
            Task_Protect_Tick(&in, 20);
            auto s = Task_Protect_GetStatus();
            CHECK(s.disabled && s.current_reasons == reasons[i]);
            CHECK(s.latched_reasons == reasons[i] && s.last_trip_ms == 20);
            CHECK(!Task_Protect_TryArm());
            Task_Protect_Tick(&in, 100);
            CHECK(Task_Protect_GetStatus().last_trip_ms == 20);
            in = healthy();
            Task_Protect_Tick(&in, 104);
            s = Task_Protect_GetStatus();
            CHECK(s.disabled && s.current_reasons == 0 && s.latched_reasons == reasons[i]);
            CHECK(Task_Protect_TryArm());
            s = Task_Protect_GetStatus();
            CHECK(!s.disabled && s.latched_reasons == 0 && s.last_trip_ms == 20);
        }
    }
    {
        // Concurrent and successive causes remain visible until explicit rearm.
        Task_Protect_Init();
        auto in = healthy();
        Task_Protect_Tick(&in, 0); CHECK(Task_Protect_TryArm());
        in.remote_online = 0; in.interlock_online = 0; in.stop_requested = 1;
        Task_Protect_Tick(&in, 10);
        const unsigned first = PROTECT_REMOTE | PROTECT_INTERLOCK | PROTECT_STOP_REQUEST;
        auto s = Task_Protect_GetStatus();
        CHECK(s.current_reasons == first && s.latched_reasons == first);
        in.remote_online = 1; in.stop_requested = 0;
        Task_Protect_Tick(&in, 14);
        s = Task_Protect_GetStatus();
        CHECK(s.current_reasons == PROTECT_INTERLOCK && s.latched_reasons == first);
        CHECK(s.last_trip_ms == 10);
        in.arm_fault = 1;
        Task_Protect_Tick(&in, 18);
        CHECK(Task_Protect_GetStatus().last_trip_ms == 18);
        in = healthy();
        Task_Protect_Tick(&in, 22);
        CHECK(Task_Protect_GetStatus().disabled);
        in.remote_online = 0;
        Task_Protect_Tick(&in, 26);
        s = Task_Protect_GetStatus();
        CHECK(s.last_trip_ms == 26); // a cause that returned is a new arrival
        CHECK(s.latched_reasons == (first | PROTECT_ARM_FAULT));
        CHECK(!Task_Protect_TryArm());
    }
    {
        Task_Protect_Init();
        Task_Protect_Trip(PROTECT_ENABLE_REJECTED, 20);
        CHECK(!Task_Protect_TryArm()); // never ticked
        CHECK(Task_Protect_GetStatus().latched_reasons == (PROTECT_BOOT | PROTECT_ENABLE_REJECTED));
        auto in = healthy();
        Task_Protect_Tick(&in, 24);
        CHECK(Task_Protect_GetStatus().disabled);
        CHECK(Task_Protect_TryArm());
        Task_Protect_Trip(PROTECT_ENABLE_REJECTED, 28);
        auto s = Task_Protect_GetStatus();
        CHECK(s.disabled && s.current_reasons == PROTECT_ENABLE_REJECTED);
        CHECK(s.latched_reasons == PROTECT_ENABLE_REJECTED && s.last_trip_ms == 28);
        CHECK(!Task_Protect_TryArm());
        Task_Protect_Trip(PROTECT_ENABLE_REJECTED, 32);
        CHECK(Task_Protect_GetStatus().last_trip_ms == 28);
        Task_Protect_Trip(PROTECT_ARM_FAULT, 36);
        CHECK(Task_Protect_GetStatus().last_trip_ms == 36);
        CHECK(Task_Protect_GetStatus().current_reasons == (PROTECT_ENABLE_REJECTED | PROTECT_ARM_FAULT));
        Task_Protect_Tick(&in, 40);
        CHECK(Task_Protect_GetStatus().disabled && Task_Protect_GetStatus().current_reasons == 0);
        CHECK(Task_Protect_TryArm());
    }
    {
        // All six boolean fields, not just online flags, are validated.
        for (unsigned i = 0; i < 6; ++i) {
            Task_Protect_Init(); auto in = healthy();
            Task_Protect_Tick(&in, 0); CHECK(Task_Protect_TryArm());
            switch (i) {
            case 0: in.configuration_confirmed = 2; break;
            case 1: in.remote_online = 255; break;
            case 2: in.interlock_online = 2; break;
            case 3: in.stop_requested = 2; break;
            case 4: in.timing_ok = 2; break;
            default: in.arm_fault = 2; break;
            }
            Task_Protect_Tick(&in, 4);
            CHECK(Task_Protect_GetStatus().disabled);
            CHECK(Task_Protect_GetStatus().current_reasons == PROTECT_CONFIG);
            CHECK(!Task_Protect_TryArm());
        }
        Task_Protect_Init();
        Task_Protect_Tick(nullptr, 10);
        CHECK(Task_Protect_GetStatus().disabled);
        CHECK(Task_Protect_GetStatus().current_reasons == PROTECT_CONFIG);
        CHECK(!Task_Protect_TryArm());
        auto in = healthy(); Task_Protect_Tick(&in, 14); CHECK(Task_Protect_TryArm());
        Task_Protect_Trip(0, 18);
        CHECK(Task_Protect_GetStatus().current_reasons == PROTECT_CONFIG);
        CHECK(!Task_Protect_TryArm());
        Task_Protect_Tick(&in, 22); CHECK(Task_Protect_TryArm());
        Task_Protect_Trip(1u << 31, 26);
        CHECK(Task_Protect_GetStatus().current_reasons == PROTECT_CONFIG);
        CHECK(!Task_Protect_TryArm());
    }
    {
        // Timestamp wrap is retained verbatim; no elapsed-time arithmetic here.
        Task_Protect_Init(); auto in = healthy();
        Task_Protect_Tick(&in, 0xfffffff0u); CHECK(Task_Protect_TryArm());
        in.stop_requested = 1; Task_Protect_Tick(&in, 0xfffffffcu);
        CHECK(Task_Protect_GetStatus().last_trip_ms == 0xfffffffcu);
        Task_Protect_Tick(&in, 0);
        CHECK(Task_Protect_GetStatus().last_trip_ms == 0xfffffffcu);
        in.arm_fault = 1; Task_Protect_Tick(&in, 4);
        CHECK(Task_Protect_GetStatus().last_trip_ms == 4);
        // Init is also deterministic after a running session.
        Task_Protect_Init();
        CHECK(Task_Protect_GetStatus().disabled && Task_Protect_GetStatus().latched_reasons == PROTECT_BOOT);
        CHECK(!Task_Protect_TryArm());
    }
    return 0;
}
