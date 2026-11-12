#include "Arm/arm_control.h"
#include <cmath>
#include <cstring>
#include <limits>
using namespace engineer;

extern "C" {
volatile unsigned test_checks = 0, test_failed_line = 0, test_done = 0;
volatile unsigned test_can_failed_line = 0;
volatile unsigned test_operator_failed_line = 0;
volatile unsigned test_cad_failed_line = 0;
volatile unsigned test_protect_failed_line = 0;
volatile unsigned test_robot_failed_line = 0;
volatile unsigned test_four_axis_failed_line = 0;
volatile unsigned test_dm_config_failed_line = 0;
volatile unsigned test_chassis_failed_line = 0;
volatile unsigned test_yaw_failed_line = 0;
volatile unsigned test_vehicle_failed_line = 0;
volatile unsigned test_robot_tuning_failed_line = 0;
}
int test_can_tx();
int test_operator();
int test_cad_mapping();
int test_protect();
int test_robot_control();
int test_four_axis();
int test_dm_id_config();
int test_chassis();
int test_yaw();
int test_vehicle();
int test_robot_tuning();
#define CHECK(x)                                                                                   \
    do {                                                                                           \
        ++test_checks;                                                                             \
        if (!(x)) {                                                                                \
            test_failed_line = __LINE__;                                                           \
            return;                                                                                \
        }                                                                                          \
    } while (0)
namespace {
Config bench() {
    Config c = kArmConfig;
    c.joint_mask = kAllJoints;
    c.hardware_confirmed = true;
    c.pitch.mapping_confirmed = true;
    return c;
}
void dji(ArmControl &a, unsigned joint, unsigned e, uint32_t ms, int rpm = 0, unsigned temp = 25) {
    uint8_t d[8] = {uint8_t(e >> 8), uint8_t(e), uint8_t(uint16_t(rpm) >> 8), uint8_t(rpm), 0, 0,
                    uint8_t(temp),   0};
    a.receive(1, a.feedback_id(joint), d, 8, ms);
}
void dm(ArmControl &a, unsigned joint, uint32_t ms, unsigned state = 0, unsigned temp = 25) {
    const unsigned id = joint == BigPitch ? 1 : 2;
    // 位置/速度/力矩都在中点附近。12位速度有量化误差，不等于精确0。
    uint8_t d[8] = {
        uint8_t((state << 4) | id), 0x80, 0x00, 0x80, 0x08, 0x00, uint8_t(temp), uint8_t(temp)};
    a.receive(1, a.feedback_id(joint), d, 8, ms);
}
void all(ArmControl &a, uint32_t ms, unsigned state = 0) {
    dji(a, Yaw, 1000, ms);
    dji(a, Roll, 1000, ms);
    dji(a, Gripper, 1000, ms);
    dm(a, BigPitch, ms, state);
    dm(a, SmallPitch, ms, state);
}
const Frame *find(const Frame *frames, unsigned n, unsigned bus, unsigned id) {
    for (unsigned i = 0; i < n; ++i)
        if (frames[i].bus == bus && frames[i].id == id)
            return frames + i;
    return nullptr;
}
bool dji_zero(const Frame *f, unsigned n) {
    for (unsigned i = 0; i < n; ++i)
        if (f[i].id == 0x200 || f[i].id == 0x2ff || f[i].id == 0x1ff || f[i].id == 0x1fe ||
            f[i].id == 0x2fe)
            for (auto b : f[i].data)
                if (b)
                    return false;
    return true;
}
void run() {
    test_robot_tuning_failed_line = test_robot_tuning();
    CHECK(test_robot_tuning_failed_line == 0);
    test_chassis_failed_line = test_chassis();
    CHECK(test_chassis_failed_line == 0);
    test_yaw_failed_line = test_yaw();
    CHECK(test_yaw_failed_line == 0);
    test_vehicle_failed_line = test_vehicle();
    CHECK(test_vehicle_failed_line == 0);
    test_dm_config_failed_line = test_dm_id_config();
    CHECK(test_dm_config_failed_line == 0);
    test_protect_failed_line = test_protect();
    CHECK(test_protect_failed_line == 0);
    test_can_failed_line = test_can_tx();
    CHECK(test_can_failed_line == 0);
    test_robot_failed_line = test_robot_control();
    CHECK(test_robot_failed_line == 0);
    test_four_axis_failed_line = test_four_axis();
    CHECK(test_four_axis_failed_line == 0);
    test_operator_failed_line = test_operator();
    CHECK(test_operator_failed_line == 0);
    test_cad_failed_line = test_cad_mapping();
    CHECK(test_cad_failed_line == 0);
    Frame f[kJointCount]{};
    { // 缺配置/反馈/标定都不得输出运动指令；无电机不阻塞。
        auto defaults = kArmConfig;
        defaults.joint_mask = kAllJoints;
        ArmControl a(defaults);
        CHECK(a.enable(kAllJoints, 0) == Result::ConfigUnconfirmed);
        auto n = a.step(0, f);
        CHECK(n == 4);
        CHECK(dji_zero(f, n));
        for (unsigned i = 0; i < n; ++i)
            CHECK(f[i].bus == 1);
        CHECK(find(f, n, 1, 1)->data[7] == 0xfd);
        CHECK(a.feedback_id(Yaw) == 0x205);
        CHECK(find(f, n, 1, 0x1fe) != nullptr); // 电流模式yaw ID1占0x1fe第一槽
        CHECK(find(f, n, 1, 0x2fe) == nullptr);
        CHECK(find(f, n, 1, 0x2ff) == nullptr); // 当前配置不再发送6020电压帧
        CHECK(kArmConfig.joint[Yaw].motor == Motor::Gm6020Current);
        ArmControl b(bench());
        CHECK(b.enable(1, 0) == Result::NotOnline);
        all(b, 0);
        CHECK(b.enable(1, 0) == Result::NotCalibrated);
        CHECK(b.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(b.enable(kAllJoints, 0) == Result::Ok);
        CHECK(b.capture_zero(1, 0) == Result::Busy);
        n = b.step(0, f);
        CHECK(find(f, n, 1, 1)->data[7] == 0xfc); // 等DM使能确认
        all(b, 1, 1);
        n = b.step(1, f);
        CHECK((find(f, n, 1, 1)->data[3] & 0xf) == 0);                           // MIT kp=0
        CHECK(find(f, n, 1, 1)->data[4] == 0 && find(f, n, 1, 1)->data[5] == 0); // kp/kd=0
    }
    { // DJI过零解包和36:1换算；首次反馈不能凭空多一圈。
        ArmControl a(bench());
        dji(a, Roll, 8190, 0);
        CHECK(a.capture_zero(1u << Roll, 0) == Result::Ok);
        dji(a, Roll, 2, 1);
        CHECK(std::fabs(a.status().joint[Roll].position_deg - 4 * 360.0f / 8192 / 36) < 0.0001f);
        dji(a, Roll, 8190, 2);
        CHECK(std::fabs(a.status().joint[Roll].position_deg) < 0.0001f);
        const auto feedback = a.feedback_id(Roll);
        uint8_t bad[8]{};
        CHECK(!a.receive(1, feedback, bad, 7, 3));
        CHECK(!a.receive(2, feedback, bad, 8, 3));
        bad[0] = 0xff;
        CHECK(!a.receive(1, feedback, bad, 8, 3));
        dji(a, Roll, 8190, 10);
        CHECK(!a.status().joint[Roll].calibrated);
    }
    { // 非法目标整包拒绝，不能部分更新；目标超时锁存停机。
        ArmControl a(bench());
        all(a, 0);
        CHECK(a.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(a.enable(1, 0) == Result::Ok);
        float q[5] = {10, 0, 0, 0, 0};
        CHECK(a.set_targets(1, q, 0) == Result::Ok);
        q[0] = std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_targets(1, q, 0) == Result::InvalidArgument);
        CHECK(a.status().joint[Yaw].target_deg == 10);
        q[0] = 91;
        CHECK(a.set_targets(1, q, 0) == Result::OutOfRange);
        for (unsigned t = 0; t <= 251; ++t) {
            all(a, t);
            a.step(t, f);
        }
        CHECK(a.status().faults & CommandTimeout);
        CHECK(a.status().enabled_mask == 0);
        CHECK(dji_zero(f, 4));
        CHECK(a.enable(1, 251) == Result::FaultLatched);
        CHECK(a.clear_faults() == Result::Ok);
        CHECK(a.enable(1, 251) == Result::Ok);
    }
    { // 失联/多圈失准需要重新校零；通讯恢复不自动重启。
        ArmControl a(bench());
        all(a, 0);
        a.capture_zero(kAllJoints, 0);
        a.enable(1, 0);
        a.step(0, f);
        a.step(1, f);
        a.step(2, f);
        a.step(3, f);
        a.step(4, f);
        a.step(5, f);
        a.step(6, f);
        CHECK(a.status().faults & PositionLost);
        CHECK(!a.status().joint[Yaw].calibrated);
        all(a, 7);
        a.step(7, f);
        CHECK(a.status().enabled_mask == 0);
        a.clear_faults();
        CHECK(a.enable(1, 7) == Result::NotCalibrated);
    }
    { // DM温度/驱动器错误、状态回退、使能确认超时都使整臂失能。
        ArmControl a(bench());
        all(a, 0);
        a.capture_zero(kAllJoints, 0);
        a.enable(kAllJoints, 0);
        a.step(0, f);
        all(a, 1, 1);
        dm(a, BigPitch, 1, 1, 80);
        auto n = a.step(1, f);
        CHECK(a.status().faults & OverTemperature);
        CHECK(dji_zero(f, n));
        CHECK(find(f, n, 1, 1)->data[7] == 0xfd);
        ArmControl b(bench());
        all(b, 0);
        b.capture_zero(kAllJoints, 0);
        b.enable(6, 0);
        b.step(0, f);
        dm(b, BigPitch, 1, 1);
        b.step(1, f);
        dm(b, BigPitch, 2, 8);
        b.step(2, f);
        CHECK(b.status().faults & MotorError);
        ArmControl c(bench());
        all(c, 0);
        c.capture_zero(kAllJoints, 0);
        c.enable(6, 0);
        for (unsigned t = 0; t <= 201; ++t) {
            all(c, t);
            c.step(t, f);
        }
        CHECK(c.status().faults & EnableTimeout);
        ArmControl e(bench());
        all(e, 0);
        e.capture_zero(kAllJoints, 0);
        e.enable(6, 0);
        for (unsigned t = 0; t <= 51; ++t)
            e.step(t, f);
        CHECK(e.status().faults & FeedbackTimeout);
    }
    { // 真实控制输出：有符号打包、通道归属、斜坡、输出限幅和正反方向。
        Config cfg = bench();
        cfg.joint[Roll].direction = -1;
        ArmControl a(cfg);
        all(a, 0);
        a.capture_zero(kAllJoints, 0);
        a.enable(1u << Roll, 0);
        float q[5] = {0, 0, 0, 90, 0};
        a.set_targets(1u << Roll, q, 0);
        float previous_v = 0, previous_q = 0;
        for (unsigned t = 0; t < 1000; ++t) {
            all(a, t * 4);
            a.set_targets(1u << Roll, q, t * 4);
            const unsigned n = a.step(t * 4, f);
            auto &s = a.status().joint[Roll];
            CHECK(a.status().faults == 0);
            CHECK(std::fabs(s.effort) <= 800);
            const float v = (s.reference_deg - previous_q) / 0.004f;
            CHECK(v >= -0.0001f && v <= 10.001f);
            CHECK(std::fabs(v - previous_v) <= 0.13f);
            previous_q = s.reference_deg;
            previous_v = v;
            const unsigned id = cfg.joint[Roll].motor_id, byte = ((id - 1) % 4) * 2;
            const auto *current = find(f, n, 1, id <= 4 ? 0x200 : 0x1ff);
            CHECK(current != nullptr);
            const int raw = int16_t(uint16_t(current->data[byte]) << 8 | current->data[byte + 1]);
            CHECK(raw == int16_t(-s.effort) && raw <= 0);
            for (unsigned j = 0; j < 8; ++j)
                if (j != byte && j != byte + 1) CHECK(current->data[j] == 0);
        }
        a.transport_fault();
        auto n = a.step(4000, f);
        CHECK(dji_zero(f, n));
        CHECK(a.status().faults & TransportError);
    }
    { // GM6020电流模式：真实闭环输出的mA->原码、ID1~7分组、槽位、符号与零输出。
        for (unsigned id = 1; id <= 7; ++id)
            for (int sign = -1; sign <= 1; sign += 2) {
                Config c = bench();
                c.joint[Yaw].motor_id = id;
                c.joint[Yaw].max_effort = 3000;
                c.joint[Yaw].velocity_kp = 1000000;
                ArmControl a(c);
                CHECK(a.status().configuration_valid);
                CHECK(a.feedback_id(Yaw) == 0x204 + id);
                all(a, 0);
                CHECK(a.capture_zero(1, 0) == Result::Ok);
                CHECK(a.enable(1, 0) == Result::Ok);
                float q[5] = {float(sign) * 20, 0, 0, 0, 0};
                CHECK(a.set_targets(1, q, 0) == Result::Ok);
                auto n = a.step(0, f);
                const auto *current = find(f, n, 1, id <= 4 ? 0x1fe : 0x2fe);
                CHECK(current != nullptr);
                CHECK(find(f, n, 1, 0x1ff) == nullptr && find(f, n, 1, 0x2ff) == nullptr);
                const unsigned byte = ((id - 1) % 4) * 2;
                const int raw =
                    int16_t(uint16_t(current->data[byte]) << 8 | current->data[byte + 1]);
                CHECK(raw == sign * 16384);
                CHECK(a.status().joint[Yaw].effort == float(sign) * 3000);
                for (unsigned j = 0; j < 8; ++j)
                    if (j != byte && j != byte + 1)
                        CHECK(current->data[j] == 0);
                a.disable();
                all(a, 4);
                n = a.step(4, f);
                CHECK(dji_zero(f, n));
            }
        Config c = bench();
        c.joint[Yaw].velocity_kp = 1000000;
        c.joint[Yaw].direction = -1;
        ArmControl a(c);
        all(a, 0);
        a.capture_zero(1, 0);
        a.enable(1, 0);
        float q[5] = {20, 0, 0, 0, 0};
        a.set_targets(1, q, 0);
        auto n = a.step(0, f);
        const auto *current = find(f, n, 1, 0x1fe);
        CHECK(current != nullptr);
        // 默认上限300mA约对应1638原码；反向只影响报文符号，状态effort仍为关节正向mA。
        CHECK(a.status().joint[Yaw].effort == 300);
        CHECK(int16_t(uint16_t(current->data[0]) << 8 | current->data[1]) == -1638);
        dji(a, Yaw, 1000, 4, 0, 80);
        n = a.step(4, f);
        CHECK(a.status().faults & OverTemperature);
        CHECK(dji_zero(f, n));
        c = bench();
        c.joint[Yaw].max_effort = 3001;
        ArmControl bad_limit(c);
        CHECK(!bad_limit.status().configuration_valid);
        c = bench();
        c.joint[Yaw].motor_id = 8;
        ArmControl bad_id(c);
        CHECK(!bad_id.status().configuration_valid);
        // 显式旧电压类型仍单独走旧帧和原码，不能把3000mA转换误用于电压模式。
        c = bench();
        c.joint[Yaw].motor = Motor::Gm6020Voltage;
        c.joint[Yaw].max_effort = 2000;
        c.joint[Yaw].velocity_kp = 1000000;
        ArmControl voltage(c);
        all(voltage, 0);
        voltage.capture_zero(1, 0);
        voltage.enable(1, 0);
        voltage.set_targets(1, q, 0);
        n = voltage.step(0, f);
        const auto *v = find(f, n, 1, 0x1ff);
        CHECK(v != nullptr && find(f, n, 1, 0x1fe) == nullptr);
        CHECK(int16_t(uint16_t(v->data[0]) << 8 | v->data[1]) == 2000);
    }
    { // 协议边界和配置冲突检查。
        uint8_t d[8];
        pack_dm_torque(99, 12.5f, 30, 10, d);
        CHECK(d[6] == 0x0f && d[7] == 0xff);
        pack_dm_torque(-99, 12.5f, 30, 10, d);
        CHECK(d[6] == 0 && d[7] == 0);
        Config c = bench();
        c.joint[Gripper].motor_id = c.joint[Roll].motor_id;
        ArmControl a(c);
        CHECK(!a.status().configuration_valid);
        CHECK(a.step(0, f) == 0);
        c = bench();
        c.joint[BigPitch].feedback_id = 1;
        ArmControl b(c);
        CHECK(!b.status().configuration_valid);
        c = bench();
        c.joint[Yaw].feedback_per_joint = 0;
        ArmControl e(c);
        CHECK(!e.status().configuration_valid);
        ArmControl g(bench());
        all(g, 0);
        g.capture_zero(6, 0);
        g.enable(6, 0);
        g.step(0, f);
        all(g, 8);
        g.step(8, f);
        CHECK(g.status().faults & TimingError);
    }
    { // 动态夹爪电流上限立即生效，且不能提高到超过配置值。
        Config c = bench();
        c.joint[Gripper].velocity_kp = 100000;
        ArmControl a(c);
        all(a, 0);
        a.capture_zero(1u << Gripper, 0);
        a.enable(1u << Gripper, 0);
        float q[5] = {0, 0, 0, 0, 45};
        a.set_targets(1u << Gripper, q, 0);
        CHECK(a.set_effort_limit(Gripper, 150) == Result::Ok);
        auto n = a.step(0, f);
        CHECK(a.status().joint[Gripper].effort == 150);
        const unsigned id = c.joint[Gripper].motor_id, byte = ((id - 1) % 4) * 2;
        const auto *current = find(f, n, 1, id <= 4 ? 0x200 : 0x1ff);
        CHECK(current != nullptr);
        CHECK(int16_t(uint16_t(current->data[byte]) << 8 | current->data[byte + 1]) == 150);
        for (unsigned j = 0; j < 8; ++j)
            if (j != byte && j != byte + 1) CHECK(current->data[j] == 0);
        CHECK(a.set_effort_limit(Gripper, 0) == Result::Ok);
        all(a, 4);
        a.step(4, f);
        CHECK(a.status().joint[Gripper].effort == 0);
        CHECK(a.set_effort_limit(Gripper, 501) == Result::InvalidArgument);
        CHECK(a.set_effort_limit(Gripper, -1) == Result::InvalidArgument);
        CHECK(a.set_effort_limit(5, 100) == Result::InvalidArgument);
    }
    { // 在线PID只在失能/反馈新鲜/已校零/静止时原子生效，不改变目标和零位。
        ArmControl a(bench());
        const JointTuning tuning{3, 40, 2, 500};
        JointTuning current{};
        CHECK(a.get_tuning(Roll, current));
        CHECK(!a.get_tuning(kJointCount, current));
        CHECK(a.set_tuning(Roll, tuning, 0) == Result::NotOnline);
        all(a, 0);
        CHECK(a.set_tuning(Roll, tuning, 0) == Result::NotCalibrated);
        CHECK(a.capture_zero(kAllJoints, 0) == Result::Ok);
        float q[5] = {0, 1, 2, 3, 4};
        CHECK(a.set_targets(kAllJoints, q, 0) == Result::Ok);
        dji(a, Roll, 1004, 1);
        const auto before = a.status();
        const auto limits = a.yaw_limits();
        CHECK(a.set_tuning(Roll, tuning, 1) == Result::Ok);
        CHECK(a.get_tuning(Roll, current));
        CHECK(current.position_kp == 3 && current.velocity_kp == 40 &&
              current.velocity_ki == 2 && current.effort_limit == 500);
        CHECK(a.status().enabled_mask == 0 && a.status().joint[Roll].calibrated);
        CHECK(a.status().joint[Roll].position_deg == before.joint[Roll].position_deg &&
              a.status().joint[Roll].target_deg == before.joint[Roll].target_deg &&
              a.status().joint[Roll].reference_deg == before.joint[Roll].reference_deg);
        CHECK(a.yaw_limits().min_deg == limits.min_deg &&
              a.yaw_limits().max_deg == limits.max_deg &&
              a.yaw_limits().version == limits.version);
        const float nan = std::numeric_limits<float>::quiet_NaN();
        const JointTuning invalid[] = {
            {nan, 40, 2, 500}, {3, nan, 2, 500}, {3, 40, nan, 500}, {3, 40, 2, nan},
            {0, 40, 2, 500}, {1001, 40, 2, 500}, {3, -1, 2, 500}, {3, 40, -1, 500},
            {3, 1000001, 2, 500}, {3, 40, 1000001, 500}, {3, 40, 2, -1}, {3, 40, 2, 801}};
        for (const auto &bad : invalid) {
            CHECK(a.set_tuning(Roll, bad, 1) == Result::InvalidArgument);
            CHECK(a.get_tuning(Roll, current));
            CHECK(current.position_kp == 3 && current.velocity_kp == 40 &&
                  current.velocity_ki == 2 && current.effort_limit == 500);
        }
        CHECK(a.set_tuning(kJointCount, tuning, 1) == Result::InvalidArgument);
        dji(a, Roll, 1004, 2, 100);
        CHECK(a.set_tuning(Roll, tuning, 2) == Result::Moving);
        dji(a, Roll, 1004, 3);
        CHECK(a.set_tuning(Roll, tuning, 60) == Result::NotOnline);

        ArmControl busy(bench());
        all(busy, 0);
        CHECK(busy.capture_zero(kAllJoints, 0) == Result::Ok);
        CHECK(busy.enable(1u << Yaw, 0) == Result::Ok);
        CHECK(busy.set_tuning(Roll, tuning, 0) == Result::Busy); // 任一轴使能都拒绝。
        busy.disable();
        busy.transport_fault();
        CHECK(busy.set_tuning(Roll, tuning, 0) == Result::FaultLatched);
        auto four = bench();
        four.joint_mask = kFourAxisJoints;
        ArmControl no_yaw(four);
        CHECK(!no_yaw.get_tuning(Yaw, current));
        CHECK(no_yaw.set_tuning(Yaw, {2, 20, 5, 100}, 0) == Result::InvalidArgument);
    }
    { // 调一台pitch也必须同时核对另一台驱动的反馈和静止条件。
        ArmControl a(bench());
        JointTuning small_before{};
        CHECK(a.get_tuning(SmallPitch, small_before));
        const JointTuning tuning{3, .04f, .002f, .2f};
        dm(a, BigPitch, 0);
        CHECK(a.set_tuning(BigPitch, tuning, 0) == Result::NotOnline);
        dm(a, SmallPitch, 0);
        CHECK(a.set_tuning(BigPitch, tuning, 0) == Result::NotCalibrated);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        uint8_t moving[8] = {2, 0x80, 0, 0xff, 0xf8, 0, 25, 25};
        CHECK(a.receive(1, a.feedback_id(SmallPitch), moving, 8, 0));
        CHECK(a.set_tuning(BigPitch, tuning, 0) == Result::Moving);
        dm(a, SmallPitch, 0);
        CHECK(a.set_tuning(BigPitch, tuning, 0) == Result::Ok);
        JointTuning small{};
        CHECK(a.get_tuning(SmallPitch, small));
        CHECK(small.position_kp == small_before.position_kp &&
              small.velocity_kp == small_before.velocity_kp &&
              small.velocity_ki == small_before.velocity_ki &&
              small.effort_limit == small_before.effort_limit);
        CHECK(a.status().enabled_mask == 0 && a.status().joint[BigPitch].calibrated &&
              a.status().joint[SmallPitch].calibrated);
        dm(a, BigPitch, 10); // 双反馈时差超限，不能用单轴新反馈掩盖。
        CHECK(a.set_tuning(BigPitch, tuning, 10) == Result::NotOnline);
    }
    { // A lower startup limit is distinct from the permitted BigPitch tuning limit.
        auto config = bench();
        config.joint[BigPitch].max_effort = 3.0f;
        config.joint[BigPitch].initial_effort_limit = .3f;
        config.joint[BigPitch].velocity_kp = .03f;
        config.joint[SmallPitch].max_effort = .3f;
        config.joint[SmallPitch].initial_effort_limit = .3f;
        ArmControl a(config);
        JointTuning big{}, small{};
        CHECK(a.get_tuning(BigPitch, big) && big.effort_limit == .3f);
        CHECK(a.get_tuning(SmallPitch, small) && small.effort_limit == .3f);
        dm(a, BigPitch, 0, 1);
        dm(a, SmallPitch, 0, 1);
        CHECK(a.capture_zero(kPitchPair, 0) == Result::Ok);
        const float q[kJointCount] = {0, 10, 0, 0, 0};
        auto step_with_velocity_error = [&](uint32_t now) {
            // MIT velocity raw 0x7d0 reports about -40 deg/s at VMAX=30 rad/s.
            const uint8_t dm_fast[8] = {0x11, 0x80, 0, 0x7d, 0x08, 0, 25, 25};
            a.receive(1, a.feedback_id(BigPitch), dm_fast, 8, now);
            dm(a, SmallPitch, now, 1);
            return a.step(now, f);
        };
        CHECK(a.enable(kPitchPair, 0) == Result::Ok);
        CHECK(a.set_targets(kPitchPair, q, 0) == Result::Ok);
        step_with_velocity_error(0);
        CHECK(a.status().joint[BigPitch].effort == .3f);

        a.disable();
        dm(a, BigPitch, 4, 1);
        dm(a, SmallPitch, 4, 1);
        big.effort_limit = 3;
        CHECK(a.set_tuning(BigPitch, big, 4) == Result::Ok);
        big.effort_limit = 3.01f;
        CHECK(a.set_tuning(BigPitch, big, 4) == Result::InvalidArgument);
        small.effort_limit = 3;
        CHECK(a.set_tuning(SmallPitch, small, 4) == Result::InvalidArgument);
        CHECK(a.get_tuning(BigPitch, big) && big.effort_limit == 3);
        CHECK(a.get_tuning(SmallPitch, small) && small.effort_limit == .3f);
        CHECK(!a.status().enabled_mask);
        CHECK(a.enable(kPitchPair, 4) == Result::Ok);
        CHECK(a.set_targets(kPitchPair, q, 4) == Result::Ok);
        const auto n = step_with_velocity_error(4);
        const float effort = a.status().joint[BigPitch].effort;
        CHECK(!a.status().faults && effort > .3f && effort <= 3);
        const auto *drive = find(f, n, 1, 1);
        CHECK(drive != nullptr);
        const unsigned wire_torque = (unsigned(drive->data[6] & 0xf) << 8) | drive->data[7];
        CHECK(std::fabs((wire_torque * 20.0f / 4095.0f - 10.0f) - effort) < .005f);
    }
    { // 在线增益真正进入原闭环；运行限流不会被夹爪owner的保持请求覆盖。
        ArmControl baseline(bench()), tuned(bench());
        all(baseline, 0);
        all(tuned, 0);
        CHECK(baseline.capture_zero(1u << Roll, 0) == Result::Ok);
        CHECK(tuned.capture_zero(1u << Roll, 0) == Result::Ok);
        CHECK(tuned.set_tuning(Roll, {2, 40, 5, 500}, 0) == Result::Ok);
        CHECK(baseline.enable(1u << Roll, 0) == Result::Ok);
        CHECK(tuned.enable(1u << Roll, 0) == Result::Ok);
        float q[5] = {0, 0, 0, 1, 45};
        CHECK(baseline.set_targets(1u << Roll, q, 0) == Result::Ok);
        CHECK(tuned.set_targets(1u << Roll, q, 0) == Result::Ok);
        baseline.step(0, f);
        tuned.step(0, f);
        CHECK(tuned.status().joint[Roll].effort > baseline.status().joint[Roll].effort * 1.9f);

        auto c = bench();
        c.joint[Gripper].velocity_kp = 100000;
        ArmControl gripper(c);
        all(gripper, 0);
        CHECK(gripper.capture_zero(1u << Gripper, 0) == Result::Ok);
        CHECK(gripper.set_tuning(Gripper, {2, 100000, 5, 40}, 0) == Result::Ok);
        CHECK(gripper.set_effort_limit(Gripper, 150) == Result::Ok);
        CHECK(gripper.enable(1u << Gripper, 0) == Result::Ok);
        CHECK(gripper.set_targets(1u << Gripper, q, 0) == Result::Ok);
        gripper.step(0, f);
        CHECK(gripper.status().joint[Gripper].effort == 40);
        gripper.disable();
        CHECK(gripper.set_tuning(Gripper, {2, 100000, 5, 500}, 0) == Result::Ok);
        CHECK(gripper.set_tuning(Gripper, {2, 100000, 5, 501}, 0) == Result::InvalidArgument);
        ArmControl restarted(c);
        JointTuning original{};
        CHECK(restarted.get_tuning(Gripper, original) && original.effort_limit == 500);
    }
    { // HAL tick绕回不能触发误失联；原码8192不是有效编码器值。
        ArmControl a(bench());
        const uint32_t t = 0xfffffffeu;
        all(a, t);
        a.capture_zero(kAllJoints, t);
        a.enable(1, t);
        a.step(t, f);
        all(a, 0xffffffffu);
        a.step(0xffffffffu, f);
        all(a, 0);
        a.step(0, f);
        CHECK(a.status().faults == 0);
    }
}
} // namespace
extern "C" void test_entry() {
    run();
    test_done = 1;
    __asm volatile("bkpt #0");
    for (;;) {}
}
