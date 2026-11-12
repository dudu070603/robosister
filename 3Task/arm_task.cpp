#include "arm_task.h"
#include "Arm/arm_control.h"
#include "CAN_bsp.h"
#include "dr16_bsp.h"
#include "robot_control_internal.h"
#include "robot_control.h"
#include "task_ins.h"
#if ARM_MOTOR_JOG_TEST
#include "Remote/operator_control.h"
#endif
#include <cstring>
#include <optional>

extern "C" {
ArmStatus g_arm_status{};
volatile ArmTuneRequest g_arm_tune_request{};
ArmTuneStatus g_arm_tune_status{};
#if ARM_MOTOR_JOG_TEST
volatile int32_t g_arm_jog_command = 0;
#endif
}

namespace {
using namespace engineer;
enum Kind : uint8_t { Zero, Enable, Clear, Target };
struct Command {
    Kind kind;
    uint8_t mask;
    float q[kJointCount];
    uint32_t submitted_ms;
};
struct Received {
    uint8_t bus, len;
    uint16_t id;
    uint32_t ms;
    uint8_t data[8];
};
osMessageQueueId_t commands = nullptr, feedback = nullptr;
volatile bool stop_requested = false;
volatile uint32_t dropped_rx = 0;
#if ARM_MOTOR_JOG_TEST
bool jog_permitted(uint32_t now) {
    DR16Status remote{};
    DR16_GetStatus(&remote);
    const auto &c = kOperatorConfig;
    if (!remote.valid || remote.consecutive_frames < 3 ||
        uint32_t(now - remote.received_ms) > c.remote_timeout_ms ||
        c.left_switch_index >= 2 || c.right_switch_index >= 2 ||
        remote.frame.switches[c.left_switch_index] != DR16_SWITCH_MID ||
        remote.frame.switches[c.right_switch_index] != DR16_SWITCH_MID)
        return false;
    const float neutral = c.deadband * 660.0f;
    for (unsigned i = 0; i < 4; ++i)
        if (remote.frame.channel[i] < -neutral || remote.frame.channel[i] > neutral)
            return false;
    return true;
}
#else
void apply_tune_request(ArmControl &arm, uint32_t now) {
    ArmTuneRequest request{};
    taskENTER_CRITICAL();
    request.joint = g_arm_tune_request.joint;
    request.position_kp = g_arm_tune_request.position_kp;
    request.velocity_kp = g_arm_tune_request.velocity_kp;
    request.velocity_ki = g_arm_tune_request.velocity_ki;
    request.effort_limit = g_arm_tune_request.effort_limit;
    request.request_seq = g_arm_tune_request.request_seq;
    taskEXIT_CRITICAL();
    if (request.request_seq == g_arm_tune_status.consumed_seq)
        return;
    const JointTuning tuning{request.position_kp, request.velocity_kp, request.velocity_ki,
                             request.effort_limit};
    const auto result = arm.set_tuning(request.joint, tuning, now);
    auto next = g_arm_tune_status;
    next.consumed_seq = request.request_seq;
    next.result = uint32_t(result);
    if (result == Result::Ok) {
        next.active_seq = request.request_seq;
        next.active_joint = request.joint;
        next.position_kp = tuning.position_kp;
        next.velocity_kp = tuning.velocity_kp;
        next.velocity_ki = tuning.velocity_ki;
        next.effort_limit = tuning.effort_limit;
    }
    taskENTER_CRITICAL();
    g_arm_tune_status = next;
    taskEXIT_CRITICAL();
}
#endif
// RX对象在任务栈上常驻；回调只复制到队列，不在ISR里做闭环/浮点解算。
void receive_callback(can_device_receive *dev, uint8_t *data) {
    if (!feedback)
        return;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!(kArmConfig.joint_mask & (1u << i)))
            continue;
        const auto &c = kArmConfig.joint[i];
        auto *handle = c.bus == 1 ? &hcan1 : &hcan2;
        uint16_t id = c.motor == Motor::Dm4310Mit
                          ? c.feedback_id
                          : uint16_t((c.motor == Motor::M2006 ? 0x200 : 0x204) + c.motor_id);
        if (!dev->matches(handle, id))
            continue;
        Received f{};
        f.bus = c.bus;
        f.id = id;
        f.len = dev->rx_dlc;
        f.ms = HAL_GetTick();
        std::memcpy(f.data, data, 8);
        if (osMessageQueuePut(feedback, &f, 0, 0) != osOK)
            ++dropped_rx;
        break;
    }
}
int submit(Kind kind, uint8_t mask, const float *q = nullptr) {
    if (!commands)
        return 0;
    Command c{};
    c.kind = kind;
    c.mask = mask;
    c.submitted_ms = HAL_GetTick();
    if (q)
        std::memcpy(c.q, q, sizeof(c.q));
    return osMessageQueuePut(commands, &c, 0, 0) == osOK;
}
void publish(const ArmControl &arm, uint32_t now) {
    ArmStatus next{};
    const auto &s = arm.status();
    next.faults = s.faults;
    next.enabled_mask = s.enabled_mask;
    next.updated_ms = now;
    next.yaw_limit_version = arm.yaw_limits().version;
    next.last_result = uint8_t(s.last_result);
    next.ready = 1;
    next.configuration_valid = s.configuration_valid;
    next.rx_dropped = dropped_rx;
    for (unsigned b = 0; b < 2; ++b)
        next.tx_errors[b] = can_tx_error_count[b];
    for (unsigned i = 0; i < kJointCount; ++i) {
        const auto &j = s.joint[i];
        auto &d = next.joint[i];
        d.position_deg = j.position_deg;
        d.velocity_deg_s = j.velocity_deg_s;
        d.target_deg = j.target_deg;
        d.reference_deg = j.reference_deg;
        d.effort = j.effort;
        d.drive_position_deg = j.drive_position_deg;
        d.drive_velocity_deg_s = j.drive_velocity_deg_s;
        d.drive_target_deg = j.drive_target_deg;
        d.drive_reference_deg = j.drive_reference_deg;
        d.drive_reference_velocity_deg_s = j.drive_reference_velocity_deg_s;
        d.feedback_effort = j.motor_effort_feedback;
        d.temperature_c = j.temperature_c;
        d.velocity_target_deg_s = j.velocity_target_deg_s;
        d.received = j.received;
        d.feedback_age_ms = now - j.last_feedback_ms;
        d.faults = j.faults;
        d.online = j.online;
        d.calibrated = j.calibrated;
        d.enabled = j.enabled;
        d.motor_state = j.motor_state;
    }
    taskENTER_CRITICAL();
    g_arm_status = next;
    taskEXIT_CRITICAL();
}
} // namespace

extern "C" int Arm_CaptureZero(uint8_t mask) {
    return submit(Zero, mask);
}
extern "C" int Arm_Enable(uint8_t mask) {
    return submit(Enable, mask);
}
extern "C" int Arm_ClearFaults() {
    return submit(Clear, 0);
}
extern "C" int Arm_SetTargets(uint8_t mask, const float *q) {
    return q ? submit(Target, mask, q) : 0;
}
extern "C" int Arm_Disable() {
    stop_requested = true;
    return 1;
}
extern "C" int Arm_GetStatus(ArmStatus *s) {
    if (!s)
        return 0;
    taskENTER_CRITICAL();
    *s = g_arm_status;
    taskEXIT_CRITICAL();
    return s->ready;
}

extern "C" void arm_control_task(void *argument) {
    DR16_Start();
#if !ARM_MOTOR_JOG_TEST
    commands = osMessageQueueNew(16, sizeof(Command), nullptr);
    if (!commands)
        Error_Handler();
#endif
    feedback = osMessageQueueNew(64, sizeof(Received), nullptr);
    if (!feedback || osKernelGetTickFreq() != 1000)
        Error_Handler();
    ArmControl arm(kArmConfig);
#if !ARM_MOTOR_JOG_TEST
    const unsigned initial_joint = (kArmConfig.joint_mask & (1u << Yaw)) ? Yaw : BigPitch;
    JointTuning initial_tuning{};
    if (arm.get_tuning(initial_joint, initial_tuning)) {
        g_arm_tune_status.active_joint = initial_joint;
        g_arm_tune_status.position_kp = initial_tuning.position_kp;
        g_arm_tune_status.velocity_kp = initial_tuning.velocity_kp;
        g_arm_tune_status.velocity_ki = initial_tuning.velocity_ki;
        g_arm_tune_status.effort_limit = initial_tuning.effort_limit;
    }
#endif
    uint8_t owned_buses = 0;
    const uint32_t period = kArmConfig.control_period_ms;
    if (!arm.status().configuration_valid) {
        publish(arm, HAL_GetTick());
        for (;;) {
            DR16_Service(); // Receiving/recovery must also work with invalid motor config.
            osDelay(4); // 错误配置禁止注册过滤器/发送电机帧
        }
    }
    for (unsigned i = 0; i < kJointCount; ++i)
        if (kArmConfig.joint_mask & (1u << i))
            owned_buses |= 1u << (kArmConfig.joint[i].bus - 1);
    // 构造时注册硬件过滤器。无电机反馈也不会阻塞初始化。
    std::optional<can_device_receive> rx[kJointCount]; // 就地构造，无堆分配；台架不注册yaw
    for (unsigned i = 0; i < kJointCount; ++i)
        if (kArmConfig.joint_mask & (1u << i))
            rx[i].emplace(kArmConfig.joint[i].bus == 1 ? &hcan1 : &hcan2, arm.feedback_id(i),
                          receive_callback);
    uint32_t previous_errors[2] = {0, 0};
    uint32_t previous_drops = 0;
    uint32_t tick = osKernelGetTickCount();
    for (;;) {
        const uint8_t previous_enabled = arm.status().enabled_mask;
        DR16_Service();
        Received f;
        // 消费先前的反馈，保持其原始接收时间，不能用消费时间冒充新反馈。
        for (unsigned n = 0; n < 64 && osMessageQueueGet(feedback, &f, nullptr, 0) == osOK; ++n)
            arm.receive(f.bus, f.id, f.data, f.len, f.ms);
        uint32_t now = HAL_GetTick();
        for (unsigned b = 0; b < 2; ++b) {
            if ((owned_buses & (1u << b)) && can_tx_error_count[b] != previous_errors[b])
                arm.transport_fault();
            previous_errors[b] = can_tx_error_count[b];
        }
        if (dropped_rx != previous_drops)
            arm.transport_fault();
        previous_drops = dropped_rx;
        if (((owned_buses & 1) && (hcan1.Instance->ESR & CAN_ESR_BOFF)) ||
            ((owned_buses & 2) && (hcan2.Instance->ESR & CAN_ESR_BOFF)))
            arm.transport_fault();
        Frame frames[kJointCount]{};
#if ARM_MOTOR_JOG_TEST
        taskENTER_CRITICAL();
        const int32_t request = g_arm_jog_command;
        if (request != 0)
            g_arm_jog_command = 0;
        const bool user_stopped = stop_requested;
        stop_requested = false;
        taskEXIT_CRITICAL();
        now = HAL_GetTick();
        const unsigned count =
            arm.step_jog(now, user_stopped ? kJogStop : request, jog_permitted(now), frames);
#else
        bool user_stopped = false;
        if (stop_requested) {
            taskENTER_CRITICAL();
            stop_requested = false;
            taskEXIT_CRITICAL();
            arm.disable();
            osMessageQueueReset(commands);
            user_stopped = true;
        } else {
            Command c;
            // 一轮只处理启动时已有命令，避免生产者持续入队饿死控制周期。
            for (unsigned n = 0; n < 16 && osMessageQueueGet(commands, &c, nullptr, 0) == osOK;
                 ++n) {
                now = HAL_GetTick();
                if (uint32_t(now - c.submitted_ms) > 50)
                    continue;
                switch (c.kind) {
                case Zero:
                    arm.capture_zero(c.mask, now);
                    break;
                case Enable:
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
                    INS_RevokeCalibrationPermission(now);
#endif
                    arm.enable(c.mask, now);
                    break;
                case Clear:
                    arm.clear_faults();
                    break;
                case Target:
                    arm.set_targets(c.mask, c.q, c.submitted_ms);
                    break;
                }
            }
        }
        now = HAL_GetTick();
        apply_tune_request(arm, now);
        const auto previous_tune_epoch = g_robot_tune_status.epoch;
        const bool previous_tune_ready = g_robot_tune_status.ready;
        Robot_Execute(arm, now, user_stopped);
        if (previous_tune_epoch != g_robot_tune_status.epoch ||
            (previous_tune_ready && !g_robot_tune_status.ready))
            // Drop old motion frames, then republish every held axis below.
            // This cancels stale Yaw commands without changing arm hold targets.
            can_cancel_pending(owned_buses);
        now = HAL_GetTick();
        const unsigned count = arm.step(now, frames);
#endif
        if (previous_enabled && !arm.status().enabled_mask)
            can_cancel_pending(owned_buses);
        for (unsigned i = 0; i < count; ++i) {
            auto &f = frames[i];
            can_device_transmit tx(f.bus == 1 ? &hcan1 : &hcan2, f.id, 8, f.data);
            if (tx.can_send_msg() != osOK) {
                arm.transport_fault();
                can_cancel_pending(owned_buses);
                break;
            }
        }
#if !ARM_MOTOR_JOG_TEST
        Robot_ReportArmState(arm.status());
#endif
        publish(arm, now);
        // 迟到后从当前tick重新排程，禁止连续补跑同一毫秒的控制环。
        const uint32_t current = osKernelGetTickCount();
        tick = int32_t(current - tick) > 0 ? current + period : tick + period;
        osDelayUntil(tick);
    }
}
