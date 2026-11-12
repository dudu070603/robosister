#include "robot_control.h"
#include "robot_control_internal.h"
#include "Remote/operator_control.h"
#include "Chassis/chassis_control.h"
#include "Vehicle/robot_tuning.h"
#include "Imu/attitude_estimator.h"
#include "Arm/yaw_config.h"
#include "chassis_task.h"
#include "arm_task.h"
#include "yaw_service.h"
#include "task_protect.h"
#include "vehicle_state_task.h"
#include "task_ins.h"
#include "dr16_bsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include <cmath>
extern "C" {
RobotControlStatus g_robot_status{};
volatile uint32_t g_robot_debug_command = ROBOT_DEBUG_IDLE;
volatile uint32_t robot_tune_mode = 0;
volatile float tune_current_ma = 0, tune_speed_rpm = 0, tune_yaw_speed_deg_s = 0,
    tune_yaw_target_deg = 0;
volatile float tune_wheel_kp = engineer::kChassisConfig.velocity_kp,
    tune_wheel_ki = engineer::kChassisConfig.velocity_ki, tune_wheel_current_limit_ma = 10000;
volatile float tune_yaw_position_kp = engineer::kYawConfig.position_kp,
    tune_yaw_velocity_kp = engineer::kYawConfig.velocity_kp,
    tune_yaw_velocity_ki = 0, tune_yaw_current_limit_ma = 100, tune_yaw_half_range_deg = 0;
RobotTuneStatus g_robot_tune_status{};
}
namespace {
engineer::Interlocks shared_interlocks;
engineer::ChassisCommand chassis_command;
bool arm_requested = false, stop_requested = false;
bool chassis_stop_latched = false; // Independent chassis STOP; cleared only by explicit command 4.
uint32_t stop_epoch = 0;
engineer::RobotTuning tuning;
bool tune_exit_wait = false;
uint8_t tune_added_mask = 0;
uint32_t neutral_since_ms = 0;
bool neutral_tracking = false;
void zero_tune_targets(float yaw) {
    taskENTER_CRITICAL();
    tune_current_ma = tune_speed_rpm = tune_yaw_speed_deg_s = 0;
    tune_yaw_target_deg = yaw;
    taskEXIT_CRITICAL();
}
uint32_t vehicle_protection(uint32_t now) {
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
    return Vehicle_GetProtectionReasons(now);
#else
    (void)now;
    return 0;
#endif
}
void publish_protection(RobotControlStatus &s) {
    const auto p = Task_Protect_GetStatus();
    s.protection_disabled = p.disabled;
    s.protection_current_reasons = p.current_reasons;
    s.protection_latched_reasons = p.latched_reasons;
    s.protection_last_trip_ms = p.last_trip_ms;
}
void observe_remote(RobotControlStatus &s, const engineer::RemoteInput &r) {
    const auto &c = engineer::kOperatorConfig;
    const uint8_t channels[] = {c.left_y_channel, c.left_x_channel,
                                c.right_y_channel, c.right_x_channel};
    for (unsigned i = 0; i < 4; ++i)
        s.remote_axis[i] = engineer::remote_axis_value(r.frame, channels[i], c.axis_direction[i],
                                                      c.deadband);
    s.remote_switch[0] = c.left_switch_index < 2 ? r.frame.switches[c.left_switch_index] : 0;
    s.remote_switch[1] = c.right_switch_index < 2 ? r.frame.switches[c.right_switch_index] : 0;
}

// Same owner as normal ArmOperator: this path selects a control layer, never a
// second motor or sender. Only already-enabled non-selected axes are held.
bool execute_tuning(engineer::ArmControl &arm, engineer::ArmOperator &controller,
                    const engineer::RemoteInput &remote, const engineer::Interlocks &locks,
                    uint32_t now, uint32_t elapsed,
                    bool stop, bool request_resume) {
    using namespace engineer;
    uint32_t requested, enable;
    float current, speed, yaw_speed, yaw_target, wheel_kp, wheel_ki, wheel_limit;
    JointTuning yaw_tune{};
    taskENTER_CRITICAL();
    requested = robot_tune_mode;
    current = tune_current_ma;
    speed = tune_speed_rpm;
    yaw_speed = tune_yaw_speed_deg_s;
    yaw_target = tune_yaw_target_deg;
    wheel_kp = tune_wheel_kp;
    wheel_ki = tune_wheel_ki;
    wheel_limit = tune_wheel_current_limit_ma;
    yaw_tune = {tune_yaw_position_kp, tune_yaw_velocity_kp, tune_yaw_velocity_ki,
                tune_yaw_current_limit_ma};
    taskEXIT_CRITICAL();
    // 0档底盘直驱：调参退出不等整车 resume/命令4。清掉调参目标后立即回到
    // 正常路径，由独立底盘命令接管；机械臂恢复仍走它自己的命令4流程。
    if (!requested && kChassisIndependent) {
        if (tuning.status().mode || tune_exit_wait) {
            TuneInput normal{};
            normal.mode = TuneNormal;
            normal.now_ms = now;
            tuning.step(normal);
            zero_tune_targets(arm.status().joint[Yaw].position_deg);
            arm.clear_yaw_test();
            if (tune_added_mask) {
                arm.disable_joints(tune_added_mask);
                tune_added_mask = 0;
            }
            controller.enable_result(false);
            tune_exit_wait = false;
        }
        return false;
    }
    if (!requested && !tuning.status().mode && !tune_exit_wait) {
        return false;
    }
    const auto prior = tuning.status();
    // Changing the selected actuator clears old targets once.
    if (requested != prior.mode) {
        zero_tune_targets(arm.status().joint[Yaw].position_deg);
        current = speed = yaw_speed = 0;
        yaw_target = arm.status().joint[Yaw].position_deg;
    }
    enable = requested == ROBOT_TUNE_LEFT_CURRENT || requested == ROBOT_TUNE_RIGHT_CURRENT
                 ? current != 0
             : requested == ROBOT_TUNE_LEFT_SPEED || requested == ROBOT_TUNE_RIGHT_SPEED
                 ? speed != 0
             : requested == ROBOT_TUNE_YAW_SPEED ? yaw_speed != 0
             : requested == ROBOT_TUNE_YAW_POSITION ? 1u : 0u;

    const bool online = remote.valid && uint32_t(now - remote.received_ms) <=
                                           kOperatorConfig.remote_timeout_ms;
    const bool timing_ok = elapsed > 0 && elapsed <= kArmConfig.control_period_ms + 2;
    // Neutral checks apply only when handing control back to normal driving.
    bool neutral = online;
    for (int channel : remote.frame.channel)
        neutral &= std::abs(channel) <= 30;
    const bool switches_valid = kOperatorConfig.left_switch_index < 2 &&
                                kOperatorConfig.right_switch_index < 2;
    neutral &= switches_valid &&
        remote.frame.switches[kOperatorConfig.right_switch_index] == DR16_SWITCH_MID;
    const bool standby = switches_valid &&
        remote.frame.switches[kOperatorConfig.left_switch_index] == DR16_SWITCH_DOWN;
    if (!neutral) neutral_tracking = false;
    else if (!neutral_tracking) {
        neutral_tracking = true;
        neutral_since_ms = now;
    }
    const bool settled = neutral_tracking && uint32_t(now - neutral_since_ms) >=
                                                kOperatorConfig.neutral_ms;
    InsStatus imu{};
    INS_GetStatus(&imu);
    // Until IMU is physically confirmed, limited bench modes do not claim
    // whole-vehicle readiness. Once confirmed, stale/invalid/tipped always stops.
    const bool imu_required = kAttitudeConfig.installation_confirmed &&
                              kAttitudeConfig.thresholds_confirmed;
    const bool imu_ok = !imu_required ||
        (imu.ready && imu.valid && !imu.tipped &&
         uint32_t(now - imu.updated_ms) <= 40 &&
         uint32_t(now - imu.received_ms) <= kAttitudeConfig.max_sample_gap_ms);
    const bool wheel_mode = requested >= ROBOT_TUNE_LEFT_CURRENT &&
                            requested <= ROBOT_TUNE_RIGHT_SPEED;
    // Wheel tuning is independent of arm calibration and IMU commissioning.
    bool fatal = stop || enable > 1 || !online || !timing_ok ||
                 (!wheel_mode && (arm.status().faults || !imu_ok));
    bool holders_still = true;
    const uint8_t enabled = arm.status().enabled_mask;
    for (unsigned i = 0; i < kJointCount; ++i) {
        if (!(enabled & (1u << i)))
            continue;
        const auto &j = arm.status().joint[i];
        if (!wheel_mode)
            fatal |= !j.online || !j.calibrated || j.faults ||
                uint32_t(now - j.last_feedback_ms) > kArmConfig.feedback_timeout_ms;
        holders_still &= std::fabs(j.drive_velocity_deg_s) <=
                              kArmConfig.joint[i].stationary_speed_deg_s;
    }
    if (fatal) {
        arm.disable();
        tune_added_mask = 0;
        zero_tune_targets(arm.status().joint[Yaw].position_deg);
    }

    ChassisStatus wheels{};
    Chassis_GetStatus(&wheels);
    const bool wheels_fresh = wheels.ready && uint32_t(now - wheels.updated_ms) <= 40;
    bool wheel_ready = wheels_fresh && wheels.configuration_valid &&
                        !wheels.faults && !wheels.tune_blocked;
    for (unsigned i = 0; i < 2; ++i)
        wheel_ready &= wheels.feedback_frames[i] && uint32_t(now - wheels.feedback_ms[i]) <=
                                                         kChassisConfig.feedback_timeout_ms;
    const auto &yaw = arm.status().joint[Yaw];
    const bool yaw_mode = requested == ROBOT_TUNE_YAW_SPEED ||
                          requested == ROBOT_TUNE_YAW_POSITION;
    const bool yaw_ready = arm.status().configuration_valid && kArmConfig.hardware_confirmed &&
        Yaw_ConfigurationReady(arm) && yaw.online && yaw.calibrated && !yaw.faults &&
        uint32_t(now - yaw.last_feedback_ms) <= kArmConfig.dji_unwrap_gap_ms &&
        yaw.temperature_c < kArmConfig.joint[Yaw].max_temperature_c;
    const bool stopped_wheels = wheels_fresh && !wheels.enabled &&
        ((!wheels.feedback_frames[0] && !wheels.feedback_frames[1]) || wheels.wheels_stationary);
    const bool still = holders_still && (yaw_mode ? stopped_wheels &&
        std::fabs(yaw.velocity_deg_s) <= kArmConfig.joint[Yaw].stationary_speed_deg_s
        : wheels_fresh && wheels.wheels_stationary);
    JointTuning accepted_yaw{};
    arm.get_tuning(Yaw, accepted_yaw);
    const bool yaw_parameters_applied =
        accepted_yaw.position_kp == yaw_tune.position_kp &&
        accepted_yaw.velocity_kp == yaw_tune.velocity_kp &&
        accepted_yaw.velocity_ki == yaw_tune.velocity_ki &&
        accepted_yaw.effort_limit == yaw_tune.effort_limit;
    const bool selected_ok = yaw_mode ? yaw_ready && yaw_parameters_applied : wheel_ready;
    auto state = tuning.step({requested, now,
        (wheel_mode || kOperatorConfig.mapping_confirmed) && !fatal,
        selected_ok, still, fatal, current, speed, yaw_speed, yaw_target,
        yaw.position_deg, arm.yaw_limits().min_deg, arm.yaw_limits().max_deg, enable == 1});

    if (state.changed) {
        zero_tune_targets(yaw.position_deg);
        arm.clear_yaw_test();
        // A Yaw enabled only by its test must not leak into a later wheel test.
        if (tune_added_mask) {
            arm.disable_joints(tune_added_mask);
            tune_added_mask = 0;
        }
        // The non-selected axes keep their existing hold targets/history.
        // Only the previous Yaw test has motion history to discard here.
        const uint8_t freeze = uint8_t((prior.mode == ROBOT_TUNE_YAW_SPEED ||
                     prior.mode == ROBOT_TUNE_YAW_POSITION) ? (arm.status().enabled_mask & 1u) : 0u);
        if (freeze && arm.hold(freeze, now) != Result::Ok)
            fatal = true;
        if (prior.mode == ROBOT_TUNE_YAW_SPEED || prior.mode == ROBOT_TUNE_YAW_POSITION) {
            JointTuning accepted{};
            arm.get_tuning(Yaw, accepted);
            arm.set_effort_limit(Yaw, accepted.effort_limit);
        }
        controller.enable_result(false); // Normal control needs an explicit resume.
        tune_exit_wait = true;
    }

    uint32_t yaw_parameter_result = g_robot_tune_status.yaw_parameter_result;
    if (yaw_mode && !fatal && yaw_ready &&
        (!state.ready || (requested == ROBOT_TUNE_YAW_SPEED ? yaw_speed == 0 :
                           std::fabs(yaw_target - yaw.position_deg) <= .5f))) {
        JointTuning actual{};
        arm.get_tuning(Yaw, actual);
        if (actual.position_kp != yaw_tune.position_kp || actual.velocity_kp != yaw_tune.velocity_kp ||
            actual.velocity_ki != yaw_tune.velocity_ki || actual.effort_limit != yaw_tune.effort_limit)
            yaw_parameter_result = uint32_t(arm.set_tuning_runtime(Yaw, yaw_tune, now));
    }
    if (state.ready && yaw_mode && !fatal) {
        INS_RevokeCalibrationPermission(now);
        if (!arm.status().joint[Yaw].enabled) {
            if (arm.enable(1u << Yaw, now) == Result::Ok)
                tune_added_mask |= 1u << Yaw;
            else
                fatal = true;
        }
        // Test range is the configured wire range, not the Drive posture envelope.
        const auto &l = arm.yaw_limits();
        if (!fatal && arm.set_yaw_envelope(l.min_deg, l.max_deg, l.max_speed_deg_s) != Result::Ok)
            fatal = true;
        JointTuning accepted{};
        arm.get_tuning(Yaw, accepted);
        if (!fatal && arm.set_effort_limit(Yaw, std::fmin(150.0f, accepted.effort_limit)) != Result::Ok)
            fatal = true;
        if (!fatal && requested == ROBOT_TUNE_YAW_SPEED) {
            fatal = arm.set_yaw_test_speed(state.value, now) != Result::Ok;
        } else if (!fatal) {
            arm.clear_yaw_test();
            float target[kJointCount]{};
            target[Yaw] = state.value;
            fatal = arm.set_targets(1u << Yaw, target, now) != Result::Ok;
        }
    } else if (yaw_mode && prior.ready && arm.status().joint[Yaw].enabled) {
        arm.clear_yaw_test();
        if (tune_added_mask) {
            arm.disable_joints(tune_added_mask);
            tune_added_mask = 0;
        } else {
            fatal = arm.hold(1u << Yaw, now) != Result::Ok;
        }
    }
    // Refresh held targets without repeatedly capturing a sagging feedback angle.
    uint8_t held = arm.status().enabled_mask;
    if (state.ready && yaw_mode)
        held &= ~(1u << Yaw);
    if (held && !fatal) {
        float targets[kJointCount]{};
        for (unsigned i = 0; i < kJointCount; ++i)
            targets[i] = arm.status().joint[i].target_deg;
        if (arm.set_targets(held, targets, now) != Result::Ok) {
            fatal = true;
            arm.disable();
        }
    }
    bool all_ready = VEHICLE_SAFETY_ENABLED && kOperatorConfig.mapping_confirmed &&
        arm.status().configuration_valid && kArmConfig.hardware_confirmed &&
        kArmConfig.pitch.mapping_confirmed && Yaw_ConfigurationReady(arm);
    for (unsigned i = 0; i < kJointCount; ++i)
        all_ready &= arm.status().joint[i].online && arm.status().joint[i].calibrated;
    const bool interlock_ok = locks.valid && uint32_t(now - locks.received_ms) <=
                                                kOperatorConfig.interlock_timeout_ms;
    if (!requested && tune_exit_wait && request_resume && !fatal && online && neutral && settled &&
        !standby && all_ready && interlock_ok && vehicle_protection(now) == 0) {
        const TaskProtectInput normal_protection{1, 1, 1, 0, uint8_t(timing_ok),
                                                 uint8_t(arm.status().faults != 0), 0};
        Task_Protect_Tick(&normal_protection, now);
        // The normal controller will recapture targets and re-check all normal
        // mode/interlock rules, preserving already-enabled joints throughout.
        if (Task_Protect_TryArm()) {
            tune_exit_wait = false;
            controller.resume_from_tuning(arm.status(), remote, now);
            g_robot_tune_status.updated_ms = now;
            g_robot_tune_status.mode = g_robot_tune_status.ready = g_robot_tune_status.blocked = 0;
            g_robot_tune_status.held_mask = arm.status().enabled_mask;
            g_robot_tune_status.reason = TuneNormalOperation;
            return false;
        }
    }

    if (fatal) {
        arm.disable();
        tune_added_mask = 0;
        TuneInput failed{};
        failed.mode = requested;
        failed.now_ms = now;
        failed.stop = true;
        state = tuning.step(failed);
    }
    if (fatal || state.blocked) {
        zero_tune_targets(arm.status().joint[Yaw].position_deg);
    }

    TaskProtectInput protection{uint8_t(wheel_mode || kOperatorConfig.mapping_confirmed), uint8_t(online), 1,
        uint8_t(stop), uint8_t(timing_ok), uint8_t(arm.status().faults != 0),
        uint32_t(wheel_mode || imu_ok ? 0 : (imu.tipped ? PROTECT_TIPPED : PROTECT_IMU))};
    Task_Protect_Tick(&protection, now);
    if (fatal || stop)
        Task_Protect_Trip(stop ? PROTECT_STOP_REQUEST : PROTECT_ARM_FAULT, now);
    else if (state.ready)
        Task_Protect_TryArm(); // Only after an explicit, healthy tuning enable.
    RobotControlStatus next{};
    observe_remote(next, remote);
    next.updated_ms = now;
    next.ready = 1;
    next.mode = ROBOT_STANDBY;
    next.active = arm.status().enabled_mask != 0;
    next.joint_mask = kArmConfig.joint_mask;
    next.remote_online = online;
    next.remote_age_ms = now - remote.received_ms;
    next.stop_epoch = stop_epoch;
    next.motion_epoch = state.epoch;
    next.stop_chassis = next.longitudinal_brake = next.reset_chassis_motion = 1;
    next.emergency_stop = fatal;
    for (unsigned i = 0; i < kJointCount; ++i)
        next.target_deg[i] = arm.status().joint[i].target_deg;
    publish_protection(next);
    ChassisCommand command{};
    command.valid = true;
    command.updated_ms = now;
    command.epoch = state.epoch;
    command.stop_epoch = stop_epoch;
    command.tune_mode = uint8_t(state.mode);
    command.tune_epoch = state.epoch;
    command.enabled = state.ready && state.mode >= 1 && state.mode <= 4 && !fatal;
    command.tune_value = state.value;
    command.tune_current_limit_ma = wheel_limit;
    command.tune_velocity_kp = wheel_kp;
    command.tune_velocity_ki = wheel_ki;
    JointTuning actual{};
    arm.get_tuning(Yaw, actual);
    taskENTER_CRITICAL();
    g_robot_status = next;
    chassis_command = command;
    g_robot_tune_status = {now, state.mode, state.epoch, state.reason,
        uint8_t(state.ready && !fatal), uint8_t(state.blocked || fatal), arm.status().enabled_mask,
        yaw_parameter_result, actual.position_kp, actual.velocity_kp,
        actual.velocity_ki, actual.effort_limit};
    taskEXIT_CRITICAL();
    return true;
}
} // namespace
extern "C" int Robot_SetInterlocks(const RobotInterlocks *v) {
    const bool valid = v && v->chassis_stopped <= 1 && v->arm_allows_drive <= 1 &&
                       v->arm_allows_fast <= 1 && v->drive_yaw_allowed <= 1 &&
                       (!v->drive_yaw_allowed ||
                        engineer::Yaw_EnvelopeAllowed(v->drive_yaw_min_deg, v->drive_yaw_max_deg,
                                                      v->drive_yaw_max_speed_deg_s));
    engineer::Interlocks next;
    if (valid) {
        next.valid = true;
        next.received_ms = HAL_GetTick();
        next.chassis_stopped = v->chassis_stopped;
        next.arm_allows_drive = v->arm_allows_drive;
        next.arm_allows_fast = v->arm_allows_fast;
        next.drive_yaw_allowed = v->drive_yaw_allowed;
        if (v->drive_yaw_allowed) {
            next.drive_yaw_min_deg = v->drive_yaw_min_deg;
            next.drive_yaw_max_deg = v->drive_yaw_max_deg;
            next.drive_yaw_max_speed_deg_s = v->drive_yaw_max_speed_deg_s;
        }
    }
    taskENTER_CRITICAL();
    shared_interlocks = next;
    taskEXIT_CRITICAL();
    return valid;
}
extern "C" int Robot_RequestControl() {
    taskENTER_CRITICAL();
    arm_requested = true;
    taskEXIT_CRITICAL();
    return 1;
}
extern "C" void Robot_StopControl() {
    taskENTER_CRITICAL();
    stop_requested = true;
    ++stop_epoch;
    taskEXIT_CRITICAL();
}
extern "C" int Robot_GetControlStatus(RobotControlStatus *v) {
    if (!v)
        return 0;
    taskENTER_CRITICAL();
    *v = g_robot_status;
    taskEXIT_CRITICAL();
    return v->ready;
}
bool Robot_GetChassisCommand(engineer::ChassisCommand *command) {
    if (!command)
        return false;
    taskENTER_CRITICAL();
    *command = chassis_command;
    taskEXIT_CRITICAL();
    return command->valid;
}
void Robot_Execute(engineer::ArmControl &arm, uint32_t now, bool force_stop) {
    using namespace engineer;
    static ArmOperator controller(kArmConfig, kOperatorConfig);
    static bool initialized = false;
    static uint32_t previous_ms = 0;
    if (!initialized)
        Task_Protect_Init();
    Interlocks locks;
    bool arm_request, stop_request;
    uint32_t debug_command;
    taskENTER_CRITICAL();
    locks = shared_interlocks;
    arm_request = arm_requested;
    stop_request = stop_requested;
    arm_requested = false;
    stop_requested = false;
    debug_command = g_robot_debug_command;
    g_robot_debug_command = ROBOT_DEBUG_IDLE;
    taskEXIT_CRITICAL();
    DR16Status rx{};
    DR16_GetStatus(&rx);
    now = HAL_GetTick();
    const uint32_t tune_mode = robot_tune_mode;
    const bool bench = kArmConfig.joint_mask == kFourAxisJoints;
    const bool independent = kChassisIndependent && !bench && tune_mode == ROBOT_TUNE_NORMAL;
    const uint8_t left = kOperatorConfig.left_switch_index < 2
                             ? rx.frame.switches[kOperatorConfig.left_switch_index]
                             : 0;
    const bool parked_tuning = VEHICLE_SAFETY_ENABLED && independent &&
                               (left == DR16_SWITCH_MID || left == DR16_SWITCH_DOWN);
    const uint8_t controlled_joints = parked_tuning ? kFourAxisJoints : kArmConfig.joint_mask;
    // 仍在唯一控制任务内执行；关控优先，不让校零或开控绕过现有保护。
    if (debug_command == ROBOT_DEBUG_STOP || force_stop) {
        taskENTER_CRITICAL();
        ++stop_epoch;
        taskEXIT_CRITICAL();
        stop_request = true;
    }
    if (!force_stop && !stop_request) {
        switch (debug_command) {
        case ROBOT_DEBUG_CAPTURE_ZERO:
            arm.capture_zero(tune_mode == ROBOT_TUNE_YAW_SPEED || tune_mode == ROBOT_TUNE_YAW_POSITION
                                 ? uint8_t(1u << Yaw) : controlled_joints, now);
            stop_request = true;
            break;
        case ROBOT_DEBUG_CLEAR_FAULTS:
            arm.clear_faults();
            stop_request = true;
            break;
        case ROBOT_DEBUG_REQUEST_CONTROL:
            arm_request = true;
            break;
        case ROBOT_DEBUG_IDLE:
            break;
        default:
            stop_request = true;
            break;
        }
    }
    // Independent chassis STOP is latched. A one-cycle stop_request would
    // otherwise allow the next cycle to restart from the still-held stick.
    if (kChassisIndependent) {
        if (force_stop || stop_request)
            chassis_stop_latched = true;
        else if (arm_request)
            chassis_stop_latched = false;
    } else {
        chassis_stop_latched = false;
    }
    RemoteInput input{rx.frame, rx.received_ms, rx.valid && rx.consecutive_frames >= 3};
    const uint32_t elapsed =
        initialized ? uint32_t(now - previous_ms) : kArmConfig.control_period_ms;
    previous_ms = now;
    initialized = true;
    const bool remote_ok =
        input.valid && uint32_t(now - input.received_ms) <= kOperatorConfig.remote_timeout_ms;
    // Direct half-range input is consumed once when changed. Failed changes do
    // not become delayed applications: edit the value again after fixing cause.
    static float previous_half_range = 0;
    const float half_range = tune_yaw_half_range_deg;
    if (half_range != previous_half_range &&
        !(std::isnan(half_range) && std::isnan(previous_half_range))) {
        previous_half_range = half_range;
        g_yaw_limit_request.half_range_deg = half_range;
        ++g_yaw_limit_request.request_seq;
    }
    Yaw_ApplyLimitRequest(arm, controller, locks, shared_interlocks, now);

    if (!bench && execute_tuning(arm, controller, input, locks, now, elapsed,
                                force_stop || stop_request, arm_request))
        return;
    const bool interlock_ok = bench || (locks.valid && uint32_t(now - locks.received_ms) <=
                                                           kOperatorConfig.interlock_timeout_ms &&
                                        (!parked_tuning || locks.chassis_stopped));
    const bool confirmed = kArmConfig.hardware_confirmed && kArmConfig.pitch.mapping_confirmed &&
                           kOperatorConfig.mapping_confirmed &&
                           (bench || (VEHICLE_SAFETY_ENABLED && Yaw_ConfigurationReady(arm)));
    const uint32_t vehicle_reasons = vehicle_protection(now);
    // Parked arm tuning uses real wheel-stop and arm feedback; drive commissioning
    // flags and drive power/attitude readiness belong to the integrated drive path.
    const TaskProtectInput protection_input{
        uint8_t(arm.status().configuration_valid && (parked_tuning || confirmed)),
        uint8_t(remote_ok),
        uint8_t(interlock_ok),
        uint8_t(force_stop || stop_request),
        uint8_t(elapsed > 0 && elapsed <= kArmConfig.control_period_ms + 2),
        uint8_t(arm.status().faults != 0),
        parked_tuning ? vehicle_reasons & (PROTECT_CHASSIS | PROTECT_TIPPED) : vehicle_reasons};
    Task_Protect_Tick(&protection_input, now);
    auto result = controller.step(input, locks, arm.status(), now, arm_request,
                                  Task_Protect_GetStatus().current_reasons != 0, parked_tuning);
    if (bench && result.mode == RobotMode::Safe && result.block == OperatorBlock::NeedArm &&
        Task_Protect_GetStatus().current_reasons == 0)
        Task_Protect_Trip(PROTECT_STOP_REQUEST, now);
    auto reject = [&]() {
        result.motion_epoch = controller.enable_result(false);
        result.wait_neutral = true;
        result.active = false;
        result.disable_arm = true;
        result.emergency_stop = true;
        result.stop_chassis = true;
        result.longitudinal_brake = true;
        result.reset_chassis_motion = true;
        result.fast_requested = false;
        result.drive_gear = DriveGear::Slow;
        result.longitudinal_speed_request = 0;
        result.turn_rate_request = 0;
        result.block = OperatorBlock::ArmUnavailable;
        arm.disable();
    };
    if (!bench && !parked_tuning)
        Yaw_ApplyOperatingEnvelope(arm, locks, result);
    if (result.request_enable) {
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
        // Revoke before ArmControl can enable an output. A stale disabled
        // snapshot cannot grant IMU calibration permission after this fence.
        INS_RevokeCalibrationPermission(now);
#endif
        const bool enabled =
            Task_Protect_TryArm() &&
            (parked_tuning ? arm.enable_parked_tuning(now) : arm.enable(kArmConfig.joint_mask, now)) ==
                Result::Ok;
        if (!enabled) {
            Task_Protect_Trip(PROTECT_ENABLE_REJECTED, now);
            reject();
        } else
            controller.enable_result(true);
    }
    if (Task_Protect_GetStatus().disabled && result.active)
        reject();
    if (result.disable_arm)
        arm.disable();
    else if (result.active) {
        if (arm.set_effort_limit(Gripper, result.gripper_effort_limit) != Result::Ok ||
            arm.set_targets(controlled_joints, result.target_deg, now) != Result::Ok) {
            Task_Protect_Trip(PROTECT_ARM_FAULT, now);
            reject();
        }
    }
    RobotControlStatus next{};
    observe_remote(next, input);
    next.updated_ms = now;
    taskENTER_CRITICAL();
    next.stop_epoch = stop_epoch;
    taskEXIT_CRITICAL();
    next.joint_mask = controlled_joints;
    next.ready = 1;
    next.mode = uint8_t(result.mode);
    next.block = uint8_t(result.block);
    next.active = result.active;
    next.wait_neutral = result.wait_neutral;
    next.independent_stop_latched = chassis_stop_latched;
    next.remote_online = remote_ok;
    next.remote_age_ms = now - input.received_ms;
    next.interlock_age_ms = bench ? 0 : now - locks.received_ms; // 台架不使用此输入，非伪造停车反馈
    next.emergency_stop = result.emergency_stop;
    next.stop_chassis = result.stop_chassis;
    next.longitudinal_brake = result.longitudinal_brake;
    next.reset_chassis_motion = result.reset_chassis_motion;
    next.motion_epoch = result.motion_epoch;
    next.fast_requested = result.fast_requested;
    next.drive_gear = uint8_t(result.drive_gear);
    next.drive_yaw_limited = result.drive_yaw_limited;
    next.longitudinal_speed_request = result.longitudinal_speed_request;
    next.turn_rate_request = result.turn_rate_request;
    next.yaw_relative_deg = result.yaw_relative_deg;
    next.gripper_effort_limit = result.gripper_effort_limit;
    for (unsigned i = 0; i < kJointCount; ++i)
        next.target_deg[i] = result.target_deg[i];
    publish_protection(next);
    // Publish the control input directly from this decision, independently of
    // the diagnostic snapshot. Four-axis and all tuning builds deny driving.
    ChassisCommand command{};
    command.valid = true;
    // 0档底盘直驱：解析值仍来自同一次 observe_remote()，只是不再要求
    // result.active（机械臂开控）和整车/云台/IMU/功率许可。
    if (independent) {
        const bool drive_switch = left == DR16_SWITCH_UP;
        command.independent = true;
        command.enabled = drive_switch && remote_ok && !force_stop && !stop_request &&
                          !chassis_stop_latched;
        command.speed_request = next.remote_axis[0];
        command.turn_request = -next.remote_axis[3];
        command.stop = !command.enabled ||
                       (command.speed_request == 0 && command.turn_request == 0);
        command.brake = !command.enabled || command.speed_request == 0;
        command.reset = !command.enabled;
        command.epoch = result.motion_epoch;
        // 诊断快照跟随底盘实际请求；active/enabled_mask 仍是机械臂真实状态。
        next.emergency_stop = 0;
        next.stop_chassis = command.stop;
        next.longitudinal_brake = command.brake;
        next.reset_chassis_motion = command.reset;
        next.longitudinal_speed_request = command.speed_request;
        next.turn_rate_request = command.turn_request;
        next.fast_requested = false;
        next.drive_gear = ROBOT_GEAR_SLOW;
        next.drive_yaw_limited = 1;
    } else {
#if VEHICLE_SAFETY_ENABLED && !ARM_FOUR_AXIS_BENCH && !DM_ID_CONFIG_MODE
        command.enabled =
            kArmConfig.joint_mask == kAllJoints && result.active && !result.emergency_stop;
#endif
        command.stop = result.stop_chassis;
        command.brake = result.longitudinal_brake;
        command.reset = result.reset_chassis_motion;
        command.speed_request = result.longitudinal_speed_request;
        command.turn_request = result.turn_rate_request;
        command.epoch = result.motion_epoch;
    }
    command.updated_ms = now;
    command.stop_epoch = next.stop_epoch;
    taskENTER_CRITICAL();
    g_robot_status = next;
    chassis_command = command;
    taskEXIT_CRITICAL();
}
void Robot_ReportArmState(const engineer::Status &s) {
    // arm.step中的反馈/驱动/轨迹故障发生在Robot_Execute之后，必须同周期覆盖共享状态。
    taskENTER_CRITICAL();
    const bool tuning_owns = tuning.status().mode != 0 || tune_exit_wait;
    const uint8_t expected = tuning_owns ? g_robot_tune_status.held_mask
                                         : g_robot_status.joint_mask;
    if (s.faults || (g_robot_status.active && s.enabled_mask != expected)) {
        Task_Protect_Trip(PROTECT_ARM_FAULT, HAL_GetTick());
        g_robot_status.active = 0;
        g_robot_status.block = uint8_t(engineer::OperatorBlock::ArmUnavailable);
        if (!chassis_command.independent) {
            g_robot_status.emergency_stop = 1;
            g_robot_status.stop_chassis = 1;
            g_robot_status.longitudinal_brake = 1;
            g_robot_status.reset_chassis_motion = 1;
            g_robot_status.fast_requested = 0;
            g_robot_status.drive_gear = ROBOT_GEAR_SLOW;
            g_robot_status.longitudinal_speed_request = 0;
            g_robot_status.turn_rate_request = 0;
            chassis_command.enabled = false;
            chassis_command.stop = true;
            chassis_command.brake = true;
            chassis_command.reset = true;
            chassis_command.speed_request = 0;
            chassis_command.turn_request = 0;
            chassis_command.tune_value = 0;
        }
        publish_protection(g_robot_status);
        if (tuning_owns) {
            engineer::TuneInput failed{};
            failed.mode = tuning.status().mode;
            failed.now_ms = HAL_GetTick();
            failed.stop = true;
            const auto &blocked = tuning.step(failed);
            zero_tune_targets(s.joint[engineer::Yaw].position_deg);
            g_robot_tune_status.ready = 0;
            g_robot_tune_status.blocked = 1;
            g_robot_tune_status.reason = blocked.reason;
        }
    }
    taskEXIT_CRITICAL();
}
