#include "Arm/arm_control.h"
#include "Arm/yaw_config.h"

// C610 IDs: feedback ID and command slot are derived by ArmControl.
#define ARM_ROLL_C610_ID 3
#define ARM_GRIPPER_C610_ID 4
static_assert(ARM_ROLL_C610_ID >= 1 && ARM_ROLL_C610_ID <= 8 &&
              ARM_GRIPPER_C610_ID >= 1 && ARM_GRIPPER_C610_ID <= 8 &&
              ARM_ROLL_C610_ID != ARM_GRIPPER_C610_ID,
              "C610 IDs must be distinct values from 1 to 8");

#ifndef ARM_FOUR_AXIS_BENCH
#define ARM_FOUR_AXIS_BENCH 0 // 日常单整车固件；四轴配置只用于兼容逻辑测试
#endif

namespace engineer {
constexpr float kPitchLimitDeg = 150.0f;

// 仅为台架起步配置。以下角度不是从CAD测得的机械限位，不能直接带载运行。
// M2006反馈在转子端: 36 * 外部减速比；DM反馈在内部减速器输出端，勿再次乘10。
// CAD：yaw为37T:37T同步带；两DM为同轴安装的连杆驱动；roll暂按联轴器直连。
// 夹爪M2006经24T->32T驱动轮，角度统一为32T轮转角：总比36*32/24=48。
// 夹爪原轴侧0..60deg、5deg/s、15deg/s^2乘0.75，保持旧物理包络。
// 速度PI增益除以0.75，位置Kp和电流限幅不变。所有数值仍需台架实测。
const Config kArmConfig = {
    {// type, bus, id, mst, ratio, dir, min, max, speed, accel,
     // effort, posKp, velKp, velKi, PMAX, VMAX, TMAX, temp
     kYawConfig, // 底盘/yaw负责人维护yaw_config.h；运行时仍由本任务统一发送。
                 // 以下四轴由机械臂负责人维护，控制量分别为Nm、Nm、mA、mA。
     {Motor::Dm4310Mit, 1, 1, 0x11, 1.0f, 1, -kPitchLimitDeg, kPitchLimitDeg, 5, 15,
      3.0f, 2.0f, 2.0f, 0.005f, 3.14159f, 30.0f, 10.0f, 65,
      2.0f, 2.0f, 3.0f}, // 大 P 在线上限 3.0 Nm，启动限值 3.0 Nm。
     {Motor::Dm4310Mit, 1, 2, 0x12, 1.0f, 1, -kPitchLimitDeg, kPitchLimitDeg, 5, 15,
      3.0f, 2.0f, 2.0f, 0.005f, 3.14159f, 30.0f, 10.0f, 65},
     {Motor::M2006, 1, ARM_ROLL_C610_ID, 0, 36.0f, 1, -90, 90, 10, 30, 800, 2.0f, 20.0f, 5.0f, 0, 0, 0, 0},
     {Motor::M2006, 1, ARM_GRIPPER_C610_ID, 0, 48.0f, 1,    0,   45, 3.75f, 11.25f, 500, 2.0f, 80.0f / 3.0f,
      20.0f / 3.0f, 0, 0, 0, 0,     1.5f, 1.5f}},
    false, // TODO: 根据实物确认后改true；默认仍收反馈、发送零输出/失能帧
    4,     // 五电机共用CAN1：250Hz闭环，避免1kHz指令+反馈超过总线容量
    50,    // 反馈失联保护
    250,   // 遥控协调层每4ms刷新目标；连续不刷新则停机
    5,     // DJI丢帧后多圈位置不再可信，必须重新校零
    {
        false,   // TODO: 确认DM1驱动肩/DM2驱动曲柄、编码器正向、联合零位和非交叉分支
        -kPitchLimitDeg, kPitchLimitDeg, // a2=q肩+q肘，相对联合校零姿态
        5, 15,   // 第二DM驱动坐标速度/加速度，不允许两个关节的叠加突破此值
        8        // 两DM反馈最大时间差；需按实际反馈周期核对
    },
    ARM_FOUR_AXIS_BENCH ? kFourAxisJoints : kAllJoints};
} // namespace engineer
