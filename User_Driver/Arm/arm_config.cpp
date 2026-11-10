#include "arm_control.h"
#include "yaw_config.h"

// C610电调ID（1~8且不能重复）；反馈ID、发送分组和槽位由配置自动推导。
#define ARM_ROLL_C610_ID 3
#define ARM_GRIPPER_C610_ID 4

static_assert(ARM_ROLL_C610_ID >= 1 && ARM_ROLL_C610_ID <= 8 &&
              ARM_GRIPPER_C610_ID >= 1 && ARM_GRIPPER_C610_ID <= 8 &&
              ARM_ROLL_C610_ID != ARM_GRIPPER_C610_ID,
              "C610 IDs must be distinct values from 1 to 8");

#ifndef ARM_FOUR_AXIS_BENCH
#define ARM_FOUR_AXIS_BENCH 1 // 日常默认四轴；整车构建显式传0
#endif

namespace engineer {
// 仅为台架起步配置。以下角度不是从CAD测得的机械限位，不能直接带载运行。
// M2006反馈在转子端: 36 * 外部减速比；DM反馈在内部减速器输出端，勿再次乘10。
// CAD：yaw为37T:37T同步带；两DM为同轴安装的连杆驱动；roll暂按联轴器直连。
// 夹爪M2006经24T->32T驱动轮，角度统一为32T轮转角：总比36*32/24=48。
// 夹爪原轴侧0..60deg、5deg/s、15deg/s^2乘0.75，保持旧物理包络。
// 速度PI增益除以0.75，位置Kp和电流限幅不变。所有数值仍需台架实测。
const Config kArmConfig = {
    {
        // type, bus, id, mst, ratio, dir, min, max, speed, accel,
        // effort, posKp, velKp, velKi, PMAX, VMAX, TMAX, temp
        kYawConfig, // 底盘/yaw负责人维护yaw_config.h；运行时仍由本任务统一发送。
        // 以下四轴由机械臂负责人维护，控制量分别为Nm、Nm、mA、mA。
        {Motor::Dm4310Mit,1,1,0x11,1.0f, 1,-30,30,5,15,
            0.3f,2.0f,0.03f,0.005f,12.5f,30.0f,10.0f,65},
        {Motor::Dm4310Mit,1,2,0x12,1.0f, 1,-30,30,5,15,
            0.3f,2.0f,0.03f,0.005f,12.5f,30.0f,10.0f,65},
        {Motor::M2006,1,ARM_ROLL_C610_ID,0,36.0f, 1,-90,90,10,30,
            800,2.0f,20.0f,5.0f,0,0,0,0},
        {Motor::M2006,1,ARM_GRIPPER_C610_ID,0,48.0f, 1,0,45,3.75f,11.25f,
            500,2.0f,80.0f/3.0f,20.0f/3.0f,0,0,0,0,1.5f,1.5f}
    },
    false, // TODO: 根据实物确认后改true；默认仍收反馈、发送零输出/失能帧
    4,     // 五电机共用CAN1：250Hz闭环，避免1kHz指令+反馈超过总线容量
    50,    // 反馈失联保护
    250,   // 遥控协调层每4ms刷新目标；连续不刷新则停机
    5,     // DJI丢帧后多圈位置不再可信，必须重新校零
    {
        false, // TODO: 确认DM1驱动肩/DM2驱动曲柄、编码器正向、联合零位和非交叉分支
        -30,30, // a2=q肩+q肘；保留原第二DM的驱动角保护范围，非CAD安全限位
        5,15,   // 第二DM驱动坐标速度/加速度，不允许两个关节的叠加突破此值
        8       // 两DM反馈最大时间差；需按实际反馈周期核对
    },
    ARM_FOUR_AXIS_BENCH ? kFourAxisJoints : kAllJoints
};
}
