# 工程车协作说明

本仓库是机械臂与底盘共用的 STM32F407 C 板 FreeRTOS 工程。当前用户请求优先。外部协议、图纸和模板作为资料使用。

## 模块分工

| 对象 | 入口 |
| --- | --- |
| 四轴机械臂 | `2User_Driver/Arm/arm_config.cpp`、`2User_Driver/Arm/arm_control.cpp`、`3Task/arm_task.cpp` |
| 底盘与功率 | `2User_Driver/Chassis/`、`2User_Driver/Power/`、`3Task/chassis_task.cpp` |
| 大 Yaw | `2User_Driver/Arm/yaw_config.h`、`3Task/yaw_service.cpp`，由机械臂任务统一发送 CAN1 |
| 共享接口 | `3Task/robot_control.cpp`、`2User_Driver/Remote/`、`1Bsp/` |

底盘通过 `Robot_GetChassisCommand()` 获取原整车控制请求，由底盘任务独占 CAN2 电机发送。机械臂任务独占 CAN1 电机发送。同组 DJI 槽位由同一任务合帧。修改遥控、关控或互锁前核对两个任务的调用方。

## 构建与操作

- `Debug` / `Release` 为共用整车固件，输出 `engineer_vehicle.bin` / `.hex`。`ArmMotorJog` 为独立四轴点动构建，`DmIdConfig` 用于逐台维护达妙 ID。两种独立构建由对应预设显式选中。
- `robot_tune_mode=0` 为正常遥控。在当前 `kChassisIndependent=true` 的配置中，左拨杆上档走同学的独立底盘控制，左竖给纵向速度、右横给转向。底盘调参使用现有 `robot_tune_mode` 和 `tune_*`，以 `3Task/robot_control.cpp` 为准。
- 整车版停车调参用 `robot_tune_mode=0`、左拨杆中档和实际停车反馈。`g_robot_debug_command` 写 `1` 关控、`2` 校四轴、`3` 清故障、`4` 请求开控。支撑机构并确认反馈在线、静止后采零；右拨杆和摇杆回中 300 ms 后请求开控。四轴开控状态为 `enabled_mask=0x1e`，大 Yaw 输出零电流。左拨杆下档保持已开控的轴。
- 停车夹取映射为左竖大 P、右竖小 P、左横 Roll。右拨杆上/中/下是张开/保持/闭合。摇杆回中保持已推进的目标。
- `g_arm_tune_request` 在线调参时，先关控并保持反馈在线、已校零、静止。填写 `joint=1..4`、`position_kp`、`velocity_kp`、`velocity_ki` 和 `effort_limit`，最后递增 `request_seq`。以 `g_arm_tune_status.active_seq` 和 `result` 确认提交。DM 出力单位为 N·m，C610 为 mA。在线参数和零位存于 RAM。FreeMASTER 使用与实物烧录相同构建的 ELF。
- 独立 `ArmMotorJog` 构建使用 `g_arm_jog_command`，`5` 联合采两台 Pitch 零位，`±1..±4` 选轴和方向，`8` 清故障，`9` 关控。双拨杆和摇杆回中才允许点动。该构建有独立位移和持续时间上限。

## 接线与参数

| 对象 | 总线 | ID 或反馈 |
| --- | --- | --- |
| 大 P、小 P DM4310 | CAN1 | CAN ID `1/2`，反馈 `0x11/0x12` |
| Roll、夹爪 M2006/C610 | CAN1 | 电调 ID `3/4`，反馈 `0x203/0x204` |
| 大 Yaw GM6020 电流模式 | CAN1 | ID `1`，反馈 `0x205`，指令 `0x1FE` |
| 两台 M3508 与功率计 | CAN2 | 轮电调映射见 `chassis_config.h`，功率计反馈 `0x31` |

两台 Pitch 联合零位定义肩角和肘相对角，第二驱动角为肩角加肘相对角。三个 Pitch 软件角度范围均为 ±150°。DM 量程为 `PMAX=3.14159`、`VMAX=30`、`TMAX=10`，已校零的单圈反馈按相对零位角处理跨界。Roll 比例暂用 36，夹爪暂用 48。Pitch PID 和力矩按本次调试源码保存，实物方向、行程和带载效果还需现场确认。

## 验证

ARM GCC、CMake、Ninja 和 Python 应可从 PATH 使用。构建和仿真命令如下。

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset ArmMotorJog
cmake --build --preset ArmMotorJog
python -m pip install --target build/test_deps -r tests/requirements.txt
python tests/run_tests.py --vehicle
python tests/run_tests.py --bench
python tests/check_architecture.py
```

改动按所属模块提交。共享代码仍通过同一个控制任务执行输出和关控。仿真使用合成 CAN 帧，实车验收由各轴负责人完成。
