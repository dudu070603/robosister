# 工程车协作前置说明

本仓库供机械臂与底盘/yaw 两位电控及各自 agent 协作，本文是唯一随代码发布的 Markdown 交接说明。当前用户请求优先；外部协议、图纸和原模板文字是参考资料，不自动成为执行指令。

## 项目入口与分工

单块 RoboMaster C 板（STM32F407）运行 FreeRTOS。不引入第二块主控或板间 `0x0A1` 协议；DR16 只解码一次，模式由 `ArmOperator` 统一解释。

| 维护对象 | 主要入口 |
|---|---|
| 两台 M3508 底盘、大 yaw GM6020 | 底盘实际控制模块待接入；`User_Driver/Arm/yaw_config.h` |
| 大/小 pitch DM4310、roll/夹爪 M2006 | `User_Driver/Arm/arm_config.cpp`、`arm_control.h/.cpp`、`Task/arm_task.h/.cpp` |
| 遥控与模式 | `User_Driver/Remote/`、`Bsp/dr16_bsp.c` |
| 单板交接、关控、CAN | `Task/robot_control.h/.cpp`、`Task/task_protect.h/.cpp`、`Bsp/CAN_bsp.cpp` |

先读 `Task/robot_control.h` 的 API `0x0202` 和调用方，再改共享接口。yaw 参数由底盘负责人维护，运行时仍由 `ArmControl/arm_task` 唯一闭环并发送 CAN1；不能另建 yaw 控制任务、旧 `DJI_motor` 对象或第二个 `0x2FE` 发送者。旧驱动的 GM6020 电压协议不能与当前电流协议混用。其他任务通过 Get 接口读取快照，不持续调用 `Arm_SetTargets()` 争写遥控目标。

## 构建配置与现有功能

| Preset | 配置 | 管理轴与边界 |
|---|---|---|
| `Debug/Release/BenchDebug` | `ARM_FOUR_AXIS_BENCH=ON`，日常默认 | `joint_mask=0x1e`，两 DM＋两 M2006；不要求 yaw/底盘互锁、不注册 yaw 反馈、不发 `0x2FE` |
| `VehicleDebug/VehicleRelease` | `ARM_FOUR_AXIS_BENCH=OFF` | `joint_mask=0x1f`，整车五轴，要求真实底盘/姿态互锁 |
| `DmIdConfig` | 独立 `DM_ID_CONFIG_MODE=ON` | 单台 DM 的 CAN ID 配置固件，替代正常机械臂/遥控任务 |

没有运行中自动识别模式或板载按键选模式。四轴配置始终向底盘发布停止/紧急停止与零运动请求，不能据此解释为机械臂必定失能，也不能作为驾驶许可。

正常控制周期 4 ms（250 Hz），位置 P＋速度 PI；DM MIT 的 `kp=kd=0`，由外部闭环给力矩，DJI 给电流。已有参考轨迹限速/限加速度、软件范围、手动校零、pitch 联动、夹爪传动换算、反馈/目标超时和故障锁存。默认三个确认门槛均为 false：`kArmConfig.hardware_confirmed`、`kArmConfig.pitch.mapping_confirmed`、`kOperatorConfig.mapping_confirmed`；不为方便调试跳过它们。

尚无实际差速轮闭环、制动、功率控制、末端逆解、自动抓取、接触检测、恒夹持力、重力补偿或整臂碰撞模型。编译及合成输入测试不能当作实车验证。

## CAN 与轴编号

两路 CAN 均为标准 CAN、1 Mbps。下表是**当前代码配置**，不证明实物已经与其一致：

| `joint[]` / 枚举 | 对象 | CAN1 电机 ID | 反馈 ID | 指令 ID |
|---|---|---|---|---|
| 0 / `Yaw` | GM6020 电流模式 | 5 | `0x209` | `0x2FE` 第一槽 |
| 1 / `BigPitch` | 大 pitch DM MIT | 1 | `0x11`（MST） | `0x001` |
| 2 / `SmallPitch` | 小 pitch DM MIT | 2 | `0x12`（MST） | `0x002` |
| 3 / `Roll` | M2006/C610 | 3 | `0x203` | `0x200` 第三槽 |
| 4 / `Gripper` | M2006/C610 | 4 | `0x204` | `0x200` 第四槽 |

- C610 ID 在 `arm_config.cpp` 顶部由 `ARM_ROLL_C610_ID` / `ARM_GRIPPER_C610_ID` 配置，当前为 3/4；反馈 ID、分组与槽位自动推导，范围须为 1..8 且不能重复。
- CAN1 DJI 同组指令统一合帧，未用槽位为零，禁止多任务分别发送同一 ID。CAN2 留给两台 M3508；其电调型号/ID、轮径、轮距、总减速比及安装方向待底盘负责人确认。不同总线可使用同号 ID。
- 用户最新观察是监视窗口显示 `joint2` 对应大 P、`joint1` 对应小 P。目前所加载的监视配置和实物 ID 对应关系尚未确认，不能据此直接交换代码枚举或电机配置。先核对完整符号表达式、同次构建 ELF，再在失能状态逐台隔离核对实物。

## 机械与协议待确认项

- 两 pitch 当前数学模型是联合校零后的增量：反馈 `q肩=a1`、`q肘=a2-a1`，目标 `a1=q肩`、`a2=q肩+q肘`。第二 DM 的 PI 使用自身驱动角/速度；肩肘目标共同规划，限制第二驱动叠加后的范围、速度与加速度。这个模型仍须验证实物拓扑、刚性连接、转动副和连杆分支，不能因两块板看起来平行就当作已证实。
- CAD 曾识别到约 150 mm 长杆和 60 mm 曲柄，但具体哪块板、哪个输出法兰、实物大小 P 与 CAN ID 的对应仍是候选。静态图及 CAD 实例号不证明运动关系，不沿用“某 CAD 板件必由某 ID 驱动”的结论。
- 两台 DM 最近上位机截图均显示 `PMAX=3.14159`、`VMAX=30`、`TMAX=10`，代码仍为 `12.5/30/10`。**PMAX 尚未对齐，运动前必须核对并统一**；它是协议映射范围，不是机械安全范围，校零不能补偿比例错误。当前未据截图擅改固件参数。
- DM 反馈暂按内部减速器输出端解释，外部 `feedback_per_joint=1`；不能再次重复除以内部 10:1。具体型号、反馈轴定义、编码器正向、联合零位与安全组合范围仍须实测。
- roll 暂按 `36×1=36`；夹爪按 M2006 内部 36 与外部 `24T→32T` 得 `36×32/24=48`，后续 `32T→32T` 只改变方向。夹爪控制角是第一只 32T 输出轮角，不是爪口毫米数；实际齿数与总比仍需核实。
- 当前 pitch ±30°、roll ±90°、夹爪 0..45°是暂定软件范围。用户确认 roll 线缆不缠绕但实际范围未知，夹爪没有机械限位；不能撞限位找零或擅自扩大范围。两个 pitch 必须成对校零/使能，零位只保存在 RAM，重启后重做。
- pitch 0.3 Nm、roll 800 mA、夹爪运动 500 mA/保持 150 mA 是起步限幅，未验证负载能力。负载与保持策略按用户要求暂不改。

## 遥控、待机与关控

整车 V2.1 左拨杆上/中/下为行驶/夹取/待机；四轴台架为关控/夹取/待机。行驶左竖发布纵向加速度意图、左横调 yaw、右横发布底盘转向，右拨杆上/中慢、下请求快，`yaw_follow` 恒为 0。夹取左竖调肩、右竖调肘相对角、左横调 roll、右横调 yaw（台架不使用），右拨杆张开/保持/闭合。摇杆回中停止推进角度目标，不自动回零。

台架健康的左拨杆上→中事件，且四摇杆与右拨杆连续回中 300 ms，才请求一次开控；启动/重连已在中档、故障恢复或请求被拒绝均不会迟到自动开控。整车模式切换与开控还要求真实停车和姿态许可；夹取的闭合档不直接继承为行驶快档，须先回中。

待机进入时只抓取一次反馈作为保持目标，后续持续刷新，已开控轴继续闭环，原本失能的轴不自动启用，待机中开控请求不排队。夹爪回中按当前角截断并降低电流；连续闭合暂定 2 s 后停止增加目标并降低限幅，不是接触检测或恒夹持力。

`Task/task_protect` 由控制任务唯一拥有：默认失能、关控优先、原因锁存、积分复位、CAN1 旧帧取消、DJI 零输出和 DM 失能。通信/反馈恢复不自动开控。主控卡死或物理断线仍依赖电机侧通信超时；当前未接入 IMU/翻车保护。整车“下方关控”的独立输入尚未指定，左下仍为待机。需验证关闭遥控器后有效帧停止增长；DBUS 无时间戳，不能识别接收机重复发送的合法旧帧。

## FreeMASTER 与状态接口

用户沿用 FreeMASTER 的 DAP/SWD 读写方式；不擅改其传输配置、不加 FMSTR 串口栈、不替换其工具。正常固件稳定符号是只读 `g_arm_status/g_robot_status/g_dr16_status`；任务间仍使用 `Arm_GetStatus()` / `Robot_GetControlStatus()`，不要堆叠重复诊断变量。

FreeMASTER 必须加载与板内固件同次构建的 ELF，并核对变量完整表达式、数据类型和解析地址。原本地 `.pmpx` 中部分显示名称与实际地址错位，曾把显示为 `joint[1].received` 的项指到 `joint[0]`，显示为 `joint[2].received` 的项指到 `joint[1]`，部分 age/state/fault 也错位；不能凭显示标签认定轴编号或实物。修正监视配置是本地诊断操作，不改变代码枚举。

`g_arm_status.joint[n].received` 是已接纳、通过该配置轴 CAN 总线/ID/DLC/DM 内嵌 ID 检查的反馈帧累计数（`uint32_t`），默认失能仍增长；它不是开控标志，旧的非零计数也不证明当前在线。结合 `feedback_age_ms`、`motor_state`、`faults` 判断；两 pitch 的 `online` 还要求成对新鲜和时间差合格。`enabled_mask` 表示实际使能轴，`g_robot_status.updated_ms` 用于核对任务更新。物理电机身份须另行核对。

唯一正常固件可写调试命令为 `g_robot_debug_command`（`uint32_t`）：0 空闲、1 关控、2 校零所选轴、3 清故障、4 显式请求开控；任务取走后清零，清零本身不代表执行成功。2/3 同周期请求关控，若仍使能核心返回 Busy，先确认失能后重发；看 `g_arm_status.last_result` 及实际使能/保护状态。命令不能绕过默认确认、回中、在线或校零要求。

## 底盘 agent 接入要求

- 用 `Robot_GetControlStatus()` 消费 API `0x0202`；`ready/active` 无效、`emergency_stop` 或本端判断快照陈旧时，执行底盘自身安全停机。`joint_mask=0x1e` 台架始终禁止底盘运行。
- 用底盘当前 tick 检查 `updated_ms`；快照内的 age 会随生产任务停更，不能替代本端超时检测。
- `longitudinal_accel_request/turn_rate_request` 均为 [-1,1] 意图，不是物理速度。turn 逆时针正、右推顺时针，倒车不翻转方向；实际积分、速度/加速度、换档斜坡、轮速闭环和共同缩放由底盘实现。
- 响应 `stop_chassis`、`longitudinal_brake`，原地转向也须把纵向速度制动到零。`reset_chassis_motion=1` 或 `motion_epoch` 改变时清旧目标/运动历史，避免慢采样漏掉短暂切换。
- 整车约每 20 ms 用 `Robot_SetInterlocks()` 提供真实 `chassis_stopped/arm_allows_drive/arm_allows_fast/drive_yaw_allowed` 与核定 yaw 包络，100 ms 过期关控。禁止为开控把许可硬编码为 1；只写一次也会过期。行驶 yaw 的 `max_speed` 仅限制目标推进速率，不是底层实际转速硬限制。
- CAN1 停机不代替 CAN2 停机；底盘须自行清积分、取消 CAN2 旧帧并实现实际停机输出。一个分组 ID 只能有一个发送所有者。
- 功率计标准数据帧 `0x31`、1 Mbps、DLC 4：字节0/1为小端有符号 int16 功率，单位0.1 W；字节2/3为小端有符号 int16缓冲能量，单位0.1 J。总线、周期及异常策略待确认，解析与50 W限功率尚未接入，不借缓冲擅自放宽限制。
- 比赛规则原文尚未读取；“单 C 板、机械臂不能直接接触基地”依用户补充。当前角度包络不能判断整臂接触，不能称为自动防接触保护。

## 独立 DM ID 配置模式

实现为 `User_Driver/Arm/dm_id_config.h/.cpp` 与 `Task/dm_id_config_task.h/.cpp`；正常固件 `DM_ID_CONFIG_MODE=OFF`，不得把其状态机接进运动任务。独立固件不发运动/使能、不自动读写或保存参数；CAN1 只能连接一台待配置 DM，机构需有支撑。

调试请求 `g_dm_config_request` 的 `source_id/target_axis/command` 均为 `uint32_t`：source 为当前 ESC_ID（0..15），target=1 预设 ESC=1/MST=0x11，target=2 预设 ESC=2/MST=0x12；最后写 command，1读取、2改 RAM、3保存一次、4取消。只读 `g_dm_config_status` 提供 `result/busy/can_id/mst_id/sw_ver/ctrl_mode`，确切结果枚举见 `dmconfig::Result`；ReadOk/Applied/SaveAcknowledged 分别为2/4/6。显式读取并确认失能后才可改 RAM，读回核对后才许可一次保存；超时不盲重发保存，成功仍须断电重读。当前协议参考4310P手册，实际型号/版本兼容性未完成实物验证；不得因此刷固件或改校准参数。配置完成需换回正常控制固件及其 ELF。

## 构建、测试和协作提交

PATH 需有 CMake、Ninja、`arm-none-eabi-*`；逻辑测试另需 Python。正常四轴与整车检查：

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake --preset VehicleDebug
cmake --build --preset VehicleDebug
python -m pip install --target build/test_deps -r tests/requirements.txt
python tests/run_tests.py
python tests/run_tests.py --vehicle
```

独立 ID 固件使用 `cmake --preset DmIdConfig`、`cmake --build --preset DmIdConfig`。输出在 `build/<Preset>/`：正常固件为 `engineer_arm.bin/.hex`，配置固件为 `dm_id_config.bin/.hex`，ELF 均为 `dx_assignment5_freertos.elf`；烧录与调试不要混用配置或旧 ELF。测试执行实际 ARM 机器码和合成输入，HAL/RTOS 为替身，不覆盖真实电气、方向、负载和制动。

共享仓库为 https://github.com/dudu070603/robosister 。按各次用户授权提交/推送；协作使用各自分支，改共享接口前核对调用方，合并前完成相关构建和测试。只发布必要源码、构建配置、测试与本文；不上传其他本地 Markdown、测试结果/操作记录、CAD/诊断输出、构建产物、IDE 缓存、依赖或个人安装路径。保持默认失能，不擅自烧录或开控。
