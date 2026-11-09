# 遥控操作、四轴台架与底盘接口

整车按《遥控器.docx》V2.1（2026-11-09）实现行驶、夹取、待机，以及默认失能的关控保护。新增独立四轴台架配置，供两台 DM4310、roll 和夹爪 M2006 调试。单块 RoboMaster C 板只解析一次 DR16，沿用唯一控制任务；CAN2 差速车轮控制、实际制动和功率限制仍未接入。

你已确认每车只用一块 C 板、机械臂不能直接接触基地。机械臂与底盘任务共享模式和互锁，不分别解释同一拨杆。V2.1 不启用云台与底盘跟随，旧的登阶、履带和伸收轮遥控路径已移除。

协作分工：机械臂负责人维护两 pitch、roll、夹爪；底盘/yaw负责人维护两台 M3508 和 GM6020。GM6020 参数在 `User_Driver/Arm/yaw_config.h`，仍由原 CAN1 任务统一发送；完整交接见 [AGENTS.md](AGENTS.md)。

## 四轴台架：选配置与操作

| 构建配置 | 控制轴 | 左拨杆上/中/下 | 外部底盘互锁 |
|---|---|---|---|
| `Debug/Release/BenchDebug`，`ARM_FOUR_AXIS_BENCH=ON`（默认） | `joint_mask=0x1e`，两 pitch＋roll＋夹爪 | 关控/夹取/待机 | 不使用，不需要伪造许可 |
| `VehicleDebug/VehicleRelease`，`ARM_FOUR_AXIS_BENCH=OFF` | `joint_mask=0x1f`，整车五轴 | 行驶/夹取/待机 | 必须持续提供真实状态 |

当前按用户选择固定四轴调试：日常构建 `Debug` 并烧录 `build/Debug/engineer_arm.bin`，以后每次上电均为四轴，无需按 C 板按钮或再次烧录来选择模式；重启仍需校零和开控。未来整车联调时才切换至 `VehicleDebug` 固件。

台架不注册 yaw 反馈、不检查 yaw 在线/校零、不发送 GM6020 的 `0x2FE`，只管理四轴 CAN1 报文。CAN2 没有实际控制；台架始终发布 `stop_chassis=1`、`emergency_stop=1` 和零底盘运动请求。因此判断机械臂是否已开控应看 `active` 和 `g_arm_status.enabled_mask`，不能把此处用于底盘的 emergency 标志理解为四轴必定失能。

台架中左竖调肩、右竖调肘相对角、左横调 roll；右横不控制电机，但开控时同样必须回中。右拨杆上张开、中保持、下闭合。左下待机只保持已经开控的关节，左上才是台架关控；这些改动不改变整车 V2.1。

按以下顺序调试，具体传动和待确认参数见 [README_ARM.md](README_ARM.md)：

1. 用日常 `Debug` 构建（原 `BenchDebug` 也可），烧录与调试加载同目录的固件和 ELF；确认 `g_robot_status.joint_mask=0x1e`。支撑机构，先在默认失能下核对四轴 ID、反馈、正向、遥控通道和协议量程。
2. `hardware_confirmed`、`pitch.mapping_confirmed`、遥控 `mapping_confirmed` 仍默认 false；只有对应实物参数确认后再修改、重新构建。台架配置不会跳过这些门槛，也不会自动校零。
3. 左拨杆上档，确认 `enabled_mask=0`。需要清故障时先排除原因，再写调试命令 3；将四轴手动摆到约定零位并固定支撑、静止且反馈在线后写命令 2，检查 `last_result=0` 和四轴 `calibrated=1`。两 pitch 必须联合校零；夹爪 0 必须是人工定义的安全张开参考，因为当前张开目标为 0°，不能任意摆放后沿用 0..45°。重启或反馈多圈失效后重做。
4. 保持四摇杆回中、右拨杆中档，在健康连接中将左拨杆从上拨到中；连续中立 300 ms 后尝试一次开控。确认 `active=1`、`enabled_mask=0x1e` 后才进行小角度、低输出试动。
5. 左下进入待机保持，返回中档需重新通过回中等待；左上立即关控。关控/失联/故障后，排除原因、必要时清故障与重新校零，再重新执行上→中。

启动或重连时左杆已经在中档不会开控。上→中事件期间非中立、反馈/校零未就绪、故障或开控被拒绝都会丢弃这次事件；之后即使条件变好也不会迟到自动开启，须重新上→中。显式 `Robot_RequestControl()` 或调试命令 4 也只消费一次，不能绕过回中/反馈/校零/保护检查；待机中请求直接忽略，不留待以后执行。

## 少量调试变量与命令

`g_arm_status`、`g_robot_status`、`g_dr16_status` 是原有三个快照的稳定全局符号，没有新增诊断副本。调试器只读取它们；其他任务仍调用 `Arm_GetStatus()/Robot_GetControlStatus()/DR16_GetStatus()` 获取快照。关节数组下标固定：0 yaw、1 大 pitch、2 小 pitch、3 roll、4 夹爪；四轴台架忽略下标 0。

| 先观察的字段 | 用途 |
|---|---|
| `g_dr16_status.frame.channel[0..3]/frame.switches[0..1]` | 首次核对通道/左右拨杆；摇杆约 ±660、回中 0，拨杆原码上=1/下=2/中=3 |
| `g_robot_status.joint_mask/active/block` | 当前配置、是否已开控、被哪一层阻塞；block 名称见 `OperatorBlock` |
| `g_robot_status.protection_current_reasons/protection_latched_reasons` | 当前关控原因与锁存原因 |
| `g_arm_status.faults/enabled_mask/last_result` | 故障、实际使能轴、最近核心操作结果 |
| `g_arm_status.joint[i].online/calibrated` | 仅排查未就绪时展开 |
| `g_arm_status.joint[i].position_deg/target_deg` | 只展开正在调试的轴，看关节反馈和目标 |
| `g_arm_status.joint[i].drive_position_deg/drive_target_deg/effort` | pitch 驱动角换算、跟踪与输出；DM 为 Nm，M2006 为 mA |

只有 `volatile uint32_t g_robot_debug_command` 是可写调试入口，每轮读取后自动清零；一次写入一个值，不同时反复写多个命令，也不要直接改状态快照或内存中的目标。

| 写入值 | 执行含义 |
|---|---|
| 0 | 空闲 |
| 1 | 关控 |
| 2 | 校零当前配置全部轴：台架 `0x1e`，整车 `0x1f`；同周期请求关控 |
| 3 | 清底层故障；同周期请求关控 |
| 4 | 显式请求开控，仍需满足所选配置所有条件 |

命令 2/3 在轴仍使能时由核心拒绝并返回 `Busy=3`，该周期仍关控；应先关控确认失能，再重新发送。它们不隐式开控。校零/清故障结果看 `g_arm_status.last_result`；开控看实际 `enabled_mask/active` 与保护原因，命令清零本身不表示成功。完整结果码在 [README_ARM.md](README_ARM.md)。关控优先于同周期的其他请求。

本工程没有 FMSTR 通信栈，也没有本机 FreeMASTER 验证。NXP [FreeMASTER 官方说明](https://docs.mcuxpresso.nxp.com/mcuxsdk/latest/middleware/freemaster/doc/user_guide/user_guide.html) 限定该工具及通信驱动用于 NXP MCU/MPU，本机 STM32F407 不作为其移植目标。建议用现有 STM32 调试器的 Live Expressions/实时变量窗口观察上述符号；这不需要增加串口调试协议。是否支持运行中读写取决于实际调试器；不要靠打断点暂停运动中的控制任务来采样。

## 整车 V2.1 操作映射

| 输入 | 行驶 | 夹取 | 待机 |
|---|---|---|---|
| 左拨杆 | 上档 | 中档 | 下档 |
| 左摇杆上下 | 归一化纵向加速度请求 | 肩角 `BigPitch` 目标变化速度 | 不接受新动作 |
| 左摇杆左右 | 独立 yaw 目标变化速度，受行驶包络许可 | roll 目标变化速度 | 不接受新动作 |
| 右摇杆上下 | 不使用 | 肘相对角 `SmallPitch` 目标变化速度 | 不接受新动作 |
| 右摇杆左右 | 底盘角速度请求，不控制 yaw | yaw 目标变化速度 | 不接受新动作 |
| 右拨杆上 | 慢速档 | 张开 | 无动作 |
| 右拨杆中 | 慢速档，默认 | 截断开合目标并限流保持 | 无动作 |
| 右拨杆下 | 请求快速，仍受 `arm_allows_fast` 许可 | 闭合 | 无动作 |

四摇杆经过死区处理。机械臂输入按关节端 deg/s 积分形成角度目标，回中后停止推进目标，不自动回机械零位。底盘 `turn_rate_request` 是 [-1,1] 的角速度意图，逆时针为正；右横原码经方向校准后右推为正，输出 `-rx`，即右推令车头顺时针。前进和倒车均保持此约定。

行驶的左横与底盘右横独立，`yaw_follow` 始终为 0。行驶时其余机械臂关节不接收摇杆新目标。左右摇杆通道、正向以及“云台 yaw/大 yaw”是否确为同一 GM6020 轴，仍需在实物上核对。

两个 pitch 的公共目标是相对于联合校零姿态的肩角和肘相对角。底层换算 `a1=q肩`、`a2=q肩+q肘`，反馈 `q肘=a2-a1`。两关节可同时操作；遥控层共同缩放达到组合限位的增量，底层联合规划参考轨迹并限制第二驱动的叠加速度、加速度。既有换算与夹爪 48 传动比保留。

## 整车模式切换与通用待机策略

进入行驶或夹取、重新获得姿态许可、以及重新开控时，先请求底盘停车；确认 `chassis_stopped` 后，四摇杆和右拨杆须连续回中 300 ms。行驶与夹取双向切换均如此。正常行驶开始后不再持续要求 `chassis_stopped=true`，避免一运动就禁止行驶。

夹取中右拨杆下档不能直接继承为行驶快速档。切入行驶必须先回中，初始为慢速；此后重新拨下才请求快速。快速许可不满足时，`fast_requested=1` 但 `drive_gear=SLOW`。此版只发布档位，底盘还须实现速度、角速度、加速度约束及平滑换档。

待机是正常模式。进入时只抓取一次当前反馈，经过合法范围约束后作为保持目标；已开控关节继续刷新目标，避免 250 ms 目标看门狗超时。停留待机期间不追随反馈下垂反复重抓目标，也不接收摇杆和右拨杆新动作。原本失能的轴保持失能，待机中的 `Robot_RequestControl()` 被忽略且不会留待退出后执行。

这是候选的位置闭环保持策略，参考轨迹仍按原速度、加速度限制收敛，不能等同于瞬间机械制动或经过验证的承重保持。pitch 暂定 0.3 Nm 上限可能不足以承载；没有重力补偿或抱闸。负载保持策略本轮按用户要求暂不改，后续再实测核定。整车离开待机重新经过停车和回中检查；台架只需回中检查。

## 关控保护

`Task/task_protect.h/.cpp` 参考 `RM_task_Chassis/Task/task_protect.c` 的默认禁止输出、保护优先、取消旧指令再停机结构。没有复制参考工程的两板 `0x0A1` 通信、IMU/翻车条件、21/30 ms 超时或自动恢复行为；这些输入尚未接入本工程。

每 4 ms 控制周期在处理运动输出前汇总保护输入。`Task_Protect_Init()` 默认失能；健康输入只清当前原因，不自动开控。所选配置的回中、反馈、校零等条件就绪后，显式开控请求（台架也可用上→中事件）才尝试解锁；整车另要求真实停车与互锁。开控被拒绝、输出提交失败或底层故障仍会锁存停机；底层控制计算期间发现的故障同周期覆盖共享状态。

| 原因位 | 名称 | 含义 |
|---|---|---|
| bit0 | `PROTECT_BOOT` | 初始等待明确开控 |
| bit1 | `PROTECT_CONFIG` | 硬件、pitch 或遥控映射未确认，或保护输入非法 |
| bit2 | `PROTECT_REMOTE` | DR16 无效或超过 100 ms 未收到有效报文 |
| bit3 | `PROTECT_INTERLOCK` | 整车底盘/安全互锁无效或超过 100 ms 未更新；四轴台架不要求此输入 |
| bit4 | `PROTECT_STOP_REQUEST` | 显式关控或 `Arm_Disable()` 请求 |
| bit5 | `PROTECT_TIMING` | 控制周期为 0 或超过当前 4+2 ms 门槛 |
| bit6 | `PROTECT_ARM_FAULT` | 机械臂底层故障、异常失能或目标提交失败 |
| bit7 | `PROTECT_ENABLE_REJECTED` | 条件检查后实际使能仍被拒绝 |

保护触发后使当前配置全部轴失能，清除闭环积分，取消 CAN1 待发旧帧，并持续发送所管理电机的 DJI 零指令和 DM 失能帧。台架不额外发送 yaw 帧。关控优先于同周期开控，恢复通信不会自动使能。待机本身不触发保护失能；负载保持策略本轮暂不改。

该逻辑仍依赖 DR16 接收机失联后停止输出有效报文。必须实测关闭遥控器后 `good_frames` 停止增长；若接收机重复发送合法旧帧，当前无时间戳的 DBUS 报文不能独自辨别。软件超时还包含任务、总线和电机响应时间，并不能代替电机侧通信超时。

整车新版“下方关控”尚未指定物理输入；已有 `Robot_StopControl()` 和遥控失联入口，整车左下仍为待机。四轴台架单独定义左上关控。若整车需要独立按钮关控，仍需明确通道；若指遥控电源，则需验证有效报文停止的实际行为。

`Task_Protect_*` 状态只由机械臂控制任务拥有。其他任务读取 `Robot_GetControlStatus()` 内的保护快照，不并发调用保护状态机；`protection_current_reasons` 表示当前原因，`protection_latched_reasons` 保留历史原因直至明确重新开控。

## 行驶 yaw 包络

底盘/安全管理通过 `Robot_SetInterlocks()` 提供 `drive_yaw_allowed` 以及已核定的最小角、最大角、目标变化速度。角度相对于 yaw 手动校零；范围和速度必须处于全局 yaw 配置内。未提供许可时只禁止行驶中的新 yaw 操作，仍可按其他许可控制底盘。

`drive_yaw_max_speed_deg_s` 只限制遥控角度目标的推进速率。底层参考轨迹和位置 P 计算出的速度仍使用全局 yaw 上限，不能把该字段称为实际转速的硬限制；如果实车需要更严格的动态转速限制，应另行落实到底层并验证。

有效包络缩小后，如果 yaw 反馈、目标或参考已经在新范围外，程序请求底盘停车、停止新操作并等待包络/姿态问题处理，再经过停稳回中；不会擅自把机构拉回新边界。撤销 yaw 许可时截断到当前角度，已有参考仍按底层轨迹收敛。该接口限制的是已知关节角和目标推进速度，不是整臂碰撞模型，也不能证明机械臂不会触及基地。

## 夹爪处理

张开、闭合均按配置速度逐步改变 32T 输出轮角目标，运动电流上限暂定 500 mA。右拨杆回中时截断到当时反馈角并以 150 mA 上限保持。连续闭合超过暂定 2 秒后，每周期把目标重置为当时反馈角并降低到 150 mA；不能称为固定角度锁定、接触检测或恒夹持力。改变拨杆档位会重置闭合计时，返回中档后再闭合是推荐操作。

夹爪反馈总比 `36×32/24=48`，后续 32T→32T 只改变转向。张开/闭合暂定 0°/45°，速度 3.75°/s、加速度 11.25°/s²；这些来自旧物理包络的单位换算，未实机整定。用户已确认夹爪没有机械限位，不能撞限位找零；本轮不自动扩大范围。一次两秒闭合最多推进约 7.5°目标，不保证覆盖 45°行程或已经夹紧。需实测零位、闭合方向、可用行程、速度、超时和保持电流。

## 底盘共享接口

`Task/robot_control.h` 的 `ROBOT_CONTROL_API_VERSION=0x0202`，新增 `joint_mask`；底盘应按新结构接入。共享状态通过同一 MCU 内的临界区快照传递；本版本不向未知 CAN2 电机 ID 发送臆造的停车帧。

整车底盘/安全管理约每 20 ms 更新真实互锁状态；100 ms 过期会触发保护。非法布尔值或非法 yaw 包络使互锁立即无效，不继续沿用旧许可。无底盘任务时不能为了开控把全部字段写成 1。

当前尚无持续提供互锁的底盘生产任务，整车必须补齐真实提供者；仅在调试器里写一次，100 ms 后仍会过期。四轴台架通过明确的 `joint_mask=0x1e` 隔离，不依赖外部互锁，也不产生驾驶许可；`interlock_age_ms=0` 仅表示该输入未使用，不是测得底盘停稳。

| `RobotInterlocks` 字段 | 提供方应保证的含义 |
|---|---|
| `chassis_stopped` | 底盘测量确认停稳，不能仅表示目标速度为 0 |
| `arm_allows_drive` | 机械臂姿态允许行驶 |
| `arm_allows_fast` | 姿态、载荷等条件允许快速；仍要求允许行驶 |
| `drive_yaw_allowed` | 当前姿态允许行驶中操作 yaw |
| `drive_yaw_min_deg/max_deg/max_speed_deg_s` | 当前已核定的 yaw 操作包络；不许可时忽略这些数值 |

| `RobotControlStatus` 字段 | 底盘消费规则 |
|---|---|
| `ready/active/emergency_stop` | 未就绪、未开控或紧急停机时执行底盘自身安全停机 |
| `joint_mask` | `0x1f` 整车 / `0x1e` 四轴台架；台架始终停底盘 |
| `updated_ms` | 用底盘当前 tick 检测快照陈旧；`remote_age_ms/interlock_age_ms` 在生产任务停更时也会停更，不能代替本端新鲜度检测 |
| `stop_chassis` | 请求受控停车；底盘实现减速、停止判定和最终输出 |
| `longitudinal_brake` | 将纵向速度制动到 0；右杆原地转向时也不能保持旧纵向速度 |
| `reset_chassis_motion/motion_epoch` | reset 为 1 或代号变化时清旧积分目标/运动历史，按实测策略停车；比较代号可发现两次读取之间短暂发生的模式切换/关控 |
| `longitudinal_accel_request` | [-1,1] 加速度意图，正向前；不是 m/s 或 m/s² |
| `turn_rate_request` | [-1,1] 角速度意图，逆时针正；不是 rad/s，倒车时不改变含义 |
| `fast_requested/drive_gear` | 原始快速请求与许可后档位；快速被拒绝时仍输出慢速档 |
| `yaw_follow/yaw_relative_deg` | 跟随恒为 0；yaw 相对角只作状态信息，不应自行启用跟随 |
| `protection_*` | 保护失能状态、当前/锁存原因及最近触发时刻 |

底盘实际控制仍需完成：纵向加速度积分、回中主动制动、换向减速、快慢档平滑切换、高速转向约束、`v左=v-Bω/2` / `v右=v+Bω/2`、轮速共同缩放、左右轮闭环、反馈失联及 CAN2 旧帧取消/停机输出。轮径、轮距、实际总减速比、电机型号/ID、安装方向、速度/加速度和快照超时阈值均需确认。

功率计尚未接入。已知协议为 1 Mbps、标准 CAN `0x31`、DLC=4，小端有符号 int16 的功率×0.1 W 与缓冲能量×0.1 J；实际总线和发送周期待确认。50 W 是本次遥控协议提出的限制目标，功率控制、异常处理及缓冲使用规则尚未实现，不能擅自借缓冲放开超过 50 W。需要新增并验证后才能称底盘符合该限制。

## 参数入口与整车上车调试

| 文件 | 内容 |
|---|---|
| `User_Driver/Remote/dr16_protocol.c` | 完整 18 字节 DBUS 解码和输入验证 |
| `Bsp/dr16_bsp.c` | DMA+空闲中断、双缓冲、有效帧及错误计数 |
| `User_Driver/Remote/operator_config.cpp` | 实物通道/正向、死区、超时、夹爪占位参数 |
| `User_Driver/Remote/operator_control.cpp` | V2.1 映射、待机、停车回中和姿态许可 |
| `Task/robot_control.h/.cpp` | 单板接口、保护连接、目标提交与状态快照 |
| `Task/task_protect.h/.cpp` | 默认失能、保护原因汇总与重新开控锁存 |
| `User_Driver/Arm/arm_config.cpp`、`yaw_config.h` | 四轴与 yaw 分开维护参数，聚合为统一 CAN1 五轴配置 |

默认仍保留 `hardware_confirmed=false`、`pitch.mapping_confirmed=false`、遥控 `mapping_confirmed=false`。刷入默认固件后不会使能电机，不应把“已经编译通过”当作可直接带载操作。

1. 保持默认失能并支撑机械臂，观察 `DR16_GetStatus()` 的四轴/两拨杆原码、有效帧和错误计数；核对暂定 ch0=右横、ch1=右竖、ch2=左横、ch3=左竖及拨杆索引。USART3 RX PC11，线上 100000 baud、8 数据位+偶校验+1 停止位；F4 HAL 配置为 `9B + EVEN`。
2. 按 `README_ARM.md` 核对电机 ID、模式、DM 量程和反馈、机械传动、方向、零位、限位及负载参数，再有依据地更新三处确认门槛。方向先从反馈和低输出验证，不能只凭 CAD 或相向安装猜测。
3. 支撑并手动摆好约定零位，静止且反馈在线后调用 `Arm_CaptureZero()` 或调试命令 2，读取执行结果。两 pitch 同时校零（mask `0x06`）；整车需五轴均校零。重启或多圈位置失效后需重校，CAD 150°偏置不是软件零位。
4. 让底盘/安全管理持续提供真实停车与姿态许可。行驶或夹取模式下，停稳、四摇杆和右拨杆回中后，明确调用一次 `Robot_RequestControl()`；待机中不会开控。返回 1 只代表请求接收，仍须检查实际状态。
5. 在有支撑、低输出和小角度条件下验证方向、跟踪误差、抖动、饱和与温升，再逐步整定 PID 和速度/加速度。两 pitch 成对运行；完整遥控协调层按五轴开控，不能另起任务争写 `Arm_SetTargets()`。
6. 验证行驶左横 yaw/右横底盘请求互不串扰、夹取映射、切模式停稳回中、快档许可、待机保持和不自动使能；随后验证显式关控、关闭遥控器、互锁失联、反馈故障及恢复后不自动开控。
7. 单独验证电机侧通信超时和真实负载的下垂/制动行为。主控卡死或物理断线时，本机软件无法保证停机帧送达。底盘实际控制接入前，接口测试不能证明车辆已经能够安全停车。

机械臂还需确认的问题、候选保持策略和参数清单见 `README_ARM.md`。

## 比赛规则 V1.1 核对状态

规则链接：<https://ocn02c8hn0lq.feishu.cn/wiki/PibYwSuKxizzPZk8f2IctNwAn5j>。

规则原文暂未读取。每车一块 C 板、机械臂不能直接接触基地以你补充的内容为依据；道具兑换高台、隧道、减速带、空气墙、物资变更的具体尺寸、数量、接触判定和罚则仍待原文核对。当前软限位和行驶 yaw 包络不能判断整条机械臂是否触及基地，不得称为自动防接触保护。

## 构建与验证

四轴日常用 `cmake --preset Debug`、`cmake --build --preset Debug`；整车将两处配置名换成 `VehicleDebug`。分别运行 `python tests/run_tests.py`（四轴）与 `python tests/run_tests.py --vehicle`（整车），依赖安装命令见 [README.md](README.md)。原 `BenchDebug` 和测试 `--bench` 入口保留。构建及测试记录见 [README.md](README.md)。

验证应覆盖整车 V2.1、pitch/夹爪换算及 CAN 保护，并覆盖四轴无 yaw/互锁依赖、不发 yaw 指令、台架上→中一次开控、回中与故障恢复不迟到自动启、调试命令关控优先。合成输入测试不代表真实 UART/CAN、电机负载或底盘制动已经验证；本轮没有自动烧录。

硬件参考：[DJI C 板遥控器示例](https://github.com/RoboMaster/Development-Board-C-Examples/tree/master/9.remote_control_dma)，[ST UART HAL 说明](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)。
