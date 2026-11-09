# C 板 CAN 配置 DM4310 ID

此模式只用于逐台配置 DM-J4310P-2EC 的 ESC_ID 和 MST_ID。它是独立固件，不运行机械臂闭环或遥控任务；四轴/整车原有配置和三个默认失能确认项保持不变。上电不发配置报文，不自动读、写或保存参数。显式读取后按实际控制模式发送失能帧，得到失能反馈后才允许继续；从不发送使能、运动目标或保存零点命令。

## 固件和 ELF

```powershell
cmake --preset DmIdConfig
cmake --build --preset DmIdConfig
```

- 烧录：`D:\RM_robosister\build\DmIdConfig\dm_id_config.bin`（或同目录 `.hex`）。
- FreeMASTER 使用同次构建的符号：`D:\RM_robosister\build\DmIdConfig\dx_assignment5_freertos.elf`。
- 配置完成后重新烧四轴 `build/Debug/engineer_arm.bin`，FreeMASTER 换回 `build/Debug/dx_assignment5_freertos.elf`。

沿用你现有的 FreeMASTER 连接与变量访问方式。本次不改其配置，不新增通信驱动；ELF 只提供符号和地址，必须与板内固件对应。不要把配置固件的变量地址用于四轴/整车固件。

## 接线

1. 机械臂做好支撑，断电接线；失能时电机不主动保持负载。
2. CAN1 总线上只接 **C 板和一台 DM**。另一个 DM、两台 M2006、yaw、其他 CAN 节点均断开。两台相同 ID 的电机无法靠本模式可靠区分，不能同时配置。
3. C 板 CAN1 的 H 接电机主接口 CAN_H，L 接 CAN_L；按设备规范连接参考地。用电机主口 XT30(2+2) 的电源/CAN 线，不用 3Pin 串口调试口，也不要接 MCU 裸 TX/RX。
4. C 板、电机分别按各自额定电压供电。4310P 有 24V/48V 版本，必须看实物铭牌，不能由 CAD 名称推断。USB/SWD 不能替电机主电源供电。
5. 当前配置固件是标准 CAN、1 Mbps。断电检查总线终端：两端各 120Ω 时 H-L 总电阻约 60Ω；未核实电机内部是否带终端，不要盲目重复加电阻。

不需要 DR16、大 yaw 或底盘在线。配置固件不上电使能；为保证保存前已经失能，每次操作仍会确认电机实际状态，不能只看上电时的默认状态。

## FreeMASTER 简洁变量表

所有字段均为 **32 位无符号整数（uint32_t）**。只读字段不要写入。

| 变量 | 读写 | 用途 |
|---|---|---|
| `g_dm_config_request.source_id` | 写 | 当前电机 ESC_ID，默认 1；本版接受 0..15 |
| `g_dm_config_request.target_axis` | 写 | 目标组：1 → ESC=1/MST=0x11；2 → ESC=2/MST=0x12 |
| `g_dm_config_request.command` | 写 | 最后写；1读取、2改ID到RAM、3保存一次、4取消；处理后自动清0 |
| `g_dm_config_status.result` | 读 | 操作结果，见下表；不要仅凭 command 清0判断成功 |
| `g_dm_config_status.busy` | 读 | 1处理中，0结束；等待结束再发下一条命令 |
| `g_dm_config_status.can_id` | 读 | 已读回的 ESC_ID；结果成功时才作为有效确认 |
| `g_dm_config_status.mst_id` | 读 | 已读回的反馈 MST_ID，建议十六进制显示 |

初次读取再补看 `g_dm_config_status.sw_ver`（固件版本原码）和 `g_dm_config_status.ctrl_mode`（1 MIT、2位置速度、3速度、4力位混控）。日常只看上述7项，无需新增曲线或其他检测变量。本模式不改控制模式；正常机械臂固件要求 MIT。

| result | 含义 |
|---|---|
| 0 | 上电空闲 |
| 1 / 3 / 5 | 正在读取 / 改ID / 保存 |
| 2 | 读取成功，且收到失能状态 |
| 4 | 两个ID写入RAM并从新地址读回成功；尚未保存 |
| 6 | 保存已收到应答，且已等待至少30ms；仍需断电复验 |
| 7 | 已取消，已发生的RAM写入不能撤回，已发出的Flash保存也不能撤销 |
| 8 | 命令或参数不支持，source须0..15、target须1或2 |
| 9 / 10 | 必须先读取成功 / 必须先改ID并读回成功 |
| 11 | 应答超时；先核对接线、电源、1Mbps和当前ID，不要盲目重发保存 |
| 12 | 身份或读回值不一致，停止继续写入 |
| 13 / 14 | 控制模式不支持 / 未确认失能 |
| 15 | CAN发送、总线或接收队列异常 |
| 16 | 忙时收到新命令，当前流程已取消；重新读取后操作 |

## 逐台操作

先配置电机 A。A/B 是本次贴在实物上的标识，不证明哪个驱动大 pitch 或小 pitch；连杆对应关系另行核对，不把 CAD 实例号当 CAN ID。

1. 烧配置固件，加载对应 ELF。上电应见 `command=0`、`busy=0`、`result=0`，无自动配置动作。
2. 设置 `source_id=电机当前ID`（不知道可先试1；手册未确认出厂一定为1），`target_axis=1`，**最后写 `command=1`**。
3. 等待 `busy=0`、`result=2`，记录 `can_id/mst_id/sw_ver/ctrl_mode`。若超时，在只有一台电机的前提下，可将 source_id 逐个改为0..15并分别触发读取；本版不扫描其他地址，也不自动换波特率。
4. 写 `command=2`，等待 `result=4`，核对 `can_id=1`、`mst_id=0x11`。该命令会重新读身份、确认失能，先改MST再改ESC，写后从新地址核对；它不会保存Flash。
5. 写一次 `command=3`，等待 `busy=0`、`result=6`。保存使用内部已经核对的新地址，**无需先改 source_id**。每次保存许可只能使用一次；重复写3会被拒绝，不会循环擦写Flash。
6. 确认 busy=0 后，将电机完全断电再上电。设 `source_id=1`，写 `command=1`，应再次得到 `result=2`、ID=1/0x11，才证明掉电保存成功。
7. 断电，换接电机 B。重新读其原 ID；将 `target_axis=2`，按读取→改ID→保存→断电重读流程操作。B 的预期值是 `can_id=2`、`mst_id=0x12`（十进制18），断电复验时 source_id 要填2。
8. 两台均通过掉电复验后断电恢复接线、烧回四轴固件。ID配置不等于机械参数已确认，不改变三个默认失能门槛。

如果写入中途超时，电机地址可能已经改变：先取消（命令4），分别在原ID和目标ID显式读取确认，或断电恢复尚未保存的参数；不要直接保存。保存超时只能说明未收到确认，不能认定保存失败；先断电读取实际结果。换电机后必须重新读取，不能沿用上一个电机的结果。

## 实现和依据

- `User_Driver/Arm/dm_id_config.*`：无HAL依赖的短状态机，命令一次性消费、应答200ms超时、写后20ms再读、保存应答后等满30ms。只有RID7/8可写。
- `Task/dm_id_config_task.*`：独立任务与两个调试结构，CAN1任意标准反馈ID接收（包括默认MST=0），丢包/发送错误结束操作并取消待发帧。
- `DM_ID_CONFIG_MODE` 默认 OFF，只有 `DmIdConfig` preset 打开；该构建不启动机械臂/遥控任务，CAN1关闭硬件自动重发。正常四轴/整车发送与过滤路径保留。
- 协议按[官方4310P型号手册V1.3](https://github.com/dmBots/DM-J4310P-2EC/blob/master/manual/DM-J4310P-2EC%20%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%20V1.3%202026-09-14.pdf)第13–18页：RID7=MST、RID8=ESC，保存命令为 `AA 01`。通用SDK存在 `AA 00` 的差异，本版不自动尝试另一种。若读/改正常但保存无应答，保留固件版本原码并先做掉电验证。

软件编译和合成CAN测试不能代替真实电机、接线、固件兼容性以及掉电保存验证。

2026-11-10 验证：`DmIdConfig`、四轴 `Debug`、整车 `VehicleDebug` 均构建通过。配置固件 Flash 40,020 B、RAM 56,616 B；ELF含两个配置调试结构、不含机械臂/旧DJI/CAN2发送任务。含新增配置状态机用例的四轴44,002条、整车44,003条ARM机器码断言通过，覆盖误帧、迟到、忙时取消、写后读回、保存单次触发和超时。未烧录或操作实物；原机械臂/遥控/保护逻辑源文件与修改前一致。
