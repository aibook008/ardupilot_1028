# 玄天参数文件差异分析（BMS.param vs wing_servo.param）

> 日期：2026-10-10。来源：`/home/codyqin/Desktop/xuantian/BMS.param`、`wing_servo.param`（同一份基础参数的两个用途变体，仅差 10 处，其中 3 处是有意修改，其余为地磁/陀螺校准、气压零点、开机计数等自然漂移）。
>
> 对照背景：`docs/bms_control_xuantian.md`（BMS 控制链路）、`docs/dronecan_alignment_tables.md`（表2 翼面舵机）。

## 1. 两份文件的有意差异（核心 3 项）

| 参数 | BMS.param（BMS 联调用） | wing_servo.param（翼面舵机联调用） | 修改思路 |
|---|---|---|---|
| `CAN_D1_UC_SRV_BM` | **69631 (0x10FFF)** = bit0~15 + **bit16** | **4095 (0xFFF)** = bit0~15 | bit16 就是 SERVO17。BMS 用途要**额外开放 SERVO17** 走 DroneCAN 作动器命令（actuator_id=17 → POGO → BMS 上/下高压）；翼面舵机用途用不到，收回到 4095 只映射 16 路常规舵机 |
| `SERVO17_FUNCTION` | **0**（GPIO，可被外部临时驱动） | **-1**（完全禁用） | BMS 场景按操作手册要求 `SERVO17_FUNCTION=0`，MP 插件用 `DO_REPEAT_SERVO` 打单脉冲 1900/1100µs，飞控经 DroneCAN 把它当 17 号作动器发出去；翼面舵机场景与高压控制无关，置 -1 防误触发 |
| `BATT_SERIAL_NUM` | **120**（绑定 DroneCAN Battery ID 120） | **-1**（不绑定） | BMS 场景按手册设 120 对接 POGO 发的 BatteryInfo（Node 120）；翼面舵机台架上没有高压电池，解绑避免出现幽灵电池/告警 |

一句话：**两份文件共用一套翼面舵机 + 姿态基础参数；BMS.param = wing_servo + 高压控制三件套（SRV_BM bit16 / SERVO17_FUNCTION=0 / BATT 绑定 120）**。切场景时只需对调这 3 个参数。

## 2. 两份文件共同的 DroneCAN 配置（理解映射的关键）

| 参数 | 值 | 含义 |
|---|---|---|
| `CAN_P1_DRIVER / BITRATE` | 1 / 1000000 | 物理 CAN1 → Driver1，1 Mbit |
| `CAN_D1_PROTOCOL` | 1 | DroneCAN |
| `CAN_D1_UC_NODE` | 10 | 飞控 NodeID=10 |
| `CAN_D1_UC_SRV_RT` | **200 Hz** | 作动器命令广播频率（远高于默认 50，翼面舵机响应带宽需求高） |
| `CAN_D1_UC_OPTION` | **64** = bit6 | 本仓库自定义构建里 bit6 对应 `USE_HIMARK_SERVO`（`AP_DroneCAN.h:148`）。⚠ 与 BMS 手册"bit4 清零发 UNITLESS"的表述对照：bit4（USE_ACTUATOR_PWM）确实为 0，命令值即 UNITLESS，两处说法一致不冲突 |
| `CAN_D1_UC_SRV_BM` | 见上表 | 舵机→DroneCAN 作动器通道的映射位图 |
| `BATT_MONITOR` | 8 | 电池监视器 = DroneCAN（两份都开着，wing 版因 SERIAL_NUM=-1 不认领） |

其余舵机通道 SERVO1~16 已有固定功能映射（SERVO1=51、SERVO2=19、SERVO3=70、SERVO4=21，SERVO5~12=4 等 Plane 舵面功能号），MIN/TRIM/MAX 均为 1100/1500/1900 标准量程——即 16 路翼面舵机走标准 `uavcan.equipment.actuator.ArrayCommand`，与 BMS 的 actuator_id=17 互不干扰。

## 3. 非有意差异（同机两次启动的自然差异，勿手改）

- `BARO1/2_GND_PRESS`：两次上电地面气压不同（~15–17 Pa 差），正常。
- `INS_GYROFFS_*` / `INS_GYR2/3OFFS_*` / `INS_GYR*_CALTEMP`：每次上电陀螺自动校准偏置略不同，正常。
- `STAT_BOOTCNT / STAT_RUNTIME`：开机次数/运行时长计数。

## 4. 参数全称与含义速查

### 4.1 电池监视器（BATT_MONITOR）所有可选"后端"

"后端"=电池数据从哪来（`AP_BattMonitor.h:90` Type 枚举）。MONITOR=8 即"用 DroneCAN 智能电池方式监控"：

| 值 | 类型 | 含义 |
|:--:|------|------|
| 0 | NONE | 不监控 |
| 3 | Analog_Voltage_Only | 模拟分压只测电压 |
| 4 | Analog_Voltage_And_Current | 模拟电压+电流（分流器/霍尔） |
| 5/6 | Solo / Bebop | 机型专用 SMBus |
| 7 | SMBus_Generic | SMBus/I²C 智能电池 |
| **8** | **UAVCAN_BatteryInfo** | **DroneCAN 智能电池（本项目，经 POGO）** |
| 9 | BLHeliESC | 电调遥测回传取电压电流 |
| 10 | Sum | 串联电池组累加 |
| 11/12/24 | FuelFlow / FuelLevel_PWM / FuelLevel_Analog | 燃油量类 |
| 17/18 | Generator_ELEC / Generator_FUEL | 发电机电/油 |
| 21/22/26 | INA2XX / INA239_SPI / LTC2946 | I²C/SPI 功率计芯片 |
| 23 | Torqeedo | 电动推进专用电池 |
| 27 | EFI | 电子喷油器取电 |
| 29 | Scripting | Lua 脚本自定义 |

### 4.2 本项目用到的关键参数释义

| 参数（全称） | 值 | 含义 |
|---|---|---|
| BATT_MONITOR（Battery Monitor Type） | 8 | 电池1 用 DroneCAN 后端 |
| BATT_SERIAL_NUM（Battery Serial Number） | 120 / -1 | 认领 BatteryInfo 的 battery_id；-1 全收 |
| BATT2_MONITOR | 0 | 电池2 槽位禁用（单电池配置） |
| CAN_Px_DRIVER / CAN_Px_BITRATE | 1 / 1000000 | 物理 CAN 口 → 驱动实例映射；比特率 |
| CAN_D1_PROTOCOL | 1 | Driver1 跑 DroneCAN 协议 |
| CAN_D1_UC_NODE（UAVCAN Node） | 10 | 飞控 NodeID |
| CAN_D1_UC_SRV_BM（Servo BitMask） | 69631/4095 | 允许上 CAN 的 SERVO 通道位图 |
| CAN_D1_UC_SRV_RT（Servo Rate） | 200 Hz | ArrayCommand 广播周期 |
| CAN_D1_UC_ESC_BM / ESC_OF / ESC_RV | 0 | 电调位图/下标偏移/可逆掩码（未用） |
| CAN_D1_UC_OPTION | 64 (bit6) | 自定义构建=USE_HIMARK_SERVO；bit4(USE_ACTUATOR_PWM)=0 即 UNITLESS |
| CAN_D1_UC_POOL | 16384 | libcanard 内存池字节数 |
| CAN_D1_UC_NTF_RT（Notify Rate） | 20 Hz | NotifyState 广播频率 |
| CAN_D1_UC_RLY_RT（Relay Rate） | 0 | DroneCAN 继电器关闭 |
| CAN_D1_UC_SER_EN（Serial Enable） | 0 | DroneCAN 串口透传关闭 |
| CAN_D1_UC_TLT_BM（Tilt BitMask） | 0 | 倾转通道位图（BMS.param 中未启用；启用后倾转走独立 TiltServoCommand@TLT_RT，与 SRV_RT=200 互不影响） |
| CAN_D1_UC_TLT_RT / TLT_MX / TLT_P0~3 | 20 Hz / 9000 / -32768 | 倾转广播频率 / 满行程 90.00° / 各通道 PWM 中立映射点（-32768=未标定） |
| SERVO17_FUNCTION | 0 / -1 | 0=GPIO（可被 MAVLink DO_*_SERVO 临时驱动）；-1=禁用 |
| BRD_SAFETY_MASK（Safety Mask） | 0 | 硬件 Safety 按下时仍允许输出的通道位图（SERVO17=bit16，本机为 0 即全被挡，联调时按手册释放） |

## 5. wing_servo.param 的 servo 通道映射方式（RC → 舵机）

### 5.1 现有映射（ wing_servo.param 实际值，功能号对照 SRV_Channel.h）

| SERVO 通道 | FUNCTION 值 | 全称/含义 | 驱动源 |
|---|---|---|---|
| SERVO1 | **51** | k_rcin1 —— **RC1 遥控通道直通** | 遥控器 RC1 摇杆，不经控制律 |
| SERVO2 | 19 | k_elevator 升降舵 | 控制律（PID） |
| SERVO3 | 70 | k_throttle 油门 | 控制律 |
| SERVO4 | 21 | k_rudder 方向舵 | 控制律 |
| SERVO5~12 | 4 | k_aileron 副翼 | 控制律 |
| SERVO13~16 | 0 | k_none 空闲 | 可被 MAVLink 临时驱动（如 BMS 用 SERVO17 同款机制） |
| SERVO17 | -1（wing版）/ 0（BMS版） | — | 见 §1 |

**"遥控器一个通道映射成舵机角度"的链路**（SERVO1 为例）：

```
遥控器 → 飞控 RC1 输入（RC1_MIN/TRIM/MAX=1100/1500/1900, DZ=30）
  → FUNCTION=51 直通（k_rcin1，无 PID，杆量即输出）
  → SERVO1 输出通道（SERVO1_MIN/TRIM/MAX 归一化）
  → SRV_push_servos 采样 → ArrayCommand{actuator_id=1, UNITLESS}
```

"角度映射"由三层参数合成：`RC1_MIN/TRIM/MAX`（摇杆行程→PWM）→ `SERVO1_MIN/TRIM/MAX`（PWM→[-1,+1]，公式 `(pwm-1000)/500-1`，AP_DroneCAN.cpp:767）→ 舵机侧把 [-1,+1] 映射到物理角度（±1 对应多少度须与供应商约定，即 `dronecan_alignment_tables.md` 表2 §2.3）。极性反了改 `SERVO1_REVERSED`；杆量死区 `RC1_DZ`。

## 6. 单翼面舵机配置清单

**核心认知：飞控侧不配置舵机的 Node ID**。ArrayCommand 是广播帧，飞控不知道谁在听；舵机在 payload 里按 `actuator_id` 认领命令。Node ID 只标识帧来源和做点对点服务（参数读写/固件升级），不参与"命令发给谁"的判断（`uav_can.md` §7）。

### 6.1 飞控侧参数

| 参数 | 单舵机建议值 | 说明 |
|---|---|---|
| CAN_P1_DRIVER / CAN_P1_BITRATE | 1 / 1000000 | 物理口 + 1M 波特率 |
| CAN_D1_PROTOCOL | 1 | DroneCAN |
| CAN_D1_UC_NODE | 10 | **飞控自己**的 NodeID，全网唯一即可 |
| CAN_D1_UC_SRV_BM | **1**（只开 bit0=SERVO1） | 从 4095 收紧到只映射实际用到的通道，空通道不上总线 |
| CAN_D1_UC_SRV_RT | 200（够用可降 50） | ArrayCommand 周期 |
| CAN_D1_UC_OPTION | 64（Himark 私有出口）或 0（标准协议） | 决定 `SRV_send_himark` vs `SRV_send_actuator`（AP_DroneCAN.cpp:556-561），须与舵机协议一致 |
| SERVO1_FUNCTION | 51（RC 直通测试）/ 4（接副翼控制律） | 测试用 51 最直观 |
| SERVO1_MIN/TRIM/MAX/REVERSED | 1100/1500/1900/0 | 行程与极性 |

### 6.2 舵机侧需确认项（找供应商，非飞控参数）

1. **舵机自身 Node ID**：在舵机控制器上配置（静态或 DNA 自动分配），1~125 全网唯一、不与飞控 10 冲突；用 DroneCAN GUI 观察在线节点列表验证上线。
2. **认领的 actuator_id = 1**（对应 SERVO1，通道序号+1，AP_DroneCAN.cpp:760）——双方约定，必须与 SERVOx 序号对上；若舵机绑死其他号，改设对应 SERVO 通道的 FUNCTION。
3. **命令值类型**：UNITLESS 还是 PWM（飞控侧由 UC_OPTION bit4 决定），必须与舵机期望一致（表2 §2.2 ★行）。
4. **消息定义**：标准 `uavcan.equipment.actuator.ArrayCommand`(1010) 零对齐成本；私有协议按 `dronecan_alignment_tables.md` 表 2 流程。

### 6.3 actuator_id / servo_id 对应关系（标准协议 vs Himark 私有协议）

**认领机制**：命令帧里没有目的 Node ID，每台舵机靠 payload 里的通道号认领自己的命令。两种协议的通道号语义不同：

**① 标准协议 `uavcan.equipment.actuator.ArrayCommand`(1010)**——通道号是显式字段 `actuator_id`：

```cpp
// SRV_send_actuator（AP_DroneCAN.cpp:747,760）
for (starting_servo = 0; ...; starting_servo++)   // 数组下标 0..15 升序
    cmd.actuator_id = starting_servo + 1;          // actuator_id = SERVOx 序号 + 1
```

| 飞控通道 | SERVO1 | SERVO2 | ... | SERVO16 | SERVO17 |
|---|---|---|---|---|---|
| actuator_id | **1** | **2** | ... | 16 | 17（BMS） |

舵机收帧后按 `actuator_id` 认领：**装升降舵的那台舵机，认领号必须设成 2**，如果飞控把升降配在 SERVO2（FUNCTION=19）。认领号≠Node ID——Node ID 是"我是谁"（全网唯一，做点对点服务），认领号是"我认几号命令"（可任意分配，但必须与飞控侧 SERVOx 对上）。

**② Himark 私有协议 `com.himark.servo.ServoCmd`(2018)**——通道号是**数组下标本身**，没有显式 id 字段：

```python
# modules/DroneCAN/DSDL/com/himark/servo/2018.ServoCmd.uavcan
uint10[<=17] cmd        # cmd[i] 即 SERVO(i+1) 的命令值，700~2300µs 语义见下
```

```cpp
// SRV_send_himark（AP_DroneCAN.cpp:812-818）
for (i = 0; i <= highest_to_send; i++) {
    if ((1U<<i) & _SRV_armed_mask) {
        msg.cmd.data[i] = constrain_int16(_SRV_conf[i].pulse - 1000, 0, 1000);  // PWM-1000
    }
}
msg.cmd.len = highest_to_send + 1;   // 截断到最后一个 pending 通道——数组必然是 0..N 连续填满
```

**Himark 的关键差异/坑**：

| 特性 | 标准 ArrayCommand | Himark ServoCmd |
|---|---|---|
| 通道区分 | 显式 `actuator_id` 字段（1~17） | **数组下标**（0~16，对应 SERVO1~17） |
| 空通道 | 直接跳过、不占数组位 | **必须连续填充**到最高 pending 通道，中间位补当前值——下标 i 的语义永远绑死 SERVO(i+1)，改 SRV_BM 掩码不会改变对应关系 |
| 命令值 | UNITLESS [-1,1] 或 PWM（bit4 选） | `(pulse-1000)`，即 0~1000 µs 偏移量（SERVO1_MIN/MAX=1100/1900 → 100/900） |
| 反馈认领 | `actuator.Status` 带 `id` 字段 | `ServoInfo(2019)` 带 `servo_id` 字段（uint5，**0~17**，DSDL 原文"servo ID from 0 to 17"；0 也可用），飞控仅记日志 CSRV（AP_DroneCAN.cpp:1401-1416），不做配对校验 |

**结论（对应关系怎么配）**：

1. **标准协议**：舵机侧认领号（local_id / actuator_id 绑定值）= 飞控 SERVOx 的 x+1。飞控侧 SERVOx 配什么 FUNCTION（elevator/aileron/rcin 直通）不影响认领号，只影响 command_value 的来源——"一旦 actuator_id=1 分给升降舵，装升降的那台就把认领号设 1"——理解正确，但注意是 **x+1 偏移**（升降在 SERVO2 时认领号应为 2）。
2. **Himark 协议**：舵机侧的 servo_id（认领号）= 数组下标 = SERVOx 序号 - 1。**舵机在出厂/配置时绑定认领下标后，飞控侧通道安排就必须迁就它**——比如舵机绑下标 1，升降就必须配在 SERVO2。反过来，若用标准协议，两边都可自由改，改完对齐认领号即可。
3. **每台舵机都要规划**：多舵机时列一张"SERVOx ↔ 认领号 ↔ 舵机物理位置 ↔ Node ID"四列对照表（即 `dronecan_alignment_tables.md` 表2 §2.2 ★行），刷参数和配置舵机各按表执行，避免错位。Node ID（1~125 唯一）与认领号（1~17）相互独立，前者标识节点、后者认领命令，不要混用。

### 6.4 自检顺序

```
① DroneCAN GUI 看到舵机节点上线（Node ID 就位）
② 拨 RC1 摇杆 → 抓包/GUI 看 ArrayCommand actuator_id=1 值随杆量线性变化
③ 舵机动作且方向正确（反了改 SERVO1_REVERSED）
④ 换 FUNCTION=4 接控制律，验证副翼权限与回中
```

## 7. 使用建议

1. **刷参数顺序**：先刷 wing_servo.param 调翼面舵机 → 需要联 BMS 时只改 3 项（SRV_BM=69631、SERVO17_FUNCTION=0、BATT_SERIAL_NUM=120）并重启，不必整份覆盖，避免把 BARO_GND_PRESS/陀螺偏置也带回去。
2. `CAN_D1_UC_SRV_RT=200` 意味着 ArrayCommand 以 200Hz 广播（含 17 号通道）；若后续同时挂倾转舵机（文件中 `CAN_D1_UC_TLT_*` 参数组已出现，TLT_MX=9000 即 90.00°，TLT_RT=20Hz），注意与 SRV_RT 叠加后的总线占比，可用 `docs/倾转RC1原始CAN.log` 的负载估算方法复核。
3. BATT_SERIAL_NUM=120 与 BATT2_MONITOR=0 说明当前是单高压电池配置；双电池时按手册启用 BATT2 槽位（BATT2_MONITOR=8 / BATT2_SERIAL_NUM=120）。
