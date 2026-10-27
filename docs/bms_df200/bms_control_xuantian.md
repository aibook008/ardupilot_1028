# BMS 控制外发（玄天项目）内容总结

> 日期：2026-10-10。来源：
> - `docs/BMS控制外发/BMS控制外发/操作手册.pdf`（400V BMS 控制系统——客户操作与交接手册，8 页）
> - `docs/BMS控制外发/BMS控制外发/missionplanner/BmsPowerControl.cs`（Mission Planner 插件 v0.13-servo17，621 行）
>
> 配套参考：`docs/bms_df200/`（pogo.lua / bms_control.lua / PCAN_1.dbc / pogo_bms_bridge.md，早期 DF-200 侧调试版本）、`docs/xuantian_param.md`（BMS.param/wing_servo.param 差异分析）、代码链路见 §7。

---

## 1. 系统是什么

400V 高压动力电池（REV7 BMS）的**上/下高压控制 + 电池遥测外发**，三段链路：

```
Mission Planner ──MAVLink──> 飞控(ArduPilot) ──DroneCAN 1 Mbit/s──> POGO 网关(Node 120)
                                                                        │
                                                                        └──原生 CAN 500 kbit/s──> BMS
```

- **控制方向**：MP 插件 → 飞控 SERVO17 单脉冲 → DroneCAN `actuator.ArrayCommand`(actuator_id=17) → POGO → BMS 原生 CAN（总压帧 0x18FF5001 / SOC·SOH 帧 0x18FF5002 / 命令确认帧 0x18FF5003）。
- **遥测方向**：BMS → POGO → DroneCAN `uavcan.equipment.power.BatteryInfo`(1092) 5 Hz → 飞控（BATT_MONITOR=8, BATT_SERIAL_NUM=120 绑定）→ MAVLink `BATTERY_STATUS` → MP 插件显示。

**关键边界（手册 §1）**：控制协议**无独立 ACK、无主副接触器辅助触点反馈**。"已发送指令"≠接触器动作成功；电压/电流/SOC 是观测数据不是到位确认；首次实机必须现场观察 BMS、接触器和母线状态。

## 2. 控制命令细节

| 项 | 上高压 | 下高压 |
|---|---|---|
| 插件动作 | `DO_REPEAT_SERVO` SERVO17=1900，1 次，0.6s 周期 | 同，SERVO17=1100 |
| 飞控侧 | SERVO17_FUNCTION=0（临时通道），MIN/TRIM/MAX 保持默认 1100/1500/1900 | |
| DroneCAN | 只发 UNITLESS（`CAN_D1_UC_OPTION` bit4 清零）；`CAN_D1_UC_SRV_BM=65536` 只选 SERVO17 | |
| POGO 侧 | 只匹配 actuator_id=17，其他通道静默忽略 | |
| CAN 帧 | POGO 在脉宽**上升沿**发一次命令 | 同 |

POGO 必须先观察到一次 1500 中立值再等待脉冲边沿（防上电误触发）。

## 3. 遥测数据口径（手册 §8）

- POGO 5 Hz 发 BatteryInfo；Node ID=120 = Battery ID=120（约定相同，全网唯一）。
- 电流方向按 ArduPilot 约定：**正=放电、负=充电**；温度 K→°C 由 ArduPilot/地面站转换。
- **双超时保护**：POGO 侧 0x18FF5001/5002 任一超 1s 未更新即**停止发布旧 BatteryInfo**；MP 插件侧 3s 未收到有效 BATTERY_STATUS 显示 `--`。
- 不发 BatteryInfoAux。

## 4. 插件实现要点（BmsPowerControl.cs）

- **2 Hz 主循环**（`LOOP_HZ=2.0`），挂 MP FlightData 页签"BMS 事件状态"（找不到就重试 6 次后转浮动窗口）。
- **上/下高压互锁**（`RequestLowVoltage`, BmsPowerControl.cs:268）：
  - 链路未开 → 阻止；
  - 心跳超时 3s（`HEARTBEAT_TIMEOUT_SEC`）或解锁状态未知 → 阻止；
  - **解锁中（HEARTBEAT base_mode & 0x80）→ 锁死下高压按钮**（安全联锁，UI 文案三态："状态未知/解锁中/可操作"）；
  - 加二次确认弹窗后才发 1100。
- **电池数据有效性过滤**（`OnPacketReceived`, BmsPowerControl.cs:157）：
  - BATTERY_STATUS 逐节电压累加（voltages + voltages_ext，跳过 0 与 0xFFFF）；
  - **总压 < 100V（`MIN_TRACTION_VOLTAGE_V`）视为非高压动力电池直接丢弃**——这就是手册说的"不按电池1/2固定取值，双电池配置也能用"；
  - charge_state == UNHEALTHY/FAILED 置故障标志，限频记日志（遥测继续显示）。
- **主动拉流**：心跳有效后每 5s 尝试 `SET_MESSAGE_INTERVAL` 请求 BATTERY_STATUS(147) @ 200000µs（5 Hz），成功一次即不再请求。
- 事件日志按内容着色（已发送/阻止/超时/恢复）。

## 5. 飞控参数速查（手册 §4）

| 参数 | 值 | 说明 |
|---|---|---|
| CAN_Px_DRIVER / BITRATE | 1 / 1000000 | 对接 POGO CAN1 |
| CAN_D1_PROTOCOL | 1 | DroneCAN |
| CAN_D1_UC_NODE | 10 | 飞控 NodeID |
| CAN_D1_UC_SRV_BM | 65536 | 只启用 SERVO17（bit16） |
| CAN_D1_UC_OPTION | bit4 清零 | 必须 UNITLESS 而非 PWM |
| SERVO17_FUNCTION | 0 | 插件临时控制 |
| BATT_MONITOR / BATT_SERIAL_NUM | 8 / 120 | 电池监视器=DroneCAN，绑定 ID 120 |

注意事项：硬件 Safety 会挡 DroneCAN 舵机输出（BRD_SAFETY_MASK 的 SERVO17=bit16）；上高压命令不保证唤醒 BMS（KL15/A+ 硬线唤醒条件必须满足）；POGO 供电必须接接触器前级低压常电，否则下高压后无法远程再上电；首次测试不接 400V 推进负载；CAN0/CAN1 千万不能并同一条总线（500k vs 1M）。

## 6. 常见问题对照（手册 §9 摘录）

| 现象 | 首要检查 |
|---|---|
| Node 120 看不到 | CAN1 线序/1 Mbit/终端电阻/脚本未真正加载 |
| DroneCAN GUI 有 BatteryInfo 但飞控无电池 | BATT*_MONITOR=8、BATT*_SERIAL_NUM=120，改后重启 |
| MP 显示已发送但 POGO 无命令 | UC_SRV_BM、SERVO17、UNITLESS、Safety、Node ID |
| 下高压按钮灰色 | 飞控解锁中或心跳未知——属正常安全联锁 |

## 7. 代码实现链路（ArduPilot 侧，2026-10-10 源码走读）

> 行号基于本仓库 `qhz/plane_4.6.3` 分支。

### 7.1 `CAN_D1_UC_SRV_BM` 怎么用

定义在 `libraries/AP_DroneCAN/AP_DroneCAN.cpp:111`（`_servo_bm`），两处消费：

**① `SRV_push_servos()`（AP_DroneCAN.cpp:923-957）**——主循环调度，决定"哪些 PWM 通道允许上 CAN"：

```cpp
// :929 只有 SERVOx_FUNCTION >= k_none（0 或有效功能）的通道才采样
if (SRV_Channels::channel_function(i) >= SRV_Channel::k_none) {
    _SRV_conf[i].pulse = SRV_Channels::srv_channel(i)->get_output_pwm();
    _SRV_conf[i].servo_pending = true;
    if (_SRV_conf[i].pulse > 0) non_zero_channels |= 1U << i;
}
uint32_t servo_armed_mask = _servo_bm & non_zero_channels;   // :942 位图在此生效
// :945-950 硬件 Safety 拔了时再 & BRD_SAFETY_MASK
_SRV_armed_mask = servo_armed_mask;
```

**② `SRV_send_actuator()`（AP_DroneCAN.cpp:734-788）**——按 `CAN_D1_UC_SRV_RT`（本项目 200Hz，`AP_DroneCAN.cpp:552` 取周期）把允许通道打包广播 `uavcan.equipment.actuator.ArrayCommand`：

```cpp
if (_SRV_conf[i].servo_pending && ((1U << i) & _SRV_armed_mask)) {   // :759
    cmd.actuator_id = starting_servo + 1;        // :760 SERVO17 → actuator_id=17（+1 偏移）
    if (option_is_set(Options::USE_ACTUATOR_PWM)) { ... } // bit4 未置位，走下面 UNITLESS 支路
    cmd.command_value = constrain_float((pulse - 1000.0)/500.0 - 1.0, -1.0, 1.0); // :767
}
```

所以 `SRV_BM=0x10FFF`：bit0~15 允许 SERVO1~16（翼面舵机）、bit16 允许 SERVO17（BMS）；不在位图内的通道即使有 PWM 也不上 CAN。发送入口的分协议选择见 `AP_DroneCAN.cpp:556-562`（`USE_HIMARK_SERVO` bit6=1 走 Himark 私有协议，否则走标准 ArrayCommand）。

### 7.2 "插件 → SERVO17 单脉冲 → ArrayCommand(actuator_id=17)" 全链路

1. **MP 插件**：`DO_REPEAT_SERVO, 17, 1900, 1次, 0.6s`（BmsPowerControl.cs:215）→ MAVLink → 飞控把 **SERVO17 输出通道打一个单脉冲**（上高压 1900µs / 下高压 1100µs）。这就是 `SERVO17_FUNCTION=0` 的原因：通道从飞控功能中解放，只受 MAVLink 命令驱动；MIN/TRIM/MAX=1100/1500/1900 保持默认，1900 恰为 MAX。
2. **飞控 `SRV_push_servos`**：下一循环采样 SERVO17=1900µs，`FUNCTION=0 ≥ k_none` 通过、在 SRV_BM bit16 内 → 进 `_SRV_armed_mask`。
3. **`SRV_send_actuator` @200Hz**：`actuator_id=17`（16+1），`command_value = (1900-1000)/500-1 = +0.8`（下高压 1100µs → **-0.8**），UNITLESS，广播 ArrayCommand。
4. **POGO**：只认领 `actuator_id=17`，且在脉宽**上升沿**（1500→1900 跳变）时向 BMS 原生 CAN 发一次命令——对应手册"POGO 上电必须先观察到一次中立值"的要求。

### 7.3 BatteryInfo 接收：`BATT_MONITOR=8` / `BATT_SERIAL_NUM=120` 的作用

链路在 `libraries/AP_BattMonitor/AP_BattMonitor_DroneCAN.cpp`：

1. **订阅分发**：`AP_DroneCAN` 收到 `uavcan.equipment.power.BatteryInfo`(1092) → `handle_battery_info_trampoline`（:223），取 **`transfer.source_node_id`（POGO Node=120）和 `msg.battery_id`（POGO 填 120）**。
2. **`get_dronecan_backend()`（:74-107）——两个参数在此生效**：

   ```cpp
   // :68 注释：serial number 为负时接受所有电池，否则必须匹配
   bool match_battery_id(uint8_t instance, uint8_t battery_id) {   // :68-72
       return AP::battery().get_serial_number(instance) < 0
              || AP::battery().get_serial_number(instance) == (int32_t)battery_id;
   }
   ```

   遍历电池实例找：`type == UAVCAN_BatteryInfo`（即 **BATT_MONITOR=8**）、driver 挂在本 AP_DroneCAN 实例、来源 NodeID 一致、且 **`BATT_SERIAL_NUM == msg.battery_id`**；找不到匹配的空闲实例就分配一个。即 MONITOR=8 = "该电池槽位用 DroneCAN 后端"，SERIAL_NUM=120 = "只认 Battery ID=120 的电池"（多电池/多网关防串数据）。
3. **数据落地** `handle_battery_info`（:115-128）→ `update_interim_state`（:130-163）：电压/电流直接存，**温度 K→°C（:140）**，SOC 存 `_soc`，SOH 非未知才消费（:124），置 `healthy=true`；无 BatteryInfoAux 时用库内电流积分算 consumed（:146-152）。POGO 不发 Aux，所以单体电压/循环次数缺失。
4. **出口**：`AP_BattMonitor` 汇编进 MAVLink `BATTERY_STATUS` 发 GCS；MP 插件另一头过滤总压 ≥100V 认领显示（见 §4）。

**wing_servo.param 里 `BATT_SERIAL_NUM=-1` 的机制**：match_battery_id 中 `serial_num < 0 → 全部接受`，台架上任何 BatteryInfo 都被认作电池 1；BMS 场景设 120 即收紧为只认 POGO。手册 §4.2"调试绑定时可临时设 -1"正是此机制。


容易误会的地方在于 GCS_MAVLink 这个库的名字——它不是“地面站的代码”，而是跑在飞控上的 MAVLink 协议栈（“GCS_MAVLink”意为“与 GCS 通信用的 MAVLink 库”，代码位于 libraries/GCS_MAVLink/，编进 arduplane
  固件）。地面站（Mission Planner）侧是纯 C# 插件，只做了一件事：

  MP 插件（地面站）：doCommand(DO_REPEAT_SERVO, 17, 1900, 1, 0.6)
       │
       │  串口/数传发一条 MAVLink COMMAND_INT / COMMAND_LONG 消息
       ▼
  飞控收到 → GCS_MAVLink::handle_command_int()（飞控上运行）

  之后飞控上按顺序跑两段代码：

  第一段：命令处理（一次性，收到消息时执行）
  handle_servorelay_message（GCS_ServoRelay.cpp:7）→ do_repeat_servo（AP_ServoRelayEvents.cpp:96）——通道资格审查、设定 repeat=2 / 
  delay_ms=300、立即打第一拍。这是飞控上的代码在飞控上执行，只是触发时机是“收到地面站消息”这个事件。

  第二段：状态机推进（周期性，飞控调度表驱动） 
  update_events() 挂在 飞控的 AP_Scheduler 任务表里（ArduPlane.cpp:81）：

  SCHED_TASK_CLASS(AP_ServoRelayEvents, ..., update_events, 50, 150, 63)
  //              ↑ 这个类实例 live 在飞控内存里      ↑ 50Hz，飞控主循环每秒调50次

  do_repeat_servo 只是填好了状态机的初始状态（channel=17, servo_value=1900, repeat=2, delay_ms=300），之后的翻转节奏完全由飞控自己的 50Hz 任务按毫秒时钟推进：

  t=0     do_repeat_servo 执行 → 立即第一拍 set_output_pwm(1900)
  t=300ms update_events(50Hz任务) 发现半拍到 → set_output_pwm(1500)
  t=600ms repeat 减到 0 → 结束

  为什么必须这样设计

  注意时序依赖：命令消息发完就没了，地面站不再参与。如果由地面站来控制脉冲时序，就要通过数传以 300ms 间隔连发多条 DO_SET_SERVO——链路延迟抖动（几百 ms
  很常见）会把脉冲波形打乱，断链则脉冲直接废掉。把状态机放在飞控上：命令到达后 0.6s 的时序由飞控本地时钟保证，期间地面站掉线都无所谓，上升沿照样产生、POGO 照样触发 BMS。这也是 MAVLink 的设计惯例——DO_*
  命令只传“意图+参数”，执行节拍由飞控侧状态机负责。   

## 8. 与 DF-200 旧版（docs/bms_df200/）的关系

旧目录是 9 月 DF-200 侧的调试资产：pogo.lua 及其 probe/step1/debug 变体、PCAN_1.dbc、pogo_bench_test_plan.md。本次外发包（BMS控制外发/）是**客户交接版**：Lua 收敛为单一 bms_control.lua（POGO 固件按 pogo.lua 命名加载）、MP 插件从早期版本演进到 v0.13（SERVO17 单脉冲 + 100V 高压过滤 + 解锁联锁 + 主动拉流），并新增 bms_sim.py 仿真器。调试方法与坑位记录可回查 `docs/bms_df200/pogo_bms_bridge.md`。
