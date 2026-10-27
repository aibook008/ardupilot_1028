# 失效分析与 ArduPilot 当前实现对照

> 创建：2026-10-13 / 更新：2026-10-16（§3.5.1 补充电机失效四层机制、DF-200 生效性判定、SITL 注入验证结果）
> 适用分支：`qhz/plane_4.6.3`（Plane 4.6.x，QuadPlane 倾转旋翼构型 DF-200/JT-20）
> 关联文档：[故障安全代码通读](../code_walkthrough_failsafe.md) | [技术协议风险分析](../technical_agreement_risk_analysis.md) §2.6/§3 | [多余度可行性调研](../redundancy_feasibility.md) | [电机失效测试规程](motor_failure_test_procedure.md)
> 所有 文件:行号 均已用工具核实。

---

## 1. 失效分析范围与方法

本文按 **失效源 → 检测机制 → 当前代码实现 → 检测链路 → 覆盖评价/缺口** 的结构，对倾转旋翼验证机可能遭遇的失效模式逐项分析，并与当前 ArduPlane 代码实现对照。

失效源分类：
1. 链路失效（RC、数传/GCS）
2. 能源失效（电池、供电）
3. 传感器失效（GNSS、IMU/EKF、空速、罗盘）
4. 执行机构失效（电机、舵面、倾转舵机）
5. 飞控本体失效（主循环卡死、崩溃、软件异常）
6. 环境失效（电子围栏、ADSB 冲突、失速、坠机）

---

## 2. 失效检测架构总览

### 2.1 任务调度表中的失效检查项（`ArduPlane/ArduPlane.cpp:62-143`）

| 任务 | 频率 | 优先级 | 行号 | 检测对象 |
|------|------|--------|------|---------|
| `check_short_failsafe` | 50Hz | 9 | :63 | RC 短失效 |
| `afs_fs_check` | 10Hz | 51 | :75 | 高级故障安全（AFS） |
| `ekf_check` | 10Hz | 54 | :77 | EKF 方差 |
| `battery.read` | 10Hz | 66 | :83 | 电池（含失效检查） |
| `three_hz_loop` | 3Hz | 93 | :94 | 长失效辅助 |
| `check_long_failsafe` | 3Hz | 96 | :95 | RC/GCS 长失效 |
| `parachute_check` | 10Hz | 129 | :120 | 降落伞沉率 |
| `update_is_flying_5Hz` | 5Hz | 135 | :124 | 坠机检测基础 |

另有一个**独立于主循环**的硬失效兜底：`failsafe_check()`（`ArduPlane/failsafe.cpp:17`）由定时器中断以 1kHz 调用，主循环 0.2s 无 tick 时直接把 RC 输入旁路到输出（:33-39 触发，:77-104 旁路输出），保证即使调度器卡死舵面仍有可控直通。

### 2.2 失效状态机（`ArduPlane/Plane.h`）

`failsafe.state` 取值：`FAILSAFE_NONE / FAILSAFE_SHORT / FAILSAFE_LONG / FAILSAFE_GCS`（`Plane.h:408` 引用）。短失效经 `fs_timeout_long`（`FS_LONG_TIMEOUT`，默认 5s，`Parameters.cpp:477`）升级为长失效；短失效超时 `FS_SHORT_TIMEOUT`（默认 1.5s，`Parameters.cpp:461`）。

---

## 3. 分项失效分析

### 3.1 RC 链路失效 ✅ 完整

**检测链路**：

```
RC 接收机失效/PWM 超时 → RC_Channels rc_failsafe_active()
  → radio.cpp:296-303 连续 10 次计数 → failsafe.rc_failsafe = true（radio.cpp:299）
  → check_short_failsafe（50Hz, system.cpp:400）→ failsafe_short_on_event（events.cpp:21）
  → 超过 FS_LONG_TIMEOUT → check_long_failsafe（3Hz, system.cpp:353）
  → failsafe_long_on_event（events.cpp:108）
```

**短失效动作**（`events.cpp:21-105`，按模式分支）：
- 固定翼手动/增稳类 → FBWA/FBWB/CIRCLE（:28-47）
- VTOL 模式 → QRTL/QLAND（:49-64，受 `Q_OPTIONS` bit5/20 影响）
- AUTO/GUIDED/LOITER → 依 `FS_SHORT_ACTN` 配置（:67-88）

**长失效动作**（`events.cpp:108-215`）：手动类模式可配置降落伞/滑翔/AUTO/RTL（:116-150），VTOL 模式 RTL/QRTL/QLAND（:152-168），AUTO 类降落伞/滑翔/AUTO/RTL（:170-199）。

**恢复**：RC 恢复后 `failsafe_short_off_event` / `failsafe_long_off_event`（`events.cpp:218/230`）恢复原模式。

**缺口**：检测依赖油门通道低于阈值（throttle failsafe），无单独的信号质量/跳包统计；串级接收机（CRSF/ELRS）的心跳级失效依赖 RC 库层，覆盖尚可。

### 3.2 GCS 数传失效 ✅ 完整

**实现**：`check_long_failsafe()`（`system.cpp:366-392`），由 `FS_GCS_ENABL` 四种模式控制：
- `1`：心跳丢失触发
- `2`：心跳丢失 **或** RADIO_STATUS.remrssi=0（单向链路检测）
- `3`：仅 AUTO 模式下心跳丢失触发（:368-371）
- 超时统一用 `fs_timeout_long`（:370-381）

**动作**：走 `failsafe_long_on_event(FAILSAFE_GCS, ...)`，与 RC 长失效同一处理路径。

### 3.3 电池/能源失效 ✅ 完整

**检测**：`AP_BattMonitor::check_failsafes()`（`libraries/AP_BattMonitor/AP_BattMonitor.cpp:851`）仅 soft_armed 时检查（:853），逐实例调用 `update_failsafes()`。低电压/低电量/临界容量多级阈值。

**动作**：回调 `Plane::handle_battery_failsafe`（`events.cpp:243`），动作枚举 `Failsafe_Action`（`Plane.h:1240-1250`）：None/RTL/Land/Terminate/QLand/Parachute/Loiter_alt_QLand。多电池优先级由 `_failsafe_priorities`（`Plane.h:1256-1257`）控制。

**缺口（DF-200 相关）**：BMS 相关协议监控（见 `../bms_control_xuantian.md`）与标准电池 failsafe 是两条独立链路；RPM/转速监控（`AP_RPM`，调度表 :97）只记录不触发失效动作。

### 3.4 传感器失效

#### 3.4.1 EKF/IMU 失效 ✅ 基本完整

**检测**：`Plane::ekf_check()`（`ekf_check.cpp:30`）10Hz。仅解锁后且 `FS_EKF_THRESH > 0` 时使能（:42-45）。`ekf_over_threshold()` 检查速度/位置/高度/罗盘方差，连续 `EKF_CHECK_ITERATIONS_MAX` 次超限置 `bad_variance`（:57-88）。

**动作**：触发后 EKF failsafe，动作由模式与配置决定（QLAND/RTL/降级）。底层 IMU 冗余由 `AP_InertialSensor` 多实例 + `AP_NavEKF3` 多 lane 切换承担（见 [code_walkthrough_ekf.md](code_walkthrough_ekf.md)）。

#### 3.4.2 GNSS 失效 ⚠️

无独立 `gps_failsafe()` 函数（grep 全仓库无此符号，旧文档表述不准）。实际覆盖路径：
- GNSS 丢失 → EKF 位置方差增大 → `ekf_check` 间接触发（间接、有延迟）
- `Q_OPTIONS`/VFR HUD 提示，AUTO 模式依赖 EKF fallback lane（非 GPS lane 或惯性-only 航位推算）
- **缺口**：无 GNSS 消息级（如 HDOP/卫星数骤降）的主动快速失效检测。

#### 3.4.3 空速传感器失效 ⚠️

无独立 `airspeed_failsafe()`。空速失效时 TECS 自动退回使用空速比例估计（`AIRSPEED_MAX`/`MIN` 与地速融合），但**没有显式失效告警→模式降级链路**。对倾转机构（转换条件依赖空速 `AIRSPEED_WAIT`，见 `quadplane.cpp:1466` 起的 SLT 状态机）影响较大：空速虚高会导致提前结束转换、多旋翼升力不足。

#### 3.4.4 罗盘失效 ✅（经由 EKF）+ 航向一致性检查

罗盘方差由 `ekf_over_threshold()` 覆盖；掉电/冻结由 EKF lane 打分切换兜底。

### 3.5 执行机构失效

#### 3.5.1 单电机失效 ⚠️ 机制已内置，Plane 侧缺触发钩子（已深入核实 + SITL 注入验证）

##### A. ArduPilot 电机失效处理机制全景

APM 对电机失效的处理分**四层**，从底到顶：

**先厘清两层的关系（常见误解澄清）**

第 1 层和第 2 层**不是串联依赖关系**，而是"执行器 + 开关"的关系：

```
第 1 层（AP_MotorsMatrix 混控器内部）:
  检测 check_for_failed_motor ──► _motor_lost_index（失效电机号）
                                        │
  重构分配逻辑（thrust boost）◄──────────┘
       ▲
       │ 受单一布尔开关 _thrust_boost 控制
       │
第 2 层（Copter 的触发任务）:
  thrust_loss_check() ──► set_thrust_boost(true)   ← 唯一的开关扳手
```

- 第 1 层的**检测部分**（`check_for_failed_motor`）每个混控周期自动运行，与开关无关——它在 DF-200 上一直在跑，持续计算"谁失效了"并存入 `_motor_lost_index`
- 第 1 层的**重构分配部分**（thrust boost）受 `_thrust_boost` 开关控制，开关关着等于这部分代码不存在
- 第 2 层**不检测也不补偿**，它只观察整机状态（油门饱和+持续下沉），然后把开关扳到 true
- 因此缺了第 2 层不会"断链"导致飞不了——混控器主分配路径（油门+RPY→8 路输出）不受开关影响，只是重构分配这一增量能力不可用

**"8 路输出 vs 7 个控制自由度"——为什么需要重构**

混控器物理上始终输出 8 路 PWM，但分配的是力矩需求：`Motor_i = 油门 + roll×R_i + pitch×P_i + yaw×Y_i`。一个电机停转后：
- 分配公式**没变**——还在给失效电机算输出值并发出 PWM，但被物理"吞掉"
- 失效电机的升力贡献（-1/8）和差动力矩贡献同时消失，**分配矩阵名义 8 列、实际有效 7 列**
- 姿态控制器只能看到"同样的 RPY 需求产生了打折的力矩"，靠积分项被动顶误差

thrust boost 激活后做的事是**把分配矩阵显式降维到 7 列**：跳过失效电机列（:313）、把它那份推力/力矩按剩余电机 yaw 裕度重新求最优解（:339-352）——这就是**控制律重构（control reallocation）的本体**：执行器集合变化后重新求解"控制输入→执行器输出"映射，并处理 yaw 裕度与饱和约束。

**"单电机失效控制律重构"工作的真实构成**

| 组成部分 | 状态 |
|---------|------|
| 重构算法本体（混控级再分配，第 1 层） | ✅ 已存在，DF-200 编译包含 |
| 触发开关（第 2 层 `thrust_loss_check`） | ❌ Plane 缺，需移植 |
| 失效后任务级策略（告警/降级/QLAND 建议） | ❌ 需新写 |
| DF-200 定制增强（第 4 层 Dynamic 矩阵） | 可选：考虑内外桨差异/倾转角度的更贴身重构 |

即：**开发量不在重构算法（现成），而在"接上开关 + 定义扳下开关后的动作链"**。

**为什么 SITL 测试缺第 2 层还能跑通**

测试只验证"失效后飞机还能不能飞"：电机停转 → 姿态/高度误差 → 常规闭环（姿态环+高度环）自动加大剩余电机输出 → 误差被顶回。这是 APM 常规控制律的自然反应，**不需要任何失效专用机制**——8 桨单桨失效、悬停小油门工况下裕度足够（见 C 节数据）。重构机制（第 1+2 层）的价值在边界工况：同侧双桨、大桨（60in）、有风、前飞中失效——此时硬顶会导致某些电机饱和（rpy_scale 裁剪，姿态先失守），混控级最优重分配才能维持控制权限。

**第 1 层：混控级失效检测与推力重分配（核心，自动生效）**

位于 `AP_MotorsMatrix`（所有矩阵构型混控的基类），QuadPlane 构建自动包含：

1. **检测**：`check_for_failed_motor()`（`libraries/AP_Motors/AP_MotorsMatrix.cpp:479`），在每次 `output_armed_stabilizing()` 末尾被调用（:468）。
   - 对各电机输出做 0.5s 低通滤波（`_thrust_rpyt_out_filt[]`，:484-490）
   - 计算推力平衡度 `thrust_balance = 最高滤波输出 × 电机数 / 输出总和`（:507-509）
   - **≥6 电机才启用**（:512）——DF-200 是 8 电机，满足门槛
   - `thrust_balance ≥ 1.5` 持续 → 判定推力最高的电机失效，记录 `_motor_lost_index`（:496-501，boost 激活期间索引锁定）
   - 恢复平衡（≤1.25）自动清除（:514-516）——**天然支持失效电机恢复**（如短暂停转后重启）
2. **补偿（thrust boost）**：`_thrust_boost=true` 时混控器在 yaw 约束下做最优推力重分配：
   - 主分配段跳过失效电机（:313）、把失效电机的推力需求按剩余电机 yaw 裕度重分配（:339-352）
   - rpy_scale 缩放段同样排除失效电机（:375-384）
   - 经 `_thrust_boost_ratio` 在 0→1 平滑过渡（:232-237），避免补偿突投
   - 自愈：`_throttle_thrust_max` 有余量且 `rpyt_high<0.9` 时自动关闭（:519-522）
3. **注意**：检测是"输出指令级"（看混控器要求哪个电机出力最大），**不是传感器级**（不看实际转速/电流）。若失效电机本来就指令低（如巡航时升力桨怠速），检测不会触发——这也是为什么需要第 2 层的状态触发器。

**第 2 层：飞控状态级触发钩子（Copter 有，Plane 缺）**

`Copter::thrust_loss_check()`（`ArduCopter/crash_check.cpp:100`），10Hz：
- 判据链：姿态目标 <15°（:125-130）→ 油门 ≥90% 或输出饱和（:132-137）→ 油门 >25%（:139-142）→ 持续下沉 vz<0（:144-148）→ 姿态角误差 <30°（:152-158）→ 持续 1s（`THRUST_LOSS_CHECK_TRIGGER_SEC`）
- 动作：记 ERROR 日志（`THRUST_LOSS_CHECK/FAILSAFE_OCCURRED`）+ GCS EMERGENCY 消息 + `motors->set_thrust_boost(true)`（:170）+ 可选释放挂载（`FlightOption::RELEASE_GRIPPER_ON_THRUST_LOSS`）
- boost 开启后由第 1 层混控器在油门恢复余量时自动关闭

**Plane/QuadPlane 现状**：调度表（`ArduPlane.cpp:62-143`）无此任务，全 Plane 代码无 `set_thrust_boost` 调用方。**这是"单电机失效控制律重构"的真实缺口**。

**第 3 层：Lua 脚本接口（Plane 构建可用）**

`bindings.desc:571-572`：`MotorsMatrix:get_lost_motor()` / `get_thrust_boost()`，depends 含 `APM_BUILD_ArduPlane`。官方 applet 与示例：
- `libraries/AP_Scripting/applets/motor_failure_test.lua`：**失效注入**——RC AUX 开关（option 300）+ `MOT_STOP_BITMASK` 选电机，`set_output_pwm_chan_timeout(..., 15ms)` 停转（超时自恢复）
- `examples/motor_lost.lua`：失效号上报

**第 4 层：混控矩阵架构支持（DF-200 特有优势）**

DF-200 用 `AP_MotorsMatrix_Scripting_Dynamic`（Q_FRAME_CLASS=17），Lua 可在运行时改混控矩阵（`load_factors()`，带信号量线程安全）——这为"失效后重新分配控制权"提供了框架级支持：检测到某电机失效后，Lua 可以把该电机的因子清零、并把其控制贡献重新归一化到剩余电机，实现比 thrust boost 更精细的定制策略。

##### B. DF-200 构型逐项生效性判定

| 机制 | DF-200 是否生效 | 原因 |
|------|----------------|------|
| 第 1 层检测 `check_for_failed_motor` | ✅ 运行中 | 8 电机 ≥6 门槛；Scripting_Dynamic 继承基类（`Scripting_Dynamic.cpp:125` 调基类 `output_to_motors()`），每混控周期执行。但 `_thrust_boost=false`，检测结果只存 `_motor_lost_index` 无人消费 |
| 第 1 层补偿 thrust boost | ❌ 永不激活 | 无触发方（第 2 层缺失），`_thrust_boost` 恒 false |
| 第 2 层触发钩子 | ❌ 不存在 | Plane 无 `thrust_loss_check`，需移植 |
| 第 3 层 Lua 查询 | ✅ 可用 | 绑定 depends 含 ArduPlane |
| 第 4 层 Dynamic 矩阵 | ✅ 可用 | DF-200 架构天然支持运行时重构分配 |
| **姿态控制器硬顶（当前实际行为）** | ✅ 生效 | 电机失效后姿态环/RPY 通道对剩余电机差动加力，无需 boost 也能顶住（8 桨裕度大）——已由 SITL 注入验证（见 C 节） |

**相关但不同的机制**（勿混淆）：
- `ArduPlane/failsafe.cpp:17` `failsafe_check()`：仅主循环卡死的 RC 直通，与电机失效无关。
- `quadplane.cpp:2585-2606` 前向电机推力损失检测（QPOS 进近段，油门饱和+下沉率+低空速 5s → 降级 QPOS_POSITION1）：只针对前向推进电机，且外层 `!tiltrotor.enabled()`（:2587）排除倾转构型——**DF-200 不生效**。
- `AC_AttitudeControl_Multi::get_throttle_boosted()`（`AC_AttitudeControl_Multi.cpp:374`）：姿态角余弦补偿，与电机失效无关（`code_walkthrough_servo.md:167` 注释有误）。

##### C. SITL 注入验证结果（log 00000030.BIN，2026-10-16）

注入脚本 `scripts/motor_failure_test_param.lua` v4（参数触发，`SCR_USER1` 1=注入/0=恢复，`MOT_STOP_BITMASK` 选电机），完整规程与历史排查见 [motor_failure_test_procedure.md](motor_failure_test_procedure.md)。

**工况**：GUIDED 起飞爬至 10m 悬停 → 注入停 Motor2（FL tilt 40in，C6 输出通道）→ 悬停观察 18s → QLAND。

**结果**：
- 停转生效：C6 从 1684 → 精确 1000 钉死整个注入窗口
- 差动补偿可见：对角/对侧电机抬升（C8 +60、C10 +191、C9 +39），姿态控制器硬顶
- 姿态最大偏差 Roll 0.19° / Pitch 0.29°（与注入前基线 0.22°/0.19° 同量级——SITL 无风+8 桨裕度下几乎无感）
- 最大下沉率 0.31 m/s，掉高 0.48m 后 ~2s 定高拉回
- 无任何 EKF/姿态 failsafe

**结论**：当前代码（无 thrust boost）下 8 桨停 1 桨已可稳住——协议 §5.1.5 可飞性在悬停工况通过。**何时需要补第 2 层**：边界工况（停同侧双桨、外桨 60in、有风、前飞中）姿态裕度不足时，thrust boost 的混控级重分配才有增量价值；单桨悬停工况现有行为已足够。

##### D. 对 DF-200 的落地建议（更新 §5 P1 项的实施路径）

1. **短期（现有一行代码级别的增强）**：把 df200_monitor.lua 挂上（已写好 `scripts/df200_monitor.lua`），失效时 GCS 上报——但注意 `_thrust_boost` 恒 false 时它恒报 MotorOK，需改为读 `_motor_lost_index` 的等效路径或等待第 2 层移植。
2. **中期（P1 主体，0.5-1 人月）**：
   - 移植 `thrust_loss_check()` 到 Plane：新增调度任务（10Hz），判据适配（多旋翼模式下姿态目标/油门/下沉链路不变；FW/转换模式需门控不判）
   - 失效后动作策略：GCS 告警 + 日志 → 按 `MOT_` 失效号决定是否 Q_ASSIST 增强/禁止转换/建议 QLAND
   - 验证 Dynamic 矩阵 `load_factors()` 与 thrust boost 交互：Lua 每周期覆盖因子表，boost 的重分配发生在因子表之后（混控器内部），理论无冲突，但需 SITL 注入+boost 开启联合验证
3. **注入-检测闭环验证方法**：用第 3 层 Lua `get_lost_motor()` 轮询，注入后应能读到失效号（当前因 boost 未激活读不到——移植第 2 层后此路径即通）

#### 3.5.2 倾转舵机失效 ❌（本项目新增执行机构）

当前倾转舵机走自定义 UAVCAN 协议（见 `../tilt_uav_can.md`），失效模式：CAN 断链、多帧 CRC 错、位置反馈异常。目前**无舵机级 failsafe**（无反馈超时→冻结/回中→转换中止链路）。

#### 3.5.3 舵面失效 ⚠️

无舵面级故障检测。底层覆盖：DroneCAN/PWM 输出链路健康依赖 CAN 心跳；SRV_Channels 无"输出指令 vs 实际位置"比对。

### 3.6 飞控本体失效

| 失效 | 机制 | 代码位置 | 覆盖 |
|------|------|---------|------|
| 主循环卡死 | 定时器中断 RC 直通 | `failsafe.cpp:17-115` | ✅ |
| 看门狗复位 | IWDG + WDT | `AP_Scheduler` / AP_HAL_ChibiOS | ✅ |
| 崩溃转储 | crash dump 钩子 | `AP_Vehicle` | ✅ |
| 输出失效（IOMCU 挂） | 主 MCU 直接输出 fallback | AP_HAL_ChibiOS | ✅ |

### 3.7 环境失效

| 失效 | 机制 | 代码位置 | 覆盖 |
|------|------|---------|------|
| 电子围栏突破 | `fence_check()` → GUIDED/RTL/LAND 动作 | `ArduPlane/fence.cpp:11`，恢复逻辑 :32-60 | ✅ |
| ADSB 冲突 | `avoidance_adsb_update()` → AVOID 模式 | `ArduPlane/avoidance_adsb.cpp:6` | ✅ |
| 失速 | **无独立失速保护**（协议 §5.1.3.7 要求，缺口） | — | ❌ |
| 高度异常 | **无独立高度 failsafe**（仅 fence 的限高） | — | ❌ |
| 坠机检测 | `crash_detection_update()`，仅 AUTO 模式 | `is_flying.cpp:210` | ⚠️ 范围有限 |

### 3.8 转换过程失效（倾转旋翼特有）⚠️

这是本项目最重要的定制缺口（协议 §5.1.3.4 明确要求"转换中断判断"）：

| 失效场景 | 当前实现 | 代码位置 |
|---------|---------|---------|
| 正向转换超时 | `Q_TRANS_FAIL` 超时 → 按 `Q_TRANS_FAIL_ACT` 切模式（或 Q_OPTIONS bit19 强制完成转换） | `quadplane.cpp:1549-1562`（参数定义 :354/:462） |
| 起飞失败 | `Q_TKOFF_FAIL_SCL` 标量超时 → QLAND | `quadplane.cpp` 参数 :399 |
| 空速不足主动中止 | **无** TRANSITION_ABORT 状态 | — |
| 反向转换异常 | 依赖固定翼→多旋翼路径（见 `../vtol_reverse_transition.md`，无失效注入测试） | — |

---

## 4. 覆盖度汇总表

| # | 失效源 | 检测 | 快速性 | 动作完整性 | 评价 | 主要缺口 |
|---|--------|------|--------|-----------|------|---------|
| 1 | RC 失效 | ✅ 10 次×50Hz 计数 | 快 | ✅ | 良好 | — |
| 2 | GCS 失效 | ✅ 心跳+RSSI | 中（3Hz 检查） | ✅ | 良好 | — |
| 3 | 电池低电量 | ✅ 多级阈值 | 快 | ✅ | 良好 | RPM/功率不触发动作 |
| 4 | EKF/IMU 异常 | ✅ 方差+多lane | 中 | ✅ | 良好 | — |
| 5 | GNSS 失效 | ⚠️ 仅经 EKF 间接 | 慢 | ⚠️ | 部分 | 无消息级主动检测 |
| 6 | 空速失效 | ⚠️ 无显式链路 | — | ❌ | 薄弱 | 转换条件被污染 |
| 7 | 单电机失效 | ⚠️ 检测+混控补偿内置（AP_MotorsMatrix 四层机制，见 §3.5.1），Plane 缺触发任务 | 中（依赖滤波+平衡度） | ⚠️ 姿态控制器硬顶有效（8桨停1桨已验证），无失效后任务级动作 | 部分缺失，悬停工况可用 | 移植 thrust_loss_check + 动作策略（§3.5.1.D） |
| 8 | 倾转舵机失效 | ❌ | — | ❌ | 缺失 | UAVCAN 反馈超时→转换中止 |
| 9 | 主循环卡死 | ✅ 1kHz 中断兜底 | 快 | ✅ | 良好 | — |
| 10 | 围栏/ADSB | ✅ | 快 | ✅ | 良好 | — |
| 11 | 失速 | ❌ | — | ❌ | 缺失 | 需新增 stall_failsafe |
| 12 | 高度异常 | ❌ | — | ❌ | 缺失 | 需新增 altitude_failsafe |
| 13 | 转换中止 | ⚠️ 仅超时降级 | 慢 | ⚠️ | 部分 | 主动回退逻辑（R2） |
| 14 | 坠机 | ⚠️ 仅 AUTO 模式 | 中 | ⚠️ | 部分 | 扩展模式范围 |

---

## 5. 缺口优先级与建议行动

结合 [technical_agreement_risk_analysis.md](technical_agreement_risk_analysis.md) §3 风险矩阵，针对失效分析的缺口排序：

| 优先级 | 缺口 | 建议方案 | 预估工作量 |
|--------|------|---------|-----------|
| P0 | 倾转舵机失效（#8） | UAVCAN 反馈超时（如 200ms）→ 冻结当前角度 + 触发 `transition_failure` 路径 | 0.5 人月 |
| P0 | 转换主动中止（#13） | SLT 状态机新增 TRANSITION_ABORT：空速持续低于阈值/姿态发散时回退多旋翼 | 0.5-1 人月（同 R2） |
| P1 | 单电机失效（#7） | 机制四层已核实（§3.5.1.A），悬停单桨注入已验证可顶住（§3.5.1.C）。实施路径（§3.5.1.D）：① 移植 `thrust_loss_check()` 为 Plane 10Hz 任务（判据门控：仅多旋翼模式判，FW/转换不判）；② 失效后动作策略（GCS 告警→按失效号决定 Q_ASSIST 增强/禁转换/QLAND 建议）；③ SITL 联合验证 Dynamic `load_factors()` 与 thrust boost 无冲突；④ 补边界工况注入（同侧双桨/外桨/有风）确认第 2 层的增量价值 | 0.5-1 人月（同 R3） |
| P1 | 空速失效（#6） | 新增空速一致性检查（空速 vs 地速/风），失效时锁定 `AIRSPEED_WAIT` 不退出转换 | 0.5 人月 |
| P2 | GNSS 快速检测（#5） | 在 ekf_check 外增加 GPS HDOP/星数预判通道，缩短触发延迟 | 0.25 人月 |
| P2 | 失速/高度异常（#11/#12） | 新增 `stall_failsafe()` / `altitude_failsafe()` 挂 10Hz 调度表 | 0.5-1 人月（同 R4） |

**多机余度方向**（板级冗余接管）不在本文范围内，见 [redundancy_feasibility.md](../redundancy_feasibility.md)。

**测试规程与验证结果**：单电机失效 SITL 注入测试的完整操作规程、脚本排查历史（log 28-30）与结果数据见 [motor_failure_test_procedure.md](motor_failure_test_procedure.md)（同目录）。

---

## 6. 与既有文档的差异修正

相对旧文档的代码位置修正（本次已逐条 grep/Read 核实）：

1. `battery_failsafe()` 实际定义在 **`ArduPlane/events.cpp:243`**（非 `failsafe.cpp:28-44`——`failsafe.cpp` 仅有主循环卡死的 RC 直通 `failsafe_check()`）。
2. **不存在** `gps_failsafe()` / `airspeed_failsafe()` 函数——GNSS 保护是经 EKF 间接实现的，旧文档"✅ 完整"应修正为"⚠️ 间接"。
3. RC 失效触发点在 `ArduPlane/radio.cpp:299`（10 次计数置位），不是旧的 "radio.cpp:244-299" 整段描述。
4. **`throttle_boost`（`get_throttle_boosted`）不是电机失效补偿**：`code_walkthrough_servo.md:167` 的注释有误。`AC_AttitudeControl_Multi.cpp:374` 是姿态角余弦补偿。真正的电机失效机制是 `AP_MotorsMatrix` 的 `check_for_failed_motor()` + thrust boost（见 §3.5.1），Copter 有 `thrust_loss_check()` 触发器可移植，Plane 缺触发钩子。
5. **`failsafe.cpp` 与电机失效无关**：该文件仅有主循环卡死 RC 直通（1kHz 定时器中断）；电机失效检测在 `libraries/AP_Motors/AP_MotorsMatrix.cpp`，注入测试脚本在 `libraries/AP_Scripting/applets/motor_failure_test.lua`。
