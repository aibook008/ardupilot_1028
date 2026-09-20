# 反向倾转 / 反向过渡逻辑调研

> 对应代码版本：Plane 4.6.x（分支 `qhz/plane_4.6.3`，工作区含走廊门控改动：`tiltrotor.cpp/h`、`Log.cpp`）。
> 所有 `文件:行号` 均已通过 grep + Read 实际验证（2026-09-17，quadplane.cpp 已恢复到提交状态 30fa6af 后核验）。
> 关联文档：正向走廊门控见 `docs/tilt_corridor_optimization.md`；能力缺口分级见 `docs/tilt_corridor_vs_apm.md`；实飞现象见 `docs/df200_fw_roll_tunning.md` §5.4。
> 日志验证：`logs/00000209.BIN`（209 架次，见本文 §8）。

---

## 0. 一句话结论

"反向倾转"在 ArduPilot 中是**三条不同代码路径**：① FW→VTOL 切模式（无任何配对约束，纯裸奔）；② VTOL 着陆序列 `QPOS_AIRBRAKE → QPOS_POSITION1`（有框架但无 θ–V 门控）；③ 倾转速率参数 `Q_TILT_RATE_UP/DN` 的方向语义（UP/DN 指"抬向垂直/压向水平"，而非数值方向）。

正向过渡已有 `corridor_gate_tilt()` 断点门控（`Q_TILT_CRD_*` 参数组），反向方向**完全没有任何 θ–V 配对**——对应 `docs/tilt_corridor_vs_apm.md` 缺口 **G5**。

209 日志验证结论：**路径②全链路工作正常且高度单调下降**（50→0 无飙升）；195 的 90m 飙升发生在路径①——**G5 缺口的暴露面主要是路径①**，反向门控设计应优先覆盖它。

---

## 1. 三条路径总览

| 路径 | 触发 | 倾转角决定方式 | 速率来源 | θ–V 配对 |
|------|------|---------------|----------|----------|
| ① FW→VTOL 切模式 | 手动切 QHOVER/QLAND/QRTL 等 | 前倾位置 → 0°，仅靠 slew | `Q_TILT_RATE_*` | ❌ 无 |
| ② AIRBRAKE 阶段 | QRTL / AUTO VTOL_LAND 距目标足够近 | TECS 油门映射 | `Q_TILT_RATE_*` | ❌ 无 |
| ③ POSITION1 阶段 | AIRBRAKE 退出条件满足 | 前向推力需求 `atan(fwd_g_demand)` | `Q_TILT_RATE_*` | ❌ 无 |

正向对照：WAIT 阶段有 `corridor_gate_tilt()` 门控（`Q_TILT_CRD_EN` 开启时）；TIMER 开环打满不门控；DONE 后旋翼 SHUT_DOWN。

---

## 2. 路径①：FW→VTOL 切模式（最简也最危险）

**调用链**：

```text
模式切换（如 RTL→QHOVER）
  └─ ModeQHover::_enter() → set_max_speed_accel_z() + set_climb_rate_cms(0) + init_throttle_wait()
下一拍 set_servos() → QuadPlane::update()
  └─ in_vtol_mode()=true → 走 VTOL 侧分支
     ├─ assisted_flight = in_vtol_airbrake()   // = false
     ├─ motors_output()
     └─ transition->VTOL_update()
tiltrotor.update() → continuous_update()
```

**关键点**：

- `SLT_Transition::VTOL_update()`（`ArduPlane/quadplane.cpp:1692-1712`）只做两件事：把过渡状态重置为 `TRANSITION_AIRSPEED_WAIT`（供下次正向过渡使用）、记录 `last_throttle`。**没有任何速度检查、没有减速–收倾转配对**。
- 电机在切模式当拍即按 `motors_output()` 生效；倾转从当前角（FW 巡航时约 90°）开始 slew 回垂直。
- FW 高速（DF-200 巡航 55 m/s）下，桨推力垂直分量 T·cosθ 随 θ 回转骤增，叠加前飞动能 → 高度暴增。这正是 195 飞次 RTL 反向倾转 alt 15m→90m、Pitch +24.7° 的机理（见 `docs/df200_fw_roll_tunning.md` §5.4）。
- 209 日志的对比证据：同一架飞机走路径②（AIRBRAKE 框架）高度单调下降、平稳落地；195 走路径①直接切模式即飙升。**G5 的设计重点应放在路径①**。

---

## 3. 路径②：VTOL 着陆 AIRBRAKE 阶段（反向倾转的主战场）

### 3.1 分支入口、状态判定与 case 结构

`QuadPlane::update()`（`ArduPlane/quadplane.cpp:1764`）：

```cpp
if (!in_vtol_mode() && !in_vtol_airbrake()) {
    // 正向过渡分支（transition->update()）
} else {
    assisted_flight = in_vtol_airbrake();     // quadplane.cpp:1783
    motors_output();                          // quadplane.cpp:1786
    transition->VTOL_update();                // quadplane.cpp:1788
}
```

两个状态判定函数是反向逻辑的分支枢纽：

- `in_vtol_mode()`（`quadplane.cpp:2119-2145`）：着陆序列中要求 `get_state() != QPOS_APPROACH && != QPOS_AIRBRAKE` 才算 VTOL（`:2125`）——即 **AIRBRAKE 期间不算 VTOL 模式**。
- `in_vtol_airbrake()`（`quadplane.cpp:4326-4338`）：仅当模式为 QRTL 或 AUTO 的 VTOL_LAND 命令且 `get_state() == QPOS_AIRBRAKE` 时为 true。
- 净效果：AIRBRAKE 期间 `in_vtol_mode()=false`、`in_vtol_airbrake()=true` → 走 VTOL 侧 else 分支且 `assisted_flight=true`（旋翼参与输出，FW 舵面同时有效）。

**case 结构（FALLTHROUGH，不是互斥失误）**：`vtol_position_controller()`（`quadplane.cpp:2342`）的 switch 中，`case QPOS_APPROACH:`（`:2378`）末尾显式 `FALLTHROUGH;`（`:2388`）贯穿到 `case QPOS_AIRBRAKE:`（`:2390`）。两个状态**共享同一段控制体**（TECS 油门 `:2423` + TECS 俯仰 + L1 滚转 `:2434`），靠内部 `get_state()==` 守卫分流：

| 代码 | 只在哪个状态生效 |
|------|------------------|
| 进入 airbrake 判定 `state==QPOS_APPROACH && distance<stop_distance`（`:2454`） | APPROACH |
| tiltrotor 守卫 `state==QPOS_AIRBRAKE`（`:2402`） | AIRBRAKE |
| `hold_hover(0)`（`:2443`） | AIRBRAKE |

所以"为什么 `case QPOS_AIRBRAKE` 里会出现 APPROACH 的判断"——case 标签只是入口，一份逻辑服务两个状态，读代码时要按状态守卫理解。

### 3.2 进入条件

`poscontrol_init_approach()`（`quadplane.cpp:2225-2258`）：

- `DISABLE_APPROACH` 选项设置、或 `dist < approach_distance` → 直接进 QPOS_POSITION1（`:2228-2232`）。
- `dist < transition_threshold()` 且旋翼未满 spool → **QPOS_AIRBRAKE**（`:2236-2248`）；tailsitter 或旋翼已满 spool 则直接 POSITION1。
- 距离还远 → QPOS_APPROACH（`:2251`）。

已进 APPROACH 后，`vtol_position_controller()` 中 `distance < stop_distance`（= `stopping_distance() + 2×closing_speed`，`:2440`）时转 QPOS_AIRBRAKE（`:2454-2471`）。

进入当拍，Z 加速度 PID 积分器清零（`quadplane.cpp:2303-2305`）。

**airbrake 这一档存在的意义**（源码注释 `:2448-2453` 原文 "use the VTOL motors as airbrakes by firing them up before we transition"）：趁还在 FW 模式把旋翼提前转起来，给后续 POSITION1 铺路，同时提供一部分减速。若进近时机队已满 spool（qassist 已在转）则跳过此档直接 POSITION1（`:2455`）。

### 3.3 AIRBRAKE 期间的控制权分配（含 45° 翻转）

`case QPOS_AIRBRAKE`（`quadplane.cpp:2390` 起）：

- **FW 侧仍然在管**：TECS 油门（`:2423`）、TECS 俯仰但**不许低头**（`:2428-2431`，`MAX(nav_pitch_cd, 0)`）、L1 滚转（`:2434`）。
- **旋翼侧起转但油门受两档守卫控制**（`:2402-2414`）：

```cpp
if (tiltrotor.enabled() && poscontrol.get_state() == QPOS_AIRBRAKE) {
    if ((now_ms - last_pidz_active_ms > 2000 && tiltrotor.tilt_over_max_angle()) ||
        tiltrotor.current_tilt >= tiltrotor.get_fully_forward_tilt()) {
        suppress_z_controller = true;      // :2411
        hold_stabilize(0.01);              // :2412 —— 固定 1% 油门
    }
}
```

  - **档 A（tilt > 45°）**：`tilt_over_max_angle()`（`tiltrotor.cpp:989-993`，判据 `current_tilt > max_angle_deg/90`，**严格大于**）为真且 Z 环久未跑（FW 段无人调用 Z 环，`last_pidz_active_ms` 停留在很久以前，条件立即成立）→ `suppress_z_controller + hold_stabilize(0.01)`，旋翼只给 1% 油门。
  - **档 B（tilt ≤ 45°）**：严格大于使守卫失效 → 改走 `hold_hover(0)`（`:2443`）→ Z 环接管，输出小油门维持零爬升。
  - 209 日志精确复现了这个翻转：tilt 到 45° 的那一拍（164.41s），`MOTB.ThrOut` 从恒定 0.010 跳到 0.021→0.043。
- **为什么是 1% 而不是 0**：源码注释 `:2405-2410`——想要一点桨阻力，但**不能让 Z 环跑**：前倾状态下 Z 环为维持爬升率 0 会把桨推到高油门，推力指向前方，越刹越快。
- **实际减速来源**不是旋翼推力，而是：旋转桨盘阻力 + 大倾转姿态 + FW 舵面（俯仰钳制 ≥0 抬头增阻）。209 实测此段减速度仅 ≈0.6 m/s²（50→45.2 m/s 用 8s）——AIRBRAKE 的主要贡献是让旋翼就位，真正的减速主力在 POSITION1（见 §3.6、§8.4）。
- **倾转角由 TECS 油门映射**（`ArduPlane/tiltrotor.cpp:518-523`，与正向 WAIT 共用同一行代码）：

```cpp
const float settilt = constrain_float((k_throttle − throttle_min) * 0.02, 0, 1);
tilt_raw = MIN(settilt * max_angle_deg * (1/90.0), get_fully_forward_tilt());
```

  此分支要求 `in_timer=false`——AIRBRAKE 满足（`assisted_flight=true` 但 `transition_state` 已被 `VTOL_update` 每拍重置为 AIRSPEED_WAIT，见 `quadplane.cpp:1706`）。

### 3.4 suppress_z_controller 机制详解

**它是 `vtol_position_controller()` 的函数内局部变量**（`quadplane.cpp:2363`，`bool suppress_z_controller = false`），每拍重置。作用域是防止 **Z 环在一拍内被跑两次**以及**在危险构型下被跑**，循环末尾统一收口（`quadplane.cpp:2894`）：

```cpp
// run the z controller unless something has already run it or set a target throttle
if (!suppress_z_controller) {
    run_z_controller();
}
```

两个赋值点、两个不同目的：

| 赋值点 | 条件 | 目的 |
|--------|------|------|
| `:2411`（airbrake 守卫） | (Z 环 >2s 没跑 && tilt>45°) \|\| tilt≥90° | **构型安全**：前倾状态下 Z 环高油门 = 正推加速，彻底不让它跑 |
| `:2445`（hold_hover 分支） | `!suppress && state==AIRBRAKE` | **防重入**：`hold_hover(0)` 内部已经跑过一次 Z 环（`hold_hover` → `run_z_controller`，`quadplane.cpp:1111-1126`），置位防止循环末尾 `:2894` 再跑第二次——**一拍内 Z 环跑两次会破坏 PID 的 dt** |

**自持逻辑**：`:2411` suppress 后 → 末尾 `run_z_controller` 被跳过 → `last_pidz_active_ms` 不再更新 → "`>2000ms`" 恒真 → 持续压制，直到 tilt 降到 45° 以下使守卫失效。`run_z_controller()` 本体见 `quadplane.cpp:1052-1077`（>20ms 未跑则重置限幅并 `init_z_controller()`，`:1062-1074`）。

**2 秒阈值的语义**：区分"Z 环真在跑"（如 APPROACH 里 qassist 让旋翼参与过）和"早就停了"（FW 段无人调用 Z 环——本架次属于这种，所以进 AIRBRAKE 第一拍就触发压制）。

### 3.5 AIRBRAKE → POSITION1 退出条件

`quadplane.cpp:2480-2498`：AIRBRAKE 持续 >1s（`min_airbrake_ms=1000`，`:2484`）且满足**任一**：

| 条件 | 判据 |
|------|------|
| 空速过低 | `aspeed < MAX(airspeed_min−2, assist.speed)`（阈值定义 `:2417`） |
| 方向偏差 >60° | `closing_vel.angle(desired_closing_vel)` |
| closing_speed 超带 | `> MAX(1.2×desired, desired+2)` 或 `< 0.5×desired` |
| 姿态误差 >10° | roll/pitch 误差 > 1000cd（`:2480`） |

退出时 `vel_forward.integrator` 用 closing_speed 插值初始化，并限幅 ≤0.5×cruise（`:2502-2513`）——POSITION1 初始前推油门有意识地压低。209 实测：交接拍把 TECS 的饱和值 100 读走做初值，`CTUN.ThO` 随后在 1.3s 内衰减 100→74→26→0。

### 3.6 POSITION1：逐步回垂直 + 减速主力

`case QPOS_POSITION1`（`quadplane.cpp:2557` 起）：

- `in_vtol_mode()` 转 true，控制权从 TECS/L1 **整体交给位置控制器**：`stopping_speed = √(2×transition_decel×distance) + position2_target_speed`（`:2577`）。
- XY 减速需求经 `accel_needed(distance, v²)`（`:2610`）→ 位置控制器输出 nav_pitch——**机体大幅抬头**（209 实测 DesPitch 2°→30.9°）+ 桨回垂直 → 推力矢量后倾分量 + 气动阻力一起刹车。这是整条反向链的**减速主力**（减速度约为 AIRBRAKE 段的 5 倍，见 §8.4）。
- **倾转角的回垂直驱动（209 架次实际路径，已日志逐点验证）**：`get_vfwd_method()`（`:2893-2948`）在 `Q_FWD_THR_USE=0(OFF)` 且无 AUX 开关时返回 **NONE** → `assign_tilt_to_fwd_thr()`（`:2687`，本体 `:2951-2963`）仅将 `q_fwd_throttle=0`（`:2954-2957`）；`QuadPlane::update()` 末尾 `:1801-1808` 因 `in_vtol_mode()`=true 把 `k_throttle` 目标值写 0（`forward_throttle_pct()` 非 NEW 分支返回 0，`:3863-3865`）。**k_throttle 通道带 `THR_SLEWRATE`（本机=100%/s）限速**（`servos.cpp:28-60` 对 `does_auto_throttle()/in_assisted_flight()/in_vtol_mode()` 的模式启用；应用点 `libraries/SRV_Channel/SRV_Channels.cpp:401-410` `calc_pwm()` 内统一 constrain）→ 实际 k_throttle 从 TECS 遗留值 100% 以 100%/s 线性放坡到 0（209 实测斜率 −99.9%/s，耗时 1.0s）。该斜坡经油门映射（`tiltrotor.cpp:521-523`，AIRBRAKE 与 POSITION1 前段共用同一分支：in_timer=false、非 qacro/stab/hover、非纯 FW）→ `tilt_raw = MIN(k×0.02,1)×45` 同步从 45°→0°，再被 tilt slew 10°/s 拉平（详见 §8.2 两段式）。
  - **`atan(0.01×forward_throttle_pct())` 分支（`tiltrotor.cpp:482-493`）在本架次未参与**：它要求 `get_vfwd_method()==NEW`（`q_fwd_thr_gain>0` 且 `Q_FWD_THR_USE=ALL/POSCTRL` 或 AUX 激活，`:2929-2941`）。若启用 NEW 方法，POSITION1 的 tilt 才由前推需求映射；本机参数下走的是上面这条"k_throttle 归零 + 通道限速"路径。
  - POSITION1 的姿态控制末尾调 `assign_tilt_to_fwd_thr()`（`:2687`）与尾部 `:2742`、`:2788` 同理——method=NONE 时均为空操作。

`forward_throttle_pct()`（`quadplane.cpp:3831-3865`）：`ActiveFwdThr::NEW` 时返回 `100×q_fwd_throttle`（`:3834-3836`）；OLD 方法走 `vel_forward` 积分器链路（`:3863-3948`）；**NONE（本机）返回 0**。

### 3.7 两阶段对比

| 特性 | QPOS_AIRBRAKE | QPOS_POSITION1 |
|------|---------------|----------------|
| in_vtol_mode / in_vtol_airbrake / assisted_flight | false / **true** / **true** | true / false / false |
| 俯仰/滚转控制 | TECS（俯仰≥0）/ L1 | 位置控制器 / 位置控制器 |
| 旋翼油门 | 1% 恒定 → 小油门 Z 环（45° 翻转） | 位置控制器 Z 环（扛重量） |
| 倾转策略 | 油门映射，前倾产生阻力 | 前推需求映射，逐步回垂直 |
| 减速贡献 | 小（≈0.6 m/s²），主要让旋翼就位 | 大（≈3 m/s²），抬头机动为主力 |

---

## 4. 倾转速率：`Q_TILT_RATE_UP/DN` 的方向语义（重要澄清）

`slew()`（`tiltrotor.cpp:266-275`）与 `tilt_max_change()`（`tiltrotor.cpp:238-261`）：

```cpp
void Tiltrotor::slew(float newtilt) {
    // up = (newtilt < current_tilt) —— 收倾转、回垂直方向
    float max_change = tilt_max_change(newtilt < current_tilt,
                                       newtilt > get_fully_forward_tilt());
    current_tilt = constrain_float(newtilt, current_tilt−max_change, current_tilt+max_change);
    ...
}

float Tiltrotor::tilt_max_change(bool up, bool in_flap_range) const {
    // up=true（回垂直） → max_rate_up_dps   (Q_TILT_RATE_UP)
    // up=false（压前倾）→ max_rate_down_dps (Q_TILT_RATE_DN)；DN≤0 时回退用 UP
    ...
}
```

**结论**：UP/DN 的"上/下"指**桨盘抬向垂直（up）/压向水平（dn）**，不是数值方向。由此得到一个文档里没点破的推论：

> `docs/df200_fw_roll_tunning.md` §5.4 候选修复①"Q_TILT_RATE_UP 10→3–5（反向放慢）"：在 **Q_TILT_RATE_DN=0（默认）时，正向过渡 WAIT 的前倾速率也回退用 UP**（前倾时 `newtilt>current` → `up=false`，DN=0 → 回退 UP），会把正向前倾也拖慢到 3–5°/s。
>
> **正确解耦做法（参数已存在，无需改代码）**：
> - `Q_TILT_RATE_DN = 10`（管正向打前倾，保持 `tilt_corridor_optimization.md` §10.4 论证的 10°/s 平滑性）
> - `Q_TILT_RATE_UP = 3–5`（只管反向收倾转回垂直，垂向推力分量缓升）

### 4.1 例外：fast_tilt 覆盖

`tilt_max_change`（`tiltrotor.cpp:246-258`）：MANUAL 模式、或 armed 且 `!in_vtol_mode() && !assisted_flight`（纯 FW 调姿）时，速率强制 `MAX(rate, 90)` °/s——手动/纯 FW 不受慢速 slew 限制。反向收倾转发生在 VTOL/辅助飞行状态，不触发此例外。

### 4.2 参数建议（针对 195 的 90m 飙升）

| 参数 | 建议值 | 理由 |
|------|--------|------|
| Q_TILT_RATE_DN | 10 | 显式接管正向 WAIT 前倾速率，解除 DN=0 回退 UP 的隐患 |
| Q_TILT_RATE_UP | 3–5 | 反向收倾转放慢（§5.4 候选修复①），垂向推力分量缓升 |
| Q_TILT_MAX | 45° 保持 | AIRBRAKE 阻力倾转上限，过大 → 抬头力矩风险 |

---

## 5. 倾转在各阶段的联动机制

**核心：全阶段只有一个状态量 `current_tilt` + 每拍 `slew()` 限速率逼近，阶段切换只换 `tilt_raw` 的来源，不换执行机构**。`Tiltrotor::update()`（`tiltrotor.cpp:585-601`）每拍调 `continuous_update()`（`:401`），各阶段在同一函数里按状态选择目标角：

| 阶段 | tilt_raw 来源 | 代码 |
|------|--------------|------|
| 纯 FW | 固定 90°（forward_flight_tilt），油门走 `output_motor_mask` | `tiltrotor.cpp:409-441` |
| 正向 WAIT | `(k_throttle−min)×0.02` 油门映射 + 走廊门控 | `:518-523`、`:529-536` |
| 正向 TIMER | 开环打满 90°，不门控 | `:510-517` |
| **AIRBRAKE** | **与 WAIT 同一段代码**（in_timer=false, assisted=true）→ TECS 油门映射 | `:518-523` |
| POSITION1 / VTOL 悬停（本机 method=NONE） | **仍走油门映射**（k_throttle 目标归 0，经 THR_SLEWRATE 限速放坡→tilt_raw 45°→0°） | `:518-523` |
| POSITION1 / VTOL 悬停（若 method=NEW） | `atan(0.01×forward_throttle_pct)` 前推需求映射 | `:482-493` |
| QACRO/QSTABILIZE/QHOVER | RC 前推通道或 0 | `:494-508` |
| 切模式 FW→VTOL（路径①，本机） | 同上：k_throttle 归零放坡 → 油门映射 | `:518-523` |

速率统一由 `slew()` 管（回垂直 `Q_TILT_RATE_UP`，压前倾 `Q_TILT_RATE_DN`，见 §4）。**阶段切换时目标函数切换、`current_tilt` 连续、slew 保证无跳变**——这就是"联动"的全部机制。

**AIRBRAKE→POSITION1 的目标切换实例**（209 实测，§8.2）：AIRBRAKE 段 tilt_raw 来自 TECS 油门映射（随 TECS 需求饱和钉在 45°）；state 3 当拍 tilt_raw 换成前推需求映射（17.4°→0° 随 vel_forward 衰减），current_tilt 从 45° 继续以 10°/s slew 到 0°，全程无跳变。

---

## 6. 与走廊门控（Q_TILT_CRD_*）的交互——209 实证无害

`corridor_gate_tilt()` 的启用条件是 `crd_enable > 0 && !in_timer`（`tiltrotor.cpp:529-530`），而 AIRBRAKE 期间 `VTOL_update()` 每拍把状态重置为 `TRANSITION_AIRSPEED_WAIT`（`quadplane.cpp:1706`），`in_timer=false`——**门控代码路径在 AIRBRAKE 阶段确实在跑**。

但细看 `corridor_gate_tilt()` 内部（`tiltrotor.cpp:318-383`）后，风险比最初预估的小：

1. **门控只作用于"前倾方向的上穿"**：第一分支 `tilt_raw <= current_tilt + eps` 直接放行（`:337-342`）。AIRBRAKE 的回垂直是 tilt **下降**方向，永远走放行分支——**门控管不住回垂直，只可能限制打前倾**。
2. **AIRBRAKE 期间空速很高**（45–50 m/s，正是刹车的目的），即使 raw 上穿触发门 Vn 检查，也立即通过（V ≫ Vn）。
3. 209 实测：TILT 日志 `Crd=-1`、`Spd=nan` 全程——门控路径在跑但从未进入 hold。

结论：**按 DF-200 当前参数（CRD_V 表 ≤45、AIRBRAKE 段 V≥45），门控对反向链实际无害**。唯一理论残留场景：二次进近时 current_tilt 已低且空速低于 Vn——此时 hold 恰是门控的设计意图（低速不给大前倾），不是缺陷。§8 的混合门参数仿真验证可以不专门覆盖 AIRBRAKE 场景。

---

## 7. 缺口定位与改进方向

### 7.1 现状判定

- **正向已有门控、反向完全裸奔**：`corridor_gate_tilt()` 只挂正向 WAIT（且 TIMER 明确跳过，`tiltrotor.cpp:510-517`）；反向无论切模式（路径①）还是 AIRBRAKE 收倾转（路径②③）都没有 θ–V 配对。
- `docs/tilt_corridor_vs_apm.md` §3.7 + 缺口表 **G5**："反向转换无减速–收倾转配对 | 高速大倾转回垂直风险 | 三期试飞前"。正向在 §6.2 已有设计级伪代码，**反向连设计级方案都还没有**。
- 195 的 90m 飙升是 G5 的实飞体现；候选修复②（"RTL 阶段 AS<15 才开始 tilt 回转"）本质上就是把正向门控思想搬到反向。
- **209 的对比证据缩小了缺口范围**：路径②（AIRBRAKE 框架）实测平稳、高度单调下降——框架内的 TECS 俯仰钳制、1% 油门、逐步回垂直已经构成事实上的"减速–收倾转"配对。**G5 的暴露面集中在路径①（高速切模式直接回垂直）**。

### 7.2 反向门控候选方案（供后续设计讨论）

借鉴正向 `corridor_gate_tilt()` 的断点思想，反向方向做"速度–回转角"配对：

1. **判据方向反转**：正向是"要更前倾（θ↑）须 V≥Vn"；反向是"要更回垂直（θ↓）须 V≤Vn"——回转角越大（越接近垂直），允许的空速越低。
2. **挂点**：`continuous_update()` 的 VTOL 侧分支（`tiltrotor.cpp:444` 起）对 `newtilt < current_tilt`（回垂直方向）的 slew 目标做门控 hold，与正向 `tilt_cmd = corridor_gate_tilt(tilt_raw)` 对称。
3. **必须覆盖路径①**：切模式瞬间没有 assisted_flight 状态可依，需在 `VTOL_update()` 或模式切换入口采样当时空速，作为反向门控的初始档位依据（高速切 VTOL 时先 hold 大前倾角，让 FW 舵面+前倾阻力先减速）。
4. **兜底**：高速下 hold 前倾角与现有 `hold_stabilize(0.01)` 保护（`quadplane.cpp:2402-2414`）语义一致；若超时（如 5s 未减速到档位），降级为 QLAND / 强制回转+油门压低。

### 7.3 排期对应

- `docs/development_plan.md` W11（10-26~11-01）："固定翼 → 旋翼反向过渡 + 降落"，交付物**反向过渡报告**——G5 的"三期试飞前"与 W11 时间点吻合。
- W13 全模态串飞含"过渡回 → 降落"。
- 近期可先做的低风险项：§4.2 参数解耦（纯参数）+ 209 已验证路径②框架可用，反向门控设计聚焦路径①。

---

## 8. 209 架次日志验证（logs/00000209.BIN）

分析方法：pymavlink DFReader 按 0.5s 采样 QPOS/TILT/MOTB/TECS/ARSP/ATT/CTUN 七通道（150–190s 窗口）。TILT 的 `Traw/Tmax/Spd/Crd` 为本仓库扩展字段（`tiltrotor.cpp:603-638`）。

### 8.1 QPOS 状态时间轴

| 时刻(s) | 迁移 | 含义 |
|---------|------|------|
| 135.34 | 0→1 | NONE→APPROACH，进近开始 |
| 159.74 | 1→2 | APPROACH→AIRBRAKE（`distance < stop_distance`） |
| 167.76 | 2→3 | AIRBRAKE→POSITION1（持续 8.0s，退出条件满足） |
| 182.14 | 3→4 | POSITION1→POSITION2（14.4s 减速段结束，AS≈9.5） |
| 183.63 | 4→5 | →LAND_DESCEND |
| 218.53 / 236.73 | 5→7→8 | LAND_FINAL→LAND_COMPLETE（6=LAND_ABORT 未触发） |

（枚举值：`quadplane.h:498-506`）

### 8.2 TILT 轨迹（回垂直全程 10°/s = Q_TILT_RATE_UP；Traw 数值经公式逐点验证）

油门映射公式（`tiltrotor.cpp:521-523`，本机 THR_MIN=0、Q_TILT_MAX=45、flap=0）：**tilt_raw_deg = MIN(k_throttle×0.02, 1)×45 = k≥50% 封顶 45°，k<50% 线性 0.45°/%**。

| 时段(s) | Tilt | Traw | k_throttle | 解读 |
|---------|------|------|-----------|------|
| 159.74–164.41 | 89.97→45.00 | 19.8→45.0 | 22→100% | state 2：slew 回垂直；Traw=MIN(k×0.02,1)×45 逐点吻合（19.83/27.60/37.33/45.00 vs 预测 19.83/28.54/38.35/45.00） |
| 164.41–167.76 | **45.00 hold** | 45.00 | 70→100%（TECS 饱和） | Tilt 追上被 **Q_TILT_MAX 封顶**的 raw（满油门 settilt=1.0 → 1.0×45°=45°）；封顶者是 Q_TILT_MAX 本身（油门映射需 200% 才能到 90°），与走廊门控无关（Crd=-1 全程） |
| 167.76–168.78 | 45.0→36.7 | 45→0（1.0s） | 100→0（斜率 −99.9%/s） | state 3：`in_vtol_mode()` → k_throttle 目标归 0（`:1801-1808`），**经 THR_SLEWRATE=100%/s 通道限速放坡**（`SRV_Channels.cpp:404-410`），Traw=MIN(k×0.02,1)×45 同步衰减；Tilt 被 slew 10°/s 拉着 |
| 168.78–173.24 | 36.7→0 | 0 | 0 | Traw 已到底，Tilt 以 10°/s 走完剩余 36.7°（4.4s） |
| 173.24 起 | 0 | 0 | 0 | 悬停构型 |

**两段式机理**：Traw 斜坡只持续 1s（k_throttle 限速 100%/s 决定），Tilt 全程受 10°/s slew 限制——两条限速链串联，慢的（tilt slew）决定实际轨迹总时长 5.2s。

速率核验：84.80→79.60 用 0.52s = 10°/s，与 Q_TILT_RATE_UP=10 一致；45→0 用 5.2s ≈ 45/10 ✓。

### 8.3 旋翼油门三阶段（MOTB.ThrOut）——守卫翻转的精确复现

| 时段(s) | 条件 | 油门来源（代码） | MOTB.ThrOut |
|---------|------|------------------|-------------|
| 159.7–164.4 | tilt>45° 且 Z 环久未跑 | `hold_stabilize(0.01)`（`:2412`） | **0.010 恒定** |
| 164.4–168.0 | tilt≤45°，严格大于失效 | `hold_hover(0)` → Z 环（`:2443`） | 0.021→0.043（维持零爬升） |
| 173.2 之后 | AS<35 机翼升力不足 | Z 环扛重量 | 0.35→0.48 |

翻转时刻 164.41s = Tilt 恰好到 45.00° 的那一拍，与 `tilt_over_max_angle()` 严格大于判据逐拍吻合。

**state 3 交接**（167.76）：`CTUN.ThO` 100→74→26→0 在 1.3s 内衰减——`:2502` 把 TECS 饱和值 100 读走做 vel_forward 积分器初值后快速衰减到 0（减速段不需要前推）。

### 8.4 减速主力在 POSITION1（不是 AIRBRAKE）

| 阶段 | 时长 | AS 变化 | 平均减速度 |
|------|------|---------|-----------|
| state 2 (AIRBRAKE) | 8.0s | 50.0→45.2 m/s | ≈0.6 m/s² |
| state 3 (POSITION1) | 12.1s | 45.2→9.5 m/s | ≈3.0 m/s²（约 5 倍） |

姿态：state 2 全程 Pitch≈2°（TECS 钳制）；state 3 DesPitch 2°→30.9°（`accel_needed` 需求），Pitch 峰值 31.5°（171.61s，小超调），随后回落。**抬头 30° + 桨回垂直的推力矢量刹车才是减速主力**；AIRBRAKE 的贡献是旋翼就位 + 温和减速。

### 8.5 TECS 油门饱和 1.0（"僵尸需求"，无害）

- state 2 段 TECS.th 0.22→1.00（160.26→166.96s）：气动阻力大，TECS 拼命保空速直至饱和。
- **state 3 起 TECS.th 恒 1.0 直到 183.5s+**（TECS 日志持续到 LAND_DESCEND）：TECS 本体仍按 50Hz 更新，目标空速 spdem 恒 ~46 m/s 而实际 sp 掉到 <5 → 欠速保护钉死 1.0。
- 但 k_throttle 在 state 3 已由 vel_forward 接管（`:2502`），**TECS 输出无人消费**——日志上吓人，实为架构现象，不是故障。

### 8.6 8 点结论

1. 代码链路（§3）与日志逐点吻合，路径②无需修改。
2. 全程高度单调下降（50→45→…→0），**无 195 式飙升**——路径②框架有效。
3. 45° hold 机理 = TECS 饱和 → settilt=1.0 → **Q_TILT_MAX 封顶**（油门映射本身需 k=200% 才到 90°，flap=0 时 MIN 上限 90° 永不先到），非走廊门控（Crd=-1 全程）。
4. 减速主力 = POSITION1 抬头 30° 机动（5 倍于 AIRBRAKE 段）。
5. 守卫翻转（0.01→hold_hover）在 tilt=45° 拍精确发生，代码-日志互证。
6. TECS 饱和 1.0 是"僵尸需求"，交接后无人消费。
7. 195（路径①飙升）vs 209（路径②平稳）对比 → **G5 缺口优先覆盖路径①**。
8. 低速末段旋翼油门 0.44–0.48（含机翼残余升力与地效，非纯悬停值，勿直接当 Q_M_HOVER 校核）。
9. **Traw 全程 = MIN(k_throttle×0.02,1)×45 逐点验证成立**（state 2 随 TECS 需求爬升、state 3 随 k_throttle 经 THR_SLEWRATE=100%/s 限速放坡归零）；本机 Q_FWD_THR_USE=0 → `atan(fwd_g_demand)` 分支未参与，两阶段共用同一油门映射分支，区别只在 k_throttle 的来源（TECS 需求 vs 归零目标）。

---

## 9. 关键代码锚点速查

| 内容 | 位置 |
|------|------|
| 分支入口：正向过渡 vs VTOL 侧 | `ArduPlane/quadplane.cpp:1764`、`:1783`、`:1788` |
| SLT_Transition::VTOL_update（切模式反向重置） | `ArduPlane/quadplane.cpp:1692-1712` |
| in_vtol_mode（AIRBRAKE 不算 VTOL） | `ArduPlane/quadplane.cpp:2119-2145`（判据 `:2125`） |
| in_vtol_airbrake | `ArduPlane/quadplane.cpp:4326-4338` |
| poscontrol_init_approach（进入 APPROACH/AIRBRAKE/POSITION1） | `ArduPlane/quadplane.cpp:2225-2258` |
| QPOS 状态枚举 | `ArduPlane/quadplane.h:498-506` |
| vtol_position_controller 主函数 | `ArduPlane/quadplane.cpp:2342` |
| suppress_z_controller 局部变量声明 | `ArduPlane/quadplane.cpp:2363` |
| FALLTHROUGH case 结构（APPROACH→AIRBRAKE） | `ArduPlane/quadplane.cpp:2378-2390` |
| AIRBRAKE 旋翼守卫（0.01 油门 / suppress） | `ArduPlane/quadplane.cpp:2402-2414` |
| hold_hover(0) 分支 + 防重入 | `ArduPlane/quadplane.cpp:2442-2446` |
| AIRBRAKE 进入时 Z 积分器清零 | `ArduPlane/quadplane.cpp:2303-2305` |
| AIRBRAKE→POSITION1 退出条件 | `ArduPlane/quadplane.cpp:2480-2498` |
| vel_forward 交接初始化 | `ArduPlane/quadplane.cpp:2502-2513` |
| QPOS_POSITION1 case（位置控制 + stopping_speed） | `ArduPlane/quadplane.cpp:2557-2611` |
| 末尾 Z 环防重入收口 | `ArduPlane/quadplane.cpp:2894` |
| run_z_controller（dt 重置/init） | `ArduPlane/quadplane.cpp:1052-1077` |
| hold_stabilize / hold_hover | `ArduPlane/quadplane.cpp:1028-1049` / `:1111-1126` |
| forward_throttle_pct | `ArduPlane/quadplane.cpp:3831-3865` |
| get_vfwd_method（NONE/OLD/NEW 判定） | `ArduPlane/quadplane.cpp:2893-2948` |
| assign_tilt_to_fwd_thr（method=NONE 时置零） | `ArduPlane/quadplane.cpp:2951-2963`（POSITION1 调用点 `:2687`） |
| in_vtol_mode 时 k_throttle=fwd_thr 写入 | `ArduPlane/quadplane.cpp:1801-1808` |
| k_throttle 通道 slew（THR_SLEWRATE 启用判定） | `ArduPlane/servos.cpp:28-60` |
| slew 统一应用点（calc_pwm） | `libraries/SRV_Channel/SRV_Channels.cpp:401-410` |
| set_slew_rate 实现 | `libraries/SRV_Channel/SRV_Channel_aux.cpp:786-815` |
| CTUN.ThO = k_throttle 日志源 | `ArduPlane/Log.cpp:112` |
| continuous_update（各阶段 tilt_raw 来源） | `ArduPlane/tiltrotor.cpp:401-537`（纯FW `:409-441`，前推映射 `:482-493`，TIMER `:510-517`，油门映射 `:518-523`，门控 `:529-536`） |
| corridor_gate_tilt 内部 | `ArduPlane/tiltrotor.cpp:318-383` |
| slew / tilt_max_change（UP/DN 方向语义） | `ArduPlane/tiltrotor.cpp:266-275`、`:238-261`（fast_tilt 例外 `:246-258`） |
| tilt_over_max_angle（严格大于） | `ArduPlane/tiltrotor.cpp:989-993` |
| TILT 日志字段（Traw/Tmax/Spd/Crd） | `ArduPlane/tiltrotor.cpp:603-638` |
| QRTL 入口 vtol_position_controller 调用 | `ArduPlane/mode_qrtl.cpp:160` |
| 正向走廊门控文档 | `docs/tilt_corridor_optimization.md` |
| G5 缺口原文 | `docs/tilt_corridor_vs_apm.md` §3.7、§4 缺口表 |
| 195 实飞现象 | `docs/df200_fw_roll_tunning.md` §5.4 |
| 209 日志验证 | 本文 §8（`logs/00000209.BIN`） |
