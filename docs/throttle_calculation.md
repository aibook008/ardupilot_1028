# ArduPilot VTOL 油门计算逻辑详解

| 项 | 内容 |
|----|------|
| 代码基线 | 分支 `qhz/plane_4.6.3`（Plane 4.6.x） |
| 关联 | `tiltrotor_motor_control.md`、`tilt_corridor_vs_apm.md`、`vtol_jt20.md` |
| 说明 | 行号均已用工具核对；改代码后引用前请重核 |

## 概述

本文档说明 ArduPilot QuadPlane / Tiltrotor 的油门计算流程，重点包括：

1. `set_servos()` 里固定翼油门（`k_throttle` / TECS）与旋翼油门（`motors->_throttle`）两条并行通道  
2. 正向过渡 **WAIT / TIMER / DONE** 各自的油门来源  
3. **JT-20（内四倾转 + 外四固定垂起）** 与「全倾转、无永久 VTOL」在 TIMER 混合上的差异  
4. `tilt_compensate` 对倾转电机与非倾转电机的不同处理  

**两条油门通道不要混成一条流水线**：旋翼通道经 `motors->output()` → `k_motorN`；固定翼前推经 TECS → `k_throttle` → `set_throttle()` / slew。二者最后都在 `servos_output()` 下硬件。

## 1. 主循环调用链（一级一级展开）

### 1.1 主循环入口

```
ArduPlane/ArduPlane.cpp:59-61  调度表定义
├─ FAST_TASK(update_control_mode)   ← 50Hz
├─ FAST_TASK(stabilize)             ← 50Hz
└─ FAST_TASK(set_servos)            ← 50Hz  ← 油门计算主入口
```

### 1.2 set_servos() 完整调用链

```
ArduPlane/servos.cpp:828  Plane::set_servos()
│
├─[849] quadplane.update()                         ← QuadPlane::update (quadplane.cpp:1717)
│   │
│   ├─[!in_vtol_mode] transition->update()         ← 固定翼模态下的过渡/辅助 (约 1778)
│   │   │
│   │   ├─ TRANSITION_AIRSPEED_WAIT (1539)
│   │   │   ├─ assist_climb_rate_cms() (1443) → climb_rate
│   │   │   ├─ hold_hover(climb_rate) (1597 / 1111)
│   │   │   │   ├─ multicopter_attitude_rate_update()
│   │   │   │   ├─ set_climb_rate_cms()
│   │   │   │   └─ run_z_controller() → update_z_controller()
│   │   │   │       └─ set_throttle_out() → motors->set_throttle()
│   │   │   └─ last_throttle = motors->get_throttle() (1615)  ← 供 TIMER 冻结用
│   │   │
│   │   ├─ TRANSITION_TIMER (1628)
│   │   │   ├─ throttle_scaled = last_throttle × (1 - t/T) (1642-1643)
│   │   │   ├─ [仅全倾转] 与 TECS k_throttle 按 tilt ratio 混合 (1655-1661)
│   │   │   └─ hold_stabilize(throttle_scaled) (1664 / 1028)
│   │   │       ├─ multicopter_attitude_rate_update()
│   │   │       └─ set_throttle_out(throttle_scaled)   ← 不跑 Z 位置环
│   │   │
│   │   └─ motors_output() (1687)                  ← WAIT/TIMER 共用出口
│   │
│   ├─[in_vtol_mode] motors_output() (1786)        ← 纯 VTOL 模态
│   │
│   └─ tiltrotor.update() (1799)                   ← continuous_update：倾转角
│       └─ motors->output() 链路内 thrust_compensation
│           └─ tilt_compensate() (tiltrotor.cpp:507)
│
├─[857] set_throttle()                             ← 仅 k_throttle（固定翼前推）
├─ throttle_slew_limit(k_throttle)
└─[929] servos_output()                            ← 唯一真正写硬件
```

### 1.2.1 关键澄清：`output_to_motors()` 不是最终硬件输出

`motors->output()` 链路里的 `output_to_motors()` **不会直接推到电调硬件**。

```
motors->output()
  └─ output_to_motors()
      └─ rc_write(i, pwm)                          [AP_Motors_Class.cpp:105]
          └─ SRV_Channels::set_output_pwm(k_motorN, pwm)
             或 SRV_Channels::set_output_scaled(...)
             ↑ 只是把值写进 SRV_Channel 对象

set_throttle() / throttle_slew_limit()
  └─ 只处理 SRV_Channel::k_throttle（固定翼前推通道）
     不影响 k_motor1..N（多旋翼电机通道）

servos_output()
  ├─ calc_pwm()        ← 所有通道统一做 scaled→PWM / slew
  └─ output_ch_all()   ← 真正写硬件：hal.rcout->write()
```

| 函数 | 作用对象 | 是否写硬件 |
|------|---------|:---------:|
| `motors->set_throttle()` | `motors._throttle` 内部变量 | ❌ |
| `motors->output()` → `rc_write()` | `SRV_Channel` 上的 `k_motor1..N` 值 | ❌ |
| `set_throttle()` | `SRV_Channel::k_throttle` 值 | ❌ |
| `servos_output()` → `output_ch_all()` | 所有 SRV 通道 | ✅ |

因此调用顺序不是“重复输出”，而是：

1. **算并暂存**：VTOL 电机值、固定翼油门值分别写到不同 SRV 功能通道
2. **统一后处理**：`set_throttle()` 只后处理固定翼 `k_throttle`
3. **统一下发**：`servos_output()` 一次性把电机 + 舵面 + 油门都推到硬件

### 1.3 update_control_mode() 中的 TECS 调用链

```
ArduPlane/ArduPlane.cpp:500  Plane::update_control_mode()
│
├─ mode->run()
│   │
│   └─[mode_auto.cpp:92/104/120] calc_throttle()
│       │
│       └─[Attitude.cpp:459] TECS_controller.get_throttle_demand()
│           │
│           └─[AP_TECS.cpp:759] _throttle_dem = ...
│               │
│               └─ SRV_Channels::set_output_scaled(k_throttle, ...)
```

## 2. 三种飞行模式的油门来源

### 2.1 纯固定翼模式

**调用链**：
```
update_control_mode() [ArduPlane.cpp:500]
  └─ mode->run() [mode_auto.cpp:77]
      └─ calc_throttle() [Attitude.cpp:459]
          └─ TECS_controller.get_throttle_demand() [AP_TECS.cpp:985]
              └─ SRV_Channels::set_output_scaled(k_throttle, ...) [Attitude.cpp:471]
```

**TECS 工作原理**：
- 输入：目标高度、目标空速、当前高度、当前空速
- 输出：油门百分比（0-100%）、俯仰角
- 核心思想：用油门控制总能量（势能+动能），用俯仰控制能量分配

**关键参数**：
- `ARSPD_FBW_MIN`：最小空速
- `ARSPD_FBW_MAX`：最大空速
- `THR_MIN`：最小油门
- `THR_MAX`：最大油门
- `TECS_CLMB_MAX`：最大爬升率

### 2.2 纯旋翼模式（VTOL Hover）

**注意**：`hold_hover()` **不在** `motors_output()` 里调用。它在模式 `run()` / 过渡状态机里先设好油门；`motors_output()` 只做角速度 PID + 混控输出。

**调用链（两段）**：
```
// 段 A：先算油门（模式 run 或 transition WAIT）
hold_hover(target_climb_rate_cms) [quadplane.cpp:1111]
  ├─ multicopter_attitude_rate_update()     ← 姿态目标
  ├─ set_climb_rate_cms()
  └─ run_z_controller()
      └─ pos_control->update_z_controller() [AC_PosControl.cpp:989]
          ├─ 位置环 / 速度环 / 加速度环
          └─ set_throttle_out() [AC_AttitudeControl_Multi.cpp:350]
              └─ motors->set_throttle()     ← 只写 motors._throttle

// 段 B：再混控并写 SRV（set_servos → quadplane.update）
motors_output() [quadplane.cpp:1687 或 1786 / 1965]
  ├─ rate_controller_run()                  ← 角速度 PID
  └─ motors->output()
      ├─ output_armed_stabilizing()
      ├─ thrust_compensation() / tilt_compensate (tiltrotor.cpp:507)
      └─ output_to_motors() → rc_write()    ← 写 k_motorN 到 SRV_Channel

// 段 C：统一硬件输出
servos_output() [servos.cpp:929]
  ├─ calc_pwm()
  └─ output_ch_all()                        ← 真正写硬件
```

**位置控制器工作原理**：
- 输入：目标高度/爬升率、当前高度、当前垂直速度
- 输出：油门百分比（0-100%）
- 核心思想：串级 PID（位置环 → 速度环 → 加速度环 → 油门）

**关键参数**：
- `Q_P_POSXY_P`：水平位置 P 增益
- `Q_P_POSZ_P`：垂直位置 P 增益
- `Q_P_VELXY_P`：水平速度 P 增益
- `Q_P_VELZ_P`：垂直速度 P 增益
- `Q_P_ACCZ_P`：垂直加速度 P 增益

### 2.3 过渡模式总览

正向过渡状态机（`SLT_Transition::update`，固定翼模态下由 `QuadPlane::update` 调用）：

```
AIRSPEED_WAIT ──V>AIRSPEED_MIN──► TIMER ──时间到/fully_fwd──► DONE
   hold_hover                         hold_stabilize              VTOL 停转
   Z 环闭环油门                        开环衰减油门
```

| 阶段 | 旋翼总油门来源 | 姿态 | 倾转角 θ | JT-20 注意 |
|------|----------------|------|----------|------------|
| **WAIT** | `hold_hover` → Z 位置/速度环 | 多旋翼姿态环 | ≤ `Q_TILT_MAX`，跟前飞油门 | 内外四同一集体油门；内四另乘 \(1/\cos\theta\) |
| **TIMER** | `last_throttle×(1-t/T)` → `hold_stabilize` | 仍稳姿态 | 强制 `slew(前飞角)` | **不做** tilt–TECS 油门混合（有永久 VTOL） |
| **DONE** | VTOL spool 关闭 | — | 前飞位 | 外四停；倾转电机可当推进 |

详细展开见 **§2.4 WAIT**、**§2.5 TIMER**。

### 2.4 WAIT 阶段油门（TRANSITION_AIRSPEED_WAIT）

#### 2.4.1 控制意图

空速未达标：用旋翼**托住飞机并稳住姿态**，同时内四在 `Q_TILT_MAX` 内倾转，用前向分量加速，直到 \(V>\texttt{AIRSPEED\_MIN}\)。

#### 2.4.2 总油门怎么来（闭环，不是手写 mg）

```
assist_climb_rate_cms()                    [quadplane.cpp:1443]
  └─ 自动油门模态：altitude_error_cm × 0.1
     或 FBW：俯仰/油门估爬升率；2s 斜坡；限幅
        │
        ▼
hold_hover(climb_rate)                     [1111]
  ├─ multicopter_attitude_rate_update()
  ├─ set_climb_rate_cms()
  └─ run_z_controller()
      └─ pos_control->update_z_controller()  [AC_PosControl.cpp:989]
          └─ set_throttle_out() → motors->set_throttle()
```

物理上：Z 环为对抗重力、跟踪爬升率指令，把 collective 油门抬到需要的值——**等效「按重力/高度误差算出油门」**，但是串级 PID，不是开环 \(T=mg\)。

倾转 tiltrotor 在 WAIT 里通常不刻意用旋翼爬高（`LEVEL_TRANSITION` 对非 tiltrotor 会压爬升率；tiltrotor 不压，以免高迎角低 VTOL 推力，见约 1588–1596 行注释）。

每次 WAIT 末尾：

```cpp
last_throttle = motors->get_throttle();   // 1615：冻结值留给 TIMER
```

#### 2.4.3 倾转角（与油门并行）

`tiltrotor.continuous_update()`：在 `assisted_flight` 且尚未到 TIMER 时，θ 跟前飞油门（TECS/`k_throttle` 或 VFWD）走，且 **θ ≤ Q_TILT_MAX**（`tiltrotor.cpp` 约 328–334 行）。  
速率受 `Q_TILT_RATE_*` 的 `slew()` 限制。

#### 2.4.4 混控后：内四 / 外四（JT-20）

`motors_output()` → `motors->output()` → `tilt_compensate()`（`tiltrotor.cpp:507`）。

固定翼过渡方向（`!in_vtol_mode()`）：

| 电机 | 补偿 | 推力方向 | WAIT 角色 |
|------|------|----------|-----------|
| **外四**（不在 tilt_mask） | ×1 | 始终向上 | 主要垂向托举 |
| **内四**（tilt_mask） | ×\(1/\cos\theta\) | 倾角 θ | 补偿后垂向 + 几何前向加速 |

另有：随 tilt 减弱内四滚转差动、按 \(\sin\theta\) 加倾转差动偏航（见 `tilt_compensate_angle`）。

```
WAIT 油门链（旋翼通道）:

  Z 环 collective
       │
       ▼
  MotorsMatrix 分到 8 电机（含姿态差动）
       │
       ├─ 外四 ×1
       └─ 内四 × 1/cosθ
       │
       ▼
  物理分解：内四 → Fz=T cosθ, Fx=T sinθ
```

#### 2.4.5 WAIT 与「重力」说法的精确表述

| 口语 | 精确含义 |
|------|----------|
| 「油门由重力算出来」 | Z 环闭环维持垂向加速度≈0（或跟踪爬升率），集体油门隐含平衡重量 |
| 「不是开环 mg」 | 无 `throttle = mass*g/...` 公式；有传感器反馈与 hover 油门学习 |
| 「tilt 再补偿」 | 仅倾转电机指令放大，使混控在垂直假设下仍够垂向分量 |

### 2.5 TIMER 阶段油门（TRANSITION_TIMER）

#### 2.5.1 控制意图（先澄清一个易错点）

**不是「油门降到 0 → 整机没前推、没动力」。**

TIMER 衰减的是 **多旋翼集体油门（垂向辅助）**，前提是空速已过 `AIRSPEED_MIN`、机翼已能分担升力。与此同时：

1. **θ 被强制往前打**（`slew(前飞角)`）→ 内四剩余推力里 **前向分量 \(T\sin\theta\) 占比变大**  
2. **TECS 仍在算 `k_throttle`**（能量/空速），固定翼舵面仍在工作  
3. θ 到全前飞 → `fully_fwd()` → **DONE**，此后倾转电机走纯 FW 路径，用 TECS 经 `output_motor_mask` 做前推  

所以 TIMER 是 **撤垂向辅助 + 把推力矢量转到前飞 + 把推进权交给 TECS**，不是空窗掉动力。

#### 2.5.2 旋翼集体油门：开环时间衰减

```cpp
// quadplane.cpp:1642-1664
transition_scale = (T - t) / T;                  // T = Q_TRANSITION_MS
throttle_scaled  = last_throttle * transition_scale;
throttle_scaled  = max(throttle_scaled, 0.01f); // 地板，避免姿态环电机完全停转

// 仅当：tiltrotor 且 !has_vtol_motor() 且 !has_fw_motor()
//   throttle_scaled = (1-r)*throttle_scaled + r*fw_throttle;
// JT-20 有外四永久 VTOL → 不进入该混合

hold_stabilize(throttle_scaled);
```

`hold_stabilize`（1028）：姿态环 + **直接** `set_throttle_out`，**不跑 Z 环**。

| | WAIT | TIMER |
|--|------|-------|
| 函数 | `hold_hover` | `hold_stabilize` |
| 旋翼集体油门 | Z 环闭环 | `last_throttle×时间因子`（开环衰减） |
| 衰减含义 | — | **撤垂向托举辅助**（机翼已接上） |
| 前向动力 | 内四 \(T\sin\theta\)（θ≤TILT_MAX） | 内四 \(T\sin\theta\) 且 **θ→90°**；TECS 在旁等待接棒 |

#### 2.5.3 倾转角与前向力（动力为什么还在）

一进 TIMER（`tiltrotor.cpp` 323–327）：

```cpp
slew(get_forward_flight_tilt());  // 目标全前飞，只受 RATE 限制
```

| 量 | TIMER 中的变化 | 对动力的含义 |
|----|----------------|--------------|
| 集体油门 \(T\) | 时间上减小 | 垂向辅助减少（外四尤其如此） |
| 倾转角 θ | RATE 限制下增大→90° | \(\sin\theta\uparrow\)：剩余推力更多变成前推 |
| 内四前向力 | \(\sim T\cdot\sin\theta\)（再经混控/补偿） | **不会**随 \(T\) 同比掉光 |
| TECS `k_throttle` | 持续计算 | DONE / 纯 FW 后经 mask 接到倾转电机 |

直观：刚进 TIMER 时 θ 约在 `Q_TILT_MAX`，还有明显垂向；TIMER 末 θ≈90°，很小的集体油门也几乎全是前向，然后 DONE 换成 TECS 前推。

#### 2.5.4 JT-20：TIMER 内四 vs 外四

| | 外四 | 内四 |
|--|------|------|
| 油门指令 | 同一套衰减 collective | 同一套衰减 + \(1/\cos\theta\) |
| 方向 | 一直向上 | θ→90° |
| 衰减时失去的 | **垂向托举**（正是 TIMER 要撤的） | 垂向分量；**前向分量靠倾角接住** |
| TECS 写入电机 | 无（不在 tilt_mask） | TIMER 内仍走 MC 混控；**fully_fwd/DONE 后**才 `output_motor_mask(TECS)` |
| 角色 | 垂直余量按时间撤掉 | 矢量转到前飞，再交给 TECS |

TIMER 期间 `assisted_flight==true`，`continuous_update` **不会**走「纯 FW + `output_motor_mask(k_throttle)`」分支（该分支要求 `!assisted_flight`，见 `tiltrotor.cpp` 222 行）。  
因此 JT-20 在 TIMER **内部**的前推，主要是 **倾转几何下的 MC 推力**；TECS 接棒发生在 **fully_fwd → DONE / 离开 assist** 之后。

#### 2.5.5 全倾转构型的 TIMER 混合（对比，非 JT-20）

当 `!has_vtol_motor() && !has_fw_motor()` 时，TIMER 内就把 TECS 混进同一套电机油门，接棒更早、更平滑：

\[
r=\frac{\theta-\theta_{\mathrm{as}}}{\theta_{\mathrm{fwd}}-\theta_{\mathrm{as}}},\quad
throttle=(1-r)\,throttle_{vtol衰减}+r\,throttle_{FW}
\]

| 时刻 | ratio | 油门来源 |
|------|-------|---------|
| 刚进 TIMER | 0 | 100% 衰减旋翼油门 |
| 中间 | 0.5 | 旋翼与 TECS 各半 |
| θ→前飞 | 1 | 100% 固定翼油门 |

**JT-20（`Q_TILT_MASK=15`，外四永久 VTOL）不走此分支**——不是「没有前推」，而是前推接棒路径不同（几何前推 → DONE 后 TECS mask）。

### 2.5.6 各电机混控后油门日志 MOTT（转 PWM 前，0~1）

`QTUN.ThI/ThO` 是**集体油门**。各电机在混控 + `tilt_compensate` 之后、转 PWM 之前的量是：

```text
AP_MotorsMatrix::_thrust_rpyt_out[i]   // 0~1，与 ThI 同量纲
```

固件已在 `AP_MotorsMatrix::output_to_motors()` 里、`thrust_to_actuator()` / `rc_write()` **之前**写 **`MOTT`**（armed 时约 50 Hz）：

| 字段 | 含义 |
|------|------|
| `TimeUS` | 时间戳 |
| `Mot1`…`Mot8` | 电机 1…8 的 `_thrust_rpyt_out` |

JT-20：`Mot1`…`Mot8` ↔ `RCOU.C5`…`C12`。  
用途：直接看倾转补偿后内四/外四差；后续功率可用 `P_i=f(Mot_i)`（盘模型）或实机 `ESC.Volt*Curr`。  
旧 log 无 `MOTT` 时，仍可用 `RCOU` + `Q_M_SPIN_*/THST_EXPO` 反推近似值。

### 2.6 DONE

- `spool SHUT_DOWN`，永久 VTOL（外四）停  
- `assisted_flight` 结束且 `!in_vtol_mode` 时，`continuous_update` 走纯 FW：倾转电机打满前飞，**`output_motor_mask(k_throttle)`** 用 TECS 做前推（`tiltrotor.cpp` 222–253）  
- 外四不在 mask → DONE 后外四保持关闭  

### 2.6.1 实测：切 FW 瞬间掉高 ~5 m + TECS 油门「未及时接棒」（00000023）

> 日志坑先记一条：**`QTUN.Alt` 在 t≈101.8（Trn=2 入口）后停录，到 160.4 才恢复**。看 FW 阶段高度必须用 `AHR2.Alt`（AMSL）/ `BARO.Alt` / `TECS.h`，不要用 `QTUN.Alt`，否则会误判「高度卡住没掉」。

#### 现象（AHR2/BARO/TECS.h 三源一致）

Trn 1->2 入口（t≈101.57，倾转刚到 90）：

| t | BARO.Alt | TECS.h | Tecs.th | CTUN.ThO | Aspd |
|----|----------|--------|---------|----------|------|
| 100.5 | 20.5 | 20.6 | 1.00 | 100 | 13 |
| 101.5 | 19.8 | 20.0 | 0.79 | 83.7 | 19 |
| 102.0 | 18.3 | 18.3 | 0.75 | 76.2 | 23 |
| 103.0 | 15.3 | 15.2 | 0.56 | 58.3 | 28.6 |
| 103.5 | **15.0** | 15.0 | 0.46 | 47.9 | 30 |
| 105.0 | 18.1 | 18.1 | 0.17 | 18.7 | 31 |
| 106.5 | 20.7 | 20.6 | 0.00 | 0 | 29 |
| 108–110 | 17–19 | 17–19 | 0 | 0 | 29（二次掉高）|

- 入口 1.5 s 内 BARO 从 19.8 掉到 15.0，**约 5 m**
- `AHR2` 算的掉高 5.12 m，`BARO` 5.3 m，一致
- 之后 TECS 把油门拉回，高度回 20.6；但 106.5–110 油门再次 0，又二次掉到 17

#### 代码级原因：TECS 油门为何「衰减 + 没及时加上去」

不是 TECS 没在算，是**它的积分被反复清、且 PD 项在切换瞬间算出来本就低**，导致前飞油门跟不上旋翼升力撤除的速率。

**(1) WAIT 阶段反复清 TECS 油门积分**（`quadplane.cpp:1609`）

```cpp
if (quadplane.tiltrotor.enabled() && !quadplane.tiltrotor.has_fw_motor()) {
    // tilt rotors without dedicated fw motors do not have forward throttle output in this stage
    // prevent throttle I wind up
    plane.TECS_controller.reset_throttle_I();   // 每拍把 _integTHR_state = 0
}
```

`reset_throttle_I()`（`AP_TECS.h:97`）直接 `_integTHR_state = 0`。WAIT 期间 VTOL 为主、TECS 不该输出前推，于是**每帧清零**。后果：进 TIMER/DONE 时 TECS 积分是 0，要重新累积，**接棒有延迟**。

**(2) TECS 油门 = PD + FF + 积分，切换瞬间三项都偏低**（`AP_TECS.cpp:759–802`）

```cpp
_throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;  // PD+FF
_integTHR_state = _integTHR_state + (_STE_error * _get_i_gain()) * _DT * K_STE2Thr;      // 积分（被清过）
...
_throttle_dem = _throttle_dem + _integTHR_state;                                          // 最终 = PD+FF+积分
```

- `ff_throttle = throttle_cruise + STEdot_dem/K_thr2STE`：起飞/过渡瞬间能量误差刚转正，FF 不大
- 积分项被 WAIT 清零，要从 0 累
- 结果：TECS 算出的 `_throttle_dem` 在切换瞬间就是 0.4~0.8（不是 1.0），且要靠速率限制慢慢爬

**(3) 油门速率限制进一步压住爬升**（`AP_TECS.cpp:784–798`）

```cpp
int8_t throttle_slewrate = aparm.throttle_slewrate;   // THR_SLEWRATE
if (throttle_slewrate != 0) {
    const float thrRateIncr = _DT * (_THRmaxf - _THRminf) * throttle_slewrate * 0.01f;
    _throttle_dem = constrain_float(_throttle_dem,
                                    _last_throttle_dem - thrRateIncr,
                                    _last_throttle_dem + thrRateIncr);
}
```

即每拍油门增量受 `THR_SLEWRATE` 限。配合积分从 0 起步，TECS 油门**爬得比旋翼撤升力慢**。

**(4) 旋翼侧是「开环时间衰减」，撤得干脆**（`quadplane.cpp:1643`）

```cpp
throttle_scaled = last_throttle * transition_scale;   // 按 Q_TRANSITION_MS 线性衰减到 0.01
```

旋翼集体油门按 `last_throttle × (1−t/T)` 线性掉，**和 TECS 是两条独立链路**。JT-20 有外四永久 VTOL，又**不走** TIMER 内 `throttle_scaled = (1−r)*MC + r*TECS` 那条混合（`quadplane.cpp:1655` 的 `!has_vtol_motor()` 条件不满足）。

#### 失效机理（一句话）

> 旋翼升力按时间线性撤除（快），TECS 前飞油门因**积分被 WAIT 反复清零 + FF 偏低 + 速率限制**而爬得慢；JT-20 又不走 TIMER 内 MC↔TECS 混合 -> 切换瞬间总推力（旋翼垂托↓ + 前飞↑）出现塌陷 -> 掉高 ~5 m。等积分重建、油门爬到 0.7+ 才接住。

#### 该看哪些日志量（避免 QTUN 坑）

| 阶段 | 高度源 | 油门源 |
|------|--------|--------|
| VTOL/WAIT/TIMER | `QTUN.Alt` 可用 | `QTUN.ThO`（旋翼集体） |
| **FW 及之后** | **`AHR2.Alt` / `BARO.Alt` / `TECS.h`**（QTUN 停录） | `TECS.th`（需求）/ `CTUN.ThO`（实输出）/ `AETR.Thr` |
| 全程连续 | `AHR2` / `BARO` | `TECS.th` |

#### 缓解方向

- **延长 TIMER**：`Q_TRANSITION_MS`（默认 5000）调大，让旋翼撤得更慢
- **倾转速率**：`Q_TILT_RATE_UP/DN` 调慢，给机翼升力建立时间
- **`THR_SLEWRATE`**：调大或设 0，让 TECS 油门爬得快
- **不要在 WAIT 清 TECS 积分**（代码级，更激进）：保留小量积分预载，但风险是 WAIT 期 TECS 误输出前推
- **JT-20 也走 TIMER 内 MC↔TECS 混合**（代码级）：放宽 `has_vtol_motor()` 条件或单独给 JT-20 一条 ratio 淡入

### 2.7 风险项：JT-20 前飞油门源硬切换（与倾转走廊同类缺口）

#### 2.7.1 现象

倾转打满 / 离开 assist 进入纯前飞时，内四油门源发生切换：

```
TIMER 内：  衰减 MC 集体油门 → 混控 → ×1/cosθ
    ↓  fully_fwd / DONE / !assisted_flight
纯 FW：    TECS k_throttle → output_motor_mask(tilt_mask)
```

全倾转构型在 TIMER 内用 tilt ratio **提前把 TECS 混进同一套油门**，注释写明目的就是 *remove a step change*（`quadplane.cpp` 1655–1661）。  
**JT-20 有外四永久 VTOL，不走该混合** → 切换处缺少交叉淡入。

`continuous_update` 对 TECS 侧仅有 `tilt_max_change` 速率限制；**already fully_fwd 时直接 `current_throttle = TECS`**（`tiltrotor.cpp` 237–238），不能保证与 TIMER 末 MC 指令数值连续。

#### 2.7.2 风险等级与后果

| ID | 风险 | 可能后果 | 与走廊关系 |
|----|------|----------|------------|
| **R-THR-01** | MC→TECS 油门台阶 | 内四推力突变 → 过载、俯仰扰动、空速抖动 | 与「无 σ–V 中线调度」同属**过渡轨迹开环、缺平滑交接** |
| **R-THR-02** | TIMER 末 MC 已很低、TECS 巡航油门偏高（或相反） | 上跳加速或下跳掉速 | 走廊要求的 \(T_i(\theta),P(\theta)\) **在切换点不连续** |
| **R-THR-03** | 切换时刻依赖 fully_fwd / assist，与 `Q_TRANSITION_MS`、RATE 不同步 | 同一套参数下仿真/试飞难复现 | 与走廊「时间表倾转 ≠ 气动中线」同类 |

本质同一类问题：**APM 用状态机 + 速率限制“蹭”过渡，不保证 (θ,V,推力) 沿设计走廊连续。**  
见 `tilt_corridor_vs_apm.md` 缺口 G2–G4；本条记为推力轴上的对称缺口 **G6（油门交接不连续）**。

#### 2.7.3 缓解（设计级，待立项）

| 优先级 | 措施 | 说明 |
|--------|------|------|
| P0 | 仿真打点 | TIMER 末记录内四 PWM/推力 vs DONE 后首拍 TECS mask；量化台阶 ΔT、Δa_x |
| P1 | 参数对齐 | 使打满 θ 时 MC 衰减末值接近 TECS 巡航需求；对齐 `Q_TRANSITION_MS` 与 `Q_TILT_RATE_DN` |
| P2 | 软件淡入 | JT-20 在 TIMER 末或 DONE 初按时间/θ 对内四做 MC↔TECS 交叉淡入（类全倾转 ratio，但不要求去掉外四） |
| P2 | 走廊调度 | \(\theta_{\mathrm{cmd}}=f(V)\) + 显式 \(T_i\) 曲线，使切换点落在走廊中线连续段（catch-up G4 联动） |

#### 2.7.4 对甲表述（脱敏）

> 一期混合倾转构型在「旋翼辅助撤出 → 固定翼前推接管」处依赖状态切换；完整方案需增加推力交接平滑与倾转角–空速走廊监视，避免切换点推力台阶。不对外写 APM 参数名与源码路径。

## 3. set_throttle() 函数详解

**调用链**：
```
set_servos() [servos.cpp:828]
  └─ set_throttle() [servos.cpp:857]
      ├─[595] 电池电压补偿
      ├─[610] 油门限制（min/max）
      └─[625] 油门抑制
```

**代码**：
```cpp
// servos.cpp:595
void Plane::set_throttle(void)
{
    // 步骤 1：电池电压补偿
    g2.fwd_batt_cmp.update();
    if (control_mode->use_battery_compensation()) {
        const float throttle = g2.fwd_batt_cmp.apply_throttle(
            SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        );
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }
    
    // 步骤 2：油门限制（min/max）
    if (control_mode->use_throttle_limits()) {
        const float limited_throttle = apply_throttle_limits(
            SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        );
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, limited_throttle);
    }
    
    // 步骤 3：油门抑制（着陆、安全等）
    if (suppress_throttle()) {
        if (g.throttle_suppress_manual) {
            // 手动模式下油门直通
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, get_throttle_input(true));
        } else if (landing.is_flaring() && landing.use_thr_min_during_flare()) {
            // 着陆拉平时使用最小油门（涡轮机场景）
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, aparm.throttle_min.get());
        } else {
            // 默认：油门归零
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        }
    }
}
```

### 3.1 电池电压补偿

**目的**：电池电压下降时，增加油门以维持推力

```cpp
// 补偿公式
compensated_throttle = original_throttle / (voltage / nominal_voltage)
```

**关键参数**：
- `BATT_VOLT_MAX`：最大电压
- `BATT_VOLT_MIN`：最小电压
- `BATT_ARM_VOLT`：解锁电压

### 3.2 油门限制

**调用链**：
```
set_throttle() [servos.cpp:610]
  └─ apply_throttle_limits() [servos.cpp:610]
```

```cpp
float Plane::apply_throttle_limits(float throttle)
{
    // 应用最小油门限制
    throttle = MAX(throttle, aparm.throttle_min.get());
    
    // 应用最大油门限制
    throttle = MIN(throttle, aparm.throttle_max.get());
    
    // 应用油门 nudge（手动微调）
    if (control_mode == &mode_manual) {
        throttle += throttle_nudge;
    }
    
    return throttle;
}
```

**关键参数**：
- `THR_MIN`：最小油门（通常 0%）
- `THR_MAX`：最大油门（通常 100%）

### 3.3 油门抑制

**调用链**：
```
set_throttle() [servos.cpp:625]
  └─ suppress_throttle() [servos.cpp:73]
```

`suppress_throttle()` 函数判断是否需要强制关闭油门：

```cpp
bool Plane::suppress_throttle(void)
{
    // 手动模式：从不抑制
    if (control_mode == &mode_manual) {
        return false;
    }
    
    // 着陆模式：抑制油门
    if (landing.is_complete() && !arming.is_armed()) {
        return true;
    }
    
    // 地面检测：速度低且高度接近 home
    if (!is_flying() && control_mode->does_auto_throttle()) {
        if (fabsf(relative_altitude) < 10.0f && ground_speed < 5.0f) {
            return true;
        }
    }
    
    return false;
}
```

## 4. throttle_slew_limit() 函数

**调用链**：
```
set_servos() [servos.cpp:883]
  └─ throttle_slew_limit(k_throttle) [servos.cpp:24]
      └─ SRV_Channels::set_slew_rate() [servos.cpp:273]
```

**目的**：限制油门变化速率，防止突变

```cpp
// servos.cpp:24
void Plane::throttle_slew_limit(SRV_Channel::Aux_servo_function_t func)
{
    // 只在自动油门模式下启用
    const bool do_throttle_slew = (
        control_mode->does_auto_throttle() || 
        quadplane.in_assisted_flight() || 
        quadplane.in_vtol_mode()
    );
    
    if (!do_throttle_slew) {
        SRV_Channels::set_slew_rate(func, 0.0, 100, G_Dt);
        return;
    }
    
    // 获取油门变化率限制
    uint8_t slewrate = aparm.throttle_slewrate;
    
    // 特殊场景使用不同的变化率
    if (control_mode == &mode_auto) {
        if (auto_state.takeoff_complete == false && g.takeoff_throttle_slewrate != 0) {
            slewrate = g.takeoff_throttle_slewrate;  // 起飞阶段
        } else if (landing.get_throttle_slewrate() != 0 && 
                   flight_stage == AP_FixedWing::FlightStage::LAND) {
            slewrate = landing.get_throttle_slewrate();  // 着陆阶段
        }
    }
    
    // VTOL 过渡使用起飞变化率
    if (g.takeoff_throttle_slewrate != 0 && 
        (flight_stage == AP_FixedWing::FlightStage::TAKEOFF ||
         flight_stage == AP_FixedWing::FlightStage::VTOL)) {
        slewrate = g.takeoff_throttle_slewrate;
    }
    
    if (g.takeoff_throttle_slewrate != 0 && quadplane.in_frwd_transition()) {
        slewrate = g.takeoff_throttle_slewrate;
    }
    
    // 设置变化率限制
    SRV_Channels::set_slew_rate(func, slewrate, 100, G_Dt);
}
```

**关键参数**：
- `THR_SLEWRATE`：油门变化率（%/秒）
- `TKOFF_THR_SLEW`：起飞油门变化率
- `LAND_THR_SLEW`：着陆油门变化率

## 5. 完整数据流示例（带完整调用链）

### 场景：过渡期间（倾转 45°）

#### 步骤 1：TECS 计算固定翼油门

**完整调用链**：
```
主循环 50Hz
  └─ update_control_mode() [ArduPlane.cpp:500]
      └─ mode->run() [mode_auto.cpp:77]
          └─ calc_throttle() [Attitude.cpp:459]
              └─ TECS_controller.get_throttle_demand() [AP_TECS.cpp:985]
                  └─ SRV_Channels::set_output_scaled(k_throttle, 80) [Attitude.cpp:471]
```

**核心代码**：
```cpp
// AP_TECS.cpp:759 - TECS 计算油门需求
_throttle_dem = (_STE_error + STEdot_error * throttle_damp) * K_STE2Thr + ff_throttle;

// 油门变化率限制（防止突变）
if (throttle_slewrate != 0) {
    const float thrRateIncr = _DT * (_THRmaxf - THRminf_clipped_to_zero) * throttle_slewrate * 0.01f;
    _throttle_dem = constrain_float(_throttle_dem,
                                    _last_throttle_dem - thrRateIncr,
                                    _last_throttle_dem + thrRateIncr);
    _last_throttle_dem = _throttle_dem;
}

// Attitude.cpp:471 - 输出到 SRV_Channel
SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _throttle_dem * 100);
```

**物理含义**：
- `_STE_error`：总能量误差（势能 + 动能）
- `STEdot_error`：总能量变化率误差
- `K_STE2Thr`：能量误差到油门的转换系数
- `ff_throttle`：前馈油门（基于飞行状态）

**示例数据**：
- 目标空速 15 m/s，当前空速 10 m/s
- 需要加速 → 油门 = 80%
- `SRV_Channels::set_output_scaled(k_throttle, 80)`

---

#### 步骤 2a：WAIT 时 hold_hover（对照）

```
transition->update() → TRANSITION_AIRSPEED_WAIT
  ├─ climb = assist_climb_rate_cms()           [1443]
  ├─ hold_hover(climb)                         [1597/1111]
  │   └─ Z 环 → motors->set_throttle()         ← 闭环集体油门
  ├─ last_throttle = motors->get_throttle()    [1615]
  └─ motors_output() → tilt_compensate
```

倾转角：`continuous_update` 中 θ≤Q_TILT_MAX，跟前飞油门；内四 ×1/cosθ，外四 ×1。

#### 步骤 2b：TIMER 衰减后再 hold_stabilize

**调用链**（先算 throttle_scaled，再给定；混合仅全倾转）：
```
transition->update() → TRANSITION_TIMER          [1628]
  ├─ throttle_scaled = last_throttle × (1-t/T)   [1642-1643]
  ├─ [可选] 全倾转：与 TECS 按 tilt ratio 混合   [1655-1661]
  ├─ hold_stabilize(throttle_scaled)             [1664/1028]
  │   ├─ multicopter_attitude_rate_update()
  │   └─ set_throttle_out(throttle_scaled)       ← 不跑 Z 环
  └─ motors_output()                             [1687]
```

```cpp
// quadplane.cpp:1642-1664（摘录）
float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
float throttle_scaled = last_throttle * transition_scale;
if (throttle_scaled < 0.01f) {
    throttle_scaled = 0.01f;
}
if (tiltrotor.enabled() && !has_vtol_motor() && !has_fw_motor()) {
    // 仅全倾转构型
    const float ratio = ...; // (θ - θ_as) / (θ_fwd - θ_as)
    throttle_scaled = (1-ratio)*throttle_scaled + ratio*fw_throttle;
}
hold_stabilize(throttle_scaled);
```

**JT-20**：有外四永久 VTOL → **跳过混合**，内四油门=衰减集体油门经混控与 \(1/\cos\theta\)；前飞靠倾角转到前面，不是 TIMER 里另开前飞油门 PID。

| 函数 | 调用场景 | 油门来源 | 是否跑位置环 |
|------|---------|---------|-------------|
| `hold_hover()` | WAIT / 纯悬停 | climb_rate → Z 环 | ✓ |
| `hold_stabilize()` | TIMER | `last_throttle×时间因子`（±全倾转混合） | ✗ |
---

#### 步骤 3：set_throttle() 后处理（仅 k_throttle）

**完整调用链**：
```
主循环 50Hz
  └─ set_servos() [servos.cpp:828]
      └─ set_throttle() [servos.cpp:857]
          ├─[595] 电池电压补偿：0.7 × 1.05 = 0.735
          ├─[610] 油门限制：constrain(0.735, 0.0, 1.0) = 0.735
          └─[625] 油门抑制：不抑制
```

**核心代码**：
```cpp
// servos.cpp:595-659
void Plane::set_throttle(void)
{
    // 1. 电池电压补偿
    if (control_mode->use_battery_compensation()) {
        const float throttle = g2.fwd_batt_cmp.apply_throttle(
            SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        );
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
    }
    
    // 2. 油门限制（min/max）
    if (control_mode->use_throttle_limits()) {
        const float limited_throttle = apply_throttle_limits(
            SRV_Channels::get_output_scaled(SRV_Channel::k_throttle)
        );
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, limited_throttle);
    }
    
    // 3. 油门抑制（着陆、安全等）
    if (suppress_throttle()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
    }
}
```

---

#### 步骤 4：tilt_compensate() 补偿

**调用链**：`motors_output()` → `motors->output()` → `thrust_compensation()` → `tilt_compensate()`（`tiltrotor.cpp:507`）。

真实逻辑（与「所有电机统一 ×1/cos」的简化说法不同）：

```cpp
// tiltrotor.cpp:507-525（结构）
if (current_tilt <= 0) return;
if (in_vtol_mode()) {
    // 回垂向：非倾转电机 × cosθ，倾转电机 ×1
    tilt_compensate_angle(thrust, n, cosθ, 1);
} else {
    // 往前倾（WAIT/TIMER 所在）：非倾转 ×1，倾转 × 1/cosθ
    tilt_compensate_angle(thrust, n, 1, 1/cosθ);
}
```

`tilt_compensate_angle`（454–499）额外对**倾转电机**：

1. 随 `current_tilt` 把滚转差动往平均值揉：`thrust = tilt*avg + thrust*(1-tilt)`  
2. 按 \(\sin\theta\) 与 `Q_TILT_YAW_ANGLE` 加差动偏航  
3. 若有电机 >1，**全体**按比例缩小  

**示例（θ=45°，往前倾）**：混控给内四 0.5 → ×√2 ≈ 0.707；外四保持混控值。若内四补偿后饱和再全局缩放。

**物理含义**：

- 仅**倾转电机**用 \(1/\cos\theta\) 弥补垂向分量损失（外四不乘）  
- 滚转效力随倾转迁移/衰减；矢量偏航用倾转差动  
- 与「集体油门先统一放大再分配」不是同一实现
---

#### 步骤 5：物理分解（倾转 45°）

**说明**：这是物理过程，不是代码计算。电机倾转 45° 后，推力自动分解为：

```
电机推力 = 1.0（归一化后）
  ├─ 垂直升力 = 1.0 × cos(45°) = 0.707 ✓ 维持高度
  └─ 前飞推力 = 1.0 × sin(45°) = 0.707 ✓ 提供加速
```

---

#### 步骤 6：电机值写入 SRV_Channel（不是硬件输出）

**完整调用链**：
```
主循环 50Hz
  └─ set_servos() [servos.cpp:828]
      └─ quadplane.update() [servos.cpp:849]
          └─ motors_output() [quadplane.cpp:1687 或 1786 / 定义于 1965]
              ├─ rate_controller_run()
              │   └─ 角速度 PID → motors.set_roll/pitch/yaw()
              └─ motors->output()
                  ├─ output_armed_stabilizing()  ← throttle+roll+pitch+yaw 混控
                  ├─ thrust_compensation()       ← tilt_compensate 回调
                  └─ output_to_motors()
                      └─ rc_write(i, pwm) [AP_Motors_Class.cpp:105]
                          └─ SRV_Channels::set_output_pwm(k_motorN, pwm)
                             ↑ 只写入 SRV_Channel，不写硬件
```

#### 步骤 7：真正硬件输出

**完整调用链**：
```
主循环 50Hz
  └─ set_servos() [servos.cpp:828]
      ├─ set_throttle() [servos.cpp:857]                 ← 已在步骤 3 完成（仅 k_throttle）
      ├─ throttle_slew_limit(k_throttle) [servos.cpp:883]
      └─ servos_output() [servos.cpp:929]
          ├─ calc_pwm() [servos.cpp:1022]
          │   └─ 所有 SRV 通道：scaled/slew → PWM
          └─ output_ch_all() [SRV_Channel_aux.cpp:107]
              └─ channels[i].output_ch()
                  └─ hal.rcout->write()  ← 真正输出到硬件
```

**为什么步骤 6 的 output_to_motors 之后还要 set_throttle / servos_output：**

| 步骤 | 改的是什么 | 说明 |
|------|-----------|------|
| 步骤 6 `rc_write(k_motorN)` | 多旋翼电机通道 | VTOL 混控结果已就绪，但还在软件缓冲里 |
| 步骤 3 `set_throttle()` | 固定翼 `k_throttle` | 与 VTOL 电机通道无关；只后处理前推油门 |
| 步骤 7 `servos_output()` | 全部通道 | 统一做混控（vtail/elevon 等）并真正写硬件 |

两条油门通道并行：

```
VTOL 电机通道：  hold_* → motors._throttle → motors->output() → k_motor1..N → servos_output()
固定翼前推通道： TECS → k_throttle → set_throttle()/slew → servos_output()
```

---

### 5.1 与相关文档对照及不一致项

| 说法 / 来源 | 核对结果 | 处理 |
|-------------|----------|------|
| 「TIMER 一定混合 TECS + 旋翼油门」 | 仅 `!has_vtol_motor && !has_fw_motor` | 正文改为「全倾转才混合」；JT-20 不混合 |
| 「tilt_compensate = 所有电机 ×1/cos」 | 往前倾时**仅倾转电机** ×1/cos；外四 ×1 | 步骤 4 已按源码改写 |
| 「滚转效力简单 thrust×(1-tilt)」 | 实际是 `tilt*avg + thrust*(1-tilt)` 揉向平均值 | 已更正 |
| 旧文档行号（hold_hover≈1092 等） | 现 `hold_hover` 1111、`hold_stabilize` 1028、`tilt_compensate` 507、`update` 1717 | 索引表已更新 |
| `tiltrotor_motor_control.md`「按 forward_throttle 定角」 | WAIT/未 TIMER：是；**TIMER：强制前飞角** | 分阶段写清，避免合成一句 |
| 「WAIT 油门=重力公式」 | 实为 Z 环闭环 | §2.4.5 精确表述 |
| 「倾转补偿在 set_throttle 流水线里」 | `set_throttle` 只动 `k_throttle`；补偿在 `motors->output` | §7.2 拆成两路 |

与 `tiltrotor_motor_control.md` 仍一致的部分：倾转电机同时承担垂向/前向；输出经混控 + `tilt_compensate`；全倾转 TIMER 可用 tilt ratio 混合油门。

## 6. 关键代码位置索引（Plane 4.6.x / 本分支核对）

| 功能 | 文件 | 行号 | 符号 |
|------|------|------|------|
| 油门主入口 | servos.cpp | 828 | `Plane::set_servos()` |
| QuadPlane 更新 | quadplane.cpp | 1717 | `QuadPlane::update()` |
| 过渡 WAIT | quadplane.cpp | 1539 | `TRANSITION_AIRSPEED_WAIT` |
| 辅助爬升率 | quadplane.cpp | 1443 | `assist_climb_rate_cms()` |
| WAIT 悬停油门 | quadplane.cpp | 1597 / 1111 | `hold_hover()` |
| 冻结 last_throttle | quadplane.cpp | 1615 | `last_throttle = get_throttle()` |
| 过渡 TIMER | quadplane.cpp | 1628 | `TRANSITION_TIMER` |
| TIMER 衰减/混合 | quadplane.cpp | 1642–1664 | `throttle_scaled` / 全倾转混合 |
| TIMER 稳定油门 | quadplane.cpp | 1028 | `hold_stabilize()` |
| 过渡末 motors_output | quadplane.cpp | 1687 | `motors_output()` |
| 电机输出 | quadplane.cpp | 1965 | `QuadPlane::motors_output()` |
| 倾转角更新 | tiltrotor.cpp | 214 | `continuous_update()` |
| 倾转补偿 | tiltrotor.cpp | 507 / 454 | `tilt_compensate` / `_angle` |
| 油门后处理（FW） | servos.cpp | 595 / 857 | `set_throttle()` |
| 统一硬件输出 | servos.cpp | 929 / 985 | `servos_output()` |
| Z 位置环 | AC_PosControl.cpp | 989 | `update_z_controller()` |
| set_throttle_out | AC_AttitudeControl_Multi.cpp | 350 | `set_throttle_out()` |

## 7. 总结

### 7.1 油门来源

| 模式 | 旋翼/倾转电机油门 | 固定翼 `k_throttle` | 备注 |
|------|-------------------|---------------------|------|
| 纯 FW | 通常关闭或 tilt mask 前推 | TECS | |
| 纯 VTOL | `hold_hover` Z 环 | 可选 VFWD | |
| **WAIT** | Z 环闭环 + tilt 补偿 | TECS 仍可算（助推 θ） | 内四×1/cos，外四×1 |
| **TIMER** | `last_throttle` 时间衰减（±全倾转混合） | TECS 继续 | JT-20 无 tilt–TECS 混合 |
| DONE | VTOL 停；倾转电机可 mask 前推 | TECS | 外四停 |

### 7.2 两路后处理（勿画成单串）

```
旋翼通道:  hold_* → motors._throttle → 混控 → tilt_compensate → k_motorN ─┐
                                                                           ├─ servos_output() → 硬件
固定翼通道: TECS → k_throttle → set_throttle / slew ───────────────────────┘
```

`set_throttle()` 的电池补偿 / min-max / suppress **只作用于 `k_throttle`**，不经过 `tilt_compensate`。

### 7.3 设计要点

1. **WAIT 闭环托举，TIMER 开环撤力**——空速门槛前后油门机理不同。  
2. **θ 与集体油门在 TIMER 解耦**：油门按时间掉，角按 RATE 往前打。  
3. **构型分叉**：全倾转才在 TIMER 混 TECS；JT-20 有永久 VTOL 则不混。  
4. **补偿分电机**：\(1/\cos\theta\) 只打在 tilt_mask 上。  
5. **走廊/外四 fo(θ)** 不是 APM 原生量——见 `tilt_corridor_vs_apm.md`；参数与脚本另接。

## 8. 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| （原稿） | — | 调用链、TECS/旋翼/过渡、tilt 补偿示例 |
| 本次 | 2026-08-26 | 补 WAIT 专节；TIMER 内四原理；JT-20 vs 全倾转；校正 tilt_compensate 与行号；梳理与旧表述不一致项 |
| +风险 | 2026-08-26 | §2.7：JT-20 MC→TECS 硬切换风险 R-THR-01..03；与走廊缺口 G6 对齐 |