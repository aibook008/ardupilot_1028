# ArduPilot VTOL 油门计算逻辑详解

## 概述

本文档详细说明 ArduPilot 中 VTOL 倾转旋翼的油门计算流程，重点分析 `set_servos()` 函数中的油门输出逻辑，以及固定翼（TECS）和旋翼（位置控制器）两种油门源的协调机制。

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
├─[849] quadplane.update()                    ← QuadPlane 主更新
│   │
│   ├─[1759] transition->update()             ← 固定翼模式/过渡阶段
│   │   │
│   │   ├─[1578] quadplane.hold_hover()       ← TRANSITION_AIRSPEED_WAIT
│   │   │   ├─[1100] multicopter_attitude_rate_update()  ← 姿态环
│   │   │   ├─[1104] set_climb_rate_cms()
│   │   │   └─[1106] run_z_controller()
│   │   │       └─ pos_control->update_z_controller() [AC_PosControl.cpp:989]
│   │   │           └─ set_throttle_out() → motors->set_throttle()  ← 仅写 motors._throttle
│   │   │
│   │   ├─[1645] quadplane.hold_stabilize()   ← TRANSITION_TIMER（混合油门，直接设置）
│   │   │   ├─[1012] multicopter_attitude_rate_update()  ← 姿态环
│   │   │   └─[1028] attitude_control->set_throttle_out()  ← 直接设置油门（不跑位置环）
│   │   │
│   │   └─[1668] quadplane.motors_output()    ← 过渡阶段电机混控输出
│   │
│   ├─[1767] motors_output()                  ← VTOL 模式
│   │   ├─[2001] attitude_control->rate_controller_run()  ← 角速度 PID → motors._roll/_pitch/_yaw
│   │   └─[2010] motors->output()
│   │       ├─ output_armed_stabilizing()     ← 混合矩阵 → _thrust_rpyt_out[]
│   │       ├─ thrust_compensation()          ← 回调 tilt_compensate()
│   │       └─ output_to_motors()             ← rc_write() → 仅写入 SRV_Channel（不是硬件输出）
│   │
│   └─[1780] tiltrotor.update()               ← 倾转舵机控制
│
├─[857] set_throttle()                        ← 仅处理 k_throttle（固定翼前推油门）
│   ├─ 电池电压补偿
│   ├─ 油门限制（min/max）
│   └─ 油门抑制
│
├─[883] throttle_slew_limit(k_throttle)       ← 仅限制 k_throttle 变化率
│
└─[929] servos_output()                       ← 唯一真正输出到硬件的地方
    ├─[1022] SRV_Channels::calc_pwm()         ← scaled → PWM
    └─[1024] SRV_Channels::output_ch_all()    ← hal.rcout->write()
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
// 段 A：先算油门（模式 run 或 transition）
hold_hover(target_climb_rate_cms) [quadplane.cpp:1092]
  ├─ multicopter_attitude_rate_update()     ← 姿态目标
  ├─ set_climb_rate_cms()
  └─ run_z_controller() [quadplane.cpp:1106]
      └─ pos_control->update_z_controller() [AC_PosControl.cpp:989]
          ├─ 位置环 / 速度环 / 加速度环
          └─ set_throttle_out() [AC_AttitudeControl_Multi.cpp:350]
              └─ motors->set_throttle()     ← 只写 motors._throttle

// 段 B：再混控并写 SRV（set_servos → quadplane.update）
motors_output() [quadplane.cpp:1767 / 1946]
  ├─ rate_controller_run()                  ← 角速度 PID
  └─ motors->output()
      ├─ output_armed_stabilizing()
      ├─ thrust_compensation() / tilt_compensate
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

### 2.3 过渡模式（关键！）

**调用链**：
```
set_servos() [servos.cpp:828]
  └─ quadplane.update() [servos.cpp:849]
      └─ transition->update() [quadplane.cpp:1759]
          │
          ├─ TRANSITION_AIRSPEED_WAIT:
          │   └─ hold_hover(climb_rate_cms) [quadplane.cpp:1578]
          │       └─ run_z_controller() → 位置环算油门
          │
          ├─ TRANSITION_TIMER: [quadplane.cpp:1636-1645]
          │   ├─ throttle_scaled = last_throttle × transition_scale
          │   ├─ 全倾转时：与 TECS 的 k_throttle 按 tilt ratio 混合
          │   └─ hold_stabilize(throttle_scaled) [quadplane.cpp:1645]
          │       ├─ multicopter_attitude_rate_update()  ← 姿态
          │       └─ set_throttle_out(throttle_scaled)   ← 直接设油门，不跑位置环
          │
          └─ motors_output() [quadplane.cpp:1668]
              └─ rate_controller_run() + motors->output() → 写 k_motorN
```

**核心代码**：
```cpp
// quadplane.cpp:1636-1642
if (tiltrotor.enabled() && !tiltrotor.has_vtol_motor() && !tiltrotor.has_fw_motor()) {
    // 全倾转构型：混合两个油门
    const float ratio = (current_tilt - airspeed_reached_tilt) 
                        / (get_fully_forward_tilt() - airspeed_reached_tilt);
    
    const float fw_throttle = MAX(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle), 0) * 0.01;
    
    // 混合公式
    throttle_scaled = constrain_float(
        throttle_scaled * (1.0 - ratio) + fw_throttle * ratio,
        0.0, 1.0
    );
}
```

**混合比例 `ratio` 的含义**：

| 时刻 | current_tilt | ratio | 油门来源 |
|------|-------------|-------|---------|
| 空速刚达标 | airspeed_reached_tilt | 0.0 | 100% 旋翼油门 |
| 过渡中间 | 50% 倾转 | 0.5 | 50% 旋翼 + 50% 固定翼 |
| 过渡完成 | fully_forward_tilt | 1.0 | 100% 固定翼油门 |

**物理含义**：
- 过渡早期：机翼升力不足，主要靠旋翼维持高度
- 过渡中期：机翼逐渐产生升力，旋翼逐步转为前飞推力
- 过渡晚期：机翼完全承担升力，旋翼完全提供前飞推力

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

#### 步骤 2：TRANSITION_TIMER 先混合，再 hold_stabilize

**完整调用链**（代码真实顺序：先混合后给定）：
```
主循环 50Hz
  └─ set_servos() [servos.cpp:828]
      └─ quadplane.update() [servos.cpp:849]
          └─ transition->update() [quadplane.cpp:1759]
              └─ case TRANSITION_TIMER:
                  ├─[1626] throttle_scaled = last_throttle × transition_scale
                  ├─[1636-1642] 全倾转时与 TECS k_throttle 混合
                  │   ratio = f(current_tilt)
                  │   throttle_scaled = vtol×(1-ratio) + fw×ratio
                  │   例：0.6×0.5 + 0.8×0.5 = 0.7
                  └─[1645] hold_stabilize(throttle_scaled) [quadplane.cpp:1009]
                      ├─[1012] multicopter_attitude_rate_update()  ← 姿态环
                      └─[1028] set_throttle_out(throttle_scaled)
                          └─ motors->set_throttle()  ← 直接设油门，不跑位置环
```

**混合核心代码**：
```cpp
// quadplane.cpp:1626-1645
float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
float throttle_scaled = last_throttle * transition_scale;

if (tiltrotor.enabled() && !has_vtol_motor() && !has_fw_motor()) {
    const float ratio = (current_tilt - airspeed_reached_tilt)
                        / (fully_forward_tilt - airspeed_reached_tilt);
    const float fw_throttle = MAX(SRV_Channels::get_output_scaled(k_throttle), 0) * 0.01;
    throttle_scaled = constrain_float(
        throttle_scaled * (1.0 - ratio) + fw_throttle * ratio, 0.0, 1.0);
}
hold_stabilize(throttle_scaled);
```

**hold_stabilize 核心代码**：
```cpp
// quadplane.cpp:1009-1030
void QuadPlane::hold_stabilize(float throttle_in)
{
    multicopter_attitude_rate_update(...);  // 姿态
    attitude_control->set_throttle_out(throttle_in, should_boost, 0); // 直接设油门
}
```

**与 `hold_hover()` 的区别**：

| 函数 | 调用场景 | 油门来源 | 是否跑位置环 |
|------|---------|---------|-------------|
| `hold_hover()` | AIRSPEED_WAIT / 纯悬停 | climb_rate → Z 位置环 | ✓ 跑 |
| `hold_stabilize()` | TRANSITION_TIMER | 外部算好的 throttle_scaled | ✗ 不跑 |

**物理含义**：
- `ratio`：0 = 纯旋翼，1 = 纯固定翼
- 混合后再直接给定油门，不经位置环
- 姿态环独立运行

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

**完整调用链**：
```
主循环 50Hz
  └─ set_servos() [servos.cpp:828]
      └─ quadplane.update() [servos.cpp:849]
          └─ motors_output() [quadplane.cpp:1767]
              └─ motors->output() [quadplane.cpp:2010]
                  ├─ output_armed_stabilizing() [AP_MotorsMulticopter.cpp:246]
                  └─ thrust_compensation() [AP_MotorsMulticopter.cpp:260]
                      └─ tilt_compensate() [tiltrotor.cpp:451]
                          ├─ 1/cos(45°) = 1.414
                          ├─ 电机推力 = 0.735 × 1.414 = 1.04
                          └─ 归一化到 1.0
```

**核心代码**：
```cpp
// tiltrotor.cpp:451-523
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    // 计算倾转因子
    const float tilt_factor = 1.0f / cosf(radians(current_tilt * 90.0f));
    
    // 对倾转电机应用补偿
    for (uint8_t i = 0; i < num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= tilt_factor;
        }
    }
    
    // 滚转效力衰减（倾转后差动推力效力下降）
    const float roll_factor = 1.0f - current_tilt;
    for (uint8_t i = 0; i < num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= roll_factor;
        }
    }
    
    // 偏航补偿（倾转后差动推力产生偏航力矩）
    if (is_vectored()) {
        const float yaw_gain = sinf(radians(current_tilt * 90.0f));
        for (uint8_t i = 0; i < num_motors; i++) {
            if (is_motor_tilting(i)) {
                const float diff_thrust = motors->get_yaw() * yaw_gain * motors->get_roll_factor(i);
                thrust[i] += diff_thrust;
            }
        }
    }
    
    // 归一化（防止超过 1.0）
    float max_thrust = 0.0f;
    for (uint8_t i = 0; i < num_motors; i++) {
        max_thrust = MAX(max_thrust, thrust[i]);
    }
    if (max_thrust > 1.0f) {
        const float scale = 1.0f / max_thrust;
        for (uint8_t i = 0; i < num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}
```

**物理含义**：
- `1/cos(θ)` 补偿：倾转后垂直升力分量减少，需要增加总推力
- 滚转效力衰减：倾转后差动推力对滚转的效力下降
- 偏航补偿：倾转后差动推力产生偏航力矩（矢量偏航）

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
          └─ motors_output() [quadplane.cpp:1668 或 1767]
              ├─ rate_controller_run() [quadplane.cpp:2001]
              │   └─ 角速度 PID → motors.set_roll/pitch/yaw()
              └─ motors->output() [quadplane.cpp:2010]
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

### 5.1 与 tiltrotor_motor_control.md 的对照验证

根据 `tiltrotor_motor_control.md` 第 3.3 节"前向过渡阶段"的描述：

| 文档描述 | 代码验证 | 一致性 |
|---------|---------|:------:|
| "电机部分倾转（0° < θ < 90°），同时承担旋翼和前推功能" | `continuous_update()` 中根据 `forward_throttle_pct()` 计算倾转角 | ✓ |
| "混合两个油门源：旋翼（位置控制器）+ 固定翼（TECS）" | `quadplane.cpp:1636-1642` 中的混合公式 | ✓ |
| "输出路径：`motors->output()` 经过混合矩阵 + `tilt_compensate`" | `motors_output()` → `motors->output()` → `output_armed_stabilizing()` → `thrust_compensation()` | ✓ |
| "混合比例 `ratio` 基于倾转角" | `ratio = (current_tilt - airspeed_reached_tilt) / (fully_forward - airspeed_reached_tilt)` | ✓ |
| "过渡早期：机翼升力不足，主要靠旋翼维持高度" | `ratio ≈ 0` 时，`throttle_scaled ≈ vtol_throttle` | ✓ |
| "过渡晚期：机翼完全承担升力，旋翼完全提供前飞推力" | `ratio ≈ 1` 时，`throttle_scaled ≈ fw_throttle` | ✓ |

**结论**：`throttle_calculation.md` 的数据流示例与 `tiltrotor_motor_control.md` 的描述完全一致，核心代码实现也验证了数据流的正确性。

## 6. 关键代码位置索引

| 功能 | 文件 | 行号 | 函数 |
|------|------|------|------|
| 主循环调度表 | ArduPlane.cpp | 59-61 | `sched_tasks[]` |
| 油门主入口 | servos.cpp | 828 | `Plane::set_servos()` |
| QuadPlane 更新 | servos.cpp | 849 | `quadplane.update()` |
| 过渡状态机 | quadplane.cpp | 1759 | `transition->update()` |
| 过渡油门混合 | quadplane.cpp | 1636-1642 | `SLT_Transition::update()` |
| 旋翼悬停控制 | quadplane.cpp | 1092 | `QuadPlane::hold_hover()` |
| 旋翼稳定控制 | quadplane.cpp | 1009 | `QuadPlane::hold_stabilize()` |
| Z 轴控制器 | quadplane.cpp | 1033 | `QuadPlane::run_z_controller()` |
| 位置控制器 | AC_PosControl.cpp | 989 | `AC_PosControl::update_z_controller()` |
| 电机输出 | quadplane.cpp | 1946 | `QuadPlane::motors_output()` |
| 倾转补偿 | tiltrotor.cpp | 451 | `Tiltrotor::tilt_compensate()` |
| 油门后处理 | servos.cpp | 857 | `Plane::set_throttle()` |
| 油门变化率限制 | servos.cpp | 883 | `Plane::throttle_slew_limit()` |
| 油门抑制判断 | servos.cpp | 73 | `Plane::suppress_throttle()` |
| PWM 输出 | servos.cpp | 929 | `Plane::servos_output()` |
| TECS 控制器 | AP_TECS.cpp | 759 | `AP_TECS::update_pitch_throttle()` |

## 7. 总结

### 7.1 油门来源的三种模式

| 模式 | 油门来源 | 控制目标 | 代码路径 |
|------|---------|---------|---------|
| 纯固定翼 | TECS | 空速 + 高度 | `TECS → SRV_Channel::k_throttle` |
| 纯旋翼 | 位置控制器 | 高度 + 爬升率 | `hold_hover → Z Controller → motors->set_throttle → motors->output → servos_output` |
| 过渡 TIMER | 混合后直接给定 | 空速 + 姿态稳定 | `hold_stabilize(mixed) → set_throttle_out → motors->output → servos_output` |

### 7.2 油门后处理流程

```
原始油门（来自 TECS 或位置控制器）
  ↓
电池电压补偿（电压下降时增加油门）
  ↓
油门限制（min/max 限制）
  ↓
油门抑制（着陆、安全等场景强制归零）
  ↓
油门变化率限制（防止突变）
  ↓
倾转补偿（1/cos(θ) 补偿垂直升力损失）
  ↓
PWM 输出（转换为硬件信号）
```

### 7.3 关键设计思想

1. **统一接口**：无论固定翼还是旋翼，最终都通过 `SRV_Channel::k_throttle` 或 `motors->set_throttle()` 设置油门
2. **平滑过渡**：通过 `ratio` 参数实现油门源的平滑切换，避免突变
3. **多重保护**：电池补偿、油门限制、变化率限制、油门抑制等多层保护机制
4. **物理分解**：通过 `1/cos(θ)` 补偿，确保倾转过程中垂直升力不丢失
