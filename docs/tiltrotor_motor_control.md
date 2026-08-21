# 倾转旋翼电机控制逻辑详解

> 适用分支：`qhz/plane_4.6.3`
> 核心文件：`ArduPlane/tiltrotor.cpp`、`ArduPlane/quadplane.cpp`、`libraries/AP_Motors/AP_MotorsMulticopter.cpp`

---

## 1. 配置与电机识别

### 1.1 配置示例（quadplane-tilthvec.parm）

```ini
Q_ENABLE         1
Q_FRAME_CLASS    1        # 四轴
Q_FRAME_TYPE     3        # X 型
Q_TILT_TYPE      2        # VectoredYaw（矢量偏航）
Q_TILT_YAW_ANGLE 10       # 矢量偏航角度
Q_TILT_MASK      15       # 电机 0-3 都是倾转电机（二进制 1111）
Q_VFWD_GAIN      0.1      # 前向推力增益
# 没有配置 k_throttle 通道 → 没有独立的固定翼前推电机
```

### 1.2 倾转电机识别

```cpp
// tiltrotor.h
bool is_motor_tilting(uint8_t motor) const {
    return tilt_mask.get() & (1U << motor);
}
```

### 1.3 构型识别

```cpp
// tiltrotor.cpp:113-127 — Tiltrotor::setup()
_have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) || ...;
// 遍历所有电机：不在 tilt_mask 中的 enabled 电机 → _have_vtol_motor = true
```

| 构型 | `_have_fw_motor` | `_have_vtol_motor` | 说明 |
|------|:-:|:-:|------|
| 全倾转 | false | false | 所有电机都在 tilt_mask 中（tilthvec 示例） |
| 独立前推电机 | true | — | 有 `k_throttle` 通道 |
| 混合 | false | true | 有不在 tilt_mask 中的永久 VTOL 电机 |

---

## 2. QuadPlane::update() 总控逻辑

入口在 `Plane::set_servos()` → `quadplane.update()`（`quadplane.cpp:1698`）。

```
QuadPlane::update()
  │
  ├─ !in_vtol_mode() && !in_vtol_airbrake()    ← 固定翼模式
  │    ├─ 手动模式（MANUAL/ACRO/TRAINING）:
  │    │    SHUT_DOWN + motors->output() + force_transition_complete()
  │    └─ 自动模式（FBWA/AUTO/...）:
  │         transition->update()                ← 过渡状态机
  │
  ├─ in_vtol_mode() || in_vtol_airbrake()      ← VTOL 模式
  │    ├─ motors_output()                       ← 姿态控制 + 混合矩阵输出
  │    └─ transition->VTOL_update()             ← 重置过渡状态
  │
  └─ tiltrotor.update()                        ← 所有模式都执行（倾转舵机 + 固定翼油门输出）
```

### 2.1 三个关键函数

| 函数 | 调用时机 | 主要作用 |
|------|---------|---------|
| `transition->update()` | 固定翼自动模式 | 管理过渡状态机，判断是否需要 VTOL 辅助，过渡阶段混合油门 |
| `motors_output()` | VTOL 模式 | 运行姿态控制器 + `motors->output()`（混合矩阵 + 倾转补偿） |
| `tiltrotor.update()` | 所有模式 | 控制倾转舵机角度；纯固定翼模式下直接输出前推油门 |

### 2.2 `in_vtol_mode()` 判定（`quadplane.cpp:2100`）

- **返回 true**：QHOVER/QLOITER/QLAND/QRTL 等纯 VTOL 模式；自动模式中的 VTOL 着陆序列
- **返回 false**：FBWA/AUTO/CRUISE 等固定翼模式；过渡阶段（前向过渡时飞行模式已切到固定翼）

---

## 3. 三种飞行模式的电机控制

### 3.1 纯固定翼模式

**条件**：`!in_vtol_mode() && !assisted_flight`

**执行路径**：
```
transition->update() → TRANSITION_DONE:
  set_desired_spool_state(SHUT_DOWN) + motors->output()   ← spool 状态管理

tiltrotor.update() → continuous_update():
  slew(get_forward_flight_tilt())                          ← 电机倾转到水平
  motors->output_motor_mask(current_throttle, tilt_mask)   ← 直接输出前推油门
```

**关键点**：
- 电机完全水平（90°），只作为前推电机
- 油门来源：TECS（`k_throttle` 通道）
- 输出路径：`output_motor_mask()` **绕过混合矩阵**，直接 `rc_write()` 写 PWM
- `TRANSITION_DONE` 中的 `motors->output()` 目的是驱动 spool 状态机迁移到 SHUT_DOWN，不是输出推力（详见第 5 节）

### 3.2 纯 VTOL 模式

**条件**：`in_vtol_mode() && !assisted_flight`

**执行路径**：
```
motors_output():
  attitude_control->rate_controller_run()   ← 角速率 PID
  motors->output()                          ← 混合矩阵 + tilt_compensate + PWM

tiltrotor.update() → continuous_update():
  根据前向推力需求计算倾转角 → slew(current_tilt)   ← 仅控制舵机角度
```

**关键点**：
- 电机垂直（0°）或有小角度倾转（根据 Q_FWD_THR_GAIN），只作为旋翼电机
- 油门来源：位置控制器（Z 轴 PID）
- 输出路径：`motors->output()` **经过混合矩阵** + `tilt_compensate` 倾转补偿
- `tiltrotor.update()` 在此模式下只控制倾转舵机角度（`k_motor_tilt` 通道），不参与电机输出

### 3.3 前向过渡阶段

**条件**：`!in_vtol_mode() && assisted_flight`（过渡时 `in_vtol_mode()` 返回 false）

**执行路径**：
```
transition->update():
  TRANSITION_AIRSPEED_WAIT → hold_hover()          ← 等待空速达标
  TRANSITION_TIMER         → hold_stabilize(mixed)  ← 混合油门，逐步衰减 VTOL 推力

tiltrotor.update() → continuous_update():
  assisted_flight && transition_state >= TRANSITION_TIMER:
    slew(get_forward_flight_tilt())                  ← 电机逐步前倾
```

**TRANSITION_TIMER 阶段的油门混合**（全倾转构型，`quadplane.cpp:1636-1642`）：
```cpp
const float ratio = (current_tilt - airspeed_reached_tilt)
                    / (get_fully_forward_tilt() - airspeed_reached_tilt);
const float fw_throttle = MAX(SRV_Channels::get_output_scaled(k_throttle), 0) * 0.01;
throttle_scaled = constrain_float(
    throttle_scaled * (1.0 - ratio) + fw_throttle * ratio, 0.0, 1.0);
```

| 时刻 | current_tilt | ratio | 油门来源 |
|------|:-----------:|:-----:|---------|
| 空速刚达标 | airspeed_reached_tilt | 0.0 | 100% 旋翼 |
| 过渡中间 | 50% | 0.5 | 50% 旋翼 + 50% 固定翼 |
| 过渡完成 | fully_forward | 1.0 | 100% 固定翼 |

**关键点**：
- 电机部分倾转（0° < θ < 90°），同时承担旋翼和前推功能
- 混合两个油门源：旋翼（位置控制器）+ 固定翼（TECS）
- 输出路径：`motors->output()` 经过混合矩阵 + `tilt_compensate`

### 3.4 VTOL 着陆反向过渡（AIRBRAKE → POSITION1）

VTOL 着陆时，飞机需要从固定翼模式反向过渡到悬停模式。这个过程分为两个阶段。

#### 3.4.0 完整调用路径总览

主循环中三个 FAST_TASK 按顺序执行：

```
主循环（每周期）:

1. update_control_mode()  [ArduPlane.cpp:500]
   └─ control_mode->update()
      ├─ ModeAuto::update()  [mode_auto.cpp:77]
      │   └─ quadplane.control_auto()  ← 如果 in_vtol_auto()
      │      └─ vtol_position_controller()  [quadplane.cpp:2323]
      │         └─ 包含 QPOS_AIRBRAKE / QPOS_POSITION1 逻辑
      │
      └─ ModeQRTL::update()  [mode_qrtl.cpp:160]
          └─ quadplane.vtol_position_controller()

2. stabilize()  [ArduPlane.cpp:60]

3. set_servos()  [ArduPlane.cpp:61]
   └─ QuadPlane::update()  [quadplane.cpp:1698]
      │
      ├─ if (!in_vtol_mode() && !in_vtol_airbrake())
      │    └─ transition->update()  ← 固定翼过渡逻辑
      │
      └─ else  ← in_vtol_mode() || in_vtol_airbrake()
           ├─ assisted_flight = in_vtol_airbrake()
           ├─ motors_output()     ← 电机输出
           └─ transition->VTOL_update()  ← 重置过渡状态
      
      └─ tiltrotor.update()  ← 所有模式都执行
           └─ continuous_update()  ← 倾转控制
```

**关键判断函数**：
- `in_vtol_mode()` [quadplane.cpp:2100]：VTOL 着陆序列中，`poscontrol.state > QPOS_AIRBRAKE` 时返回 true
- `in_vtol_airbrake()` [quadplane.cpp:4208]：QRTL 或 AUTO(VTOL_LAND) 且 `poscontrol.state == QPOS_AIRBRAKE` 时返回 true

**注意**：`vtol_position_controller()` 在 `update_control_mode()` 中调用，**在 `set_servos()` 之前执行**。它负责设置 `poscontrol.state` 和控制逻辑，而 `set_servos()` 中的 `quadplane.update()` 负责根据状态执行电机输出。

#### 3.4.1 QPOS_AIRBRAKE 阶段（空气制动）

**触发条件**：
- 飞行模式为 QRTL 或 AUTO（VTOL 着陆航点）
- 距离目标小于 `stop_distance`（停止距离 + 2秒余量）
- `in_vtol_mode()` 返回 **false**，`in_vtol_airbrake()` 返回 **true**

**完整调用路径（按主循环顺序）**：

```
主循环周期 1: update_control_mode()
  └─ ModeAuto::update() / ModeQRTL::update()
     └─ vtol_position_controller()  [quadplane.cpp:2323]
        └─ case QPOS_AIRBRAKE:
           ├─ 固定翼舵面控制：
           │   ├─ SRV_Channels::set_output_scaled(k_throttle, TECS油门)
           │   ├─ nav_pitch_cd = TECS俯仰指令（限制 ≥ 0）
           │   └─ calc_nav_roll()  ← L1导航计算滚转
           │
           ├─ 旋翼控制（如果电机完全前倾）：
           │   └─ hold_stabilize(0.01)  [quadplane.cpp:1009]
           │       ├─ multicopter_attitude_rate_update()  ← 设置期望姿态
           │       │   └─ attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
           │       │       nav_roll_cd, nav_pitch_cd, yaw_rate)
           │       │       ← 计算 roll/pitch/yaw 期望角速度，存入姿态控制器
           │       │
           │       └─ attitude_control->set_throttle_out(0.01, ...)  [AC_AttitudeControl_Multi.cpp:350]
           │           └─ motors->set_throttle(0.01)  ← 设置油门值到 motors._throttle
           │
           └─ 退出条件检测（空速/速度/姿态误差）

主循环周期 2: stabilize()
  └─ 固定翼姿态稳定（使用上面设置的 nav_roll/pitch）

主循环周期 3: set_servos()
  └─ QuadPlane::update()  [quadplane.cpp:1698]
     ├─ !in_vtol_mode() && in_vtol_airbrake() → else 分支
     │   ├─ assisted_flight = true
     │   ├─ motors_output()  [quadplane.cpp:1946]
     │   │   ├─ attitude_control->rate_controller_run()  [quadplane.cpp:2001]
     │   │   │   └─ 执行角速度 PID：
     │   │   │       roll_out  = PID(期望roll_rate  - 实际gyro.x)
     │   │   │       pitch_out = PID(期望pitch_rate - 实际gyro.y)
     │   │   │       yaw_out   = PID(期望yaw_rate   - 实际gyro.z)
     │   │   │       → motors->set_roll/pitch/yaw(roll_out, pitch_out, yaw_out)
     │   │   │
     │   │   └─ motors->output()  [AP_MotorsMulticopter.cpp:246]
     │   │       ├─ output_armed_stabilizing()  ← 混合矩阵：
     │   │       │   thrust[i] = throttle + roll×roll_factor[i] 
     │   │       │                      + pitch×pitch_factor[i] + yaw×yaw_factor[i]
     │   │       ├─ thrust_compensation() → tilt_compensate()  ← 倾转补偿
     │   │       └─ output_to_motors()  ← 转换为 PWM 输出
     │   │
     │   └─ transition->VTOL_update()
     │
     └─ tiltrotor.update()  [quadplane.cpp:1780]
         └─ continuous_update()  [tiltrotor.cpp:214]
            └─ !in_vtol_mode() && assisted_flight = true
               → slew() 根据油门设置倾转角（受 Q_TILT_MAX 限制）
```

**关键说明**：

1. **`hold_stabilize(0.01)` 的作用**（在 `update_control_mode()` 阶段调用）：
   - 调用 `multicopter_attitude_rate_update()`：将 `nav_roll_cd`、`nav_pitch_cd`、`yaw_rate` 转换为期望角速度，存入姿态控制器
   - 调用 `set_throttle_out(0.01)`：将 1% 油门存入 `motors._throttle`
   - **此时还没有执行 PID 计算和电机混合**，只是设置了期望值和油门

2. **`motors_output()` 的作用**（在 `set_servos()` 阶段调用）：
   - 调用 `rate_controller_run()`：执行角速度 PID，将期望角速度与实际陀螺仪数据比较，输出 roll/pitch/yaw 控制量
   - 调用 `motors->output()`：执行混合矩阵，将 throttle + roll + pitch + yaw 分配到每个电机，经过倾转补偿后输出 PWM

3. **两个阶段的分工**：
   - `update_control_mode()`：设置"想要什么"（期望姿态、期望油门）
   - `set_servos()`：计算"怎么做到"（PID 计算、电机混合、PWM 输出）

**倾转策略**：
- 电机从水平位置**反向倾转**（向前倾转），利用旋翼产生阻力减速
- 倾转角受 `Q_TILT_MAX` 限制（默认 45°），防止过度抬头
- 倾转角由 TECS 油门决定：`settilt = (k_throttle - throttle_min) * 0.02`

**退出条件**（在 `vtol_position_controller()` 中检测）：
```cpp
// quadplane.cpp:2466-2479
if (poscontrol.get_state() == QPOS_AIRBRAKE &&
    poscontrol.time_since_state_start_ms() > 1000 &&  // 至少 1 秒
    (aspeed < airspeed_min - 2 ||                     // 空速过低
     fabsf(closing_vel.angle(desired_closing_vel)) > 60 ||  // 方向偏差 > 60°
     closing_speed > 1.2 * desired_closing_speed ||   // 速度过快
     closing_speed < 0.5 * desired_closing_speed ||   // 速度过慢
     abs(roll_sensor - nav_roll_cd) > 1000 ||         // 滚转误差 > 10°
     abs(pitch_sensor - nav_pitch_cd) > 1000)) {      // 俯仰误差 > 10°
    poscontrol.set_state(QPOS_POSITION1);
}
```

#### 3.4.2 QPOS_POSITION1 阶段（位置控制减速）

**触发条件**：
- 从 AIRBRAKE 退出（满足上述退出条件）
- 或距离目标较近直接跳过 AIRBRAKE
- `in_vtol_mode()` 返回 **true**（`poscontrol.state > QPOS_AIRBRAKE`）

**完整调用路径（按主循环顺序）**：

```
主循环周期 1: update_control_mode()
  └─ ModeAuto::update() / ModeQRTL::update()
     └─ vtol_position_controller()  [quadplane.cpp:2323]
        └─ case QPOS_POSITION1:  [quadplane.cpp:2538]
           ├─ 位置控制器计算目标速度和加速度：
           │   ├─ stopping_speed = sqrt(2 × transition_decel × distance)
           │   ├─ target_speed = MIN(pos1_speed_limit, stopping_speed)
           │   └─ target_accel = MIN(accel_needed, transition_decel × 2)
           │
           ├─ 位置控制器输出期望姿态：
           │   ├─ pos_control->input_vel_accel_xy(target_speed, target_accel)
           │   ├─ run_xy_controller()  ← 执行 XY 位置 PID
           │   ├─ nav_roll_cd = pos_control->get_roll_cd()
           │   └─ nav_pitch_cd = pos_control->get_pitch_cd()
           │
           ├─ 姿态控制器设置期望值：
           │   └─ attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
           │       nav_roll_cd, nav_pitch_cd, yaw_rate)
           │       ← 计算 roll/pitch/yaw 期望角速度，存入姿态控制器
           │
           └─ 退出条件检测（距离 < 10m 且速度 < 9m/s）

主循环周期 2: stabilize()
  └─ 固定翼姿态稳定（使用上面设置的 nav_roll/pitch）

主循环周期 3: set_servos()
  └─ QuadPlane::update()  [quadplane.cpp:1698]
     ├─ in_vtol_mode() = true → else 分支
     │   ├─ assisted_flight = false  ← 注意：airbrake 结束
     │   ├─ motors_output()  [quadplane.cpp:1946]
     │   │   ├─ attitude_control->rate_controller_run()  [quadplane.cpp:2001]
     │   │   │   └─ 执行角速度 PID：
     │   │   │       roll_out  = PID(期望roll_rate  - 实际gyro.x)
     │   │   │       pitch_out = PID(期望pitch_rate - 实际gyro.y)
     │   │   │       yaw_out   = PID(期望yaw_rate   - 实际gyro.z)
     │   │   │       → motors->set_roll/pitch/yaw(roll_out, pitch_out, yaw_out)
     │   │   │
     │   │   └─ motors->output()  [AP_MotorsMulticopter.cpp:246]
     │   │       ├─ output_armed_stabilizing()  ← 混合矩阵：
     │   │       │   thrust[i] = throttle + roll×roll_factor[i] 
     │   │       │                      + pitch×pitch_factor[i] + yaw×yaw_factor[i]
     │   │       ├─ thrust_compensation() → tilt_compensate()  ← 倾转补偿
     │   │       └─ output_to_motors()  ← 转换为 PWM 输出
     │   │
     │   └─ transition->VTOL_update()
     │
     └─ tiltrotor.update()  [quadplane.cpp:1780]
         └─ continuous_update()  [tiltrotor.cpp:214]
            └─ in_vtol_mode() = true
               → 进入 VTOL 分支（非纯固定翼）
               → 根据前向推力需求计算倾转角

**关键说明**：

1. **位置控制器的作用**（在 `update_control_mode()` 阶段调用）：
   - 计算目标速度剖面（基于距离和减速度）
   - 执行 XY 位置 PID，输出期望滚转和俯仰角
   - 调用姿态控制器设置期望姿态（但不执行 PID 计算）

2. **`motors_output()` 的作用**（在 `set_servos()` 阶段调用）：
   - 调用 `rate_controller_run()`：执行角速度 PID，将期望角速度与实际陀螺仪数据比较
   - 调用 `motors->output()`：执行混合矩阵，将 throttle + roll + pitch + yaw 分配到每个电机
   - 经过倾转补偿后输出 PWM

3. **与 AIRBRAKE 阶段的区别**：
   - AIRBRAKE：使用 TECS 控制俯仰（限制 ≥ 0），L1 控制滚转，电机前倾产生阻力
   - POSITION1：使用位置控制器控制滚转和俯仰，电机逐步回到垂直，准备悬停

**倾转策略**：
- 电机从 AIRBRAKE 的前倾位置**逐步回到垂直**（准备悬停）
- `continuous_update()` 中：`in_vtol_mode() = true`，走 VTOL 分支
- 倾转角由前向推力需求决定：`fwd_g_demand = 0.01 * forward_throttle_pct()`

**退出条件**：
```cpp
// quadplane.cpp:2689-2692
if ((plane.auto_state.wp_distance < position2_dist_threshold) && 
    tiltrotor.tilt_angle_achieved() &&
    fabsf(rel_groundspeed_sq) < sq(3*position2_target_speed)) {
    poscontrol.set_state(QPOS_POSITION2);  // 进入最终着陆阶段
}
```

#### 3.4.3 AIRBRAKE vs POSITION1 对比

| 特性 | QPOS_AIRBRAKE | QPOS_POSITION1 |
|------|---------------|----------------|
| `in_vtol_mode()` | **false** | **true** |
| `in_vtol_airbrake()` | **true** | **false** |
| `assisted_flight` | **true** | **false** |
| 控制分支 | else 分支（airbrake） | else 分支（VTOL） |
| 电机输出路径 | `motors_output()` + `hold_stabilize(0.01)` | `motors_output()` + 位置控制器 |
| 倾转策略 | 向前倾转（产生阻力） | 逐步回到垂直（准备悬停） |
| 俯仰控制 | TECS（限制 ≥ 0） | 位置控制器 |
| 滚转控制 | L1 导航控制器 | 位置控制器 |
| 主要目的 | 空气制动减速 | 精确位置控制减速 |

---

## 4. 总结对比

### 4.1 同一个电机的三种角色

| 飞行阶段 | 电机角色 | 油门来源 | 输出路径 | 倾转角度 |
|---------|---------|---------|---------|:-------:|
| 纯固定翼 | 前推电机 | TECS（k_throttle） | `output_motor_mask`（绕过混合矩阵） | 水平（90°） |
| 过渡阶段 | 旋翼+前推 | 混合（旋翼+TECS） | `motors->output()`（经过混合矩阵） | 部分倾转 |
| 纯 VTOL | 旋翼电机 | 位置控制器 | `motors->output()`（经过混合矩阵） | 垂直（0°） |

### 4.2 控制路径选择逻辑

```cpp
if (!in_vtol_mode() && !assisted_flight) {
    // 纯固定翼：output_motor_mask（直接输出，绕过混合矩阵）
} else if (in_vtol_mode() && !assisted_flight) {
    // 纯 VTOL：motors->output()（经过混合矩阵 + tilt_compensate）
} else {
    // 过渡：混合油门 + motors->output()（经过混合矩阵 + tilt_compensate）
}
```

### 4.3 关键设计思想

1. **统一硬件，软件切换**：同一个电机通过软件逻辑在不同阶段扮演不同角色
2. **双油门源**：TECS（固定翼）和位置控制器（旋翼）分别计算油门
3. **平滑过渡**：通过 `ratio` 参数（基于倾转角）实现油门源的平滑切换
4. **物理分解**：通过倾转角和 `tilt_compensate`（`1/cos(θ)` 补偿）实现升力和推力的物理分解
5. **灵活配置**：支持全倾转、部分倾转、独立前推电机等多种构型

---

## 5. TRANSITION_DONE 与 output_motor_mask 的关系（常见疑问）

### 5.1 问题

纯固定翼模式下，`TRANSITION_DONE` 调了 `motors->output()`，`continuous_update()` 又调了 `output_motor_mask()`，是否重复？

### 5.2 执行时序

```
QuadPlane::update()
  ├─ transition->update()
  │    case TRANSITION_DONE:  (quadplane.cpp:1660)
  │      set_desired_spool_state(SHUT_DOWN)
  │      motors->output()          ← spool 状态管理，SHUT_DOWN 下输出最低值
  │      return;                   ← 不到达 motors_output()
  │
  └─ tiltrotor.update()
       └─ continuous_update()  (tiltrotor.cpp:222)
            slew(get_forward_flight_tilt())
            output_motor_mask(current_throttle, tilt_mask)  ← 覆盖倾转电机的 PWM
```

### 5.3 职责对比

| 操作 | 作用对象 | 实际效果 |
|------|---------|---------|
| TRANSITION_DONE 的 `motors->output()` | 所有电机 | 驱动 spool 状态机迁移到 SHUT_DOWN；关闭非倾转电机 |
| `continuous_update()` 的 `output_motor_mask()` | 仅倾转电机 | 写入 TECS 前推油门，覆盖上一步对倾转电机写的最低值 |

- **全倾转构型**：`output_motor_mask()` 覆盖所有电机 → PWM 层面看似"重复"，但 `motors->output()` 的真正目的是驱动 spool 状态机
- **混合构型**：两者各管各的电机，完全不重复

**结论**：互补关系。前者管状态机，后者管实际推力输出。

---

## 6. output_motor_mask 实现

```cpp
// AP_MotorsMulticopter.cpp:743
void AP_MotorsMulticopter::output_motor_mask(float thrust, uint16_t mask, float rudder_dt)
{
    _motor_mask_override = mask;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && (mask & (1U << i)) != 0) {
            if (armed() && get_interlock()) {
                float diff_thrust = get_roll_factor(i) * rudder_dt * 0.5f;
                set_actuator_with_slew(_actuator[i], thrust + diff_thrust);
            } else {
                _actuator[i] = 0.0;
            }
            rc_write(i, pwm_min + pwm_range * _actuator[i]);  // 直接输出 PWM
        }
    }
}
```

**特点**：
- 直接输出到电机，**不经过混合矩阵**，不检查 spool state
- 只输出 `mask` 中指定的电机
- 包含方向舵差动推力（`rudder_dt`）
- 通过 `rc_write()` → `SRV_Channel` → 最终到 PWM 或 DroneCAN

---

## 7. 输出路径总览

```
rc_write(chan, pwm)
  → SRV_Channels::set_output_pwm(motor_function, pwm)
    → SRV_Channel 系统
      ├─ hal.rcout->write(pwm)           → 硬件 PWM 信号 → ESC
      └─ AP_DroneCAN::SRV_push_servos()  → CAN RawCommand  → ESC
```

所有电机输出（无论 `motors->output()` 还是 `output_motor_mask()`）最终都通过 `SRV_Channel` 系统，PWM 和 DroneCAN 是两种并行的硬件输出方式。

---

## 8. 差动推力控制（Differential Thrust）

差动推力是指通过左右电机的推力差来实现姿态控制。在倾转旋翼中，差动推力在不同飞行阶段扮演不同角色。

### 8.1 三种差动推力机制

| 机制 | 代码位置 | 作用轴 | 生效阶段 |
|------|---------|--------|---------|
| 反扭矩偏航 | 混合矩阵 `_yaw_factor[i]` | 偏航 | VTOL + 过渡 |
| 差动滚转（固定翼） | `output_motor_mask()` 中的 `rudder_dt` | 滚转（固定翼坐标系下的偏航） | 纯固定翼 |
| 差动偏航（倾转补偿） | `tilt_compensate_angle()` 中的 `diff_thrust` | 偏航 | 过渡 |

### 8.2 纯 VTOL 模式（电机垂直，θ = 0°）

**偏航控制：反扭矩差动**

电机垂直时，CW 和 CCW 电机的反扭矩差产生偏航力矩。这由混合矩阵中的 `_yaw_factor[i]` 处理：

```
电机垂直时：
  CW 电机加速 + CCW 电机减速 → 反扭矩差 → 偏航力矩
  偏航效力 ∝ cos(0°) = 100%
```

**滚转控制：旋翼差动**

左右电机的推力差通过力臂产生滚转力矩：

```
  左侧电机加速 + 右侧电机减速 → 滚转力矩
  滚转效力 ∝ cos(0°) = 100%
```

**此时没有差动推力用于固定翼滚转**，因为电机垂直，水平分量为零。

### 8.3 过渡阶段（电机部分倾转，0° < θ < 90°）

过渡阶段存在**两种偏航机制的交接**：

**机制 A：反扭矩偏航（逐渐减弱）**

```
电机倾转 θ 角后：
  反扭矩偏航效力 ∝ cos(θ)
  θ = 0° → 100%
  θ = 45° → 71%
  θ = 90° → 0%（完全失效）
```

**机制 B：差动推力偏航（逐渐增强）**

在 `tilt_compensate_angle()` 中（`tiltrotor.cpp:484-486`）：

```cpp
// 差动推力用于偏航控制，按倾转角 sin(θ) 缩放
const float diff_thrust = motors->get_roll_factor(i)    // 左右位置（+0.5 或 -0.5）
                         * (motors->get_yaw() + motors->get_yaw_ff())  // 偏航需求
                         * sin_tilt                       // sin(θ)，水平分量比例
                         * yaw_gain;                      // sin(tilt_yaw_angle)，矢量偏航增益
thrust[i] += diff_thrust;
```

```
电机倾转 θ 角后：
  差动推力偏航效力 ∝ sin(θ)
  θ = 0° → 0%（无水平分量）
  θ = 45° → 71%
  θ = 90° → 100%（完全水平）
```

**两种机制互补**：

| 倾转角 | 反扭矩偏航（机制 A） | 差动推力偏航（机制 B） | 总偏航能力 |
|:------:|:-------------------:|:---------------------:|:---------:|
| 0°（垂直） | 100% | 0% | 正常 |
| 45° | 71% | 71% | 充足 |
| 90°（水平） | 0% | 100% | 正常 |

**滚转效力的衰减**（`tiltrotor.cpp:483`）：

```cpp
// 随倾转角衰减滚转控制效力
thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1 - current_tilt);
```

```
电机倾转 θ 角后：
  滚转差动效力 ∝ cos(θ)（近似为 1 - current_tilt）
  θ = 0° → 100%
  θ = 90° → 0%（差动推力变为纯前后，无力臂产生滚转力矩）
```

### 8.4 纯固定翼模式（电机水平，θ = 90°）

电机完全水平后，走 `output_motor_mask()` 路径（`tiltrotor.cpp:252`）：

```cpp
motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
```

**差动推力用于滚转控制**（`AP_MotorsMulticopter.cpp:760`）：

```cpp
// 方向舵差动推力
// 注意：copter 坐标系的 roll = plane 坐标系的 yaw
float diff_thrust = get_roll_factor(i) * rudder_dt * 0.5f;
set_actuator_with_slew(_actuator[i], thrust + diff_thrust);
```

```
电机水平时：
  左侧电机加速 + 右侧电机减速 → 水平推力差 → 偏航力矩（固定翼坐标系）
  这就是固定翼的方向舵差动效果
```

**此时反扭矩偏航完全失效**（cos(90°) = 0），偏航控制完全依赖：
1. 差动推力（通过 `rudder_dt`）
2. 固定翼舵面（方向舵、升降舵等）

### 8.5 各阶段差动控制总结

| 飞行阶段 | 偏航控制 | 滚转控制 | 差动推力角色 |
|---------|---------|---------|-------------|
| 纯 VTOL（θ=0°） | 反扭矩差动（100%） | 旋翼差动（100%） | 无 |
| 过渡（0°<θ<90°） | 反扭矩（cosθ）+ 差动推力（sinθ） | 旋翼差动（cosθ，衰减中） | 补充偏航 |
| 纯固定翼（θ=90°） | 差动推力（rudder_dt）+ 固定翼舵面 | 固定翼副翼 | 替代方向舵 |

### 8.6 为什么用 sin(θ) 而不是 1/sin(θ)

如果用除法精确补偿：
```
θ = 5° → sin(5°) = 0.087
diff_thrust = yaw_demand / 0.087 = 11.5 × yaw_demand  ← 远超电机能力
θ = 0° → sin(0°) = 0 → 除零错误
```

用乘法的设计：
```
diff_thrust = yaw_demand × sin(θ) × yaw_gain
偏航力矩 = diff_thrust × sin(θ) × d = yaw_demand × sin²(θ) × d
总偏航 = 反扭矩（∝ cosθ）+ 差动推力（∝ sin²θ）
→ 在整个角度范围内平滑过渡，无奇点
```

### 8.7 tilt_yaw_angle 参数的作用

`Q_TILT_YAW_ANGLE` 参数（`tiltrotor.cpp:475`）：

```cpp
const float yaw_gain = sinf(radians(tilt_yaw_angle));
```

- 定义了矢量偏航机构在最小倾转角时的偏航增益
- `tilt_yaw_angle = 0` → `yaw_gain = 0` → 不应用差动推力偏航
- `tilt_yaw_angle = 10°` → `yaw_gain = 0.174` → 适度差动推力偏航
- 值越大，差动推力偏航效果越强
