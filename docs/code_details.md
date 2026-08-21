# ArduPilot Plane 代码细节

## 1. use_fw_attitude_controllers() 判断逻辑

**位置**: `ArduPlane/quadplane.cpp:4250-4282`

### 函数实现

```cpp
bool QuadPlane::use_fw_attitude_controllers(void) const
{
    if (available() &&
        motors->armed() &&
        motors->get_desired_spool_state() >= AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED &&
        !tailsitter.enabled() &&
        poscontrol.get_state() != QPOS_AIRBRAKE &&
        !force_fw_control_recovery) {

        if (in_vtol_mode()) {
            return false;  // VTOL 模式下始终使用 VTOL 控制器
        }

        if (transition->use_multirotor_control_in_fwd_transition()) {
            return false;  // 矢量偏航倾转旋翼前向过渡中
        }
    }

    if (force_fw_control_recovery) {
        plane.rollController.set_in_recovery();
    }

    return true;
}
```

### 返回 false（使用 VTOL 姿态控制器）的条件

必须**同时满足**以下所有前提条件，然后进入分支判断：

| 前提条件 | 含义 |
|---------|------|
| `available()` | QuadPlane 功能已启用 |
| `motors->armed()` | 电机已解锁 |
| `motors->get_desired_spool_state() >= THROTTLE_UNLIMITED` | 油门不受限 |
| `!tailsitter.enabled()` | 不是尾坐式 |
| `poscontrol.get_state() != QPOS_AIRBRAKE` | 不在空气刹车状态 |
| `!force_fw_control_recovery` | 不在强制 FW 恢复模式 |

满足前提后，以下情况返回 **false**：

| 场景 | 返回值 | 说明 |
|------|--------|------|
| **VTOL 模式**（QHOVER/QLOITER/QLAND/QRTL） | `false` | 始终使用 VTOL 姿态控制器 |
| **前向过渡中**（仅矢量偏航倾转旋翼） | `false` | `Tiltrotor_Transition::use_multirotor_control_in_fwd_transition()` 返回 true 的条件：`tiltrotor.is_vectored() && transition_state <= TRANSITION_TIMER` |
| **纯固定翼模式**（FBWA/AUTO 等） | `true` | 使用固定翼 PID 控制器 |
| **尾坐式** | `true` | 尾坐式不走此逻辑 |
| **空气刹车状态**（QPOS_AIRBRAKE） | `true` | 使用固定翼控制器 |
| **强制 FW 恢复**（`force_fw_control_recovery=true`） | `true` | 姿态超出 2×Q_ANGLE_MAX 时触发恢复 |
| **电机未解锁** | `true` | 使用固定翼控制器 |
| **油门受限**（spool state < THROTTLE_UNLIMITED） | `true` | 使用固定翼控制器 |

### 对 stabilize_roll/pitch 的影响

**位置**: `ArduPlane/Attitude.cpp:127-203`

当返回 **false** 时：

```cpp
// stabilize_roll_get_roll_out() - Attitude.cpp:131-149
if (!quadplane.use_fw_attitude_controllers()) {
    // 固定翼 PID 的输入来自 VTOL 角速率 PID 的目标值
    const auto &pid_info = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
    const float roll_out = rollController.get_rate_out(degrees(pid_info.target), speed_scaler);
    rollController.decay_I();  // 衰减积分器，防止两个控制器对抗
    return roll_out;
}
// 返回 true 时：固定翼 PID 直接使用角度误差
return rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor, speed_scaler, ...);
```

**关键区别**：
- **false**：固定翼 PID 输入 = VTOL 角速率目标值（`pid_info.target`），同时衰减 FW 积分器
- **true**：固定翼 PID 输入 = 角度误差（`nav_roll_cd - ahrs.roll_sensor`）

### force_fw_control_recovery 触发条件

**位置**: `ArduPlane/VTOL_Assist.cpp:146-203`

```cpp
// 姿态超出 2×Q_ANGLE_MAX 时触发
const float abs_angle_cd = fabsf(Vector2f{roll_sensor, pitch_sensor}.length());
if (abs_angle_cd > 2 * angle_max_cd) {
    quadplane.force_fw_control_recovery = true;  // 强制使用 FW 控制器恢复
}

// 回到 Q_ANGLE_MAX 内时解除
if (abs_angle_cd <= angle_max_cd) {
    quadplane.force_fw_control_recovery = false;
    quadplane.attitude_control->reset_target_and_rate(false);
    // 如果地速较大，还重置位置和高度控制器
}
```

**作用**：VTOL 模式下姿态严重失控（超出 2 倍角度限制），强制切换到固定翼控制器帮助恢复。

---

## 2. calc_throttle() 调用时机

**位置**: `ArduPlane/Attitude.cpp:459-472`

```cpp
void Plane::calc_throttle()
{
    if (aparm.throttle_cruise <= 1) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        return;
    }
    // 从 TECS 能量控制器获取油门需求
    float commanded_throttle = TECS_controller.get_throttle_demand();
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, commanded_throttle);
}
```

### 调用位置

| 文件 | 行号 | 调用场景 |
|------|------|----------|
| `ArduPlane/navigation.cpp` | 446 | `update_altitude()` 中，**主循环 50Hz 调用** |
| `ArduPlane/mode_loiter.cpp` | 28 | `ModeLoiter::run()` |
| `ArduPlane/mode_auto.cpp` | 104, 120 | `ModeAuto::run()`（非起飞阶段） |
| `ArduPlane/mode_guided.cpp` | 98 | `ModeGuided::run()` |
| `ArduPlane/mode_rtl.cpp` | 47 | `ModeRTL::run()` |
| `ArduPlane/mode_circle.cpp` | 21 | `ModeCircle::run()` |
| `ArduPlane/mode_thermal.cpp` | 25 | `ModeThermal::run()` |
| `ArduPlane/mode_takeoff.cpp` | 192 | `ModeTakeoff::run()`（非起飞初始阶段） |
| `ArduPlane/takeoff.cpp` | 290 | `takeoff_calc_throttle()` 中 |

### 主循环中的位置

```
AP_Vehicles::loop()
  └─> AP_Scheduler::tick()
        ├─> update_control_mode()     // 50Hz - 设置 nav_roll/pitch
        ├─> stabilize()               // 50Hz - 姿态稳定（副翼/升降舵）
        ├─> set_servos()              // 50Hz - 电机输出
        └─> update_altitude()         // 50Hz - 高度/能量控制
              ├─> calc_throttle()     // ← TECS 计算固定翼油门
              └─> calc_nav_pitch()    // TECS 计算俯仰角
```

### 与 VTOL 的关系

- **固定翼模式**：`calc_throttle()` 通过 TECS 计算油门，输出到 `SRV_Channel::k_throttle`
- **VTOL 模式**：油门由 `hold_hover()` → `run_z_controller()` → `motors.set_throttle()` 设置，**不经过 `calc_throttle()`**
- **过渡阶段**（TRANSITION_TIMER）：VTOL 油门和 FW 油门（来自 `calc_throttle()`）通过 `ratio` 混合（`quadplane.cpp:1636-1642`）

注意：`calc_throttle()` 在固定翼模式下**每个周期都会被调用**（通过 `update_altitude()`），即使当前是 VTOL 模式——它始终从 TECS 计算油门值，只是 VTOL 模式下这个值不被用于电机输出（但会在过渡阶段被读取用于混合）。

---

## 3. in_vtol_mode() 判断逻辑

**位置**: `ArduPlane/quadplane.cpp:2100-2126`

**目的**: 判断当前是否处于 VTOL 模式，用于决定**是否运行过渡处理代码**（transition handling code）。注意：AIRBRAKE 状态不算 `in_vtol_mode`，即使此时 VTOL 电机正在运行。

### 函数实现

```cpp
bool QuadPlane::in_vtol_mode(void) const
{
    if (!available()) {
        return false;  // QuadPlane 未启用
    }
    if (in_vtol_land_sequence()) {
        // VTOL 降落序列中：排除 APPROACH 和 AIRBRAKE 阶段
        return poscontrol.get_state() != QPOS_APPROACH && poscontrol.get_state() != QPOS_AIRBRAKE;
    }
    if (plane.control_mode->is_vtol_mode()) {
        return true;   // 当前飞行模式本身是 VTOL 模式
    }
    if (plane.control_mode->is_guided_mode()
        && plane.auto_state.vtol_loiter &&
        poscontrol.get_state() > QPOS_APPROACH) {
        return true;   // GUIDED 模式 + VTOL 悬停指令 + 已过了进近阶段
    }
    if (plane.control_mode == &plane.mode_guided &&
        guided_takeoff) {
        return true;   // GUIDED 模式下的 VTOL 起飞
    }
    if (in_vtol_auto()) {
        // AUTO 模式中的 VTOL 任务
        if (!plane.auto_state.vtol_loiter || poscontrol.get_state() > QPOS_AIRBRAKE) {
            return true;
        }
    }
    return false;
}
```

### 判断条件汇总

按优先级从高到低：

| 优先级 | 条件 | 返回 | 说明 |
|--------|------|------|------|
| 0 | `!available()` | `false` | QuadPlane 未启用，直接 false |
| 1 | `in_vtol_land_sequence()` 且状态 ∉ {APPROACH, AIRBRAKE} | `true` | VTOL 降落序列中（QRTL/进近/下降/最终降落），但排除早期的 APPROACH 和 AIRBRAKE 阶段 |
| 2 | `control_mode->is_vtol_mode()` | `true` | 当前模式本身是 Q 模式 |
| 3 | GUIDED 模式 + `vtol_loiter` + 状态 > APPROACH | `true` | GUIDED 下执行 VTOL 悬停 |
| 4 | GUIDED 模式 + `guided_takeoff` | `true` | GUIDED 下执行 VTOL 起飞 |
| 5 | `in_vtol_auto()` + (非 vtol_loiter 或 状态 > AIRBRAKE) | `true` | AUTO 模式中的 VTOL 任务 |
| 6 | 以上都不满足 | `false` | 固定翼模式 |

### 哪些模式 is_vtol_mode() = true

定义在 `ArduPlane/mode.h`，以下 7 个 Q 模式重写了 `is_vtol_mode()` 返回 true：

| 模式类 | 模式名 | 说明 |
|--------|--------|------|
| `ModeQStabilize` | QSTABILIZE | VTOL 自稳（所有 Q 模式的基类） |
| `ModeQHover` | QHOVER | VTOL 悬停 |
| `ModeQLoiter` | QLOITER | VTOL 定点悬停 |
| `ModeQLand` | QLAND | VTOL 降落 |
| `ModeQRTL` | QRTL | VTOL 返航 |
| `ModeQAcro` | QACRO | VTOL 特技 |
| `ModeQAutotune` | QAUTOTUNE | VTOL 自调参 |

### in_vtol_auto() 判断

**位置**: `ArduPlane/quadplane.cpp:2062-2091`

在 AUTO 模式下，根据当前任务指令判断是否处于 VTOL 阶段：

```cpp
bool QuadPlane::in_vtol_auto(void) const
{
    if (!available()) return false;
    if (plane.control_mode != &plane.mode_auto) return false;
    if (plane.auto_state.vtol_mode) return true;  // 标记为 VTOL 阶段

    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:     return true;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT:    return plane.auto_state.vtol_loiter;
    case MAV_CMD_NAV_TAKEOFF:          return is_vtol_takeoff(id);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
    case MAV_CMD_NAV_PAYLOAD_PLACE:    return is_vtol_land(id);
    default:                           return false;
    }
}
```

### in_vtol_land_sequence() 判断

**位置**: `ArduPlane/quadplane.cpp:4188-4191`

```cpp
bool QuadPlane::in_vtol_land_sequence(void) const
{
    return plane.control_mode == &plane.mode_qrtl
        || in_vtol_land_approach()
        || in_vtol_land_descent()
        || in_vtol_land_final();
}
```

### 关键设计意图

1. **AIRBRAKE 不算 VTOL 模式**：虽然 AIRBRAKE 阶段 VTOL 电机在运行，但此时飞机仍用固定翼姿态控制来减速，所以 `in_vtol_mode()` 返回 false
2. **APPROACH 不算 VTOL 模式**（仅在降落序列中）：降落进近阶段飞机还在用固定翼方式飞行
3. **与 `use_fw_attitude_controllers()` 的关系**：`in_vtol_mode()` 返回 true 时，`use_fw_attitude_controllers()` 返回 false，即使用 VTOL 姿态控制器而非固定翼 PID
4. **与过渡逻辑的关系**：`quadplane.update()` 中根据 `in_vtol_mode()` 决定走 `transition->update()`（固定翼侧）还是 `motors_output()`（VTOL 侧）

### Q 模式（VTOL 模式）说明

**Q 模式 = QuadPlane 模式 = VTOL 模式**，即以多旋翼方式控制飞机的模式。这些模式使用：
- VTOL 姿态控制器（`AC_AttitudeControl_Multi`）
- VTOL 位置控制器（`AC_PosControl`）
- 电机混控输出（`AP_MotorsMatrix`）

判断方式：`control_mode->is_vtol_mode()` 返回 true 的模式就是 Q 模式，包括 QSTABILIZE、QHOVER、QLOITER、QLAND、QRTL、QACRO、QAUTOTUNE。

### 完整 VTOL 飞行流程

#### 1. 垂直起飞（VTOL Takeoff）

**场景**：AUTO 模式执行 `MAV_CMD_NAV_VTOL_TAKEOFF` 任务，或手动切换到 QHOVER 模式起飞

| 阶段 | 飞行模式 | poscontrol state | in_vtol_mode() | 说明 |
|------|----------|------------------|----------------|------|
| 起飞前 | QHOVER 或 AUTO(VTOL_TAKEOFF) | QPOS_NONE | **true** | 电机垂直向上，VTOL 控制器计算油门 |
| 爬升中 | QHOVER 或 AUTO(VTOL_TAKEOFF) | QPOS_NONE | **true** | 位置控制器控制高度 |
| 到达目标高度 | QHOVER 或 AUTO(VTOL_TAKEOFF) | QPOS_NONE | **true** | 悬停等待 |

**为什么是 VTOL 模式**：飞机静止或低速，机翼无升力，必须靠旋翼垂直升力。

#### 2. 前向过渡（Forward Transition）

**场景**：从 QHOVER 切换到 FBWA/AUTO，或 AUTO 中从 VTOL_TAKEOFF 进入巡航航点

| 阶段 | 飞行模式 | poscontrol state | transition_state | in_vtol_mode() | 说明 |
|------|----------|------------------|------------------|----------------|------|
| 开始过渡 | FBWA/AUTO（刚切出 Q 模式） | QPOS_NONE | TRANSITION_AIRSPEED_WAIT | **false** | 已切到固定翼模式，但 VTOL 电机仍工作 |
| 等待空速 | FBWA/AUTO | QPOS_NONE | TRANSITION_AIRSPEED_WAIT | **false** | VTOL 电机保持悬停姿态，等待空速达标 |
| 过渡计时 | FBWA/AUTO | QPOS_NONE | TRANSITION_TIMER | **false** | 空速达标，VTOL 推力渐减，固定翼舵面渐增 |
| 过渡完成 | FBWA/AUTO | QPOS_NONE | TRANSITION_DONE | **false** | 纯固定翼飞行，VTOL 电机关闭或转为前飞推力 |

**为什么不是 VTOL 模式**：虽然过渡期间 VTOL 电机仍在工作，但飞行模式已切换到固定翼模式（FBWA/AUTO），`control_mode->is_vtol_mode()` 返回 false。过渡逻辑由 `transition->update()` 处理，而不是 `motors_output()`。

**关键**：`in_vtol_mode()` 返回 false 时，`use_fw_attitude_controllers()` 可能返回 true（取决于 `force_fw_control_recovery` 和 `use_multirotor_control_in_fwd_transition()`），决定固定翼 PID 的输入来源。

#### 3. 固定翼巡航

| 阶段 | 飞行模式 | poscontrol state | in_vtol_mode() | 说明 |
|------|----------|------------------|----------------|------|
| 巡航 | FBWA/AUTO/LOITER 等 | QPOS_NONE | **false** | 纯固定翼控制，TECS 计算油门 |

#### 4. 后向过渡（Back Transition）

**场景**：从 FBWA/AUTO 切换到 QHOVER/QLOITER/QLAND/QRTL

| 阶段 | 飞行模式 | poscontrol state | in_vtol_mode() | 说明 |
|------|----------|------------------|----------------|------|
| 切换模式瞬间 | QHOVER/QLOITER/QLAND/QRTL | QPOS_NONE | **true** | 立即进入 VTOL 模式 |
| VTOL 电机启动 | QHOVER/QLOITER/QLAND/QRTL | QPOS_NONE | **true** | 电机从全前快速转到全上，提供垂直升力 |
| 减速悬停 | QHOVER/QLOITER/QLAND/QRTL | QPOS_NONE | **true** | VTOL 控制器接管姿态和高度 |

**为什么是 VTOL 模式**：用户主动切换到 Q 模式，`control_mode->is_vtol_mode()` 立即返回 true。VTOL 电机立即启动，固定翼舵面逐步退出。

**注意**：后向过渡没有像前向过渡那样的状态机（TRANSITION_AIRSPEED_WAIT/TIMER/DONE），而是直接由 VTOL 控制器接管。倾转旋翼的电机倾转角通过 `tiltrotor.update()` 从全前渐变到全上。

#### 5. VTOL 降落（VTOL Landing）

**场景**：AUTO 模式执行 `MAV_CMD_NAV_VTOL_LAND`，或 QRTL/QLAND 模式

| 阶段 | 飞行模式 | poscontrol state | in_vtol_mode() | 说明 |
|------|----------|------------------|----------------|------|
| 进近（Approach） | AUTO(VTOL_LAND) 或 QRTL | QPOS_APPROACH | **false** | 固定翼方式飞向降落点，VTOL 电机不工作 |
| 空气刹车（Airbrake） | AUTO(VTOL_LAND) 或 QRTL | QPOS_AIRBRAKE | **false** | VTOL 电机启动但用固定翼姿态控制减速 |
| 定位（Position1） | AUTO(VTOL_LAND) 或 QRTL | QPOS_POSITION1 | **true** | 进入 VTOL 模式，VTOL 控制器接管 |
| 下降（Land Descend） | AUTO(VTOL_LAND) 或 QRTL | QPOS_LAND_DESCEND | **true** | 垂直下降 |
| 最终降落（Land Final） | AUTO(VTOL_LAND) 或 QRTL | QPOS_LAND_FINAL | **true** | 接近地面，降低下降速率 |
| 降落完成 | AUTO(VTOL_LAND) 或 QRTL | QPOS_LAND_COMPLETE | **true** | 电机停转 |

**为什么 APPROACH 和 AIRBRAKE 不是 VTOL 模式**：
- **APPROACH**：飞机还在用固定翼方式飞行，需要机翼升力维持高度
- **AIRBRAKE**：虽然 VTOL 电机已启动，但飞机速度仍较高，用固定翼姿态控制更有效（机翼仍有升力），VTOL 电机主要提供阻力减速

**为什么 POSITION1 之后是 VTOL 模式**：飞机已减速到可以安全悬停的速度，VTOL 控制器接管姿态和高度控制。

### 流程图总结

```
垂直起飞                    前向过渡                    固定翼巡航
  QHOVER/AUTO                 FBWA/AUTO                   FBWA/AUTO
  in_vtol=true               in_vtol=false               in_vtol=false
  posctl=NONE                trans_state=WAIT→TIMER→DONE posctl=NONE
      │                           │                           │
      └───────────────────────────┴───────────────────────────┘
                                  │
                                  ↓
                            后向过渡（切到 Q 模式）
                              QHOVER/QLOITER/QRTL
                              in_vtol=true
                              posctl=NONE
                                  │
                                  ↓
                            VTOL 降落序列
                              AUTO(VTOL_LAND)/QRTL
                              ├─ APPROACH:  in_vtol=false (固定翼进近)
                              ├─ AIRBRAKE:  in_vtol=false (VTOL 电机减速)
                              ├─ POSITION1: in_vtol=true  (VTOL 接管)
                              ├─ LAND_DESCEND: in_vtol=true
                              ├─ LAND_FINAL: in_vtol=true
                              └─ LAND_COMPLETE: in_vtol=true
```

### 设计原则

1. **VTOL 模式 = 需要 VTOL 控制器**：当机翼升力不足或速度很低时，需要旋翼提供垂直升力和姿态控制
2. **非 VTOL 模式 = 可以用固定翼控制器**：当机翼有足够升力时，固定翼控制器更高效
3. **过渡期间**：虽然 VTOL 电机工作，但飞行模式已切换，由 `transition->update()` 管理混合
4. **降落序列**：早期阶段（APPROACH/AIRBRAKE）飞机还有速度，用固定翼控制；后期阶段（POSITION1+）速度低，切到 VTOL 控制

---

## 4. quadplane.update() 分支逻辑分析

**位置**: `ArduPlane/quadplane.cpp:1745-1771`

### 代码片段

```cpp
if (!in_vtol_mode() && !in_vtol_airbrake()) {
    // we're in a fixed wing mode, cope with transitions and check
    // for assistance needed
    if (plane.control_mode == &plane.mode_manual ||
        plane.control_mode == &plane.mode_acro ||
        plane.control_mode == &plane.mode_training) {
        // in manual modes quad motors are always off
        if (!tailsitter.enabled()) {
            set_desired_spool_state(AP_Motors::DesiredSpoolState::SHUT_DOWN);
            motors->output();
        }
        transition->force_transition_complete();
        assisted_flight = false;
    } else {
        transition->update();
    }

} else {
    assisted_flight = in_vtol_airbrake();
    motors_output();
    transition->VTOL_update();
}
```

### 注释问题

注释 `// we're in a fixed wing mode` **不准确**。`!in_vtol_mode() && !in_vtol_airbrake()` 这个条件覆盖的不仅仅是固定翼模式，而是三种情况：

| 场景 | in_vtol_mode() | in_vtol_airbrake() | 条件结果 |
|------|---------------|-------------------|---------|
| 纯固定翼巡航 | false | false | **true** ✓ |
| 前向过渡（TRANSITION_AIRSPEED_WAIT/TIMER/DONE） | false | false | **true** ✓ |
| VTOL 辅助飞行（assisted_flight=true） | false | false | **true** ✓ |
| 降落 AIRBRAKE 阶段 | false | **true** | false ✗ |
| Q 模式（QHOVER/QLOITER 等） | **true** | false | false ✗ |

**准确含义**：不在 VTOL 模式，也不在空气刹车状态。即需要由过渡状态机来管理的场景。

### 分支逻辑

```cpp
if (!in_vtol_mode() && !in_vtol_airbrake()) {
    // 实际含义：非 VTOL 模式、非空气刹车
    // 包括：纯固定翼、前向过渡、VTOL 辅助
    if (MANUAL/ACRO/TRAINING) {
        // 手动模式：VTOL 电机永远关闭
        motors->SHUT_DOWN;
        transition->force_transition_complete();
        assisted_flight = false;
    } else {
        // 自动/固定翼模式：让过渡状态机决定
        transition->update();  // 内部判断是否需要辅助/过渡
    }
} else {
    // VTOL 模式 或 空气刹车阶段
    assisted_flight = in_vtol_airbrake();
    motors_output();           // VTOL 电机直接输出
    transition->VTOL_update(); // 重置过渡状态
}
```

### transition->update() 的作用

`transition->update()` 是过渡状态机的入口，它需要处理三种情况：

1. **纯固定翼**：检查 `should_assist()`，如果空速太低需要 VTOL 辅助，设置 `assisted_flight = true`
2. **前向过渡中**：驱动状态机 AIRSPEED_WAIT → TIMER → DONE
3. **VTOL 辅助飞行**：维持 VTOL 电机输出辅助固定翼

这三种场景的共同点是：飞行模式不是 Q 模式，但 VTOL 电机可能需要工作。所以 `transition->update()` 内部会判断具体该做什么。

### 整体控制流

```
quadplane.update()
  │
  ├─ if (!in_vtol_mode() && !in_vtol_airbrake())
  │    │
  │    ├─ if (MANUAL/ACRO/TRAINING)
  │    │    └─ VTOL 电机关闭，强制过渡完成
  │    │
  │    └─ else (AUTO/FBWA/LOITER 等)
  │         └─ transition->update()
  │              ├─ 检查是否需要 VTOL 辅助（空速太低）
  │              ├─ 驱动前向过渡状态机
  │              └─ 维持辅助飞行
  │
  └─ else (VTOL 模式 或 AIRBRAKE)
       ├─ motors_output()      // VTOL 电机直接输出
       └─ transition->VTOL_update()  // 重置过渡状态
```

### 关键理解

1. **`transition->update()` 不是只在过渡阶段调用**：它在所有"非 VTOL 模式、非空气刹车"的场景下都会被调用，包括纯固定翼巡航（用于检查是否需要辅助）
2. **过渡状态机是内部驱动的**：`transition->update()` 内部根据当前状态（transition_state）和条件（空速、时间等）决定下一步动作
3. **assisted_flight 标志**：当固定翼模式空速太低时，`transition->update()` 会设置 `assisted_flight = true`，启动 VTOL 电机辅助

---

## 5. attitude_control->rate_controller_run() 详解

### 作用

**角速率内环 PID 控制器**。将期望角速度（`_ang_vel_body`）与实际陀螺仪角速度（`gyro`）做 PID 运算，输出归一化的 roll/pitch/yaw 推力指令给电机库。

这是 VTOL 姿态控制链路的最底层——上游的角度环/前馈已经算好了"需要多大的角速度"，这里负责"用多大的力矩去达到这个角速度"。

### 实现

**位置**: `libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:442-470`

```cpp
void AC_AttitudeControl_Multi::rate_controller_run_dt(const Vector3f& gyro, float dt)
{
    Vector3f ang_vel_body = _ang_vel_body;  // 期望角速度（上游已设好）

    update_throttle_gain_boost();   // 油门大幅变化时增强角度 P
    update_throttle_rpy_mix();      // 更新油门与姿态的混合权重

    ang_vel_body += _sysid_ang_vel_body;  // 叠加系统辨识激励

    // 三轴 PID：期望角速度 vs 实际陀螺仪
    _motors.set_roll (get_rate_roll_pid ().update_all(ang_vel_body.x, gyro.x, dt, _motors.limit.roll,  _pd_scale.x) + _actuator_sysid.x);
    _motors.set_roll_ff(get_rate_roll_pid().get_ff());

    _motors.set_pitch(get_rate_pitch_pid().update_all(ang_vel_body.y, gyro.y, dt, _motors.limit.pitch, _pd_scale.y) + _actuator_sysid.y);
    _motors.set_pitch_ff(get_rate_pitch_pid().get_ff());

    _motors.set_yaw  (get_rate_yaw_pid  ().update_all(ang_vel_body.z, gyro.z, dt, _motors.limit.yaw,   _pd_scale.z) + _actuator_sysid.z);
    _motors.set_yaw_ff(get_rate_yaw_pid().get_ff() * _feedforward_scalar);
}
```

### 输入 → 输出

```
输入：
  _ang_vel_body (x,y,z)    ← 期望角速度，由上游设置
  gyro (x,y,z)             ← 实际角速度，来自 AHRS 陀螺仪
  dt                       ← 控制周期

处理：
  每轴运行 PID：error = 期望 - 实际 → P + I + D + FF → 归一化输出

输出（写入 motors 对象）：
  motors.set_roll(roll_out)      → motors._roll_in    [-1, +1]
  motors.set_pitch(pitch_out)    → motors._pitch_in   [-1, +1]
  motors.set_yaw(yaw_out)        → motors._yaw_in     [-1, +1]
  motors.set_roll_ff(ff)         → motors._roll_in_ff
  motors.set_pitch_ff(ff)        → motors._pitch_in_ff
  motors.set_yaw_ff(ff)          → motors._yaw_in_ff
```

### 在调用链中的位置

```
QuadPlane::motors_output()            [quadplane.cpp:1946]
  │
  ├─ attitude_control->rate_controller_run()   ← 这里
  │    └─ PID(期望角速度 - 陀螺仪) → motors._roll_in / _pitch_in / _yaw_in
  │
  └─ motors->output()                          [quadplane.cpp:2010]
       └─ output_armed_stabilizing()
            ├─ roll_thrust  = motors._roll_in   × compensation_gain
            ├─ pitch_thrust = motors._pitch_in  × compensation_gain
            ├─ yaw_thrust   = motors._yaw_in    × compensation_gain
            └─ 混合矩阵分配 → 每个电机推力 → tilt_compensate → PWM
```

### 谁设置了 _ang_vel_body（期望角速度）

`_ang_vel_body` 由上游的姿态控制函数设置，在调用 `rate_controller_run()` 之前执行：

| 上游函数 | 设置方式 | 调用场景 |
|---------|---------|---------|
| `input_euler_angle_roll_pitch_euler_rate_yaw()` | 角度 P → 角速度目标 + 前馈 | QHOVER/QLOITER 悬停 |
| `input_euler_angle_roll_pitch_yaw()` | 角度 P → 角速度目标 + 前馈 | QSTABILIZE |
| `input_rate_bf_roll_pitch_yaw()` | 直接设置角速度目标 | QACRO 特技模式 |
| `relax_attitude_control()` | 设为零 | 电机未解锁/过渡完成 |

### 对下游的影响

`rate_controller_run()` 的输出直接决定了 `motors->output()` 中混合矩阵的三个姿态分量：

```
motors._roll_in   → roll_thrust   → 混合矩阵 → 各电机推力差 → 滚转力矩
motors._pitch_in  → pitch_thrust  → 混合矩阵 → 各电机推力差 → 俯仰力矩
motors._yaw_in    → yaw_thrust    → 混合矩阵 → 各电机推力差 → 偏航力矩
```

如果不调用 `rate_controller_run()`：
- `motors._roll_in / _pitch_in / _yaw_in` 保持上一次的值（或零）
- 混合矩阵没有姿态修正量，电机只输出总油门
- 飞机无法维持姿态稳定

### 在 Plane 中的调用点

ArduPlane 中只有一处调用：

```cpp
// quadplane.cpp:2001 — QuadPlane::motors_output()
if (run_rate_controller) {
    // ...
    attitude_control->rate_controller_run();  // ← 唯一调用点
    attitude_control->rate_controller_target_reset();
    // ...
}

motors->output();
```

`motors_output()` 在 `quadplane.update()` 的 else 分支（VTOL 模式或 AIRBRAKE）中被调用，以及 `transition->update()` 中辅助飞行/前向过渡时通过 `hold_hover()` / `hold_stabilize()` 间接调用。

### 与固定翼 PID 的关系

| | VTOL rate_controller_run() | 固定翼 stabilize_roll/pitch/yaw() |
|---|---|---|
| **输入** | 角速度误差（期望 - 实际） | 角度误差（期望 - 实际） |
| **输出** | 归一化推力 [-1, +1] | 舵面偏转角度 [-4500, +4500] |
| **执行对象** | 电机（通过 motors 对象） | 舵面（通过 SRV_Channels） |
| **运行频率** | 主循环频率（~300Hz） | 50Hz |
| **何时运行** | VTOL 模式 / 辅助飞行 | 固定翼模式 / 过渡中 |

过渡期间两者并行运行，通过 `use_fw_attitude_controllers()` 决定固定翼 PID 的输入来源（见第 1 节）。
