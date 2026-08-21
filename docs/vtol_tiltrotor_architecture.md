# ArduPilot VTOL 倾转旋翼架构与算法通读

> 创建：2026-08-09
> 适用机型：垂直起降倾转飞机（Tiltrotor VTOL）
> 核心代码文件：`ArduPlane/quadplane.h/cpp`、`tiltrotor.h/cpp`、`transition.h`、`tailsitter.h/cpp`

---

## 一、整体架构：QuadPlane 框架

ArduPilot 的 VTOL 实现统一在 **QuadPlane** 框架下，支持三种推力构型：

```cpp
// ArduPlane/quadplane.h:80-84
enum class ThrustType : uint8_t {
    SLT=0,        // Separate Left Thrust（传统四轴+推拉力）
    TAILSITTER,   // 尾坐式
    TILTROTOR,    // 倾转旋翼
};
```

**关键类层次**：
```
Plane (ArduPlane 主类)
  └─ QuadPlane (VTOL 管理器)
       ├─ Tiltrotor (倾转旋翼控制)
       ├─ Tailsitter (尾坐式控制)
       ├─ Transition (过渡逻辑基类)
       │    ├─ SLT_Transition (传统四轴过渡)
       │    └─ Tiltrotor_Transition (倾转过渡，继承自 SLT)
       └─ VTOL_Assist (辅助飞行逻辑)
```

---

## 二、主循环核心调用顺序

### 1. 调度入口

`ArduPlane/ArduPlane.cpp` 中定义调度任务表，主循环按优先级执行：

```
AP_Vehicle::loop()
  └─> AP_Scheduler::tick()          // 按优先级遍历 task 表
        ├─> update_GPS()            // 10Hz
        ├─> update_speed_height()   // 50Hz → quadplane.update_throttle_mix()
        ├─> stabilize()             // 50Hz → control_mode->run()
        ├─> set_servos()            // 50Hz → quadplane.update()
        ├─> update_control_mode()   // 50Hz → control_mode->update()
        └─> ...
```

**关键：`stabilize()` 和 `set_servos()` 是 VTOL 逻辑的两个核心入口。**

### 2. 模式层（决定"做什么"）

```
update_control_mode()
  └─> control_mode->update()
        ├─> ModeQHover::update()   → QSTABILIZE::update()  // 设 nav_roll/pitch
        ├─> ModeQLoiter::update()  → QSTABILIZE::update()  // + 位置环
        ├─> ModeQLand::update()    → QLOITER::update()     // 自动降落
        ├─> ModeQRTL::update()     → QLOITER::update()     // 返航悬停
        └─> ModeAuto::update()     → quadplane.control_auto()
                                      ├─> takeoff_controller()
                                      ├─> vtol_position_controller()
                                      └─> waypoint_controller()
```

模式进入时（`_enter()`）初始化位置控制器、清零爬升率、重置油门积分。

### 3. 姿态控制层（stabilize → 算出期望角度）

```
stabilize()                           // 50Hz
  └─> control_mode->run()
        │
        │  ── VTOL 模式 ──
        ├─> quadplane.hold_hover(climb_rate)       // QHOVER
        │     ├─> multicopter_attitude_rate_update()
        │     │     └─> attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw()
        │     │           // 把期望角度/角速率喂给 AC_AttitudeControl
        │     ├─> set_climb_rate_cms()
        │     │     └─> pos_control->input_vel_accel_z()   // 速度→加速度
        │     └─> run_z_controller()
        │           └─> pos_control->update_z_controller()  // 高度 PID → 油门
        │
        ├─> quadplane.hold_stabilize(throttle)     // 过渡中/辅助飞行
        │     ├─> multicopter_attitude_rate_update()
        │     └─> pos_control->input_vel_accel_z()
        │
        │  ── 固定翼模式（过渡中仍有 FW 舵面） ──
        ├─> plane.stabilize_roll()     // 副翼
        └─> plane.stabilize_pitch()    // 升降舵
```

### 4. 舵机输出层（set_servos → 电机实际输出）

```
set_servos()                          // 50Hz
  └─> quadplane.update()
        │
        │  ── 固定翼模式（含过渡） ──
        ├─> transition->update()      // SLT_Transition::update()
        │     ├─> should_assist()     // 判断是否需要 VTOL 辅助
        │     ├─> TRANSITION_AIRSPEED_WAIT:
        │     │     ├─> hold_hover()          // 保持悬停姿态
        │     │     └─> set_throttle_mix_max(1.0)  // 最大 VTOL 油门混合
        │     ├─> TRANSITION_TIMER:
        │     │     ├─> hold_stabilize()      // 渐减 VTOL 推力
        │     │     └─> set_throttle_mix_value(0.5 * scale)
        │     └─> TRANSITION_DONE:
        │           └─> motors->set_desired_spool_state(SHUT_DOWN)
        │
        │  ── VTOL 模式 ──
        ├─> motors_output()
        │     ├─> attitude_control->rate_controller_run()  // 角速率 PID
        │     │     └─> 输出 roll_thrust / pitch_thrust / yaw_thrust
        │     └─> motors->output()
        │           ├─> output_logic()              // 解锁/锁定逻辑
        │           ├─> output_armed_stabilizing()   // 电机混控矩阵
        │           │     └─> calc_thrust()          // 各电机推力计算
        │           └─> output_to_motors()           // → PWM
        │
        ├─> transition->VTOL_update()   // 重置过渡状态
        └─> tiltrotor.update()          // 倾转舵机
              ├─> continuous_update()   // 连续倾转
              │     ├─> VTOL: slew 到全上
              │     ├─> 过渡中: slew 到 get_forward_flight_tilt()
              │     └─> FW: motors->output_motor_mask()
              ├─> binary_update()       // 二位置倾转
              └─> vectoring()           // 矢量偏航
```

### 5. 过渡状态机流转

```
transition->update() 状态流转：

  TRANSITION_DONE (纯固定翼)
       │
       │  切到 VTOL 模式 / should_assist() == true
       ▼
  TRANSITION_AIRSPEED_WAIT
       │  hold_hover() 维持悬停姿态
       │  等待空速达到 transition_airspeed
       │  或超时 → TRANS_FAIL 处理
       ▼
  TRANSITION_TIMER
       │  hold_stabilize() 渐减 VTOL 推力
       │  FW 舵面逐步接管
       │  等待过渡完成条件
       ▼
  TRANSITION_DONE (纯固定翼)
```

**反向过渡**（FW → VTOL）更简单——切模式即生效：
```
mode change → QHOVER::_enter()
  ├─> pos_control->set_max_speed_accel_z()
  ├─> set_climb_rate_cms(0)
  └─> init_throttle_wait()

set_servos() → motors_output()    // 电机立即生效
```

### 6. 完整调用链一览

```
main loop (50Hz)
 │
 ├─ update_control_mode()
 │    └─ mode->update()
 │         └─ 设定期望 nav_roll/nav_pitch/nav_yaw
 │
 ├─ stabilize()
 │    └─ mode->run()
 │         ├─ attitude_control->input_euler_angle_*()   ← 期望角度
 │         ├─ pos_control->input_vel_accel_z()           ← 期望爬升率
 │         └─ pos_control->update_z_controller()         ← 高度 PID → 油门
 │
 └─ set_servos()
      └─ quadplane.update()
           ├─ [FW] transition->update()
           │     └─ hold_hover() / hold_stabilize()     ← 过渡中姿态
           │
           ├─ [VTOL] motors_output()
           │     ├─ rate_controller_run()                ← 角速率 PID
           │     └─ motors->output()                     ← 混控 → PWM
           │
           ├─ tiltrotor.update()                         ← 倾转舵机
           │     └─ slew() → SRV_Channel output
           │
           └─ forward_throttle_pct()                     ← 前向推力
                 └─ SRV_Channel::k_throttle
```

### 7. 关键文件索引

| 层级 | 文件 | 核心函数 |
|------|------|----------|
| 调度入口 | `ArduPlane/ArduPlane.cpp` | `sched_tasks[]` |
| 模式分发 | `ArduPlane/stabilize.cpp` | `stabilize()` |
| 模式实现 | `ArduPlane/mode_q*.cpp` | `run()` / `update()` |
| QuadPlane 总控 | `ArduPlane/quadplane.cpp` | `update()` / `motors_output()` |
| 过渡状态机 | `ArduPlane/transition.h` + `quadplane.cpp` | `SLT_Transition::update()` |
| 姿态控制 | `libraries/AC_AttitudeControl/` | `input_euler_angle_*()` / `rate_controller_run()` |
| 电机混控 | `libraries/AP_Motors/` | `output()` → `calc_thrust()` |
| 倾转控制 | `ArduPlane/tiltrotor.cpp` | `update()` / `continuous_update()` |
| 舵机输出 | `ArduPlane/servos.cpp` | `set_servos()` |

---

## 三、倾转旋翼（Tiltrotor）核心参数

通过 `Q_TILT_*` 参数族控制（`tiltrotor.cpp:5-86`）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Q_TILT_MASK` | 0 | 倾转电机位掩码（bit0=电机1, bit1=电机2...） |
| `Q_TILT_TYPE` | 0 | 倾转类型：0=连续，1=二值，2=矢量偏航，3=双旋翼 |
| `Q_TILT_RATE_UP` | 40°/s | 从水平到垂直的最大倾转速率 |
| `Q_TILT_RATE_DN` | 0°/s | 从垂直到水平的最大倾转速率（0=使用 UP 值） |
| `Q_TILT_MAX` | 45° | 多旋翼控制允许的最大倾转角 |
| `Q_TILT_YAW_ANGLE` | 0° | 矢量偏航的最小倾转角（用于悬停偏航控制） |
| `Q_TILT_FIX_ANGLE` | 0° | 固定翼模式下的前倾角（用于矢量滚转/俯仰） |
| `Q_TILT_FIX_GAIN` | 0 | 固定翼模式下倾转增益 |
| `Q_TILT_WING_FLAP` | 0° | 倾转翼的襟翼角度（过渡完成判据） |

---

## 四、四种倾转类型详解

### 1. Continuous（连续倾转，TYPE=0）

**工作方式**：舵机可以连续转动到任意角度（0°~90°）

```cpp
// tiltrotor.cpp:214-333
void Tiltrotor::continuous_update() {
    // 纯固定翼模式：倾转到前进位置
    if (!quadplane.in_vtol_mode()) {
        slew(get_forward_flight_tilt());
        motors->output_motor_mask(throttle, tilt_mask, rudder_dt);
        return;
    }
    
    // VTOL 模式：根据前向推力需求计算倾转角
    float fwd_g_demand = 0.01 * quadplane.forward_throttle_pct();
    float fwd_tilt_deg = MIN(degrees(atanf(fwd_g_demand)), max_angle_deg);
    slew(MIN(fwd_tilt_deg * (1/90.0), get_forward_flight_tilt()));
}
```

**速率限制逻辑**（`tiltrotor.cpp:187-196`）：
```cpp
void Tiltrotor::slew(float newtilt) {
    float max_change = tilt_max_change(newtilt < current_tilt);
    current_tilt = constrain_float(newtilt, 
                                   current_tilt - max_change, 
                                   current_tilt + max_change);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}
```

### 2. Binary（二值倾转，TYPE=1）

**工作方式**：舵机只有两个位置（完全垂直/完全水平），用于收放式舵机

```cpp
// tiltrotor.cpp:356-375
void Tiltrotor::binary_update() {
    if (!quadplane.in_vtol_mode()) {
        binary_slew(true);   // 前进位置
        if (current_tilt >= 1) {
            motors->output_motor_mask(throttle, tilt_mask, rudder_dt);
        }
    } else {
        binary_slew(false);  // 垂直位置
    }
}
```

### 3. VectoredYaw（矢量偏航，TYPE=2）

**工作方式**：左右电机差动倾转实现偏航控制（悬停时），滚转/俯仰控制（前飞时）

```cpp
// tiltrotor.cpp:550-662
void Tiltrotor::vectoring() {
    const float total_angle = 90 + tilt_yaw_angle + fixed_angle;
    const float zero_out = tilt_yaw_angle / total_angle;
    float base_output = zero_out + (current_tilt * (level_out - zero_out));
    
    if (in_vtol_mode()) {
        // 悬停：差动偏航
        float yaw_out = motors->get_yaw() + motors->get_yaw_ff();
        float tilt_offset = tilt_scale * yaw_range;
        left_tilt = base_output + tilt_offset;
        right_tilt = base_output - tilt_offset;
    } else {
        // 前飞：差动滚转/俯仰（类似升降副翼混合）
        float right = gain * elevon_right;
        float left = gain * elevon_left;
        left_tilt = base_output - right;
        right_tilt = base_output - left;
    }
}
```

### 4. Bicopter（双旋翼，TYPE=3）

**工作方式**：两个倾转旋翼 + 尾翼，必须使用 tailsitter 机架类（10）

```cpp
// tiltrotor.cpp:667-760
void Tiltrotor::bicopter_output() {
    if (!quadplane.in_vtol_mode() && fully_fwd()) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, -SERVO_MAX);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, -SERVO_MAX);
        return;
    }
    // VTOL 模式：混合倾转 + 油门
}
```

---

## 五、控制指令生成详解（roll/yaw/pitch cmd 是怎么来的）

### 1. 旋翼模式：从遥控器输入到电机输出

**完整链路**（300Hz 主循环）：

```
遥控器/自主导航输入
    ↓
QuadPlane::update() [quadplane.cpp:1698]
    ↓
attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(
    nav_roll_cd, nav_pitch_cd, desired_auto_yaw_rate_cds)  [quadplane.cpp:982]
    ↓
AC_AttitudeControl::input_euler_angle_roll_pitch_euler_rate_yaw() 
    [AC_AttitudeControl.cpp:345-396]
    ├─ 将期望欧拉角转为四元数 _attitude_target
    ├─ 计算前馈角速度 _ang_vel_target（如果启用）
    └─ 调用 attitude_controller_run_quat()
    ↓
attitude_controller_run_quat() [AC_AttitudeControl.cpp:865-905]
    ├─ 获取当前姿态四元数 attitude_body
    ├─ 计算姿态误差 attitude_error = _attitude_target - attitude_body
    ├─ 将误差转为角速度修正 ang_vel_body = P_controller(attitude_error)
    ├─ 加上前馈 ang_vel_body += ang_vel_body_feedforward
    └─ 输出目标角速度 _ang_vel_body
    ↓
rate_controller_run() [AC_AttitudeControl.cpp]
    ├─ roll_out = PID_roll(_ang_vel_body.x - gyro.x)
    ├─ pitch_out = PID_pitch(_ang_vel_body.y - gyro.y)
    ├─ yaw_out = PID_yaw(_ang_vel_body.z - gyro.z)
    └─ 设置到 _motors.set_roll/pitch/yaw()
    ↓
AP_MotorsMatrix::output_armed_stabilizing() [AP_MotorsMatrix.cpp:213-380]
    ├─ roll_thrust = _roll_in * compensation_gain
    ├─ pitch_thrust = _pitch_in * compensation_gain
    ├─ yaw_thrust = _yaw_in * compensation_gain
    ├─ throttle_thrust = get_throttle() * compensation_gain
    │
    ├─ 混合矩阵分配：
    │   for each motor i:
    │       _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i]
    │                           + pitch_thrust * _pitch_factor[i]
    │                           + yaw_thrust * _yaw_factor[i]
    │                           + throttle_thrust
    │
    ├─ tilt_compensate() 倾转补偿（见第九节）
    │
    └─ 归一化 + 输出到电机
```

**关键理解**：
- `roll_thrust` 是**归一化力矩**（-1 到 +1），不是角度
- +1 表示最大右滚力矩，-1 表示最大左滚力矩
- 混合矩阵将力矩分配给每个电机（根据电机位置和旋转方向）

### 2. 固定翼模式：从导航到舵面输出

**完整链路**（50Hz 主循环）：

```
导航控制器（L1/TECS）
    ↓
Plane::calc_nav_roll() [ArduPlane.cpp:605-610]
    ├─ commanded_roll = nav_controller->nav_roll_cd()
    └─ nav_roll_cd = constrain(commanded_roll, -roll_limit_cd, roll_limit_cd)
    ↓
Plane::calc_nav_pitch() [ArduPlane.cpp:595-599]
    ├─ commanded_pitch = TECS_controller.get_pitch_demand()
    └─ nav_pitch_cd = constrain(commanded_pitch, pitch_limit_min*100, pitch_limit_max*100)
    ↓
Plane::stabilize_roll() [ArduPlane.cpp:130-159]
    ├─ roll_error_cd = nav_roll_cd - ahrs.roll_sensor
    ├─ 如果 !quadplane.use_fw_attitude_controllers():
    │   ├─ 使用 VTOL 姿态控制器的输出（过渡期间）
    │   └─ roll_out = rollController.get_rate_out(pid_info.target, speed_scaler)
    ├─ 否则：
    │   └─ roll_out = rollController.get_servo_out(roll_error_cd, speed_scaler)
    └─ SRV_Channels::set_output_scaled(k_aileron, roll_out)
    ↓
Plane::stabilize_pitch() [ArduPlane.cpp:166-178]
    ├─ pitch_error_cd = nav_pitch_cd - ahrs.pitch_sensor
    ├─ pitch_out = pitchController.get_servo_out(pitch_error_cd, speed_scaler)
    └─ SRV_Channels::set_output_scaled(k_elevator, pitch_out)
    ↓
Plane::stabilize_yaw() [ArduPlane.cpp:456-478]
    ├─ 协调转弯：yaw_rate = g * tan(nav_roll) / airspeed
    ├─ yaw_out = yawController.get_servo_out(yaw_rate, speed_scaler)
    └─ SRV_Channels::set_output_scaled(k_rudder, yaw_out)
```

**关键理解**：
- `nav_roll_cd` 是**期望滚转角**（厘度，-4500 到 +4500 对应 -45° 到 +45°）
- 固定翼 PID 控制器输入是**角度误差**，输出是**舵面偏转角度**
- 与旋翼不同：旋翼输出归一化力矩，固定翼输出物理角度

### 3. 倾转阶段：两套控制器的混合

**核心问题**：过渡期间，旋翼控制器和固定翼控制器同时运行，如何混合？

**答案**：通过 `transition_progress` 权重（0.0 = 纯旋翼，1.0 = 纯固定翼）

#### 旋翼侧的权重应用

**代码位置**：`quadplane.cpp:908-970`（QuadPlane::multicopter_attitude_rate_update()）

```cpp
void QuadPlane::multicopter_attitude_rate_update()
{
    // 计算旋翼权重（从 1.0 逐步衰减到 0.0）
    float mc_weight = 1.0f - transition_progress;
    
    // 旋翼油门按权重衰减
    float scaled_throttle = throttle * mc_weight;
    motors->set_throttle(scaled_throttle);
    
    if (mc_weight > 0.0f) {
        // 旋翼姿态控制器仍然运行
        attitude_control->input_euler_angle_roll_pitch_yaw(
            plane.nav_roll_cd,
            plane.nav_pitch_cd,
            desired_yaw * 100,
            true
        );
        
        // 运行角速度 PID
        attitude_control->rate_controller_run();
        
        // 电机输出（会自动应用权重，因为 throttle 已经衰减）
        motors->output();
    }
}
```

#### 固定翼侧的权重应用

**代码位置**：`ArduPlane/ArduPlane.cpp:130-250`（Plane::stabilize_roll/pitch/yaw()）

```cpp
void Plane::stabilize_roll()
{
    // 计算固定翼权重（从 0.0 逐步增加到 1.0）
    float fw_weight = transition_progress;
    
    if (fw_weight > 0.0f) {
        float roll_error_cd = nav_roll_cd - ahrs.roll_sensor;
        
        // 关键分支：过渡期间使用 VTOL 姿态控制器还是固定翼 PID？
        if (!quadplane.use_fw_attitude_controllers()) {
            // 过渡早期：使用 VTOL 姿态控制器的输出
            const auto &pid_info = quadplane.attitude_control->get_rate_roll_pid().get_pid_info();
            roll_out = rollController.get_rate_out(degrees(pid_info.target), speed_scaler);
            rollController.decay_I();  // 衰减积分器，防止饱和
        } else {
            // 过渡后期：使用标准固定翼 PID
            roll_out = rollController.get_servo_out(roll_error_cd, speed_scaler, false);
        }
        
        // 舵面输出乘以权重
        roll_out *= fw_weight;
        
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
    }
}
```

#### 权重切换的核心代码位置

| 功能 | 文件 | 行号 | 说明 |
|------|------|------|------|
| transition_progress 计算 | quadplane.cpp | 1466-1671 | SLT_Transition::update() |
| 旋翼权重应用 | quadplane.cpp | 908-970 | mc_weight = 1 - transition_progress |
| 旋翼油门衰减 | quadplane.cpp | 1432 | hold_stabilize(throttle_scaled) |
| 固定翼权重应用 | ArduPlane.cpp | 130-250 | fw_weight = transition_progress |
| 姿态控制器切换判断 | ArduPlane.cpp | 131 | if (!quadplane.use_fw_attitude_controllers()) |

**关键理解**：
- 旋翼和固定翼控制器**并行运行**，不是非此即彼
- 旋翼侧：油门逐步衰减，但姿态控制器继续运行（保持姿态稳定）
- 固定翼侧：舵面输出逐步增加，从 0% 到 100%
- 过渡早期：固定翼 PID 的输入来自 VTOL 姿态控制器的输出（`pid_info.target`）
- 过渡后期：固定翼 PID 直接控制角度误差

---

## 六、过渡控制（Transition）状态机

过渡逻辑在 `SLT_Transition::update()` 中实现（`quadplane.cpp:1466-1671`），有三个状态：

```cpp
enum {
    TRANSITION_AIRSPEED_WAIT,  // 等待空速达到 AIRSPEED_MIN
    TRANSITION_TIMER,          // 空速达标后，逐步减小多旋翼推力
    TRANSITION_DONE            // 过渡完成，纯固定翼模式
};
```

### 状态 1：TRANSITION_AIRSPEED_WAIT

**触发条件**：从 VTOL 模式切入固定翼模式

**行为**：
- 保持悬停姿态（`hold_hover(climb_rate_cms)`）
- 等待空速 > `AIRSPEED_MIN`
- 倾转电机逐步前倾
- 重置固定翼 PID 积分项
- 禁用偏航控制（减少机翼扭转）

**失败处理**：
```cpp
if ((now - transition_start_ms) > transition_failure.timeout * 1000) {
    if (tiltrotor && groundspeed > AIRSPEED_MIN * 0.5) {
        transition_state = TRANSITION_TIMER;  // 强制继续过渡
        in_forced_transition = true;
    } else {
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TRANSITION);
    }
}
```

### 状态 2：TRANSITION_TIMER

**触发条件**：空速达标后进入

**行为**：
- 逐步衰减多旋翼油门
- 过渡时间由 `Q_TRANS_TIME_MS` 控制（默认 5000ms，范围 500~30000ms）
- 固定翼控制器逐步接管

```cpp
float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
float throttle_scaled = last_throttle * transition_scale;
quadplane.hold_stabilize(throttle_scaled);
```

### 状态 3：TRANSITION_DONE

**行为**：关闭多旋翼电机（`SHUT_DOWN`），固定翼完全接管

---

## 七、旋翼阶段控制指令生成（roll/pitch/yaw cmd 是怎么来的）

### 1. 从期望姿态到控制指令的完整链路

```
用户/导航输入（期望角度/角速度）
    ↓
input_euler_angle_roll_pitch_yaw()          [AC_AttitudeControl.cpp:345]
    ↓
设置期望姿态四元数 _attitude_target
    ↓
attitude_controller_run_quat()              [AC_AttitudeControl.cpp:865]
    ↓
计算姿态误差 → 角速度目标 _ang_vel_body
    ↓
rate_controller_run()                       [AC_AttitudeControl.cpp:xxx]
    ↓
roll/pitch/yaw 三个 PID 控制器
    ↓
输出 roll_thrust, pitch_thrust, yaw_thrust  (归一化值 [-1, +1])
    ↓
AP_MotorsMatrix::output_armed_stabilizing() [AP_MotorsMatrix.cpp:213]
    ↓
混合矩阵分配 → 每个电机的 _thrust_rpyt_out[i]
```

### 2. 姿态控制器的输入（期望姿态从哪来）

在 `quadplane.cpp:982`，VTOL 模式下调用：

```cpp
attitude_control->input_euler_angle_roll_pitch_yaw(
    plane.nav_roll_cd,      // 期望滚转角（厘度）
    plane.nav_pitch_cd,     // 期望俯仰角（厘度）
    desired_yaw * 100,      // 期望偏航角（厘度）
    true
);
```

**关键点**：`nav_roll_cd` 和 `nav_pitch_cd` 在旋翼模式下**由 VTOL 位置控制器计算**（`quadplane.cpp:2665-2666`），不是来自 L1/TECS：

```cpp
plane.nav_roll_cd = pos_control->get_roll_cd();   // 来自位置控制器
plane.nav_pitch_cd = pos_control->get_pitch_cd(); // 来自位置控制器
```

### 3. 姿态控制器如何输出角速度目标

`input_euler_angle_roll_pitch_yaw()` 的工作流程（`AC_AttitudeControl.cpp:399`）：

```cpp
void AC_AttitudeControl::input_euler_angle_roll_pitch_yaw(...)
{
    // 1. 将期望欧拉角转换为四元数目标姿态
    _attitude_target.from_euler(euler_roll_angle, euler_pitch_angle, euler_yaw_angle);
    
    // 2. 如果启用前馈，计算期望角速度（用于平滑跟踪）
    if (_rate_bf_ff_enabled) {
        _euler_rate_target.x = input_shaping_angle(...);  // 滚转角速度
        _euler_rate_target.y = input_shaping_angle(...);  // 俯仰角速度
        _euler_rate_target.z = input_shaping_angle(...);  // 偏航角速度
        
        // 3. 将欧拉角速度转换到机体坐标系
        euler_rate_to_ang_vel(_attitude_target, _euler_rate_target, _ang_vel_target);
    }
    
    // 4. 调用核心控制函数
    attitude_controller_run_quat();
}
```

### 4. 核心控制函数：从姿态误差到角速度

`attitude_controller_run_quat()`（`AC_AttitudeControl.cpp:865`）：

```cpp
void AC_AttitudeControl::attitude_controller_run_quat()
{
    // 1. 获取当前姿态四元数
    _ahrs.get_quat_body_to_ned(attitude_body);
    
    // 2. 计算姿态误差（目标姿态 vs 当前姿态）
    thrust_heading_rotation_angles(_attitude_target, attitude_body, attitude_error, ...);
    // attitude_error.x = 滚转误差（弧度）
    // attitude_error.y = 俯仰误差（弧度）
    // attitude_error.z = 偏航误差（弧度）
    
    // 3. 将姿态误差转换为角速度修正（通过 P 控制器）
    Vector3f ang_vel_body = update_ang_vel_target_from_att_error(attitude_error);
    // ang_vel_body.x = 滚转角速度修正
    // ang_vel_body.y = 俯仰角速度修正
    // ang_vel_body.z = 偏航角速度修正
    
    // 4. 加上前馈角速度（期望角速度）
    ang_vel_body += ang_vel_body_feedforward;
    
    // 5. 输出：_ang_vel_body（机体坐标系下的目标角速度）
    _ang_vel_body = ang_vel_body;
}
```

### 5. 角速度 PID 控制器：从角速度到推力

`rate_controller_run()` 对三个轴分别运行 PID：

```cpp
void AC_AttitudeControl::rate_controller_run()
{
    // 获取当前陀螺仪角速度
    Vector3f gyro = get_latest_gyro();
    
    // 滚转 PID
    float roll_out = get_rate_roll_pid().update_all(_ang_vel_body.x - gyro.x, _dt);
    
    // 俯仰 PID
    float pitch_out = get_rate_pitch_pid().update_all(_ang_vel_body.y - gyro.y, _dt);
    
    // 偏航 PID
    float yaw_out = get_rate_yaw_pid().update_all(_ang_vel_body.z - gyro.z, _dt);
    
    // 输出到电机库
    _motors.set_roll(roll_out);    // 设置到 _roll_in
    _motors.set_pitch(pitch_out);  // 设置到 _pitch_in
    _motors.set_yaw(yaw_out);      // 设置到 _yaw_in
}
```

### 6. 电机库接收控制指令

`AP_MotorsMatrix::output_armed_stabilizing()`（`AP_MotorsMatrix.cpp:213`）：

```cpp
void AP_MotorsMatrix::output_armed_stabilizing()
{
    // 从姿态控制器获取归一化的 roll/pitch/yaw 推力指令
    const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    
    // roll_thrust, pitch_thrust, yaw_thrust 都是 [-1, +1] 范围
    // 正值表示正向控制（右滚、前俯、右偏）
    // 负值表示反向控制（左滚、后仰、左偏）
    
    // 接下来通过混合矩阵分配给每个电机（见第六节）
}
```

**关键理解**：
- `roll_thrust`：不是角度，是**归一化的控制力矩**（+1 表示最大右滚力矩）
- `pitch_thrust`：归一化的俯仰力矩
- `yaw_thrust`：归一化的偏航力矩
- 这些值经过混合矩阵后，变成每个电机的推力增量

---

## 八、固定翼阶段控制指令生成（roll/pitch/yaw cmd 是怎么来的）

### 1. 固定翼与旋翼的根本区别

**旋翼模式**：直接控制电机推力（归一化力矩）
**固定翼模式**：控制舵面偏转角度（角度指令）

### 2. 固定翼控制链路

```
导航控制器（L1/TECS）
    ↓
计算期望姿态角：nav_roll_cd, nav_pitch_cd
    ↓
Plane::stabilize_roll()                    [ArduPlane/ArduPlane.cpp:157]
    ↓
rollController.get_servo_out(nav_roll_cd - ahrs.roll_sensor)
    ↓
PID 控制器：角度误差 → 舵面偏转角
    ↓
SRV_Channels::set_output_scaled(k_aileron, roll_out)
```

### 3. 固定翼姿态控制器详解

`Plane::stabilize_roll()`（`ArduPlane/ArduPlane.cpp:157`）：

```cpp
void Plane::stabilize_roll()
{
    // 计算滚转角度误差
    float roll_error_cd = nav_roll_cd - ahrs.roll_sensor;
    
    // 调用固定翼滚转 PID 控制器
    float roll_out = rollController.get_servo_out(
        roll_error_cd,           // 角度误差（厘度）
        speed_scaler,            // 空速缩放因子
        false                    // 是否禁用积分器
    );
    
    // roll_out 是舵面偏转指令（-4500 到 +4500，对应 -45° 到 +45°）
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
}
```

**关键点**：
- `rollController` 是 `AP_FixedWing::PIDController` 类型（不是 AC_PID）
- 输入是**角度误差**（期望角度 - 实际角度），不是角速度
- 输出是**舵面偏转指令**（角度），不是力矩
- `speed_scaler` 用于补偿空速变化对舵面效力的影响

### 4. 固定翼俯仰控制

`Plane::stabilize_pitch()`（`ArduPlane/ArduPlane.cpp:180`）：

```cpp
void Plane::stabilize_pitch()
{
    float pitch_out = stabilize_pitch_get_pitch_out();
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, pitch_out);
}

float Plane::stabilize_pitch_get_pitch_out()
{
    // 从 TECS 获取俯仰指令
    float demanded_pitch = TECS_controller.get_pitch_demand();
    
    // 计算俯仰角度误差
    float pitch_error_cd = nav_pitch_cd - ahrs.pitch_sensor;
    
    // 调用固定翼俯仰 PID 控制器
    float pitch_out = pitchController.get_servo_out(
        pitch_error_cd,
        speed_scaler,
        false
    );
    
    return pitch_out;
}
```

### 5. 固定翼偏航控制（协调转弯）

固定翼通常**不主动控制偏航**，而是通过**协调转弯**自动生成偏航指令：

```cpp
void Plane::stabilize_yaw()
{
    // 协调转弯：根据滚转角计算需要的偏航角速度
    // yaw_rate = g * tan(roll) / airspeed
    float coordination_yaw_rate = GRAVITY_MSS * tanf(radians(nav_roll_cd * 0.01f)) / smoothed_airspeed;
    
    // 调用偏航 PID 控制器
    float yaw_out = yawController.get_servo_out(coordination_yaw_rate, speed_scaler, false);
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, yaw_out);
}
```

**关键理解**：
- 固定翼的 roll/pitch/yaw 输出都是**舵面偏转角度**（-4500 到 +4500）
- 不是归一化力矩，是物理角度
- 通过 `SRV_Channels` 映射到具体的舵机 PWM 输出

---

## 九、倾转阶段权重切换（核心代码位置）

### 1. 权重切换的核心变量

`transition_progress`：0.0 = 纯旋翼，1.0 = 纯固定翼

计算位置：`quadplane.cpp:1466-1671`（SLT_Transition::update()）

```cpp
void SLT_Transition::update()
{
    switch (transition_state) {
        case TRANSITION_AIRSPEED_WAIT:
            transition_progress = 0.0f;  // 纯旋翼
            break;
            
        case TRANSITION_TIMER:
            // 从 0.0 逐步增加到 1.0
            transition_timer_ms = now - transition_start_ms;
            transition_progress = constrain_float(
                transition_timer_ms / (float)transition_time_ms,
                0.0f, 1.0f
            );
            break;
            
        case TRANSITION_DONE:
            transition_progress = 1.0f;  // 纯固定翼
            break;
    }
}
```

### 2. 旋翼控制的逐步退出

位置：`quadplane.cpp:908-970`（QuadPlane::multicopter_attitude_rate_update()）

```cpp
void QuadPlane::multicopter_attitude_rate_update()
{
    // 计算旋翼权重
    float mc_weight = 1.0f - transition_progress;
    
    // 旋翼油门逐步衰减
    float scaled_throttle = throttle * mc_weight;
    motors->set_throttle(scaled_throttle);
    
    // 姿态控制器仍然运行，但输出被权重限制
    if (mc_weight > 0.0f) {
        // 计算旋翼姿态指令
        attitude_control->input_euler_angle_roll_pitch_yaw(
            plane.nav_roll_cd,
            plane.nav_pitch_cd,
            desired_yaw * 100,
            true
        );
        
        // 运行角速度 PID
        attitude_control->rate_controller_run();
    }
}
```

### 3. 固定翼控制的逐步接管

位置：`ArduPlane/ArduPlane.cpp:157-250`（Plane::stabilize_roll/pitch/yaw()）

```cpp
void Plane::stabilize_roll()
{
    // 计算固定翼权重
    float fw_weight = transition_progress;
    
    if (fw_weight > 0.0f) {
        // 计算角度误差
        float roll_error_cd = nav_roll_cd - ahrs.roll_sensor;
        
        // 调用固定翼 PID 控制器
        float roll_out = rollController.get_servo_out(roll_error_cd, speed_scaler, false);
        
        // 舵面输出乘以权重
        roll_out *= fw_weight;
        
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, roll_out);
    }
}
```

### 4. 关键代码位置总结

| 功能 | 文件 | 行号 | 函数 |
|------|------|------|------|
| 权重计算 | quadplane.cpp | 1466-1671 | SLT_Transition::update() |
| 旋翼权重应用 | quadplane.cpp | 908-970 | QuadPlane::multicopter_attitude_rate_update() |
| 旋翼油门衰减 | quadplane.cpp | 1432 | hold_stabilize(throttle_scaled) |
| 固定翼权重应用 | ArduPlane.cpp | 157-250 | Plane::stabilize_roll/pitch/yaw() |
| 姿态指令来源切换 | quadplane.cpp | 982 | attitude_control->input_euler_angle_roll_pitch_yaw() |

---

## 十、旋翼模式混合矩阵（Mixing Matrix）

### 1. 混合矩阵定义

每个电机对姿态控制的贡献由三个因子决定（`AP_MotorsMatrix.cpp:501-525`）：

```cpp
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, ...)
{
    _roll_factor[motor_num] = roll_fac;    // 对滚转的贡献系数
    _pitch_factor[motor_num] = pitch_fac;  // 对俯仰的贡献系数
    _yaw_factor[motor_num] = yaw_fac;      // 对偏航的贡献系数
}
```

### 2. 贡献系数的计算

对于标准多旋翼，贡献系数由电机位置决定：

```cpp
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, ...)
{
    // roll_fac, pitch_fac: 电机相对于重心的位置（归一化到 [-1, 1]）
    // yaw_fac: 电机旋转方向（+1 顺时针，-1 逆时针）
    add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, ...);
}
```

**示例：四轴 X 构型**
```
        前
   M1 ↻    M2 ↺
     \      /
      \    /
       重心
      /    \
     /      \
   M3 ↺    M4 ↻

M1: roll=-0.5, pitch=+0.5, yaw=+1 (前左，顺时针)
M2: roll=+0.5, pitch=+0.5, yaw=-1 (前右，逆时针)
M3: roll=-0.5, pitch=-0.5, yaw=-1 (后左，逆时针)
M4: roll=+0.5, pitch=-0.5, yaw=+1 (后右，顺时针)
```

### 3. 推力分配公式

多旋翼模式下，每个电机的输出由以下公式计算（`AP_MotorsMatrix.cpp:451-497`）：

```cpp
void AP_MotorsMatrix::output_to_motors()
{
    float thrust_out[AP_MOTORS_MAX_NUM_MOTORS];
    
    for (uint8_t i = 0; i < _num_motors; i++) {
        if (!motor_enabled[i]) continue;
        
        // 基础推力 = 总油门
        float thrust = throttle_thrust;
        
        // 姿态控制叠加
        thrust += roll_thrust * _roll_factor[i];   // 滚转分量
        thrust += pitch_thrust * _pitch_factor[i]; // 俯仰分量
        thrust += yaw_thrust * _yaw_factor[i];     // 偏航分量（差动推力）
        
        thrust_out[i] = thrust;
    }
    
    // 归一化：如果任何电机超过 1.0，等比例缩小所有电机
    float max_thrust = 0;
    for (i = 0; i < _num_motors; i++) {
        max_thrust = MAX(max_thrust, thrust_out[i]);
    }
    if (max_thrust > 1.0f) {
        float scale = 1.0f / max_thrust;
        for (i = 0; i < _num_motors; i++) {
            thrust_out[i] *= scale;
        }
    }
    
    // 倾转补偿（见下文）
    tiltrotor.tilt_compensate(thrust_out, _num_motors);
    
    // 输出到电机
    for (i = 0; i < _num_motors; i++) {
        motors->set_output(i, thrust_out[i]);
    }
}
```

### 4. 关键理解

- **roll_thrust, pitch_thrust, yaw_thrust**：由姿态控制器（`AC_AttitudeControl`）计算，表示需要的滚转/俯仰/偏航力矩
- **贡献系数**：表示每个电机对应该力矩的"杠杆效应"，由电机位置决定
- **差动推力实现偏航**：顺时针和逆时针电机的推力差异产生偏航力矩

---

## 十一、固定翼模式期望姿态角来源

### 1. 期望滚转角（nav_roll_cd）

来自 **L1 导航控制器**（`ArduPlane/GCS_Mavlink.cpp:1456-1470`）：

```cpp
float Plane::nav_roll_cd()
{
    // L1 控制器计算期望滚转角
    float roll_limit = roll_limit_cd * 0.01f;  // 转换为度
    float nav_roll = L1_controller.nav_roll(roll_limit);
    
    // 限制在安全范围内
    nav_roll = constrain_float(nav_roll, -roll_limit, roll_limit);
    
    return nav_roll * 100;  // 转换为厘度（centidegrees）
}
```

**L1 控制器原理**：
- 根据当前位置和目标航点，计算"参考点"（前方 L1 距离处的虚拟点）
- 计算到参考点的期望侧向加速度
- 通过物理公式 `a = g * tan(roll)` 反推需要的滚转角
- 公式：`roll = atan(lateral_accel / g)`

### 2. 期望俯仰角（nav_pitch_cd）

来自 **TECS（Total Energy Control System）** 能量控制器（`ArduPlane/TECS.cpp:234-267`）：

```cpp
float Plane::nav_pitch_cd()
{
    // TECS 计算期望俯仰角
    float pitch_limit_min = pitch_limit_min_cd * 0.01f;
    float pitch_limit_max = pitch_limit_max_cd * 0.01f;
    
    float nav_pitch = TECS_controller.get_pitch_demand();
    
    // 限制在安全范围内
    nav_pitch = constrain_float(nav_pitch, pitch_limit_min, pitch_limit_max);
    
    return nav_pitch * 100;  // 转换为厘度
}
```

**TECS 原理**：
- **总能量** = 动能 + 势能 = `0.5 * V² + g * h`
- 计算总能量误差和能量分配误差
- 通过 PID 控制器计算期望俯仰角（控制势能）和油门（控制动能）
- 公式：`pitch_demand = PID(energy_distribution_error)`

### 3. 期望偏航角速率

固定翼通常不直接控制偏航角，而是通过**协调转弯**（`ArduPlane/APM_Control.cpp:456-478`）：

```cpp
float Plane::nav_yaw_rate_cd()
{
    // 协调转弯公式：yaw_rate = g * tan(roll) / V
    float roll_rad = nav_roll_cd * 0.01f * DEG_TO_RAD;
    float airspeed = ahrs.airspeed;
    
    float yaw_rate = GRAVITY_MSS * tanf(roll_rad) / airspeed;
    
    return yaw_rate * RAD_TO_DEG * 100;  // 转换为厘度/秒
}
```

### 4. 关键理解

- **固定翼控制的是姿态角**（roll, pitch），不是力矩
- **L1 控制器**：将导航误差转化为期望滚转角
- **TECS**：将能量误差转化为期望俯仰角和油门
- **协调转弯**：通过滚转角自动产生偏航速率，无需单独控制

---

## 十二、倾转过程中的控制混合机制

### 1. 混合权重（transition_progress）

过渡状态机维护一个 `transition_progress` 变量（`quadplane.cpp:1466-1671`）：

```cpp
void SLT_Transition::update()
{
    switch (transition_state) {
        case TRANSITION_AIRSPEED_WAIT:
            transition_progress = 0.0f;  // 纯多旋翼
            break;
            
        case TRANSITION_TIMER:
            // 从 0.0 逐步增加到 1.0
            transition_timer_ms = now - transition_start_ms;
            transition_progress = constrain_float(
                transition_timer_ms / (float)transition_time_ms,
                0.0f, 1.0f
            );
            break;
            
        case TRANSITION_DONE:
            transition_progress = 1.0f;  // 纯固定翼
            break;
    }
}
```

### 2. 多旋翼控制的逐步退出

```cpp
void QuadPlane::multicopter_attitude_rate_update()
{
    // 计算多旋翼的推力权重
    float mc_weight = 1.0f - transition_progress;
    
    // 多旋翼油门逐步衰减
    float scaled_throttle = throttle * mc_weight;
    motors->set_throttle(scaled_throttle);
    
    // 姿态控制器仍然运行，但输出被权重限制
    if (mc_weight > 0.0f) {
        attitude_control->update();
    }
}
```

### 3. 固定翼控制的逐步接管

```cpp
void Plane::stabilize_roll()
{
    // 计算固定翼的舵面权重
    float fw_weight = transition_progress;
    
    if (fw_weight > 0.0f) {
        // 固定翼滚转控制器运行
        int32_t roll_error = nav_roll_cd - ahrs.roll_sensor;
        int32_t servo_out = rollController.get_servo_out(roll_error);
        
        // 舵面输出乘以权重
        servo_out *= fw_weight;
        
        SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, servo_out);
    }
}
```

### 4. 两套控制器并行运行

```cpp
// 每个控制周期（300Hz）
void Plane::update_flight_mode()
{
    // 1. 固定翼控制器始终运行（计算 nav_roll_cd, nav_pitch_cd）
    calc_nav_roll();   // L1 控制器
    calc_nav_pitch();  // TECS 控制器
    
    // 2. 多旋翼控制器在过渡期间仍然运行
    if (quadplane.in_vtol_mode() || transition_progress < 1.0f) {
        quadplane.multicopter_attitude_rate_update();
    }
    
    // 3. 固定翼舵面输出（权重 = transition_progress）
    stabilize_roll();
    stabilize_pitch();
    stabilize_yaw();
    
    // 4. 倾转舵机控制
    tiltrotor.update();
}
```

### 5. 关键理解

- **transition_progress**：0.0 = 纯多旋翼，1.0 = 纯固定翼
- **多旋翼侧**：油门逐步衰减，但姿态控制器仍然运行（保持姿态稳定）
- **固定翼侧**：舵面输出逐步增加，从 0% 到 100%
- **并行运行**：两套控制器同时计算，通过权重混合输出
- **平滑过渡**：避免控制权突然切换导致的姿态突变

---

## 十三、倾转过程中的油门来源与混合机制

### 1. 核心问题：两个"油门"如何融合

倾转过程中存在两个独立的推力需求：
- **旋翼高度控制**：需要垂直升力维持高度（来自多旋翼 Z 控制器）
- **固定翼前飞推力**：需要水平推力加速（来自 TECS 控制器）

**关键理解**：APM 不是混合两个推力矢量，而是**混合两个油门标量，然后通过物理倾转分解为升力和推力**。

### 2. 各阶段的油门来源

#### 阶段 1：纯旋翼悬停（current_tilt = 0）

**油门来源**：多旋翼高度位置控制器（Z Controller）

```cpp
// quadplane.cpp:1092 - hold_hover()
void QuadPlane::hold_hover(float target_climb_rate_cms)
{
    set_climb_rate_cms(target_climb_rate_cms);
    run_z_controller();  // 高度 PID 计算总油门
}
```

**物理含义**：所有电机垂直向上，油门完全用于维持高度。

#### 阶段 2：空速等待（TRANSITION_AIRSPEED_WAIT）

**油门来源**：仍然是高度控制器，但倾转角度由固定翼油门控制

```cpp
// quadplane.cpp:1578
quadplane.hold_hover(climb_rate_cms);  // 高度控制

// tiltrotor.cpp:330 - 倾转角度与固定翼油门成正比
float settilt = constrain_float(
    (SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) - throttle_min) * 0.02,
    0, 1
);
```

**物理含义**：
- 电机推力仍由高度控制器决定（维持高度）
- 倾转角度由 TECS 前飞油门控制（加速需求越大，倾转越快）
- 通过 `1/cos(θ)` 补偿，确保垂直升力不丢失

#### 阶段 3：过渡计时（TRANSITION_TIMER）——关键阶段

**油门来源**：旋翼油门和固定翼油门的混合

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

#### 阶段 4：纯固定翼（current_tilt = 1.0）

**油门来源**：纯固定翼 TECS 控制器

```cpp
// tiltrotor.cpp:232-250 - 纯固定翼模式
float new_throttle = constrain_float(
    SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * 0.01, 0, 1
);
motors->output_motor_mask(current_throttle, tilt_mask, plane.rudder_dt);
```

**物理含义**：电机完全水平，只提供前飞推力，升力由机翼承担。

### 3. 标量如何分解为升力和推力

**关键机制**：`1/cos(θ)` 补偿 + 物理倾转分解

```cpp
// tiltrotor.cpp:504-523
void Tiltrotor::tilt_compensate(float *thrust, uint8_t num_motors)
{
    if (quadplane.in_vtol_mode()) {
        // 过渡到悬停：补偿垂直升力损失
        const float tilt_factor = cosf(radians(current_tilt * 90));
        tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
    } else {
        // 过渡到固定翼：补偿前飞推力需求
        float inv_tilt_factor = 1.0 / cosf(radians(current_tilt * 90));
        tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
    }
}
```

**物理推导**：

```
电机倾转 θ 角，总推力 = F

垂直分量 = F × cos(θ)  → 提供升力
水平分量 = F × sin(θ)  → 提供前飞推力

问题：如果混合矩阵输出 F = 0.5（假设需要 0.5 升力）
      但电机倾转 60°，实际垂直升力 = 0.5 × cos(60°) = 0.25（掉了一半！）

解决：补偿后 F = 0.5 / cos(60°) = 0.5 × 2.0 = 1.0
      实际垂直升力 = 1.0 × cos(60°) = 0.5 ✓ 恢复正确
      实际前飞推力 = 1.0 × sin(60°) = 0.866 ✓ 提供前飞加速
```

### 4. 完整数据流示例

**场景**：倾转 45°，需要 0.6 升力维持高度

```
步骤 1：多旋翼控制器计算
  throttle_mc = 0.6（高度控制器输出）

步骤 2：固定翼控制器计算
  fw_throttle = 0.8（TECS 前飞需求）

步骤 3：混合两个油门（TRANSITION_TIMER）
  ratio = 0.5（假设倾转 45° 对应 50% 进度）
  throttle_out = 0.6 × 0.5 + 0.8 × 0.5 = 0.7

步骤 4：传入姿态控制器
  hold_stabilize(0.7)

步骤 5：混合矩阵计算每个电机推力
  thrust[i] = 0.7 + roll + pitch + yaw

步骤 6：tilt_compensate 补偿
  inv_tilt_factor = 1 / cos(45°) = 1.414
  thrust[i] *= 1.414  → thrust[i] ≈ 0.99

步骤 7：物理分解（电机倾转 45°）
  垂直升力 = 0.99 × cos(45°) = 0.7  ✓ 维持高度
  前飞推力 = 0.99 × sin(45°) = 0.7  ✓ 提供加速
```

### 5. 为什么不是矢量混合？

**关键理解**：APM 不混合两个推力矢量，而是：

1. **混合两个油门标量**（通过 `ratio`）
2. **用单一标量驱动所有电机**（通过混合矩阵）
3. **靠物理倾转自动分解**（`cos(θ)` 垂直升力 + `sin(θ)` 水平推力）

**优势**：
- 代码简单：不需要矢量运算
- 物理直观：倾转角度直接决定升力/推力比例
- 易于调参：只需调整 `ratio` 的计算方式

---

## 十四、倾转补偿算法深度解析（Tilt Compensation）

### 1. 核心问题

电机倾转时，推力矢量方向改变，导致：
- **垂直升力分量减少**：`vertical = F × cos(θ)`
- **水平推力分量增加**：`horizontal = F × sin(θ)`
- **差动推力效力变化**：滚转效力下降，偏航效力上升

### 2. 补偿算法的物理推导

#### 2.1 升力补偿：为什么是 `1/cos(θ)`

**问题**：电机倾转后，垂直升力分量减少

```
电机垂直时：                    电机倾转 θ 角后：
  F↑                              F ↗
  |                                /θ
  |                              /
  +-- 电机                       Fv = F × cos(θ)  ← 垂直分量
                                 Fh = F × sin(θ)  ← 水平分量
```

如果混合矩阵输出 `F = 0.5`（期望 0.5 升力），但电机倾转 60°：
- 实际垂直升力 = 0.5 × cos(60°) = 0.5 × 0.5 = **0.25**（丢失一半！）

**解决**：补偿电机推力

```
补偿后 F = 0.5 / cos(60°) = 0.5 / 0.5 = 1.0
实际垂直升力 = 1.0 × cos(60°) = 1.0 × 0.5 = 0.5 ✓ 恢复正确
实际前飞推力 = 1.0 × sin(60°) = 1.0 × 0.866 = 0.866 ✓ 提供加速
```

**代码实现**：
```cpp
float inv_tilt_factor = 1.0 / cosf(radians(current_tilt * 90));
tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
//                                              ↑         ↑
//                                    非倾转电机×1   倾转电机×1/cos
```

#### 2.2 滚转效力衰减：为什么是线性插值

**问题**：电机倾转后，左右差动推力对滚转的控制效力下降

```
垂直时（θ=0°）：                    水平时（θ=90°）：
  M1(左) ↑    M2(右) ↑                M1(左) →    M2(右) ←
  差动产生滚转 ✓                      差动只产生前后推力 ✗
  （力臂在 Y 轴，力方向垂直）          （力臂在 Y 轴，力方向水平→无力矩）
```

**物理分析**：
- 垂直分量产生的滚转力矩 ∝ cos(θ)
- 水平分量通过 z_arm（电机与重心高度差）产生的滚转力矩 ∝ sin(θ) × z_arm
- 通常 z_arm ≈ 0（电机和重心几乎同高度），所以滚转效力 ∝ cos(θ)

**代码的简化处理**：用 `current_tilt`（0~1）近似 cos(θ) 的衰减

```cpp
thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1 - current_tilt);
//          ↑                               ↑
//    倾转越多，越趋向平均值（无差动）    倾转越少，越保留原始差动
```

**为什么不是精确的 cos(θ)？**
- 工程近似足够（过渡时间短，PID 有积分余量）
- 线性插值计算更快
- `current_tilt` 本身就是归一化值（0=垂直，1=水平），物理上等价于 sin(θ)

#### 2.3 偏航补偿：为什么是 `yaw × sin(θ)` 而不是 `yaw / sin(θ)`

**核心洞察**：倾转旋翼有两个偏航控制机制

**机制 A：反扭矩偏航**（混合矩阵已处理）

```
CW 电机加速 + CCW 电机减速 → 反扭矩差 → 偏航
偏航效力 ∝ cos(θ)（电机垂直时 100%，水平时 0%）
```

**机制 B：差动推力偏航**（tilt_compensate 补充）

```
左侧电机加速 + 右侧电机减速 → 水平推力差 → 偏航
偏航效力 ∝ sin(θ)（电机垂直时 0%，水平时 100%）
```

**两个机制互补**：

| 倾转角 | 机制 A 效力 | 机制 B 效力 | 总偏航能力 |
|--------|------------|------------|-----------|
| 0°（垂直） | 100% | 0% | 正常 |
| 45° | 71% | 71% | 充足 |
| 90°（水平） | 0% | 100% | 正常 |

**为什么用乘法 `yaw × sin(θ)` 而不是除法 `yaw / sin(θ)`？**

如果用除法（精确逆解）：
```
θ = 5° → sin(5°) = 0.087
ΔF = 0.3 / 0.087 = 3.45 ← 超出电机能力（最大 1.0）

θ = 0° → sin(0°) = 0
ΔF = 0.3 / 0 = ∞ ← 数学奇点！
```

用乘法的设计思想：
```
diff_thrust = yaw_demand × sin(θ) × yaw_gain × roll_factor

偏航力矩 = diff_thrust × sin(θ) × d
         = yaw_demand × sin²(θ) × d

总偏航 = 机制 A（∝ cos(θ)）+ 机制 B（∝ sin²(θ)）
       → 在整个角度范围内平滑过渡，无奇点
```

**代码实现**：
```cpp
const float diff_thrust = motors->get_roll_factor(i) *  // 左右位置（+0.5 或 -0.5）
                          (motors->get_yaw() + motors->get_yaw_ff()) *  // 偏航需求
                          sin_tilt *  // sin(θ)，水平分量比例
                          yaw_gain;   // sin(tilt_yaw_angle)，矢量偏航增益
thrust[i] += diff_thrust;
```

**物理含义**：
- `get_roll_factor(i)`：决定哪个电机加速、哪个减速（左右差动）
- `get_yaw()`：姿态控制器需要的偏航力矩
- `sin_tilt`：当前倾转角下，差动推力能产生多少偏航效力
- `yaw_gain`：矢量偏航机构的几何增益

**如果不加这个补偿会怎样？**

| 阶段 | 反扭矩偏航 | 差动推力偏航 | 总偏航 | 结果 |
|------|-----------|------------|--------|------|
| 垂直（0°） | 正常 | 无 | 正常 | ✅ 没问题 |
| 过渡中（45°） | 71% | 没加 | 71% | ⚠️ 偏航迟钝 |
| 接近水平（80°） | 17% | 没加 | 17% | ❌ 几乎无偏航 |
| 水平（90°） | 0% | 没加 | **0%** | ❌ 完全失控 |

#### 2.4 俯仰控制：为什么代码没有显式处理

**问题**：代码只调整了滚转和偏航，没有对俯仰做类似处理

**原因 1：俯仰效力不会衰减到 0**

```
滚转效力 ∝ cos(θ) → θ=90° 时为 0（完全失效）
俯仰效力 ∝ cos(θ) + sin(θ)×(z_arm/x_arm) → θ=90° 时为 z_arm/x_arm（仍有残余）
```

当电机完全水平时，水平推力通过 z_arm（电机与重心的高度差）仍然产生俯仰力矩。

**原因 2：过渡完成后，俯仰由固定翼舵面接管**

```
θ = 0°（垂直）：俯仰由旋翼控制
θ = 90°（水平）：俯仰由升降舵控制（fixed wing stabilize_pitch）

过渡期间：固定翼俯仰控制逐步接管（fw_weight = transition_progress）
         旋翼俯仰控制逐步退出
```

**原因 3：z_arm 通常很小**

对于大多数倾转旋翼，电机和重心几乎同高度（z_arm ≈ 0），所以：
```
sin(θ) × z_arm ≈ 0
→ 俯仰效力 ≈ cos(θ)，和滚转类似
→ 1/cos(θ) 补偿已经足够
```

**如果 z_arm 很大怎么办？**

某些构型（如 V-22 Osprey）的电机明显高于重心，此时：
- 代码的 1/cos(θ) 补偿会低估俯仰效力
- 但过渡时间短（5-10 秒），姿态控制器有足够的 PID 积分余量处理
- 可以通过调参（如 `Q_TILT_FIX_GAIN`）来补偿

### 3. 补偿算法实现

```cpp
// tiltrotor.cpp:451-497
void Tiltrotor::tilt_compensate_angle(float *thrust, uint8_t num_motors, 
                                       float non_tilted_mul, float tilted_mul) {
    // 1. 对倾转电机施加倾斜因子
    for (uint8_t i = 0; i < num_motors; i++) {
        if (is_motor_tilting(i)) {
            thrust[i] *= tilted_mul;
        } else {
            thrust[i] *= non_tilted_mul;
        }
    }
    
    // 2. 滚转效力随倾转角衰减
    const float sin_tilt = sinf(radians(current_tilt * 90));
    const float yaw_gain = sinf(radians(tilt_yaw_angle));
    
    for (uint8_t i = 0; i < num_motors; i++) {
        if (is_motor_tilting(i)) {
            // 滚转控制效力衰减
            thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1 - current_tilt);
            // 偏航差动推力
            const float diff_thrust = motors->get_roll_factor(i) * 
                                      (motors->get_yaw() + motors->get_yaw_ff()) * 
                                      sin_tilt * yaw_gain;
            thrust[i] += diff_thrust;
        }
    }
    
    // 3. 归一化（防止饱和）
    if (largest_tilted > 1.0f) {
        float scale = 1.0f / largest_tilted;
        for (uint8_t i = 0; i < num_motors; i++) {
            thrust[i] *= scale;
        }
    }
}
```

**过渡到前飞**（`tilt_compensate_down`）：
```cpp
float inv_tilt_factor = 1.0 / cosf(radians(current_tilt * 90));
tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
```

**过渡到悬停**（`tilt_compensate_up`）：
```cpp
const float tilt_factor = cosf(radians(current_tilt * 90));
tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
```

### 3. tilt_compensate 的调用时机与回调机制

**关键问题**：`tilt_compensate` 在什么时候被调用？

**答案**：通过回调机制，在**每个控制周期**（300Hz = 3.3ms）自动调用。

#### 3.1 回调注册（初始化阶段）

```cpp
// tiltrotor.cpp:136 - Tiltrotor::setup()
void Tiltrotor::setup()
{
    // ... 其他初始化代码 ...
    
    // 注册倾转补偿回调函数
    motors->set_thrust_compensation_callback(
        FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t)
    );
}
```

**关键点**：
- 在 Tiltrotor 初始化时，将 `tilt_compensate` 函数注册为回调
- 使用 `FUNCTOR_BIND_MEMBER` 将成员函数绑定为回调
- 回调签名：`void(float *, uint8_t)`，参数为推力数组和电机数量

#### 3.2 每个控制周期的调用时序

```cpp
// AP_MotorsMulticopter.cpp:246
void AP_MotorsMulticopter::output()
{
    // 步骤 0：预处理
    update_throttle_filter();
    thr_lin.update_lift_max_from_batt_voltage();
    output_logic();
    
    // 步骤 1：混合矩阵，构建 _thrust_rpyt_out[]
    output_armed_stabilizing();
    
    // 步骤 2：调用倾转补偿回调（关键步骤！）
    thrust_compensation();
    
    // 步骤 3：输出 PWM
    output_to_motors();
    
    // 步骤 4：后处理
    output_boost_throttle();
    output_rpyt();
    update_external_limits();
    _motor_mask_override = 0;
}
```

#### 3.3 回调实现

```cpp
// AP_MotorsMatrix.cpp:1349
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
        //                                      ↑
        //                          传入整体推力数组（throttle + roll + pitch + yaw）
    }
}
```

#### 3.4 完整时序图

```
每个控制周期（300Hz = 3.3ms）：
│
├─ motors->output()  [AP_MotorsMulticopter.cpp:246]
│   │
│   ├─ output_armed_stabilizing()  [AP_MotorsMatrix.cpp:213]
│   │   ├─ 提取四个控制分量：roll_thrust, pitch_thrust, yaw_thrust, throttle_thrust
│   │   ├─ 第一遍循环：_thrust_rpyt_out[i] = roll × roll_factor + pitch × pitch_factor
│   │   ├─ 计算 yaw_allowed（偏航空间限制）
│   │   ├─ 第二遍循环：_thrust_rpyt_out[i] += yaw × yaw_factor
│   │   ├─ 归一化（防止 rpy 饱和）
│   │   └─ 第三遍循环：_thrust_rpyt_out[i] += throttle_thrust
│   │       ↓
│   │   此时 _thrust_rpyt_out[i] = throttle + roll + pitch + yaw（四项之和）
│   │
│   ├─ thrust_compensation()  [AP_MotorsMatrix.cpp:1349]
│   │   └─ _thrust_compensation_callback(_thrust_rpyt_out, num_motors)
│   │       └─ Tiltrotor::tilt_compensate()  [tiltrotor.cpp:504]
│   │           ├─ 第一段循环：整体 × 1/cos(θ)（统一补偿升力+姿态效力衰减）
│   │           ├─ 第二段循环：滚转差动衰减 + 偏航差动补充
│   │           └─ 归一化（防止饱和）
│   │
│   └─ output_to_motors()  [AP_MotorsMatrix.cpp:143]
│       └─ 将 _thrust_rpyt_out[i] 转换为 PWM 输出到电机
│
└─ 下一个控制周期...
```

#### 3.5 关键理解

1. **自动调用**：`tilt_compensate` 不是手动调用的，而是通过回调机制在 `motors->output()` 中自动调用
2. **每个周期**：只要 Tiltrotor 启用了，它就会在每个控制周期（3.3ms）被调用一次
3. **输入是整体推力**：回调接收的是 `_thrust_rpyt_out[]`，包含 throttle + roll + pitch + yaw 四项之和
4. **输出是修正后的推力**：回调直接修改 `_thrust_rpyt_out[]`，后续的 `output_to_motors()` 使用修正后的值

---

### 4. tilt_compensate 的输入：整体推力（不只是高度控制）

**关键问题**：传入 `tilt_compensate` 的 `thrust[]` 数组到底是什么？

**答案**：是**四个控制分量的总和**，不是仅高度控制器的结果。

#### 步骤 1：`output_armed_stabilizing()` 构建完整推力（AP_MotorsMatrix.cpp:213）

```cpp
void AP_MotorsMatrix::output_armed_stabilizing()
{
    // 提取四个控制分量
    const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;
    float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;
    float throttle_thrust = get_throttle() * compensation_gain;  // ← 高度控制器的结果
    
    // 第一遍：分配 roll + pitch（不含油门和偏航）
    for (uint8_t i = 0; i < num_motors; i++) {
        _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] 
                            + pitch_thrust * _pitch_factor[i];
    }
    
    // ... 计算 yaw_allowed（偏航空间限制）...
    
    // 第二遍：加上 yaw
    for (uint8_t i = 0; i < num_motors; i++) {
        _thrust_rpyt_out[i] += yaw_thrust * _yaw_factor[i];
    }
    
    // ... 归一化（防止 rpy 分量饱和）...
    
    // 第三遍：加上 throttle（油门最后加，确保姿态控制优先级更高）
    for (uint8_t i = 0; i < num_motors; i++) {
        _thrust_rpyt_out[i] += throttle_thrust;
    }
}
```

#### 步骤 2：`thrust_compensation()` 调用 `tilt_compensate`

```cpp
// AP_MotorsMatrix.cpp:1349
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
        //                                      ↑
        //              传入的是整体推力数组，包含 throttle + roll + pitch + yaw 四项之和
    }
}
```

#### 传入 tilt_compensate 的 thrust[i] 到底是什么？

```
thrust[i] = throttle_thrust                      // 高度控制（Z 控制器算出的总油门）
           + roll_thrust × roll_factor[i]         // 滚转差动
           + pitch_thrust × pitch_factor[i]       // 俯仰差动
           + yaw_thrust × yaw_factor[i]           // 偏航差动
```

**是四项之和，不是仅高度控制器的结果。**

#### 为什么对整体推力做 `1/cos(θ)` 补偿是正确的？

因为**所有四个控制通道的效力都按 cos(θ) 衰减**：

| 控制通道 | 垂直时效力 | 倾转 θ 后效力 | 衰减因子 |
|---------|-----------|-------------|---------|
| 油门（升力） | 100% | cos(θ) | cos(θ) |
| 滚转差动 | 100% | cos(θ) | cos(θ) |
| 俯仰差动 | 100% | cos(θ) | cos(θ) |
| 偏航（反扭矩） | 100% | cos(θ) | cos(θ) |

所有通道都衰减同样的比例 → 整体放大 `1/cos(θ)` 是对所有通道的统一补偿。

**具体数值示例**（倾转 45°，需要 0.5 升力 + 0.1 滚转差动）：

```
补偿前：
  thrust[i] = 0.5(油门) + 0.1(滚转) = 0.6

补偿后（1/cos(45°) = 1.414）：
  thrust[i] = 0.6 × 1.414 = 0.849

物理分解（倾转 45°）：
  垂直升力 = 0.849 × cos(45°) = 0.6  ← 恢复原始值 ✓
  其中：垂直升力(油门) = 0.5 × 1.414 × 0.707 = 0.5 ✓
        垂直升力(滚转) = 0.1 × 1.414 × 0.707 = 0.1 ✓
  水平推力 = 0.849 × sin(45°) = 0.6  ← 前飞推力

滚转差动也正确恢复：
  滚转力矩 = 0.1 × 1.414 × cos(45°) = 0.1 ✓
```

**如果不单独对油门补偿，而只对油门做 1/cos(θ)？** 那姿态控制的差动分量在倾转后会变得很弱（只有 cos(θ) 的效力），飞机姿态会变得迟钝。整体放大是最简洁且正确的做法。

#### 第二段循环的精细调整

第一段 `1/cos(θ)` 补偿恢复了所有通道的效力，但**滚转差动推力在物理上确实在失去对滚转的控制能力**（因为力矩臂方向变了），所以第二段需要进一步处理：

| 处理步骤 | 作用对象 | 含义 |
|---------|---------|------|
| 混合矩阵 | throttle + roll + pitch + yaw | 四项分量按电机位置分配 |
| `1/cos(θ)` 整体放大 | **整体推力** | 统一补偿所有通道的 cos(θ) 衰减 |
| 滚转衰减 | 仅倾转电机 | 物理上差动推力对滚转的力矩在消失，用线性插值衰减 |
| 偏航补偿 | 仅倾转电机 | 新的偏航机制（水平差动推力）生效，补充反扭矩偏航的损失 |

### 4. 完整推力计算流程

```
AP_MotorsMulticopter::output()  [AP_MotorsMulticopter.cpp:246]
│
├─ output_armed_stabilizing()  [AP_MotorsMatrix.cpp:213]
│   ├─ 提取四个控制分量：roll_thrust, pitch_thrust, yaw_thrust, throttle_thrust
│   ├─ 第一遍循环：_thrust_rpyt_out[i] = roll × roll_factor + pitch × pitch_factor
│   ├─ 计算 yaw_allowed（偏航空间限制）
│   ├─ 第二遍循环：_thrust_rpyt_out[i] += yaw × yaw_factor
│   ├─ 归一化（防止 rpy 饱和）
│   └─ 第三遍循环：_thrust_rpyt_out[i] += throttle_thrust
│       ↓
│   此时 _thrust_rpyt_out[i] = throttle + roll + pitch + yaw（四项之和）
│
├─ thrust_compensation()  [AP_MotorsMatrix.cpp:1349]
│   └─ 调用 tilt_compensate(_thrust_rpyt_out, num_motors)
│       ├─ 第一段循环：整体 × 1/cos(θ)（统一补偿升力+姿态效力衰减）
│       ├─ 第二段循环：滚转差动衰减 + 偏航差动补充
│       └─ 归一化（防止饱和）
│
└─ output_to_motors()  [AP_MotorsMatrix.cpp:143]
    └─ 将 _thrust_rpyt_out[i] 转换为 PWM 输出到电机
```

### 5. hold_hover 与 tilt_compensate 的关系

`hold_hover()` **不调用分配矩阵，也不调用 tilt_compensate**。它只计算并存储油门值：

```
hold_hover(target_climb_rate)  [quadplane.cpp:1092]
  ├─ set_desired_spool_state(THROTTLE_UNLIMITED)
  ├─ multicopter_attitude_rate_update()  → 设置姿态目标（不计算输出）
  ├─ set_climb_rate_cms()                → 设置爬升率目标
  └─ run_z_controller()
       └─ pos_control->update_z_controller()  [AC_PosControl.cpp:989]
            ├─ 位置 PID → 速度目标
            ├─ 速度 PID → 加速度目标
            ├─ 加速度 PID → thr_out（油门值）
            └─ attitude_control.set_throttle_out(thr_out, ...)
                 └─ motors.set_throttle(throttle_in)  ← 存储到 motors._throttle
```

后续 `motors_output()` 调用 `motors->output()` 时，`get_throttle()` 读取的就是 `hold_hover` 设置的值，然后在 `output_armed_stabilizing()` 中参与混合矩阵计算，最终传入 `tilt_compensate`。

**数据流总结**：
```
hold_hover → 计算油门 → 存储到 motors._throttle
                            ↓（时间上后续调用）
motors_output → motors.output()
                  → output_armed_stabilizing()
                    → get_throttle() 读取油门
                    → 混合矩阵：throttle + roll + pitch + yaw → _thrust_rpyt_out[]
                  → thrust_compensation()
                    → tilt_compensate(_thrust_rpyt_out[])  ← 整体推力，不只是油门
                  → output_to_motors()  → PWM 输出
```

---

## 十五、舵机输出映射

倾转舵机通过 `SRV_Channel::k_motor_tilt` 控制：

```cpp
void Tiltrotor::slew(float newtilt) {
    current_tilt = constrain_float(newtilt, ...);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}
```

**舵机通道配置**：
- `SERVOx_FUNCTION = 136`（Motor Tilt）
- `SERVOx_FUNCTION = 137`（Tilt Motor Left，矢量偏航左）
- `SERVOx_FUNCTION = 138`（Tilt Motor Right，矢量偏航右）
- `SERVOx_FUNCTION = 139`（Tilt Motor Rear，矢量偏航后）

---

## 十六、300kg 大飞机关键参数调优

| 参数 | 代码默认 | 300kg 建议范围 | 原因 |
|------|---------|---------------|------|
| `Q_TILT_RATE_UP` | 40°/s | **10~20°/s** | 大型舵机响应慢，避免姿态突变 |
| `Q_TILT_RATE_DN` | 0（同 UP） | **15~25°/s** | 过渡回悬停可稍快 |
| `Q_TRANS_TIME_MS` | 5000ms | **10000~20000ms** | 大惯性需要更长加速时间 |
| `Q_TRANS_FAIL_TMO` | 10s | **20~30s** | 给过渡更多时间 |
| `Q_TILT_MAX` | 45° | **30~40°** | 限制过渡阶段的最大倾转 |
| `ARSPD_FBW_MIN` | - | **实测确定** | 过渡空速门槛，必须试飞标定 |

---

## 十七、代码扩展点

### 1. 自定义倾转曲线（S 曲线）

```cpp
void Tiltrotor::slew(float newtilt) {
    float progress = fabsf(newtilt - current_tilt);
    float dynamic_rate = max_rate * (0.5 - 0.5 * cosf(progress * M_PI));
    float max_change = dynamic_rate * plane.G_Dt * (1/90.0);
    current_tilt = constrain_float(newtilt, current_tilt - max_change, current_tilt + max_change);
}
```

### 2. 自定义过渡判据

```cpp
case TRANSITION_TIMER: {
    // 新增判据：姿态稳定 + 空速持续达标
    bool attitude_stable = (fabsf(plane.ahrs.roll) < 10°) && (fabsf(plane.ahrs.pitch) < 5°);
    bool airspeed_stable = (aspeed > ARSPD_FBW_MIN * 1.1) && (aspeed_stable_count++ > 50);
    
    if (attitude_stable && airspeed_stable) {
        transition_state = TRANSITION_DONE;  // 提前完成过渡
    }
}
```

### 3. 自定义倾转补偿（非线性滚转衰减）

```cpp
float roll_effectiveness = powf(1 - current_tilt, 0.7);  // 指数 0.7 可调
thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * roll_effectiveness;
```

---

## 十八、代码文件索引

| 文件 | 核心功能 |
|------|---------|
| `ArduPlane/quadplane.h/cpp` | VTOL 管理器，过渡状态机，电机混合 |
| `ArduPlane/tiltrotor.h/cpp` | 倾转控制，速率限制，补偿算法 |
| `ArduPlane/transition.h` | 过渡状态枚举（SLT_Transition 基类） |
| `ArduPlane/tailsitter.h/cpp` | 尾坐式过渡逻辑 |
| `ArduPlane/VTOL_Assist.h/cpp` | 辅助飞行判据 |
| `ArduPlane/servos.cpp` | 舵机输出映射，`k_motor_tilt` 处理 |
| `libraries/AP_Motors/AP_MotorsMatrix.cpp` | 电机混合，`output_motor_mask()` |

**关键函数调用链**：
```
Plane::update_flight_mode()
  → QuadPlane::update()
    → SLT_Transition::update()  // 过渡状态机
      → Tiltrotor::update()     // 倾转控制
        → continuous_update()   // 连续倾转
          → slew(newtilt)       // 速率限制
            → SRV_Channels::set_output_scaled(k_motor_tilt, ...)
```

---

## 十九、主循环频率

VTOL 固件默认主循环 **300Hz**（`SCHED_LOOP_RATE` 自动升级），姿态估计和控制环都跑在 300Hz。强制要求 ≥100Hz（`ArduPlane/AP_Arming.cpp:173-174`）。

对于 300kg 大飞机，物理带宽低，100Hz 足够，300Hz 是浪费。

---

## 二十、尾翼倾转电机的功率限制策略

### 1. 问题背景

对于尾翼带有倾转电机的构型（如串列式倾转旋翼），在倾转过渡过程中存在以下气动干扰问题：

- **旋翼下洗流冲击尾翼**：前旋翼的下洗流直接冲击尾翼倾转电机，产生额外气动力矩
- **气流方向变化**：倾转过程中，电机推力方向从垂直逐渐转为水平，气流对尾翼的冲击角度和强度持续变化
- **推力不对称风险**：如果尾翼电机与前旋翼电机推力不匹配，会导致俯仰/偏航力矩失衡

### 2. 现有代码中的功率限制机制

ArduPilot 提供了几种内置的功率限制手段：

#### 2.1 油门缩放（transition_scale）

```cpp
// quadplane.cpp:TRANSITION_TIMER 阶段
float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
float throttle_scaled = last_throttle * transition_scale;
quadplane.hold_stabilize(throttle_scaled);
```

**特点**：线性衰减，所有电机统一缩放，无法单独控制尾翼电机

#### 2.2 电机掩码（motor_mask）

```cpp
// tiltrotor.cpp:continuous_update()
const uint16_t mask = is_zero(current_throttle) ? 0U : tilt_mask.get();
motors->output_motor_mask(current_throttle, mask, plane.rudder_dt);
```

**特点**：可以控制哪些电机参与输出，但只有开/关两种状态，无法精细调节功率

#### 2.3 倾转补偿（tilt_compensate）

```cpp
// tiltrotor.cpp:tilt_compensate_angle()
if (is_motor_tilting(i)) {
    thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1 - current_tilt);
}
```

**特点**：根据倾转角自动调整推力分配，但对所有倾转电机统一处理

### 3. 推荐的功率限制方案

针对尾翼倾转电机的气动干扰问题，建议采用**分段功率曲线 + 独立限制**的组合方案：

#### 方案 A：基于倾转角度的功率限制曲线

```cpp
// 新增函数：计算功率限制系数
float Tiltrotor::calculate_power_limit() {
    float power_limit = 1.0f;
    
    // 阶段 1：0°~30°（起飞/降落），全功率
    if (current_tilt < 0.33f) {
        power_limit = 1.0f;
    }
    // 阶段 2：30°~60°（过渡危险区），S 曲线限制
    else if (current_tilt < 0.67f) {
        float progress = (current_tilt - 0.33f) / 0.34f;  // 0→1
        power_limit = 1.0f - 0.4f * sinf(progress * M_PI);  // 最低 60%
    }
    // 阶段 3：60°~90°（接近前飞），逐步恢复
    else {
        float progress = (current_tilt - 0.67f) / 0.33f;  // 0→1
        power_limit = 0.6f + 0.4f * progress;  // 60%→100%
    }
    
    // 空速越高，限制越低（防止结构过载）
    if (plane.ahrs.airspeed > 0) {
        float aspeed_ratio = constrain_float(
            plane.ahrs.airspeed / plane.aparm.airspeed_max, 
            0, 1
        );
        power_limit *= (1.0f - aspeed_ratio * 0.3f);  // 最多再降 30%
    }
    
    return constrain_float(power_limit, 0.3f, 1.0f);
}
```

**修改位置**：在 `tiltrotor.cpp::continuous_update()` 中调用

```cpp
void Tiltrotor::continuous_update() {
    // ... 计算 current_tilt ...
    
    // 应用功率限制
    float power_limit = calculate_power_limit();
    float limited_throttle = current_throttle * power_limit;
    
    motors->output_motor_mask(limited_throttle, tilt_mask, plane.rudder_dt);
}
```

#### 方案 B：尾翼电机独立限制

如果尾翼电机与主旋翼电机分开控制（通过 `tilt_mask` 区分），可以单独限制尾翼电机功率：

```cpp
// 新增参数：尾翼电机功率限制系数
// tiltrotor.h
AP_Float tail_motor_power_limit;  // 默认 0.7（70%）

// tiltrotor.cpp::var_info
AP_GROUPINFO("TAIL_PWR_LIM", 15, Tiltrotor, tail_motor_power_limit, 0.7f),

// 新增函数：对尾翼电机施加独立限制
void Tiltrotor::apply_tail_motor_limits(float *thrust, uint8_t num_motors) {
    for (uint8_t i = 0; i < num_motors; i++) {
        if (is_tail_motor(i)) {  // 需要定义 is_tail_motor() 判断逻辑
            float tail_limit = tail_motor_power_limit.get();
            
            // 过渡阶段额外限制
            if (current_tilt > 0.3f && current_tilt < 0.7f) {
                tail_limit *= 0.7f;  // 过渡危险区降到 70%
            }
            
            thrust[i] = MIN(thrust[i], tail_limit);
        }
    }
}
```

**修改位置**：在 `AP_MotorsMatrix::output_to_motors()` 中调用

```cpp
void AP_MotorsMatrix::output_to_motors() {
    // ... 混合矩阵计算 ...
    // ... tilt_compensate ...
    
    // 应用尾翼电机独立限制
    tiltrotor.apply_tail_motor_limits(thrust_out, _num_motors);
    
    // 输出到电机
}
```

#### 方案 C：气动耦合前馈补偿

如果尾翼下洗流影响可以建模，可以加入前馈补偿：

```cpp
// 新增函数：气动耦合前馈补偿
void Tiltrotor::apply_aerodynamic_compensation() {
    // 根据倾转角和油门，计算尾翼需要的补偿舵量
    float downwash_effect = current_tilt * (1.0f - current_tilt) * 4.0f;  // 0~1~0 的钟形曲线
    float throttle_effect = current_throttle;
    
    float compensation = downwash_effect * throttle_effect * k_downwash_gain;
    
    // 补偿到升降舵（抵消下洗流导致的抬头力矩）
    float elevator_cmd = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
    elevator_cmd += compensation;
    
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 
                                    constrain_float(elevator_cmd, -4500, 4500));
}
```

**修改位置**：在 `tiltrotor.cpp::update()` 中调用

```cpp
void Tiltrotor::update() {
    // ... 倾转控制 ...
    
    // 应用气动补偿
    apply_aerodynamic_compensation();
}
```

### 4. 推荐组合策略

对于 300kg 大飞机，建议采用以下组合：

| 方案 | 用途 | 优先级 |
|------|------|--------|
| 方案 A | 基础功率限制，防止过渡阶段过载 | 必选 |
| 方案 B | 尾翼电机独立限制（如果与主旋翼分开控制） | 推荐 |
| 方案 C | 气动补偿前馈（如果下洗流影响显著） | 可选 |

### 5. 参数建议

| 参数 | 建议值 | 说明 |
|------|--------|------|
| `Q_TILT_TAIL_PWR_LIM` | 0.6~0.8 | 尾翼电机基础功率限制 |
| `Q_TILT_TRANS_PWR_MIN` | 0.5~0.7 | 过渡阶段最小功率 |
| `Q_TILT_AIRSPD_GAIN` | 0.2~0.4 | 空速对功率限制的影响系数 |
| `Q_TILT_DOWNWASH_GAIN` | 0.1~0.3 | 下洗流补偿增益 |

### 6. 实现步骤

1. **先实现方案 A**：基础功率曲线，验证过渡稳定性
2. **观察日志**：分析过渡阶段的姿态偏差、电机推力、舵面输出
3. **按需添加方案 B/C**：如果尾翼电机明显过载或姿态不稳定，添加独立限制或气动补偿
4. **参数调优**：通过试飞数据调整功率限制系数和补偿增益

---

## 二十一、测试验证建议

1. **SITL 仿真**：先用 ArduPilot SITL 跑通标准倾转旋翼模型
2. **硬件在环（HITL）**：验证舵机输出映射
3. **系留测试**：地面固定，验证倾转速率、补偿算法
4. **悬停测试**：纯多旋翼模式起飞
5. **过渡测试**：低空短时过渡
6. **完整过渡**：逐步增加过渡时间
