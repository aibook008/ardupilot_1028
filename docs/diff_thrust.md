# 差动倾转 vs 统一倾转分析

## 概述

在倾转旋翼（Tiltrotor）设计中，有两种主要的偏航控制方式：
- **差动倾转（VectoredYaw）**：左右电机倾转角度不同，通过推力方向不对称产生偏航力矩
- **统一倾转**：所有电机倾转角度相同，通过电机反扭矩差产生偏航力矩

## 两种偏航控制方式

### 方式一：差动倾转（VectoredYaw）

**代码位置**：`ArduPlane/tiltrotor.cpp:616-663`

```cpp
float tilt_scale = throttle_scaler * yaw_out * cos_tilt + avg_roll_factor * roll_out * sin_tilt;

float left_tilt  = base_output + tilt_offset;  // 左侧多倾一点
float right_tilt = base_output - tilt_offset;  // 右侧少倾一点
```

**原理**：左右电机倾转角度不同，推力方向不对称，产生偏航力矩。

```
        机头
         ↑
   ←↗        ↖→     ← 左侧倾角大，右侧倾角小
   左            右     → 水平分力形成力偶 → 偏航力矩
   ↖            ↗
```

### 方式二：统一倾转 + 电机差速

所有电机倾转角度相同，偏航靠**电机反扭矩差**——CW 和 CCW 电机转速差产生偏航力矩。这就是普通四旋翼的偏航方式。

## 为什么两种都能工作

| 偏航方式 | 力矩来源 | 何时有效 |
|---------|---------|---------|
| 差动倾转 | 推力方向不对称形成的力偶 | 电机部分倾转时（0° < θ < 90°） |
| 电机差速 | CW/CCW 电机反扭矩差 | 所有状态都有效 |

在 VTOL 悬停时，两种方式**同时工作**——电机差速一直在提供偏航控制，差动倾转只是额外增加了一个偏航力矩来源。

## 为什么统一倾转可能更稳定

有三个关键原因：

### 1. 偏航与滚转的耦合

差动倾转时，左右倾角不同导致**垂直升力不对称**，产生非预期的滚转力矩：

```
差动倾转：
  左侧倾角 30°  → 垂直升力 = T × cos(30°) = 0.866T
  右侧倾角 20°  → 垂直升力 = T × cos(20°) = 0.940T
  → 升力差 = 0.074T → 非预期滚转！
```

飞控必须不断补偿这个耦合，增加了控制负担。统一倾转没有这个问题。

### 2. 油门相关的偏航增益

代码中有油门缩放：

```cpp
throttle_scaler = constrain_float(motors->get_throttle_hover() / throttle, 0.5, 2.0);
```

低油门时 `throttle_scaler` 变大 → 差动偏转增益变大 → 偏航响应过强 → 容易振荡。统一倾转的偏航增益与油门无关，更一致。

### 3. 倾转角度的限制

差动倾转的偏航力矩与 `cos_tilt` 成正比：

```cpp
tilt_scale = throttle_scaler * yaw_out * cos_tilt + ...
```

- 悬停时（θ ≈ 0°）：cos(0°) = 1，差动有效
- 过渡时（θ ≈ 45°）：cos(45°) = 0.707，差动减弱
- 接近固定翼（θ ≈ 90°）：cos(90°) ≈ 0，差动几乎无效

也就是说差动倾转的偏航控制力随飞行状态变化很大，而电机差速的偏航控制力始终稳定。

## 什么时候该用差动

| 场景 | 差动倾转 | 统一倾转 |
|------|---------|---------|
| 悬停抗风 | ✅ 额外偏航力矩 | ✅ 电机差速足够 |
| 低速精确悬停 | ✅ 响应更快 | ⚠️ 可能够用 |
| 过渡阶段 | ⚠️ 引入耦合 | ✅ 更稳定 |
| 高速前飞 | ❌ 几乎无效 | ✅ 电机差速有效 |
| 大尺寸/重型 | ✅ 电机差速不够用 | ⚠️ 看电机扭矩 |

**结论**：对于中小型 quadplane，电机差速提供的偏航力矩通常够用，统一倾转更简单、耦合更少、更稳定。差动倾转主要在大尺寸飞行器（电机反扭矩不够）或需要极强悬停抗风能力时才有明显优势。

## 参数配置对比

### 差动倾转配置（Q_TILT_TYPE=2）

```ini
Q_TILT_TYPE      2          # VectoredYaw（矢量偏航）
Q_TILT_MASK      15         # 电机 0-3 都是倾转电机

SERVO12_FUNCTION 76         # k_tiltMotorRight
SERVO13_FUNCTION 75         # k_tiltMotorLeft
SERVO12_MIN 1000
SERVO12_MAX 2000
SERVO13_MIN 1000
SERVO13_MAX 2000
```

### 统一倾转配置（Q_TILT_TYPE=0）

```ini
Q_TILT_TYPE      0          # 连续模式（统一倾转）
Q_TILT_MASK      15         # 电机 0-3 都是倾转电机

SERVO12_FUNCTION 41         # k_motor_tilt（统一倾转）
SERVO13_FUNCTION 41         # k_motor_tilt（统一倾转）
SERVO12_MIN 1000
SERVO12_MAX 2000
SERVO13_MIN 1000
SERVO13_MAX 2000
```

**注意**：统一倾转配置下，SERVO12 和 SERVO13 都映射到 `k_motor_tilt`（功能码 41），两个舵机通道输出相同的值，SITL 模型中虽然分了 `pitch_servo=7` 和 `pitch_servo=8`，但因为读取到的 PWM 值相同，所以所有电机倾转角度一致。

## 相关文档

- `docs/tilt_servo_mapping_summary.md` - 倾转舵机映射关系
- `docs/tiltrotor_motor_control.md` - 倾转旋翼电机控制逻辑
- `docs/vtol_intialization.md` - QuadPlane 初始化流程
