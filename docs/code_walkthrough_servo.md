# 舵机/输出模块代码通读

> 适用分支：`qhz/plane_4.6.3`
> 关联文档：[技术协议摘要](technical_agreement_summary.md) §3.2 转换模态 | [VTOL架构通读](vtol_tiltrotor_architecture.md)

---

## 1. 模块总览

| 模块 | 代码位置 | 职责 |
|------|---------|------|
| SRV_Channel | `libraries/SRV_Channel/` | 舵机通道管理 |
| AP_Motors | `libraries/AP_Motors/` | 电机控制 |
| AP_DroneCAN | `libraries/AP_DroneCAN/` | DroneCAN 输出 |
| AP_HAL PWM | `libraries/AP_HAL_*/` | HAL 层 PWM 输出 |

---

## 2. SRV_Channel 舵机通道

### 2.1 通道函数枚举（`SRV_Channel.h:47+`）

```cpp
enum Aux_servo_function_t {
    k_rcin1 = 0,           // RC 输入直通
    k_rcin2 = 1,
    // ...
    k_aileron = 4,         // 副翼
    k_elevator = 19,       // 升降舵
    k_rudder = 21,         // 方向舵
    k_steering = 26,       // 转向
    k_flap = 2,            // 襟翼
    k_flap_auto = 3,       // 自动襟翼
    k_flaperon_left = 24,  // 左升降副翼
    k_flaperon_right = 25, // 右升降副翼
    k_throttle = 70,       // 油门
    k_throttleLeft = 73,   // 左油门
    k_throttleRight = 74,  // 右油门
    k_motor1 = 33,         // 电机1
    k_motor2 = 34,         // 电机2
    k_motor3 = 35,         // 电机3
    k_motor4 = 36,         // 电机4
    k_motor5 = 37,         // 电机5
    k_motor6 = 38,         // 电机6
    k_motor7 = 39,         // 电机7
    k_motor8 = 40,         // 电机8
    k_motor_tilt = 41,     // 倾转电机 ⚠️ VTOL关键
    k_tiltMotorRear = 45,  // 后倾转电机
    k_tiltMotorLeft = 75,  // 左倾转电机
    k_tiltMotorRight = 76, // 右倾转电机
};
```

### 2.2 SRV_Channel 类（`SRV_Channel.h:37`）

```cpp
class SRV_Channel {
    int16_t servo_min;      // 最小 PWM
    int16_t servo_max;      // 最大 PWM
    int16_t servo_trim;     // 中位 PWM
    bool reversed;          // 反向标志
    Aux_servo_function_t function;  // 通道函数
    int16_t output_pwm;     // 当前输出 PWM
};
```

### 2.3 关键方法

**设置输出**（`SRV_Channel.cpp:set_output_pwm()`）：
```cpp
void SRV_Channel::set_output_pwm(int16_t pwm) {
    output_pwm = constrain_int16(pwm, servo_min, servo_max);
}
```

**设置缩放输出**（`SRV_Channel.cpp:set_output_scaled()`）：
```cpp
void SRV_Channel::set_output_scaled(float value) {
    // value: -1.0 ~ 1.0 或 0.0 ~ 1.0
    int16_t pwm = servo_trim + (value * (servo_max - servo_min) / 2);
    set_output_pwm(pwm);
}
```

---

## 3. SRV_Channels 管理器（`SRV_Channel.h:358`）

### 3.1 静态方法

**批量设置**（`SRV_Channels::set_output_scaled()`）：
```cpp
void SRV_Channels::set_output_scaled(Aux_servo_function_t function, float value) {
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].get_function() == function) {
            channels[i].set_output_scaled(value);
        }
    }
}
```

**电机函数映射**（`SRV_Channels::get_motor_function()`）：
```cpp
Aux_servo_function_t SRV_Channels::get_motor_function(uint8_t motor_num) {
    if (motor_num < 8) {
        return (Aux_servo_function_t)(k_motor1 + motor_num);
    } else {
        return (Aux_servo_function_t)(k_motor9 + (motor_num - 8));
    }
}
```

### 3.2 PWM 计算（`SRV_Channels.cpp:calc_pwm()`）

```cpp
void SRV_Channels::calc_pwm() {
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        // 应用反向
        if (channels[i].is_reversed()) {
            channels[i].reverse_output();
        }
        // 应用故障安全 PWM
        if (failsafe_active) {
            channels[i].set_output_pwm(channels[i].get_failsafe_pwm());
        }
    }
}
```

### 3.3 输出到 HAL（`SRV_Channels.cpp:output_ch_all()`）

```cpp
void SRV_Channels::output_ch_all() {
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        hal.rcout->write(i, channels[i].get_output_pwm());
    }
}
```

---

## 4. AP_Motors 电机控制

### 4.1 多旋翼电机（`AP_MotorsMatrix.cpp`）

**电机掩码输出**（`AP_MotorsMatrix.cpp:output_motor_mask()`）：
```cpp
void AP_MotorsMatrix::output_motor_mask(float throttle, uint32_t mask, float rudder_dt) {
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (mask & (1U<<i)) {
            rc_write(i, throttle_to_pwm(throttle));
        } else {
            rc_write(i, 0);  // 关闭电机
        }
    }
}
```

**VTOL 过渡期间使用**：
- 前向过渡：逐步关闭旋翼电机
- 后向过渡：逐步启动旋翼电机

### 4.2 推力补偿（`AP_MotorsMatrix.cpp:throttle_boost()`）

```cpp
void AP_MotorsMatrix::throttle_boost(float boost) {
    // 单电机失效时提升其他电机推力
    for (uint8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_throttle[i] *= (1.0f + boost);
        }
    }
}
```

⚠️ **Plane QuadPlane 侧较弱**：此功能主要在 Copter 中实现，QuadPlane 需移植。

---

## 5. AP_DroneCAN 输出（`AP_DroneCAN.cpp`）

### 5.1 ESC 输出（`AP_DroneCAN.cpp:824-871`）

**SRV_send_esc()**：
```cpp
void AP_DroneCAN::SRV_send_esc() {
    // 即使 disarmed 也发送零值 ⚠️
    if (armed) {
        esc_cmd.throttle[i] = SRV_Channels::get_output_scaled(k_motor1 + i);
    } else {
        esc_cmd.throttle[i] = 0;  // 但消息照发
    }
    canardBroadcast(&esc_cmd);
}
```

⚠️ **冗余系统关键问题**：从机 disarmed 时仍发送零值，会与主机抢 ESC。

**修复方案**（`redundancy_feasibility.md` §3.4）：
```cpp
void AP_DroneCAN::SRV_send_esc() {
    if (standby_mode) {
        return;  // 从机静默，不发送
    }
    // ... 原有逻辑
}
```

### 5.2 ArmingStatus 广播（`AP_DroneCAN.cpp:1257-1261`）

```cpp
void AP_DroneCAN::send_arming_status() {
    ArmingStatus msg;
    msg.status = hal.util->get_soft_armed() ? 1 : 0;
    canardBroadcast(&msg);
}
```

AP_Periph 接收后：
```cpp
// Tools/AP_Periph/can.cpp:517-524
void handle_arming_status(const ArmingStatus &msg) {
    hal.util->set_soft_armed(msg.status);
}
```

### 5.3 DroneCAN 线程（`AP_DroneCAN.cpp:503`）

```cpp
void AP_DroneCAN::thread_create() {
    hal.scheduler->thread_create(
        FUNCTOR_BIND_MEMBER(&AP_DroneCAN::can_thread, void),
        "DroneCAN", 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0);
}
```

**关键**：CAN 收发在独立线程，不受主循环阻塞影响。

---

## 6. HAL 层 PWM 输出

### 6.1 ChibiOS 后端（`AP_HAL_ChibiOS/RCOutput.cpp`）

```cpp
void RCOutput::write(uint8_t ch, uint16_t period_us) {
    // 设置定时器比较值
    pwm_set_period(ch, period_us);
}
```

### 6.2 SITL 后端（`AP_HAL_SITL/RCOutput.cpp`）

```cpp
void RCOutput::write(uint8_t ch, uint16_t period_us) {
    // 写入共享内存，供 SITL 物理模型读取
    sitl_state->servos[ch] = period_us;
}
```

---

## 7. 倾转舵机控制

### 7.1 k_motor_tilt 通道

**设置倾转角**（`tiltrotor.cpp:187-196`）：
```cpp
void Tiltrotor::slew(float newtilt) {
    current_tilt = constrain_float(newtilt, ...);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}
```

**通道配置**：
- `SERVOx_FUNCTION = 136`（Motor Tilt）
- `SERVOx_FUNCTION = 137`（Tilt Motor Left）
- `SERVOx_FUNCTION = 138`（Tilt Motor Right）

### 7.2 矢量偏航通道

**左右差动**（`tiltrotor.cpp:550-662`）：
```cpp
void Tiltrotor::vectoring() {
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt);
}
```

---

## 8. 故障安全 PWM

### 8.1 设置故障安全值（`SRV_Channels::set_failsafe_pwm()`）

```cpp
void SRV_Channels::set_failsafe_pwm(Aux_servo_function_t function, int16_t pwm) {
    for (uint8_t i=0; i<NUM_SERVO_CHANNELS; i++) {
        if (channels[i].get_function() == function) {
            channels[i].set_failsafe_pwm(pwm);
        }
    }
}
```

### 8.2 触发时机

- RC 信号丢失
- GCS 心跳丢失
- EKF 故障
- 手动触发

---

## 9. 输出混控

### 9.1 升降副翼混控（`ArduPlane/servos.cpp`）

```cpp
void Plane::servo_output_mixing() {
    // 升降副翼：elevator + aileron
    float elevon_left = elevator + aileron;
    float elevon_right = elevator - aileron;
    
    SRV_Channels::set_output_scaled(k_flaperon_left, elevon_left);
    SRV_Channels::set_output_scaled(k_flaperon_right, elevon_right);
}
```

### 9.2 VTOL 混控（`tiltrotor.cpp:tilt_compensate_angle()`）

过渡期间混合固定翼舵面 + 旋翼推力：
- 滚转：副翼 + 旋翼差动
- 俯仰：升降舵 + 旋翼前后差动
- 偏航：方向舵 + 旋翼反扭矩

---

## 10. 关键参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `SERVOx_FUNCTION` | 通道函数 | 0 |
| `SERVOx_MIN` | 最小 PWM | 1100 |
| `SERVOx_MAX` | 最大 PWM | 1900 |
| `SERVOx_TRIM` | 中位 PWM | 1500 |
| `SERVOx_REVERSED` | 反向 | 0 |
| `SERVOx_FAILSAFE` | 故障安全 PWM | 0 |
| `Q_TILT_MASK` | 倾转电机掩码 | 0 |
| `Q_TILT_TYPE` | 倾转类型 | 0 |

---

## 11. 协议要求覆盖

| 协议要求 | APM 实现 | 状态 |
|---------|---------|------|
| 舵面控制 | SRV_Channel + 混控 | ✅ |
| 电机控制 | AP_Motors + PWM | ✅ |
| 倾转控制 | k_motor_tilt + slew | ✅ |
| DroneCAN 输出 | AP_DroneCAN ESC | ✅ |
| 故障安全 | failsafe_pwm | ✅ |
| 冗余输出 | ⚠️ 从机静默需修复 | ⚠️ |

---

## 12. 二次开发接口

### 12.1 设置舵机

```cpp
SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, 0.5f);  // 50% 上偏
SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, -0.3f); // 30% 左偏
```

### 12.2 设置电机

```cpp
SRV_Channels::set_output_scaled(SRV_Channel::k_motor1, 0.8f);  // 80% 油门
SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 0.6f); // 60% 倾转
```

### 12.3 读取输出

```cpp
int16_t pwm = SRV_Channels::get_output_pwm(SRV_Channel::k_elevator);
float scaled = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
```

