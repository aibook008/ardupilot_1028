# QuadPlane 初始化流程详解

以 `Tools/autotest/default_params/quadplane-tilthvec.parm` 为例，追踪从参数加载到 PWM 输出的完整初始化链路。

## 1. 参数文件解读

```ini
# quadplane-tilthvec.parm
Q_ENABLE         1          # 启用 QuadPlane
Q_FRAME_CLASS    1          # 四轴 (MOTOR_FRAME_QUAD)
Q_FRAME_TYPE     3          # X 型 (MOTOR_FRAME_TYPE_X)
Q_RTL_MODE       1          # RTL 模式
Q_TILT_TYPE      2          # VectoredYaw（矢量偏航）
Q_TILT_YAW_ANGLE 10         # 矢量偏航角度
Q_TILT_MASK      15         # 电机 0-3 都是倾转电机（二进制 1111）
Q_VFWD_GAIN      0.1        # 前向推力增益
Q_ANGLE_MAX      4500       # 最大角度 45°
Q_ASSIST_SPEED   6          # VTOL 辅助速度阈值

# 倾转舵机通道（矢量偏航需要左右独立倾转）
SERVO12_FUNCTION 76         # SERVO12 = k_tiltMotorRight (76)
SERVO13_FUNCTION 75         # SERVO13 = k_tiltMotorLeft  (75)
SERVO12_MIN 1000
SERVO12_MAX 2000
SERVO13_MIN 1000
SERVO13_MAX 2000

# PID 参数
Q_A_RAT_RLL_P 0.15
Q_A_RAT_PIT_P 0.15
Q_A_RAT_RLL_D 0.002
Q_A_RAT_PIT_D 0.002
```

## 2. 初始化入口

```
Plane::init_ardupilot()                    // ArduPlane/system.cpp
  → quadplane.setup()                      // system.cpp:106
```

`QuadPlane::setup()` 是 QuadPlane 的核心初始化函数（`quadplane.cpp:682-861`）。

## 3. QuadPlane::setup() 详细流程

### 3.1 前置检查

```cpp
// quadplane.cpp:684-694
if (initialised) return true;              // 已初始化则跳过
if (!enable || hal.util->get_soft_armed()) return false;  // Q_ENABLE=0 或已解锁则不初始化

// 内存检查
if (hal.util->available_memory() < 4096 + sizeof(*motors) + ...) {
    AP_BoardConfig::config_error("Not enough memory for quadplane");
}
```

**参数对应**：`Q_ENABLE = 1` → 通过检查，继续初始化。

### 3.2 设置电机默认通道

```cpp
// quadplane.cpp:701-704
switch ((AP_Motors::motor_frame_class)frame_class) {
case AP_Motors::MOTOR_FRAME_QUAD:          // Q_FRAME_CLASS = 1
    setup_default_channels(4);             // 4 个电机
    break;
```

`setup_default_channels(4)` 的实现：

```cpp
// quadplane.cpp:674-679
void QuadPlane::setup_default_channels(uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        SRV_Channels::set_aux_channel_default(
            SRV_Channels::get_motor_function(i),   // k_motor1+i
            CH_5+i                                  // 默认通道 5-8
        );
    }
}
```

**效果**：
- Motor 0 → k_motor1 (33) → SERVO5
- Motor 1 → k_motor2 (34) → SERVO6
- Motor 2 → k_motor3 (35) → SERVO7
- Motor 3 → k_motor4 (36) → SERVO8

这些是默认映射，参数文件没有显式配置 SERVO5-8 的 function，所以使用默认值。

### 3.3 动态分配核心对象

```cpp
// quadplane.cpp:755-758
motors = NEW_NOTHROW AP_MotorsMatrix(rc_speed);   // Q_FRAME_CLASS=QUAD → AP_MotorsMatrix
motors_var_info = AP_MotorsMatrix::var_info;

// quadplane.cpp:768
ahrs_view = ahrs.create_view(ROTATION_NONE, ahrs_trim_pitch);

// quadplane.cpp:773
attitude_control = NEW_NOTHROW AC_AttitudeControl_TS(*ahrs_view, aparm, *motors);

// quadplane.cpp:779
pos_control = NEW_NOTHROW AC_PosControl(*ahrs_view, inertial_nav, *motors, *attitude_control);

// quadplane.cpp:784
wp_nav = NEW_NOTHROW AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);

// quadplane.cpp:790
loiter_nav = NEW_NOTHROW AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
```

每个对象创建后都调用 `AP_Param::load_object_from_eeprom()` 从 EEPROM 加载参数。

### 3.4 电机初始化

```cpp
// quadplane.cpp:802-804
motors->init(frame_class, frame_type);       // 设置电机布局
motors->update_throttle_range();             // 更新油门 PWM 范围
motors->set_update_rate(rc_speed);           // 设置更新频率 (Q_RC_SPEED=490)
```

`motors->init(QUAD, X)` 调用 `setup_quad_matrix(MOTOR_FRAME_TYPE_X)`：

```cpp
// AP_MotorsMatrix.cpp:591-599
case MOTOR_FRAME_TYPE_X: {
    static const AP_MotorsMatrix::MotorDef motors[] {
        {  45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },   // Motor 0: 右前, CCW
        { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },   // Motor 1: 左后, CCW
        {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },   // Motor 2: 右后, CW
        {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },   // Motor 3: 左前, CW
    };
    add_motors(motors, ARRAY_SIZE(motors));
}
```

**电机布局**（俯视图）：

```
    机头方向 →
  Motor3(CW)    Motor0(CCW)
      \          /
       \        /
        中心
       /        \
      /          \
  Motor1(CCW)   Motor2(CW)
```

每个电机的混合因子（roll/pitch/yaw）由角度和旋转方向自动计算。

### 3.5 倾转旋翼初始化

```cpp
// quadplane.cpp:839
tiltrotor.setup();
```

`Tiltrotor::setup()`（`tiltrotor.cpp:98-154`）：

```cpp
void Tiltrotor::setup()
{
    // 1. 自动启用（如果配置了 tilt_mask）
    if (!enable.configured() && ((tilt_mask != 0) || (type == TILT_TYPE_BICOPTER))) {
        enable.set_and_save(1);
    }
    if (enable <= 0) return;

    // 2. 设置推力类型
    quadplane.thrust_type = QuadPlane::ThrustType::TILTROTOR;

    // 3. 检测矢量偏航
    _is_vectored = tilt_mask != 0 && type == TILT_TYPE_VECTORED_YAW;
    // Q_TILT_MASK=15, Q_TILT_TYPE=2 → _is_vectored = true

    // 4. 检测构型
    _have_fw_motor = SRV_Channels::function_assigned(SRV_Channel::k_throttle) || ...;
    // 参数文件没有配置 k_throttle → _have_fw_motor = false

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; ++i) {
        if (motors->is_motor_enabled(i) && !is_motor_tilting(i)) {
            _have_vtol_motor = true;
            break;
        }
    }
    // tilt_mask=15 (1111), 电机 0-3 都在 mask 中 → _have_vtol_motor = false
    // 结论：全倾转构型（All motors tilting）

    // 5. 矢量偏航：禁用偏航力矩
    if (_is_vectored) {
        motors->disable_yaw_torque();
    }

    // 6. 设置倾转补偿回调
    if (tilt_mask != 0) {
        motors->set_thrust_compensation_callback(
            FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t)
        );
    }

    // 7. 设置矢量偏航舵机范围
    if (type == TILT_TYPE_VECTORED_YAW) {
        SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);  // 75
        SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);  // 76
    }

    // 8. 创建过渡状态机
    transition = NEW_NOTHROW Tiltrotor_Transition(quadplane, motors, *this);
    quadplane.transition = transition;
}
```

### 3.6 完成初始化

```cpp
// quadplane.cpp:849-859
wp_nav->wp_and_spline_init();
transition->force_transition_complete();
AP_Param::invalidate_count();

gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane initialised, %s", frame_and_type_string);
initialised = true;
```

## 4. 通道与 PWM 输出的链接

### 4.1 SRV_Channel 系统

ArduPilot 的舵机/电机输出通过 `SRV_Channel` 系统管理。每个通道有一个 `function`（功能码），决定它控制什么。

**关键功能码**（`SRV_Channel.h`）：

| 功能码 | 名称 | 说明 |
|---|---|---|
| 33-44 | k_motor1 - k_motor12 | 多旋翼电机 |
| 41 | k_motor_tilt | 倾转舵机（统一控制） |
| 70 | k_throttle | 固定翼油门 |
| 75 | k_tiltMotorLeft | 左侧倾转舵机（矢量偏航） |
| 76 | k_tiltMotorRight | 右侧倾转舵机（矢量偏航） |

### 4.2 参数到通道的映射

参数文件中的 `SERVOx_FUNCTION` 直接设置通道的 function：

```ini
SERVO12_FUNCTION 76    # SERVO12 的 function = k_tiltMotorRight
SERVO13_FUNCTION 75    # SERVO13 的 function = k_tiltMotorLeft
```

电机通道（SERVO5-8）没有显式配置，使用 `setup_default_channels()` 设置的默认值：

```
SERVO5  → k_motor1 (33) → Motor 0
SERVO6  → k_motor2 (34) → Motor 1
SERVO7  → k_motor3 (35) → Motor 2
SERVO8  → k_motor4 (36) → Motor 3
```

### 4.3 PWM 输出流程

```
1. 控制器计算输出
   motors->output()
     → output_armed_stabilizing()   // 混合矩阵
     → thrust_compensation()        // 倾转补偿
     → output_to_motors()           // 转换为 actuator 值 (0-1)
       → rc_write(i, pwm)           // 写入 SRV_Channel

2. SRV_Channel 处理
   SRV_Channel::set_output_pwm(pwm)
     → output_pwm = pwm
     → have_pwm_mask |= (1U << ch_num)

3. 最终输出
   SRV_Channels::calc_pwm()         // 所有通道
     → SRV_Channel::calc_pwm(output_scaled)
       → output_pwm = pwm_from_scaled_value(output_scaled)

   SRV_Channels::output_ch_all()
     → hal.rcout->write(pwm_values) // 输出到硬件 PWM

   或 DroneCAN 路径：
   AP_DroneCAN::SRV_push_servos()
     → SRV_send_esc()               // 发送 CAN 消息
```

### 4.4 倾转舵机输出

矢量偏航模式下，左右倾转舵机独立控制：

```cpp
// tiltrotor.cpp:195
SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
```

`k_motor_tilt` (41) 是统一的倾转控制通道。在矢量偏航模式下，左右舵机通过差动实现偏航：

```
左舵机角度 = base_tilt + yaw_offset
右舵机角度 = base_tilt - yaw_offset
```

具体实现在 `Tiltrotor::tilt_compensate()` 和矢量偏航逻辑中。

## 5. 初始化总结

### 5.1 对象创建顺序

```
1. AP_MotorsMatrix          (电机混合矩阵)
2. AP_AHRS_View             (姿态视图)
3. AC_AttitudeControl_TS    (姿态控制器)
4. AC_PosControl            (位置控制器)
5. AC_WPNav                 (航点导航)
6. AC_Loiter                (悬停导航)
7. AC_WeatherVane           (风标效应)
8. Tiltrotor_Transition     (过渡状态机)
```

### 5.2 参数加载顺序

```
1. QuadPlane 参数 (Q_xxx)
   → AP_Param::load_object_from_eeprom(quadplane, var_info)

2. 电机参数 (MOT_xxx)
   → AP_Param::load_object_from_eeprom(motors, motors_var_info)

3. 姿态控制参数 (Q_A_xxx)
   → AP_Param::load_object_from_eeprom(attitude_control, var_info)

4. 位置控制参数 (Q_P_xxx)
   → AP_Param::load_object_from_eeprom(pos_control, var_info)

5. 导航参数 (Q_WP_xxx)
   → AP_Param::load_object_from_eeprom(wp_nav, var_info)
```

### 5.3 通道配置总结

| 通道 | Function | 参数配置 | 说明 |
|---|---|---|---|
| SERVO5 | k_motor1 (33) | 默认 | Motor 0 (右前) |
| SERVO6 | k_motor2 (34) | 默认 | Motor 1 (左后) |
| SERVO7 | k_motor3 (35) | 默认 | Motor 2 (右后) |
| SERVO8 | k_motor4 (36) | 默认 | Motor 3 (左前) |
| SERVO12 | k_tiltMotorRight (76) | SERVO12_FUNCTION=76 | 右侧倾转舵机 |
| SERVO13 | k_tiltMotorLeft (75) | SERVO13_FUNCTION=75 | 左侧倾转舵机 |

### 5.4 构型识别结果

基于参数文件：
- `_is_vectored = true` (Q_TILT_MASK=15, Q_TILT_TYPE=2)
- `_have_fw_motor = false` (没有 k_throttle 通道)
- `_have_vtol_motor = false` (所有电机都在 tilt_mask 中)
- **结论**：全倾转矢量偏航构型

## 6. 关键代码位置

| 组件 | 文件 | 行号 |
|---|---|---|
| QuadPlane::setup() | ArduPlane/quadplane.cpp | 682-861 |
| Tiltrotor::setup() | ArduPlane/tiltrotor.cpp | 98-154 |
| AP_MotorsMatrix::init() | libraries/AP_Motors/AP_MotorsMatrix.cpp | 23-39 |
| setup_quad_matrix() | libraries/AP_Motors/AP_MotorsMatrix.cpp | 575-670 |
| SRV_Channel::calc_pwm() | libraries/SRV_Channel/SRV_Channel.cpp | 213-234 |
| SRV_Channels::output_ch_all() | libraries/SRV_Channel/SRV_Channels.cpp | - |

## 7. 相关文档

- `docs/tiltrotor_motor_control.md`：倾转旋翼电机控制逻辑
- `docs/redundancy_feasibility.md`：多余度飞控调研
- `CLAUDE.md`：项目整体架构
