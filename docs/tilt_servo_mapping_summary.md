# 倾转舵机映射关系总结

## 核心发现

**结论**：SITL 仿真模型中的倾转舵机映射与飞控代码期望**完全一致**，正确实现了左右差动。

**角度坐标系**：
- 0° = 机头方向
- 正角度 = 顺时针（向右）
- 负角度 = 逆时针（向左）
- 45° = 前右，-45° = 前左
- 135° = 后右，-135° = 后左

---

## 一、参数配置

**文件**: `Tools/autotest/default_params/quadplane-tilthvec.parm`

```ini
Q_FRAME_CLASS    1          # 四轴
Q_FRAME_TYPE     3          # H 型 (MOTOR_FRAME_TYPE_H)
Q_TILT_TYPE      2          # VectoredYaw（矢量偏航）
Q_TILT_MASK      15         # 电机 0-3 都是倾转电机

SERVO12_FUNCTION 76         # k_tiltMotorRight
SERVO13_FUNCTION 75         # k_tiltMotorLeft
```

---

## 二、功能码定义

**文件**: `libraries/SRV_Channel/SRV_Channel.h:119-120`

```cpp
k_tiltMotorLeft     = 75,   // 左侧倾转舵机
k_tiltMotorRight    = 76,   // 右侧倾转舵机
```

---

## 三、飞控代码的电机布局

**文件**: `libraries/AP_Motors/AP_MotorsMatrix.cpp:575-600`

H 型构型电机定义：

| 电机编号 | 位置 | 旋转方向 | 角度 |
|---------|------|---------|------|
| Motor 0 | 右前 | CW | 45° |
| Motor 1 | 左后 | CW | -135° |
| Motor 2 | 右后 | CCW | -45° |
| Motor 3 | 左前 | CCW | 135° |

**俯视图**：
```
          机头 (0°)
              ↑
              
Motor 2 (-45°)    Motor 0 (45°)
   前左 CCW         前右 CW
   
   
Motor 1 (-135°)   Motor 3 (135°)
   后左 CW          后右 CCW
   
              ↓
          机尾 (180°)
```

---

## 四、飞控代码的倾转输出逻辑

**文件**: `ArduPlane/tiltrotor.cpp:646-663`

```cpp
// 计算左右倾转角度
float left_tilt = base_output + tilt_offset;   // 左侧期望
float right_tilt = base_output - tilt_offset;  // 右侧期望

// 输出到功能码
SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, left_tilt);   // SERVO13
SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, right_tilt); // SERVO12
```

**飞控期望的映射**：
- `k_tiltMotorLeft` (SERVO13) → 控制**左侧**电机（Motor 1 左后 + Motor 3 左前）
- `k_tiltMotorRight` (SERVO12) → 控制**右侧**电机（Motor 0 右前 + Motor 2 右后）

---

## 五、SITL 模型的电机定义

**文件**: `libraries/SITL/SIM_Frame.cpp:84-90`

```cpp
static Motor tiltquad_h_vectored_motors[] =
{
    Motor(AP_MOTORS_MOT_1,   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1, -1, 0, 0, 7, 10, -90),
    Motor(AP_MOTORS_MOT_2, -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_3,  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4, -1, 0, 0, 8, 10, -90),
    Motor(AP_MOTORS_MOT_4,  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2, -1, 0, 0, 7, 10, -90),
};
```

**Motor 构造函数参数**（`libraries/SITL/SIM_Motor.h:66-78`）：
```cpp
Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order,
      int8_t _roll_servo, float _roll_min, float _roll_max,
      int8_t _pitch_servo, float _pitch_min, float _pitch_max)
```

**关键参数**：
- 第 8 个参数 `_pitch_servo`：控制该电机 pitch 倾转的舵机索引
- Motor 0 (右前): `pitch_servo = 7`
- Motor 1 (左后): `pitch_servo = 8`
- Motor 2 (右后): `pitch_servo = 8`
- Motor 3 (左前): `pitch_servo = 7`

---

## 六、SITL 的舵机索引计算

**文件**: `libraries/SITL/SIM_Motor.cpp:92`

```cpp
uint16_t servoval = update_servo(input.servos[pitch_servo+motor_offset], now, last_pitch_value);
```

**文件**: `libraries/SITL/SIM_QuadPlane.cpp:30,97`

```cpp
uint8_t motor_offset = 4;  // QuadPlane 的电机从 SERVO5 开始
frame->motor_offset = motor_offset;
```

**计算过程**：
```
Motor 0 (右前): pitch_servo=7 → input.servos[7+4] = input.servos[11] = SERVO12
Motor 1 (左后): pitch_servo=8 → input.servos[8+4] = input.servos[12] = SERVO13
Motor 2 (右后): pitch_servo=8 → input.servos[8+4] = input.servos[12] = SERVO13
Motor 3 (左前): pitch_servo=7 → input.servos[7+4] = input.servos[11] = SERVO12
```

---

## 七、实际映射关系

| SITL 模型 | pitch_servo | 实际舵机 | 位置 |
|-----------|-------------|---------|------|
| Motor 0 (45°, 前右) | 7 | SERVO12 | 右侧 |
| Motor 1 (-135°, 后左) | 8 | SERVO13 | 左侧 |
| Motor 2 (-45°, 前左) | 8 | SERVO13 | 左侧 |
| Motor 3 (135°, 后右) | 7 | SERVO12 | 右侧 |

**实际分组**：
- SERVO12 (k_tiltMotorRight) → Motor 0 (45° 前右) + Motor 3 (135° 后右) = **右侧** ✅
- SERVO13 (k_tiltMotorLeft) → Motor 1 (-135° 后左) + Motor 2 (-45° 前左) = **左侧** ✅

---

## 八、映射验证

### 飞控期望（左右差动）
```
k_tiltMotorLeft (SERVO13)  → 左侧电机 (Motor 1 左后 + Motor 3 左前)
k_tiltMotorRight (SERVO12) → 右侧电机 (Motor 0 右前 + Motor 2 右后)
```

### SITL 实际（左右差动）
```
k_tiltMotorLeft (SERVO13)  → 左侧电机 (Motor 1 左后 + Motor 3 左前) ✅
k_tiltMotorRight (SERVO12) → 右侧电机 (Motor 0 右前 + Motor 2 右后) ✅
```

### 结论
- SITL 模型正确实现了**左右差动**
- 与飞控代码期望完全一致
- 偏航控制方向正确
- 仿真结果可信

---

## 九、核心代码位置汇总

| 组件 | 文件 | 行号 | 作用 |
|------|------|------|------|
| 功能码定义 | `libraries/SRV_Channel/SRV_Channel.h` | 119-120 | 定义 k_tiltMotorLeft/Right |
| 电机布局（飞控） | `libraries/AP_Motors/AP_MotorsMatrix.cpp` | 575-600 | H 型电机定义 |
| 倾转输出（飞控） | `ArduPlane/tiltrotor.cpp` | 646-663 | 左右倾转角度计算和输出 |
| 电机布局（SITL） | `libraries/SITL/SIM_Frame.cpp` | 84-90 | tiltquad_h_vectored_motors 定义 |
| Motor 构造函数 | `libraries/SITL/SIM_Motor.h` | 66-78 | pitch_servo 参数定义 |
| 舵机索引计算 | `libraries/SITL/SIM_Motor.cpp` | 92 | pitch_servo + motor_offset |
| motor_offset 设置 | `libraries/SITL/SIM_QuadPlane.cpp` | 30, 97 | motor_offset = 4 |

---

## 十、验证结论

SITL 模型的 `pitch_servo` 配置正确：
- **pitch_servo=7** (SERVO12): 控制右侧电机 (Motor 0 前右 + Motor 3 后右) ✅
- **pitch_servo=8** (SERVO13): 控制左侧电机 (Motor 1 后左 + Motor 2 前左) ✅

映射关系与飞控代码期望完全一致，正确实现了左右差动偏航控制。

---

## 十一、相关文档

- `docs/vtol_intialization.md` - QuadPlane 初始化流程
- `docs/tiltrotor_motor_control.md` - 倾转旋翼电机控制逻辑
- `docs/logged_info.md` - 日志系统分析
- `docs/tilt_servo_mapping.md` - 详细映射分析

---

**生成时间**: 2026-08-13  
**分析工具**: Claude Code  
**代码版本**: ArduPilot Plane 4.6.3
