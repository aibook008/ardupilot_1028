# QuadPlane / VTOL 模块代码通读

> 适用分支：`qhz/plane_4.6.3`
> 关联文档：[VTOL架构总览](vtol_tiltrotor_architecture.md) | [技术协议摘要](technical_agreement_summary.md)

---

## 1. 类层次总览

```
Plane (ArduPlane 主类)
  └─ QuadPlane (VTOL 管理器)           ← ArduPlane/quadplane.h:34
       ├─ Tiltrotor (倾转旋翼控制)      ← ArduPlane/tiltrotor.h
       ├─ Tailsitter (尾坐式控制)       ← ArduPlane/tailsitter.h
       ├─ Transition* (过渡逻辑基类)     ← ArduPlane/transition.h
       │    ├─ SLT_Transition           ← ArduPlane/quadplane.cpp:1466
       │    ├─ Tiltrotor_Transition     ← ArduPlane/tiltrotor.h
       │    └─ Tailsitter_Transition    ← ArduPlane/tailsitter.h:158
       └─ VTOL_Assist (辅助飞行)        ← ArduPlane/VTOL_Assist.h
```

**推力构型枚举**（`ArduPlane/quadplane.h:80-84`）：
```cpp
enum class ThrustType : uint8_t {
    SLT=0,        // Separate Lift Thrust（传统四轴+推拉力）
    TAILSITTER,   // 尾坐式
    TILTROTOR,    // 倾转旋翼
};
```

---

## 2. QuadPlane 管理器（`ArduPlane/quadplane.h/cpp`）

### 2.1 类定义（`quadplane.h:34`）

核心成员：
- `Tiltrotor tiltrotor` — 倾转控制实例
- `Tailsitter tailsitter` — 尾坐控制实例
- `Transition *transition` — 过渡逻辑指针（运行时指向具体实现）
- `VTOL_Assist assist` — 辅助飞行决策

### 2.2 主循环入口 `QuadPlane::update()`（`quadplane.cpp:1698`）

每主循环迭代调用一次。核心逻辑：
1. 判断当前是否在 VTOL 模式 → `in_vtol_mode()`
2. 若在固定翼模式 → 调用 `transition->update()` 处理过渡
3. 若在 VTOL 模式 → 调用 `transition->VTOL_update()` 准备下次过渡
4. `tiltrotor.update()` 独立运行，管理舵机位置（`quadplane.cpp:1780`）

### 2.3 VTOL 模式判断 `in_vtol_mode()`

返回 true 当当前飞行模式为 Q 系列（QSTABILIZE/QHOVER/QLOITER/QLAND/QTAKEOFF/QAUTOTUNE）或处于过渡状态。

---

## 3. Tiltrotor 倾转控制（`ArduPlane/tiltrotor.h/cpp`）

### 3.1 参数表（`tiltrotor.cpp:5-86`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Q_TILT_MASK` | 0 | 倾转电机位掩码 |
| `Q_TILT_TYPE` | 0 | 0=连续, 1=二值, 2=矢量偏航, 3=双旋翼 |
| `Q_TILT_RATE_UP` | 40°/s | 水平→垂直最大倾转速率 |
| `Q_TILT_RATE_DN` | 0°/s | 垂直→水平（0=同UP） |
| `Q_TILT_MAX` | 45° | 多旋翼控制允许最大倾转角 |
| `Q_TILT_YAW_ANGLE` | 0° | 矢量偏航最小倾转角 |
| `Q_TILT_FIX_ANGLE` | 0° | 固定翼前倾角 |

### 3.2 速率限制 `slew()`（`tiltrotor.cpp:187-196`）

```cpp
void Tiltrotor::slew(float newtilt) {
    float max_change = tilt_max_change(newtilt < current_tilt);
    current_tilt = constrain_float(newtilt,
                                   current_tilt - max_change,
                                   current_tilt + max_change);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor_tilt, 1000 * current_tilt);
}
```

### 3.3 Continuous 连续倾转（`tiltrotor.cpp:214-333`）

- 纯固定翼模式：倾转到前进位置 `get_forward_flight_tilt()`
- VTOL 模式：根据前向推力需求计算倾转角 `atanf(fwd_g_demand)`

### 3.4 Binary 二值倾转（`tiltrotor.cpp:356-375`）

只有两个位置（完全垂直/完全水平），用于收放式舵机。

### 3.5 VectoredYaw 矢量偏航（`tiltrotor.cpp:550-662`）

左右电机差动倾转：
- 悬停时：差动偏航控制
- 前飞时：差动滚转/俯仰（类似升降副翼混合）

### 3.6 Bicopter 双旋翼（`tiltrotor.cpp:667-760`）

两个倾转旋翼+尾翼，必须使用 tailsitter 机架类(10)。

### 3.7 倾转补偿 `tilt_compensate_angle()`（`tiltrotor.cpp:451-497`）

**核心算法**：电机倾转时差动推力对滚转/偏航的控制效力变化补偿。

```
1. 对倾转电机施加倾斜因子
2. 滚转效力随倾转角衰减：sin_tilt * yaw_gain
3. 偏航差动推力补偿
4. 归一化防饱和
```

过渡到前飞 `tilt_compensate_down`：
```cpp
float inv_tilt_factor = 1.0 / cosf(radians(current_tilt * 90));
tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
```

过渡到悬停 `tilt_compensate_up`：
```cpp
const float tilt_factor = cosf(radians(current_tilt * 90));
tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
```

---

## 4. 过渡状态机（`ArduPlane/quadplane.cpp:1466-1671`）

### 4.1 SLT_Transition::update() 三阶段

```cpp
enum {
    TRANSITION_AIRSPEED_WAIT,  // 等空速达到 AIRSPEED_MIN
    TRANSITION_TIMER,          // 逐步减小多旋翼推力
    TRANSITION_DONE            // 纯固定翼模式
};
```

**状态 1：TRANSITION_AIRSPEED_WAIT**
- 保持悬停姿态 `hold_hover(climb_rate_cms)`
- 等待空速 > AIRSPEED_MIN
- 倾转电机逐步前倾
- 重置固定翼 PID 积分项
- 禁用偏航控制

失败处理：
```cpp
if ((now - transition_start_ms) > transition_failure.timeout * 1000) {
    if (tiltrotor && groundspeed > AIRSPEED_MIN * 0.5) {
        transition_state = TRANSITION_TIMER;  // 强制继续
        in_forced_transition = true;
    } else {
        plane.set_mode(plane.mode_qland, ModeReason::VTOL_FAILED_TRANSITION);
    }
}
```

**状态 2：TRANSITION_TIMER**
- 逐步衰减多旋翼油门
- `Q_TRANS_TIME_MS` 控制衰减时间（默认5000ms）

```cpp
float transition_scale = (trans_time_ms - transition_timer_ms) / trans_time_ms;
float throttle_scaled = last_throttle * transition_scale;
quadplane.hold_stabilize(throttle_scaled);
```

**状态 3：TRANSITION_DONE**
- 关闭多旋翼电机（SHUT_DOWN）
- 固定翼完全接管

### 4.2 Tailsitter_Transition（`tailsitter.h:158-221`）

独立于 SLT_Transition，三阶段：
1. `TRANSITION_ANGLE_WAIT_FW`（:841-865）：rate-limits 俯仰从 VTOL 到 FW
2. `TRANSITION_ANGLE_WAIT_VTOL`（:868-869）：由 FW 姿态控制器处理
3. `TRANSITION_DONE`（:872-873）：no-op

### 4.3 Tiltrotor_Transition

继承自 `SLT_Transition`，在过渡过程中加入倾转舵机控制逻辑。

---

## 5. Tailsitter 尾坐控制（`ArduPlane/tailsitter.h/cpp`）

### 5.1 setup()（`tailsitter.cpp:200-253`）

- 根据 frame class 或电机掩码启用
- 自动计算 `transition_rate_fw`
- 检测可用控制面（升降舵/副翼/方向舵/升降副翼/V尾）
- `enable == 2`：纯电机控制（无控制面）

### 5.2 output()（`tailsitter.cpp:285-520`）

- 前飞/VTOL过渡：FW 控制器油门 + 电机掩码
- VTOL 模式：完整多旋翼输出路径
- 矢量悬停增益混合

### 5.3 speed_scaling()（`tailsitter.cpp:638-795`）

计算油门和速度标量，用于控制面权威度缩放。组合多种缩放方式：油门、姿态+油门、桨盘理论、高度。

---

## 6. VTOL_Assist 辅助飞行（`ArduPlane/VTOL_Assist.h/cpp`）

判断何时需要旋翼辅助固定翼飞行（如低速时自动启动旋翼）。关键参数：
- `Q_ASSIST_SPEED`：低于此空速时启动辅助
- `Q_ASSIST_ALT`：低于此高度时启动辅助
- `Q_ASSIST_DELAY`：辅助启动延迟时间

---

## 7. 舵机输出映射

| 通道函数 | 枚举值 | 代码位置 |
|---------|--------|---------|
| `k_motor_tilt` | 41 | `SRV_Channel.h:90` |
| `k_tiltMotorLeft` | 75 | `SRV_Channel.h:119` |
| `k_tiltMotorRight` | 76 | `SRV_Channel.h:120` |
| `k_tiltMotorRear` | 45 | `SRV_Channel.h:92` |

配置方式：`SERVOx_FUNCTION = 136`（Motor Tilt）

---

## 8. 关键函数调用链

```
Plane::update_flight_mode()
  → QuadPlane::update()                    // quadplane.cpp:1698
    → SLT_Transition::update()             // quadplane.cpp:1466-1671
      → Tiltrotor::update()                // tiltrotor.cpp
        → continuous_update()              // :214-333
          → slew(newtilt)                  // :187-196
            → SRV_Channels::set_output_scaled(k_motor_tilt, ...)
```

---

## 9. 文件索引

| 文件 | 核心功能 |
|------|---------|
| `ArduPlane/quadplane.h/cpp` | VTOL管理器，过渡状态机，电机混合 |
| `ArduPlane/tiltrotor.h/cpp` | 倾转控制，速率限制，补偿算法 |
| `ArduPlane/transition.h` | 过渡状态枚举 |
| `ArduPlane/tailsitter.h/cpp` | 尾坐式过渡逻辑 |
| `ArduPlane/VTOL_Assist.h/cpp` | 辅助飞行判据 |
| `ArduPlane/servos.cpp` | 舵机输出映射 |
| `libraries/AP_Motors/AP_MotorsMatrix.cpp` | 电机混合 |
