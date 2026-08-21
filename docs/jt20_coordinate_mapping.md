# JT-20 八旋翼倾转：Gazebo → ArduPilot SITL 坐标系换算

> 源模型：`jt_20.sdf`（Gazebo）。  
> 目标：ArduPilot 飞控参数 + `libraries/SITL` 仿真电机表（后续实现用）。  
> 本文只做**坐标与编号映射**，不改代码。

---

## 1. 坐标系约定

### 1.1 Gazebo（本 SDF）

| 轴 | 方向 |
|----|------|
| X | 机头向前 |
| Y | **向左** |
| Z | **向上** |

`pose` 格式：`x y z roll pitch yaw`（相对 `base_link`）。

### 1.2 ArduPilot 机体轴（飞控 / SITL Motor）

| 轴 | 方向 |
|----|------|
| X | 机头向前 |
| Y | **向右** |
| Z | **向下** |

SITL `Motor` 安装角：`angle` 为从机头起、**顺时针为正**（度），在机体 XY 平面上：

```
angle = atan2(Y_apm, X_apm) * 180/π
```

`yaw_factor`：正 = CW（俯视顺时针），负 = CCW（与 `AP_MOTORS_MATRIX_YAW_FACTOR_CW/CCW` 一致）。

### 1.3 位置换算公式

同一物理点：

```
X_apm =  X_gz
Y_apm = -Y_gz
Z_apm = -Z_gz
```

姿态角若也要换（本机电机安装主要用位置）：

```
roll_apm  =  roll_gz
pitch_apm = -pitch_gz
yaw_apm   = -yaw_gz
```

（绕各自轴向右手定则；Y/Z 反向后 pitch/yaw 符号翻转。）

---

## 2. 构型结论（按 SDF，不是口头“外四固定”的另一种理解）

| 组 | SDF 实体 | 数量 | 能力 |
|----|----------|------|------|
| **内圈倾转** | `motor_0`…`motor_3` + `rotor_0`…`rotor_3` | 4 | 相对机体绕 **Y** 倾转（`motor_*_joint`，`xyz=0 1 0`） |
| **外圈固定** | `rotor_4`…`rotor_7`（无 motor tilt link） | 4 | 推力固定沿机体 **+Z_gz（向上）**，即 APM **−Z** |

与口头「外四固定、中间倾转」一致：**内圈倾转、外圈固定垂起**。

---

## 3. Gazebo 原始 pose（相对 base_link）

| ID | 实体 | pose (x,y,z) [m] | 转向 (SDF) | 倾转 |
|----|------|------------------|------------|------|
| 0 | motor_0 / rotor_0 | ( 0.365, −0.395, −0.02) / z≈0 | ccw | 有 |
| 1 | motor_1 / rotor_1 | (−0.570,  0.395, −0.02) | ccw | 有 |
| 2 | motor_2 / rotor_2 | ( 0.360,  0.395, −0.02) | cw | 有 |
| 3 | motor_3 / rotor_3 | (−0.570, −0.395, −0.02) | cw | 有 |
| 4 | rotor_4 | ( 0.240, −0.885,  0) | ccw | 无 |
| 5 | rotor_5 | (−0.450,  0.885,  0) | ccw | 无 |
| 6 | rotor_6 | ( 0.240,  0.885,  0) | cw | 无 |
| 7 | rotor_7 | (−0.450, −0.885,  0) | cw | 无 |

Gazebo 俯视（X 前、Y 左）：

```
              X_gz 前
                ↑
     fix_5 ·         · fix_6     (Y大，靠外)
     tilt_1·         · tilt_2
Y左 ←――――――――――――――――→ Y右(−)
     tilt_3·         · tilt_0
     fix_7 ·         · fix_4
                ↓
              机尾
```

（表中 Y 正为左，故左侧是 +Y。）

---

## 4. 换算到 APM 机体轴

| Gazebo ID | APM 位置 (X,Y,Z) [m] | 安装角 ° | 象限 | SDF 转向 | APM yaw_factor |
|-----------|----------------------|----------|------|----------|----------------|
| 0 倾转 | ( 0.365,  0.395,  0.020) | **+47.3** | 前右 | ccw | **CCW (−)** |
| 1 倾转 | (−0.570, −0.395,  0.020) | **−145.3** | 后左 | ccw | **CCW (−)** |
| 2 倾转 | ( 0.360, −0.395,  0.020) | **−47.7** | 前左 | cw | **CW (+)** |
| 3 倾转 | (−0.570,  0.395,  0.020) | **+145.3** | 后右 | cw | **CW (+)** |
| 4 固定 | ( 0.240,  0.885,  0.000) | **+74.8** | 前右外 | ccw | **CCW (−)** |
| 5 固定 | (−0.450, −0.885,  0.000) | **−117.0** | 后左外 | ccw | **CCW (−)** |
| 6 固定 | ( 0.240, −0.885,  0.000) | **−74.8** | 前左外 | cw | **CW (+)** |
| 7 固定 | (−0.450,  0.885,  0.000) | **+117.0** | 后右外 | cw | **CW (+)** |

APM 俯视（X 前、Y 右）：

```
              X_apm 前
                ↑
     fix_6 ·         · fix_4
     tilt_2·         · tilt_0
Y左(−) ←―――――――――――――→ Y右(+)
     tilt_1·         · tilt_3
     fix_5 ·         · fix_7
                ↓
              机尾
```

SITL 里常用整数角近似（与现有 `tilthvec` 风格一致）：

| ID | 建议 angle | 说明 |
|----|------------|------|
| 0 | 45° | ≈47.3 |
| 1 | −135° | ≈−145.3（前后臂不等长，角度非严格 135） |
| 2 | −45° | ≈−47.7 |
| 3 | 135° | ≈145.3 |
| 4 | 75° | ≈74.8 |
| 5 | −117° | 或 −120° |
| 6 | −75° | ≈−74.8 |
| 7 | 117° | 或 120° |

精确仿真应用上表浮点角或 `Motor` 位置向量（若走 json model）。

---

## 5. 推力 / 倾转轴方向

### 5.1 悬停推力

- Gazebo：桨盘默认推 **+Z_gz**（向上）。
- APM/SITL：`thrust_vector` 默认 `(0,0,−1)`（NED 向上为负 Z）。  
换算一致：GZ 向上 ↔ APM −Z。

### 5.2 倾转关节（motor_0…3）

- Gazebo：`motor_*_joint` 轴 `0 1 0`（绕 **+Y_gz**，向左轴）。
- APM：绕 **+Y_apm**（向右）与 GZ 的 +Y 反向，故**倾转角符号**在接 SITL `pitch_servo` 时要注意与 PWM 方向一致（常通过 `SERVOx_REVERSED` 或 `pitch_min/max` 对调）。

Gazebo MAVLink 通道（SDF `control_channels`）：

| input_index | 用途 |
|-------------|------|
| 0–7 | rotor_0…7 转速 |
| 8–11 | motor_0…3 倾转位置 |
| 12–15 | 左右 elevon / 左右尾舵 |

---

## 6. 建议的 APM 电机编号（与飞控 `Motor1…8`）

QuadPlane 惯例：`motor_offset = 4` 时，旋翼占用 SERVO5 起的通道；倾转另占功能码 41/75/76/45…。编号建议（便于对照 SDF）：

| APM Motor | 对应 Gazebo | 倾转 | SERVO 功能（建议） |
|-----------|-------------|------|-------------------|
| Motor1 | rotor_0 | 是 | 33 |
| Motor2 | rotor_1 | 是 | 34 |
| Motor3 | rotor_2 | 是 | 35 |
| Motor4 | rotor_3 | 是 | 36 |
| Motor5 | rotor_4 | 否 | 37 |
| Motor6 | rotor_5 | 否 | 38 |
| Motor7 | rotor_6 | 否 | 39 |
| Motor8 | rotor_7 | 否 | 40 |

倾转掩码：`Q_TILT_MASK = 15`（bit0–3 = Motor1–4）。  
外圈固定：不进 mask。

倾转舵机（飞控侧，与现有 tiltrotor 代码对齐）：

| 物理 | 建议功能码 | 备注 |
|------|------------|------|
| 左前/左后倾转（GZ 左侧 = APM −Y） | `k_tiltMotorLeft`=75 等 | 需与 SDF motor 1/2 对应关系再定左右 |
| 右前/右后 | `k_tiltMotorRight`=76 | 同上 |
| 或统一 | `k_motor_tilt`=41 | 无差动偏航矢量时 |

SITL `Motor(..., pitch_servo, pitch_min, pitch_max)` 的 `pitch_servo` 为**相对 `motor_offset` 的通道下标**，必须与 parm 里 `SERVOx_FUNCTION` 一致（参见 `docs/tilt_servo_mapping_summary.md`）。

飞控混控：`Q_FRAME_CLASS=3`（OCTA）+ 合适 `Q_FRAME_TYPE`；现成 octa 表与 JT-20 几何不完全一致，**精确力臂要等自定义 frame**。

---

## 7. 质量 / 惯量（SDF → 参考）

| 量 | SDF | APM 注意 |
|----|-----|----------|
| 机身 mass | 10 kg（`base_link`） | SITL `Frame::init` 里 QuadPlane 还会 `×1.5` 估机身 |
| ixx,iyy,izz | 2.168 / 1.242 / 3.016 | GZ 惯量轴与 APM 轴需按同一变换理解；勿直接混用未换算张量 |
| 重心 | inertial pose (−0.105, 0, −0.08)_gz | APM：(−0.105, 0, 0.08) |

---

## 8. 后续实现清单（本文不做）

1. `SIM_Frame.cpp`：新增 8 电机表（4 带 `pitch_servo`，4 固定）+ `supported_frames` 名如 `jt20`。  
2. `SIM_QuadPlane.cpp`：`strstr(..., "-jt20")` → `frame_type="jt20"`。  
3. `vehicleinfo.py`：`quadplane-jt20` + defaults parm。  
4. parm：`Q_ENABLE`、`Q_FRAME_CLASS=3`、`Q_TILT_MASK=15`、`Q_TILT_TYPE`、SERVO 电机与倾转功能。  
5. 启动：`sim_vehicle.py -v ArduPlane -f quadplane-jt20 -w`。

---

## 9. 换算脚本（可复现）

```python
import math
# Gazebo: X fwd, Y left, Z up → APM: X fwd, Y right, Z down
def gz_to_apm(x, y, z):
    return (x, -y, -z)

def apm_angle_deg(x, y, z):
    xa, ya, _ = gz_to_apm(x, y, z)
    return math.degrees(math.atan2(ya, xa))
```

---

## 10. 一句话

**Gazebo（X前 Y左 Z上）点 (x,y,z) 在 APM（X前 Y右 Z下）为 (x,−y,−z)；安装角 `atan2(Y_apm,X_apm)`。** JT-20：内四倾转、外四固定；转向 cw/ccw 映射为 APM 的 `yaw_factor` ±1。精确 SITL 需新 frame，勿直接套现成 `octa`/`tilthvec` 几何。
