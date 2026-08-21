# JT-20 八旋翼倾转旋翼（SITL）工作梳理

> 对应会话：`5a3024d3-d403-4b01-be75-98038af72256`  
> 代码分支：`qhz/plane_4.6.3`（Plane 4.6.3）  
> 源几何：`jt_20.sdf`（Gazebo）  
> 关联文档：`docs/jt20_coordinate_mapping.md`、`docs/tilt_servo_mapping_summary.md`

---

## 1. 目标构型

| 项目 | 定义 |
|------|------|
| 布局 | 八旋翼：内四可倾转 + 外四固定垂起 |
| 内圈 | Gazebo `motor_0…3` / `rotor_0…3`，绕 Y 轴倾转 |
| 外圈 | Gazebo `rotor_4…7`，推力固定向上（APM −Z） |
| 飞控侧 | QuadPlane + Tiltrotor + 自定义 Lua 混控 |
| 仿真侧 | SITL 内置物理（非 JSBSim），frame 名 `jt20` |

口头约定「外四固定翼电机、中间倾转」与 SDF 一致：**内倾转、外固定**。

---

## 2. 工作思路（按时间线）

1. **坐标映射**：把 `jt_20.sdf` 的 Gazebo 轴（X前 Y左 Z上）换到 APM（X前 Y右 Z下），算出 8 个电机安装角与 CW/CCW。  
   → 文档：`docs/jt20_coordinate_mapping.md`
2. **SITL 机架**：在 `SIM_Frame.cpp` 增加 `jt20_motors[8]`，并在 `SIM_QuadPlane` / `vehicleinfo` 注册 `-f quadplane-jt20`。
3. **默认参数**：`quadplane-jt20.parm`（电机通道、倾转 mask、倾转类型）。
4. **自定义混控**：标准 OCTA-X 几何 ≠ JT-20 臂角 → 改用 `Q_FRAME_CLASS=15`（SCRIPTING_MATRIX）+ Lua `MotorsMatrix:add_motor_raw`，因子与 SITL 角严格同源。
5. **联调**：SITL + QGC 验证参数/脚本加载；用 `logs/00000008.BIN` 看垂起行为。

原则：**混控表 = SITL 电机表的逆映射**；Motor 序号、安装角、转向三者必须锁死。

---

## 3. 已改动的文件

### 3.1 SITL 动力学

**`libraries/SITL/SIM_Frame.cpp`**

- 新增 `jt20_motors[]`（8 电机）
- `supported_frames` 增加 `Frame("jt20", 8, jt20_motors)`

内四带 `pitch_servo`（相对 `motor_offset` 的下标 8/9 → SERVO13/14）；外四无倾转参数。

| Motor | 角 ° | yaw | 倾转舵机下标 | 物理 |
|-------|------|-----|--------------|------|
| M1 | +47.3 | CCW | 9（Right） | 前右内 |
| M2 | −145.3 | CCW | 8（Left） | 后左内 |
| M3 | −47.7 | CW | 8（Left） | 前左内 |
| M4 | +145.3 | CW | 9（Right） | 后右内 |
| M5 | +74.8 | CCW | — | 前右外 |
| M6 | −117.0 | CCW | — | 后左外 |
| M7 | −74.8 | CW | — | 前左外 |
| M8 | +117.0 | CW | — | 后右外 |

**`libraries/SITL/SIM_QuadPlane.cpp`**

```cpp
} else if (strstr(frame_str, "-jt20")) {
    frame_type = "jt20";
    thrust_scale = 0;
}
```

### 3.2 机架注册

**`Tools/autotest/pysim/vehicleinfo.py`**

```python
"quadplane-jt20": {
    "waf_target": "bin/arduplane",
    "default_params_filename": [
        "default_params/quadplane.parm",
        "default_params/quadplane-jt20.parm",
    ],
},
```

### 3.3 参数文件

**`Tools/autotest/default_params/quadplane-jt20.parm`**

| 参数 | 值 | 含义 |
|------|----|------|
| `Q_ENABLE` | 1 | 开 QuadPlane |
| `Q_FRAME_CLASS` | **15** | SCRIPTING_MATRIX（Lua 混控） |
| `Q_FRAME_TYPE` | 0 | 自定义矩阵时无标准 type |
| `Q_TILT_TYPE` | 2 | continuous / vectoring 类倾转 |
| `Q_TILT_MASK` | **15** | bit0–3 → Motor1–4 倾转 |
| `Q_TILT_YAW_ANGLE` | 10 | 倾转偏航矢量角 |
| `SCR_ENABLE` | 1 | 开 Lua |
| `SERVO5…12_FUNCTION` | 33…40 | Motor1…8（`motor_offset=4`） |
| `SERVO13_FUNCTION` | 75 | `k_tiltMotorLeft` |
| `SERVO14_FUNCTION` | 76 | `k_tiltMotorRight` |

`Q_TILT_MASK=15`（`0b00001111`）正确：`is_motor_tilting(motor)` 用 `1U << motor`（0-based）。

### 3.4 Lua 混控脚本

**`scripts/jt20_motor_matrix.lua`**

```lua
-- roll  = cos(angle_deg + 90)
-- pitch = cos(angle_deg)
-- yaw: CW=+1, CCW=-1
MotorsMatrix:add_motor_raw(0, -0.7349,  0.6782, -1, 1) -- M1  47.3 CCW
MotorsMatrix:add_motor_raw(1,  0.5693, -0.8221, -1, 2) -- M2 -145.3 CCW
MotorsMatrix:add_motor_raw(2,  0.7396,  0.6730,  1, 3) -- M3  -47.7 CW
MotorsMatrix:add_motor_raw(3, -0.5693, -0.8221,  1, 4) -- M4 145.3 CW
MotorsMatrix:add_motor_raw(4, -0.9650,  0.2622, -1, 5) -- M5  74.8 CCW
MotorsMatrix:add_motor_raw(5,  0.8910, -0.4540, -1, 6) -- M6 -117.0 CCW
MotorsMatrix:add_motor_raw(6,  0.9650,  0.2622,  1, 7) -- M7  -74.8 CW
MotorsMatrix:add_motor_raw(7, -0.8910, -0.4540,  1, 8) -- M8 117.0 CW
assert(MotorsMatrix:init(8), "Failed to init JT-20 MotorsMatrix")
motors:set_frame_string("JT-20 custom 8x tiltquad")
gcs:send_text(6, "JT-20 motor matrix loaded")
```

SITL 从启动目录读 `scripts/`，因此脚本路径为仓库根下：

```text
/home/codyqin/Desktop/git/ardupilot/scripts/jt20_motor_matrix.lua
```

### 3.5 其它同分支改动（非 JT-20 核心，但同在工作区）

| 文件 | 内容 | 与 JT-20 关系 |
|------|------|----------------|
| `ArduPlane/tiltrotor.cpp` | 注释掉的 debug printf | 无关，可忽略 |
| `libraries/AP_Motors/AP_MotorsMulticopter.cpp` | 空行 | 无关 |
| `Tools/autotest/default_params/quadplane-tilthvec.parm` | 倾转改 `k_motor_tilt=41` | **另一机架**，不是 jt20 |

---

## 4. 坐标系与编号约定（摘要）

换算（详见 `docs/jt20_coordinate_mapping.md`）：

```text
X_apm =  X_gz
Y_apm = -Y_gz
Z_apm = -Z_gz
angle_deg = atan2(Y_apm, X_apm)   # 机头起顺时针为正
```

编号约定：**APM Motor1…8 = Gazebo rotor_0…7 顺延**（为了和 SDF/Gazebo 通道对照），**不是**标准 OCTA-X 官方顺序。

PWM 映射（QuadPlane `motor_offset=4`）：

| 逻辑 | 舵机通道 | FUNCTION |
|------|----------|----------|
| Motor1…8 | SERVO5…12 | 33…40 |
| Left tilt | SERVO13 | 75 |
| Right tilt | SERVO14 | 76 |

---

## 5. 怎么跑

```bash
cd /home/codyqin/Desktop/git/ardupilot

# 首次清参数
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-jt20 -w

# 若本机无 mavproxy，可用 QGC：
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-jt20 -w --no-mavproxy
```

### 加载检查清单

| 检查项 | 期望 |
|--------|------|
| `Q_FRAME_CLASS` | 15 |
| `SCR_ENABLE` | 1 |
| `Q_TILT_MASK` | 15 |
| `Q_TILT_TYPE` | 2 |
| `SERVO5_FUNCTION` | 33 |
| `SERVO13/14_FUNCTION` | 75 / 76 |
| GCS 文本 | `JT-20 motor matrix loaded` |
| Frame 字符串 | `JT-20 custom 8x tiltquad`（日志里会截断） |

建议先 **QHOVER** 再 `arm`，不要一上来跑固定翼 AUTO 起飞任务。

---

## 6. 当前联调状态（`logs/00000008.BIN`）

### 6.1 已确认成功的部分

- 参数按 jt20 parm 生效（`Q_FRAME_CLASS=15`、`Q_TILT_MASK=15`、`SCR_ENABLE=1`、倾转 75/76）。
- Lua 混控已加载：日志有  
  `QuadPlane Frame: JT-20 custom 8x tiltqu`。
- 电机通道有输出；倾转 PWM 会从高位往低位走（有倾转动作）。

### 6.2 垂起「整体动作不对」——日志现象

| 现象 | 日志证据 |
|------|----------|
| 模式不是 Q 悬停 | `MODE`：`GUIDED(15)` → `AUTO(10)`，没有 `QHOVER(18)` |
| 初始倾转偏前飞 | 早期 `RCOU`：C5–C12≈1000，**C13/C14≈2000**；`TILT.Tilt≈89.9°`（接近前飞） |
| 姿态发散 | `ATT` 最大 \|Roll\|≈**117°**，\|Pitch\|≈38° |
| 撞地 | `SIM Hit ground at … m/s` |
| 高度曾起来 | `Alt` 约到 12 m，但控制不住 |

结论（当前判断）：

1. **脚本/参数链路是通的**，问题在「飞的方式 + 倾转/混控闭环是否与动力学一致」。
2. 这次 log 更像 **AUTO/GUIDED 任务起飞**，不是干净的 **QHOVER 悬停验证**；倾转初值偏前飞，垂起力矩分配会怪。
3. 即便电机对称抬升，大滚转说明 **力矩方向或倾转角符号仍可能与机体轴不一致**（左右 tilt 分配、`SERVOx_REVERSED`、SITL `pitch_min/max=−90…10` 与飞控 tilt 方向都要再对）。

### 6.3 建议的下一步验证顺序

1. `-w` 清参后只进 **QHOVER**，油门缓慢推，看：
   - SERVO13/14 是否停在**垂直附近**（约 1000–1100，而不是 2000）
   - 纯 roll/pitch 杆量时，哪几个电调增减是否符合混控表
2. 若悬停仍横滚：优先查 **Left/Right tilt 与 M1–M4 的对应**、以及 `SERVO13/14_REVERSED`。
3. 确认无误后再做 QLOITER / AUTO VTOL takeoff。

---

## 7. 与「标准 OCTA」的差别（为何必须 Lua）

| | 标准 `Q_FRAME_CLASS=3` OCTA | JT-20 |
|--|------------------------------|-------|
| 臂角 | 固定 OCTA-X 表 | SDF 换算出的 ±47°/±145° + 外圈 ±75°/±117° |
| 倾转 | 通常四倾转对称布局 | **仅内四**倾转，外四永远垂起 |
| 混控 | 固件内置 | **必须** `CLASS=15` + 与 `jt20_motors` 同源因子 |

若只用 OCTA 内置表，飞控分配的力矩臂 ≠ SITL 几何，闭环必然拧巴。

---

## 8. 文件索引

| 路径 | 作用 |
|------|------|
| `jt_20.sdf` | Gazebo 源模型 |
| `docs/jt20_coordinate_mapping.md` | 坐标/编号/角度换算 |
| `docs/tilt_servo_mapping_summary.md` | 倾转舵机左右映射约定 |
| `docs/vtol_jt20.md` | 本文：JT-20 工作总览 |
| `libraries/SITL/SIM_Frame.cpp` | `jt20_motors` + frame 注册 |
| `libraries/SITL/SIM_QuadPlane.cpp` | `-jt20` → `frame_type` |
| `Tools/autotest/pysim/vehicleinfo.py` | `quadplane-jt20` 入口 |
| `Tools/autotest/default_params/quadplane-jt20.parm` | 默认参数 |
| `scripts/jt20_motor_matrix.lua` | 自定义 8 电机混控 |
| `logs/00000008.BIN` | 首次联调日志（动作异常） |

---

## 9. 一句话状态

**JT-20 的 SITL 机架、参数、Lua 混控骨架已经搭好且能加载；垂起闭环尚未调通**——下一刀应固定在 QHOVER 下核对倾转垂直初值与滚转响应是否和 `jt20_motors` / Lua 表一致，再谈任务起飞。
