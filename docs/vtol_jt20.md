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
| `Tools/autotest/models/jt20.json` | SITL 物理模型（质量/力臂/推力/电机位置） |
| `scripts/jt20_motor_matrix.lua` | 自定义 8 电机混控（yaw 取反，与 json 对齐） |
| `docs/jt20_sitl_model_plan.md` | json 模型设计与调参说明 |

---

## 9. SITL 物理模型（jt20.json）调参经验

骨架能加载后，真正让 JT-20「能飞且稳」的是 `Tools/autotest/models/jt20.json`。这一节记录踩过的坑和结论，后续换真机参数时直接对照。

### 9.1 为什么必须有 json

无 json 时，`SIM_Frame` 用 `default_model`（`SIM_Frame.h`）：mass=3、diagonal_size=0.35、num_motors=4。QuadPlane 构造还会 `mass *= 1.5`，即默认只有 **4.5kg、0.35m 力臂**。这与 JT-20（SDF 约 10kg、臂长 0.4~1m）差很多：
- 00000008~00000013 都是这套默认模型，表现为低头/偏航瞬态被放大、电机差动异常。

### 9.2 json 字段含义（推力相关）

SITL **没有直接的「升力系数」字段**，推力由动量盘模型从这几个量推出：

| 字段 | 作用 | 想让推力变大 |
|------|------|--------------|
| `mass` | 悬停需推力 = mass·g | 按真机（注意 QuadPlane 会再 ×1.5） |
| `refCurrent`×`refVoltage` | 参考功率 → 喷流速度上限 | ↑ |
| `hoverThrOut` | 悬停油门；满油门推力 ≈ mass·g/hoverThrOut | ↓ |
| `disc_area` / `num_motors` | 总桨盘面积，均分到每桨 | ↑ |
| `propExpo` | 油门→推力曲线形状 | 按电机 |

#### 推力公式拆解（符号与来源）

链路：`PWM → command(0~1) → v_out → thrust(N)`。代码：`SIM_Motor::pwm_to_command` / `SIM_Motor::calc_thrust`，标定在 `SIM_Frame::init`。

**① PWM → command**

```
pwm_thrust_min = PWM_MIN + spin_min × (PWM_MAX − PWM_MIN)
pwm_thrust_max = PWM_MIN + spin_max × (PWM_MAX − PWM_MIN)
command = saturate( (pwm − pwm_thrust_min) / (pwm_thrust_max − pwm_thrust_min), 0, 1 )
```

| 符号 | 含义 | 当前值 |
|------|------|--------|
| `pwm` | 电机 PWM | 1000~2000 |
| `PWM_MIN/MAX` | json `pwmMin/pwmMax` | 1000 / 2000 |
| `spin_min/max` | 有效推力起止比例（同 MOT_SPIN_*） | 0.15 / 0.95 |
| `command` | 归一化油门 | 0=无推力，1=满推力 |

当前：`pwm_thrust_min=1150`，`pwm_thrust_max=1950`。PWM≤1150 → command=0；PWM≥1950 → command=1。

**② 标定 `velocity_max` 与 `A_eff`（只在 init 做一次）**

```
hover_thrust   = mass × g
hover_power    = refCurrent × refVoltage
hover_velocity = 2 × hover_power / hover_thrust     // 动量盘：P ≈ T·(v_out/2)
velocity_max   = hover_velocity / √hoverThrOut      // 满油门喷流上限
effective_disc = hover_thrust / (0.5 × ρ × hover_velocity²)
A_eff          = effective_disc / num_motors        // 每桨有效面积
```

`hover_velocity = 2P/T` 来自理想动量盘：诱导功率 `P ≈ T·v_i`，悬停下游喷流约 `2 v_i`，故 `v_out ≈ 2P/T`。再除以 `√hoverThrOut`：悬停只用部分油门，满油门喷流应更快（近似 `v ∝ √command`）。

**③ command → v_out（喷流速度）**

```
Vscale = V / V_max
v_out  = Vscale × velocity_max × √[ (1−expo)×command + expo×command² ]
```

| 符号 | 含义 | 当前值 |
|------|------|--------|
| `V` / `V_max` | 当前/满电电压 | / `maxVoltage=25.2` |
| `velocity_max` | 满油门喷流上限 | 见下表算例 |
| `expo` | json `propExpo`（MOT_THST_EXPO） | 0.65 |
| `v_out` | 桨盘下游气流速度 | m/s |

`√[(1−expo)c + expo c²]` 是 ArduPilot 工程曲线：`expo=0` 偏 `√c`，`expo=1` 偏 `c`，中间折中低油门手感。

**④ v_out → thrust（动量盘）**

```
thrust = 0.5 × ρ × A_eff × (v_out² − v_in²)
```

| 符号 | 含义 |
|------|------|
| `ρ` | 空气密度（海平面取 1.225 kg/m³） |
| `A_eff` | 该电机有效桨盘面积 |
| `v_in` | 来流在推力方向的分量；悬停 ≈ 0 |
| `thrust` | 单桨推力 (N) |

来自致动盘理论：气流被加速，`T = ṁ Δv`，在常用简化下写成 `½ρA(v_out²−v_in²)`。悬停 `v_in=0` 时 `T ≈ ½ρ A_eff v_out²`。

粗估满油门总推力：`T_max ≈ mass·g / hoverThrOut`。

日志 ESC RPM = `command × SIM_VIB_MOT × 60`，**仅显示用，不参与推力**。

#### 当前 jt20.json 数值算例（00000019 可飞版本）

输入：`mass=10.255`，`refCurrent=45`，`refVoltage=25.2`，`hoverThrOut=0.35`，`propExpo=0.65`，`num_motors=8`，`ρ=1.225`。注意 QuadPlane 还会 `mass×1.5` → 飞行质量 **15.383 kg**；标定仍用 json 的 10.255。

| 标定量 | 数值 |
|--------|------|
| `hover_thrust = mass·g` | **100.57 N**（10.26 kgf） |
| `hover_power = I·V` | **1134 W** |
| `hover_velocity = 2P/T` | **22.55 m/s** |
| `velocity_max` | **38.12 m/s** |
| `A_eff`（每桨） | **0.0404 m²** |
| `T_max`（8 桨合计，cmd=1） | **287.3 N**（29.3 kgf） |
| 相对标定质量 T/W | **2.86** |
| 相对飞行质量 15.38 kg T/W | **1.90** |

PWM → command → 总推力（满电、`v_in=0`）：

| PWM | command | v_out (m/s) | 单桨 T (N) | 总推力 (N) | 总 kgf | T/W（相对 15.38 kg） |
|-----|---------|-------------|------------|------------|--------|----------------------|
| 1150 | 0.00 | 0.0 | 0.0 | 0.0 | 0.0 | 0.00 |
| 1300 | 0.19 | — | — | 25.4 | 2.6 | 0.17 |
| 1500 | 0.44 | — | — | 79.7 | 8.1 | 0.53 |
| 1700 | 0.69 | — | — | 157.4 | 16.0 | 1.04 |
| 1850 | 0.88 | — | — | 231.0 | 23.6 | 1.53 |
| 1950 | 1.00 | 38.1 | 35.9 | **287.3** | 29.3 | **1.90** |

command 扫表（同上条件）：

| command | v_out | 单桨 T | 总推力 | T/W（15.38 kg） |
|---------|-------|--------|--------|-----------------|
| 0.35 | 17.1 | 7.3 | 58.1 | 0.39 |
| 0.50 | 22.2 | 12.1 | 97.0 | 0.64 |
| 0.65 | 27.0 | 18.0 | 144.3 | 0.96 |
| 0.80 | 31.8 | 25.0 | 200.0 | 1.33 |
| 1.00 | 38.1 | 35.9 | 287.3 | 1.90 |

飞行质量 15.383 kg 悬停需 **150.9 N** → 反算 `command ≈ 0.67`，`v_out ≈ 27.6 m/s`，单桨约 18.9 N。  
00000019 实测悬停 `ThO≈0.50`（略低于 0.67）：高度环/油门映射与 command 不是严格同一刻度，且标定用的是 ×1.5 前的 mass，属正常偏差；关键是 **T/W≈1.9、悬停不满油门**，与「能飞」一致。

### 9.3 两个真实建模问题与修法

#### （A）起飞猛低头 → 推力中心在 SITL CG 之后

现象：00000016/17 推满油门时 Pitch 冲到 **−70°**，飞机往前窜、抬不起来。

根因：等推力时净俯仰力矩 = `T_total × x̄`，其中 `x̄ = mean(motor_i.x)`。
SDF 原始电机位置（APM 坐标）x 均值：
```
x̄ = (0.365−0.570+0.360−0.570+0.240−0.450+0.240−0.450)/8 ≈ −0.104 m
```
即推力中心在 SITL 原点（默认 CG）**之后 10cm**，等推力就有低头力矩。真机 `base_link` 惯性中心在 x=−0.105（与推力中心几乎重合），但 **SITL 不读 SDF 的 CG，固定在原点**，所以 SITL 才低头。

修法：json 里把所有 `motorN_position` 的 x 统一 **+0.104**，让推力中心回到 x≈0（等价于把 SITL 有效 CG 对齐真机）。改后 00000018/19 的 max|Pitch| 降到 **8°**。

计算方法：
```
x̄ = mean(所有 motorN_position.x)
shift = −x̄
motorN_position.x += shift
```

#### （B）偏航失控 → Lua yaw 与 json yaw 必须对齐

现象：00000010 默认模型下偏航狂转 ~−440°/s，曾把 **Lua 的 yaw 全部取反**临时压住；但 00000018 切到 json 后又转 ~−180°/s。

根因：SITL 反扭矩权威 `∝ diagonal_size`（`rotor_torque ∝ 0.05*diagonal*thrust`）。json 的 diagonal=1.99 比默认 0.35 大约 5.7 倍，一旦飞控混控 yaw 符号与仿真 `motorN_yaw` 反了，就越纠越转。

原则：**Lua 的 yaw 因子 与 json 的 `motorN_yaw` 必须同号同源**。最终两边都用「取反后」的符号（M1/2/5/6=+1，M3/4/7/8=−1），00000019 偏航降到平均 −4°/s（慢漂，非失控）。

### 9.4 启动方式（关键）

`-f` 的 frame 串若带 `:path.json`，`options_for_frame()` 会按前缀 `quadplane` 回退到通用条目，**丢掉 `quadplane-jt20.parm`**。因此正确做法是在 `vehicleinfo.py` 里给 `quadplane-jt20` 显式写：
```python
"quadplane-jt20": {
    "model": "quadplane-jt20:Tools/autotest/models/jt20.json",
    ...
}
```
然后用不带 `:json` 的命令：
```bash
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-jt20 -w --no-mavproxy
```
验证三行日志：
```
Loaded defaults from .../quadplane-jt20.parm
QuadPlane Frame: JT-20 custom 8x tiltqu
Loaded model params from jt20.json
```

### 9.5 当前 json 关键值（00000019 可飞版本）

| 字段 | 值 | 说明 |
|------|----|------|
| mass | 10.255 | SDF 总质量；×1.5 后约 15.4kg（偏重，可后续调到 6.7→约10kg） |
| diagonal_size | 1.99 | 真实机臂直径 |
| num_motors | 8 | |
| hoverThrOut | 0.35 | 00000019 悬停 ThO≈0.50，有余量 |
| refCurrent / refVoltage | 45 / 25.2 | 6S，功率参考 |
| disc_area | 2.26 | 8×0.6m 桨 |
| motorN_position.x | 已 +0.104 | 推力中心对齐 CG |
| motorN_yaw | M1/2/5/6=+1, M3/4/7/8=−1 | 与 Lua 对齐 |

### 9.6 固定翼气动：SDF vs SITL 默认（方案 A 已落地）

悬停只依赖多旋翼物理（json）。但**前飞/倾转过渡**用的是 `SIM_Plane`（机身+机翼+舵面气动），其系数目前是**内置默认的 Skywalker 航模**那套，不是 JT-20。对照 SDF `AdvancedLiftDragPlugin`：

| 量 | SDF（jt_20.sdf） | 当前 SITL 默认 | 差异 |
|----|------------------|----------------|------|
| 机翼面积 S | **0.4761 m²** | 0.45 m² | +6% |
| 展长 b | AR=10.64 → ≈2.25 m | 1.88 m | +20% |
| 平均弦 mac | **0.2115 m** | 0.24 m | −12% |
| CL0 | **0.403** | 0.56 | |
| CLa | **7.39** | 6.9 | |
| CD0（≈c_drag_p） | **0.01887** | 0.1 | 默认阻力偏大 |
| Oswald e | **0.97** | 0.9 | |
| α_stall | **0.3391 rad ≈ 19.4°** | 0.4712 rad ≈ 27° | SDF 更早失速 |
| Cma | **−1.06** | −0.7 | SDF 静稳定更强 |
| Cmq | **−22.89** | −20 | 俯仰阻尼更大 |
| 气动参考点 | ref_pt **(−0.126, 0, −0.08)** | CGOffset (−0.15,0,−0.05) | |

**结论**：JT-20 真机更细长（AR 高）、失速更早、静稳定更强、零升阻力更小，与默认 Skywalker 明显不同。之前这些系数**没有被任何地方引用**（json 只管多旋翼）。

#### 方案 A：在 `SIM_Plane::Plane()` 加 `-jt20` 分支（已实现并编译通过）

`libraries/SITL/SIM_Plane.cpp` 构造尾部新增：

```cpp
if (strstr(frame_str, "-jt20")) {
    // JT-20 fixed-wing aero from jt_20.sdf AdvancedLiftDragPlugin
    coefficient.s = 0.4761f;
    coefficient.b = sqrtf(coefficient.s * 10.6383f); // AR*S, ~2.25 m
    coefficient.c = 0.2115f;
    coefficient.c_lift_0 = 0.403f;
    coefficient.c_lift_a = 7.39f;
    coefficient.c_drag_p = 0.01887f;  // approx CD0
    coefficient.oswald = 0.97f;
    coefficient.alpha_stall = 0.3391f; // 19.4 deg
    coefficient.c_m_a = -1.06f;        // static stability
    coefficient.c_m_q = -22.89f;       // pitch damping
    coefficient.CGOffset = Vector3f(-0.126f, 0.0f, -0.08f);
}
```

说明：
- `-jt20` 分支在 `SIM_QuadPlane::QuadPlane()` 已有（决定 frame_type=jt20、thrust_scale=0）；这里补的是 **`SIM_Plane` 的固定翼气动**，两者独立，共同由 frame 串 `quadplane-jt20:...json` 触发。
- `b` 由 `b = √(AR·S)` 推算（SDF 未直接给展长）。
- 舵面（elevon×2 + tail×2）的 `CL_ctrl/Cem_ctrl` 暂未映射到 ArduPilot 的 `deltaa/deltar` 参数；若后续做精细过渡，再把 4 个舵面控制导数补上。

验证：`./waf --targets bin/arduplane` 编译通过。前飞特性需在 SITL 里用 FBWA / 倾转过渡跑一局确认（后续）。

### 9.7 VTOL takeoff → 前飞过渡：必须先对齐航向（已实现）

#### 问题（00000020）

AUTO VTOL takeoff 到高度后立刻 `transition->restart()`。非 vectored（`Q_TILT_TYPE=0`）在过渡里只 `reset_yaw_target_and_rate()`——**锁当前航向，不对齐下一航点**。过渡开始时航向误差约 **−75°**，边倾转边飞出。

`jt20_motor_matrix.lua` **只管 8 电机混控**（`Q_FRAME_CLASS=15`），不读航点、不挡过渡；纯 Lua 也无法否决 `verify_vtol_takeoff`。

#### 固件门控（任务层目标）

| 项 | 内容 |
|----|------|
| 开关 | `Q_OPTIONS` bit **23** `ALIGN_HDG_BEFORE_TRANS` |
| 门限 | `Q_TF_YAW_ERR`（deg，默认 15） |
| 告警间隔 | `Q_TF_YAW_TMO`（s；0=每 5s 告警一次；**永不强制过渡/QLAND**） |
| 目标方位 | 当前位 → **下一 nav 航点** `get_bearing_to`；无下一 WP 则不门控（避免 takeoff-only 死锁） |

流程：

1. 爬升到 takeoff 高度（现有逻辑不变）  
2. 若 bit23 开启：`verify_vtol_takeoff` **先不** `restart()`，返回 false 留在 VTOL takeoff  
3. `takeoff_controller` 用绝对航向 `input_euler_angle_roll_pitch_yaw(..., target_bearing_cd, true)` 转向，同时高度保持  
4. `|yaw−target| ≤ Q_TF_YAW_ERR` 后才 `transition->restart()`，GCS 提示 `yaw aligned, transitioning`  
5. 之后非 vectored 过渡仍冻结航向——此时已对准航线，行为正确  

jt20 默认：`Q_OPTIONS=10485760`（bit21+bit23），`Q_TF_YAW_ERR=15`。

代码：`ArduPlane/quadplane.h` / `quadplane.cpp`（`verify_vtol_takeoff`、`takeoff_controller`、`get_takeoff_align_yaw_cd`、`takeoff_yaw_aligned`）。

### 9.8 后续调参方向

- mass 偏大（×1.5 后 15.4kg），若想更贴真机可把 json mass 设 ~6.7（×1.5≈10kg），再按 `QTUN.ThO` 回调 `hoverThrOut`。
- 00000019 稳态低头 ~−5°：检查是否还有残余前后配平（可微调 x-shift），或属正常气动配平。
- 偏航慢漂 ~−4°/s：先确认 RC4 居中（日志 RC4 min=1425），再看是否需要 trim。
- 真机参数（桨径、KV、电压、电池）确定后，重标 `refCurrent`、`disc_area`、`hoverThrOut`。
- 固定翼：方案 A 已接上主要升力/阻力/力矩系数；下一步在 SITL 里验证前飞，再把 4 舵面控制导数按 SDF 补全（见 9.6）。
- 航向门控：SITL AUTO Takeoff→故意偏航的 WP，确认先出现 `aligning yaw` / `waiting yaw align`，误差进门后再 `Transition started`。

---

## 10. 一句话状态

**JT-20 在 SITL 已能稳定 QLOITER 飞行**（00000019），并已增加 **VTOL takeoff 后必须对齐下一航点再倾转**（`Q_OPTIONS` bit23 + `Q_TF_YAW_ERR`）。骨架 + 多旋翼物理 + 固定翼气动系数 + 过渡前航向门控已打通；剩余是质量精标、舵面导数、以及门控的实飞/SITL 回归验证。
