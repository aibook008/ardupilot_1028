# DF-200 SITL 电机推力参数推导过程

> 配套脚本：`docs/scripts/df200_motor_thrust_derive.py`
> 数据来源：`docs/电机电池.docx`、`docs/数据库_v2.xlsx`
> 代码：`libraries/SITL/SIM_Frame.cpp:531-536`、`libraries/SITL/SIM_Motor.cpp:321`

---

## 1. SITL 推力模型（代码还原）

### 1.1 推力公式（`SIM_Motor.cpp:321`）

```text
Vout = voltage_scale × velocity_max × sqrt((1-expo)·cmd + expo·cmd²)
T   = 0.5 × ρ × A_eff × (Vout² - Vin²)
```

这是**动量理论**简化模型：桨盘产生喷流 `Vout`，来流 `Vin`，推力 = 动量差。

### 1.2 各量的物理意义

| 量 | 物理意义 | 和转速有关？ |
|---|---------|:---:|
| `Vin` | 桨盘来流速度（外部风 + 机体旋转产生的桨尖线速度） | ❌ 不含转速 |
| `Vout` | 桨盘喷流速度（由油门 cmd + 电压推算） | ✅ 间接（cmd → Vout） |
| `A_eff` | 有效桨盘面积（从悬停标定反推） | ❌ |
| `ρ` | 空气密度 | ❌ |
| `expo` | 油门曲线非线性度（0=线性, 1=平方） | ❌ |

**关键**：SITL **不显式算转速**——用 `cmd`（油门指令 0-1）经 `sqrt` 算 `Vout`，跳过了真实电机的 `电压→电流→扭矩→转速（有惯量延迟）→桨推力` 链条。

### 1.3 参数推导链（`SIM_Frame.cpp:531-536`）

从 json 的 `refCurrent`、`refVoltage`、`hoverThrOut` 出发：

```
hover_thrust       = mass × g                              # 总悬停推力 (N)
hover_power        = refCurrent × refVoltage               # 总悬停功率 (W)
hover_velocity_out = 2 × hover_power / hover_thrust        # 悬停等效喷流速度 (m/s)
velocity_max       = hover_velocity_out / sqrt(hoverThrOut)  # 最大喷流速度 (m/s)
effective_disc_area = hover_thrust / (0.5 × ρ × hover_velocity_out²)  # 等效桨盘面积
effective_prop_area = effective_disc_area / num_motors     # 每电机面积
```

物理逻辑：`hover_power = 0.5 × hover_thrust × hover_velocity_out`（动量理论功率=½×推力×喷流速度），反推出 `hover_velocity_out`，再除以 `sqrt(hoverThrOut)` 得到 `velocity_max`。

---

## 2. 当前 json 参数代入计算

### 2.1 json 值

```json
"mass"        : 253.333,   // 飞行质量 = 253.333 × 1.5 = 380 kg (QuadPlane mass *= 1.5)
"refCurrent"  : 200.0,     // A
"refVoltage"  : 400.0,     // V
"hoverThrOut" : 0.30,      // 悬停油门
"propExpo"    : 0.65,      // 油门曲线
"disc_area"   : 10.54,     // m² (json 总桨盘面积)
"num_motors"  : 8
```

### 2.2 逐步推导

```
① hover_thrust = 380 × 9.81 = 3728 N

② hover_power = 200 × 400 = 80000 W

③ hover_velocity_out = 2 × 80000 / 3728 = 42.9 m/s

④ velocity_max = 42.9 / sqrt(0.30) = 42.9 / 0.5477 = 78.4 m/s

⑤ effective_disc_area = 3728 / (0.5 × 1.225 × 42.9²)
                       = 3728 / (0.5 × 1.225 × 1840)
                       = 3728 / 1126 = 3.304 m²

⑥ effective_prop_area = 3.304 / 8 = 0.413 m²/motor
```

### 2.3 悬停油门下的 Vout

```
cmd = hoverThrOut = 0.30

Vout(0.30) = 1.0 × 78.4 × sqrt((1-0.65)×0.30 + 0.65×0.30²)
           = 78.4 × sqrt(0.105 + 0.0585)
           = 78.4 × sqrt(0.1635)
           = 78.4 × 0.4044
           = 31.7 m/s    ← 悬停油门时喷流速度
```

### 2.4 巡航速度下的推力

```
AS = 50 m/s（巡航来流）
cmd = 0.30（悬停油门）

Vout = 31.7 m/s
Vin  = 50 m/s

T = 0.5 × 1.225 × 0.413 × (31.7² - 50²)
  = 0.5 × 1.225 × 0.413 × (1005 - 2500)
  = 0.5 × 1.225 × 0.413 × (-1495)
  = -378 N/motor    ← 负推力！

总推力 = -378 × 8 = -3027 N    ← 飞机被来流"压制"，产生阻力而非推力
```

### 2.5 问题：Vout < Vin

```
Vout(0.30) = 31.7 m/s
Vin(AS=50) = 50 m/s
Vout < Vin → Vout² - Vin² < 0 → 推力为负
```

**SITL 当前参数下，悬停油门在 AS=50 时推力为负**——这在真实 DF-200 上不会发生（真实桨尖速度 287 m/s >> 50）。

---

## 3. Vin 差异放大器

### 3.1 公式推导

左右电机相同 cmd、相同 Vout，但 Vin 不同：

```
T_left  = 0.5 × ρ × A × (Vout² - Vin_left²)
T_right = 0.5 × ρ × A × (Vout² - Vin_right²)

ΔT = T_left - T_right
   = 0.5 × ρ × A × (Vin_right² - Vin_left²)
   = 0.5 × ρ × A × (Vin_right - Vin_left) × (Vin_right + Vin_left)
   = 0.5 × ρ × A × ΔVin × (Vin_left + Vin_right)
```

**放大倍数 = 0.5 × ρ × A × (Vin_left + Vin_right)**

### 3.2 当前参数的放大倍数

```
A = 0.413 m², Vin_left + Vin_right ≈ 100 m/s (AS=50)

放大倍数 = 0.5 × 1.225 × 0.413 × 100 = 25.5

即：Vin 差 1 m/s → 推力差 25.5 N/motor
```

这就是 `position%gyro` 正反馈的根源——机体角速度经 `position%gyro` 产生左右 Vin 差异，被平方差放大 25.5 倍。

---

## 4. 真实电机数据对比

### 4.1 docx 数据

**60in 垂起桨（外4桨，表0）**：

| RPM | 推力(N) | 扭矩(Nm) | 轴功率(W) | 电流(A) |
|-----|---------|----------|----------|--------|
| 1551 | 278 | 18.77 | 3048 | 10.84 |
| 1895 | 444 | 29.07 | 5768 | 17.63 |
| 2294 | 645 | 43.08 | 10347 | 30.40 |
| 2759 | 970 | 65.99 | 19062 | 56.57 |
| 3181 | 1379 | 96.21 | 32043 | 99.46 |

**40in 巡航桨（内4桨，表1）V=50 m/s**：

| RPM | 推力(N) | 功率(kW) |
|-----|---------|---------|
| 2800 | 17.8 | 1.36 |
| 3000 | 55.7 | 3.38 |
| 3600 | 196.7 | 11.55 |
| 4000 | 319.2 | 19.11 |
| 4200 | 390.2 | 23.53 |

### 4.2 真实悬停功率

```
悬停推力 = 350 × 9.81 / 8 = 429 N/motor（docx 用 350kg）

外桨(60in): T=429N → RPM~1850 → 轴功率~5300 W/motor → ×4 = 21200 W
内桨(40in): T=429N → RPM~3200 → 轴功率~12480 W/motor → ×4 = 49920 W

真实总悬停功率 ≈ 71120 W = 71.1 kW
```

### 4.3 真实桨尖速度

```
60in桨: D=1.524m, RPM_max=3181
  桨尖速度 = π × 1.524 × 3181 / 60 = 253.8 m/s

40in桨: D=1.016m, RPM_max=4200
  桨尖速度 = π × 1.016 × 4200 / 60 = 223.4 m/s
```

真实桨尖速度 220-254 m/s >> 来流 50 m/s，所以真实桨在巡航时推力始终为正。

### 4.4 对比

| 项 | 真实 | SITL | 差异 |
|---|------|------|------|
| 悬停功率 | 71 kW | 80 kW (200A×400V) | 接近 |
| 最大桨尖/喷流速度 | 224-254 m/s | velocity_max=78.4 m/s | **SITL 远小于真实** |
| Vout(hover) | ~120 m/s（真实等效） | 31.7 m/s | **SITL 远小于来流 50** |
| AS=50 推力 | 正（RPM>3000 时） | 负 | **SITL 错误** |

---

## 5. 修正方案

### 5.1 目标

让 `Vout(hover cmd) > AS`，即 `Vout(0.30) > 50`，留余量取 60。

### 5.2 反推 refCurrent

```
目标 Vout(0.30) = 60 m/s

Vout(cmd) = velocity_max × sqrt((1-expo)×cmd + expo×cmd²)
60 = velocity_max × sqrt(0.105 + 0.0585)
60 = velocity_max × 0.4044
velocity_max = 60 / 0.4044 = 148.3 m/s

velocity_max = hover_velocity_out / sqrt(hoverThrOut)
148.3 = hover_velocity_out / sqrt(0.30)
hover_velocity_out = 148.3 × 0.5477 = 81.2 m/s

hover_velocity_out = 2 × hover_power / hover_thrust
81.2 = 2 × hover_power / 3728
hover_power = 81.2 × 3728 / 2 = 151437 W

refCurrent = hover_power / refVoltage = 151437 / 400 = 379 A
```

### 5.3 验证（refCurrent=510，更保守）

```
hover_power = 510 × 400 = 204000 W
hover_velocity_out = 2 × 204000 / 3728 = 109.4 m/s
velocity_max = 109.4 / 0.5477 = 199.8 m/s

Vout(0.30) = 199.8 × 0.4044 = 80.8 m/s >> 50 ✓

T(0.30, AS=50) = 0.5 × 1.225 × A_eff × (80.8² - 50²)
               = 0.5 × 1.225 × A_eff × (6529 - 2500)
               = 0.5 × 1.225 × A_eff × 4029

A_eff 从重新推导：
  effective_disc_area = 3728 / (0.5 × 1.225 × 109.4²) = 3728 / 7334 = 0.508 m²
  effective_prop_area = 0.508 / 8 = 0.0635 m²

T = 0.5 × 1.225 × 0.0635 × 4029 = 156.7 N/motor → 正推力 ✓
```

### 5.4 放大倍数对比

```
refCurrent=200: A_eff=0.413, 放大=0.5×1.225×0.413×100 = 25.5 N per 1 m/s Vin差
refCurrent=510: A_eff=0.0635, 放大=0.5×1.225×0.0635×100 = 3.9 N per 1 m/s Vin差
```

放大倍数从 25.5 降到 3.9（降 6.5 倍）——`position%gyro` 正反馈增益大幅降低。

### 5.5 refCurrent 推荐值

| refCurrent | Vout(0.3) | T@AS=50 | Vin放大 | 评价 |
|-----------|-----------|---------|---------|------|
| 200（当前） | 31.7 | -378 N ✗ | 25.5 | 高速推力负 |
| 258 | 40.9 | -126 N ✗ | 15.4 | 仍负 |
| 379 | 60.0 | ~0 N | ~6 | 临界 |
| **510** | **80.8** | **+157 N ✓** | **3.9** | **推荐** |

---

## 6. 使用脚本

```bash
# 查当前配置
python3 docs/scripts/df200_motor_thrust_derive.py

# 试不同 refCurrent
python3 docs/scripts/df200_motor_thrust_derive.py --refcurrent 510

# 指定目标 Vout
python3 docs/scripts/df200_motor_thrust_derive.py --target-vout 80
```

脚本输出：json 参数 → SITL 推导 → 各工况推力 → Vin 放大倍数 → 修正建议。
