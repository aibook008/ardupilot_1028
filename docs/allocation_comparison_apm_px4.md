# 分配矩阵对比：ArduPilot vs PX4

> 对照文档：`/home/codyqin/Desktop/git/PX4-Autopilot/docs/allocation_matrix_px4.md`（PX4 v1.15.3 fork）  
> APM 侧依据：`AP_MotorsMatrix::setup_quad_matrix` / `add_motor`、`docs/scripts/gen_motor_matrix.py`、`docs/scripts/verify_motor_matrix.py`  
> 用途：明确两边「分配矩阵」是否同一概念，以及 DF200/JT20 混控该按哪套理解。

---

## 1. 一句话结论

| | PX4 | ArduPilot（`setup_quad_matrix` / Lua Motors_dynamic） |
|--|--|--|
| 本质 | 先建物理有效性矩阵 \(B\)，再伪逆得混控 \(B^+\) | **直接写**归一化混控系数表 |
| yaw | 进 \(B\) 的物理反扭项 \(k_m\) | 单独的旋向符号 ±1，与力臂无关 |
| 位置用法 | \(\mathbf{r}\times\mathbf{a}\) 出力矩 | 只用方位角 \(\mathrm{atan2}(Y,X)\) 出 roll/pitch 比例 |

**两边说的「分配矩阵」不是同一层对象。**  
PX4 的 mix 是 \(B^+\)；APM 的 `_roll/_pitch/_yaw_factor` 是直接给定的混控增益，中间没有伪逆。

---

## 2. 控制链路对照

### PX4

```
力矩/推力期望 τ
    → ActuatorEffectiveness*  计算 B (6×N)
    → ControlAllocation*      B⁺ = pinv(B)，再归一化
    → u = trim + B⁺ · (τ − τ_trim)
```

见 PX4 文档 §1–§3。

### ArduPilot

```
RAT_* PID → roll/pitch/yaw/throttle 无量纲指令
    → AP_MotorsMatrix 查表：
         thrust_i = thr
                  + roll_cmd  * _roll_factor[i]
                  + pitch_cmd * _pitch_factor[i]
                  + yaw_cmd   * _yaw_factor[i]
    → （静态机架）normalise_rpy_factors：各轴 max|factor| → 0.5
    → （Lua Dynamic）可跳过归一化，运行时 load_factors 覆盖
```

官方四旋翼表：`libraries/AP_Motors/AP_MotorsMatrix.cpp::setup_quad_matrix`  
DF200：`scripts/df200_motor_matrix.lua`（`Q_FRAME_CLASS=17`）

---

## 3. 计算公式对照

### 3.1 PX4：物理 \(B\) 再伪逆

对第 \(i\) 个电机（`ActuatorEffectivenessRotors::computeEffectivenessMatrix`）：

\[
\begin{aligned}
\mathbf{a}_i &\leftarrow \mathbf{a}_i / \|\mathbf{a}_i\| \\
\mathbf{F}_i &= C_{T,i}\,\mathbf{a}_i \\
\mathbf{M}_i &= C_{T,i}\,(\mathbf{r}_i\times\mathbf{a}_i) - C_{T,i}\,k_{m,i}\,\mathbf{a}_i
\end{aligned}
\]

\(B(:,i)=[M_x,M_y,M_z,F_x,F_y,F_z]^T\)，再：

\[
B^+=\mathrm{pinv}(B)\quad\text{（Moore-Penrose + RPY/推力归一化）}
\]

向上桨 \(\mathbf{a}=(0,0,-1)\) 时简化为：

| 量 | 公式 |
|----|------|
| \(M_x\) | \(C_T\cdot r_y\) |
| \(M_y\) | \(-C_T\cdot r_x\) |
| \(M_z\) | \(C_T\cdot k_m\)（反扭） |
| \(F_z\) | \(-C_T\) |

**力臂长度进入 \(B\)；yaw 与 roll/pitch 同一物理模型。**

### 3.2 ArduPilot：角度 → 直接混控系数

`AP_MotorsMatrix::add_motor` / `docs/scripts/gen_motor_matrix.py`：

```
angle = atan2(Y, X)                 # APM body: X fwd, Y right
roll  = cos(radians(angle + 90))  = -sin(radians(angle))
pitch = cos(radians(angle))
yaw   = ±1                          # 旋向符号，不是 r×a，也不是 km
```

`setup_quad_matrix` 里 `MotorDef = { angle_degrees, yaw_factor, testing_order }`，  
`add_motors` → `add_motor` → 上式写入 `_roll/_pitch/_yaw_factor`。

**没有伪逆，没有 \(C_T/k_m\)，没有 6×N 的 \(B\)。**  
位置只决定方位角；**对称机上力臂长度不进入混控系数。**

---

## 4. 逐项差异表

| 项目 | PX4 | APM / `gen_motor_matrix.py` |
|------|-----|-----------------------------|
| 输入 | \(\mathbf{r},\mathbf{a},C_T,k_m\) | \(\mathbf{r}\)（或角度）+ yaw 符号 |
| roll 来源 | \(C_T(\mathbf{r}\times\mathbf{a})_x \propto r_y\) | \(-\sin(\mathrm{atan2}(y,x))\) |
| pitch 来源 | \(\propto -r_x\) | \(\cos(\mathrm{atan2}(y,x))\) |
| yaw 来源 | \(-C_T k_m\mathbf{a}\)（反扭） | 人为 ±1（差速反扭） |
| 力臂长度 | **进入** \(B\) | **不进入**（只影响角） |
| 推力轴倾斜 | \(\mathbf{a}\) 改了整列 \(B\) | 普通 Matrix 不管；倾转另走 tiltrotor |
| 求解 | 伪逆 \(B^+\) | 查表加权求和 |
| 归一化 | 对 \(B^+\) 的 RPY/推力列 | 对系数表 max→0.5（或 Lua raw） |
| 饱和 | Sequential Desaturation 等 | `MOT_YAW_HEADROOM` + 油门平移 |
| 故障 | 列置零后重算伪逆，冻结归一化 | 有限的 thrust-boost / 无完整重分配 |
| 参数入口 | `CA_AIRFRAME` / `CA_ROTOR*` | `FRAME_CLASS`+`FRAME_TYPE` 或 Lua |

---

## 5. 对称四旋翼上为何「看起来像」

对称 Quad-X、向上桨时：

- PX4：\(M_x\propto r_y\)，\(M_y\propto -r_x\)，归一化后各电机 roll/pitch 增益同幅异号  
- APM：\(\theta=\pm45°/\pm135°\) → \(\mathrm{roll/pitch}=\pm\sqrt{2}/2\)，再归一化到 ±0.5  

形态接近，所以官方 `+` / `x` / `bfx` / `djix` / `cwx` 用 APM 表即可飞，不必建 \(B\)。

**真正分叉的场景：**

1. **非对称布局**（JT20 / DF200）  
   - PX4：前后/左右力臂长度进 \(B\)，伪逆自动加权  
   - APM：只认角度 → 前后臂长不等时，系数幅值仍只由角决定；JT20 Lua 出现 \(\Sigma\) pitch ≠ 0 即此类几何不对称的表现  
2. **yaw**  
   - PX4：物理 \(k_m\)  
   - APM：混控 ±1；SITL 植物另用经验式 `0.05 * diagonal_size * thrust * command * yaw_factor`（`SIM_Motor.cpp`），与混控系数解耦  
3. **倾转 / 故障**  
   - PX4：改 \(B\) 再重算伪逆  
   - APM：Lua `load_factors` 衰减，或固定表 + tiltrotor 矢量 yaw 旁路  

---

## 6. 与本仓库脚本的对应关系

| 脚本 | 路线 | 产出 |
|------|------|------|
| `docs/scripts/gen_motor_matrix.py` | **纯 APM** | 从位置/预设生成 roll/pitch/yaw 系数；可出 Lua / MotorDef |
| `docs/scripts/verify_motor_matrix.py` | **纯 APM** | 核对 Lua↔JSON↔`SIM_Frame`，以及官方 SITL↔`setup_quad_matrix` |
| PX4 `docs/allocation_matrix_px4.md` | **PX4 \(B\rightarrow B^+\)** | 有效性矩阵与伪逆说明 |

若要对齐 PX4 文档做数值对照，应另算 \(B\)（及可选 `pinv`），**不要**把 PX4 的 \(B^+\) 元素直接当成 APM 的 `_yaw_factor`。

建议核对顺序（与 PX4 文档 §7 一致，并加上 APM 侧）：

1. 坐标系：两边都是机体 FRD，向上 \(-Z\)  
2. 行/列含义：PX4 是 \(6\times N\) 物理 \(B\)；APM 是 \(N\times\{R,P,Y,T\}\) 混控增益  
3. yaw：PX4 \(k_m\) vs APM ±1 + SITL 经验反扭  
4. 是否归一化、归一化定义是否相同  
5. 非对称机：力臂长度有没有进混控  

---

## 7. DF200 / JT20 实践含义

- 混控（飞控侧）：走 APM 路线 —— Lua BASE 用 `cos(ang±)` + yaw 符号；高速用 floor 衰减。  
- 植物（SITL）：JSON `motorN_position` 出力臂几何力矩 \(\mathbf{r}\times\mathbf{F}\)；`motorN_yaw` **只缩放**反扭大小（DF200 曾用 `|yaw|=9`），符号须与 Lua 一致。  
- 因此：**验证混控用 `verify_motor_matrix.py` / `gen_motor_matrix.py`；验证植物力矩用转子力臂 + `SIM_Motor` 反扭公式，不要用 PX4 的 \(B^+\) 去改 APM Lua。**

---

## 8. 相关文件

| 文件 | 内容 |
|------|------|
| `libraries/AP_Motors/AP_MotorsMatrix.cpp` | `setup_quad_matrix`、`add_motor`、`normalise_rpy_factors` |
| `libraries/SITL/SIM_Frame.cpp` | SITL 机架角 / yaw 宏；`df200_motors` / `jt20_motors` |
| `libraries/SITL/SIM_Motor.cpp` | 经验反扭 `yaw_scale = 0.05 * diag * thrust` |
| `scripts/df200_motor_matrix.lua` | DF200 动态混控 |
| `scripts/jt20_motor_matrix.lua.off` | JT20 静态混控（参考） |
| `docs/scripts/gen_motor_matrix.py` | 由位置生成 APM 分配矩阵 |
| `docs/scripts/verify_motor_matrix.py` | APM 侧对应关系校验 |
| `PX4-Autopilot/docs/allocation_matrix_px4.md` | PX4 \(B\) / \(B^+\) 基线说明 |
