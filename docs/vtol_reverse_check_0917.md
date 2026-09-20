# 反向链解读补充：油门信号对照 / vel_forward / 抬头限制方案

> 日期：2026-09-17。对应代码：Plane 4.6.x（工作区含走廊门控改动），行号均经 grep+Read 验证。
> 主文档：`docs/vtol_reverse_transition.md`（代码链路与 209 日志主分析）。本文是其 Q&A 深挖补充，针对 4 个信号级疑问 + 抬头 30° 限制需求。

---

## 1. 油门四级信号对照（回答"看哪个信号"）

油门从"控制器需求"到"电调输出"有四级，日志各有一个信号，**不能混用**：

| 级别 | 日志信号 | 物理含义 | 代码源 |
|------|---------|---------|--------|
| ① TECS 需求 | **TECS.th**（0~1） | 速度-高度控制器原始输出 | `_throttle_dem`（`libraries/AP_TECS/AP_TECS.cpp:1327,1347`） |
| ② k_throttle 通道 | **CTUN.ThO**（经 slew）/ **CTUN.ThD**（=①原样） | `SRV_Channel::k_throttle` 的 scaled 值 | `ArduPlane/Log.cpp:112`（ThO）、`:114`（ThD） |
| ③ 旋翼油门 | **QTUN.ThO** | `motors->get_throttle()`，多旋翼混控实际用的油门 | `ArduPlane/quadplane.cpp:3803` |
| ④ 电调输出 | **MOTB.ThrOut** | 混控后电调平均输出 | AP_Motors 内部 |

**tilt 的油门映射（`tiltrotor.cpp:521`）读的是 ②（k_throttle）**。所以：

- "k_throttle 目标归 0" 说的是 ② 的目标值；实际 ② 经 THR_SLEWRATE=100%/s 限速在 1.0s 内从 100 放坡到 0（209 实测斜率 −99.9%/s）。
- state 3 期间 ③（QTUN.ThO）和 ④（MOTB.ThrOut 0.35~0.48）**不是 0**——旋翼出力由 Z 环另管，扛重量。tilt_raw→0 只表示桨回到垂直，不代表桨推力归零。
- CTUN.ThD vs CTUN.ThO 的差 = THR_SLEWRATE 限速器效果；TECS.th vs CTUN.ThO 的差 = 写入者切换 + 限速。

## 2. TECS 油门 vs 前推：同一个 k_throttle 的两个写入者

"TECS 油门"和"前推"**不是两个通道，是同一个 k_throttle（②）在两个阶段的两个写入者**：

```text
state 2（AIRBRAKE）:  TECS_controller.get_throttle_demand()  --:2423-->  k_throttle
state 3（POSITION1）:  forward_throttle_pct()                --:1807-->  k_throttle
                          └─ 本机 method=NONE → 恒 0（"前推需求"=0）
```

- `forward_throttle_pct()`（`quadplane.cpp:3831-3865`）：NEW 方法返回 `100×q_fwd_throttle`（`:3834-3836`）；OLD 方法返回 vel_forward 积分器输出（`:3863` 之后）；**NONE 返回 0**（`:3863-3865`）。
- `tiltrotor.cpp:521` 的油门映射公式两个阶段**共用同一行代码**，变的只是 k_throttle 的写入者与数值。
- 所以"state 3 换成前推需求映射"的准确表述：**写入者从 TECS（:2423）换成 fwd_thr（:1807），值从 TECS 遗留 100% 变为目标 0%**，经通道限速放坡，tilt_raw 跟随衰减。`atan(fwd_g_demand)` 分支（`tiltrotor.cpp:482-493`）本机未参与（需 method=NEW）。

## 3. AIRBRAKE 段 TECS.th 爬升机理（0.22→1.00）

TECS 日志关键字段（`AP_TECS.cpp:1318-1333` 官方定义）：`spdem`=TECS 想达到的真空速、`sp`=当前估计真空速、`th`=油门需求、`hdem/h`=高度需求/高度。

209 实测（state 2 全程 hdem=20.0 恒定，即高度任务不变）：

| sec | spdem | sp | th |
|-----|-------|-----|-----|
| 159.8 | 51.5 | 51.4 | 0.41 |
| 160.5 | 51.3 | 51.0 | 0.24 |
| 163.5 | 50.3 | 48.9 | 0.58 |
| 167.0 | 49.2 | 46.5 | **1.00 饱和** |

机理：桨盘阻力上来后 **sp 掉得比 spdem 快**（目标只从 51.5 缓降到 49.2，实际从 51.4 掉到 46.5），空速误差扩大 → TECS 加油门扛阻力 → 饱和。它把"保空速"当任务，但阻力是它打不赢的敌人。**这个饱和正是 tilt_raw 被推到 45° 封顶的动力源**（油门映射 k≥50% 即封顶）。

## 4. vel_forward 详解

### 4.1 定义与归属

```cpp
// ArduPlane/quadplane.h:424-429（QuadPlane 成员，匿名 struct）
struct {
    AP_Float gain;       // ← 参数 Q_VFWD_GAIN（quadplane.cpp:174 声明，默认 0）
    float integrator;    // 前推油门积分器，[-100, 100]（%）
    uint32_t last_ms;    // 上次运行时间戳
    float last_pct;      // 上次输出（%，OLD 分支的返回值）
} vel_forward;
```

### 4.2 它是什么

**老方法（method=OLD）的前推油门积分器**：在带位置/速度控制器的 VTOL 模式（QLOITER/QLAND/QRTL/AUTO VTOL 等）里，用**前向速度误差的积分**生成前推油门，喂给前推电机/前倾旋翼。参数：

- `Q_VFWD_GAIN`（`quadplane.cpp:174`）：积分增益。**本机 =0 → 整条链路未启用**。
- `Q_VFWD_ALT`（`quadplane.cpp:223`）：低于此离地高度前推淡出（落地保护）。

### 4.3 计算体（`quadplane.cpp:3860-3948`，10Hz）

```text
入口: get_vfwd_method() == OLD（:3863，Q_VFWD_GAIN>0 且 pos_control is_active_xy）
:3882  取位置控制器期望速度
:3895  速度误差转机体系取 x 分量
:3900  归一化 /= MAX(airspeed_max, 5)
:3905  减去俯仰需求贡献（低头减速直接反映到前推）
:3914  integrator += 误差 × dt × gain × 100
:3918  限幅 [throttle_min, throttle_cruise]
:3939-3943  低高度淡出（Q_VFWD_ALT）
```

### 4.4 与 209 架次的关系：零关系

1. `Q_VFWD_GAIN=0` → method=NONE（`quadplane.cpp:2943-2945`）→ `:3863` 入口即 return 0。
2. 唯一沾边：交接拍 `:2502` 把 k_throttle 当前值（TECS 遗留 100）读进 `vel_forward.integrator` 当初值——但随后**无人更新也无人消费**（gain=0、method 判定就返回 0），死代码路径。
3. state 3 的 k_throttle 斜坡是 THR_SLEWRATE 通道限速，与 vel_forward 无关。
4. 何时才真正参与：设 `Q_VFWD_GAIN>0` 激活 OLD 方法后，POSITION1 段"低头刹车+前推补速"的协调由它做。

## 5. 确认：AIRBRAKE 段 Z 环确实不跑（档 A 判据成立）

`run_z_controller()` 全部 6 个调用点：`mode_qloiter.cpp:175`、`mode_qrtl.cpp:120`（Climb 子模式）、`hold_hover` 内 `quadplane.cpp:1125`、`vtol_position_controller` 末尾 `:2896`、`waypoint_controller` 内 `:3236`、`control_auto` 内 `:3280`。

- QRTL 着陆段走 `vtol_position_controller()`（`mode_qrtl.cpp:160`），state 2 期间 `:2896` 被 suppress 拦住，`hold_hover(0)`（`:2443`）只在 tilt≤45° 后跑一次且随即防重入。
- 纯 FW 段 `run_z_controller` 开头 spool 检查早退（`:1054`）。
- 结论：**state 2 期间 Z 环一直没跑，`last_pidz_active_ms` 停留在 FW 段之前的旧值 → 进 AIRBRAKE 第一拍 `>2000ms` 即成立**，档 A 判据链完整成立。

## 6. 30° 抬头的来源链（accel→angle 逐级验证）

### 6.1 POSITION1 减速链全貌

```text
:2610  target_accel = MIN(accel_needed(distance, v²), 2×Q_TRANS_DECEL)   运动学减速需求
:2668  pos_control->input_vel_accel_xy(target_speed_xy_cms, target_accel_cms)
:2671  run_xy_controller(MAX(target_accel, Q_TRANS_DECEL)×1.5)
         :2203-2206  accel_cmss = MAX(wpnav_accel, accel_limit×100)      位置控制器加速度上限
         :2214  set_lean_angle_max_cd(MIN(45°, MAX(accel_to_angle(accel_limit), Q_ANGLE_MAX)))
AC_PosControl::update_xy_controller（libraries/AC_AttitudeControl/AC_PosControl.cpp:705-723）
         :706-710  angle_max=MIN(althold_lean, lean_max_override)；accel_max=angle_to_accel(angle_max)
         :710  limit_accel_xy —— 加速度目标被钳到 accel_max
         :723  accel_to_lean_angles —— 加速度目标 → roll/pitch 目标角
:2684-2685  nav_roll_cd/nav_pitch_cd ← pos_control->get_roll_cd()/get_pitch_cd()
:2689  set_VTOL_roll_pitch_limit（Q_BACKTRANS_MS 包络在此，见 6.3）
:2700  attitude_control->input_euler_angle_roll_pitch_yaw(...)
```

`accel_to_angle(a) = degrees(atan(a/g))`（`libraries/AP_Math/control.cpp:535-539`）。

### 6.2 数值验证（209）

Q_TRANS_DECEL=2 → target_accel ≤ 4 → accel_limit = MAX(4,2)×1.5 = **6 m/s²** → lean 上限 = atan(6/9.81) = **31.4°**（> Q_ANGLE_MAX=20°，故 20° 的 floor 不起作用，MIN(45°,31.4°)=31.4°）。实测 DesPitch 峰值 30.9°（172.6s）、Pitch 31.5°（171.6s）——**与 31.4° 上限精确吻合，峰值是被 lean-angle-max 钳出来的**。注意 `set_VTOL_roll_pitch_limit` 对正俯仰只在极低速（<0.5×airspeed_min，`quadplane.cpp:4611-4612`）才钳，20° 的 Q_ANGLE_MAX 也不约束正俯仰（`:4598` 只钳负向）——所以 30° 能放出来。

### 6.3 Q_BACKTRANS_MS 包络机理（含重置语义修正）

`SLT_Transition::set_VTOL_roll_pitch_limit()`（`quadplane.cpp:4580-4644`）：

- 参数 `Q_BACKTRANS_MS`（`quadplane.cpp:456`，默认 3000；本机 3000）。
- 包络：`max_limit = 线性插值(last_fw_nav_pitch≈2° → MIN(Q_ANGLE_MAX=20°, pitch_limit_max=15°)=15°, dt, 0, Q_BACKTRANS_MS)`，`pitch_cd = MIN(pitch_cd, max_limit)`（`:4637-4643`）。
- **重置语义**：`set_last_fw_pitch()`（`:4672-4676`）重置计时器。调用点包括 FW 段每拍（`:1682`）、进 APPROACH 直接 POSITION1（`:2240`）、**AIRBRAKE→POSITION1 交接（`:2499`）**等。即 **dt 从 POSITION1 进入时刻重新计数**（209：state3 起点 167.76s，峰值 171.61s，dt=3.85s > 3.0s 包络刚过期 → 30.9° 无约束放出，与日志一致）。
- 包络上限 15°：即使延长 Q_BACKTRANS_MS，包络末端也只到 15°，过期后完全放开。

## 7. 限制方案（气动包线 10°）

### 7.1 方案①（推荐）：Q_BCK_PIT_LIM 对 method=NONE 生效（一行级代码改动）

`Q_BCK_PIT_LIM`（`quadplane.cpp:543-548`，默认 10，本机=10）注释原文就是为此设计：*"prevent excessive structural loads when pitching up to decelerate. pitch_limit = Q_BCK_PIT_LIM × (AIRSPEED_MIN/IAS)²"*——**速度平方反比**：AS=45（=AIRSPEED_MIN）时限 10°，AS=20 时放开到 50°，低速段不拖累减速。

应用块在 `assign_tilt_to_fwd_thr()` 内（`:2987-3003`）：0.5s 平滑（`:2996-3000`）+ `nav_pitch_cd = MIN(nav_pitch_cd, limit)`（`:3003`）+ QBRK 日志（`:3006-3014`）。**但整个块在 method=NONE 的提前 return（`:2954-2957`）之后——本机 USE=0 根本执行不到**。

改法（二选一）：

- **最小 diff**：把 `:2987-3014` 的 BCK_PIT_LIM 块挪到 method 判定之前（或复制一份），使 USE=0 也生效；
- **更干净**：在 POSITION1 分支 `:2685` 之后（`nav_pitch_cd` 赋值后、`:2700` 姿态输入前）直接施加同样的速度平方钳制，钳到时调 `pos_control->set_externally_limited_xy()`（防位置控制器 I 项 windup，镜像 `:2689-2691` 的既有模式）。

效果预估（209 数据代入）：峰值需求 30.9° 时 AS≈39 m/s → 限值 = 10×(45/39)² = 13.3°，峰值被压到 ~13°；减速后段 AS<42 时逐步放开。**需同步看 7.4 的减速账**。

### 7.2 方案②（纯参数）：Q_BACKTRANS_MS 延长

包络是"从 2° 线性放开到 15°"的**上升斜坡钳制**。延长它 = 让钳制罩住整个减速段：

| Q_BACKTRANS_MS | dt=3.9s 时钳制值 | 末端 | 评估 |
|----------------|-----------------|------|------|
| 3000（现状） | 已过期，30.9° 放出 | 3s | 峰值超标 |
| 8000 | 2+13×3.9/8 = 8.3° | 8s 后放开 | 减速未完（dt=8s 时 AS≈19.5）放开后 ~20° 仍超 |
| **10000~12000** | 6~7° | 10~12s 后放开 | 减速基本完成（AS<10），放开时需求已 <5° ✓ |

建议 **Q_BACKTRANS_MS = 12000**。代价：减速段前半抬头被压，减速变慢——见 7.4。

### 7.3 方案③（代码）：压低 :2671 的加速限→lean 角链

期望角由 accel 经 `:2214` 映射，压 accel_limit 即压角度：`pitch_cap = atan(1.5×MAX(target_accel, decel)/g)`。

| Q_TRANS_DECEL | accel_limit | pitch_cap |
|---------------|-------------|-----------|
| 2（现状） | 6 | 31.4° |
| 1.0 | 3 | 17.0° |
| 0.58 | 1.73 | **10.0°** |

问题：`target_accel` 同时是 input_vel_accel_xy 的整形减速需求（`:2610` 上限 2×decel），压到 0.58 使整条减速剖面慢 3~4 倍，**不建议单独用**；且 `run_xy_controller` 里 `accel_cmss = MAX(wpnav_accel, accel_limit×100)`（`:2203-2206`）有 wpnav 加速度下限，还需确认其不顶掉限幅。此法只作理论参照——**期望角限制的正确位置是姿态需求端（方案①），不是加速度需求端**。

### 7.4 推荐组合与落点偏差账

```text
① 代码：BCK_PIT_LIM 对 method=NONE 生效（方案①，速度平方律，10° 包线硬保证）
② Q_BCK_PIT_LIM = 8~10（气动 10° 留裕度；本机已是 10，不动）
③ Q_BACKTRANS_MS = 12000（上升斜坡软限制，与①双保险；纯参数先行可单独验证）
④ 落点偏差接受：限幅后减速 ~3 m/s² → ~1.5 m/s²，45→9.5 m/s 需 ~6.5s → ~13s，
   减速距离 ~100m → ~175m（+75m）。stop_distance 机制自动提前进 AIRBRAKE，
   approach 航点离降落点 ≥200m 即可；POSITION2 门限（10m/3m/s，:2708-2709）晚到 2-4s，
   前飘量 SITL 量化后写进任务航点偏置。
```

验证路径：SITL 复跑 209 同任务 → 核对 ATT.Pitch 峰值 ≤10°、QBRK 日志（限幅生效后自动出现）、落点偏差、减速总时长（>20s 则加大 approach 距离或 Q_TRANS_DECEL 名义值）。

---

## 附：本文涉及锚点速查

| 内容 | 位置 |
|------|------|
| TECS 日志字段定义 | `libraries/AP_TECS/AP_TECS.cpp:1314-1352` |
| accel_to_angle | `libraries/AP_Math/control.cpp:535-539` |
| 位置控制器 lean 上限→加速度钳制 | `libraries/AC_AttitudeControl/AC_PosControl.cpp:705-723` |
| CTUN.ThO/ThD 日志源 | `ArduPlane/Log.cpp:112,114` |
| QTUN.ThO 日志源 | `ArduPlane/quadplane.cpp:3803` |
| k_throttle 两写入者 | `ArduPlane/quadplane.cpp:2423`（TECS）/ `:1801-1808`（fwd_thr） |
| vel_forward struct / 参数 | `ArduPlane/quadplane.h:424-429` / `quadplane.cpp:174,223` |
| vel_forward 计算体 | `ArduPlane/quadplane.cpp:3860-3948` |
| run_z_controller 调用点 | `mode_qloiter.cpp:175`、`mode_qrtl.cpp:120`、`quadplane.cpp:1125,2896,3236,3280` |
| POSITION1 减速链 | `ArduPlane/quadplane.cpp:2610,2668,2671,2684-2685,2689,2700` |
| run_xy_controller / lean 上限 | `ArduPlane/quadplane.cpp:2201-2219`（`:2214`） |
| Q_BCK_PIT_LIM 参数/应用/日志 | `ArduPlane/quadplane.cpp:543-548,2987-3014` |
| Q_BACKTRANS_MS 参数/包络/重置 | `ArduPlane/quadplane.cpp:451-456,4622-4644,4672-4676`（调用点 `:1682,2240,2499`） |
| set_VTOL_roll_pitch_limit 本体 | `ArduPlane/quadplane.cpp:4580-4644` |
