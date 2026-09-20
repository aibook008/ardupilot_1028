# DF-200 固定翼滚转/转弯调参专题

> 代码/模型：Plane 4.6.x / `qhz/plane_4.6.3`，`Tools/autotest/models/df200.json`
> 数据源：`logs/00000194`–`00000197.BIN`（ATT/PIDR/NTUN/CTUN/ARSP/BARO/AOA）
> 关联：`docs/df200_control_freq_response.md`（控制环幅频总论）、`docs/vtol_df200.md`、`docs/df200_session_20260902.md`

本文档专门记录 DF-200 固定翼段**滚转控制 + 航点转弯**的调参全过程，从 194 转弯炸机到 197 调成基线的四次迭代。**当前基线参数见 §6。**

---

## 0. 阅读指引

| 顺序 | 章节 | 作用 |
|---|---|---|
| 1 | §6（基线） | 197 调好的参数，直接抄 |
| 2 | §1（迭代记录） | 194→195→196→197 每次改什么、什么效果 |
| 3 | §2–§4 | 三个关键参数的机理与坑（RMAX / L1 / LIM_BANK） |
| 4 | §5 | 下一步：切点时机优化 |

---

## 1. 调参迭代记录（194→197）

### 194 — 起点问题：转弯炸机

| 项 | 内容 |
|---|---|
| 参数 | NAVL1_PERIOD=14, WP_RADIUS=90, RLL2SRV_RMAX=40, ROLL_LIMIT_DEG=65 |
| 现象 | WP2→WP3 航向跳变 84°，t=191.93 后 Roll 1.5s 才追上 DesRoll，过冲 14%，NavBrg 追不上撞地 |
| 根因 | RLL2SRV_RMAX=40（stock 小飞机默认）把期望角速率限死，Roll 追不上 |
| 详见 | `df200_control_freq_response.md` §1 |

### 195 — 放开 RMAX，但暴露大倾角掉高

| 项 | 内容 |
|---|---|
| 参数改动 | RLL2SRV_RMAX 40→120（生效），NAVL1_PERIOD 14→22，WP_RADIUS 90→200，NAVL1_LIM_BANK=55 |
| 现象 | 转弯没炸（RMAX 让 Roll 追上了），但 Roll 飙到 66°、alt 掉到 1.1m（擦地）、AS 飙到 77.9 m/s（临界）、反弹飙到 91m |
| 根因 | RMAX 解决了"转得动"，但暴露"大倾角升力失配"：65° 倾角下垂直升力只剩 0.42·L，AOA 才 3° 不够补，掉高转速度 |
| 失败模式变化 | 从"转不过来撞地"→"转过来了但掉高+超速" |
| RTL 段 | 反向倾转时 alt 从 15m 飙到 90m，Pitch +24.7°（单独问题，§5 待办） |

### 196 — 加大 WP_RADIUS，减轻但没根治

| 项 | 内容 |
|---|---|
| 参数改动 | WP_RADIUS 200→400，NAVL1_LIM_BANK 55→30（试图限倾角），RMAX 保持 120 |
| 现象 | 不再擦地（alt min 16.1m）、不再飙升（alt max 30m）、AS 最高 70.8。**但 Roll 仍 63.9°** |
| 关键发现 | **NAVL1_LIM_BANK=30 完全没起到限倾角作用**（Roll 还是 63°）——见 §4 代码事实 |
| 结论 | WP_RADIUS 加大方向对（提前切点），但倾角没限住 |

### 197 — 基线：ROLL_LIMIT_DEG=40，调好了

| 项 | 内容 |
|---|---|
| 参数改动 | **ROLL_LIMIT_DEG 65→40**（真正限倾角），NAVL1_LIM_BANK→0（删无用参数），WP_RADIUS=400, RMAX=120, PERIOD=22 |
| 现象（转弯段 t=151–157） | Roll 稳定在 39.7–41.1°（精准顶在 40°限幅）、alt 18.7–21.9m（几乎不掉高）、AS 54.2–55.8（纹丝不动）、DesRoll 跟踪误差仅 1.2° |
| 稳态段（t=90–145） | Roll ±0.18°、AS ±0.6、Alt ±3.4（极稳，无 phugoid） |
| 结论 | **转弯调好了，作为基线** |

### 迭代效果对比表

| 指标 | 194 | 195 | 196 | **197** |
|---|---|---|---|---|
| Alt 最低 | 坠地 | 1.1m（擦地） | 16.1m | **18.7m** |
| Alt 最高 | — | 91m（飙升） | 30m | **21.9m** |
| AS 最高 | 55→坠地 | 77.9（临界） | 70.8 | **55.8** |
| Roll 峰值 | 跟不上→炸 | 66° | 63.9° | **41.1°** |
| DesRoll 跟踪 | 差 42° | 过冲 | 过冲 | **误差 1.2°** |
| 坠地 | 是 | 否 | 否 | **否** |

---

## 2. RLL2SRV_RMAX — 期望滚转角速率上限

**代码事实**（已验证行号）：
- `libraries/APM_Control/AP_RollController.cpp:47`：参数定义
- `AP_RollController.cpp:286-290`：限幅点

```cpp
if (gains.rmax_pos && desired_rate < -gains.rmax_pos) {
    desired_rate = -gains.rmax_pos;
} else if (gains.rmax_pos && desired_rate > gains.rmax_pos) {
    desired_rate = gains.rmax_pos;
}
```

### 关键认知：RMAX 不是"增益"，是"请求限幅"

- RMAX 限制的是**导航/角度环请求的滚转速率**（deg/s），不是 PID 的 P/I 增益
- **不是"大飞机改 rate 增大更危险"**——增大 RMAX 只是把"飞控请求的拐弯速率上限"放开，不改变舵面能力或 PID 增益
- 194 证据：`PIDR.Tar`（期望速率）被钉死在 40 达 1.1s，`PIDR.Act`（实际速率）顶在 40 平台 → 不是舵效不够，是请求被限幅
- stock 默认 40 是针对 Skywalker（0.5kg 小机），DF-200 是 350kg 大机，40 太严

### 195 验证：RMAX=120 让 Roll 追上 DesRoll，但不够

RMAX 40→120 后 Roll 能追上 DesRoll 了（不再晚 1.5s），但暴露了大倾角掉高问题。**RMAX 解决"转得动"，不解决"倾角太大"**——后者要靠 ROLL_LIMIT_DEG。

### 结论

RMAX=120 是合适的（197 沿用），**不是危险参数**。它只放开请求上限，真正的安全由 ROLL_LIMIT_DEG 兜底。

---

## 3. NAVL1_PERIOD — L1 引导的"追哪个点"

**代码事实**（已验证行号）：
- `libraries/AP_L1_Control/AP_L1_Control.cpp:257`：L1 距离公式
- `AP_L1_Control.cpp:361-363`：circle tracking 增益

```cpp
// :257 — L1 距离
_L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);
// = (1/π) · damping · period · V

// :361-363 — circle tracking 增益（loiter/盘旋时）
float omega = (6.2832f / _L1_period);  // = 2π/period
float Kx = omega * omega;              // 位置增益 ∝ 1/period²
float Kv = 2.0f * _L1_damping * omega; // 阻尼 ∝ 1/period
```

### 物理意义：飞机追的是"目标前方 L1 距离处的虚拟点"

```
L1_dist = 0.3183 × damping × period × groundSpeed
        = 0.3183 × 0.75 × period × 55
```

| period | L1 距离 (V=55) | 效果 |
|---|---|---|
| 14 | 184m | 追近点，拐得早、激进，可能抖 |
| 17 (默认) | 223m | 标准 |
| 22 (现用) | 290m | 追远点，平缓、拐得晚 |
| 25 | 329m | 更平缓、更晚 |

### "越大越平缓"对，但有代价

- **period 大** → L1 距离大 → 瞄准目标前方更远处 → 转弯曲、平缓
- **但平缓 = 拐得晚**：飞机接近航点才开始大幅修正
- **period 大 → omega 小 → Kx/Kv 小** → 跟踪增益软（第二重"平缓"）

### 与 WP_RADIUS 必须配套

L1 引导何时开始拐弯由 L1_dist 决定；WP_RADIUS 决定何时判定"到达切下一点"。两者关系（`AP_L1_Control.cpp:124-127`）：

```cpp
float AP_L1_Control::turn_distance(float wp_radius) const {
    wp_radius *= sq(_ahrs.get_EAS2TAS());
    return MIN(wp_radius, _L1_dist);  // 实际转弯提前量取两者较小值
}
```

- **WP_RADIUS < L1_dist**：切点早于引导完成，飞机冲到航点跟前才切方向 → L1 虚拟点瞬变 → 硬拐（195 的现象）
- **WP_RADIUS ≥ L1_dist**：切点和引导配合，平滑过渡（196/197 的改善）

### 197 的取值 PERIOD=22

PERIOD=22 → L1_dist≈290m，配 WP_RADIUS=400（>290），配合得当。**PERIOD 不是 197 成功的关键**，ROLL_LIMIT_DEG 才是。PERIOD 可在 18–22 间微调，当前 22 可接受。

---

## 4. NAVL1_LIM_BANK 的坑——对航点转弯无效

**这是本次调参最大的坑，必须记录防止复发。**

### 代码事实（已验证行号）

- `AP_L1_Control.cpp:39`：`NAVL1_LIM_BANK` 对应 `_loiter_bank_limit`
- `AP_L1_Control.cpp:79-94`：`nav_roll_cd()`——**航点转弯倾角由它算，只受 ±90° 硬限，不读 LIM_BANK**
- `AP_L1_Control.cpp:151`：`_loiter_bank_limit` 只在 `loiter_radius()` 里用
- 调用点：`mode_loiter.cpp:46`、`mode_rtl.cpp:151`——**只用于 loiter/盘旋模式算半径**

```cpp
// AP_L1_Control.cpp:79-94 — 航点转弯倾角（不读 LIM_BANK）
int32_t AP_L1_Control::nav_roll_cd(void) const {
    float ret;
    float pitchL1 = constrain_float(_ahrs.get_pitch(),-pitchLimL1,pitchLimL1);
    ret = degrees(atanf(_latAccDem * (1.0f/(GRAVITY_MSS * cosf(pitchL1))))) * 100.0f;
    ret = constrain_float(ret, -9000, 9000);  // 只限 ±90°，不读 LIM_BANK
    return ret;
}
```

### 实测验证

196 设 NAVL1_LIM_BANK=30，实际 Roll=63.9°（完全没限住）→ 证实对 WP 转弯无效。

### 正确的倾角限幅参数：ROLL_LIMIT_DEG

`nav_roll_cd()` 返回后，在 `ArduPlane/Attitude.cpp:608` 被 `roll_limit_cd`（=ROLL_LIMIT_DEG×100）约束：

```cpp
// Attitude.cpp:607-608
int32_t commanded_roll = nav_controller->nav_roll_cd();
nav_roll_cd = constrain_int32(commanded_roll, -roll_limit_cd, roll_limit_cd);
```

- `ArduPlane/Parameters.cpp:544`：`ROLL_LIMIT_DEG` → `aparm.roll_limit`
- `Attitude.cpp:295-296, 608, 644`：多处用 `roll_limit_cd` 约束 nav_roll
- **这才是全局倾角硬限，对航点转弯/loiter/FBW 全部生效**

### 教训

| 参数 | 名字误导 | 实际作用 | 对 WP 转弯 |
|---|---|---|---|
| NAVL1_LIM_BANK | "LIM_BANK" 像限倾角 | **只限 loiter 半径** | **无效** |
| ROLL_LIMIT_DEG | 不带 LIM 字样 | **全局倾角硬限** | **有效** |

**下次要限倾角，改 ROLL_LIMIT_DEG，别碰 NAVL1_LIM_BANK。**

---

## 5. 下一步：切点时机优化

197 转弯已不炸，但切点时机仍偏晚（出弯后 NavBrg 过冲 40° 再收回）。三个方向，从易到难：

### 5.1 WP_RADIUS 继续加大（最简单）

现 WP_RADIUS=400 ≈ L1_dist=290。加大到 600，切点更早，飞机在更远处平滑拐向 WP3。

### 5.2 NAVL1_PERIOD 减小

PERIOD 22→16，L1_dist 290→211m，引导更激进、拐得更早。但太小会抖，需试。

### 5.3 Spline waypoint（最干净，但需验证支持）

把 WP2/WP3 改成 spline 类型（`MAV_CMD_NAV_SPLINE_WAYPOINT` cmd=82），飞机沿样条曲线平滑过弯，全程小倾角，不靠 L1 硬拐。

**待验证**：QuadPlane 的 RTL 路径（自动生成航点）是否支持 spline。需查 `ArduPlane/commands_logic.cpp` 和 QuadPlane RTL 实现。

### 5.4 RTL 反向倾转升高（195 暴露，197 未飞到，单独待办）

195 在 RTL 反向倾转时 alt 从 15m 飙到 90m（tilt 回转时桨推力垂直分量骤增 + 前飞速度 50 m/s 动能）。197 这趟 t=176 即结束没飞到 RTL。下一飞需专门验证 RTL 段。

可能方向：
- `Q_TILT_RATE_UP` 10→3–5（反向倾转放慢）
- RTL 阶段设更低目标速度，AS<15 才开始 tilt 回转
- `Q_TRANSITION_MS` 给反向油门衰减时间

---

## 6. 当前基线参数（197，已验证）

### 固定翼滚转/转弯相关

| 参数 | 197 值 | 作用 | 备注 |
|---|---|---|---|
| **ROLL_LIMIT_DEG** | **40** | 全局倾角硬限 | 197 核心，65→40 解决大倾角掉高 |
| RLL2SRV_RMAX | 120 | 期望滚转速率上限 | 195 验证够用，放开"转得动" |
| RLL2SRV_TCONST | 0.5 | 角度→速率时间常数 | stock |
| NAVL1_PERIOD | 22 | L1 距离/引导软硬 | L1_dist≈290m，可微调 |
| NAVL1_DAMPING | 0.75 | L1 阻尼 | stock |
| WP_RADIUS | 400 | 航点到达判定 | >L1_dist，配合得当 |
| NAVL1_LIM_BANK | 0（删） | 无用，只管 loiter | §4 坑，别再设 |
| SCALING_SPEED | 45 | 速率环增益缩放 | 187 修复，OK |
| AIRSPEED_CRUISE | 55 | 巡航速度 | 189 修复，OK |
| TECS_TIME_CONST | 5 | TECS 高度环 | 190 修复，OK |

### 转弯半径物理核算

倾角 40° + 巡航 55 m/s：
```
转弯半径 r = V² / (g·tanφ) = 55² / (9.81 × tan40°) = 3025 / 8.23 ≈ 368m
```

WP_RADIUS=400 > 368m，物理转弯放得下。航点间距 WP2→WP3=2.56km，368m 半径绰绰有余。

### 197 验证数据

**转弯段（t=151–157）**：

| t | Roll | DesRoll | alt | AS |
|---|---|---|---|---|
| 151.14 | 39.7 | 40.0 | 18.7 | 55.8 |
| 152.14 | 41.1 | 39.9 | 19.9 | 55.5 |
| 156.24 | 40.2 | 40.0 | 21.8 | 54.4 |

**稳态段（t=90–145）**：Roll ±0.18°、AS ±0.6、Alt ±3.4（极稳，无 phugoid）

---

## 7. 相关文档

| 文档 | 内容 |
|---|---|
| `docs/df200_control_freq_response.md` | 控制环幅频总论，含 194 速率限幅详细分析、历次控制问题索引 |
| `docs/vtol_df200.md` §11 | 横航向自激墙 + position%gyro 正反馈机理 |
| `docs/df200_session_20260902.md` | 185–190 纵向修复链（SCALING_SPEED/CRUISE/TIME_CONST） |
| `docs/scripts/df200_transition_log.py` | 里程碑/onset 诊断 |

---

## 8. 调参防复发清单

1. **要限倾角改 ROLL_LIMIT_DEG，别碰 NAVL1_LIM_BANK**（§4 坑）
2. **WP_RADIUS 必须 ≥ L1_dist**（L1_dist=0.3183×damping×period×V），否则切点早于引导完成，硬拐
3. **RMAX 不是增益、不是危险参数**，只放开请求速率上限；大飞机从 stock 40 放到 120 合理，安全由 ROLL_LIMIT_DEG 兜底
4. **每飞只改一类参数**（185→190 教训，因果链干净的前提）
5. **改参后 `mavlogdump.py --type PARM` 核对生效**（188 假失败教训；195 的 `RLL2SRV_RMAX 120–180` 非法值被解析成 120 也是坑）
6. **本日志 TimeUS 重建时间轴**：`_timestamp` 失效，必须用 `d['TimeUS']/1e6`；ATT/ARSP/PIDR ~10Hz，RATE 20Hz 但语义不同，roll rate 看 PIDR.Act
