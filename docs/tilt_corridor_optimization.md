# 倾转走廊优化计划（θ–V 门控为主）

| 项 | 内容 |
|----|------|
| 文档编号 | FC-VTOL-INT-CORRIDOR-OPT-001 |
| 版本 | V0.9 |
| 日期 | 2026-09-17 |
| 性质 | 内部实现计划（参数化门控 → 仿真验证 → 功率兜底） |
| 代码基线 | 分支 `qhz/plane_4.6.3`（Plane 4.6.x） |
| 关联 | [`JT200_tilt_corridor.md`](JT200_tilt_corridor.md)、[`tilt_corridor_vs_apm.md`](tilt_corridor_vs_apm.md)、[`tilt_corridor_catchup_plan.md`](tilt_corridor_catchup_plan.md)、[`throttle_calculation.md`](throttle_calculation.md)、[`df200_power_corridor.md`](df200_power_corridor.md) |
| 脚本 | [`scripts/df200_corridor.py`](scripts/df200_corridor.py) |

---

## 1 目的与结论先行

### 1.1 目的

把半倾转过渡从「单点 `AIRSPEED_MIN` + TIMER 开环打满」升级为**可参数化的 θ–V 走廊门控**，使实际轨迹落在功率上界与升力下界围成的包线内。

### 1.2 结论先行

| 判断 | 说明 |
|------|------|
| **主因** | 倾转过快、空速未跟上 → 垂向缺口大 → 油门/功率顶黄区（表象） |
| **主手段** | WAIT **断点门控**：要越过 `CRD_Tn` 时须 `V≥CRD_Vn`，否则 `hold` 在 `current_tilt`；**不是**每帧 `θ_max(V)` 插值 |
| **与 Q_TILT_MAX** | WAIT 的 `tilt_raw` **仍封顶 `Q_TILT_MAX`**（与原生相同）；门控只约束「爬到 MAX 之前」的若干角 |
| **TIMER 角色** | **不跑门控**；开环打满倾 + 集体油门时间衰减。`T4=Q_TILT_MAX` 时若 TIMER 仍 `min` 会卡死满倾 |
| **辅手段** | 分阶段功率帽 + 置 `throttle_upper`（兜底，不替代门控） |
| **参数对齐** | 表末点与 `Q_TILT_MAX` / `AIRSPEED_MIN` 同量级（现默认 45）；**勿**把关外桨 `V_min(θ)` 表直接当 WAIT 加速门 |
| **不先做** | 完整最优轨迹规划器；上游合入；在 TIMER 内加长等待 / 改 TECS（O5） |

### 1.3 与 catch-up 计划关系

本文件是 [`tilt_corridor_catchup_plan.md`](tilt_corridor_catchup_plan.md) 在**半倾转 / JT-200·DF-200** 上的实现细化：

- catch-up：要对甲具备「可监视、可处置」能力  
- 本计划：对内先落地 **θ–V 门控参数 + SITL 验收**，功率限幅作第二阶段

---

## 2 问题与现状

### 2.1 原生状态机（仅一段 WAIT）

```text
AIRSPEED_WAIT ── V > AIRSPEED_MIN ──► TIMER ──► DONE
  θ ≤ Q_TILT_MAX（跟 k_throttle）     θ → 90°（开环 RATE）
  hold_hover                          hold_stabilize 衰减
```

代码锚点（改前请重核行号）：

| 行为 | 位置 |
|------|------|
| 状态枚举 | `ArduPlane/transition.h`：`TRANSITION_AIRSPEED_WAIT / TIMER / DONE` |
| WAIT 退出 | `ArduPlane/quadplane.cpp` `SLT_Transition::update`：`aspeed > airspeed_min` |
| WAIT 倾转上限 | `ArduPlane/tiltrotor.cpp` `continuous_update`：θ←`k_throttle`，封顶 `Q_TILT_MAX` |
| TIMER 强制打满 | 同文件：`slew(get_forward_flight_tilt())` |

### 2.2 半倾转矛盾（JT-200）

外四固定垂起，前飞必须关掉。走廊核心是：

> 在功率封顶内，空速涨到什么程度才能继续倾转 / 关外桨？

见 [`JT200_tilt_corridor.md`](JT200_tilt_corridor.md) §3：`V_min(θ)` 与 90/110/25 kW 封顶。

### 2.3 193 暴露的策略缺口

| 现象 | 含义 |
|------|------|
| 卡 45° 等空速 ~18 s | 单点门控；等的时候功率均值尚可（~76 kW），但进出尖峰贴 110 |
| TIMER 后 45→90 | 空速已过总门槛后仍可能 θ 增速 > V 增速 |
| 起飞尖峰 ~150 kW | 差动 + 大油门，与走廊门控正交，另案压 |

**优化方向**：多个关键 θ 都要「空速够了再往下倾」，而不是只在 45° 等一次。

---

## 3 目标架构

### 3.1 主约束：WAIT 断点门控（V0.6 实装）

```text
CRD_EN=0:
  WAIT:  θ_raw = f(k_throttle) ≤ Q_TILT_MAX
  TIMER: θ_raw = fully_forward
  θ_cmd = θ_raw → slew（RATE）

CRD_EN=1:
  WAIT:  θ_raw = f(k_throttle) ≤ Q_TILT_MAX   // 与原生相同
         θ_cmd = corridor_gate_tilt(θ_raw)     // 仅在越过 Tn 时查 Vn
  TIMER: θ_raw = fully_forward
         θ_cmd = θ_raw                         // **不跑门控**
  → slew(θ_cmd)
```

**断点语义（不是连续插值）：**

```text
爬向下一门 Tn：允许 θ_cmd = min(tilt_raw, Tn)     // 先到门口
已到达 Tn（current_tilt ≥ Tn−eps）且 raw 还要往前：
    若 speed ≥ Vn + margin → 放行 tilt_raw
    否则 hold 在 Tn
```

| 要点 | 说明 |
|------|------|
| 比什么 | **已经到达、还想越过的最高门**；未到门口不查速度 |
| 何时比 | 贴在 Tn 上还要前进时；爬升途中不 hold |
| hold 基准 | 门角 `Tn`（不是半途的 current） |
| 速度 | 优先空速；无 AS 时早期可用 GS；无效 → hold 在 Tn |
| 与关外桨表 | `V_min(θ)` 是外桨已关升力下界；**不要**直接当 WAIT 加速门 |

**205 bug：** 旧逻辑在 `current < Tn ≤ raw` 时就查 Vn → RATE 才爬到 ~1° 就被 hold，AS 永远到不了 V1=10 → 死锁。已改为「先爬到 Tn 再检查」。

曾用连续 `θ_max(V)=interp(V)` 的方案已弃：每帧贴曲线会压低中段 θ，Z 环油门下 \(T\sinθ\) 不够。

### 3.2 默认门控点（parm / 固件默认）

塑形 **0→`Q_TILT_MAX`**，末点对齐 45°；**45° 之后 WAIT 不再加门**（满倾只在 TIMER）：

| 点 | T ° | V m/s | 含义 |
|----|-----|-------|------|
| 1 | 10 | 10 | 低速允许小倾转 |
| 2 | 20 | 25 | |
| 3 | 30 | 35 | |
| 4 | 45 | 45 | 到 `Q_TILT_MAX`；再往后等 `AIRSPEED_MIN` 进 TIMER |

### 3.3 与现有参数共存

| 参数 | 角色 |
|------|------|
| `Q_TILT_MAX` | WAIT `tilt_raw` 上限（EN=0/1 皆然） |
| `AIRSPEED_MIN` | 结束 WAIT、进入 TIMER；建议与表末速度同量级（现 45） |
| `Q_TILT_RATE_*` | `slew` 转速；门控放行后仍限速爬升 |
| `Q_TRANSITION_MS` | TIMER 集体油门衰减（代码夹 0.5–30 s）；**不为等空速服务** |
| `Q_TILT_CRD_*` | 仅 WAIT 断点；TIMER 忽略 |

### 3.4 阶段分工（策略核心，V0.6）

```text
WAIT  (hold_hover，Z 环托高)
  ├─ θ_raw ≤ Q_TILT_MAX（跟 k_throttle）
  ├─ 越过 CRD_Tn 前检查 V≥Vn；不够则 hold@current_tilt
  ├─ 外桨保持；前向靠 T·sinθ
  └─ V > AIRSPEED_MIN → TIMER

TIMER (hold_stabilize，开环衰减)
  ├─ throttle = last × (1−t/T)     ← 撤垂向；半倾转不混 TECS
  ├─ θ → 90°（**无走廊 min**）
  └─ 禁止当成长时间等空速

DONE  → 外桨 SHUTDOWN / TECS 接棒
```

| 阶段 | 油门 | 倾转 | 等空速？ |
|------|------|------|----------|
| **WAIT** | Z 环闭环 | raw≤MAX + 断点门 | **是（仅跨门时）** |
| **TIMER** | 开环时间衰减 | 冲满倾 | **否** |
| **DONE** | TECS / 外桨关 | 前飞位 | — |

**为何 TIMER 必须跳过门控：** 表末常为 `T4=Q_TILT_MAX`；若 TIMER 仍 `min(fully_fwd, θ_max)`，θ 会永远卡在 45°。详见 §10.2。

**推荐调参：**

1. 表只覆盖 0→`Q_TILT_MAX`；不要在 45° 后用关外桨 `V_min` 再卡 WAIT  
2. `AIRSPEED_MIN` ≈ 表末 V（现 45）  
3. `Q_TRANSITION_MS` 只服务撤垂向快慢（有效 ≤30 s）  
4. SITL 看 `TILT.Crd`：WAIT 跨门可为 1；**TIMER 应为 0**

### 3.5 辅约束：功率兜底（第二阶段）

门控为主、且 WAIT 内等空速之后，功率帽只处理残余尖峰：

| 阶段 | 帽 | 动作 |
|------|----|------|
| VTOL 爬升 | 90 kW（尖峰可短超） | 集体油门 + 差动收缩 |
| WAIT 门控等待 / 倾转 | 90 可持续；110 累计 ≤10 s | 超黄区：**冻 θ**（与门控一致） |
| TIMER | 同倾转帽；窗口应已缩短 | 不在此阶段靠限功率「拖时间」 |
| FW | ~25 kW | TECS / 外桨已 SHUTDOWN |

限幅时必须 `set_external_limits(..., throttle_upper=true)`，复用 PosControl 饱和保护（见 §6）。  
**硬底**：`P_cap ≥ P_hold(V,θ)`（托重功率），否则掉高。

---

## 4 参数设计（先可配，后优化表）

### 4.1 建议新增参数组（名称可调）

已落地在 `Q_TILT_` 子组（全名 ≤16）。功率相关参数仍属阶段 C，尚未加。

| 参数 | 含义 | 固件/parm 默认 |
|------|------|----------------|
| `Q_TILT_CRD_EN` | 0=关闭 1=WAIT 断点门控 | parm 飞 DF-200 时为 1；固件默认 0 |
| `Q_TILT_CRD_MRG` | 门速度余量 m/s（加在 Vn 上） | 0 |
| `Q_TILT_CRD_T1`…`T4` | 倾转角断点 ° | 10 / 20 / 30 / 45 |
| `Q_TILT_CRD_V1`…`V4` | 越过对应 Tn 所需最小速度 m/s | 10 / 25 / 35 / 45 |

代码：`Tiltrotor::corridor_gate_tilt()` + `continuous_update`（**仅** `!in_timer`）。  
`current_tilt` 仅由 `slew()` 更新（RATE 限制后的内部状态）。

无正式气动包前：WAIT 表按**加速友好**断点，**不要**照抄关外桨 `V_min(θ)`；有飞次后再拧 Tn/Vn。

### 4.2 表关闭时的退化

```text
Q_TILT_CRD_EN=0 → 完全原生（Q_TILT_MAX + AIRSPEED_MIN；TIMER 开环打满）
便于 A/B：同一固件对比「单点 vs 多点门控」
```

### 4.3 日志（TILT 已扩展）

| 字段 | 含义 |
|------|------|
| `Tilt` | `current_tilt×90` °（`slew` 后内部状态） |
| `Traw` | 本拍 `tilt_raw×90` ° |
| `Tmax` | 正在检查的门 Tn °（未跨门时可为本角） |
| `Spd` | 门控用速度；无效/未查为 NaN |
| `Crd` | `0` 关闭或 TIMER；`1` hold 等速度；`-1` 开启未 hold |

脚本：`df200_transition_log.py` KEY_PARMS 含 `Q_TILT_CRD_*`；`df200_corridor.py` 画 (θ,V)。

### 4.4 实现要点（防回归）

1. **`current_tilt` 只在 `slew()` / `binary_slew` 写入**；门控 hold 时 `slew(current_tilt)` → 本拍不变。  
2. **跨门条件**：须先 `current_tilt ≥ Tn−eps` 且 `raw > Tn` 才查 Vn；禁止「raw 指向 Tn 但角还远」就 hold（205 死锁）；也禁止按 raw 最高门要速度。  

3. **速度只读一次**（`corridor_speed_ms`），勿在门控里再套已弃用的插值函数。  
4. **TIMER 禁止调用门控**。  
5. 连续 `θ_max(V)` 插值已删除，勿加回 WAIT 加速段。

---

## 5 实现分阶段

### 阶段 A — 监视与表（不改飞控行为）

| 任务 | 出口 |
|------|------|
| A1 固化默认 `(θ, V_min)` 表（380 kg 口径） | 与 `JT200_tilt_corridor` 数值一致 |
| A2 SITL 日志后处理：画实际 (θ,V) vs 走廊 | 图进 `docs/df200_corridor_imgs/` |
| A3 量化「θ 超前 V」时段与功率相关 | 确认主因可复现 |

### 阶段 B — WAIT 断点门控 — **V0.6 已合入，待 SITL**

| 任务 | 状态 | 出口 |
|------|------|------|
| B1 断点门 `corridor_gate_tilt`（跨 Tn 查 Vn / hold） | ✅ | 取代连续插值 |
| B2 仅 WAIT 调用；TIMER 跳过 | ✅ | 避免 T4=MAX 卡死满倾 |
| B3 `Q_TILT_CRD_*` + TILT 日志 Traw/Tmax/Spd/Crd | ✅ | |
| B4 WAIT `tilt_raw` 仍用 `Q_TILT_MAX` | ✅ | 与原生一致 |
| B5 表 10/10…45/45，`AIRSPEED_MIN=45` | ✅ | parm |

**落点**：`ArduPlane/tiltrotor.cpp` / `.h`，`Log.cpp` TILT 扩展。  

**下一步：** SITL `CRD_EN=0/1`；看 WAIT 平台是否出现在 Tn；TIMER 段 `Crd=0` 且 θ→90。
### 阶段 C — 功率兜底 + 饱和标志

| 任务 | 出口 |
|------|------|
| C1 电机表 `P_est`（与走廊脚本同构） | SITL 先通 |
| C2 超帽夹 collective + `set_external_limits(throttle_upper)` | PosControl 冻积分 |
| C3 `P_hold` 硬底；黄区累计冻 θ | 与门控同方向 |
| C4 起飞差动尖峰（可选）单桨 cmd 上限 | 降 150 kW 类尖峰 |

### 阶段 D — 走廊数据优化（有合理设计后再做）

| 任务 | 触发条件 |
|------|----------|
| D1 用正式气动 / 更多飞次重拟合 `V_min(θ)` | 表与实飞偏差可量化 |
| D2 加密断点或改 margin | 门控已稳定、只调形状 |
| D3 可选中线调度 \(θ_{\mathrm{cmd}}=f(V)\)（不只上限） | 需要更短过渡时间时 |
| D4 对甲脱敏写入 phase1 `05`/`06` | 与 catch-up T5 对齐 |

**原则**：先让门控结构正确且可关可开，再优化表；避免未验证结构时调点。

---

## 6 油门保护（限功率时如何「尽量不掉高」）

现有机制（**无**专用「限功率保高」模块）：

| 层 | 代码 | 作用 |
|----|------|------|
| 混控置位 | `AP_MotorsMatrix`：`throttle ≥ thrust_max` → `limit.throttle_upper` | 饱和标志 |
| 电流限 | `get_current_limit_max_throttle()`（`Q_M_BAT_CURR_MAX`） | 夹 `_throttle_thrust_max` |
| Z 环冻积分 | `AC_PosControl`：`update_all(..., throttle_upper)` | 防松限猛冲 |
| 高度目标 | `update_pos_vel_accel(..., _limit_vector.z)` | 饱和方向不再追高 |
| 外部限幅 | `set_external_limits(..., throttle_upper)` | **自研功率帽应走此口** |

对内策略：

1. 门控减少「必须顶油门」的时段（治本）  
2. 功率帽 ≥ `P_hold(V,θ)`（托重）  
3. 限幅必置 `throttle_upper`  
4. 功率紧张时可强制 `climb_rate ≤ 0`（tiltrotor 版 LEVEL_TRANSITION 精神）  
5. 仍掉高 → 松帽或走既有 transition failure / QLAND  

---

## 7 验收标准（SITL）

最小同时满足：

1. **轨迹**：全程 \(V \ge V_{\min}(θ) + \mathrm{margin}\)（允许短时去抖窗口内轻微触边）  
2. **关外桨**：θ≈90° 时 \(V \ge V_{\min}(90°)\)（现表 48.4；门槛建议 ≥50）  
3. **功率**：倾转段均值主要在 90 kW 下；≥110 kW 累计 ≤10 s  
4. **开关**：`Q_TILT_CRD_EN=0` 行为与改前一致（回归）  
5. **对比**：同任务 `CRD_EN=1`+`AIRSPEED_MIN≈CRD_V4` vs `EN=0`，黄区时间、掉高、过渡总时长有记录  
6. **TIMER 窗口**：进 TIMER 时 θ 已接近满倾允许；TIMER 内「θ 被门控夹住」时长应很短（目标 ≪ 衰减窗），无显著掉高  

用例建议挂名：`SIM-COR-OPT-01`（门控可见）、`02`（故意低空速不放行倾转）、`03`（正常过渡在绿区）、`04`（`AIRSPEED_MIN` 对齐前后 TIMER 残留对比）。

---

## 8 风险与非目标

| 风险 | 缓解 |
|------|------|
| 门控过严 → 过渡过长 / 超时失败 | margin 可调；与 `Q_TRANSITION_FAIL` 协调 |
| 表偏乐观 → 仍掉高 | margin；实飞标定前偏保守 |
| 限功率过狠 → 掉高 | `P_hold` 硬底；先开门控后开功率 |
| 与 TECS/G6 油门交接耦合 | 门控不解决交接台阶；见 `throttle_calculation.md` G6 |
| 参数名长度 / 上游冲突 | 项目分支先用；命名评审后再说合入 |

非目标：自研最优轨迹；替代 TECS；一阶段同时大改混控与 EKF。

---

## 9 建议工作顺序（可执行）

```text
1. A：后处理画 (θ,V) vs 走廊，钉死「倾转超前」证据
2. B 代码：已合入（CRD_* + assisted min）
3. SITL 主用例：CRD_EN=1 + AIRSPEED_MIN=50（≈CRD_V4）— 等空速在 WAIT
4. 对照坏组合：CRD_EN=1 + AIRSPEED_MIN=45 — 量化 TIMER 内 θ 冻结与掉高
5. EN=0 回归 → 写进 JT200_tilt_corridor 验收节
6. C：功率兜底（可选）
7. D：正式走廊数据到位后只优表
```

---

## 10 开放问题

| ID | 问题 | 倾向 |
|----|------|------|
| O1 | TIMER 是否保留时间衰减，还是「θ 到 90° 且 V 够」即 DONE？ | 先保留时间衰减，门控只限 θ |
| O2 | `Q_TILT_MAX` 在 ENABLE=1 时是否忽略？ | **V0.6：不忽略**；WAIT raw 仍封顶 MAX；门控只加跨门速度检查 |
| O3 | 空速用空速计还是地速？ | 有 AS 用 AS；无风 SITL 可对齐；实飞要校准 |
| O4 | 外桨卸载曲线是否挂到同一张表？ | 二期；先门控 θ，卸载仍按现逻辑 / SHUTDOWN 时机 |
| O5 | **TECS 前向油门 vs 门控：会不会「打架」？**（先记录，不急于定案） | 见 §10.1 |
| O6 | **TIMER 油门开环衰减 vs θ 门控等待：能否在 TIMER 里「等空速」？** | 见 §10.2；**倾向：多点等待放在 WAIT/门控侧，勿假设 TIMER 适合长等** |

### 10.1 O5 备忘：TECS `k_throttle` 与 θ–V 门控的耦合

**现象风险（待核实）：**

原生 WAIT 里 θ 跟的是 TECS/`k_throttle`（≥50% 就顶到 `Q_TILT_MAX`）。门控落地后变成：

```text
θ_cmd_raw = f(k_throttle)     ← TECS 能量/空速需求推出来的「想要的倾转」
θ_cmd     = min(θ_cmd_raw, θ_max(V))  ← 走廊只允许这么大
```

若 TECS 因空速不够持续要大油门，会一直给出「θ 应该很大」的结论，而门控把它压住——**行为上安全，但语义上两套逻辑在拉扯**：

| 侧 | 在说什么 |
|----|----------|
| TECS | 能量不够 → 加大前向需求 → 间接要更大 tilt |
| 走廊门控 | V 不够 → 禁止更大 tilt（先等空速） |

需要以后回答、避免策略打架的点：

1. **TECS 前向油门具体怎么算**（能量误差、速度误差、FF、限幅、与 `AIRSPEED_MIN`/`CRUISE` 关系）——调用链见 `throttle_calculation.md` / `AP_TECS`，此处不展开。  
2. 门控夹紧 θ 时，TECS 是否因「倾转前向分量上不来」而**积分飙升 / 油门需求更满**，形成空转。  
3. WAIT 里对无专用前推电机已有 `TECS.reset_throttle_I()`——门控长期夹 θ 时，I 项/需求油门是否仍合理。  
4. 是否应改为：**门控阶段不再用 `k_throttle` 映射 θ**，改为沿走廊中线 \(θ_{\mathrm{cmd}}=f(V)\) 或「仅速率爬升到 \(θ_{\max}(V)\)」，TECS 只管舵面/DONE 后推进，避免「TECS 要大 tilt、走廊说不要」的对打。  
5. TIMER/DONE 交接（G6）与门控同时存在时，油门台阶是否被放大。

**记录原则：** 阶段 B 实现门控时先 `min(θ_raw, θ_max)` 保安全；O5 在门控可飞之后再专项走读 TECS，再决定是否拆开「tilt 调度源」与「TECS 推进源」。不在本阶段改 TECS。

### 10.2 O6 备忘：TIMER 油门策略与「在 TIMER 里等空速」

**TIMER 里油门实际在干什么（半倾转 / DF-200）：**

```text
throttle_scaled = last_throttle × (1 − t/T)   // T = clamp(Q_TRANSITION_MS, 0.5s..30s)
hold_stabilize(throttle_scaled)               // 开环，不跑 Z 环
```

- 进 TIMER 的前提：原生认为 \(V>\texttt{AIRSPEED\_MIN}\)，**机翼已能分担升力**，所以开始**按时间撤垂向辅助**。
- JT-20/DF-200 有外四永久 VTOL → **不走**全倾转那条 `(1-r)·MC + r·TECS` 混合（`!has_vtol_motor()` 不成立）。
- TIMER 内前推主要靠 **θ 增大后的 \(T\sinθ\)**；TECS 经 `output_motor_mask` 接棒要等 **fully_fwd / 离开 assist**。
- 你们 parm 里 `Q_TRANSITION_MS=500000`，但代码 **硬夹到 ≤30 s**（`quadplane.cpp` TIMER 分支）——实际衰减窗最多约 30 s，不是 500 s。

**门控打开后的冲突：**

| 假设 | 实际 |
|------|------|
| 进 TIMER 后 θ 很快 →90°，几何前推接住衰减的 collective | `CRD_EN=1` 时 θ 可能卡在 \(θ_{\max}(V)\)，**空速从 45 爬到 50 期间 θ 几乎不动** |
| 集体油门仍按墙上时钟往 0.01 掉 | 掉的是垂向也是前向（θ 未继续增大时 \(T\sinθ\) 一起掉） |
| 外四在衰减 | 若 θ 还没到可关桨区，外四减推力 = **主动挖垂向缺口** |

因此：**在 TIMER 阶段用门控「再等一截空速」并不天然合理**——TIMER 的油门语义是「已经够快了，开始撤旋翼」，不是「再悬停加速一会儿」。

**策略倾向（尚未改代码）：**

1. **多点等空速应尽量 Occurs 在 WAIT / 等价闭环托高阶段**（`hold_hover`），而不是拖进 TIMER 长等。  
2. 门控与 `AIRSPEED_MIN` 对齐（**V0.4 已选定 A 为主**）：  
   - **A（默认）** 提高 `AIRSPEED_MIN` 贴近 `CRD_V4`（如 50）；  
   - **B（备选）** 进 TIMER 前要求 θ 已接近门控点；  
   - **C（后置）** TIMER 内门控仍夹 θ 时暂停衰减——仅当 A/B 不够时再改代码。  
3. `CRD_EN=1` + `AIRSPEED_MIN=45` + `CRD_V4=50` 仅作**对照飞次**，不作推荐构型。  
4. 与 O5 相关：TIMER 内 TECS 不进半倾转电机；θ 冻住时**不能指望 TECS 补前推**。

**记录原则：** 门控治「倾转超前」；TIMER 油门治「空速够了撤垂向」。**不要把 TIMER 当成第二个 AIRSPEED_WAIT**。  
**已升格进主策略：** §1.2 / §3.4（V0.4）。

### 10.3 门控方案的功率实测对比与推荐策略（V0.8）

**方法**：`docs/scripts/df200_power_mott_compare.py`（MOTT → 电机表；内桨 `velin≈AS·sin(tilt)` 近似，无 rotor.log；相对对比有效，绝对值有偏差）。产出：`docs/scripts/out/power_mott_193_203_206.csv`。

**三飞次对比（WAIT 段）**：

| 飞次 | 门控 | WAIT 时长 | 均功率 | 峰值 | 能耗 |
|------|------|-----------|--------|------|------|
| 193 | 无（原生单点 45°） | 21.4 s | 44.1 kW | 102.8 kW | **262 Wh** |
| 203 | EN=0 | 25.2 s | 41.4 kW | 100.8 kW | **289 Wh** |
| 206 | EN=1（10/10、20/25、30/35、45/45） | 48.3 s | 46.2 kW | 97.8 kW | **620 Wh** |

**206 按门 hold 成本拆解**：

| 门 | hold | 均功率 | 能耗 |
|----|------|--------|------|
| 10° | 19.2 s | 69 kW | **369 Wh** |
| 20° | 15.3 s | 40 kW | 168 Wh |
| 30° | 8.0 s | 20 kW | 43 Wh |
| 45° | 4.7 s | 17 kW | 21 Wh |

**核心结论**：

1. **低倾角 hold 是「伪安全、真悬停」**——外桨仍主导（Pout≈44 kW）、\(sinθ\) 小、加速效率低；省下的只是「看起来 θ 小」，代价是准悬停焦耳。
2. **193 式「直接冲到 45°」反而省电**：等待发生在 \(sinθ\) 大、机翼已参与的高效区（45° 平台均功率 35→17 kW 沿程下降）。
3. **门控对削峰贡献有限**（103→98 kW）；主要影响时长与能耗。
4. 门越密、Vn 越紧，WAIT 能耗越高；能耗近似按门「hold 时长 × 该角功率」累加。

**推荐策略（参数未改，待仿真确认）**：

| 方案 | 表 | 预期 WAIT | 预期能耗 |
|------|-----|-----------|----------|
| **混合门（推荐）** | 仅 **T1=30/V1=30**（+T2=45/V2=45 收尾；T3/T4 与 T2 重复） | ~25 s | ~280–300 Wh（≈193 量级） |
| 保守版 | T1=30/V1=**25** + 45/45 | 略短于 206 | 介于 193 与 206 |
| 206 现状 | 10/20/30/45 全卡 | 48 s | 620 Wh |

设计意图：**低角自由爬升**（`Q_TILT_RATE_UP` 兜底），**只在 30° 设一道防呆门**（V≥30 时翼升力 ≈0.37W，垂向缺口小），45° 后仍等 `AIRSPEED_MIN` 进 TIMER。

**顺带结论（曾提出，暂不采纳）**：把 `Q_TILT_RATE_UP` 提到 20–30°/s 可省 ~10–15 Wh，但见 §10.4——倾转速率不只服务速度，还服务**平滑性**，暂保持 10°/s。

---

### 10.4 O7 备忘：`Q_TILT_RATE_UP` 不建议为省电调快（稳定性考虑）

**观点**：倾转速率不能只看「转得快=省时间」，还要看**系统核心参数变化速度**：

- θ 是推力矢量的方向，直接进入**混控、tilt_compensate（\(1/\cosθ\)）、姿态动力学**；
- θ 变化快 → 每拍推力分配、垂向/前向分量、机翼升力占比都在快速漂移 → 控制环跟踪的「被控对象」本身在动；
- 10°/s 下 0→45° 用 4.5 s，各环有时间跟上；20–30°/s 时 1.5–2.5 s 内改完，混控瞬态、Z 环响应、FW 舵面接管都更容易出现台阶。

**权衡记录**：

| 方面 | 10°/s（现状） | 20–30°/s |
|------|---------------|----------|
| WAIT 能耗 | 多 ~10–15 Wh | 省 |
| 参数漂移速率 | 慢、各环好跟 | 快、易出瞬态 |
| 出问题时定位 | 平台明显、好查 | 变化快、难归因 |

**结论**：速率保持 10°/s；省电优先靠**门控形状**（§10.3 混合门），不动 RATE。

---

## 11 修订记录

| 版本 | 日期 | 说明 |
|------|------|------|
| V0.1 | 2026-09-10 | 初稿：θ–V 门控为主、参数化、功率兜底为辅；对齐 JT200 走廊与油门饱和链 |
| V0.2 | 2026-09-10 | 阶段 B 代码落地：`Q_TILT_CRD_*`、assisted `min(θ_raw,θ_max(V))`；参数名以实装为准 |
| V0.3 | 2026-09-10 | 增 O6：TIMER 开环油门 vs 门控等待；结论倾向勿在 TIMER 长等空速 |
| V0.4 | 2026-09-10 | 策略收紧：§3.4 阶段分工；`AIRSPEED_MIN≈CRD_V4`；WAIT 等空速 / TIMER 撤垂向 |
| V0.5 | 2026-09-10 | （中间探索）曾试 CRD 时取消 MAX 封顶 / 连续插值；已被 V0.6 取代 |
| V0.6 | 2026-09-10 | WAIT 断点门控 `corridor_gate_tilt`；raw 仍≤MAX；TIMER 跳过；表 10/10…45/45；修「按最高门要速」bug |
| V0.7 | 2026-09-10 | 修 205：先爬到 Tn 再查 Vn（旧 `current<Tn≤raw` 在 ~1° 死锁）；hold 在门角 Tn |
| V0.8 | 2026-09-10 | §10.3 功率对比（193/203/206，`df200_power_mott_compare.py`）：低角 hold 费电；推荐混合门（仅 30°/30 + 45°/45），参数未改；§10.4 增 O7：RATE 保持 10°/s 取平滑性 |
| V0.9 | 2026-09-17 | V0.7 断点门控经 206/207/208 验证；气动切换为全严格 xlsx 包（Cnβ 0.25→0.015 等，见 [`vtol_df200.md`](vtol_df200.md) §4）；208 = 门控 + 严格气动联合基线，三历史风险未复现 |
