# DF-200 控制问题专题：幅频响应与欠阻尼分析

> 代码/模型：Plane 4.6.x / `qhz/plane_4.6.3`，`Tools/autotest/models/df200.json`
> 数据源：`logs/*.BIN`（ATT/PIDR/PIDP/NTUN/CTUN/ARSP/BARO/RATE）+ `/tmp/df200_*.log`
> 关联文档：`docs/df200_session_20260902.md`、`docs/vtol_df200.md` §11、`docs/throttle_calculation.md`

本文档把 DF-200 SITL 历次出现的**控制环幅频响应问题**（响应慢、欠阻尼、正反馈发散）归集到一处，并以 194 转弯炸机为详细案例，给出可复现的判读方法。

---

## 0. 阅读指引

| 顺序 | 章节 | 作用 |
|---|---|---|
| 1 | §1（194 案例） | 完整分析一次欠阻尼/响应慢的判读过程，含逐拍数据 |
| 2 | §2 | 控制环结构与限幅点（SCALING_SPEED/RMAX/NAVL1 三层） |
| 3 | §3 | 历次控制问题索引（AS27 墙/phugoid/position%gyro/转弯慢） |
| 4 | §4 | 幅频响应判读方法（判别欠阻尼 vs 响应慢 vs 正反馈） |
| 5 | §5 | 待验证的修改清单与单参数试验法 |

---

## 1. 194 转弯炸机：响应慢 + 速率限幅的详细分析

### 1.1 现象

Mission 点 2（直飞，TBrg=−5°，向西北）→ Mission 点 3（TBrg=+79°，向东北），**航向跳变 ~84°**。t=191.93 切换 WP3 后，飞机在 t=197.06 坠地，全程 5s。

里程碑（来自 `df200_transition_log.py`）：

```
191.93  Mission: 3 WP            ← 航向目标跳变
192.09  Ail 打满 566（量程 ~38%）
192.53  Roll 实际 5.7° vs DesRoll 48°   ← 差 42°
193.23  Roll 31.4° vs DesRoll 47.8°
193.73  Roll 49.3° 终于追上 DesRoll 48°  ← 晚 1.5s
194.7+  Roll 高位 55-63°，NavBrg 追不上 TBrg
197.06  SIM Hit ground
```

### 1.2 判读步骤（可复现）

**第一步：确认稳态段是否也有问题（排除稳态幅频）**

统计巡航稳态段 t=110–191（FW done 后、转弯前）：

| 量 | mean | std | range |
|---|---|---|---|
| Roll | 0.12° | 0.08 | 0.26 |
| DesRoll | 0.13° | 0.10 | 0.35 |
| AS | 55.01 | 0.09 | 0.57 |
| BARO.Alt | 19.90 | 0.61 | 3.99 |

**结论：稳态段极稳，无 phugoid（190 修 `TECS_TIME_CONST=5` 的成果保住了）。问题只在转弯瞬态。**

**第二步：提取转弯段逐拍数据（ATT + PIDR）**

关键数据列（`PIDR` 消息，10 Hz，`Act`=真实角速率 rad/s，`Tar`=期望角速率 rad/s，`Err`=角速率误差）：

```
用 TimeUS 字段重建时间轴（本日志 _timestamp 失效，必须用 TimeUS/1e6）

t=192.03  Roll=0.3  dRoll/dt= 0.2  Act= 0.21  Tar= 0.18  Err=-0.00
t=192.13  Roll=0.4  dRoll/dt= 0.6  Act= 1.18  Tar=40.00  Err= 0.38  ← Tar 钳到 40
t=192.23  Roll=0.7  dRoll/dt= 3.7  Act= 5.73  Tar=40.00  Err= 0.39
t=192.33  Roll=1.7  dRoll/dt= 9.7  Act=12.42  Tar=40.00  Err= 0.32
t=192.43  Roll=3.4  dRoll/dt=16.8  Act=19.50  Tar=40.00  Err= 0.24
t=192.53  Roll=5.7  dRoll/dt=23.5  Act=25.84  Tar=40.00  Err= 0.17
t=192.63  Roll=8.6  dRoll/dt=29.1  Act=30.93  Tar=40.00  Err= 0.11
t=192.73  Roll=12.0 dRoll/dt=33.3  Act=34.63  Tar=40.00  Err= 0.06
t=192.83  Roll=15.6 dRoll/dt=36.2  Act=37.12  Tar=40.00  Err= 0.03
t=192.93  Roll=19.4 dRoll/dt=38.1  Act=38.72  Tar=40.00  Err= 0.01
t=193.03  Roll=23.3 dRoll/dt=39.0  Act=39.72  Tar=40.00  Err= 0.00
t=193.13  Roll=27.3 dRoll/dt=40.1  Act=40.38  Tar=40.00  Err=-0.00  ← Act 到 40，顶满
t=193.23  Roll=31.4 dRoll/dt=40.6  Act=40.82  Tar=32.90  Err=-0.05  ← Tar 开始降
t=193.33  Roll=35.5 dRoll/dt=41.0  Act=40.62  Tar=24.86
t=193.43  Roll=39.5 dRoll/dt=39.7  Act=39.13  Tar=17.28
t=193.53  Roll=43.2 dRoll/dt=37.2  Act=36.09  Tar=10.40
t=193.63  Roll=46.5 dRoll/dt=33.2  Act=31.62  Tar= 4.46
t=193.73  Roll=49.3 dRoll/dt=28.0  Act=26.15  Tar=-0.30  ← 追上 DesRoll=48，开始减速
t=193.83  Roll=51.5 dRoll/dt=22.1  Act=20.20  Tar=-3.74  ← 过冲！Roll>DesRoll
t=193.93  Roll=53.1 dRoll/dt=16.0  Act=14.26  Tar=-5.88
t=194.03  Roll=54.1 dRoll/dt=10.2  Act= 8.81  Tar=-6.78
t=194.13  Roll=54.6 dRoll/dt= 5.1  Act= 4.18  Tar=-6.60
t=194.23  Roll=54.7 dRoll/dt= 1.0  Act= 0.56  Tar=-5.56  ← 峰值 54.7°
t=194.33  Roll=54.5 dRoll/dt=-2.0  Act=-1.96  Tar=-3.90
```

**第三步：从数据读出三个事实**

1. **期望速率 `Tar` 被钉死在 40.00 rad/s 达 1.1s**（192.13→193.13）——但 40 rad/s = 2292 deg/s，物理上不可能。这说明 **PIDR.Tar 记录的不是 deg/s，是被 `RLL2SRV_RMAX` 限幅后的内部归一化值**。真正的角速度看 `Act`：从 0.21 爬到 40.38 rad/s... 但 Act 的单位也要核对——`dRoll/dt`（ATT 差分，deg/s）和 `Act` 数值量级一致（40.6 vs 40.38），**所以 `Act` 的单位是 deg/s，不是 rad/s**。即真实滚转速率峰值 ~41 deg/s。

2. **真实角速度 `Act` 顶在 ~40 deg/s 平台**（192.93→193.23，Act=38.7→40.8）——正好等于 `RLL2SRV_RMAX=40 deg/s`。**这是速率限幅的直接铁证。**

3. **过冲**：DesRoll=48°，Roll 峰值 54.7°（过冲 6.7°，~14%），且到达峰值后 Roll 衰减很慢（54.7→54.5 用了 0.2s 才开始降）。**这是典型的二阶欠阻尼阶跃响应**，阻尼比 ζ 估算见 §1.3。

### 1.3 阻尼比估算（二阶系统拟合）

把 Roll 阶跃响应（0→54.7°）当作二阶系统。上升时间 t_r（10%→90%）：

- 10% = 5.5° @ t≈192.52，90% = 49.2° @ t≈193.69 → **t_r ≈ 1.17s**
- 峰值时间 t_p = 194.23 − 192.13 = **2.10s**
- 过冲 M_p = (54.7−48)/48 = **14%**

二阶系统公式：
- 过冲 → 阻尼比：`ζ = -ln(M_p) / sqrt(π² + ln²(M_p))` = -ln(0.14)/sqrt(9.87+0.28) = **ζ ≈ 0.55**
- 峰值时间 → 自然频率：`ω_n = π/(t_p·sqrt(1-ζ²))` = π/(2.10·0.835) = **ω_n ≈ 1.79 rad/s**（f_n ≈ 0.28 Hz）

**判读**：ζ=0.55 属于"可用但偏欠阻尼"（理想 0.7），真正致命的不是阻尼比，而是 **ω_n 太低 + 速率限幅双重拖慢**：自然周期 T_n = 3.5s，而转弯只给了 ~2s 就坠地——**响应根本来不及收敛**。叠加 RMAX=40 把爬升段斜率限死，等效把 ω_n 进一步压低。

### 1.4 根因（控制环三层结构）

DF-200 固定翼 roll 控制环从外到内三层，每层都有一个限幅/缩放点：

```
┌─ L1 导航 (NAVL1) ─────────────────────────────────┐
│  输入: WP航向 / 当前位置                            │
│  输出: DesRoll (期望倾角)                           │
│  参数: NAVL1_PERIOD=14, NAVL1_DAMPING=0.75        │
│        NAVL1_LIM_BANK=0 (不限倾角)                │
└──────────────────────────────────────────────────┘
              ↓ DesRoll
┌─ 角度→速率 (AP_RollController::get_servo_out) ────┐
│  期望速率 = (DesRoll - Roll) / TCONST              │
│  限幅: RLL2SRV_RMAX (deg/s)  ← AP_RollController:286-290 │
│  参数: RLL2SRV_TCONST=0.5, RLL2SRV_RMAX=40        │
└──────────────────────────────────────────────────┘
              ↓ desired_rate (限幅后)
┌─ 速率 PID (AP_RollController::_get_rate_out) ─────┐
│  P/I 项 × scaler²  ← AP_RollController:183       │
│  scaler = SCALING_SPEED / aspeed  ← :239          │
│  参数: SCALING_SPEED=45, RLL2SRV_P/I/D/FF         │
│  输出: 副翼 centi-deg → RCOU C1                    │
└──────────────────────────────────────────────────┘
```

194 转弯段三层状态：

| 层 | 参数 | 现值 | 194 状态 | 问题 |
|---|---|---|---|---|
| L1 导航 | NAVL1_PERIOD | 14 | DesRoll=48° 直接给 | 大飞机 + 55 m/s 巡航，period 偏小，转弯引导不够提前 |
| 角度→速率 | **RLL2SRV_RMAX** | **40** | **Tar 钉死 40，Act 顶 40** | **主瓶颈：stock 小飞机默认，DF-200 应放大** |
| 速率 PID | SCALING_SPEED | 45 | scaler=45/55=0.82，增益×0.67 | 187 修过，已 OK，副翼只打 38% 就够力 |

**核心结论**：SCALING_SPEED（187 修过）解决了"增益被压扁"，但 **RLL2SRV_RMAX=40 这组速率限幅没同步调**——等于"油门踩到底但限速器卡在 40 deg/s"。再叠加 NAVL1_PERIOD=14 的软引导，84° 航向跳变下飞机 1.5s 才转到一半，转弯半径远大于航点间距，转不过来撞地。

### 1.5 气动数据要不要回头看

194 转弯段气动本身健康：
- 副翼 `c_l_deltaa=0.15`（157 修过），t=192.1 只输出 38% 量程就把 Roll 推起来——**舵效够**
- 转弯段 AS 55→72（转弯掉高加速），无失速迹象
- Roll 一旦跟上就稳在 49–63°，`c_l_b`（上反角 −0.12，162 后恢复）配平正常

**但有一个隐患**：转弯后期 Roll=55–63° 大倾角时 NavBrg 持续追不上 TBrg（93→124° vs TBrg 锁 80°），可能不全是速率限幅——大倾角是荷兰滚耦合（§3.1）最易激发的工况，`c_n_r=−0.065`（Excel 弱值）偏航阻尼不足会拖累 roll-yaw 协调。**建议改完 RMAX/NAVL1 后看 NavBrg 是否跟上，若仍滞后再回看 `c_n_r` 试 stock −1.0**（§3.4、`vtol_df200.md` §11.11）。

### 1.6 修改建议（按优先级，一次一类）

| # | 参数 | 现值 | 建议 | 理由 |
|---|---|---|---|---|
| 1 | **RLL2SRV_RMAX** | 40 | 120–180 | 直接解开速率限幅瓶颈；大飞机惯量大但 SCALING_SPEED 已修，可放开 |
| 2 | **PTCH2SRV_RMAX_UP/DN** | 40/40 | 60–90 | 俯仰速率上限同步（186 修 SCALING_SPEED 后这组也该调） |
| 3 | NAVL1_PERIOD | 14 | 18–22 | 大飞机 55 m/s 巡航，period 应更长，引导更平滑提前 |
| 4 | WP_RADIUS | 90 | 150–200 | 翼展 7.5m + 55 m/s，转弯半径至少 150m；或用 spline WP |
| 5 | NAVL1_LIM_BANK | 0 | 可选 55 | 限制转弯最大倾角防过冲（若 #1 落地可不动） |
| 6 | WP_YAW_BEHAVIOR | 缺失 | 设 0 或 3 | 确保转弯不触发额外 yaw 对齐 |

**最关键 #1 + #3**。先单参数验证 `RLL2SRV_RMAX 40→150`，看 Roll 追 DesRoll 延迟是否从 1.5s 降到 0.3s 内。

---

## 2. 控制环结构与限幅点（代码事实，已验证行号）

### 2.1 roll 速率环三限幅点

| 限幅点 | 代码位置 | 作用 | 194 现值 |
|---|---|---|---|
| RMAX 限幅 | `libraries/APM_Control/AP_RollController.cpp:286-290` | 限制期望角速率 deg/s | 40 |
| scaler² 缩放 | `AP_RollController.cpp:183` | P/I 项乘 (SCALING_SPEED/aspeed)² | 45→scaler=0.82 |
| FF 缩放 | `AP_RollController.cpp:193` | FF 项 ÷ (scaler·eas2tas) | — |

scaler 定义见 `AP_RollController.cpp:239`：`scaler = scaling_speed / aspeed`。

### 2.2 三轴对应参数

| 轴 | TCONST | RMAX | SCALING_SPEED 影响 |
|---|---|---|---|
| Roll | RLL2SRV_TCONST=0.5 | RLL2SRV_RMAX=40 | 是 |
| Pitch | PTCH2SRV_TCONST=0.5 | PTCH2SRV_RMAX_UP/DN=40/40 | 是 |
| Yaw | YAW2SRV_DAMP=2 | (无 RMAX，有 IMAX=1500) | 否（yaw 不受 SCALING_SPEED 缩放） |

### 2.3 关键诊断字段（PIDR/PIDP/PIDY）

| 字段 | 含义 | 单位 | 判读 |
|---|---|---|---|
| `Tar` | 期望角速率 | deg/s（被 RMAX 限幅后） | 钉死某圆数 = 限幅触发 |
| `Act` | 实际角速率 | deg/s | 顶在 RMAX = 限幅生效 |
| `Err` | 角速率误差 | deg/s | 持续大 = 跟不上 |
| `P/I/D/FF` | PID 各项 | rad 系（输出×100=centi-deg 舵量） | I 爬升慢 = 积分不足 |

> **坑**：本日志 `_timestamp` 失效，必须用 `TimeUS` 字段重建时间轴（`t = d['TimeUS']/1e6`）。ATT/ARSP/PIDR 在 194 只有 ~10 Hz（1672 条），RATE 是 20 Hz 但记录的是 attitude controller 输出语义，roll rate 实际看 PIDR.Act。

---

## 3. 历次控制问题索引（幅频响应类）

> 按问题类型归集，每条给出现象、根因、修复、所在文档章节。

### 3.1 横航向自激墙（AS≈27，114–149）—— 正反馈发散

| 项 | 内容 |
|---|---|
| 现象 | 每趟过渡 AS≈27.6 炸，onset 顺序固定：先 yaw → 再 roll → 最后 pitch |
| 类型 | **正反馈发散**（不是欠阻尼）：荷兰滚能量环供能>耗能，振幅指数增长 |
| 根因 | 旋翼差动被 Lua fade 收掉、舵面=0 → Clβ/Cnβ/Clr/Cnp 耦合环无阻尼 |
| 验证 | 150（四项全 0）→ roll 发散消失，证明耦合环是主因 |
| 修复 | 恢复 Cnβ=0.25 + Excel Cnr=−0.065 + 方向舵三件套（151） |
| 文档 | `vtol_df200.md` §11.2–11.5 |
| 判别特征 | 发散不取决于种子大小（149 削到几 Nm 仍炸）→ 正反馈，非欠阻尼 |

### 3.2 TIMER 段 roll 突跳（154–161）—— SITL 桨模型数值正反馈

| 项 | 内容 |
|---|---|
| 现象 | tilt≈38°、AS≈49 时 roll 速率一步跳到 ±34.9 rad/s（饱和），0.2–3s 坠地 |
| 类型 | **正反馈**：`position%gyro` → velin 分裂 → `calc_thrust` 平方差放大 |
| 根因 | `calc_thrust`（`SIM_Motor.cpp:321`）推力差增益 ∝ (v1+v3)，高速 velin≈50 时增益≈81 > 1 |
| 验证 | 162 把 gyro 置 0 → mvelz 不分裂 → 过墙（根因证实） |
| 修复 | 诊断手段（gyro 置 0 物理不真实），真修方向见 `vtol_df200.md` §11.10 |
| 文档 | `vtol_df200.md` §11.10 |
| 判别特征 | "之前机体旋转也不大却没炸，到 tilt 38°+高速才炸" = 放大器在高速才打开 |

### 3.3 phugoid 长周期振荡（189）—— 高度环欠阻尼

| 项 | 内容 |
|---|---|
| 现象 | FW 巡航段 h ±3.5m、sp 54↔59、th 0.23↔59、周期 ~6-7s，振幅稳定不发散 |
| 类型 | **欠阻尼极限环**：TECS 高度环追太紧，对 phugoid 固有周期正反馈拉扯 |
| 根因 | TECS_TIME_CONST=3 + TECS_HGT_OMEGA=3，对 ~7s 模态欠阻尼 |
| 修复 | TECS_TIME_CONST 3→5（190），高度摆幅 ±3.5m→±0.3m 收敛 |
| 文档 | `df200_session_20260902.md` §3.8–3.9 |
| 判别特征 | 振荡稳定不发散 = 欠阻尼（ζ<1 但>0），非正反馈 |

### 3.4 偏航阻尼不足（145–149）—— yaw 权威 vs 惯量

| 项 | 内容 |
|---|---|
| 现象 | 垂起 yaw align 失败/临界，过渡 yaw 方波 ±406°/s（150 后） |
| 类型 | **阻尼不足**：Cnβ/Cnr 太弱 + motor_yaw 被削 |
| 根因 | Excel Cnr=−0.065 比 stock −1.0 弱 15 倍；motor_yaw=±0.5 在大 Izz 下不够 |
| 修复 | 降 Izz 1052→500（垂起可控性 ∝ 偏航权威/Izz）；Cnβ=0.25 恢复风向标 |
| 文档 | `vtol_df200.md` §11.6、§11.8 |
| 判别特征 | yaw 方波翻转但 roll 不发散 = 纯偏航通道阻尼问题 |

### 3.5 增益被 SCALING_SPEED 压扁（185）—— 伪"舵失效"

| 项 | 内容 |
|---|---|
| 现象 | FW 段升降舵只打 5%，俯仰卡死不动，2.2s 坠地 |
| 类型 | **增益缩放过低**（非欠阻尼、非正反馈）：舵面能力够但链路增益被压到 4.3% |
| 根因 | SCALING_SPEED=12 vs AS=58，scaler=0.207，P/I×scaler²=0.043 |
| 修复 | SCALING_SPEED 12→45（187），三轴 FW 增益一次放开 |
| 文档 | `df200_session_20260902.md` §3.5.1 |
| 判别特征 | 满舵能力是低头矩 20 倍但只输出 5% = 增益链问题，非气动问题 |

### 3.6 转弯响应慢 + 速率限幅（194）—— 本文档 §1

| 项 | 内容 |
|---|---|
| 现象 | WP2→WP3 84° 航向跳变，Roll 1.5s 才追上 DesRoll，过冲 14%，NavBrg 追不上撞地 |
| 类型 | **响应慢 + 欠阻尼**：ω_n=1.79 rad/s（周期 3.5s）+ RMAX=40 限幅双重拖慢 |
| 根因 | RLL2SRV_RMAX=40（stock 小飞机默认）+ NAVL1_PERIOD=14 引导软 |
| 修复 | RMAX 40→150（#1）+ NAVL1_PERIOD 14→20（#3），待验证 |
| 文档 | 本文 §1 |
| 判别特征 | Act 顶在 RMAX 平台 + Tar 钉死 = 速率限幅；过冲 14% = 欠阻尼 |

### 3.7 升降舵隔离遗留（151–152）—— 舵面通道缺失

| 项 | 内容 |
|---|---|
| 现象 | 交接掉高 + 升降舵=0 无备份，低头坠地 |
| 类型 | **控制通道缺失**（非幅频问题，但归入控制类） |
| 根因 | 隔离试验把 Cmδe 关 0 没恢复 |
| 修复 | 153 恢复 Cmδe=0.42 |
| 文档 | `vtol_df200.md` §11.8 |

---

## 4. 幅频响应判读方法（判别四类问题）

面对一个"控制不对"的现象，按此顺序判别：

### 4.1 先分稳态 vs 瞬态

```
统计稳态段（无输入跳变）的 std/range：
  - std 大 + 周期性振荡  → 稳态幅频问题（phugoid/极限环）→ §3.3 类
  - std 小（如 194 巡航 ±0.13°）         → 问题在瞬态 → 继续
```

### 4.2 瞬态看 Act 是否顶限幅

```
提取 PIDR.Act（真实角速率）vs PIDR.Tar（期望，限幅后）：
  - Act 顶在某圆数平台 = 速率限幅触发        → §3.6 类（194）
  - Act 远低于 Tar 且爬升慢 = 增益不足        → §3.5 类（SCALING_SPEED）
  - Act 超调后振荡收敛 = 欠阻尼               → §3.3 类
  - Act 指数增长不收敛 = 正反馈               → §3.1/§3.2 类
```

### 4.3 算阻尼比（二阶拟合）

阶跃响应有过冲时：
```
过冲 M_p = (峰值-稳态)/稳态
ζ = -ln(M_p) / sqrt(π² + ln²(M_p))
ω_n = π / (t_p · sqrt(1-ζ²))     # t_p=峰值时间

ζ<0.3   严重欠阻尼，必改
ζ 0.3-0.7 可用，看 ω_n 是否够快
ζ>0.7   阻尼 OK，问题在别处
ω_n 对应周期 T_n=2π/ω_n，若 T_n > 可用响应时间 → 响应太慢
```

194：ζ=0.55（阻尼尚可），但 T_n=3.5s >> 转弯给的 2s → **响应太慢是主因，叠加 RMAX 限幅**。

### 4.4 正反馈 vs 欠阻尼的判别

| 特征 | 欠阻尼 | 正反馈 |
|---|---|---|
| 振幅演化 | 稳定极限环 / 衰减振荡 | 指数增长 |
| 种子大小影响 | 影响起振时刻 | **不影响是否发散**（149 证） |
| 置 0 耦合项 | 仍有其他通道 | 发散消失（150 证） |
| 典型 | phugoid（§3.3） | 荷兰滚墙（§3.1）、position%gyro（§3.2） |

---

## 5. 待验证修改清单与单参数试验法

### 5.1 转弯问题（194，第一优先）

| 试验 | 改动 | 验证指标 |
|---|---|---|
| 195 | RLL2SRV_RMAX 40→150 | Roll 追 DesRoll 延迟 <0.3s；Act 不再顶 40 平台 |
| 196 | +NAVL1_PERIOD 14→20 | NavBrg 跟上 TBrg；无过冲坠地 |
| 197 | +WP_RADIUS 90→180 | 转弯半径匹配翼展/速度 |

### 5.2 历史遗留（未闭环）

| # | 事项 | 来源 | 优先级 |
|---|---|---|---|
| A | 外桨 FW 段不 SHUTDOWN（交接断崖根修） | `df200_session_20260902.md` §4 #4 | 高 |
| B | gyro 置零开关 Run2（验证表模型能否压住 position%gyro） | `df200_session_20260902.md` §4 #6 | 中 |
| C | c_n_r Excel −0.065 试 stock −1.0（偏航阻尼） | `vtol_df200.md` §11.11 | 中（194 转弯后期 NavBrg 滞后时再动） |
| D | 横航向互耦 c_l_b/c_l_r/c_n_p 已恢复（162 后） | `vtol_df200.md` §11.11 | 已完成 |

### 5.3 单参数试验铁律（来自 `df200_session_20260902.md` §6）

1. **每飞只动一个参数**（185→190 因果链干净的前提）
2. **改参后先 `mavlogdump.py --type PARM` 核对生效**（188 假失败教训）
3. **做力账用 `/tmp/df200_rotor.log` + `df200_aero.log` 逐拍对齐**，别只看 RCOU/TECS 单侧
4. **改惯量/电机模型必须同步增益与气动**（181 教训）

---

## 6. 相关文档与脚本

| 文档/脚本 | 内容 |
|---|---|
| `docs/df200_session_20260902.md` | 9.02 会话完整记录，§3.5.1 增益链、§3.8-3.9 phugoid |
| `docs/vtol_df200.md` §11 | AS27 墙 + position%gyro 正反馈机理 |
| `docs/throttle_calculation.md` | TECS/油门状态机、掉高与 TECS 接棒 |
| `docs/scripts/df200_transition_log.py` | 里程碑/onset 诊断 |
| `docs/scripts/df200_force_budget.py` | 力账+力矩+混控器一体化 |
| `docs/jt_power_anaylisis.md` | MOTT 功率分析方法（可复用） |

---

## 附录：194 关键参数快查（PARM 实测）

| 参数 | 值 | 备注 |
|---|---|---|
| SCALING_SPEED | 45 | 187 修复，OK |
| RLL2SRV_TCONST | 0.5 | — |
| RLL2SRV_RMAX | 40 | **stock 默认，主瓶颈** |
| PTCH2SRV_TCONST | 0.5 | — |
| PTCH2SRV_RMAX_UP/DN | 40/40 | **同 RMAX，建议同步调** |
| NAVL1_PERIOD | 14 | 大飞机偏小 |
| NAVL1_DAMPING | 0.75 | — |
| NAVL1_LIM_BANK | 0 | 不限倾角 |
| NAVL1_XTRACK_I | 0.02 | — |
| WP_RADIUS | 90 | 大飞机偏小 |
| ROLL_LIMIT_DEG | 65 | 倾角上限 |
| TECS_TIME_CONST | 5 | 190 修复，OK |
| TECS_HGT_OMEGA | 3 | — |
| TECS_SPDWEIGHT | 0.5 | — |
| AIRSPEED_CRUISE | 55 | 189 修复，OK |
| c_l_deltaa | 0.15 | 157 修复，OK |
| c_m_deltae | 0.42 | 153 修复，OK |
| c_l_b (Clβ) | -0.12 | 162 后恢复，OK |
| c_n_r (Cnr) | -0.065 | Excel 弱值，待观察 |
