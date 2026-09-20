# DF-200 功率分析会话总结（2026-09-03）

> 会话目标：基于 `电机电池.docx` 的功率数据，统计 DF-200 控制律在全飞行过程（垂起→过渡→巡航）的电机总功率需求，形成功率走廊。
> 关联文档：`docs/df200_power_corridor.md`（报告）、`docs/df200_motor_data.md`（电机数据手册）、`docs/scripts/df200_power_analysis.py`（脚本）
> 数据日志：`logs/00000193.BIN` + `/tmp/df200_rotor.log`

---

## 1. 产出

| 文件 | 内容 |
|------|------|
| `docs/df200_motor_data.md` | 把 `电机电池.docx` 两张表（60in 外桨静态 15 点 / 40in 内桨风洞 63 点）整理成纯文本 md，含单位换算、落地核对、功率走廊速查。以后不用再解析 docx |
| `docs/scripts/df200_power_analysis.py` | 独立分析脚本：移植电机表+插值、解析 rotor.log、按 velin 反查内外桨功率、对齐 BIN 分阶段、tilt 切片、matplotlib 画图、CSV 导出 |
| `docs/df200_power_corridor.md` | 功率走廊报告：分阶段统计表 + tilt 切片 + 4 张 PNG 图 |
| `docs/df200_power_imgs/` | 4 张图：总功率、内桨各电机、外桨各电机、功率 vs tilt |

---

## 2. 最终功率走廊（193 仿真实测，数据正确）

| 阶段 | 时长s | 均值kW | 峰值kW | p95kW | 内桨均 | 外桨均 | 能耗Wh |
|------|-------|--------|--------|-------|--------|--------|--------|
| vtol_climb（垂起爬升） | 30.8 | 92.5 | 150.1 | 107.4 | 40.7 | **51.8** | 791.8 |
| transition（过渡） | 25.8 | 75.8 | 112.7 | 104.9 | 35.5 | 40.3 | 544.3 |
| fw_cruise（FW巡航） | 24.0 | 7.6 | 79.6 | 15.4 | 7.6 | **0** | 50.4 |

transition 按 tilt 切片（功率随倾转角，非时间）：

| tilt | 时长s | 均值kW | 内桨均 | 外桨均 |
|------|-------|--------|--------|--------|
| 0-30° | 2.9 | 88.0 | 38.5 | 49.5 |
| 30-45° | 1.4 | 102.8 | 43.2 | 59.6 |
| 45-60° | 18.3 | 76.2 | 36.3 | 39.9 |
| 60-90° | 3.0 | 47.8 | 24.0 | 23.8 |

**核心物理结论**（正确版）：
- 外桨（60in 固定垂起）在 **tilt=0（垂起）满推 ~52kW**，随 tilt 增大单调衰减，到 **tilt=90（前飞）=0**
- 内桨（40in 倾转）全程参与，过渡段叠加外桨功率最高
- 峰值 150kW 在起飞瞬间（t=57.8s，8 电机 cmd≈0.9 集体油门爬升）

---

## 3. 为什么误解 tilt——中间出现的问题

### 3.1 根本原因：rotor.log 与 BIN 时间轴错配

**最大的坑**：`/tmp/df200_rotor.log` 是 SITL 用 `fopen("a")` **追加写**的（`SIM_Frame.cpp:680`），多次飞行数据叠加在一起。191 和 192 两趟的 rotor 数据叠在同一个文件里，中间有一次时间重置（t 从 136.9 跳回 0.05）。

我的脚本第一版 `load_rotor` 用"过滤所有非递增时间戳"逻辑处理这个文件——结果**错误地把第二趟（192）开头 t=0 的数据全当成"时间倒退"丢掉了**，只保留第一趟（191）。然后用 191 的 rotor 数据配 192 的 BIN 边界分析，**全程错位**。

### 3.2 错误的连锁反应

错配导致了一系列错误结论，且互相"自洽"地误导：

1. **tilt=0 时外桨=0**（错误）：rotor.log（实为 191）在 transition 段 t=131.5-137 显示外桨 cmd=0，我错误外推成"tilt 0-45° 外桨=0"，又错误套到"垂起段 tilt=0 外桨该满推但=0"，得出矛盾。
2. **怀疑反查链有 bug**：sanity check 一开始用 `Tzi/Txi`（体轴投影分量）对比，过渡段差 1.5 倍（1/cos tilt），误以为反查错。实际 `thrmag` 才是推力大小，修正后误差<0.4%。
3. **"峰值 150kW 是异常值"** 的怀疑：实际是起飞瞬间集体油门 0.9 的真实爬升功率，sanity check 证实反查正确。

### 3.3 用户的关键纠偏

用户多次指出问题，每次都准确：

- **"transition 是怎么算的，要按 tilt 切换角度算"** → 触发 tilt 切片改进
- **"有些电机油门大有些小，不能都这么大"** → 引出 cmd 分布分析
- **"外桨为0时候是tilt 90度，之前一直有值，你是不是理解错了"** → **最关键的一句**，直接点出我的 tilt 理解反了
- **"tilt 为0 的时候作为垂起桨肯定有输出，你看 bin 文件里 mott 直接和 tilt 对应"** → 指出应该用 BIN MOTT 而非 rotor.log
- **"如果 log 和 bin 数据不一致我就重新跑一次"** → 触发重新跑 193，拿到干净数据

### 3.4 最终定位

通过 BIN MOTT vs TILT 直接对比发现：
- BIN MOTT（飞控 actuator 输出）在垂起爬升段（t=90-130，tilt=0）外桨 Mot5-8 一直 0.47-0.68 **有值**
- rotor.log（SITL Motor command）在同段显示外桨=0
- **两套数据矛盾 → 证明 rotor.log 不是这趟飞行的数据 → 追加写叠加问题**

修复 `load_rotor`：取最后一趟飞行（最后一次时间重置后的数据）后，193 的 rotor.log 与 BIN 完全对齐，所有数据自洽。

---

## 4. 下一次做得更好的地方

### 4.1 数据源管理（最重要）

- **每次 SITL 跑前删 `/tmp/df200_*.log`**：避免追加写叠加多趟。脚本里加了这个提示，但应该养成习惯：
  ```bash
  rm -f /tmp/df200_rotor.log /tmp/df200_aero.log /tmp/df200_mix.log
  ```
- **rotor.log 应带飞行 ID 或时间戳**：SITL 侧 `SIM_Frame.cpp` 的日志写入可以加个起飞时间标记，便于多趟分割。这是代码改进点。
- **脚本默认取最后一趟**：已修复，但仍建议跑前清空，避免误判。

### 4.2 先验证数据一致性再分析

- **教训**：得到矛盾数据（tilt=0 外桨=0 vs 物理应有值）时，**第一时间怀疑数据源错配**，而不是怀疑物理或反查逻辑。
- **正确做法**：分析前先跑一个一致性检查——rotor.log 的 AS 时间序列 vs BIN 的 ARSP，对不上就停，先查数据源。这个应该加进脚本的前置校验。

### 4.3 用对数据源

- **功率反查用 rotor.log 的 cmd/rpm**（SITL Motor 层，已含 EXPO 后的 actuator）是对的
- **阶段边界、tilt、高度、MOTT 用 BIN**（飞控日志，权威）
- **不要混用**：我之前用 rotor.log 的 cmd 做功率、又用 BIN 的 tilt 做切片，两套时间轴必须先确认一致。

### 4.4 sanity check 设计

- 早期 sanity check 用 `Tzi/Txi`（投影分量）对比是错的——倾转后推力投影会差 cos(tilt)。
- 正确用 `thrmag`（推力大小，未投影），且要解释密度缩放（比值 1.060 = 1.225/ρ，不是 1.0）。
- **教训**：sanity check 的对照量必须和被检量在**同一个域**（都是推力大小，而非一个大小一个投影）。

### 4.5 不要急于下物理结论

- 在数据源未确认一致前，不该写"tilt 0-45° 外桨=0 是 SITL 真实行为"这种结论——这是把数据错配伪装成物理现象。
- **先数据、后物理**：数据自洽性验证通过，再谈物理解读。

### 4.6 环境工具

- matplotlib 因系统 numpy 2.x 与 apt matplotlib 3.5.1 不兼容无法用，最终 `pip install --user "numpy<2" matplotlib` 解决。
- **教训**：环境依赖冲突要早发现早处理，别用文本图凑合（虽然文本图作为 fallback 也有价值）。

---

## 5. 技术备忘

### 5.1 功率反查链（已验证正确）

```
rotor.log: rpm[8] (SITL Motor get_rpm = cmd×3181外/4200内)
           velin[8] (桨盘法向来流，内桨用)
  外桨 MOT_5..8 功率 = prop60_power(rpm)        [静态表，不依赖 v]
  内桨 MOT_1..4 功率 = prop40_power(rpm, velin)  [风洞表，v=velin 非 AS]
  总功率 = Σ8
```

与 SITL 内部 `SIM_Motor.cpp:138-140` 完全同函数同输入。sanity check：反算推力 vs `thrmag`，中位误差 0.4%。

### 5.2 ThO vs cmd（用户问过）

| 量 | 域 | 经 EXPO | 代码 |
|----|----|---------|------|
| CTUN.ThO | 推力域 | ❌ 之前 | `AP_MotorsMatrix.cpp:431` |
| rotor.log cmd | actuator 域 | ✅ 之后 | `SIM_Motor.cpp:38` |
| MOTT.Mot1-8 | actuator 域 | ✅ 之后 | 与 cmd 同域 |

功率反查用 cmd（actuator 域，EXPO 后），不能用 ThO（推力域，EXPO 前）。DF-200 `Q_M_THST_EXPO=1.0`，两者是平方关系。

### 5.3 脚本用法

```bash
# 默认（0.5s 滤波 + 图 + 报告）
python3 docs/scripts/df200_power_analysis.py logs/00000193.BIN --report docs/df200_power_corridor.md --csv /tmp/df200_power.csv
# 调滤波窗长
python3 docs/scripts/df200_power_analysis.py logs/00000193.BIN --smooth 1.0
# 关图
python3 docs/scripts/df200_power_analysis.py logs/00000193.BIN --no-plot
```

### 5.4 电机表数据源

- `libraries/SITL/SIM_MotorTable.h`：60in 外桨静态 15 点、40in 内桨风洞 7 风速分支
- `docs/df200_motor_data.md`：纯文本版，含单位换算和落地核对状态
- 编号：MOT_1-4 内倾转（40in）、MOT_5-8 外固定垂起（60in）

---

## 6. 待办（下一会话）

1. **跑一趟带 QHOVER 悬停段的飞行**：193 仍是 AUTO takeoff 爬升即 transition，无稳态悬停。要验证理论悬停功率（~12kW），需手动 QHOVER 悬停 30-60s。
2. **降峰值 150kW**：起飞瞬间集体油门 0.9，可调 `Q_WP_ACCEL_Z`（爬升加速度）或起飞先 QHOVER 避免瞬态。
3. **transition 时长**：卡在 tilt 45-60° 18s（等空速），调 `AIRSPEED_MIN` 或 `Q_TILT_RATE_UP` 可缩短。
4. **外桨 SHUTDOWN 根修**：`df200_session_20260902.md` §4 待办 #1，做后 transition 掉高会改善。
5. **对比电池规格**：垂起 90kW/50s（仿真稳态爬升 ~95kW 吻合）、倾转 110kW/10s（仿真峰值 112kW 吻合）、递减到 25（仿真巡航 7.6kW 远低，气动效率好）。
