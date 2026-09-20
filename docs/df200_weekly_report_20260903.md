# DF-200 倾转旋翼飞控调试周报（2026-08-28 ~ 09-03）

> 机型：DF-200 八旋翼倾转旋翼（内四倾转 40in + 外四固定垂起 60in，350kg，~400V）
> 代码分支：`qhz/plane_4.6.3`（ArduPlane 4.6.x）
> 本周主线：从"过渡即炸"到"巡航三通道锁定 + 功率走廊量化"，形成可交付的控制律与功率模型

---

## 0. 一句话状态

**DF-200 全程打通**：垂起爬升 → 过渡 → FW 巡航稳定飞行（193），纵向控制律闭环（190）；电机功率走廊已量化（悬停 ~95kW / 过渡 ~76kW / 巡航 ~7.6kW），峰值 150kW 成因定位。剩余：外桨 SHUTDOWN 根修、带悬停段的功率复测。

---

## 1. 本周工作主线（按时间）

| 时间 | 工作 | 关键产出/结论 |
|------|------|--------------|
| 8/28-29 | JT-20 功率分析 + 多余度调研收尾 | `jt_power_anaylisis.md`；多余度调研结论归档 |
| 8/30-31 | DF-200 过渡攻关（114-162） | 定位横航向自激墙 AS≈27、position%gyro 正反馈根因，`vtol_df200.md` §11 |
| 9/1 | 电机表模型推导与落地 | `df200_motor_thrust_derivation.md` + `SIM_MotorTable.h`，183 首次 FW done |
| 9/2 | 纵向闭环（185→190） | SCALING_SPEED/CRUISE=55/TIME_CONST=5 三连修，巡航锁定 |
| 9/3 | 功率走廊计算（本会话） | `df200_power_corridor.md` + 图 + 分析脚本 |

---

## 2. 交付物清单

### 2.1 代码（已提交，commit f6867ac 等）

| 文件 | 改动 |
|------|------|
| `libraries/SITL/SIM_MotorTable.h` | **新增**，60in 外桨静态表 15 点 + 40in 内桨风洞表 63 点（源自 `电机电池.docx`） |
| `libraries/SITL/SIM_Motor.h/.cpp` | 新增 `ThrustModel` 枚举、`calc_thrust_table`、`get_rpm`、表功率算电流 |
| `libraries/SITL/SIM_Frame.cpp` | df200 帧定义、电机表分配（内4=40in/外4=60in）、`/tmp/df200_*.log` 力账日志、zero_gyro 诊断开关 |
| `libraries/SITL/SIM_QuadPlane.cpp` | 上游单位 bug 修复（rot_accel/=inertia）、`df200_dyn.log` |
| `libraries/SITL/SIM_Plane.cpp` | `-df200` 固定翼气动系数（xlsx）、横航向导数 |
| `Tools/autotest/models/df200.json` | 质量/惯量/推力标定（mass×1.5=350kg，hoverThrOut 0.30→表模型后 0.49） |
| `Tools/autotest/default_params/quadplane-df200.parm` | 350kg 增益表、SCALING_SPEED=45/CRUISE=55/TIME_CONST=5 |

### 2.2 文档（本周新增/更新）

| 文档 | 内容 |
|------|------|
| `docs/vtol_df200.md` | DF-200 构型/参数/调参总册，§11 横航向自激机理与 114-162 攻关记录 |
| `docs/df200_session_20260902.md` | 9/2 纵向闭环会话：电机表落地→185→190 全过程，力账+根因 |
| `docs/df200_thrust_chain.md` | 推力链路详解（PID→EXPO→actuator→PWM→cmd→表→牛顿），EXPO/HOVER 同步坑 |
| `docs/df200_motor_thrust_derivation.md` | 电机表模型推导（单位换算、插值规则、标定链） |
| `docs/df200_motor_data.md` | **9/3 新增**，电机电池.docx 数据手册（纯文本，含功率走廊速查） |
| `docs/df200_power_corridor.md` | **9/3 新增**，功率走廊报告（4 图 base64 内嵌，可单文件发送） |
| `docs/df200_power_session_20260903.md` | **9/3 新增**，功率分析会话总结（含 tilt 误解复盘） |
| `docs/jt_power_anaylisis.md` | JT-20 倾转过渡功率分析 |

### 2.3 分析脚本

| 脚本 | 用途 |
|------|------|
| `docs/scripts/df200_power_analysis.py` | **9/3 新增**，功率走廊计算（滤波/tilt切片/matplotlib图/CSV/差动影响分析） |
| `docs/scripts/df200_force_budget.py` | 力账+力矩+混控器一体化诊断 |
| `docs/scripts/df200_transition_log.py` | 里程碑/onset/时间线诊断 |
| `docs/scripts/df200_motor_thrust_derive.py` | 电机表推导辅助 |
| `docs/scripts/jt20_power_from_log.py` | JT-20 功率从日志反推 |

---

## 3. 核心技术成果

### 3.1 DF-200 过渡墙攻克（横航向自激，§11）

**问题**：AS≈27 反复炸机，roll 速率指数发散。

**根因链**（162 隔离试验证实）：
```
小扰动 δr → position%gyro 给左右电机反向 mvelz
→ projected(thrust_vector) 放大 → velin 左右分裂
→ calc_thrust 平方差（高速增益~81）→ 推力不对称
→ 几何 Mx/Mz 同号 → 角加速度增大 → 回到 δr（正反馈）
```
**关键**：发散不取决于种子大小，取决于阻尼/耦合净符号；`position%gyro` 切断即不炸（诊断用）。

### 3.2 纵向控制律闭环（185→190，三轮单参数修复）

| 修复 | 参数 | 效果 |
|------|------|------|
| 俯仰链解锁 | SCALING_SPEED 12→45 | 三轴 FW 增益不再被压 93%，舵效恢复 |
| TECS 无低头需求 | AIRSPEED_CRUISE 75→55 | 交接点即平衡点，机翼 1.0W 托住 |
| phugoid 收敛 | TECS_TIME_CONST 3→5 | 高度摆幅 ±3.5m→±0.3m，巡航锁定 |

**190 收尾**：sp=56.6/56.6（零误差）、h=20.0/hdem=20.0、ph=+0.7°配平——三通道全部锁定。

### 3.3 电机表模型落地（替代动量盘）

**为什么**：旧 `calc_thrust` 动量盘在 AS=50 给负推力，且把 velin 差异平方成同号 Mx 正反馈。
**怎么做**：`SIM_MotorTable.h` 查表（60in 静态/40in 风洞 RPM×风速），`last_rpm=cmd×(3181/4200)`，`prop40_power(rpm,velin)` + `prop60_power(rpm)`。
**配套**：必须同步 `Q_M_THST_EXPO=1.0`（T∝a²）、`Q_M_THST_HOVER=0.49`（=W/T_max），否则飞控推力线性化错配。

### 3.4 功率走廊量化（9/3，本会话）

**功率走廊**（193 仿真实测，sanity check 反查误差 0.4%）：

| 阶段 | 时长s | 均值kW | 峰值kW | 能耗Wh |
|------|-------|--------|--------|--------|
| vtol_climb（垂起爬升） | 30.8 | 92.5 | 150.1 | 792 |
| transition（过渡） | 25.8 | 75.8 | 112.7 | 544 |
| fw_cruise（FW巡航） | 24.0 | 7.6 | 79.6 | 50 |

**transition 按 tilt 切片**（功率随倾转角，非时间）：

| tilt | 0-30° | 30-45° | 45-60° | 60-90° |
|------|-------|--------|--------|--------|
| 均值kW | 88.0 | 102.8 | 76.2 | 47.8 |
| 外桨 | 49.5 | 59.6 | 39.9 | 23.8 |

**物理结论**：外桨 tilt=0 满推 52kW → tilt=90 归 0，单调衰减符合物理。

### 3.5 差动对功率影响的数学论证

**结论**：恒定总推力下，差动越大功率越高（凸函数性质）。
- `T∝rpm²`、`P∝rpm³` → `P∝T^1.5`（凸）
- 詹森不等式：`ΣT_i^1.5 ≥ n·(ΣT/n)^1.5`，分布越不均功率越高
- 数值验证：总推力 1805N，均匀分布 23.6kW，大差动（90/10）31.2kW（+32%）
- **指导**：峰值 150kW 成因之一是起飞瞬间外桨"两满两关"强差动；限制单桨 cmd 上限可降峰值

---

## 4. 关键数据校验（sanity）

- **功率反查链**：与 SITL 内部 `SIM_Motor.cpp:138-140` 同函数同输入，反算推力 vs 日志 thrmag 中位误差 0.4%
- **内桨用 velin 非 AS**：40in 表是 RPM×风速二维表，velin 是桨盘法向来流（源码 `SIM_Motor.cpp:66/140`）
- **ThO vs cmd**：ThO 是推力域（EXPO 前），cmd 是 actuator 域（EXPO 后），DF-200 EXPO=1.0 两者平方关系，功率反查用 cmd

---

## 5. 待办（下周）

| 优先级 | 事项 | 说明 |
|--------|------|------|
| P0 | **外桨 FW 段 SHUTDOWN 根修** | `df200_session_20260902.md` §4 #1，交接断崖 −0.33g 归零，掉高 3.5m→~1m |
| P0 | **带 QHOVER 悬停段的功率复测** | 193 仍无稳态悬停，需手动 QHOVER 30-60s 验证理论悬停功率 ~12kW |
| P1 | **降峰值 150kW** | 调 Q_WP_ACCEL_Z、起飞先 QHOVER 避免瞬态、或限制单桨 cmd 上限 |
| P1 | **缩短 transition 时长** | 卡在 tilt 45-60° 18s 等空速，调 AIRSPEED_MIN 或 Q_TILT_RATE_UP |
| P2 | Q_ASSIST_SPEED 10→30 | 低成本兜底 |
| P2 | gyro 置零开关 Run2 | 验证表模型能否压住 position%gyro 正反馈 |

---

## 6. 与电池规格的对比

| 阶段 | 电池规格 | 仿真实测 | 一致性 |
|------|---------|----------|--------|
| 垂起 | 90kW/50s | 稳态爬升 ~95kW，峰值 150kW（起飞瞬态） | 稳态吻合 |
| 倾转 | 110kW/10s | 峰值 112.7kW | **峰值吻合** ✓ |
| 递减 | 50s 峰值→25 | 巡航稳态 7.6kW | 仿真更低（气动效率好） |

峰值 150kW 高于电池规格 90kW 的原因：起飞瞬间集体油门 0.9 + 强差动（两满两关）。降法见待办 P1。

---

## 7. 文档索引（快速定位）

- 控制律调参/构型：`docs/vtol_df200.md`
- 9/2 纵向闭环全过程：`docs/df200_session_20260902.md`
- 推力链路/EXPO坑：`docs/df200_thrust_chain.md`
- 电机表数据：`docs/df200_motor_data.md`
- 功率走廊报告（含图）：`docs/df200_power_corridor.md`
- 9/3 功率会话复盘：`docs/df200_power_session_20260903.md`
- 横航向自激机理：`docs/vtol_df200.md` §11
