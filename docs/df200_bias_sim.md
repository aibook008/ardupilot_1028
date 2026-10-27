# DF-200 拉偏仿真自动化流水线 — 开发计划 + 实施状态

> 日期：2026-09-20 定版；**2026-09-29 更新实施状态（§0a/§9/§10）**
> 目标：为协议 1.4「鲁棒性拉偏仿真」（见 `docs/飞控开发说明.md` 节点一）打通自动化测试流程——**升阻特性、舵效、重量重心、风场四维拉偏矩阵**，一键跑批、逐 case 出结果档案，支撑仿真测试报告（节点 1.5）。
> 前提：DF-200 全链 SITL 已通（正向转换 208 架次、反向刹车链 225 架次验证，见 `docs/vtol_df200.md`、`docs/vtol_reverse_braking_summary.md`）。
> 所有行号基于工作区 HEAD `06f5093`（2026-09-20 验证）。

---

## 0a. 实施状态总览（2026-09-29）

> 阅读提示：§0-§8 为 09-20 定版的设计计划（保留作设计依据）；**§9 为实际落地详述**（参数名、idx、脚本用法均以 §9 为准——如升降舵效参数实名为 `SIM_AERO_ELE_SCL`，计划稿中的 `SIM_AERO_ELEV_SCL` 未采用）；§10 为剩余工作。

原计划的 6 步实施顺序，当前进度：

| 步骤（§6） | 状态 | 说明 |
|------|------|------|
| 1. C++ 参数组（§2） | ✅ **已完成** | 十参数已实装（参数名/idx 与计划有差异，见 §9.1），编译通过，默认值零漂移 |
| 2. mission 5 件套（§3） | ✅ **已完成（4+1）** | `DF200_{full,hover,transition,brake,fw_cruise}.txt` 已建（§9.2）；另有 `DF200_hover_gqc.txt`、`DF_POINT.TXT` |
| 3. df200.py TestSuite（§4） | ⚠️ **方案变更** | 未走 autotest.py TestSuite 路线，改为**独立 MAVLink 脚本** `docs/scripts/df200_case_run.py`（§9.3），功能等价且免 TestSuite 框架依赖 |
| 4. 分科目 4 测试 | ✅ 经手动/脚本验证 | hover / transition / brake / full 均有成功飞行记录 |
| 5. PerturbationMatrix | 🟡 **部分跑通** | 矩阵模式已跑：cd0+30、cla-20 PASS（14 里程碑）；baseline 全链成功但状态字段有历史版本残留（见 §9.5）；早期 FAIL 均为连接/注入时序问题，已由当前脚本版本修复 |
| 6. 指标离线分析 | ⬜ 未接入 | `df200_transition_log.py` / `df200_force_budget.py` 可对 BIN 手动分析，未批量挂接 |

配套新增（计划外）：

- `docs/scripts/df200_mission_run.py` —— 纯脚本任务执行器（hover/auto 双 profile，不依赖 QGC/MAVProxy）；
- `docs/scripts/df200_sitl_restart.sh` —— 矩阵 case 间的 SITL 重启包装（kill → `-w` 清参 → KSFO 冷启动）；
- **模型对齐前置工作**：`df200.json` 恢复 Excel 真实几何 + zero-gyro 诊断开关关闭（Run#1/Run#2 验证），SITL 进入"真实几何+真实入流+桨表推力"全物理配置——拉偏矩阵的 baseline 以此为准（总结见 `docs/bias_sim.md`，详析见 `docs/df200_model_alignment_plan.md`）。

---

---

## 0. 现状与差距

| 已有 | 缺口 |
|------|------|
| 全链飞行能力（垂起→转换→FW→反向刹车→降落） | 无 DF-200 专属 mission 文件（CMAC 场地容不下 55 m/s + 4 km 刹车段） |
| 气动系数写入 `SIM_Plane.cpp` `-df200` 分支 | 系数为**编译期常量**，无法运行时拉偏 |
| 风场参数 `SIM_WIND_SPD/DIR/TURB` 运行时可注入 | 无统一 case 矩阵调度器 |
| 日志分析脚本 `docs/scripts/df200_transition_log.py` 等 | 未接入自动判定/归档 |
| autotest TestSuite 框架（`Tools/autotest/quadplane.py`） | 无 DF200 子类 |

## 1. 总体架构

```text
┌─────────────────────────────────────────────────────────────┐
│ autotest.py  test.DF200.DF200PerturbationMatrix             │
│   └─ df200.AutoTestDF200 (TestSuite 子类)                    │
│       for case in CASES:                                    │
│         start_SITL(wipe=True, home=KSFO)                    │
│         set_parameters(case.params)   ← 四维拉偏注入         │
│         load_mission(case.mission)                          │
│         arm → wait_waypoint → wait_disarmed (里程碑判定)     │
│         拷贝 logs/*.BIN → case 档案 + result.json            │
│       汇总 → summary 表格                                    │
└─────────────────────────────────────────────────────────────┘
拉偏注入通道：
  ① SIM_AERO_* / SIM_MASS_SCL / SIM_INERTIA_SCL   新增 C++ 参数（本文 §2）
  ② SIM_WIND_SPD/DIR/TURB                          现有参数
  ③ mission 选择                                   全链 / 分科目
```

设计原则：

- **判据只判里程碑**（用户定版）：PASS = 降落 disarm、无 crash；姿态/掉高指标记录进档案不判超差，供报告离线分析。
- **拉偏用参数注入，不用 per-case json**：PARAM_SET 运行时即生效（先例：`arduplane.py:941` 飞行中改 `SIM_WIND_SPD`），避免 json 相对路径/cwd 坑。
- **气动基准值不动**：`SIM_Plane.cpp` `-df200` 分支保持硬编码，缩放乘在**消费点**——参数默认 1.0 = 现有 208/225 已验证行为零漂移。

---

## 2. C++ 改动一：SIM_AERO 拉偏参数组（维度①②③）

### 2.1 参数定义（加在 `SITL::var_info2` 表尾部）

`libraries/SITL/SITL.cpp:262` 起为 var_info2，现有最大 idx=63（`VIB_MOT_MULT`，:503）。新参数续用 idx 64-73。参数文档注释按仓库规范写 `@Param/@DisplayName/@Description/@Range`。注册链路无需改动（SIM 构造函数对 var_info2 已有 `setup_object_defaults`，`SITL.h:114-129`）。

| 参数 | idx | 默认 | 语义 |
|------|-----|------|------|
| `SIM_AERO_CLA_SCL` | 64 | 1.0 | 升力线斜率 ×`c_lift_a` |
| `SIM_AERO_CD0_SCL` | 65 | 1.0 | 零升阻力 ×`c_drag_p` |
| `SIM_AERO_IND_SCL` | 66 | 1.0 | 诱导阻力：oswald **除**以该值（等效 e 拉偏，>1 更差） |
| `SIM_AERO_CMA_SCL` | 67 | 1.0 | 纵向静稳定 ×`c_m_a` |
| `SIM_AERO_ELEV_SCL` | 68 | 1.0 | 升降舵效 ×`c_m_deltae` |
| `SIM_AERO_AIL_SCL` | 69 | 1.0 | 副翼舵效 ×`c_l_deltaa` |
| `SIM_AERO_RUD_SCL` | 70 | 1.0 | 方向舵效 ×`c_n_deltar` |
| `SIM_AERO_CGX_OFF` | 71 | 0.0 | 重心偏移**增量**（m，+ = 后移），叠加到 CGOffset.x |
| `SIM_MASS_SCL` | 72 | 1.0 | 整机质量缩放（基准 380 kg） |
| `SIM_INERTIA_SCL` | 73 | 1.0 | 三轴惯量同乘 |

成员声明加 `libraries/SITL/SITL.h` SIM 类（`AP_Float aero_cla_scale;` 等 10 个）。

### 2.2 乘入位置（4 个消费点）

气动系数的**唯一汇聚点**是 `SIM_Plane.cpp` 中四处局部绑定/计算：

1. **`Plane::liftCoeff`**（`SIM_Plane.cpp:242-262`）：`c_lift_a0` 绑定行（:247）乘 `cla_scale`。
2. **`Plane::dragCoeff`**（:264-277）：`c_drag_p` 绑定行（:268）乘 `cd0_scale`；`oswald` 绑定行（:271）改为 `oswald / ind_scale`。
3. **`Plane::getTorque`**（:280-350）：局部绑定段（:297-316）中 `c_l_deltaa`(:304)、`c_m_deltae`(:309)、`c_n_deltar`(:315) 各乘对应 `_scl`；**CGOffset**（:316 引用，叉乘 :343-345）改为局部副本：
   ```cpp
   Vector3f CGOffset = coefficient.CGOffset;
   CGOffset.x += sitl->aero_cgx_off;
   ```
   物理依据见 §2.4。
4. **`Plane::getForce`**（:351-403）：:371-372 调 `liftCoeff/dragCoeff` 自动继承 1/2 的缩放；实现时检查函数内是否还有独立系数绑定需同步。

取参方式：`const SIM *_sitl = AP::sitl();`（`SIM_Plane.cpp` 内已有先例 :455 读 `batt_voltage`）。

### 2.3 /tmp/df200_aero.log 同步（必须）

`SIM_Plane.cpp:483-553` 的 df200 力矩分解日志用**与 getTorque 相同的公式重算**（:498 起 `coefficient.c_l_b`、surface 项、CG 叉乘 :512）。缩放若不同步，`df200_force_budget.py` 对账将失真——这是本项目 17 架次积累的主要诊断手段，属硬要求。改法：该分支套用同一组 scale（复用同一局部函数/宏，避免两处漂移）。

### 2.4 CG 拉偏的物理正确性

`CGOffset = aeroCenter − CG`（df200 = (−0.054,0,0)，`SIM_Plane.cpp:178`）。SITL 中 Frame 电机位置 r 本身**相对悬停 CG** 定义（`df200.json` 注释），即「电机不动 + CGOffset 平移」已覆盖重心移动的全部动力学效应（重力过 CG 无力矩、电机力臂不变），因此：

- **正确**：`CGOffset.x += SIM_AERO_CGX_OFF`（重心后移为 +，等效静稳定裕度减小/抬头趋势）；
- **错误**：缩放电机位置 r——那是改机架几何，破坏悬停标定。

### 2.5 质量与惯量

两处质量消费点，**必须同口径**：

| 消费点 | 现状 | 改法 |
|--------|------|------|
| `Frame::calculate_forces` 末尾 `body_accel = thrust/aircraft.gross_mass()`（`SIM_Frame.cpp:771`） | `gross_mass()` = `mass + external_payload_mass`（`SIM_Aircraft.h:128`） | 改为 `thrust / (aircraft.gross_mass() * mass_scale)` |
| `Plane::calculate_forces` 的 `accel_body /= mass`（`SIM_Plane.cpp:582`） | 裸 `mass`（Plane 部分比重力/加速度换算） | 同步乘 `mass_scale` |

**注意**：`SIM_QuadPlane.cpp:112-113` 的 `mass = frame->get_mass()*1.5; frame->set_mass(mass)` 是初始化一次性赋值，保持不动（scale 乘在消费端，×1.5 层级不变，基准=380 kg 飞行质量）。

惯量：`Frame::calculate_forces` 的 `rot_accel = torque/moment_of_inertia`（`SIM_Frame.cpp:739-741`）三行分母乘 `inertia_scale`。（`SIM_QuadPlane.cpp:170-173` 是把 Plane 力矩除以 Frame 惯量的单位修正，分母用同一 `frame->get_inertia()`——若在 `get_inertia()` 返回处统一乘 scale 则两处一次覆盖，实现时二选一并注明。）

---

## 3. C++ 改动二：mission 与场地（维度④的载体）

### 3.1 新 mission 文件（`Tools/autotest/Generic_Missions/`）

QGC WPL 110 格式，模板参照 `CMAC-VTOL-ccw.txt`。场地用 **KSFO**（`locations.txt:26`：37.619373,-122.376637,5.3,118；沿 090/270 有足够海上直线空间）。

关键约束：**反向刹车链由 RTL 触发**，进入段 `stop_distance = MAX(4000, 2phase+2clos)`（`ArduPlane/quadplane.cpp:2492`）——RTL 航向必须对准 home 且**回程直线段 ≥4 km**，否则 `distance < stop_distance`（:2507）不满足、AIRBRAKE/交棒不触发。

| 文件 | 剖面 | 用途 |
|------|------|------|
| `DF200_full.txt` | takeoff(84)→爬升→出航 ~6km 直线→巡航段→回程对准 home 直线 ≥4.5km→**RTL(20)**→VTOL land(85) | 全链一镜到底（正向转换+FW+反向刹车链） |
| `DF200_hover.txt` | takeoff→悬停 WP→land | 垂起悬停科目 |
| `DF200_transition.txt` | takeoff→DO_VTOL_TRANSITION(2200,p1=4)→短航程→land | 正向转换科目 |
| `DF200_fw_cruise.txt` | 转换→3 WP 长航线→land | FW 巡航科目 |
| `DF200_brake.txt` | 转换→出航 ≥5.5km→RTL→land | 反向刹车科目 |

坐标数值实现时以 KSFO 为原点沿 090/270 航向计算（1°纬≈111.1 km，1°经@38°N≈87.6 km），并先手动 sim_vehicle 验证 RTL 触发链（§6 步骤 2）。

### 3.2 vehicleinfo.py

`Tools/autotest/pysim/vehicleinfo.py:303` 的 `quadplane-df200` 条目不动（frame 原样传入会精确命中，规避 `--model` 覆盖丢 default_params 的前缀回退坑）。

---

## 4. TestSuite：`Tools/autotest/df200.py`

### 4.1 类骨架

```python
class AutoTestDF200(quadplane.AutoTestQuadPlane):
    def default_frame(self): return "quadplane-df200"   # 精确命中 vehicleinfo 条目
    def default_speedup(self): return 10                # 大惯量+1200Hz 内环；父类 100 不可用
    def default_bodyno(self): return "DF200"
```

继承复用：`start_SITL`（`vehicle_test_suite.py:9140`，wipe=True 隔离）、`load_mission`（:5278，MISSION_ITEM_INT 协议上传）、`set_parameters`（:6250，带回读校验）、`wait_waypoint/wait_altitude/wait_disarmed/wait_statustext`、`context_collect('STATUSTEXT')`。

### 4.2 测试注册

```python
def tests(self):
    return [
        ("DF200Hover",              self.test_hover_takeoff),
        ("DF200ForwardTrans",       self.test_forward_transition),
        ("DF200FWCruise",           self.test_fw_cruise),
        ("DF200ReverseBrake",       self.test_reverse_brake),
        ("DF200FullChain",          self.test_full_chain),
        ("DF200PerturbationMatrix", self.test_perturbation_matrix),
    ]
```

autotest.py 挂表（`Tools/autotest/autotest.py:347` `tester_class_map`）：

```python
"test.DF200": df200.AutoTestDF200,
```

（`split_specific_test_step`（:329）的正则 `((fly|drive|dive|test)[.][^.]+)[.](.*)` 把 `test.DF200.xxx` 拆成 `("test.DF200", "xxx")`，两级键名可用，参照 `test.CopterTests1a` 先例。）

调用：

```bash
./Tools/autotest/autotest.py --vehicle Plane test.DF200.DF200FullChain
./Tools/autotest/autotest.py --vehicle Plane test.DF200.DF200PerturbationMatrix
```

### 4.3 拉偏矩阵（核心）

```python
CASES = [
    dict(name="baseline",      mission="DF200_full",   params={}),
    # 升阻
    dict(name="cla+20",        params={"SIM_AERO_CLA_SCL": 1.2}),
    dict(name="cla-20",        params={"SIM_AERO_CLA_SCL": 0.8}),
    dict(name="cd0+30",        params={"SIM_AERO_CD0_SCL": 1.3}),
    dict(name="ind+30",        params={"SIM_AERO_IND_SCL": 1.3}),
    # 舵效
    dict(name="elev-30",       params={"SIM_AERO_ELEV_SCL": 0.7}),
    dict(name="ail-30",        params={"SIM_AERO_AIL_SCL": 0.7}),
    dict(name="rud-30",        params={"SIM_AERO_RUD_SCL": 0.7}),
    # 重量重心
    dict(name="cgx+8cm",       params={"SIM_AERO_CGX_OFF": 0.08}),
    dict(name="mass+15",       params={"SIM_MASS_SCL": 1.15}),
    dict(name="inertia+30",    params={"SIM_INERTIA_SCL": 1.3}),
    # 风场
    dict(name="wind10_head",   params={"SIM_WIND_SPD": 10, "SIM_WIND_DIR": 270}),
    dict(name="wind15_xwind",  params={"SIM_WIND_SPD": 15, "SIM_WIND_DIR": 0}),
    dict(name="turb+combo",    params={"SIM_WIND_TURB": 3, "SIM_MASS_SCL": 1.1,
                                       "SIM_AERO_CGX_OFF": 0.05}),
    # ... 拉偏幅度按协议要求增删；矩阵内容独立成模块级 list 便于维护
]

def test_perturbation_matrix(self):
    results = []
    for case in CASES:
        case_dir = os.path.join(self.logdir(), "df200_matrix", case["name"])
        try:
            self.start_SITL(home=KSFO_HOME_STRING, wipe=True, speedup=self.speedup)
            self.set_parameters(case.get("params", {}))
            self.load_mission(MISSIONS[case["mission"]])
            self.change_mode("AUTO")
            self.arm_vehicle()
            self.wait_waypoint(0, num_wp-1, timeout=...)   # 显式 timeout 兜底
            self.wait_disarmed(timeout=...)
            status = "PASS"
        except Exception as e:
            status = "FAIL: %s" % repr(e)
        finally:
            self._archive_case(case_dir, case, status)     # 拷 BIN + result.json
            self.stop_SITL()  # 强杀残留 SITL（防 crash hang 住后续 case）
        results.append((case["name"], status))
    self.progress("MATRIX: %s" % results)
    # 单 case FAIL 只记录不中断矩阵；汇总表打印到 progress/summary.md
```

要点：

- **隔离**：每 case `start_SITL(wipe=True)`——wipe 清 eeprom，SIM 参数复位默认值，杜绝跨 case 残留。case 内 `set_parameters` 注入（带回读校验）。
- **失败处理**：单 case FAIL 只记录不中断；finally 强杀 SITL。
- **超时**：全链 12-16 km @55 m/s ≈ 5 min 仿真时 + 转换/刹车余量；speedup=10 时 wall-clock ≈ 1.5-2 min/case，20 case 矩阵约 40 min。
- **调试日志清理**：`/tmp/df200_aero.log`、`/tmp/df200_rotor.log`、`/tmp/df200_mix.log`、`/tmp/df200_dyn.log` 均为追加写（`SIM_Plane.cpp:527` 等），每 case 开始前清空并归档副本，否则跨 case 混叠无法对账。

### 4.4 结果档案

每 case 目录 `{logdir}/df200_matrix/{case_name}/`：

- `result.json`：case 配置、status、里程碑时间戳（从 STATUSTEXT 提取：VTOL takeoff complete / Transition done / airspeed reached / VTOL position1 / Land complete）、指标摘要（可后续接入 `df200_transition_log.py` 的 `phase_flags()/detect_onsets()` 离线补充）；
- `NNNNNNNN.BIN`（从 SITL cwd `logs/` 拷贝）；
- 清理后的 `/tmp/df200_*.log` 副本。

汇总：`{logdir}/df200_matrix/summary.md`（case × status 表）。

---

## 5. 文件清单

| 操作 | 路径 | 内容 |
|------|------|------|
| 改 | `libraries/SITL/SITL.h` | SIM 类新增 10 个 AP_Float 成员 |
| 改 | `libraries/SITL/SITL.cpp` | var_info2 尾部 +10 个 AP_GROUPINFO（idx 64-73） |
| 改 | `libraries/SITL/SIM_Plane.cpp` | 4 消费点乘 scale + CGOffset 增量 + aero log 同步 + `accel_body /= mass` 乘 mass_scale |
| 改 | `libraries/SITL/SIM_Frame.cpp` | `:771` gross_mass 乘 mass_scale、`:739-741` 惯量除法乘 inertia_scale |
| 新 | `Tools/autotest/df200.py` | TestSuite 子类 + CASES + matrix 调度 + 档案 |
| 新 | `Tools/autotest/Generic_Missions/DF200_{full,hover,transition,fw_cruise,brake}.txt` | 5 个 mission |
| 改 | `Tools/autotest/autotest.py` | import df200 + tester_class_map 一行 |
| 新 | `docs/df200_bias_sim.md` | 本文档 |

---

## 6. 实施顺序与验证

| 步骤 | 内容 | 验证 |
|------|------|------|
| 1 | C++ 参数组（§2） | `./waf plane` 编译过；手动 `param set SIM_AERO_CLA_SCL 1.5`，对照 `/tmp/df200_aero.log` 力矩数值确认生效；默认值 1.0 时跑一次全链对照 208/225 行为无漂移 |
| 2 | mission 5 件套（§3） | 手动 `sim_vehicle.py + MAVProxy` 飞通 `DF200_full.txt`：确认 RTL 进入段触发 AIRBRAKE（STATUSTEXT "VTOL position1"）→ 交棒 → 降落 disarm |
| 3 | df200.py 骨架 + FullChain 单测 | `./Tools/autotest/autotest.py --vehicle Plane test.DF200.DF200FullChain` PASS |
| 4 | 分科目 4 测试 | 逐个 autotest.py PASS |
| 5 | PerturbationMatrix（§4.3） | 全矩阵跑通，baseline PASS；逐 case 复核 result.json 里程碑齐套 |
| 6 | 指标离线分析接入（可选二期） | df200_transition_log.py 批量出各 case 掉高/姿态摘要，纳入 summary |

## 7. 风险与注意事项

1. **speedup=10 数值稳定性**：169 架次曾出现 `La_p` Euler 发散（`vtol_df200.md` §11.8，高频率大速率项）；拉偏（尤其 `SIM_INERTIA_SCL` 变小方向）会加剧。若矩阵 case 出现 baseline 没有的高频振铃，先降 speedup=5 复跑排除数值因素，再判 FAIL。
2. **aero log 同步**：§2.3 不同步会让力账工具链（df200_force_budget.py）失真——硬要求。
3. **RTL 触发依赖航迹对准**：L1 转弯未完成时进 RTL，反向链的 attitude 3s 时间门 / direction 500 m 距离门（`ArduPlane/quadplane.cpp:2540` 附近）可能延迟交棒。mission 设计保证回程直线段充足；判 FAIL 前先看 STATUSTEXT 是否走完五段。
4. **未提交工作区**：`quadplane-df200.parm`（θ–V 走廊 `Q_TILT_CRD_*`）当前为未提交修改——流水线依赖它，跑批前确认工作区状态；`SIM_MotorTable.h`、`docs/scripts/` 等未跟踪文件建议尽快纳入版本控制，避免流水线环境缺文件。
5. **eeprom/wipe**：TestSuite SITL cwd 为 autotest 运行目录，勿手动塞 eeprom.bin；wipe=True 是唯一隔离手段，case 中途不要重启 SITL。
6. **mass 双口径**：只改 `SIM_Frame.cpp:771` 不改 `SIM_Plane.cpp:582` 会产生隐式重力拉偏（Plane 部分 weight 换算错误），务必同步（§2.5 表）。
7. **报告联动**：矩阵 summary + 每 case result.json/BIN 即节点 1.5「仿真测试报告」的原始素材；离线指标判据（掉高阈值、roll RMS 等）二期再接入（节点 1.4 出口准则「不发散、可恢复」一期以里程碑判定覆盖）。

---

## 8. 一句话状态

**方案定版（2026-09-20）：SIM_AERO_* 十参数 + DF200 TestSuite + KSFO 五 mission 三件套；判据只判里程碑；实现顺序 C++ → mission → Suite → 矩阵。**
**实施推进（2026-09-29）：C++ 参数组与 mission 已落地；TestSuite 改为独立脚本 df200_case_run.py；矩阵模式已跑通（cd0+30 / cla-20 PASS），余 case 待批量执行。**

---

# 9. 已完成实现详述（2026-09-29）

> 本章记录实际落地的代码与用法，与上方原计划（§0-§8，保留作设计依据）冲突处以本章为准。

## 9.1 C++ 拉偏参数组：已实装（未提交，工作区）

实装与计划（§2.1）的差异：参数最终挂在 **SIM 参数空间**、名字去掉 `SIM_` 前缀由前缀规则自动带上、idx 复用表内空洞（11-14/18-20/7-9），而非计划中的 64-73。定义在 `libraries/SITL/SITL.cpp:505` 起（带 `@Param/@DisplayName/@Description/@Range` 注释，20 行文档注解），成员在 `libraries/SITL/SITL.h:419` 附近：

| 参数（实际名） | idx | 默认 | 语义（乘入点） |
|------|-----|------|------|
| `SIM_AERO_CLA_SCL` | 11 | 1.0 | `c_lift_a`，乘在 `liftCoeff()`（`SIM_Plane.cpp:251`） |
| `SIM_AERO_CD0_SCL` | 12 | 1.0 | `c_drag_p`，乘在 `dragCoeff()`（`SIM_Plane.cpp` 绑定行） |
| `SIM_AERO_IND_SCL` | 13 | 1.0 | oswald **除**以该值（`MAX(ind_scale,0.05)` 防除零），>1 更差 |
| `SIM_AERO_CMA_SCL` | 14 | 1.0 | `c_m_a`，乘在 `getTorque()` |
| `SIM_AERO_ELE_SCL` | 18 | 1.0 | `c_m_deltae` × elev_scale |
| `SIM_AERO_AIL_SCL` | 19 | 1.0 | `c_l_deltaa` × ail_scale |
| `SIM_AERO_RUD_SCL` | 20 | 1.0 | `c_n_deltar` × rud_scale |
| `SIM_AERO_CGX_OFF` | 7 | 0.0 | CGOffset.x 增量（+ 后移），m，Range −0.3~0.3 |
| `SIM_MASS_SCL` | 8 | 1.0 | 质量，**双消费点同步**（见下） |
| `SIM_INERTIA_SCL` | 9 | 1.0 | 三轴惯量分母同乘（`MAX(_,0.05)` 防除零） |

乘入点全部在**消费端**（计划 §2.2 原则兑现）：默认 1.0/0.0 时与标定基线逐位一致。`/tmp/df200_aero.log` 力矩分解已按 §2.3 要求同步 scale（`SIM_Plane.cpp` df200 日志分支内 ail/elev/rud scale 与 getTorque 同源）。

质量双消费点已同步（§2.5 硬要求兑现）：
- `SIM_Frame.cpp`：`body_accel = thrust / (gross_mass() * mass_scale)`；
- `SIM_Plane.cpp:618`：`mass_eff = mass * mass_scale`。

惯量：`SIM_Frame.cpp` 三行 `rot_accel = torque / (moment_of_inertia * inertia_scale)`（计划 §2.5 中"二选一"的选择：乘在 Frame 端除法处；`SIM_QuadPlane.cpp:170-173` 的单位修正用同一 `frame->get_inertia()` 未动——其引用的仍是无缩放 json 值，两侧比值一致故行为正确，若后续做惯量拉偏对账需注意此处口径）。

编译：`./waf plane` 通过。

## 9.2 mission 文件：已建（KSFO，未提交）

`Tools/autotest/Generic_Missions/` 下，QGC WPL 110 格式，home=KSFO（37.619373,-122.376637,5.3,118）：

| 文件 | 内容 | 用途 |
|------|------|------|
| `DF200_full.txt` | home → VTOL takeoff(84, 50m) → WP(北 2.8km) → VTOL land(85) | 全链（转换+FW+返场 VTOL 降落），A→B 直线约 2.5 min |
| `DF200_brake.txt` | takeoff(30m) → DO_VTOL_TRANSITION(2200,p1=4) → WP(北 5.5km, 120m) → **RTL(20)** → VTOL land | 反向刹车链（RTL 触发 AIRBRAKE→交棒→多旋翼降落） |
| `DF200_transition.txt` / `DF200_fw_cruise.txt` / `DF200_hover.txt` | 对应分科目 | 单科目 |
| `DF200_hover_gqc.txt`、`DF_POINT.TXT` | 辅助/草稿 | — |

注：`full` 的回程段按脚本化返场（position1/position2 链）实现而非原计划的 4.5km 直线 RTL——VOL position1/Overshoot/position2 里程碑在 baseline_004 中全部出现，反向链触发正常。

## 9.3 运行脚本：`docs/scripts/df200_case_run.py`（核心）

**取代原计划的 `Tools/autotest/df200.py` TestSuite 方案**。理由：不依赖 autotest.py 框架（其 SITL 启动/串口分配对多 case 矩阵不友好），直接 pymavlink 连 SITL 串口，单文件闭环。

单 case 闭环流程（`run_case()`，`df200_case_run.py:177`）：

```text
建档案目录 logs/df200_cases/<case>_<NNN>/
→ [可选] 执行 restart_cmd 重启 SITL（默认 df200_sitl_restart.sh：kill 旧 SITL → -w 清参 → KSFO 冷启动，等 20s）
→ 连接（5762/5760 双端口重试，真心跳校验）
→ 注入拉偏参数（PARAM_SET，带回读校验）
→ 上传 mission（逐条握手 + ack 校验）
→ AUTO + arm
→ 监控 STATUSTEXT 至 DISARMED，抓取 14 类里程碑文本
→ 归档：run.log + flight.BIN(最新 logs/*.BIN 拷贝) + result.json（params/里程碑/status）
→ PASS 判定：Land complete 出现 且 无"未伴随 Land complete 的 Hit ground"
```

矩阵模式：`--matrix` 后接 CASE_LIB 名单，逐 case 顺序执行（每 case 前自动重启 SITL），结束写 `logs/df200_cases/summary_<ts>.md` 汇总表。内置 CASE_LIB 12 个 case（baseline/cla±20/cd0+30/ind+30/cma-25/ele-30/ail-30/rud-30/cgx+8cm/mass+15/inertia+30）。

**用法**（仓库根目录）：

```bash
# 前提：已 ./waf configure --board sitl && ./waf plane（工作区含拉偏参数改动）

# 1) 单 case（CASE_LIB 名）
python3 docs/scripts/df200_case_run.py --case cd0+30

# 2) 单 case（任意参数组合）
python3 docs/scripts/df200_case_run.py --case mytest --param SIM_AERO_CGX_OFF=0.08 --param SIM_MASS_SCL=1.1

# 3) 矩阵（每 case 自动重启 SITL 清参数）
python3 docs/scripts/df200_case_run.py --matrix baseline cla+20 cla-20 cd0+30

# 4) 换 mission / 超时 / 禁用重启
python3 docs/scripts/df200_case_run.py --case baseline --mission Tools/autotest/Generic_Missions/DF200_brake.txt \
    --max-time 900 --restart-cmd none   # none=复用现有 SITL（注意参数会跨 case 泄漏）
```

产物：

```text
logs/df200_cases/<case>_<NNN>/
  run.log        # 全程脚本输出（STATUSTEXT 时间线）
  flight.BIN     # DataFlash 日志拷贝
  result.json    # case/params/milestones[]/status/log_bin
logs/df200_cases/summary_<ts>.md   # 矩阵汇总表
```

## 9.4 辅助脚本：`df200_mission_run.py`（手动科目执行器）

定位：**SITL 已在跑**时的轻量任务执行（配合 QGC 或人工观察），case_run.py 是它的"全自动矩阵"版。

```bash
# hover profile：GUIDED→MAV_CMD_NAV_TAKEOFF→QLOITER 悬停 --hold 秒→QLAND→disarm
python3 docs/scripts/df200_mission_run.py --profile hover --alt 20 --hold 20

# auto profile：上传 mission→AUTO→arm→监控到 disarm（--param 可先注入拉偏）
python3 docs/scripts/df200_mission_run.py --profile auto \
    Tools/autotest/Generic_Missions/DF200_brake.txt --param SIM_MASS_SCL=1.15
```

细节：连接优先 5762（5760 常被 QGC 占用）；mission 上传后**下载回读校验**；hover profile 用 `MAV_CMD_NAV_TAKEOFF`（GUIDED 下走 `QuadPlane::do_user_takeoff`），QLOITER 属 `in_vtol_mode()` 不触发 FW 转换状态机。

## 9.5 矩阵执行记录（截至 09-29）

`logs/df200_cases/` 现存档案：

| case | 尝试 | 最终状态 | 里程碑数 | 备注 |
|------|------|---------|---------|------|
| cd0+30 | 2 | **PASS** | 14 | 首个全绿 case |
| cla-20 | 3 | **PASS** | 14 | 前 2 次 FAIL 为 param 注入时序（旧版脚本） |
| cla+20 | 2 | 全链飞完 | 14 | result.json 状态字段为 RUNNING（旧版脚本无 PASS 判定），飞行本身成功（Hit ground 0.50 m/s） |
| baseline | 4 | 全链飞完 | 14 | 同上；baseline_004 全程：takeoff→转换(70.4s reached)→FW done→position1→Overshoot→position2→Land final→363s 触地 0.50 m/s→369s Land complete+disarm |

早期 FAIL 模式与教训（已修复/规避）：
- `FAIL: param inject`（cd0+30_001/cla-20_001）：SITL 刚启动串口服务未就绪——现版 `connect()` 加真心跳校验 + 跨端口重试解决；
- `FAIL: upload`（baseline_002）：mission 上传握手中断——现版逐条 ack 校验 + 重试；
- 状态字段 `RUNNING` 残留：旧版脚本缺 `ok_end` PASS 判定（`df200_case_run.py:273-282` 现版已加），按现行代码逻辑复算这些 case 均应为 PASS，飞行日志本身无异常。

## 9.6 与模型对齐工作的关系（baseline 口径）

拉偏矩阵的"baseline"自 2026-09-21 起指向**新物理配置**：`df200.json` Excel 真实几何（惯量 [783.6,464.3,1052.3]、倾转桨 Z −0.409/−0.944）+ 桨表推力模型 + 真实 `position%gyro` 入流（`ed2e650` 关闭 zero-gyro 诊断开关，Run#2 验证 |pqr|<1.5 rad/s）。意义：

1. 拉偏出来的敏感度是相对"物理正确"的基准，而非叠加了诊断性阉割的旧基准；
2. 过渡中段升降舵配平 −1000 N·m 级 z·Fx 项成为 baseline 常态——后续若某拉偏 case 在同段 FAIL，先对照 baseline_004 的 Elev/My 水平再下结论；
3. 待办：反扭物理化（`motor*_yaw=±9` → P/Ω）落地后 baseline 需再刷新一次，届时矩阵整体重跑。

---

# 10. 剩余工作（更新于 2026-09-29）

| 项 | 优先级 | 说明 |
|----|--------|------|
| 矩阵全量跑批 | 高 | CASE_LIB 12 case 中 3 个已有全链记录；`--matrix baseline cla+20 cla-20 cd0+30 ind+30 cma-25 ele-30 ail-30 rud-30 cgx+8cm mass+15 inertia+30` 一键补全（约 30-60 min） |
| 工作区提交 | 高 | C++ 参数组（SIM_Plane/SIM_Frame/SITL.*）、mission 5 件套、脚本三件、`SIM_MotorTable.h` 均未提交；跑批前固化，避免环境漂移 |
| summary 汇总表接入指标摘要 | 中 | 对每个 flight.BIN 批量跑 `df200_transition_log.py`，把掉高/姿态摘要并进 result.json（§6 步骤 6） |
| 风场维度 case | 中 | CASE_LIB 尚无 `SIM_WIND_SPD/DIR/TURB` 组合 case，格式已支持（params dict 任意键） |
| autotest.py TestSuite 路线 | 低 | 独立脚本已满足需求；若需进 CI（autotest 体系）再补 `Tools/autotest/df200.py` |
| baseline 刷新联动 | 低 | 反扭物理化落地后重跑 baseline + 全矩阵 |
