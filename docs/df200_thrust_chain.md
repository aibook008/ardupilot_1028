# DF-200 推力链路详解：从 PID 输出到牛顿（thrust→actuator→PWM→cmd→表）

> 配套文档：`docs/df200_session_20260902.md`（本轮攻关记录）、`docs/df200_motor_thrust_derivation.md`（表模型推导）
> 代码版本：Plane 4.6.x（qhz/plane_4.6.3 分支），所有行号已实际验证（2026-09-02）
> 适用读者：需要理解"为什么改电机模型必须同步 Q_M_THST_EXPO/HOVER"、或要复用这套表模型机制的人

---

## 1. 全链总览：两个 0~1 域，中间一次翻译

这条链上有**两个 [0,1] 归一化值**，名字都像"油门"，但物理含义完全不同：

| | `thrust`（推力域） | `actuator`（油门域） |
|---|---|---|
| 单位含义 | 推力占满推力的比例 | 电机 PWM 归一化位置 |
| 谁产生 | PID 控制律 | 推力曲线翻译 |
| 对应物理 | 我要多少牛顿 | 我把电机开到多大 |
| 数学角色 | 推力曲线的**纵轴** | 推力曲线的**横轴** |

```text
PID输出 thrust ──①词典──> actuator ──②线性──> PWM ──③线性──> cmd ──④表──> 牛顿
  [0,1]        EXPO曲线      [0,1]     PWM范围    [1000,2000]  spin映射   [0,1]   查表插值
```

| 段 | 代码位置 | 线性？ |
|---|---|---|
| ① thrust→actuator | `libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp:101-106`（入口）+ `:120-136`（曲线） | **非线性（EXPO 曲线），全链唯一弯段** |
| ② actuator→PWM | `libraries/AP_Motors/AP_MotorsMulticopter.cpp:410-421`（`output_to_pwm`） | **线性**：`pwm = pwm_min + (pwm_max−pwm_min)×actuator` |
| ③ PWM→cmd | `libraries/SITL/SIM_Motor.cpp:377-386`（`pwm_to_command`） | **线性**：`cmd = (pwm − pwm_thrust_min)/range` |
| ④ cmd→推力 | `libraries/SITL/SIM_Motor.cpp:368-385`（`calc_thrust_table`） | 查表（实测曲线，近二次） |

**两段线性搬运（②③）只负责传话；①④ 互为镜像、互相抵消——EXPO 曲线的形状必须与实测表一致，否则每个 PID 指令都被错误翻译。**

---

## 2. 飞控侧调用链（自上而下，每层职责）

### 第 1 层：位置/加速度控制（thrust 域）

`AC_PosControl::update_z_controller()`（`libraries/AC_AttitudeControl/AC_PosControl.cpp:989`）

```cpp
// :1042-1048
thr_out = _pid_accel_z.update_all(_accel_target.z, z_accel_meas, ...) * 0.001f;  // PID 结果 (推力域)
thr_out += _pid_accel_z.get_ff() * 0.001f;
thr_out += _motors.get_throttle_hover();          // ← ★ HOVER 在这里作为基点加上
_attitude_control.set_throttle_out(thr_out, true, ...);
```

关键理解：**HOVER 不是曲线参数，是纵轴的零点偏置**。它告诉 PID "悬停对应 thrust=0.49"，让积分器不用从 0 爬起来。悬停时 PID 只输出 ±0.0x 的微调。

另外 `:1034-1037`：IMAX 自动抬高到至少 `hover×1000`，保证积分器有足够权限覆盖悬停油门——HOVER 错了这里也跟着错。

### 第 2 层：姿态控制器（透传 + angle boost）

`AC_AttitudeControl_Multi::set_throttle_out()`（`libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp:350-368`）

```cpp
_throttle_in = throttle_in;                          // :352 存底 (init_z_controller 回读用)
throttle_in = get_throttle_boosted(throttle_in);     // :358 angle boost (姿态倾斜时补油门)
_motors.set_throttle(throttle_in);                   // :366 进电机库, 仍是 thrust 域
_motors.set_throttle_avg_max(...);                   // :367 给混控器留 RPY 余量
```

此层**不做任何推力曲线翻译**，只做角度补偿后透传。

### 第 3 层：电机库混控（thrust 域完成全部加法）

每 400Hz：`AP_MotorsMulticopter::output()`（`libraries/AP_Motors/AP_MotorsMulticopter.cpp:290`）
→ `output_armed_stabilizing()`（`libraries/AP_Motors/AP_MotorsMatrix.cpp:242`）

```cpp
// :257  throttle thrust input value, 0.0 - 1.0
float throttle_thrust = get_throttle() * compensation_gain;
// :313-427  roll/pitch/yaw 差动分配 + 饱和缩放 (rpy_scale/yaw headroom)
_thrust_rpyt_out[i] = (best_throttle × _throttle_factor[i]) + (rpy_scale × 差动);
```

**整个混控都在 thrust 域做线性加法**（每电机 +0.1 thrust = +0.1×T_max 牛顿）。物理上推力可加、油门不可加（+0.1 油门在低/高端加的牛顿数不同），所以混控器必须先在推力域算完，最后统一翻译——这就是曲线必须存在于混控之后而非之前的结构性原因。

### 第 4 层：推力曲线词典（EXPO/HOVER 唯一参与点）

`AP_MotorsMatrix::output_to_motors()`（`libraries/AP_Motors/AP_MotorsMatrix.cpp:147`，:170-172）

```cpp
set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]));
```

`Thrust_Linearization::thrust_to_actuator()`（`libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp:101-106`）

```cpp
thrust_in = constrain_float(thrust_in, 0.0, 1.0);
return spin_min + (spin_max - spin_min) * apply_thrust_curve_and_volt_scaling(thrust_in);
```

`apply_thrust_curve_and_volt_scaling()`（同文件 ：120-136，核心式在 ：128）

```cpp
// EXPO≠0 时:
throttle_ratio = ((e−1) + safe_sqrt((1−e)² + 4e·lift_max·thrust)) / (2e)
```

这个公式看着复杂，**本质是二次方程求根公式**（推导见 §4）。

### 输出末端：actuator → PWM（线性，你的判断正确）

`AP_MotorsMulticopter::output_to_pwm()`（`libraries/AP_Motors/AP_MotorsMulticopter.cpp:410-421`）

```cpp
pwm_output = get_pwm_output_min() + (get_pwm_output_max() - get_pwm_output_min()) * actuator;
```

DF-200（`Tools/autotest/models/df200.json:39-42`：pwmMin=1000, pwmMax=2000, spin_min=0.15, spin_max=0.95）下：

```text
PWM = 1000 + 1000×actuator                      [纯线性]
```

---

## 3. SITL 侧：PWM → cmd → 表 → 牛顿

### PWM→cmd（线性，含 spin 死区）

`Motor::pwm_to_command()`（`libraries/SITL/SIM_Motor.cpp:377-386`）：

```text
pwm_thrust_min = 1000 + 0.15×1000 = 1150    (spin_min=0.15)
pwm_thrust_max = 1000 + 0.95×1000 = 1950    (spin_max=0.95)
cmd = (pwm − 1150)/800, clamp [0,1]
```

与飞控端串联：`cmd = 1.25×actuator − 0.1875`（actuator<0.15 时 cmd=0，桨不转）。

### cmd→RPM（线性假设）

`SIM_Motor.cpp:73-76`：

```cpp
last_rpm = command * (thrust_model == TABLE_DF200_OUTER_60IN ? 3181.0f : 4200.0f);
```

**注意：cmd→RPM 是开环线性假设**——没有电机 KV、电压跌落、转速惯量模型。这是表模型机制的理论支点，也是它的边界（§7）。

### RPM→牛顿（查表）

`Motor::calc_thrust_table()`（`libraries/SITL/SIM_Motor.cpp:368-385`）：

```cpp
thrust = DF200MotorTable::prop60_thrust(last_rpm);            // 外4桨, 一维表
thrust = DF200MotorTable::prop40_thrust(last_rpm, velocity_in); // 内4桨, 二维表(RPM×风速)
return MAX(0.0f, thrust) * (air_density / 1.225f);
```

表数据/插值细节：`libraries/SITL/SIM_MotorTable.h`（一维插值 ：106-127，二维 ：179-200；表下限 T∝RPM² 外推保证平滑归零，上限钳位，风速钳 [0,50]）。数据源 `docs/电机电池.docx`。

---

## 4. EXPO 曲线的数学来源（求根公式推导）

### 4.1 词典曲线的定义

飞控假设的推力模型（与 SITL 旧动量模型的 Vout 公式同源）：

```text
T(cmd) = (1−e)·cmd + e·cmd²      ← e = THST_EXPO
```

从纯线性（e=0）到纯平方（e=1）的形状插值：

| e | T(cmd) | 物理假设 |
|---|---|---|
| 0 | cmd | 推力∝油门（理想恒距桨低速近似） |
| 0.5 | 0.5cmd + 0.5cmd² | 中间形状 |
| 1 | cmd² | 推力∝转速²，转速∝油门（典型桨） |

### 4.2 反函数推导

控制律工作在 thrust 域（已知 T），混控器需要 cmd——**反解二次方程**：

```text
e·cmd² + (1−e)·cmd − thrust = 0
a=e, b=1−e, c=−thrust → 求根公式:
cmd = [−b + √(b²−4ac)] / (2a)
    = [−(1−e) + √((1−e)² + 4e·thrust)] / (2e)
```

与代码逐项对上：

| 求根公式 | 代码（:128） |
|---|---|
| `−b = −(1−e)` | `(thrust_curve_expo - 1.0)` |
| `b² − 4ac` | `(1.0−e)² + 4e·lift_max·thrust`（`−4ac = +4e·thrust`，lift_max 为电压补偿系数，满电=1） |
| `2a` | `(2.0 * thrust_curve_expo)` |

### 4.3 端点验证

```text
e=1: cmd = [0 + √(4·thrust)]/2 = √thrust        ← 退化为根号，最简
e=0: 分母为 0 → 代码 is_zero(e) 分支直接返回 thrust（线性无需解方程，:124-127）
e=0.5, thrust=0.49: cmd = [−0.5+√(0.25+0.98)]/1 = 0.609
    验算: 0.5×0.609 + 0.5×0.609² = 0.49 ✓
```

**一句话**：那行公式不是经验拟合，就是 `T=e·cmd²+(1−e)·cmd` 的解析反函数；e=1 时整坨塌缩为 `√thrust`。

---

## 5. HOVER：学得出来吗？（学习机制与三个坑）

### 5.1 在线学习机制

`AP_MotorsMulticopter::update_throttle_hover()`（`libraries/AP_Motors/AP_MotorsMulticopter.cpp:501-506`）：

```cpp
if (_throttle_hover_learn != HOVER_LEARN_DISABLED) {
    _throttle_hover.set(constrain_float(
        _throttle_hover + (dt/(dt+10.0f)) × (get_throttle() − _throttle_hover),
        0.125, 0.6875));    // TC=10s 一阶趋近实际油门
}
```

原则正确：拿实际输出油门慢慢拉 hover，"一开始不准后面也能改"。**但 DF-200 上学不准，必须手设，三个原因**：

1. **学习源被污染**：它学的是 `get_throttle()`（混控后的油门域值）。若词典（EXPO）形状错，学习器收敛到"能让飞机悬停的错误 cmd"，PID 围绕错值稳定——但推力域与牛顿的映射是歪的，angle boost/rpy_mix/IMAX 全跟歪。
2. **窗口短且有硬上限**：TC=10s；`Q_TRANSITION_MS=32767`（AS 门控）下悬停工况只有 ~25s，每 10s 只走 63% 差距，没收敛就退出旋翼工况。上限 0.6875（本机需要的 0.49 在界内）。
3. **表模型的悬停点是设计值**：实测表给出 T_max=7656N、W=3727N，悬停推力占比 = 3727/7656 = **0.488，解析可算**。手设 0.49 一步到位，避免学习期间积分器错启动。

### 5.2 HOVER 的消费点（改错时谁受害）

| 位置 | 消费方式 |
|---|---|
| `AC_PosControl.cpp:1048` | 加速度环输出基点 `thr_out += hover` |
| `AC_PosControl.cpp:818, 849-851` | 积分器 relax/init 以 `(throttle − hover)` 为基准 |
| `AC_PosControl.cpp:1034-1037` | IMAX 自动 ≥ hover×1000 |
| `AC_AttitudeControl_Multi.cpp:398` | RPY 混控基准 `hover × _throttle_rpy_mix` |
| `ArduPlane/quadplane.cpp:1131-1146` | Q 模式手动油门曲线中位映射 |

---

## 6. DF-200 数值实例（全部数字已用表数据逐点验算）

### 6.1 例 1：悬停点（thrust=0.49）

```text
① 词典: cmd = √0.49 = 0.700            (EXPO=1)
② PWM = 1000 + 1000×0.700 = 1700 µs
③ cmd_sitl = (1700−1150)/800 = 0.688
④ 表: 外60in: RPM=0.688×3181=2188 → 592 N/桨
     内40in: RPM=0.688×4200=2890 → v0分支 = 354 N/桨
     合计 4×(592+354) = 3784 N ≈ W=3727 N (T/W=1.015 ✓)
```

飞控要 49%，物理交付 50.9%——词典对齐后的常态误差（表非严格二次），PID 积分器吃掉。

### 6.2 例 2：满油门（thrust=1.0）

```text
① cmd=1.000 → PWM=2000 → cmd_sitl=1.000
④ 外桨 1379.2 N/桨（表末点）+ 内桨 534.8 N/桨
   合计 4×1914 = 7656 N = T_max ✓（精确自洽，HOVER 定义来源）
```

### 6.3 例 3：低推力（thrust=0.15）——表的非线性显形

```text
① cmd=0.387 → PWM=1387 → cmd_sitl=0.297
④ 外桨 RPM=945 < 表下限 → 二次外推 103 N/桨; 内桨 66 N/桨
   合计 676 N vs 词典预期 1148 N → 只交付 59%
```

低推力区偏差大（spin 死区+二次外推），但仅 spool/着陆瞬间用到，积分器自动补。

### 6.4 全链数值表

| thrust | cmd₁=√t | PWM | cmd_sitl | 外60 N/桨 | 内40 N/桨 | 4桨合计 N | 实际/T_max |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.15 | 0.387 | 1387 | 0.297 | 103 | 66 | 676 | 0.088 |
| 0.25 | 0.500 | 1500 | 0.438 | 300 | 188 | 1952 | 0.255 |
| **0.49（悬停）** | **0.700** | **1700** | **0.688** | **592** | **354** | **3784** | **0.494** |
| 0.70 | 0.837 | 1837 | 0.859 | 946 | 508 | 5816 | 0.760 |
| 1.00 | 1.000 | 2000 | 1.000 | 1379 | 535 | 7656 | 1.000 |

中间区（0.25~0.7，飞行常用区）偏差 <2%：60in 表实测点本身近似落在二次曲线上。**EXPO=1 不是巧合是选择**——表验证了 T∝RPM²，假设 cmd∝RPM 线性，则 T∝cmd²，词典取 e=1（√t）恰好互逆。

### 6.5 反例：175 弹跳的错位账（历史教训的算术本质）

表模型落地时沿用旧参数 EXPO=0.65/HOVER=0.45：

```text
真实悬停 cmd=0.68（表） → 推力 3727N
词典: thrust=0.45 经 EXPO=0.65 翻译 → cmd=0.64
表:   (0.64/0.68)² × W = 86%W → 悬停指令只打出 86% 推力
→ 掉高 → 积分加码 → 过冲 → 18.5Hz 极限环（弹跳）
```

修正 EXPO=1.0/HOVER=0.49（`Tools/autotest/default_params/quadplane-df200.parm:42,158`）后自洽。

---

## 7. 机制边界（这个思路"没做"的事）

1. **cmd→RPM 开环线性**：无电机 KV/电压跌落/转速惯量延迟（真实 ~100ms 级）。同步 Q_M_THST_EXPO/HOVER 补的是"形状对齐"，不补动态。
2. **电压不建模**：表在 398V 标定；飞控侧 batt_voltage_filt 补偿在 SITL 无对应物（voltage_scale=1）。
3. **风速钳位 [0,50]**：俯冲 AS>50 时表到头——好在此时内桨推力需求小。
4. **密度只有线性缩放** ρ/1.225。

## 8. 代码位置速查表

| 功能 | 文件:行号 |
|---|---|
| 词典核心式（thrust→cmd） | `libraries/AP_Motors/AP_Motors_Thrust_Linearization.cpp:120-136`（:128 为求根公式） |
| 词典入口（含 spin 映射） | 同文件 `:101-106`（`thrust_to_actuator`） |
| 反向词典（cmd→thrust） | 同文件 `:138-152`（tailsitter 用，代数对照看） |
| EXPO 参数声明 | 同文件 `:44-45`；`AP_MotorsMulticopter.cpp:47-53`（子组挂载，GCS 显示 Q_M_THST_EXPO） |
| HOVER 参数声明/默认 | `AP_MotorsMulticopter.cpp:133-138`；`AP_MotorsMulticopter.h:10-13`（默认 0.35/上下限 0.125-0.6875） |
| HOVER 在线学习 | `AP_MotorsMulticopter.cpp:501-506` |
| 混控器（thrust 域加法） | `AP_MotorsMatrix.cpp:242-440`（:257 取油门，:427 每电机分配） |
| 词典调用点 | `AP_MotorsMatrix.cpp:147,170-172`（`output_to_motors` → `thrust_to_actuator`） |
| actuator→PWM（线性） | `AP_MotorsMulticopter.cpp:410-421`（`output_to_pwm`，:418 核心行） |
| 位置环 HOVER 基点 | `AC_PosControl.cpp:1042-1048`；积分器 init `:818,849-851`；IMAX `:1034-1037` |
| 姿态环透传 | `AC_AttitudeControl_Multi.cpp:350-368`；混控基准 `:395-400` |
| Q 手动油门曲线 | `ArduPlane/quadplane.cpp:1131-1146`；`AP_Math/AP_Math.cpp:131-146`（throttle_curve） |
| SITL 旧动量模型（对照） | `SIM_Motor.cpp:350-357`（`calc_thrust`，平方差结构） |
| SITL cmd→RPM | `SIM_Motor.cpp:73-76`（60in:3181 / 40in:4200） |
| SITL 查表出牛顿 | `SIM_Motor.cpp:368-385`（`calc_thrust_table`） |
| PWM→cmd（含 spin 死区） | `SIM_Motor.cpp:377-386`（`pwm_to_command`） |
| 表数据与插值 | `SIM_MotorTable.h`（一维 :106-127，二维 :179-200） |
| 表模型接线 | `SIM_Frame.cpp:557-567`（df200 → 40IN/60IN） |
| DF-200 参数 | `Tools/autotest/default_params/quadplane-df200.parm:42`（HOVER 0.49）`:158`（EXPO 1.0） |
| PWM 范围/spin | `Tools/autotest/models/df200.json:39-42` |

> **grep 提示**：代码里搜 `Q_M_THST_EXPO` 是搜不到的——参数真名是 `MOT_THST_EXPO`（`AP_GROUPINFO("THST_EXPO", ...)`），QuadPlane 把 AP_Motors 挂为子对象后 GCS 显示加 `Q_M_` 前缀。源码搜索用 `THST_EXPO`/`THST_HOVER`。

## 9. 一图总结

```text
  PID 世界 (thrust 域, "我要多少推力")          电机世界 (cmd 域, "开多大油门")
 ┌────────────────────────────────┐    EXPO曲线    ┌────────────────────┐
 │ accel PID ──┐                  │   ┌─────────┐   │                    │
 │             ├→ thr_out ────────┼──→│ f(t)=   │──→│ _actuator → PWM →  │
 │ (基准=HOVER)│                  │   │ √t (e=1)│   │ SITL 表 → 牛顿     │
 │ rpy 差动 ───┘ (线性可加域)      │   └─────────┘   │ (T∝cmd²)           │
 └────────────────────────────────┘   THST_EXPO 定义 └────────────────────┘
                                        ↑                ↑
                                    曲线形状          曲线真实形状
                                                       (实测表, T∝RPM²)
                                    两边必须同形, 否则每条 PID 指令
                                    都被错误翻译 → 弹跳/掉高

 三层职责: EXPO=形状 | HOVER=锚点(悬停行号) | 表=物理事实(牛顿出口)
```
