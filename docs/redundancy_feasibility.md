# 基于 ArduPilot 的任务对齐三余度飞控系统可行性调研

> 创建：2026-08-06　状态：调研完成（未开始实现）；2026-08-06 更新：任务对齐明确为**屏障同步**（§3.5），新增 APM 多频率调度机制说明（§2.5）
> 前置工作：PX4 v1.15.3 双余度飞控（主从心跳 + 故障接管 + AutoLand）已实现并通过 HIL 验证

---

## 0. 参考文档索引

本调研基于已有的 PX4 双余度项目文档与四篇学位论文笔记，均在本机：

| 文档 | 路径 | 内容 |
|---|---|---|
| PX4 双余度主设计文档 | `/home/codyqin/Desktop/git/PX4-Autopilot/docs/dual_redundancy_fc.md` | 心跳链路 + 从机接管的完整设计与实现状态 |
| 任务级同步讨论稿 | `/home/codyqin/Desktop/git/PX4-Autopilot/docs/dual_redundancy_task_sync.md` | frame_id + timestamp + 假活检测；三余度扩展的同步地基论证（§6） |
| HIL 测试台架设计 | `/home/codyqin/Desktop/git/PX4-Autopilot/docs/dual_redundancy_testbench.md` | 单 jMAVSim + hil_router 双飞控 HIL 拓扑、T1-T8 测试路线 |
| 论文笔记·邵海龙 | `/home/codyqin/Desktop/git/PX4-Autopilot/docs/paper_notes_shao_hailong.md` | 三重互补故障检测协议（心跳+状态机+CFL）、任务级同步、优先级轮换 |
| 论文笔记·邓泽勇 | `/home/codyqin/Desktop/git/PX4-Autopilot/docs/paper_notes_deng_zeyong.md` | FMECA 驱动设计、心跳帧携带优先级队列、比较量筛选（剔除积分项）、阈值推导、150ms 切换实测 |
| 论文笔记·白雪琛 | `/home/codyqin/Desktop/git/PX4-Autopilot/docs/paper_notes_bai_xuechen.md` | 故障分级 + 置信度/健康度/优先级加权仲裁、选通优先于融合、仲裁器单点防护 |
| 论文笔记·蒋红阳 | `/home/codyqin/Desktop/git/PX4-Autopilot/docs/paper_notes_jiang_hongyang.md` | 三套飞控 + 分层容错结构、故障注入对照实验方法 |
| 论文原文 PDF | `/home/codyqin/Desktop/git/PX4-Autopilot/redudency_paper/` | 上述四篇论文 PDF |
| PX4 项目说明 | `/home/codyqin/Desktop/git/PX4-Autopilot/CLAUDE.md` | PX4 项目编译/HIL 拓扑/参数速查 |

---

## 1. 背景与目标

PX4 项目已完成**双余度主从接管**（心跳计数器 → 超时判死 → 从机强制 arm → AutoLand），并规划了任务级同步（frame_id/timestamp/假活检测）作为三余度表决的地基。

本调研回答一个问题：**同样的余度体系能否在 ArduPilot 上合理地实现，尤其是"任务对齐"的三余度（或更多余度）系统？** 结论先行：**可行，且任务对齐部分比 PX4 更顺；但输出仲裁与模块挂载方式必须按 APM 架构重新设计**（详见 §2、§3.4）。

---

## 2. APM 与 PX4 的关键架构差异（设计前提）

| 维度 | PX4 | ArduPilot |
|---|---|---|
| 进程模型 | **分布式**：模块独立任务，uORB 发布/订阅解耦 | **单体式**：单进程单主循环，库间直接函数调用 + 单例（`AP::ahrs()` 等） |
| 传感器冗余 | 驱动层独立 voter/data validator，实时投票选最佳传感器 | **无独立投票模块**：多 IMU 一致性仅 pre-arm 检查（`AP_InertialSensor::gyros_consistent`），运行时冗余全在 EKF3 多核 lane 切换 |
| 输出门控 | 单点：commander 发布 `actuator_armed`，MixingOutput 消费 | **分散三处**：`AP_Arming::armed` / `AP_Motors::_armed` / `hal.util->soft_armed`，收口点在载具 `AP_Arming` 子类 |
| disarmed 时 ESC 输出 | 从机静默（不发 RawCommand） | **持续广播零值 RawCommand，不静默**（`libraries/AP_DroneCAN/AP_DroneCAN.cpp:849-853`） |
| 冗余管理器挂载方式 | 独立模块旁挂，不碰主链路 | 无中间件可挂，必须嵌入载具代码（scheduler 任务 + DroneCAN 回调 + arming 钩子） |
| CAN 协议栈 | libuavcan（C++，UAVCAN v0） | libcanard（C，DroneCAN），自定义 DSDL 机制同构 |

**两个推论**：

1. APM 单进程意味着**单板软件崩溃 = 该板所有余度机制（EKF lane、failsafe、传感器切换）同时死**，板级余度是防软件共模崩溃的唯一手段——在 APM 上做板级余度的动机比 PX4 更硬。
2. APM 单循环对**任务对齐反而是优势**：loop 边界就是任务边界，一次帧快照内 armed/mode/home/mission_seq 全部来自同一轮解算，天然原子，不存在 PX4 分布式下"逐字段随到随用导致撕裂读取"的问题（见 PX4 侧 `dual_redundancy_task_sync.md` §1.2 担心的问题）。进一步地，APM 主循环本来就阻塞在传感器采样等待上（§2.5），是天然带屏障锚点的架构。

---

## 2.5 APM 的多频率任务调度机制（与 PX4 的 work queue 对比）

PX4 是**多 work queue**：不同频率的模块跑在不同线程/队列（rate_ctrl ~2kHz、attitude ~400Hz、navigation ~200Hz 等）。APM 相反：**所有主任务在同一个主循环里，不同频率靠 tick 整数分频实现**。理解这套机制是理解屏障同步挂载点的前提。

### 节拍来源：主循环阻塞在 IMU 采样上

`AP_Scheduler::loop()`（`libraries/AP_Scheduler/AP_Scheduler.cpp:342-347`）每次迭代开头：

```cpp
void AP_Scheduler::loop() {
    AP::ins().wait_for_sample();   // 阻塞等 INS 新采样，主循环节奏由传感器驱动
    ...
    tick();                        // 计数器 +1
    run(time_available);           // 扫描整张任务表，跑到期的任务
}
```

主循环频率由参数 `SCHED_LOOP_RATE` 决定（`AP_Scheduler.cpp:68`）：Copter/Sub 默认 400Hz，Plane/Rover 默认 50Hz，钳位 50–2000Hz，重启生效。

### 分频机制：rate_hz 是主循环的整数分频比，不是独立时钟

`AP_Scheduler::run()`（`AP_Scheduler.cpp:226-235`）对每个任务：

```cpp
const uint16_t dt = _tick_counter - _last_run[i];        // 距上次运行过了几个 tick
uint32_t interval_ticks = _loop_rate_hz / task.rate_hz;  // 整数除法，向下取整
if (dt < interval_ticks) continue;                       // 未到期，跳过
```

以 Copter（400Hz，周期 2.5ms）任务表（`ArduCopter/Copter.cpp:112-262`）为例：

| 任务 | 声明 rate_hz | interval_ticks | 实际频率 |
|---|---|---|---|
| FAST_TASK 组（INS/姿态控制/电机输出/EKF） | 每 tick 必跑 | 1 | 400Hz |
| rc_loop | 250 | 400/250=1（截断） | 实际 400Hz |
| GPS update | 50 | 8 | 50Hz |
| update_batt_compass | 10 | 40 | 10Hz |
| three_hz_loop | 3 | 133 | 3.0075Hz |
| one_hz_loop | 1 | 400 | 1Hz |
| scheduler 日志 | 0.1 | 4000 | 0.1Hz |

注意两点：① 声明频率被向下取整且只能 ≤ LOOP_RATE（250Hz 在 400Hz loop 下实际跑 400Hz）；② 所有频率共享同一 tick 计数，**一个 loop 边界天然管辖所有周期的任务**。

### 任务分类与预算

- **FAST_TASK 区**：优先级槽位 0–2 保留（`MAX_FAST_TASK_PRIORITIES=3`，`AP_Scheduler.h:100-103`），每 tick 必跑、不分频、不受时间预算门控——普通任务优先级从 3 开始（Copter 表中 rc_loop 优先级 3）。
- **时间预算**：同一 tick 内按优先级升序跑，`time_available = 循环周期 − 已耗时`，预算不足的任务跳过本 tick（`AP_Scheduler.cpp:249-253`）；任务持续超时触发 `extra_loop_us` 自适应扩容（每 tick +100µs，上限 5ms，`AP_Scheduler.cpp:400-403`）——**主循环整体变慢来保低频任务不丢拍**。
- **双表合并**：载具任务表与 `AP_Vehicle` 公共任务表按优先级交错合并（`AP_Scheduler.cpp:190-223`）。
- **主循环之外**：只有 OS 线程和中断——INS 后端采样线程、**DroneCAN 线程**（`AP_DroneCAN.cpp:503` `thread_create`）、RC 输入、UART 中断等。这些不受主循环阻塞影响（§3.5 屏障方案的安全性基础）。

### 对屏障同步的意义

APM 不需要回答 PX4 式的问题"屏障挂在哪个 work queue 上"——只有一个挂载点（`AP_Scheduler::loop()` 头部），且该点本来就是阻塞等待状态。屏障放行后，从 400Hz 姿态环到 0.1Hz 日志任务被同一个对齐信号统一触发。

---

## 3. APM 侧关键钩子盘点（已逐项核实）

### 3.1 心跳/状态通道（对应 PX4 的 HeartbeatCounter + FcStatus）——直接可行

- 自定义 DSDL 放 `libraries/AP_DroneCAN/dsdl/`（ardupilot 命名空间已存在），顶层 `wscript` 的 dronecangen 自动收集生成 C 代码，发布用 `canardBroadcast`。机制与 PX4 侧 libuavcan DSDL 方案同构。
- 现成先例：APM 已周期广播 `ArmingStatus`（`libraries/AP_DroneCAN/AP_DroneCAN.cpp:1257-1261`），AP_Periph 收到后直接 `hal.util->set_soft_armed()` 跟随（`Tools/AP_Periph/can.cpp:517-524`）——**跨节点同步 armed 在 APM 生态里是标准通道**。
- 节点 ID 由 CAN 参数配置，双板设不同 node id 即可。

### 3.2 调度器挂点（对应 PX4 的 redundancy_manager 10Hz 任务）——直接可行

- 载具任务表：`ArduCopter/Copter.cpp:112`（`scheduler_tasks[]`）、`ArduPlane/ArduPlane.cpp:56`。
- 加一行 `SCHED_TASK_CLASS(Redundancy, &redundancy, update, 10, 100, <优先级>)` 即可。主循环频率由 `SCHED_LOOP_RATE` 控制（Copter 默认 400Hz、Plane 50Hz），任务节拍精确确定——**这正是"帧号"的天然来源**（loop 计数或 10Hz tick 计数即 frame_id）。

### 3.3 接管 override（对应 PX4 的 actuator_armed override）——可行，先例比 PX4 更干净

- **强制 arm 通道现成**：`AP_Arming::arm(method, false)` 跳过全部预检（`libraries/AP_Arming/AP_Arming.h:98-101`）。
- **运行时强制 arm 先例**：Plane 的 BlackBox 模式在飞行中自动 `arm(Method::BLACKBOX, false)` / `disarm`（`ArduPlane/AP_Arming.cpp:411-429`）——官方支持不经预检的运行时强制接管，冗余接管可完全模仿，新增 `Method::REDUNDANCY` 枚举即可。
- 三个 armed 布尔量的唯一收口点是载具 `AP_Arming` 子类的 arm/disarm（如 `ArduCopter/AP_Arming.cpp:668/781` 同时驱动三者），接管钩子加在这里。
- 接管后切模式：`AP_Vehicle::set_mode(RTL/LAND, ModeReason::...)`；disarmed 下模式检查全跳过（`ArduCopter/mode.cpp:290`），路径已被各 failsafe 验证。`libraries/AP_Vehicle/ModeReason.h` 需新增一个 redundancy reason。

### 3.4 ⚠️ APM 特有的坑：从机不静默（必须解决）

PX4 方案靠"从机 disarmed → 不发 RawCommand"实现天然无冲突仲裁；**APM 的 `SRV_send_esc()` 在 disarmed 时仍持续广播零值指令**（`libraries/AP_DroneCAN/AP_DroneCAN.cpp:824-871`，第 849-853 行：armed 发真值，否则发 0，但消息照发）。热备从机会与主机持续抢 ESC（last-writer-wins），接管语义被破坏。两条修法：

1. `SRV_Channels::set_emergency_stop()`（`libraries/SRV_Channel/SRV_Channels.cpp:582`）在 calc_pwm 层掐断从机输出——改动小，但 DroneCAN 层仍可能发帧，需实测确认；
2. 在 `SRV_send_esc()` 加 standby 时整帧不发的补丁——更干净，语义上等价 PX4 的静默（**推荐**）。

### 3.5 任务对齐：屏障同步方案（barrier synchronization）

目标形态是**真正的屏障同步**（邵海龙论文的任务级同步）：各通道跑到任务边界停住，等对齐信号到齐，再触发当期全部工作——对齐信号是**因**（触发当期工作），不是事后打的帧号标签。

#### 3.5.1 为什么 APM 能做、PX4 不能

PX4 侧讨论稿（`dual_redundancy_task_sync.md` §0）论证过屏障搬不进 PX4：各模块是独立事件驱动任务、没有统一屏障平面、任何控制环阻塞等对端都会引入抖动。APM 的三个结构特性改变了判断：

1. **主循环本来就阻塞等待**：`AP_Scheduler::loop()` 开头就是 `wait_for_sample()`（§2.5），任务边界天然存在，且主循环习惯被同步事件阻塞——把 INS 等待扩展为"INS 采样 + 对齐信号"双条件即可，改动集中在一个函数；
2. **单循环分频模型**（§2.5）：一个 loop 边界管辖所有频率的任务，屏障放行后 400Hz 姿态环到 0.1Hz 日志被同一个对齐信号统一触发，不存在"屏障挂哪个队列"的问题；
3. **等齐期间系统不瞎**：心跳收发在 DroneCAN 线程（`AP_DroneCAN.cpp:503`，独立于主循环）、IMU 采样在 INS 后端线程、看门狗在主循环每迭代喂狗（`HAL_ChibiOS_Class.cpp:316`）。某机死锁在屏障上 → 看门狗复位 → 心跳消失 → 被对端判死出局。**同步机制本身兼任故障检测**，正是邵海龙论文 SYNC 屏障的机制（等不到 = 超时 = 故障信号）。

#### 3.5.2 核心原则：分离"计算"与"比较"（高频任务与低频屏障的矛盾解法）

**矛盾**：姿态环等 FAST_TASK 每 tick 都跑（400Hz），不能等屏障；但其输出要参与表决。若标"入屏障"则控制环被 10Hz 卡死，若不标则表决的不是同一时刻的值。

**解法**：把"计算频率"与"比较频率"分开。**屏障不拦计算，拦的是比较时刻**：

```
通道 A: ahrs(400Hz 自由跑) ─┐
通道 B: ahrs(400Hz 自由跑) ─┼─→ 屏障到达时各采样当前姿态/位置/速度值
通道 C: ahrs(400Hz 自由跑) ─┘         ↓
                              交换 ALIGN 帧 → 按轮次表决
```

高频任务照跑，屏障到达时各通道把**当前值**打包进 ALIGN 帧广播比较。比较的是值，不是进程——对齐的是"比较时刻"，不是"计算过程"。

**表决什么物理量决定屏障结构**（这是设计的第一性问题）：

| 物理量 | 变化速率 | 表决策略 | 依据 |
|---|---|---|---|
| 姿态角/角速率 | 快（ms 级） | **不直接表决**。表决其源头（EKF 状态/传感器源选择），各通道从共识源头自己解算姿态 | 姿态是 EKF 派生量，源头一致则姿态自然收敛 |
| 位置/速度/高度 | 中（100ms 级） | 直接表决 EKF 输出 | 慢变量，10Hz 屏障足够 |
| 控制输出（舵面/电机指令） | 快 | 表决指令值本身（10Hz 采样比较） | 邓泽勇原则：**剔除积分项**——只比不含 I 项累积的瞬时指令量 |
| 模式/armed/mission 进度 | 慢（s 级） | 直接表决（状态一致性比对） | 假活检测即靠它（§3.5.4） |

这与邓泽勇"比较量筛选"原则同源：**比源头状态与瞬时量，不比时间累积量与派生快变量**。

#### 3.5.3 屏障挂载位置（"屏障点"）

AP_Scheduler 是单循环分频模型（§2.5），屏障只有两种挂法：

- **挂 loop 头部**（`AP_Scheduler::loop()` 的 `wait_for_sample()` 处扩展）：实现最简，但**所有任务**（含 GCS/日志）都被等齐卡住，不可取；
- **挂表决任务组入口**（推荐）：在 `AP_Scheduler::run()` 扫描任务表时，于"入屏障"任务的执行入口插入等待。任务表加标志位声明归属：

```cpp
// AP_Scheduler.h —— Task 结构扩展
struct Task {
    task_fn_t function;
    const char *name;
    float rate_hz;
    uint16_t max_time_micros;
    uint8_t priority;
    bool in_barrier;   // 新增：该任务解算的量参与跨通道表决
};
```

`run()` 中：当本期第一个 `in_barrier=true` 且到期的任务出现时，执行 `wait_align_or_timeout()`（等齐或硬超时除名），通过后本轮其余任务不再等待。

以 Plane 任务表（`ArduPlane/ArduPlane.cpp:56-145`）为例的天然分层：

| 任务组 | 屏障归属 | 说明 |
|---|---|---|
| FAST_TASK：`ahrs_update`/`stabilize`/`set_servos` | **不入屏障**，但输出在屏障时刻被采样进 ALIGN 帧 | 高频控制环自由跑；`set_servos` 的指令值作为"控制输出表决量" |
| `update_GPS_10Hz`/`navigate`/`update_alt`/`update_speed_height`/`compass`（优先级 30-48） | **入屏障**（`in_barrier=true`） | 解算位置/速度/高度等表决核心量，屏障到达才跑，保证三通道同输入同轮次 |
| GCS/日志/`one_second_loop`（优先级 57+） | 不入屏障 | 纯本地工作，等对端时照常跑 |

10Hz 屏障在分频模型下是自然值：`interval_ticks = 400/10 = 40`（Copter）或 `50/10 = 5`（Plane），对齐帧号可直接用 `_tick_counter/interval`。

#### 3.5.4 屏障流程（每个对齐周期）

```
1. wait_for_sample()                     ← 原有：等 IMU 新采样
2. 屏障到达（10Hz tick 边界）：
   采样本通道当前值：EKF 位置/速度、姿态摘要、控制指令、mode/armed/mission_seq
   → 打包 ALIGN 帧（轮次号 + 时间戳 + 上述量）广播
3. 等齐条件：收齐对齐组全部通道本轮 ALIGN 帧
   ├─ 到齐      → 表决（位置/速度多数裁决；状态一致性比对）→ 共识结果写回
   ├─ 等待超时  → 该通道从对齐组除名，剩余通道继续（降级 N→N-1）
   └─ 只剩自己  → 单机自主运行，继续广播 ALIGN 等重新入组
4. 放行 in_barrier 任务（navigate/update_alt 等），用共识后的状态解算
5. FAST_TASK 不受影响，持续自由运行（用共识后的传感器源/状态）
```

#### 3.5.5 设计决策

| 决策 | 选项 | 建议 |
|---|---|---|
| **节拍来源** | A. leader 采样即广播 ALIGN，从机跟随；B. 各机独立采样只对齐开工时刻 | **A**：同输入同节拍，表决意义完整；leader 失效按优先级队列递补（复用邓泽勇的队列共识，见 §4） |
| **屏障周期** | 400Hz 主循环 vs. 低频屏障 | **10Hz 屏障 + 内环自由运行**：CAN 来回毫秒级，400Hz 全屏障的等待开销占比过大；对应论文的"任务周期"而非"控制周期"。快变量靠源头共识（§3.5.2），不需要高频屏障 |
| **屏障覆盖范围** | 全部任务 vs. 仅表决相关 | **仅表决相关**：`in_barrier` 标志声明，导航/估计类任务入屏障，GCS/日志留屏障外（§3.5.3） |

#### 3.5.6 降级形态与软对齐的关系

屏障不可用（对端全灭、CAN 断）时自动降级为**单机自主 + 帧号软对齐**：frame_id（tick 计数）+ timestamp（`AP_HAL::micros64()`）继续广播，供后续恢复入组与假活检测用。假活检测逻辑（心跳活但任务帧停滞）从 PX4 讨论稿（`dual_redundancy_task_sync.md` §4）平移，只把 uORB/DSDL 换成 libcanard 等价物。恢复信息同步（armed/nav_state/home/mission_seq）扩展 FcStatus 类 DSDL 字段即可；APM 的 mission 在 `AP_Mission` 单例中，取 seq 无障碍。

#### 3.5.7 风险

- 动主循环时序属核心路径改动，必须加独立参数开关（默认关闭），且只在显式配置为"对齐组"成员时激活；
- 屏障等待上限必须有硬超时（≤ 一个对齐周期），超时即除名，绝不允许无限等待；
- SITL 验证需双/三实例 + 网络互联（vcan 或 UDP 链路模拟 CAN），再上硬件；
- leader 切换瞬间的节拍抖动需实测（建议 leader 交接时从机以本地节拍续跑一拍再入新组）。

---

## 4. 三余度表决的可行性

- **APM 板内没有任何输出表决设施**（连 PX4 那种传感器 voter 都没有，EKF lane 切换是估计器级冗余，形态不同），三通道输出表决必须从零写。
- 可行形态：三块飞控帧对齐后，把**非积分量**（姿态角/角速率/位置）+ 舵面/电机指令打包进自定义 DSDL 广播；表决器有两个选择：
  - 三机互为表决方（每机收齐另两机同轮次帧后本地表决，多数者输出）；
  - **独立表决节点：用 AP_Periph 改造**——AP_Periph 本就是 APM 的轻量 CAN 外设固件生态位，对应白雪琛论文 CPLD 仲裁器的软件化版本，且天然隔离"表决器单点故障与飞控故障"。
- 可直接引用的工程参数（邓泽勇论文）：比较量**剔除积分项**（PID I 项、EKF 偏差估计绝不参与比较，松散同步下必然发散误判）；阈值取解算结果的 **5~8%**；滑窗防抖（N 拍内 m 次超限才判故障）；切换时间目标 **150ms**。
- 三余度共识机制：心跳/FcStatus 帧中携带**完整优先级队列**（主控/第一从/第二从），三机各自维护同一队列、故障者移队尾——这是邓泽勇方案中三通道收敛的关键，扩展自定义 DSDL 时预留该字段。
- 接管后**不自动回切**（latch），故障机恢复后以从控身份归队——PX4 项目 `_failover_latched` 的做法与邵海龙论文一致，APM 沿用。

---

## 5. 风险与开放问题

| # | 风险 | 说明 |
|---|---|---|
| 1 | CAN 总线单点 | APM 无独立硬线 CFL；缓解：双 CAN 口双发（板级支持视具体飞控板），文档中需明示残余风险 |
| 2 | 接管绕过 pre-arm | 强制 arm 跳过健康检查，需确保从机自身传感器健康（从机 EKF 应常时融合 GPS，PX4 项目已验证该做法） |
| 3 | SITL 验证边界 | DroneCAN 路径需硬件或 Linux socketcan（vcan）验证；SITL 可验证调度/状态机逻辑，CAN 收发待确认（APM SITL 对 DroneCAN 的仿真支持比 PX4 的 NuttX-only 限制宽松，待实测） |
| 4 | 三通道"下游听谁的" | 双机靠从机静默即可；三机表决后需明确执行器接收策略（表决器统一输出 or 多数指令 last-writer-wins），在阶段 4 前定型 |
| 5 | 共性设计故障 | 三机同固件同编译 → 软件共模故障无法靠表决消除；邓泽勇的"非相似开发环境"思想在开源单固件场景只能部分缓解（参数差异化、不同载具配置），接受此残余风险 |

---

## 6. 实施路线（分阶段）

| 阶段 | 内容 | 关键改动 | 验证方式 |
|---|---|---|---|
| P1 | 双板心跳链路 | 自定义 DSDL（HeartbeatCounter）+ scheduler 10Hz 任务 + 超时检测 | 双板 CAN 互联，断主机验证超时判定 |
| P2 | 从机接管 | `Method::REDUNDANCY` 强制 arm + `SRV_send_esc` 静默补丁 + set_mode 切 RTL/Land + 接管 latch | HIL 台架（沿用 `dual_redundancy_testbench.md` 的 router 拓扑思路，换成 APM SITL/硬件） |
| P3 | 任务对齐与假活检测 | FcStatus 帧加 frame_id/timestamp/arming_state；从机节拍消费 + 缺帧假活判定 | 故障注入：停任务帧不停心跳，验证识破 |
| P4 | 三余度表决 | 三机互发非积分量 + 指令；AP_Periph 表决器（或本地三取二）；优先级队列共识 | 按邓泽勇方法做简化 FMEA 表驱动故障注入用例 |

测试方法学：先按邓泽勇论文做一张简化 FMEA 表（故障模式→现象→检测手段→处置），直接产出故障注入用例表，避免拍脑袋列用例；对照实验方法参考蒋红阳论文（剪断/干扰注入，单余度 vs 多余度对照）。

---

## 7. 预计涉及的文件（APM 侧）

| 文件 | 改动类型 |
|---|---|
| `libraries/AP_DroneCAN/dsdl/`（新增 RedundancyHeartbeat/FcStatus 等 .uavcan） | 新增 |
| `libraries/AP_DroneCAN/AP_DroneCAN.{h,cpp}` | 新增 pub/sub 与回调 |
| `libraries/AP_DroneCAN/AP_DroneCAN.cpp`（`SRV_send_esc`） | 改：standby 静默 |
| 新库 `libraries/AP_Redundancy/`（或并入 AP_DroneCAN） | 新增：冗余管理器状态机 |
| `ArduCopter/AP_Arming.cpp`（或对应载具） | 改：接管 arm/disarm 钩子 |
| `libraries/AP_Arming/AP_Arming.h` | 改：`Method::REDUNDANCY` |
| `libraries/AP_Vehicle/ModeReason.h` | 改：新增 redundancy reason |
| `ArduCopter/scheduler.cpp` 等载具任务表 | 改：加 10Hz 任务 |
| `Tools/AP_Periph/`（阶段 P4） | 改：表决器固件 |
