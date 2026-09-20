# 倾转舵机 DroneCAN 协议（tilt_uavcan，DTID 2025/2026）集成执行方案

> 面向对象：**舵机供应商**（执行方）+ 飞控方（ArduPilot）。
> 日期：2026-09-18。协议源定义：`modules/DroneCAN/DSDL/tilt_uavcan/`（3 文件），飞控侧已用 dsdlc 试编译通过。
>
> 配套文档（飞控内部参考，供应商可不读）：
> `dsdl_custom_guide.md`（signature 机制）、`uav_can.md`（CAN ID/过滤/负载）、`df200-uavcan.md`（LSB-first 打包先例）。

---

## 0. 方案总览（一句话 + 拓扑图）

**飞控作为 CAN 上的一个节点，以固定周期广播 `TiltServoCommand` 控制最多 4 路倾转舵机；舵机控制器以固定周期广播 `TiltServoFeedback` 回传位置/电流/状态/故障。双方靠「DTID + signature」匹配消息，靠 payload 里的通道序号认领各自通道。**

```text
                    CAN 2.0B 扩展帧，1 Mbps，全网统一波特率
 ┌────────────────────┐                                ┌─────────────────────┐
 │  飞控 (ArduPilot)   │                                │  倾转舵机控制器       │
 │  NodeID = 10       │   ── TiltServoCommand(2025) ─▶ │  NodeID = 100 起     │
 │                    │       12B，50 Hz 广播           │                     │
 │  倾转通道 ch0..ch3  │   ◀── TiltServoFeedback(2026) ─│  4 路丝杠舵机        │
 │                    │       25B，按约定周期广播        │                     │
 └────────────────────┘                                └─────────────────────┘
 优先级 8 (HIGH)            广播帧 CAN ID 格式：
                            (priority<<24) | (DTID<<8) | (src_node_id)
                            Command 示例：0x0807E90A
                                          │    │     └ 飞控 NodeID=10
                                          │    └ 2025=0x7E9
                                          └ priority=8
```

**分工边界**：飞控只负责"按协议广播"和"按协议解析"；舵机侧负责订阅、多帧重组、认领通道、执行和回传。协议不一致时的唯一裁决标准是本文档 §1 的 `.uavcan` 源定义。

---

## 1. 协议定义（唯一事实源，双方各持一份 `.uavcan`）

### 1.1 TiltServoCommand（飞控 → 舵机，DTID **2025**）

```python
# modules/DroneCAN/DSDL/tilt_uavcan/2025.TiltServoCommand.uavcan
# Total payload size: 12 bytes
#include TiltServoLockDef

int16 ch0_pos_cmd     # 单位 0.01°，如 -9000 = -90.00°，+9000 = +90.00°
int16 ch1_pos_cmd
int16 ch2_pos_cmd
int16 ch3_pos_cmd
uint8 ch0_lock        # 0x55=锁刹车，0xAA=解锁，其他值=只执行位置、忽略锁动作
uint8 ch1_lock
uint8 ch2_lock
uint8 ch3_lock
```

### 1.2 TiltServoFeedback（舵机 → 飞控，DTID **2026**）

```python
# modules/DroneCAN/DSDL/tilt_uavcan/2026.TiltServoFeedback.uavcan
# Total payload size: 25 bytes，字段顺序即打包顺序，不可调换
int16 ch0_pos          # 单位 0.005mm（直线丝杠行程）
int16 ch1_pos
int16 ch2_pos
int16 ch3_pos
uint8 supply_voltage   # 单位 0.2V/LSB，real_volt = raw × 0.2
int16 ch0_current      # 单位 0.01A/LSB
int16 ch1_current
int16 ch2_current
int16 ch3_current
uint8 ch0_status       # 位域，bit 定义见 §4 待供应商提供
uint8 ch1_status
uint8 ch2_status
uint8 ch3_status
uint8 ch0_fault        # 位域，bit 定义见 §4 待供应商提供
uint8 ch1_fault
uint8 ch2_fault
uint8 ch3_fault
```

### 1.3 刹车锁常量

```python
LOCK_LOCK    = 0x55   # 锁刹车
LOCK_UNLOCK  = 0xAA   # 解锁刹车
```

### 1.4 编译产物与 signature（飞控侧已实测）

| 项 | Command | Feedback |
|---|---|---|
| DTID | 2025 | 2026 |
| signature | `0x23B8062531071311` | `0x37C09D0F031097C9` |
| MAX_SIZE | 12 B | 25 B |

**signature 由「消息全名 + 字段结构」自动计算**。任何一方改了字段（哪怕是加注释外的实际字段），signature 就变 → 对方收帧 CRC 校验失败 → **整条消息静默丢弃，不报错**。所以：**字段改动必须双方同步，改完各自跑 dsdlc 比对 signature 一致才算对齐**（自检方法见 §6）。

### 1.5 序列化规则（双方实现必须一致）

| 规则 | 值 |
|---|---|
| 字节序 | **小端（little-endian）**，LSB-first 位打包（UAVCAN v0 标准） |
| int16 | 2 字节，低字节在前 |
| uint8 | 1 字节，字节对齐（本协议所有字段都恰好字节对齐，无跨字节位段） |
| 字段顺序 | DSDL 从上到下 = 打包从前往后，**无标签、无跳过**，顺序错=整包解乱 |

---

## 2. 传输层要求（联调失败的第一高发点，请重点确认）

### 2.1 两条消息都超 8 字节，**必须实现 UAVCAN v0 多帧传输**

| 消息 | payload | +1B tail byte 后 | 帧数 | 说明 |
|---|---|---|---|---|
| Command | 12 B | 13 B | **2 帧** | 首帧开头多 2 字节 transfer CRC |
| Feedback | 25 B | 26 B | **4 帧** | 同上 |

多帧规则（UAVCAN v0 标准）：

```text
首帧: [CRC低字节][CRC高字节][数据...][tail byte]   tail: START=1, END=0, TOGGLE=1
中间: [数据...][tail byte]                        tail: START=0, END=0, TOGGLE交替
末帧: [数据...][tail byte]                        tail: START=0, END=1, TOGGLE交替

transfer CRC = CRC16/X.25(初始0xFFFF)，先把 signature 的 8 字节（小端）参与计算，
再算全部 payload 字节。
```

**舵机固件必须**：
1. 按上面规则重组多帧（toggle 交替校验、transfer ID 跳变丢弃、CRC 含 signature）；
2. Command 断流保护：**建议 ≥200ms 收不到完整 Command transfer 即进入安全位**（回垂直或保持现位+锁刹车，具体行为见 §4 待确认项）。

> **为什么必然是多帧、没有"砍成单帧"的选项**：单帧净荷上限 7 字节，而 DF-200 是 4 路倾转，仅 4×int16 pos_cmd 就已 8 字节，加上 4×lock 共 12 字节。4 通道是构型需求（内四倾转、差动后 4 路独立角度），不可裁剪。Feedback 25 字节同样绕不开多帧——**舵机侧的多帧收发能力是本协议的硬性前置条件**。

### 2.2 运行参数（双方各自配置，需对齐的打 ★）

| 参数 | 值 | 说明 |
|---|---|---|
| ★ 波特率 | **1 Mbps**（1000000） | 全网所有节点必须一致，否则物理层不通 |
| ★ priority | 飞控发 Command 用 **8（HIGH）**；舵机发 Feedback 建议 16~24（中低即可） | priority 在 CAN ID 高 5 位，随帧走，接收方不需要配置 |
| ★ 飞控 NodeID | **10**（默认，可用参数改） | |
| ★ 舵机 NodeID | **建议 100 起，每台 +1** | 全网唯一即可，1~125 |
| 发送周期（Command） | **默认 50 Hz**，飞控参数可调（20~200 Hz） | 请告知舵机侧最低可接受的命令刷新率 |
| 发送周期（Feedback） | **待定**：建议 10~50 Hz（见 §4 表） | 舵机侧配置 |
| DTID | Command=2025，Feedback=2026，固定不可改 | 写在 `.uavcan` 文件名前缀里 |

---

## 3. 通道映射与角度约定（飞控侧实现，需舵机方知晓）

### 3.1 通道映射（DF-200：4 台倾转电机，4 路全用满）

DF-200 构型为内四倾转（`Q_TILT_MASK=15`，Motor1~M4 各带一台倾转舵机）。飞控输出通道按序号填入协议通道，映射如下（电机编号沿用 `docs/vtol_df200.md` §2 与 Lua 混控 M1~M4 的约定）：

```text
TiltServoCommand.ch0_pos_cmd  ←→  Motor1 前右倾转舵机（数据库①）
TiltServoCommand.ch1_pos_cmd  ←→  Motor2 后左倾转舵机（数据库⑥）
TiltServoCommand.ch2_pos_cmd  ←→  Motor3 前左倾转舵机（数据库⑤）
TiltServoCommand.ch3_pos_cmd  ←→  Motor4 后右倾转舵机（数据库②）
```

- `Q_TILT_TYPE=0`（统一倾转）阶段 4 路发同一角度；切 `Q_TILT_TYPE=2`（差动倾转）后 4 路各自独立角度——**协议无需任何改动，这是 4 通道设计的直接收益**。
- **若舵机是 4 台独立节点**（每台单通道）：每台从同一条广播里取自己约定的 ch 序号执行，其余忽略。
- **若是 1 台 4 通道控制器**：一条消息全收，按 ch 分发到内部 4 路。
- 两种拓扑飞控侧发送完全相同，**请供应商在交付文档里写明采用哪种**。

### 3.2 角度定义（★ 需双方确认的唯一角度语义）

```text
飞控内部约定：0°  = 桨叶垂直向上（垂起位）
             90° = 桨叶水平向前（巡航位）
pos_cmd = 角度 × 100（0.01° 为 LSB）
例：ch0_pos_cmd = 0      → 垂直
    ch0_pos_cmd = 9000   → 水平前推
    ch0_pos_cmd = -1000  → 垂直向后倾 10°（负方向是否有效待确认）
```

**待供应商确认**：
1. 舵机执行的角度零点和方向定义是什么？（飞控发 0 时舵臂应指向哪？）
2. 若舵机内部是"行程 mm"或别的坐标系，请给出 **角度↔行程换算公式**（或标定表），否则飞控日志里 Feedback 的 pos（0.005mm）无法换算成角度读数。
3. 负角度是否有效？机械限位是多少度到多少度？

---

## 4. status / fault 位域定义（**缺供应商输入，阻塞项**）

DSDL 里 status/fault 只定义了"uint8 位域，应用层解析"，**bit 含义必须由供应商提供**。飞控需要它做两件事：GCS 告警文案、日志判读。请按下表格式填回：

| bit | status 字段含义 | fault 字段含义 |
|:---:|----------------|----------------|
| 0 | （示例：跟踪误差正常/超差） | （示例：无故障/堵转） |
| 1 | | |
| 2 | | |
| ... | | |
| 7 | | |

同时请确认以下**失效行为**（写入交付文档）：

| 场景 | 舵机侧行为 |
|------|-----------|
| Command 断流 >200ms | ？（回垂直 / 保持+锁刹车 / 其他） |
| 收到 lock=0x55 立即锁还是到位后锁 | ？ |
| 上电未收到任何 Command | ？（是否锁刹车待机） |
| Feedback 中 fault 位置位后能否自恢复 | ？ |

---

## 5. 飞控侧实现方案（对供应商透明，供知悉）

飞控侧不改 QuadPlane 倾转控制律，只在 DroneCAN 输出末端新增一条协议出口：

| 改动 | 内容 |
|---|---|
| `AP_DroneCAN.h/cpp` | 新增 `SRV_send_tiltservo()`：读 `SERVOx_FUNCTION=41` 通道的 PWM，按 min/max 归一化换算成 0.01° 角度，填 4 路 pos_cmd + 恒发 0xAA 解锁，50Hz 广播 |
| 接收 | 按 Volz 完整三件套订阅 Feedback（避免重蹈 Himark 接收死代码），写日志 `CTIL` + fault 限频告警 |
| 开关 | `CAN_Dx_OPTIONS` 新增 bit10 `USE_TILT_SERVO`，与现有标准协议/Himark 协议互斥切换 |
| 新参数 | `CAN_Dx_TILT_MIN_DEG` / `CAN_Dx_TILT_MAX_DEG`（默认 0/90，PWM 量程↔角度映射标定用） |

---

## 6. 实机联调流程与自检命令（双方按序执行）

```text
第 1 步  静态对齐（无需硬件）
  双方各自跑 dsdlc 编译 tilt_uavcan，比对输出头的 ID/SIGNATURE/MAX_SIZE 三件套：
    飞控侧: python3 modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py -O /tmp/out \
              modules/DroneCAN/DSDL/tilt_uavcan
    grep -E "ID|SIGNATURE|MAX_SIZE" /tmp/out/include/tilt_uavcan.*.h
  期望完全一致：2025/0x23B8062531071311/12 与 2026/0x37C09D0F031097C9/25
  不一致 → 字段定义有分歧，回到 §1 逐字段核对（这是唯一裁决源）

第 2 步  飞控发 / 舵机收（实机）
  飞控接 4 台舵机+CAN 分析仪，上电解锁后验证：
    □ 抓到 DTID 2025，CAN ID = 0x0807E90A（priority 8, DTID 2025, node 10）
    □ 50Hz 周期稳定
    □ 多帧 2 帧、toggle 交替、CRC 校验通过（4 台舵机侧均重组成功）
    □ 4 台舵机各自按 ch0~ch3 的 pos_cmd 动作，角度方向与 §3.2 一致
    □ 逐台核对通道对应关系（§3.1 映射表与实物一致）

第 3 步  舵机发 / 飞控收（实机）
  舵机上电后飞控侧验证：
    □ Feedback 进订阅表（飞控日志出现 CTIL 行）
    □ 4 路 pos/current/voltage 数值合理，换算公式与 §3.2 标定一致
    □ 人为注入 fault → GCS 10 秒限频告警出现

第 4 步  失效注入（实机）
    □ 拔 Command 线 >200ms → 4 台舵机均进 §4 约定的安全位
    □ 波特率改错 → 双方都无输出（物理层隔离验证）
```

---

## 7. 待办与责任划分（汇总）

| # | 事项 | 责任方 | 状态 |
|---|------|--------|------|
| 1 | §4 status/fault 位域表填写 | **供应商** | ❗缺失，阻塞告警文案与日志判读 |
| 2 | §4 失效行为（断流/上电/lock 时序/自恢复） | **供应商** | ❗缺失 |
| 3 | §3.2 角度零点/方向/负角度/机械限位 | **供应商** | ❗缺失 |
| 4 | Feedback pos（mm）↔ 角度换算公式 | **供应商** | ❗缺失 |
| 5 | Feedback 回传周期确认（建议 10~50Hz） | 双方 | 待定 |
| 6 | Command 最低刷新率确认 | **供应商** | 待定（飞控默认 50Hz） |
| 7 | 拓扑形态声明（1×4通道 or 4×单通道）+ 各通道与物理舵机（Motor1~4）的对应关系 | **供应商** | 待定 |
| 8 | 多帧传输实现与自测（§2.1 清单，Command 2 帧 / Feedback 4 帧） | **供应商** | 飞控侧已具备，硬性前置 |
| 9 | 飞控侧发送/接收代码 | 飞控方 | 方案就绪，待 §1~§4 冻结后实施 |
| 10 | DSDL 字段如有变更 → 双方同步 + signature 比对 | 双方 | 任何字段变更都必须走此流程 |

**原则：先冻结协议（§1~§4 全部打勾），再写代码。** 避免出现 Himark 协议那种"字段两边不一致、signature 写死掩盖、联调期才发现"的成本。

---

## 8. 相关文件索引

| 文件 | 说明 |
|------|------|
| `modules/DroneCAN/DSDL/tilt_uavcan/2025.TiltServoCommand.uavcan` | 命令 DSDL（本协议源定义） |
| `modules/DroneCAN/DSDL/tilt_uavcan/2026.TiltServoFeedback.uavcan` | 反馈 DSDL |
| `modules/DroneCAN/DSDL/tilt_uavcan/TiltServoLockDef.uavcan` | 刹车锁常量 |
| `docs/tilt_uav_can.md` | 本文档 |
| `docs/dsdl_custom_guide.md` | signature 机制详解（飞控内部） |
| `docs/uav_can.md` | CAN ID 布局/五层过滤/总线负载（飞控内部） |
| `docs/df200-uavcan.md` | LSB-first 打包演示与验证脚本（飞控内部） |
