# ArduPilot DroneCAN 机制说明：波特率、周期、DSDL 与 CAN 2.0 的对应

> 代码版本对应：ArduPlane 4.6.x（分支 `qhz/plane_4.6.3`），引用行号已基于当前工作树验证。
>
> 核心结论先行：**DSDL 只定义"消息长什么样"，不决定波特率也不决定发送周期。** 波特率在 CAN 驱动层配置，周期在应用层调度；DSDL 与 CAN 2.0 的关联点是把每条消息的 `data_type_id` 编进 29 位 CAN 扩展帧 ID。

---

## 目录

1. [分层模型](#1-分层模型)
2. [波特率：CAN 驱动层 + 参数](#2-波特率can-驱动层--参数)
3. [发送周期：应用层调度](#3-发送周期应用层调度)
4. [DSDL 与 CAN 2.0 的对应：data_type_id → 29 位 CAN ID](#4-dsdl-与-can-20-的对应data_type_id--29-位-can-id)
5. [接收处理：解析还是丢弃（以飞控为例）](#5-接收处理解析还是丢弃以飞控为例)
6. [节点配置：NodeID、priority、DNA](#6-节点配置nodeidprioritydna)
7. [怎么区分 servo 和 motor：标准消息 + actuator_id](#7-怎么区分-servo-和-motor标准消息--actuator_id)
8. [最小配置清单](#8-最小配置清单)
9. [总线负载分析与优化（13 路舵机场景）](#9-总线负载分析与优化13-路舵机场景)

---

## 1. 分层模型

DroneCAN（原 UAVCAN v0）在 ArduPilot 里分四层，各自职责不重叠：

```
┌─────────────────────────────────────────────────────────┐
│ 应用层  AP_DroneCAN::loop()                              │
│   - 软件定时产生发送周期（SRV_RT / NTF_RT ...）           │
│   - 按 SERVOx_FUNCTION 决定通道是舵机还是电机            │
├─────────────────────────────────────────────────────────┤
│ 序列化层  DSDL 生成的消息结构 + encode/decode            │
│   - 每条消息固定 data_type_id、data_type_signature       │
│   - 字段 bit-packing 规则                                │
├─────────────────────────────────────────────────────────┤
│ 协议层  libcanard                                       │
│   - 组 29 位 CAN ID（priority/type/node）                │
│   - 多帧拆包、transfer ID、CRC、订阅分发                  │
├─────────────────────────────────────────────────────────┤
│ 物理层  HAL CAN 驱动 (bxCAN/FDCAN)                       │
│   - 波特率寄存器、发送邮箱、接收过滤                      │
│   - CAN 2.0B 扩展帧 (29-bit ID + ≤8 字节/帧)             │
└─────────────────────────────────────────────────────────┘
```

关键认知：

- **波特率**是物理层的事，由参数 → CAN 驱动 init → CAN 控制器寄存器决定。
- **周期**是应用层的事，由 `AP_DroneCAN::loop()` 软件定时 + `*_RT` 参数决定。
- **DSDL**是序列化层的事，定义消息字段和类型号，并在协议层被编进 CAN ID。
- **priority**随每个 CAN 帧走，发送方单边设定，用于总线仲裁，不需要对端配置。

---

## 2. 波特率：CAN 驱动层 + 参数

DSDL 里没有任何波特率字段。波特率是物理层/CAN 控制器的配置，在初始化时写进 CAN 外设波特率寄存器。

### 2.1 参数

定义在 `libraries/AP_CANManager/AP_CANIfaceParams.cpp`：

```cpp
// @Param: BITRATE
// @Description: Bit rate can be set up to from 10000 to 1000000
// @Range: 10000 1000000
AP_GROUPINFO("BITRATE", 2, AP_CANManager::CANIface_Params, _bitrate, 1000000),  // 行 36

// CAN-FD 数据段波特率（仅 HAL_CANFD_SUPPORTED 板子）
AP_GROUPINFO("FDBITRATE", 3, AP_CANManager::CANIface_Params, _fdbitrate, HAL_CANFD_SUPPORTED),  // 行 44
```

- 参数名：`CAN_D1_BITRATE` / `CAN_D2_BITRATE`（每个 CAN 接口一组）。
- 默认 **1 Mbps**，范围 10 k–1 M。
- CAN-FD 数据段另有 `CAN_Dx_FDBITRATE`（1M–8M）。

### 2.2 生效点

`libraries/AP_CANManager/AP_CANManager.cpp:181`：

```cpp
can_initialised = hal_mutable.can[i]->init(
    _interfaces[i]._bitrate,
    _interfaces[i]._fdbitrate * 1000000,
    AP_HAL::CANIface::NormalMode);
```

参数传给 HAL CAN 驱动，再往下 `AP_HAL_ChibiOS` 的 bxCAN/FDCAN 驱动用这个值计算分频、BS1、BS2、SJW 寄存器。

### 2.3 要点

- 总线上**所有节点（飞控 + 所有外设）必须设成一样**，否则物理层都通不了。
- 部分板子可在 hwdef 里固定，但主线 Cube/Pixhawk 基本都走参数，默认 1 Mbps。

---

## 3. 发送周期：应用层调度

发送频率是 `AP_DroneCAN::loop()` 这个独立线程里软件定时的（`libraries/AP_DroneCAN/AP_DroneCAN.cpp:516` 起）。主循环每 100 µs 跑一轮 `canard_iface.process(1)`，然后各消息自己判断"到点没"。

### 3.1 各类消息的周期来源

| 消息 | 周期来源 | 默认 | 代码位置 |
|------|----------|------|----------|
| 舵机/ESC 输出 | `1000000 / CAN_D1_SRV_RT` | 50 Hz | `AP_DroneCAN.cpp:552` |
| Notify 状态灯 | `1000 / CAN_D1_NTF_RT` | 20 Hz | `AP_DroneCAN.cpp:977` |
| Relay/Hardpoint | `1000 / CAN_D1_RLY_RT` | 0（关） | `AP_DroneCAN.cpp:1270` |
| NodeStatus | 硬编码 1000 ms | 1 Hz | `AP_DroneCAN.cpp:662` |
| Hobbywing ESC ID 查询 | 硬编码 1000 ms | 1 Hz | `AP_DroneCAN.cpp:585` |
| GPS Fix/Safety/Arming | 随主循环或事件触发 | — | `AP_DroneCAN.cpp:540` 起 |

舵机输出的定时逻辑（`AP_DroneCAN.cpp:551-553`）：

```cpp
uint32_t now = AP_HAL::micros();
const uint32_t servo_period_us = 1000000UL / unsigned(_servo_rate_hz.get());
if (now - _SRV_last_send_us >= servo_period_us) {
    _SRV_last_send_us = now;
    // ... SRV_send_actuator() / SRV_send_himark()
}
```

参数默认值（`AP_DroneCAN.cpp`）：

```cpp
AP_GROUPINFO("SRV_RT", 4, AP_DroneCAN, _servo_rate_hz, 50),    // 行 126
AP_GROUPINFO("NTF_RT", 6, AP_DroneCAN, _notify_state_hz, 20),  // 行 141
AP_GROUPINFO("RLY_RT", 23, AP_DroneCAN, _relay.rate_hz, 0),    // 行 171
```

### 3.2 DSDL 的 `timeout_ms` 是"过期丢弃"，不是周期

DSDL 在这条链路里唯一沾边的是 `timeout_ms`——它是"这条消息在 TX 队列里最多等多久，过期就丢弃"，**不是发送周期**。

`libraries/AP_DroneCAN/AP_Canard_iface.cpp:94`：

```cpp
.deadline_usec = AP_HAL::micros64() + (bcast_transfer.timeout_ms * 1000),
```

所有周期消息的超时被统一设成 `CAN_PERIODIC_TX_TIMEOUT_MS = 2`（`AP_DroneCAN.cpp:280`），注释明说 "set this to 1 to minimise resend of stale msgs"——周期消息宁可丢旧帧也不补发过时数据。

C++ 包装层 `Sender` 的默认值（`modules/DroneCAN/libcanard/canard/publisher.h:78-79`）：

```cpp
uint8_t priority = CANARD_TRANSFER_PRIORITY_MEDIUM;
uint32_t timeout = 1000;
```

APM 在初始化时按消息重要性覆盖这两个值。

**结论：周期由 `loop()` 的定时逻辑 + `*_RT` 用户参数决定；DSDL 只管数据格式，不管"多久发一次"。**

---

## 4. DSDL 与 CAN 2.0 的对应：data_type_id → 29 位 CAN ID

这是 DSDL 真正和 CAN 2.0 产生对应的地方。CAN 2.0B 用 **29 位扩展帧 ID**，DroneCAN 把这 29 位切成若干字段，其中**消息类型号（data_type_id）就编在 CAN ID 里**。

### 4.1 DSDL 编译时给每条消息固定 ID 和签名

构建时 `dronecan_dsdlc` 把 `.uavcan`/DSDL 定义生成 C 头文件（位于 `build/<board>/modules/DroneCAN/libcanard/dsdlc_generated/include/`）。每条消息头里都有两个常量，以 `ardupilot.equipment.power.BatteryPeriodic` 为例：

```c
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_ID        20011      // data_type_id
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_SIGNATURE (0xF012494E97358D2ULL)
```

- `_ID`（20011）：**这条消息在总线上的类型号**，由 DSDL 命名空间分配，全总线唯一、固定。
- `_SIGNATURE`：64 位数据结构 CRC，用于版本兼容性校验（字段一改签名就变）。它不放进 CAN ID，但参与多帧传输的 CRC。

### 4.2 libcanard 把 data_type_id 装进 29 位 CAN ID

发送时 `broadcast()` 构造一个 transfer（带 `data_type_id`、`priority`、payload），交给 libcanard。`modules/DroneCAN/libcanard/canard.c` 根据传输类型把 CAN ID 拼出来——这就是 DSDL 类型到 CAN 2.0 帧 ID 的精确映射。

**普通消息广播**（`canard.c:217`）：

```c
can_id = ((uint32_t) transfer_object->priority << 24U)
       | ((uint32_t) transfer_object->data_type_id << 8U)
       | (uint32_t) canardGetLocalNodeID(ins);
```

**匿名消息**（node id 为 0 时，如动态 ID 申请，`canard.c:212`）：

```c
can_id = ((uint32_t) transfer_object->priority << 24U)
       | ((uint32_t) discriminator << 9U)
       | ((uint32_t) (transfer_object->data_type_id & DTIDMask) << 8U)
       | (uint32_t) canardGetLocalNodeID(ins);
```

**服务请求/响应**（`canard.c:370`，bit7 置 1 以区分消息帧）：

```c
const uint32_t can_id = ((uint32_t) transfer_object->priority << 24U)
                      | ((uint32_t) transfer_object->data_type_id << 16U)
                      | ((uint32_t) transfer_object->transfer_type << 15U)
                      | ((uint32_t) destination_node_id << 8U)
                      | (1U << 7U)
                      | (uint32_t) canardGetLocalNodeID(ins);
```

#### 29 位 CAN ID 精确位域布局

位域提取宏定义在 `modules/DroneCAN/libcanard/canard.c:43-49`：

```c
#define SOURCE_ID_FROM_ID(x)   ((uint8_t)(((x) >> 0U)  & 0x7FU))  // bit 0-6
#define SERVICE_NOT_MSG_FROM_ID(x) ((bool)(((x) >> 7U)  & 0x1U)) // bit 7
#define DEST_ID_FROM_ID(x)     ((uint8_t)(((x) >> 8U)  & 0x7FU))  // bit 8-14
#define SRV_TYPE_FROM_ID(x)    ((uint8_t)(((x) >> 16U) & 0xFFU))  // bit 16-23 (服务)
#define MSG_TYPE_FROM_ID(x)    ((uint16_t)(((x) >> 8U) & 0xFFFFU))// bit 8-23 (消息)
#define PRIORITY_FROM_ID(x)    ((uint8_t)(((x) >> 24U) & 0x1FU))  // bit 24-28
```

消息帧（广播，bit7=0）和服务帧（请求/响应，bit7=1）布局不同：

```
消息广播帧 (ArrayCommand 等)，29 位:
 31  29 28      24 23                  8 7   0
  ┌──┬──────────┬──────────────────────┬──┬─────┐
  │  │ priority │    data_type_id      │0 │ src │
  │  │  (5bit)  │       (16bit)        │  │node │
  └──┴──────────┴──────────────────────┴──┴─────┘
  bit31-29:0  bit28-24      bit23-8         bit7 bit6-0
  例: priority=HIGH, data_type_id=1010(actuator.ArrayCommand), src=10(飞控)

服务请求/响应帧 (参数读写等)，29 位:
 31  29 28   24 23     16 15 14   8 7  6   0
  ┌──┬───────┬─────────┬──┬───────┬──┬──────┐
  │  │priori │ data_   │rq│ dest  │1 │ src  │
  │  │  ty   │ type_id │st│ node  │  │ node │
  └──┴───────┴─────────┴──┴───────┴──┴──────┘
            bit23-16  bit15 bit14-8 bit7 bit6-0
```

| 位段 | 消息广播帧 | 服务请求/响应帧 |
|------|-----------|----------------|
| bit 24-28 | priority（5 bit，0–31，越小越高） | priority |
| bit 8-23 | data_type_id（16 bit，如 1010） | data_type_id（bit 16-23，高 8 bit） |
| bit 15 | —（保留 0） | request_not_response（1=请求，0=响应） |
| bit 8-14 | —（并入 data_type_id） | **destination NodeID**（7 bit） |
| bit 7 | 0（消息标志） | 1（服务标志） |
| bit 0-6 | **source NodeID**（7 bit） | **source NodeID**（7 bit） |

接收时反过来：CAN 控制器收到一帧 29 位 ID，libcanard 从里抠出 `data_type_id`，查注册过的订阅表（`shouldAcceptTransfer`）决定要不要收、用哪个 `_decode` 函数解析。**所以 DSDL 的 `data_type_id` 就是"CAN ID 里识别这是什么消息"的字段。**

#### source NodeID 和 destination NodeID 怎么体现

- **source NodeID 永远在 bit 0-6**（`SOURCE_ID_FROM_ID`）：每一帧都带发送方的 NodeID。广播帧（如舵机命令 ArrayCommand）只有 source，没有专门的 dest 字段——因为它是发给所有节点的；
- **destination NodeID 只在服务帧里出现**（bit 8-14，`DEST_ID_FROM_ID`）：点对点通信（参数读写 `param.GetSet`、固件更新、DNA 分配）才填目标 NodeID。接收方在第 ② 层（`canard.c:422`）检查：服务帧的 dest 不是自己就丢弃；
- **广播帧的 bit 8-14 是 data_type_id 的一部分**，不是 dest——这就是为什么舵机命令能一条广播被所有节点收到，而点对点参数访问只能被特定节点收到。

### 4.3 NodeID 的要求与作用

#### 4.3.1 NodeID 的要求

- **范围 1–127**，7 bit 字段（bit 0-6 能表示 0–127）。0 是匿名/未分配，127 是广播（`CANARD_BROADCAST_NODE_ID`），实际可用节点号 **1–125**（`MAX_NODE_ID 125`，`AP_DroneCAN_DNA_Server.cpp:39`）；
- **全网唯一**：同一条 CAN 总线上两个节点不能同号，否则协议无法区分帧来源、DNA 会冲突；
- **波特率一致**：所有节点必须设相同波特率（`CAN_Dx_BITRATE`，默认 1M），否则物理层不通；
- **飞控默认 NodeID=10**（`CAN_D1_NODE`，`AP_DroneCAN.cpp:84,103`）。检测到 NodeID=0 会回退默认值。

#### 4.3.2 NodeID 的作用

NodeID **不参与"这条命令给哪个舵机"的判断**（那是 payload 里 actuator_id 的事，见第 5.5 节），它的作用是：

1. **标识帧的来源**：source NodeID 在每帧 bit 0-6，接收方知道"这条消息是谁发的"。例如飞控收 `actuator.Status` 时，虽然按 actuator_id 记录，但 `transfer.source_node_id` 可用于定位物理节点；
2. **点对点寻址**：服务帧的 dest NodeID 让参数读写、固件更新、DNA 分配能精确发给某个节点——这些操作不能广播（比如给某个舵机改中位，不能让所有舵机都改）；
3. **健康/在线监控**：每个节点按 1 Hz（2ms–1s 之间）发 `NodeStatus`（data_type_id=341），飞控按 source NodeID 跟踪"谁在线、健康状态、模式"，超时 3s 判定离线；
4. **DNA 动态分配**：新外设 NodeID=0 发匿名申请，飞控 DNA Server 按硬件唯一 ID 分配一个全网唯一的 NodeID 并持久化记住。

#### 4.3.3 NodeID 和 actuator_id 的区别（关键）

| | NodeID | actuator_id |
|---|---|---|
| 位置 | CAN ID 的 bit 0-6 / 8-14 | payload（消息体）里 |
| 可见层 | ①–④层（CAN ID 层）就能看到 | 必须 decode 后第 ⑤ 层才看到 |
| 作用 | 标识物理节点、点对点寻址 | 标识执行器通道 |
| 唯一性 | 全网唯一 | 全网唯一 |
| 一个节点可拥有 | 1 个 | 1 个（独立舵机）或多个（分配器） |
| 谁分配 | 静态配置 / DNA | 飞控输出通道顺序决定，舵机侧绑定 |

以你的混合拓扑为例：16 个舵机有 16 个 actuator_id（1–16），但只有 13 个 NodeID（12 个独立舵机各 1 个 + 分配器 1 个代表 4 个舵机）。飞控发广播命令时 CAN ID 里只有自己的 source NodeID=10、没有 dest；舵机在 payload 里按 actuator_id 认领命令。

### 4.4 完整收发链路

```
应用层:  填 ardupilot...BatteryPeriodic 结构体 (DSDL 定义字段)
   │
encode:  _encode() 按 DSDL 的 bit-packing 规则序列化 → payload 字节流
   │
broadcast: 构造 CanardTransfer{ data_type_id=20011, priority, payload, timeout_ms=2 }
   │
libcanard (canard.c):
   ├─ 算 transfer CRC（含 data_type_signature）
   ├─ 组 29 位 CAN ID: priority<<24 | 20011<<8 | node_id   ← DSDL→CAN ID 对应点
   └─ 多帧拆分（>8字节时加 tail byte / transfer ID）
   │
HAL CAN 驱动:  把 CAN ID + payload 塞进 bxCAN/FDCAN 邮箱，按 1Mbps 波特率发
   │
物理总线:  CAN 2.0B 扩展帧 (29-bit ID + ≤8 字节/帧)
```

接收侧对称：HAL 收中断取帧 → libcanard 按 CAN ID 里的 data_type_id 分发 → `_decode()` 反序列化回结构体 → 调回调。

---

## 5. 接收处理：解析还是丢弃（以飞控为例）

飞控收到一帧 CAN 消息后，经过**五层筛选**才到实际执行，任何一层不通过都丢弃。前四层基于 CAN ID 和协议，第五层（应用层）才解析 payload 里的通道号。核心代码在 `modules/DroneCAN/libcanard/canard.c:403` 的 `canardHandleRxFrame()` 和 `libraries/AP_DroneCAN/AP_Canard_iface.cpp:329` 的 `processRx()`。

```
CAN 总线物理帧
  │
 ① 协议帧格式层     扩展帧/RTR/错误帧/长度检查（canard.c:411）         ← 看帧
  │
 ② 目标地址层       点对点帧 dst 是否本机（canard.c:422）             ← 看 CAN ID 里的 NodeID
  │
 ③ 订阅过滤层       should_accept 查注册表（handler_list.h:59）       ← 看 CAN ID 里的 data_type_id
  │
 ④ 重组+decode层    多帧拼装/CRC/DSDL decode → 业务回调（subscriber.h:77） ← 解开 payload
  │
 ⑤ 应用回调层       payload 里的 actuator_id 是否归我管（节点固件）    ← 看消息体里的通道号
```

> **关键区分**：①–④层只能看到 29 位 CAN ID 里的字段（priority / data_type_id / NodeID），**看不到 payload 里的 actuator_id**。所以"这是不是舵机命令"在第 ③ 层判断；"这是给哪个舵机的命令"必须等第 ④ 层 decode 之后，在第 ⑤ 层判断。

### 5.1 第一层：CAN 控制器取帧

`CanardInterface::processRx()`（`AP_Canard_iface.cpp:329` 起）轮询每个 CAN iface，从接收邮箱取帧并封成 `CanardCANFrame`，再调 `canardHandleRxFrame(&canard, &rx_frame, timestamp)`（`:371`）。

> DroneCAN 主要依赖**软件过滤**（订阅注册表），不依赖 CAN 控制器的硬件 ID filter——因为订阅是动态的，硬件 filter 表不够用。

### 5.2 第二层：libcanard 协议层快速丢弃

`canardHandleRxFrame()`（`canard.c:403`）一进来做两个快速判断：

```c
// ① 必须是扩展帧(EFF)、不能是 RTR 远程帧、不能是错误帧、长度≥1
if ((frame->id & CANARD_CAN_FRAME_EFF) == 0 ||
    (frame->id & CANARD_CAN_FRAME_RTR) != 0 ||
    (frame->id & CANARD_CAN_FRAME_ERR) != 0 ||
    (frame->data_len < 1))
    return -CANARD_ERROR_RX_INCOMPATIBLE_PACKET;   // 丢弃

// ② 点对点帧（非广播）：目标 NodeID 不是本机 → 丢弃
if (transfer_type != CanardTransferTypeBroadcast &&
    destination_node_id != canardGetLocalNodeID(ins))
    return -CANARD_ERROR_RX_WRONG_ADDRESS;          // 丢弃
```

通过后从 CAN ID 解析出 `source_node_id`、`data_type_id`、`transfer_descriptor`，再取尾字节 `tail_byte`（多帧标志，`canard.c:434`）。

按"是不是多帧传输的第一帧"分两路（`canard.c:437` 起）：

```c
if (IS_START_OF_TRANSFER(tail_byte)) {
    // 第一帧：问应用层"你要不要这个消息？"
    if (ins->should_accept(ins, &sig, data_type_id, transfer_type, source_node_id))
        rx_state = traverseRxStates(ins, transfer_descriptor);  // 建重组状态
    else
        return -CANARD_ERROR_RX_NOT_WANTED;         // 没订阅 → 丢弃
} else {
    // 后续帧：必须能找到第一帧建的状态，否则漏了首帧
    rx_state = findRxState(ins, transfer_descriptor);
    if (rx_state == NULL)
        return -CANARD_ERROR_RX_MISSED_START;       // 漏首帧 → 丢弃
}
```

设计要点：

- **首帧才查订阅表**：`should_accept` 只在一个 transfer 的第一帧调一次，不想要的消息后续帧直接 `MISSED_START` 丢掉，省 CPU；
- **多帧重组**：超过 8 字节的消息拆多帧（带 transfer ID、CRC、toggle bit），由 `rx_state` 拼装；超时、transfer ID 跳变、接口切换等会重置状态（`canard.c:461` 起）。

### 5.3 第三层：should_accept 查订阅注册表

`should_accept` 是 `canardInit` 时注册的回调（`AP_Canard_iface.cpp:66`），最终到 `HandlerList::accept_message`（`modules/DroneCAN/libcanard/canard/handler_list.h:59`）：

```cpp
static bool accept_message(uint8_t index, uint16_t msgid, uint64_t &signature) {
    HandlerList* entry = head[index];
    while (entry != nullptr) {
        if (entry->msgid == msgid) {        // 按 data_type_id 匹配
            signature = entry->signature;   // 回填 DSDL 签名供校验
            return true;                    // 有人订阅 → 接收
        }
        entry = entry->next;
    }
    return false;                           // 没人订阅 → 丢弃
}
```

这是一条按消息类型号串起来的链表。只有启动时注册过的 `data_type_id` 才返回 true，其余一律 `RX_NOT_WANTED` 丢弃。

### 5.4 第四层：收齐后 decode + 回调分发

一个 transfer 所有帧收齐、CRC 通过后，libcanard 调 `onTransferReception`（`AP_Canard_iface.cpp:178`）→ `handle_message`，再由 `HandlerList::handle_message`（`handler_list.h:78`）按 `data_type_id + transfer_type` 找到对应 Subscriber：

```cpp
static void handle_message(uint8_t index, const CanardRxTransfer& transfer) {
    HandlerList* entry = head[index];
    while (entry != nullptr) {
        if (transfer.data_type_id == entry->msgid &&
            entry->transfer_type == transfer.transfer_type) {
            entry->handle_message(transfer);   // → Subscriber<msgtype>::handle_message
            return;
        }
        entry = entry->next;
    }
}
```

最终到 `Subscriber<msgtype>::handle_message`（`subscriber.h:77`），这里才真正用 DSDL 生成的 decode 反序列化并调业务回调：

```cpp
void handle_message(const CanardRxTransfer& transfer) override {
    msgtype msg {};
    if (msgtype::cxx_iface::decode(&transfer, &msg)) {
        return;                    // decode 失败（字段越界/长度不对）→ 丢弃
    }
    // decode 成功，调所有注册在这个消息上的回调
    Subscriber<msgtype>* entry = branch_head[index];
    while (entry != nullptr) {
        entry->cb(transfer, msg);  // 业务回调
        entry = entry->next;
    }
}
```

即使订阅了，**decode 失败也会丢弃**——这是最后一道防线。

### 5.5 第五层：应用回调里的通道过滤（actuator_id）

前四层只判断"这是什么消息"（data_type_id），回答不了"这是给哪个舵机的命令"，因为 `actuator_id` 在 payload 里，必须 decode 之后才知道。这层过滤发生在业务回调里。

以 `Tools/AP_Periph/can.cpp:656` 的 `handle_act_command` 为例（AP_Periph 就是一个分配器节点的参考实现，一个 CAN 节点带多路 PWM 输出）：

```cpp
void AP_Periph_FW::handle_act_command(CanardInstance* canard_instance, CanardRxTransfer* transfer)
{
    uavcan_equipment_actuator_ArrayCommand cmd;
    if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &cmd)) {
        return;                              // ④ decode 失败丢弃
    }

    for (uint8_t i = 0; i < cmd.commands.len; i++) {   // ⑤ 遍历全部 Command
        const auto &c = cmd.commands.data[i];
        switch (c.command_type) {
        case UNITLESS: rcout_srv_unitless(c.actuator_id, c.command_value); break;
        case PWM:      rcout_srv_PWM(c.actuator_id, c.command_value);      break;
        }
    }
}
```

回调把每个 command 的 `actuator_id` 传给输出层（`Tools/AP_Periph/rc_out.cpp:108`）：

```cpp
void AP_Periph_FW::rcout_srv_unitless(uint8_t actuator_id, const float command_value) {
    // actuator_id=1 → k_rcin1, actuator_id=13 → k_rcin13 ...
    const auto function = SRV_Channel::Aux_servo_function_t(SRV_Channel::k_rcin1 + actuator_id - 1);
    SRV_Channels::set_output_norm(function, command_value);   // 本地没配的 function 是空操作
}
```

节点的行为由它绑定了几个 actuator_id 决定：

- **单体 CAN 舵机**：固件只绑一个 `my_actuator_id`，回调里 `if (c.actuator_id == my_actuator_id) drive_motor(...)`，其余忽略；
- **多通道分配器**（AP_Periph 模型）：本地 `RCn_FUNCTION` 参数配置了哪几个输出通道，就只对那几个 actuator_id 产生 PWM，其余静默忽略。一个 NodeID 可以代表多个 actuator_id。

**这就解释了"一条广播驱动所有舵机"的机制**：所有订阅了 1010 的节点都在第 ③ 层通过、在第 ④ 层解出完整 commands 数组，但只在第 ⑤ 层各取所需。data_type_id 决定消息"到不到节点手里"，actuator_id 决定节点"执不执行"。

### 5.6 飞控在哪里注册"要接收哪些消息"

`AP_DroneCAN::init()`（`libraries/AP_DroneCAN/AP_DroneCAN.cpp:370` 起）让各驱动注册各自关心的消息：

```cpp
AP_GPS_DroneCAN::subscribe_msgs(this);          // :372
AP_Compass_DroneCAN::subscribe_msgs(this);      // :375
AP_Baro_DroneCAN::subscribe_msgs(this);         // :378
AP_BattMonitor_DroneCAN::subscribe_msgs(this);  // :380
AP_Airspeed_DroneCAN::subscribe_msgs(this);     // :382
AP_RangeFinder_DroneCAN::subscribe_msgs(this);  // :388
AP_RCProtocol_DroneCAN::subscribe_msgs(this);   // :391
// ... EFI/Proximity/Mount/Temperature/RPM 等
```

以电池监控为例（`libraries/AP_BattMonitor/AP_BattMonitor_DroneCAN.cpp:45`）：

```cpp
void AP_BattMonitor_DroneCAN::subscribe_msgs(AP_DroneCAN* ap_dronecan) {
    Canard::allocate_sub_arg_callback(ap_dronecan,
        &handle_battery_info_trampoline, ap_dronecan->get_driver_index());
    // ...
}
```

`allocate_sub_arg_callback` new 一个 `Subscriber<消息类型>` 挂进 `HandlerList` 链表，并记录回调函数指针。**没挂的消息，第三层 `accept_message` 直接返回 false 丢弃。**

舵机状态回传方面，飞控订阅了标准 `uavcan.equipment.actuator.Status`（`AP_DroneCAN.h:332-333`，回调 `handle_actuator_status` 在 `AP_DroneCAN.cpp:1383`）；Volz 私有协议 `com.volz.servo.ActuatorStatus` 默认关闭（`AP_DRONECAN_VOLZ_FEEDBACK_ENABLED 0`，`AP_DroneCAN.h:67-68`）。

### 5.7 外设侧对称

总线上其他节点（CAN 舵机/ESC/GPS 外设，固件如 `Tools/AP_Periph/`）走同一套 libcanard 机制，只是订阅的消息少（舵机只订 `actuator.ArrayCommand`，ESC 只订 `esc.RawCommand`），发送的是遥测/状态。例如飞控发的 `esc.RawCommand`，ESC：收帧 → 检查是广播且类型号匹配 → 订阅表命中 → 收齐 decode → 回调里取数组下标对应自己的通道值 → 驱动电机。舵机命令同理：所有舵机节点都通过第 ③ 层（都订阅了 1010），在第 ⑤ 层按 actuator_id 各取所需。

### 5.8 丢弃点汇总

| 丢弃阶段 | 触发条件 | 返回码/位置 |
|----------|----------|-------------|
| 帧格式 | 非扩展帧/RTR/错误帧/长度0 | `RX_INCOMPATIBLE_PACKET` `canard.c:411` |
| 目标地址 | 点对点帧且 dst 非本机 | `RX_WRONG_ADDRESS` `canard.c:422` |
| 未订阅 | 首帧查订阅表没命中 | `RX_NOT_WANTED` `canard.c:448` |
| 漏首帧 | 后续帧找不到 rx_state | `RX_MISSED_START` `canard.c:456` |
| 多帧异常 | transfer ID 跳变/超时/CRC 错 | 状态重置 `canard.c:461` 起 |
| decode 失败 | DSDL 反序列化字段越界 | `subscriber.h:79` return |
| 回调内 | 业务层判断不处理 | 回调 return（通道号不属自己等） |

---

## 6. 节点配置：NodeID、priority、DNA

### 6.1 NodeID——必须配置且全网唯一

DroneCAN 节点号范围 **1–125**（0 是匿名/未分配，见 `libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp:39` 的 `MAX_NODE_ID 125`）。

**飞控**：参数 `CAN_D1_NODE`（`libraries/AP_DroneCAN/AP_DroneCAN.cpp:98-103`）：

```cpp
// @Param: NODE
AP_GROUPINFO("NODE", 1, AP_DroneCAN, _dronecan_node, AP_DRONECAN_DEFAULT_NODE);
```

默认值（`AP_DroneCAN.cpp:83-84`）：

```cpp
#ifndef AP_DRONECAN_DEFAULT_NODE
#define AP_DRONECAN_DEFAULT_NODE 10
```

飞控检测到 NodeID 为 0 时会回退默认值（`AP_DroneCAN.cpp:331-332`）。

**外设（CAN 舵机/ESC/GPS）**两种方式：

- **静态**：用厂商配置工具给每个外设写死 NodeID（常见、稳定）。
- **动态 DNA**（Dynamic Node ID Allocation）：外设上电时 NodeID=0 发匿名申请，飞控作为 **DNA Server** 给它分配一个号并记住其硬件唯一 ID（`AP_DroneCAN_DNA_Server.cpp:121` 的 `handle_allocation`）。分配表持久化到飞控存储，重启后同一硬件仍拿同一号。

### 6.2 priority——发送方单边设定，不需要对端知道

priority 嵌在每个 CAN 帧的 29 位 ID 高位（`priority<<24`，见 `canard.c:217`），作用是**总线仲裁**——CAN 协议本身靠 ID 数值小的优先赢得总线。所以：

- priority 是**发送方自己决定**的，接收方解析 CAN ID 时能读到，但不需要预先协商或配置一致；
- 代码里按消息重要性硬编码（`AP_DroneCAN.cpp:411-485`）：

| 消息 | priority | 行号 |
|------|----------|------|
| ESC RawCommand（电机，最关键） | `HIGH - 1`（最高） | 415 |
| 舵机 ArrayCommand | `HIGH` | 411 |
| Hobbywing/HiMark 输出 | `HIGH` | 419, 424 |
| Safety/Arming/GPS/Notify/Relay | `LOW` | 434–470 |
| NodeStatus / Stats | `LOWEST` | 479–485 |

C++ 包装层默认 `CANARD_TRANSFER_PRIORITY_MEDIUM`（`publisher.h:78`），APM 在初始化时按上表覆盖。这保证电机指令在总线拥塞时最先发出去。

> **"互相要知道"的只有 NodeID（唯一）+ 波特率（一致）；priority 不需要对端配置，它随帧走。**

---

## 7. 怎么区分 servo 和 motor：标准消息 + actuator_id

**关键认知：不是靠 NodeID 区分设备角色，而是靠"消息类型 + 消息里的通道号字段"。**

DroneCAN/uavcan 标准定义了不同的标准消息（各有固定 data_type_id），设备固件只订阅它认识的那种消息，再从消息里按自己的通道绑定取走属于自己的命令。

### 7.1 舵机（servo）——`uavcan.equipment.actuator.ArrayCommand`

代码 `SRV_send_actuator()`（`libraries/AP_DroneCAN/AP_DroneCAN.cpp:734-788`）：

```cpp
void AP_DroneCAN::SRV_send_actuator(void)
{
    ...
    do {
        uavcan_equipment_actuator_ArrayCommand msg;       // 行 743
        // DroneCAN can hold maximum of 15 commands in one frame
        for (i = 0; starting_servo < DRONECAN_SRV_NUMBER && i < 15; starting_servo++) {
            uavcan_equipment_actuator_Command cmd;
            if (_SRV_conf[starting_servo].servo_pending && ...) {
                cmd.actuator_id = starting_servo + 1;     // 行 760，通道号 1-based
                if (option_is_set(Options::USE_ACTUATOR_PWM)) {
                    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_PWM;
                    cmd.command_value = _SRV_conf[starting_servo].pulse;
                } else {
                    cmd.command_type = UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS;
                    cmd.command_value = constrain_float(((float)_SRV_conf[starting_servo].pulse - 1000.0) / 500.0 - 1.0, -1.0, 1.0);
                }
                msg.commands.data[i] = cmd;
                i++;
            }
        }
        msg.commands.len = i;
        if (i > 0) {
            act_out_array.broadcast(msg);                 // 广播
        }
        ...
    } while (repeat_send);
}
```

要点：

- 一条广播帧最多装 15 个 `Command`（行 746）；
- 每个 `Command` 带 **`actuator_id`**（1-based，行 760），即"这条命令发给第几个舵机"；
- 每路舵机值可能是 PWM 脉宽（1000–2000）或归一化 [-1,1]，由 `command_type` 标明（行 762-768）；
- **CAN 舵机收到广播后**：解析出 `actuator_id`，若等于它自己绑定的通道号（如该舵机配置成响应 actuator_id=3）就执行 `command_value`，否则忽略。

### 7.2 电机/ESC（motor）——`uavcan.equipment.esc.RawCommand`

代码 `SRV_send_esc_raw()`（`AP_DroneCAN.cpp:826` 起）：

```cpp
uavcan_equipment_esc_RawCommand esc_msg;                 // 行 826
...
// esc offset allows for efficient packing of higher ESC numbers in RawCommand
if (_SRV_conf[i].esc_pending) { esc_msg.cmd[...] = scaled; }  // 行 838
...
esc_raw.broadcast(esc_msg);
```

要点：

- 这是**另一个 data_type_id** 的标准消息，跟舵机的 ArrayCommand 完全分开；
- `RawCommand.cmd[]` 是数组，**数组下标就是 ESC 通道号**（有 `CAN_D1_ESC_OFFSET` 参数可重排，参数说明在 `AP_DroneCAN.cpp:145`）；
- 值范围 [-1,1]（正向）或双向（可逆 ESC，正负值，行 159）；
- **CAN ESC 只订阅 `esc.RawCommand`**，按数组下标取自己那份，不关心舵机的 ArrayCommand，反之亦然。

### 7.3 通道角色由 `SERVOx_FUNCTION` 决定

`_SRV_conf[i].servo_pending` vs `esc_pending` 两个标志，是在 RCOutput 层根据该通道的 `SERVOx_FUNCTION`（电机/舵机功能号）打标的。同一份 `_SRV_conf[]` 里：

- 配置成舵机功能的通道 → 进 `ArrayCommand`（带 actuator_id）→ 发给 CAN 舵机；
- 配置成电机功能的通道 → 进 `RawCommand`（数组下标）→ 发给 CAN ESC。

### 7.4 整体机制图

```
uavcan 标准规定两种消息（各有固定 data_type_id）:
  ┌─ uavcan.equipment.actuator.ArrayCommand  → 舵机命令，每条带 actuator_id
  └─ uavcan.equipment.esc.RawCommand        → 电机命令，数组下标即通道

广播帧 CAN ID 里含 data_type_id:
  ├─ 舵机设备: 硬件只过滤/订阅 actuator.ArrayCommand 的 ID，按 actuator_id 认领
  └─ ESC 设备: 硬件只过滤/订阅 esc.RawCommand 的 ID，按数组下标认领

飞控侧: SERVOx_FUNCTION 决定一个输出通道算舵机还是电机，
       分别塞进对应消息，再以 HIGH / HIGH-1 优先级广播出去
```

外设厂商只要按 uavcan 标准实现这两个消息的解析，任何品牌的 CAN 舵机/ESC 都能被 APM 驱动——这就是"通用协议"的互操作性。NodeID 只用来标识"哪个物理节点"和做点对点服务（参数、固件更新），不参与"这条命令给舵机还是电机"的判断。

### 7.4.1 混合拓扑实例：12 个独立舵机 + 1 个分配器（4 个舵机）

实际系统里这两种形态**共存于同一条总线**，飞控一视同仁：

```
                    CAN 总线 (1 Mbps)
        ┌──────┬──────┼──────┬──────┬──────┐
        │      │      │      │      │      │
     Node20 Node21  ...   Node31 Node40    飞控 Node10
     (id=1)(id=2)        (id=12)  │
       │      │               ┌──┴──┬──┴──┬──┴─┐
     舵机1  舵机2  ...     舵机13 舵机14 舵机15 舵机16
     (独立) (独立)         (分配器内部 PWM 驱动)

 13 个 CAN 节点：12 个独立舵机 + 1 个分配器
 16 个 actuator_id：1~12 给独立舵机，13~16 给分配器的 4 个舵机
```

工作过程（结合第 5 章的五层过滤）：

1. **飞控发 2 条 `ArrayCommand`**（data_type_id=1010）：因为一条消息最多装 15 个 Command（`Command[<=15]`，见 `AP_DroneCAN.cpp:747` 的 `i < 15` 和 `:783` 的 `repeat_send`），16 个作动器由 `SRV_send_actuator()` 的 `do...while(repeat_send)` 循环拆成两条独立广播——第一条装 15 个（actuator_id 1–15），第二条装 1 个（actuator_id 16）。两条都是独立 transfer，独立多帧、独立 CRC；
2. **所有 13 个节点对每条都在第 ③ 层通过**（它们都订阅了 1010），都在第 ④ 层 decode 出各条里的 commands 数组；
3. **第 ⑤ 层各取所需**（跨两条消息累积）：
   - Node20（独立舵机）只发现 actuator_id=1 是自己的，驱动舵机1，忽略其余 15 个；
   - Node21 只取 actuator_id=2；……Node31 只取 actuator_id=12；
   - Node40（分配器）发现 actuator_id=13/14/15/16 都是自己管的，通过内部 PWM/总线分发给 4 个物理舵机（代码就是 `Tools/AP_Periph/can.cpp:656` 的 `handle_act_command`）；
4. **回传对称**：独立舵机各用自己的 NodeID 发 `Status`（1011）；分配器用同一个 NodeID 40 连发 4 条 `Status`，分别填 actuator_id=13/14/15/16。飞控按 actuator_id 分别记录，不关心来自哪个 NodeID。

关键点：

- **飞控只看到 16 个 actuator_id，拓扑对它透明**；把 4 个独立舵机换成一个分配器，只要 actuator_id 不变，飞控配置（`SERVOx_FUNCTION`）完全不用改；
- **NodeID 数 ≠ 舵机数**：这里 16 个舵机只占 13 个 NodeID（分配器 1 个 NodeID 代表 4 个 actuator_id）；
- **data_type_id 决定"消息到不到节点"，actuator_id 决定"节点执不执行"**——这两个标识符分处第 ③ 层和第 ⑤ 层，不能混为一谈。

### 7.5 厂商私有消息

代码里也有 `com.hobbywing.esc.RawCommand`、`com.himark.servo.ServoCmd` 这类厂商自定义 DSDL（`AP_DroneCAN.cpp:593`、`:820`、`:879`），通过 `CAN_D1_OPTIONS` 位（HobbyWing=7、UseHimark=6，见 `:131`）切换。这是因为某些 ESC 需要专有协议（双向、ID 学习等）。通用场景走标准 uavcan 消息即可，私有消息是可选项。

---

## 8. 最小配置清单

| 配置项 | 参数/位置 | 必须？ | 说明 |
|--------|-----------|:--:|------|
| 波特率 | `CAN_D1_BITRATE` | ✅ 全网一致 | 默认 1M，所有节点同值 |
| 飞控 NodeID | `CAN_D1_NODE` | ✅ | 默认 10，全网唯一 |
| 外设 NodeID | 外设配置工具 / DNA 自动 | ✅ | 静态写死或让飞控 DNA 分配 |
| 输出通道功能 | `SERVOx_FUNCTION` | ✅ | 决定通道进舵机消息还是电机消息 |
| 输出频率 | `CAN_D1_SRV_RT` | 可选 | 默认 50 Hz，舵机/ESC 发送周期 |
| Notify 频率 | `CAN_D1_NTF_RT` | 可选 | 默认 20 Hz |
| Relay 频率 | `CAN_D1_RLY_RT` | 可选 | 默认 0（关） |
| ESC 通道偏移 | `CAN_D1_ESC_OFFSET` | 可选 | 重排 RawCommand 数组下标 |
| 可逆 ESC 位掩码 | `CAN_D1_ESC_RVMASK` | 可选 | 标记哪些通道用双向 ESC |
| priority | 代码硬编码 | 不用配 | 发送方按消息类型自动设（电机最高） |
| actuator_id | 外设绑定 | 看设备 | CAN 舵机要绑定响应哪个通道号 |
| 私有协议选项 | `CAN_D1_OPTIONS` | 可选 | HobbyWing/HiMark 等专有设备 |

---

## 9. 总线负载分析与优化（13 路舵机场景）

> 场景：一路 CAN（1 Mbps）接 13 个标准 uavcan 舵机，飞控发命令、舵机回传状态。下面给出带宽估算和减负手段。
>
> 消息字节大小已基于当前代码树 DSDL 源定义（`modules/DroneCAN/DSDL/uavcan/...`）核实。注意 DSDL 编译器对所有消息统一生成 `_MAX_SIZE` 宏，但只有含动态数组 `T[<=N]` 的消息才真的变长；定长消息的 `_MAX_SIZE` 就是它的固定大小：
>
> | 消息 | 定长/变长 | 实际大小 | 字段 |
> |------|:--:|:--:|------|
> | `actuator.Command` | 定长 | 4 B | actuator_id(8b)+command_type(8b)+command_value(float16,16b) |
> | `actuator.Status` | 定长 | 8 B | id(8b)+position/force/speed(float16×3=48b)+void1+power_rating_pct(7b) |
> | `protocol.NodeStatus` | 定长 | 7 B | uptime(32b)+health(2b)+mode(3b)+sub_mode(3b)+vendor(16b) |
> | `actuator.ArrayCommand` | **变长** | 1~61 B | `Command[<=15]` 动态数组：4b 长度 + N×4B；13 路=53B，满 15 路=61B |

### 9.1 CAN 2.0 帧的实际开销

DroneCAN 用 CAN 2.0B 扩展帧。一帧在总线上占的位数**不只是 payload**：

```
帧起始 1 + 仲裁域 29(扩展ID) + 控制域 6 + 数据域 0~64(DLC=8时为64)
+ CRC 15 + ACK 2 + 帧结束 7 + 位填充(最坏约 +20%)
```

按工程常用估算（含位填充），**每帧总线占用 ≈ 110 位（DLC=8）/ 55 位（DLC=0）**。1 Mbps 下：

- 满载每秒约 **9000 帧**（110 bit/帧）；
- 一帧传输时间约 **110 µs**。

> 精确公式：标准 8 字节数据帧 = 47 bit 开销 + 8×8=64 bit 数据 = 111 bit，再加最坏 ~19 位填充 ≈ 130 bit/帧。下面用 110 bit/帧（典型）估算，留 ~15% 余量。

### 9.2 下行：飞控 → 舵机（命令）

飞控用 `uavcan.equipment.actuator.ArrayCommand` 广播（`AP_DroneCAN.cpp:743`）。每个 `Command` = 4 B（actuator_id 1B + command_type 1B + command_value 用 float16 压成 2B），13 个命令共 52 B payload + 1B 数组长度 = **53 B**，超过单帧 8 B，会被 libcanard 拆成**多帧传输（Multi-frame Transfer）**。

多帧传输每帧带 1B tail byte（含 transfer ID、toggle、start/end 标志），所以每帧最多 7B 有效数据：

- 53 B 命令 ÷ 7 B/帧 ≈ **8 个 CAN 帧**；
- 加首帧的 transfer CRC（2B，占一帧部分空间），实际约 **8~9 帧**。

按默认 `CAN_D1_SRV_RT = 50 Hz`（`AP_DroneCAN.cpp:126`）：

| 项 | 计算 | 带宽 |
|----|------|------|
| 每周期帧数 | ~8.5 帧 | — |
| 每秒帧数 | 8.5 × 50 | **425 帧/秒** |
| 占用位率 | 425 × 110 bit | **46.8 kbps ≈ 4.7% 总线负载** |

即使 13 路全用，**下行命令只占约 5%**，非常轻。ArrayCommand 把所有舵机命令打包进一条广播，这是它比"每舵机一条消息"高效的关键。

### 9.3 上行：舵机 → 飞控（状态回传）

每个舵机回传 `uavcan.equipment.actuator.Status`（最大 8 B：actuator_id 1B + position/force/speed 各 float16 2B + 1B 预留）。8 B payload + 1B tail = 9 B > 8 B，需拆成 **2 帧**（或某些舵机只填部分字段压进单帧，按 2 帧保守算）。

另外每个节点 1 Hz 发 `NodeStatus`（7 B + tail = 8 B，**单帧**）。

假设舵机状态回传频率为 `F_status` Hz：

| 项 | 单舵机 | 13 舵机合计 |
|----|--------|-------------|
| Status 帧数 | 2 帧 × F_status | 26 × F_status 帧/秒 |
| NodeStatus 帧数 | 1 帧 × 1 Hz | 13 帧/秒 |
| 总帧/秒 | — | 26·F_status + 13 |
| 总线负载 | — | (26·F_status + 13) × 110 / 1e6 |

不同回传频率下：

| 舵机 Status 频率 | 上行帧/秒 | 上行负载 |
|:--:|:--:|:--:|
| 1 Hz | 39 | **0.4%** |
| 5 Hz | 143 | **1.6%** |
| 10 Hz | 273 | **3.0%** |
| 25 Hz | 663 | **7.3%** |
| 50 Hz（与命令同频） | 1313 | **14.4%** |

### 9.4 总负载

| 场景 | 下行(命令50Hz) | 上行(Status) | NodeStatus | **合计** |
|------|:--:|:--:|:--:|:--:|
| Status 1 Hz | 4.7% | 0.3% | 0.1% | **≈ 5%** |
| Status 10 Hz | 4.7% | 2.9% | 0.1% | **≈ 8%** |
| Status 25 Hz | 4.7% | 7.2% | 0.1% | **≈ 12%** |
| Status 50 Hz | 4.7% | 14.3% | 0.1% | **≈ 19%** |

**结论**：13 个舵机在 1 Mbps 下，即使命令 50 Hz + 状态 50 Hz 回传，总线负载也只有约 **20%**，余量充足。真正容易把总线打满的是**高频率遥测（ESC 电压/电流/温度/RPM）+ 多电机**，而不是舵机。

> 注意：以上是理想估算。实际还要叠加位填充、错误重传、其他节点（GPS/罗盘/电池）的消息、DNA/参数服务等。工程上建议总线峰值负载控制在 **50–70%** 以下，给仲裁重传和突发留余量。

### 9.5 减少总线负载的手段

按效果从大到小：

1. **降低舵机状态回传频率**：这是最直接的杠杆。舵机状态一般 5–10 Hz 就够监控用，不必跟命令同频。状态回传频率通常在**舵机侧配置**（厂商工具），飞控不主动设定；标准 `actuator.Status` 本身没有强制频率。把 Status 从 50 Hz 降到 5 Hz，上行负载从 14% 降到 1.6%。

2. **降低命令频率 `CAN_D1_SRV_RT`**：默认 50 Hz。普通舵机 25–50 Hz 足够，部分大扭矩舵机甚至 20 Hz。从 50 降到 25 Hz，下行负载减半（4.7% → 2.4%）。注意要 ≥ 舵机内部控制环需求，太低会影响响应。

3. **只发有变化/需要的通道**：APM 的 `_SRV_conf[i].servo_pending` 机制只把 `pending` 的通道打进 `ArrayCommand`（`AP_DroneCAN.cpp:759`）。但要注意只要有一个 pending，整条消息还是会发；通道数减少能减少多帧拆分的帧数。确保没用到的输出通道 `SERVOx_FUNCTION` 设为禁用（-1），避免空跑。

4. **命令值用 float16 而非 float32**：标准 `Command` 的 `command_value` 已经是 float16（2B，见生成头文件 `Command.h:49`），这是 DSDL 层面的优化，无需用户配置。13 个命令 52 B 而不是 78 B。

5. **避免不必要的节点消息**：关掉不用的 DroneCAN 设备/驱动（如不需要 ESC 遥测就别开 `CAN_D1_OPTIONS` 的 HobbyWing 位），GPS/罗盘/电池等只接需要的。`NodeStatus`（1 Hz）和 DNA 是协议必需，开销很小（每节点 0.1%），不用动。

6. **升级到 CAN-FD**：如果板子和外设都支持 CAN-FD（`CAN_Dx_FDBITRATE`，数据段可达 8 Mbps），同样的消息在数据段用更高速率传，载荷也可达 64 B/帧，`ArrayCommand` 可能一帧就装下 13 个命令。这是硬件层面的根本提速，但需要全总线设备都支持 FD，且 DroneCAN over CAN-FD 需要对应配置。

7. **分总线**：如果负载确实高（比如几十个电机 + 高频 ESC 遥测），把设备分到两路 CAN（`CAN_D1_*` / `CAN_D2_*`），物理隔离流量。Pixhawk6X/Cube 等多数板子有两路 CAN。13 个舵机单路已绰绰有余，一般不需要。

8. **用事件触发代替周期回传**：某些厂商舵机支持"值变化超过阈值才上报"，可显著降低 Status 帧频，但这是厂商私有功能。

### 9.6 给你的 13 舵机场景的建议配置

- `CAN_D1_BITRATE` = 1000000（默认 1M，足够）
- `CAN_D1_SRV_RT` = 50（默认，舵机响应好；若想省电可降到 25）
- 舵机侧 Status 回传设 **5–10 Hz**（监控足够），NodeStatus 保持 1 Hz
- 不用的 `SERVOx_FUNCTION` 设 -1
- 总负载约 **8%**，余量很大，无需 CAN-FD 或分总线

---

## 附：关键代码位置索引

| 主题 | 文件:行号 |
|------|-----------|
| 波特率参数 | `libraries/AP_CANManager/AP_CANIfaceParams.cpp:36,44` |
| 波特率生效（CAN init） | `libraries/AP_CANManager/AP_CANManager.cpp:181` |
| 周期消息超时常量 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:280` |
| 主 loop 定时 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:516-575` |
| 舵机周期定时 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:552-553` |
| SRV/NTF/RLY 频率参数 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:126,141,171` |
| 广播 deadline 计算 | `libraries/AP_DroneCAN/AP_Canard_iface.cpp:94` |
| C++ wrapper 默认 priority/timeout | `modules/DroneCAN/libcanard/canard/publisher.h:78-79` |
| DSDL 消息 ID/签名 | `build/<board>/modules/DroneCAN/libcanard/dsdlc_generated/include/*.h` |
| CAN ID 组装（广播） | `modules/DroneCAN/libcanard/canard.c:217` |
| CAN ID 组装（匿名） | `modules/DroneCAN/libcanard/canard.c:212` |
| CAN ID 组装（服务） | `modules/DroneCAN/libcanard/canard.c:370` |
| CAN ID 位域提取宏（source/dest/priority） | `modules/DroneCAN/libcanard/canard.c:43-49` |
| 服务帧 dest 检查（非本机丢弃） | `modules/DroneCAN/libcanard/canard.c:420-424` |
| 飞控 NodeID 参数/默认 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:84,103` |
| DNA Server / MAX_NODE_ID | `libraries/AP_DroneCAN/AP_DroneCAN_DNA_Server.cpp:39,121` |
| 各消息 priority 设定 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:411-485` |
| 舵机 ArrayCommand 发送 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:734-788`（actuator_id 在 760） |
| 电机 RawCommand 发送 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:826` 起 |
| ESC_OFFSET 参数说明 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:145` |
| 私有协议选项位 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:131` |
| RX 取帧 processRx | `libraries/AP_DroneCAN/AP_Canard_iface.cpp:329,371` |
| RX 协议层过滤 | `modules/DroneCAN/libcanard/canard.c:403-460` |
| should_accept 回调注册 | `libraries/AP_DroneCAN/AP_Canard_iface.cpp:66,183-189` |
| 订阅注册表 accept_message | `modules/DroneCAN/libcanard/canard/handler_list.h:59-73` |
| 分发表 handle_message | `modules/DroneCAN/libcanard/canard/handler_list.h:78-92` |
| decode + 回调 | `modules/DroneCAN/libcanard/canard/subscriber.h:77-89` |
| 各驱动订阅注册入口 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:370-407` |
| 舵机 Status 订阅/回调 | `libraries/AP_DroneCAN/AP_DroneCAN.h:332-333`、`AP_DroneCAN.cpp:1383` |
| actuator.ArrayCommand 发送 | `libraries/AP_DroneCAN/AP_DroneCAN.cpp:734-788` |
