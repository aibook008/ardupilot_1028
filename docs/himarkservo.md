# Himark 舵机：代码落点 + GUI Tool 下发操作

> 协议对照见 `docs/df200-uavcan.md`（PDF ↔ DSDL ↔ LSB-first）。
>
> 本文讲两件事：
> 1. 仓库里 Himark 相关代码在哪、缺什么
> 2. 用 DroneCAN GUI Tool 现成 Console **手动下发 `ServoCmd`** 的操作步骤

---

## 0. 一句话结论

- 协议定义：`modules/DroneCAN/DSDL/com/himark/servo/`（`ServoCmd` DTID=2018 / `ServoInfo` DTID=2019）
- 飞控驱动：`libraries/AP_DroneCAN/` —— **发送完整，接收半成品**（handler 有、Subscriber 未挂）
- GUI Tool：**没有 Himark 专用面板**；用 Console 的 `broadcast()` 就能发 `com.himark.servo.ServoCmd`
- **Priority 必须用 24**（PDF 写死 `0x18`）。GUI Tool 默认是 30，不显式传会变成 `0x1E07E201` 而不是官方示例 `0x1807E201`
- 启用飞控侧 Himark 输出：`CAN_Dx_OPTIONS` bit6 `UseHimarkServo`（需 `AP_DRONECAN_HIMARK_SERVO_SUPPORT`）

---

## 1. 代码落点

### 1.1 DSDL

| 文件 | 说明 |
|------|------|
| `modules/DroneCAN/DSDL/com/himark/servo/2018.ServoCmd.uavcan` | 飞控→舵机，`uint10[<=17] cmd`，`OVERRIDE_SIGNATURE 0x5D09E48551CE9194` |
| `modules/DroneCAN/DSDL/com/himark/servo/2019.ServoInfo.uavcan` | 舵机→飞控反馈，`OVERRIDE_SIGNATURE 0xCA8F4B8F97D23B57` |

### 1.2 飞控驱动（`libraries/AP_DroneCAN/`）

| 位置 | 作用 |
|------|------|
| `AP_DroneCAN.h`：`AP_DRONECAN_HIMARK_SERVO_SUPPORT` | 编译开关（默认 `BOARD_FLASH_SIZE>1024`） |
| `AP_DroneCAN.h`：`Options::USE_HIMARK_SERVO = (1U<<6)` | 运行时选项位 |
| `AP_DroneCAN.h`：`himark_out` | `Canard::Publisher<com_himark_servo_ServoCmd>` |
| `AP_DroneCAN.cpp`：`SRV_send_himark()` | 组包发送：PWM→`pulse-1000` 限幅 0–1000 → `broadcast` |
| `AP_DroneCAN.cpp`：调度分支 | `USE_HIMARK_SERVO` 置位走 Himark，否则走标准 `SRV_send_actuator()` |
| `AP_DroneCAN.cpp`：`handle_himark_servoinfo()` | 把反馈写成 CSRV 日志 |

构建特性声明：`Tools/scripts/build_options.py` 的 `HimarkServo` feature。

### 1.3 接收侧缺口（死代码）

Volz 等厂商反馈是三件套：

```cpp
// 处理函数 + ObjCallback + Subscriber
Canard::ObjCallback<...> xxx_cb{this, &AP_DroneCAN::handle_...};
Canard::Subscriber<...> xxx_listener{xxx_cb, _driver_index};
```

Himark 目前**只有** `handle_himark_servoinfo()`，没有 `ObjCallback` / `Subscriber`。
CAN 上的 `ServoInfo` 进不了订阅表 → handler 永远不会被调用。

若要补齐，按 Volz 同款加：

```cpp
#if AP_DRONECAN_HIMARK_SERVO_SUPPORT
    Canard::ObjCallback<AP_DroneCAN, com_himark_servo_ServoInfo>
        himark_servoinfo_cb{this, &AP_DroneCAN::handle_himark_servoinfo};
    Canard::Subscriber<com_himark_servo_ServoInfo>
        himark_servoinfo_listener{himark_servoinfo_cb, _driver_index};
#endif
```

---

## 2. GUI Tool 里有哪些下发方式

仓库：`/home/codyqin/Desktop/git/gui_tool`

| 方式 | 能发指令 | 能发 Himark `ServoCmd` |
|------|:--------:|:----------------------:|
| Console `broadcast(...)` | ✅ | ✅（手动组包） |
| Console `request(...)` | ✅（服务） | ❌（ServoCmd 是消息不是服务） |
| Actuator Panel | ✅ | ❌（发 `ArrayCommand`） |
| ESC Panel | ✅ | ❌（`RawCommand` / Hobbywing） |
| Himark 专用面板 | ❌ 不存在 | ❌ |

**现成可用路径 = Console `broadcast()`。**

---

## 3. 操作步骤（GUI Tool Console）

### 3.1 前置

1. 启动 DroneCAN GUI Tool，接上总线（**1 Mbps**，与舵机一致）。
2. Setup → **Location of custom DSDL definitions** 选：
   ```
   /home/codyqin/Desktop/git/ardupilot/modules/DroneCAN/DSDL/com
   ```
3. 主窗口 **Local Node ID** 设为非 0（例如 `1`）。Anonymous 发不出去。
4. Node Monitor 确认能看到舵机节点（建议 NodeID 从 100 起）。

可选：把 `com`（以及需要的 `px4`）链到默认 vendor 目录，避免每次手选：

```bash
mkdir -p ~/dronecan_vendor_specific_types
ln -sf /home/codyqin/Desktop/git/ardupilot/modules/DroneCAN/DSDL/com \
       ~/dronecan_vendor_specific_types/com
```

### 3.2 确认 DSDL 已加载

打开 Console，执行：

```python
dronecan.TYPENAMES['com.himark.servo.ServoCmd']
dronecan.TYPENAMES['com.himark.servo.ServoInfo']
```

能打印类型对象 = 加载成功。`KeyError` → 回去重选 DSDL 路径并重启。

也可核对 signature（应等于 OVERRIDE 写死值）：

```python
t = dronecan.TYPENAMES['com.himark.servo.ServoCmd']
print(hex(t._data_type_signature), 'dtid', t.default_dtid)
# 期望：0x5d09e48551ce9194  dtid 2018
```

### 3.3 Priority 必须用 24（对齐 PDF）

CAN ID 公式：

```
帧 ID = (Priority << 24) | (DTID << 8) | NodeID
```

| 来源 | Priority | 示例 CAN ID（DTID=2018, node=1） |
|------|----------|----------------------------------|
| Himark PDF | **24** (`0x18`) | **`0x1807E201`** ← 要对齐这个 |
| GUI Tool `broadcast()` 默认 | 30 (`0x1E`) | `0x1E07E201` ← 不传 priority 就会这样 |
| ArduPilot `himark_out` | 8 (`HIGH`) | `0x0807E201` |

**联调一律显式传 `priority=24`**，不要依赖 GUI Tool 默认值。

### 3.4 发一次 ServoCmd（试连通）

```python
msg = dronecan.com.himark.servo.ServoCmd()
msg.cmd = [500, 500, 500, 500, 500]   # 5 个舵机，中位约 500（范围 0–1000）
broadcast(msg, priority=24)            # 必须 24，对齐 PDF 的 0x1807E201
```

说明：

- `cmd[i]` 对应舵机编号 `i`（0–17）
- 中位一般用 `500`；`0` / `1000` 是两端
- 只控前 N 个就填 N 个值，例如 `msg.cmd = [500, 600]`
- 抓包确认 CAN ID 应为 `0x1807E2xx`（`xx` = 本地 Node ID），不是 `0x1E07E2xx`

### 3.5 周期下发（舵机通常需要持续刷新）

厂商建议命令频率 >200 Hz；联调可先用 50–100 Hz：

```python
msg = dronecan.com.himark.servo.ServoCmd()
msg.cmd = [500, 500, 500, 500, 500]
h = broadcast(msg, priority=24, interval=0.01)   # 100 Hz，priority=24
```

停止：

```python
h.remove()
# 或停掉 Console 里所有周期任务：
stop()
```

### 3.6 边发边看反馈

Console：

```python
subscribe(dronecan.com.himark.servo.ServoInfo)
```

或用菜单 **Subscriber** 窗口，类型填：

```
com.himark.servo.ServoInfo
```

正常应看到 `servo_id / pos_cmd / pos_sensor / voltage / current / pcb_temp / motor_temp / error_status`。

### 3.7 推荐安全顺序

1. 先 `subscribe(ServoInfo)`，确认舵机在吐反馈。
2. 发一帧中位：
   ```python
   msg = dronecan.com.himark.servo.ServoCmd()
   msg.cmd = [500]*5
   broadcast(msg, priority=24)
   ```
3. 抓包确认 CAN ID = `0x1807E2xx`；看 `pos_cmd` / `pos_sensor` 是否跟上。
4. 再小幅动一个通道，例如：
   ```python
   msg.cmd = [500, 520, 500, 500, 500]
   h = broadcast(msg, priority=24, interval=0.01)
   ```
5. 测完 `h.remove()` 或 `stop()`，避免一直刷命令。

### 3.8 常见坑

| 现象 | 原因 |
|------|------|
| 抓到 `0x1E07E201` 而不是 `0x1807E201` | 忘了传 `priority=24`，GUI Tool 默认 30 |
| `RuntimeError: Local node is configured in anonymous mode` | Local Node ID 还是 0 |
| `KeyError: com.himark.servo.ServoCmd` | 没加载 `DSDL/com` |
| 发出去舵机不动 | 波特率不是 1M / 命令频率太低 / `cmd` 下标与舵机编号对不上 / Priority 不是 24 |
| ≤5 单帧、≥6 多帧 | 正常；传输层自动拆帧，应用层只管填 `msg.cmd` |

---

## 4. 飞控侧启用（对照）

若要用 ArduPilot 自己发 Himark（不是 GUI Tool）：

1. 板子 flash > 1MB，或构建时打开 `HimarkServo` feature（`AP_DRONECAN_HIMARK_SERVO_SUPPORT`）。
2. 对应 CAN 驱动参数 `CAN_Dx_OPTIONS` 置 bit6 `UseHimarkServo`。
3. 舵机通道仍走正常 `SERVO*` 输出；底层从 `ArrayCommand` 换成 `ServoCmd`，映射为 `pulse-1000` → 0–1000。
4. **Priority**：当前代码 `himark_out.set_priority(CANARD_TRANSFER_PRIORITY_HIGH)` = **8**，和 PDF 的 24 不一致。
   若厂商坚持 24，需改成：
   ```cpp
   himark_out.set_priority(24);  // 对齐 Himark PDF Priority=0x18
   ```
   （`AP_DroneCAN.cpp` 初始化 `himark_out` 附近）

注意：即使飞控在发 `ServoCmd`，在补上 Subscriber 之前，飞控**不会**把 `ServoInfo` 记进 CSRV；看反馈仍用 GUI Tool Subscriber / `dronecan sub`。

---

## 5. 相关文件索引

| 文件 | 说明 |
|------|------|
| `docs/df200-uavcan.md` | PDF ↔ DSDL ↔ LSB-first 协议对照 |
| `docs/dsdl_custom_guide.md` | 自定义 DSDL / OVERRIDE / 多方对齐 |
| `docs/uav_can.md` | 波特率 / 周期 / DTID→CAN ID / 过滤 |
| `modules/DroneCAN/DSDL/com/himark/servo/*.uavcan` | Himark DSDL |
| `libraries/AP_DroneCAN/AP_DroneCAN.{h,cpp}` | 飞控 Himark 收发实现 |
| `/home/codyqin/Desktop/git/gui_tool/` | DroneCAN GUI Tool 源码 |
| `gui_tool/.../main.py` | Console 注入的 `broadcast` / `request` / `subscribe` |
| `gui_tool/.../panels/actuator_panel.py` | 标准 Actuator Panel（非 Himark） |
