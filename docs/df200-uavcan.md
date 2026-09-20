# DF200 Himark 舵机 UAVCAN 协议对照（PDF ↔ DSDL ↔ LSB-first）

> 对应厂商协议：`UAVCAN通讯协议 V1.0 HM-XY-CAN-24001.pdf`（Himark 舵机，基于 UAVCAN v0）。
>
> 配套阅读：
> - `docs/dsdl_custom_guide.md`（自定义 DSDL 怎么写、OVERRIDE_SIGNATURE、多方对齐）
> - `docs/uav_can.md`（波特率 / 周期 / DTID→CAN ID / 收发过滤）
>
> 本文只讲一件事：**这份 PDF 怎么对应到仓库里现成的 `.uavcan`，以及 payload 字节怎么按 LSB-first 还原成字段。**

---

## 0. 一句话结论

- PDF 定义的两条消息就是仓库里现成的：
  - 命令：`modules/DroneCAN/DSDL/com/himark/servo/2018.ServoCmd.uavcan`（DTID=2018）
  - 反馈：`modules/DroneCAN/DSDL/com/himark/servo/2019.ServoInfo.uavcan`（DTID=2019）
- 两条消息都用了 **`OVERRIDE_SIGNATURE`**（方式 B，对接量产固件烧死的老舵机），signature 与 PDF 一字不差：
  - ServoCmd = `0x5D09E48551CE9194`
  - ServoInfo = `0xCA8F4B8F97D23B57`
- payload 字段打包规则是 **UAVCAN v0 的 LSB-first bit packing**（最低有效位先铺），不是"每字节对应一个字段"。
- **≤5 个舵机单帧、≥6 个舵机多帧**的根因是数学：单帧净空间 56 bit ÷ 10 bit/个 = 5.6，最多塞 5 个 `uint10`。
- GUI Tool / dronecan Python 包加载 `.../DSDL/com` 即可订阅 `com.himark.servo.ServoCmd` / `ServoInfo`，OVERRIDE 对加载方透明，无需特殊代码。

---

## 1. PDF 与仓库 DSDL 的对应

### 1.1 两条消息总览

| 方向 | PDF 章节 | DTID | 帧 ID 示例 | DSDL 文件 | 全名 | OVERRIDE_SIGNATURE |
|------|-----------|------|------------|-----------|------|--------------------|
| 飞控 → 舵机 | §3 控制命令 | 2018 (`0x7E2`) | `0x1807E201`（prio=24, src=1） | `2018.ServoCmd.uavcan` | `com.himark.servo.ServoCmd` | `0x5D09E48551CE9194` |
| 舵机 → 飞控 | §2 反馈工作数据 | 2019 (`0x7E3`) | `0x1807E364`（prio=24, node=100） | `2019.ServoInfo.uavcan` | `com.himark.servo.ServoInfo` | `0xCA8F4B8F97D23B57` |

CAN ID 组装（与 `uav_can.md` §4 一致）：

```
帧 ID = (Priority << 24) | (Data Type ID << 8) | (Node ID)
```

### 1.2 运行参数（PDF 约定）

| 参数 | PDF 值 | 说明 |
|------|--------|------|
| 波特率 | 1000 Kb/s | 全网必须一致 |
| 最大舵机数 | 18（编号 0–17） | 见 §3.2 关于 `<=17` 的历史差异 |
| 建议 NodeID | 舵机 0 → Local Node ID=100，其后依次 +1 | 可用编程卡改 |
| 命令频率 | >200 Hz（实测 18 舵机 @1Mbps + 反馈 50Hz 正常） | |
| 反馈频率 | 1/2/5/10(默认)/50 Hz | 编程卡改 |
| Priority | `0x18` = 24 | PDF 写死不可改 |

### 1.3 ServoCmd 字段（PDF §3 ↔ `.uavcan`）

```
# modules/DroneCAN/DSDL/com/himark/servo/2018.ServoCmd.uavcan
OVERRIDE_SIGNATURE 0x5D09E48551CE9194
uint10[<=17] cmd
```

| PDF 描述 | DSDL 写法 | 说明 |
|----------|-----------|------|
| CAN CMD 范围 0–1000 | `uint10` | 10 bit 装 0–1023，够用 |
| 最多 18 个舵机 | `uint10[<=17]` | 动态数组；历史定义是 `<=17`（见 §3.2） |
| 1–5 单帧 / 6–18 多帧 | （不进 DSDL） | 传输层事，libcanard 按 payload 长度自动拆 |

### 1.4 ServoInfo 字段（PDF §2 ↔ `.uavcan`）

```
# modules/DroneCAN/DSDL/com/himark/servo/2019.ServoInfo.uavcan
OVERRIDE_SIGNATURE 0xCA8F4B8F97D23B57

uint5  servo_id        # 0–17
uint12 pwm_input       # 700–2300 us（或 CAN 命令 0–1000）
int16  pos_cmd         # -18000–18000，1 LSB = 0.01°
int16  pos_sensor      # -18000–18000，1 LSB = 0.01°
uint12 voltage         # 0–4095，1 LSB = 0.01 V
uint10 current         # 0–1023，1 LSB = 0.01 A
uint10 pcb_temp        # 0–1023，temp = dat*0.2 - 40 (°C)
uint10 motor_temp      # 0–1023，temp = dat*0.2 - 40 (°C)
uint5  error_status    # 5 bit 状态位（见下）
```

PDF StatusInfo bit 定义：

| Bit | =0 | =1 |
|-----|----|----|
| 0 | 正常 | 初始化异常（上电没收到有效命令 / ADC 未校正 / 参数异常） |
| 1 | 正常 | 工作中控制命令丢失（上电已收到过命令之后才丢） |
| 2 | 正常 | 舵臂失力（收到 CAN 命令 1020） |
| 3 | 正常 | 堵转 |
| 4 | 正常 | 高温（任一温度 ≥ 85°C） |

> 字段顺序 = PDF 解码顺序 = `.uavcan` 从上到下顺序。**顺序错了整包解乱。**

---

## 2. PDF 里哪些进 DSDL、哪些不进

| PDF 内容 | 进不进 DSDL | 去哪 |
|----------|:----------:|------|
| 字段名 / 类型 / 范围 / 单位 | ✅ | `.uavcan` 字段定义 |
| Data Type ID（2018 / 2019） | ✅ | **文件名前缀**，不写正文 |
| signature（`0x5D09...` / `0xCA8F...`） | ✅ | `OVERRIDE_SIGNATURE` 行（方式 B） |
| Priority / Node ID | ❌ | 运行时参数，发送时组 CAN ID |
| 帧尾 tail byte、CRC | ❌ | 传输层，libcanard 自动加 |
| "1–5 单帧 / 6–18 多帧" | ❌ | libcanard 按 payload 长度自动拆 |
| 波特率 / 周期 / 反馈频率 | ❌ | 驱动层 / 应用层（见 `uav_can.md`） |

---

## 3. 为什么用 OVERRIDE_SIGNATURE（方式 B）

Himark 舵机**已经量产、固件烧死**。那批舵机固件按 himark 原始仓库的路径算 signature。ArduPilot 把 `.uavcan` 挪进自己仓库后路径变了 → 自动算会得到不同 signature → 老舵机 CRC 对不上 → 丢帧。

所以从 PDF 抄那个写死的 signature（§3.2.2 / §2.2 的 `crcAddSignature(crc, 0x5D09...)` / `0xCA8F...`）写进 `OVERRIDE_SIGNATURE`，强行对齐。这就是 `dsdl_custom_guide.md` §3.4 的**方式 B**。

| 对齐方式 | 做法 | 适用 |
|----------|------|------|
| A. 相对子结构对齐 | 双方保留同样相对路径，让 dsdlc 自动算 | **新建消息**（推荐） |
| B. OVERRIDE_SIGNATURE | 抄对方固件里的 signature 写死 | **对接存量老设备**（himark 就是） |

方式 B 下注意：
1. signature / DTID / 字段结构三件套仍必须一致。
2. signature 写死后**不再随字段自动更新**——改字段不会被发现，会解出垃圾值还显示"签名一致"。
3. 对齐自检：`grep OVERRIDE_SIGNATURE *.uavcan` 互相比对 + 人工比对字段。

### 3.2 `<=17` vs PDF "18 个舵机" 的历史差异

PDF 反复说"最大支持 18 个舵机，编号 0 到 17"，但 `.uavcan` 写的是 `uint10[<=17]`（最多 17 个元素）。这是 himark 历史定义的固有写法。

- 如果舵机固件就是按 `<=17` 实现的（PDF 示例发到编号 17）→ **保持原样别动**。
- 若强行改成 `<=18`：自动算 signature 会变；即便 OVERRIDE 写死绕过，字段布局也可能和老固件不一致。

---

## 4. LSB-first bit packing（核心）

### 4.1 这是哪一层的规则

| 层 | 规则 | 说明 |
|----|------|------|
| CAN 物理层 | 硬件自己处理 bit stuffing | 和 UAVCAN 无关，应用层不用管 |
| **UAVCAN v0 DSDL 序列化** | **LSB-first** | libuavcan / libcanard 把结构体字段打包成字节流时用这个 |
| CAN ID（29 位） | MSB 起算（priority 在最高位） | 帧头，和 payload 打包是两码事 |

**LSB-first = Least Significant Bit first = 最低有效位先铺。**

### 4.2 用 `uint10` 演示

`cmd = 100`，10 bit = `00 0110 0100`：

```
位编号:  bit9 bit8 bit7 bit6 bit5 bit4 bit3 bit2 bit1 bit0
值:       0    0    0    1    1    0    0    1    0    0
          高位(MSB)                          低位(LSB)
```

- **LSB-first（UAVCAN 用）**：从 bit0 开始往字节流铺 → byte0 = bit0..bit7 = `0110 0100` = `0x64`
- **MSB-first（另一种）**：从 bit9 开始铺 → byte0 = bit9..bit2 = `0001 1001` = `0x19`（完全不同）

### 4.3 为什么"单字节 ≠ 单个 cmd"（除 byte0 凑巧）

5 个 `uint10` = 50 bit，从 bit0 连续铺，每 8 bit 切一字节。**没有任何 cmd "独占"一个字节**（除非前一个 cmd 的位恰好填满字节边界）。

以 `cmd = [100, 200]` 为例（LSB-first）：

```
cmd[0]=100 = 00 0110 0100
cmd[1]=200 = 00 1100 1000

byte0 = cmd[0] 的 bit0..bit7        = 0110 0100 = 0x64   ← 正好是 100 的低 8 位（凑巧）
byte1 = cmd[0] 的 bit8,bit9 (00)    放 bit0,1
      ⊕ cmd[1] 的 bit0..bit5        放 bit2..7
      = 0x20                         ← 既不是 100 也不是 200！
byte2 = cmd[1] 的剩余 4 位 + 后续...
```

实测打包结果：`cmd=[100,200]` → 前 3 字节 = `0x64 0x20 0x03`。

**byte0 之所以能对上单个值**：因为 100 ≤ 255，高 2 位是 0，LSB-first 下低 8 位独立占满 byte0。从 byte1 起每个字节都混着两个相邻 cmd 的片段，单字节对照必然失败——必须按 bit 流每 10 bit 切。

### 4.4 完整 5 舵机示例（cmd = 100,150,200,250,300）

LSB-first 打包后（剥掉 tail byte 的 7 字节 payload）：

```
0x64  0x58  0x82  0x8c  0x3e  0x2c  0x01
```

反向按每 10 bit 切回去，能 100% 还原 `{100,150,200,250,300}`。这就是 libcanard 收发两端干的事。

> PDF 第 4 页抓包图的真实字节可能不是这组（取决于图里实际 cmd 值）。验证方法：拿抓包字节 + 标注 cmd 值，跑一遍 LSB-first 解码，对上 = 规则确认。

### 4.5 手算 / 验证脚本

```python
# 打包：cmd 列表 → 字节流（LSB-first）
def pack_uint10(cmd):
    bits = []
    for v in cmd:
        for i in range(10):
            bits.append((v >> i) & 1)
    # 补 0 到字节边界
    while len(bits) % 8:
        bits.append(0)
    out = []
    for i in range(0, len(bits), 8):
        b = 0
        for j in range(8):
            b |= bits[i + j] << j
        out.append(b)
    return out

# 拆包：字节流 → cmd 列表
def unpack_uint10(data, n):
    bits = []
    for b in data:
        for j in range(8):
            bits.append((b >> j) & 1)
    res = []
    for k in range(n):
        v = 0
        for j in range(10):
            v |= bits[k * 10 + j] << j
        res.append(v)
    return res
```

---

## 5. 为什么 ≤5 单帧、≥6 多帧

一个 CAN 帧 payload **最多 8 字节**。

**单帧（tail byte 的 S=1, E=1）**：
- tail byte 占 1 字节 → **净数据 7 字节 = 56 bit**
- 56 ÷ 10 = 5.6 → **最多 5 个完整 `uint10`**

```
5 个舵机: 5 × 10 = 50 bit ≤ 56 bit  → 单帧 ✅，无 CRC
6 个舵机: 6 × 10 = 60 bit > 56 bit  → 必须多帧 ❌
```

**多帧（6–18 舵机）**：

```
帧1: [CRC 2字节][数据 5字节][tail 1字节]   S=1, E=0
帧2: [数据 7字节][tail 1字节]              S=0, E=0（中间帧）
帧N: [数据 7字节][tail 1字节]              S=0, E=1（最后一帧）
```

- 第 1 帧前 2 字节是 CRC，计算要把 **signature 参与进去**：
  ```c
  crc = 0xFFFF;
  crc = crcAddSignature(crc, 0x5D09E48551CE9194);  // ServoCmd
  crc = crcAdd(crc, payload, len);
  ```
  （PDF §1.2 / §3.2.2，ServoInfo 用 `0xCA8F4B8F97D23B57`）
- tail byte 的 Toggle bit 相邻帧交替，防帧错位。

---

## 6. 接收端解码链路（飞控发 → 舵机收）

以 5 舵机单帧为例：

```
1. CAN 帧到达
   CAN ID = 0x1807E201
   → priority=24, DTID=2018, src node=1

2. 识别消息
   DTID=2018 → 查订阅表 → com.himark.servo.ServoCmd
   取 signature = 0x5D09E48551CE9194（OVERRIDE 写死）

3. 剥传输层
   单帧：剥掉 tail byte（S=1,E=1），无 CRC
   剩 7 字节纯数据给 DSDL 解码层

4. DSDL bit-packing 解码（LSB-first）
   从 bit0 起每 10 bit 切一个数 → cmd[0..4]
   得到结构体：
     cmd_len = 5
     cmd[]   = {100, 150, 200, 250, 300}   // 示例值

5. 应用层
   舵机 Servo_ID=k 执行 cmd[k]（0–1000）
```

**飞控发送侧完全对称**：填结构体 → libcanard 按 `uint10[<=17]` LSB-first 打包 → 加 tail byte → 组 CAN ID → 发。两端代码只管填/读结构体字段，不用手算字节布局。

---

## 7. GUI Tool / 命令行怎么看这两条消息

### 7.1 加载 DSDL

`load_dsdl` 的路径必须指到 **namespace 根目录**（第一层子目录是小写 namespace 名）。对 himark：

```
正确：.../modules/DroneCAN/DSDL/com          ← 里面是 himark/、hobbywing/...
错误：.../modules/DroneCAN/DSDL              ← 会把大写 DSDL 当 namespace，报错
```

DroneCAN GUI Tool：
1. setup 窗口 **"Location of custom DSDL definitions"** 选上面那个 `com` 目录
2. 订阅框填全限定名：
   - `com.himark.servo.ServoCmd`
   - `com.himark.servo.ServoInfo`

命令行：

```bash
dronecan sub com.himark.servo.ServoInfo \
  --dsdl /home/codyqin/Desktop/git/ardupilot/modules/DroneCAN/DSDL/com \
  -d /dev/ttyACM0 -b 1000000
```

### 7.2 让 px4 + com 两套自定义 namespace 默认加载

```bash
mkdir -p ~/dronecan_vendor_specific_types
ln -sf /home/codyqin/Desktop/git/PX4-Autopilot/src/drivers/uavcan/dsdl/px4 \
       ~/dronecan_vendor_specific_types/px4
ln -sf /home/codyqin/Desktop/git/ardupilot/modules/DroneCAN/DSDL/com \
       ~/dronecan_vendor_specific_types/com
```

`dronecan.load_dsdl()` 启动时会自动扫 `~/dronecan_vendor_specific_types/` 下的子目录，GUI Tool 不用每次手选。

### 7.3 OVERRIDE 对 GUI Tool 透明

实测：`dronecan.load_dsdl(.../com)` 后

```
DATATYPES[(2018,1)]._data_type_signature = 0x5d09e48551ce9194  # = OVERRIDE 值
DATATYPES[(2019,1)]._data_type_signature = 0xca8f4b8f97d23b57
```

parser 读到 `OVERRIDE_SIGNATURE` 就用写死的值，读不到就自己算。**加载方（GUI Tool / 脚本）无需任何特殊处理。**

---

## 8. 对齐自检

### 方式 B（himark，OVERRIDE）

```bash
grep -r "OVERRIDE_SIGNATURE" \
  modules/DroneCAN/DSDL/com/himark/servo/*.uavcan
# 输出必须与 PDF 一致：
#   ServoCmd  → 0x5D09E48551CE9194
#   ServoInfo → 0xCA8F4B8F97D23B57
```

再人工比对 `.uavcan` 字段定义与 PDF §2 / §3 字段表（方式 B 下 signature 不再反映字段变化）。

### Python 侧确认

```python
import dronecan
dronecan.load_dsdl('modules/DroneCAN/DSDL/com')
for n in ('com.himark.servo.ServoCmd', 'com.himark.servo.ServoInfo'):
    t = dronecan.TYPENAMES[n]
    print(n, hex(t._data_type_signature), 'port', t.default_dtid)
```

---

## 9. 相关文件索引

| 文件 | 说明 |
|------|------|
| `~/Desktop/git/UAVCAN通讯协议 V1.0 HM-XY-CAN-24001.pdf` | Himark 厂商协议原文 |
| `modules/DroneCAN/DSDL/com/himark/servo/2018.ServoCmd.uavcan` | 命令 DSDL |
| `modules/DroneCAN/DSDL/com/himark/servo/2019.ServoInfo.uavcan` | 反馈 DSDL |
| `docs/dsdl_custom_guide.md` | 自定义 DSDL / OVERRIDE / 多方对齐 |
| `docs/uav_can.md` | 波特率 / 周期 / DTID→CAN ID / 过滤 |
| `docs/dronecan_alignment_tables.md` | 对齐表模板 |

---

## 附：接收链路分层速查

| 层 | 谁负责 | 依据 | 产出 |
|----|--------|------|------|
| CAN ID 里的 DTID | libcanard | 文件名 `2018.` / `2019.` | 识别消息类型 |
| signature 校验 | libcanard | `OVERRIDE_SIGNATURE` | 多帧 transfer CRC |
| tail byte / 帧拆合 | libcanard | UAVCAN v0 传输规则 | 单帧/多帧重组 |
| **bit 打包/拆包** | libcanard | **`.uavcan` 字段 + LSB-first** | 字节 ↔ 字段值 |
| 应用层语义 | 飞控 / 舵机固件 | PDF 注释（0–1000 量化等） | 执行动作 / 上报状态 |
