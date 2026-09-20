# DroneCAN 自定义 DSDL 指南与多方对齐清单

> 代码版本对应：ArduPlane 4.6.x（分支 `qhz/plane_4.6.3`），所有行号、文件路径、符号名均基于当前工作树与构建产物实际验证。
>
> 配套阅读：`docs/uav_can.md`（DSDL 与 CAN ID 的分层对应、收发五层过滤、最小配置清单）。本文聚焦"自定义 DSDL 怎么写、怎么编进构建、多方不在同一仓库时如何对齐"。

---

## 0. 一句话结论

- **`data_type_id`（DTID）写在 `.uavcan` 文件名前缀里**（如 `20011.BatteryPeriodic.uavcan`），不是写在文件内容里。构建时 `dronecan_dsdlc` 读文件名得到 DTID，算出 64 位 `signature`，三者一起生成进 C 头文件。
- **多方对齐的本质是"同一个 DTID 数字 + 同一份字段结构 → 同一个 signature"**。CAN 总线上靠这两个值匹配，不靠符号名。
- **命名空间路径之所以要告知对端**：它决定消息全名（`myco.equipment.actuator.MyCmd`），而 `signature = CRC(全名 + 字段布局)`，路径错了 signature 就变了，运行时对端收到的帧会被 `should_accept` 拦掉（`uav_can.md` §5）。次要原因是让双方生成的符号名一致，减少踩坑。
- 光给 `.uavcan` 文件不够，要把"命名空间路径 + DTID + 收发角色 + 频率 + 波特率 + NodeID 方案 + 字段语义"一起交付。文末有一张对齐表，各方填写即可对齐。

---

## 1. DSDL 在 ArduPilot 仓库里的位置

| 路径 | 性质 | 说明 |
|------|------|------|
| `modules/DroneCAN/DSDL/` | 子模块（`0856687`） | 标准 + 各厂商 DSDL 源定义。`uavcan/`（标准协议集）、`dronecan/`（DroneCAN 扩展：remoteid/sensors）、`ardupilot/`（APM 厂商私有）、`com/`（hobbywing/himark/volz…）、`cuav/`、`mppt/`。**勿手动改动，同步用 `./waf submodulesync`。** |
| `modules/DroneCAN/dronecan_dsdlc/` | 编译器子模块 | `dronecan_dsdlc.py`，把 `.uavcan` 编译成 C 头文件。 |
| `libraries/AP_DroneCAN/dsdl/` | **厂商自定义槽位** | README 明示 "This directory can be used for vendor specific DSDL for ArduPilot"。你自己的 DSDL 放这里，跟代码一起提交，不动子模块。 |
| `build/<board>/modules/DroneCAN/libcanard/dsdlc_generated/include/` | 构建产物 | 编译生成的 `*.h`，每个消息一个头文件。 |

### 构建怎么把 DSDL 编进去

顶层 `wscript:743`（带 CAN 的板子，非 AP_Periph）：

```python
bld(
    features='dronecangen',
    source=bld.srcnode.ant_glob(
        'modules/DroneCAN/DSDL/[a-z]* libraries/AP_DroneCAN/dsdl/[a-z]*',
        dir=True, src=False),
    output_dir='modules/DroneCAN/libcanard/dsdlc_generated/',
    ...
)
```

要点：

- glob 模式 `modules/DroneCAN/DSDL/[a-z]*` 和 `libraries/AP_DroneCAN/dsdl/[a-z]*` 会**自动把两处所有小写字母开头的子目录**喂给 dsdlc。
- **所以你在 `libraries/AP_DroneCAN/dsdl/` 下新建一个 `myco/` 命名空间目录，下一次 `./waf` 就会自动编进去**，无需改任何构建脚本。
- `dronecangen` 任务定义在 `Tools/ardupilotwaf/dronecangen.py`，编译器目录在 `:86` 配置为 `modules/DroneCAN/dronecan_dsdlc/`。

---

## 2. data_type_id 是怎么从 DSDL 进入 CAN ID 的（精确链路）

这是 `uav_can.md` §4 的补充，把"DS­DL 文件名 → 生成头 → libcanard 组 CAN ID"的每一步坐实。

### 2.1 第一步：DTID 写在文件名，dsdlc 读进生成头

`.uavcan` 文件名格式是 **`<DTID>.<MessageName>.uavcan`**，DTID 是文件名前缀，不在文件正文里。以 `ardupilot/equipment/power/20011.BatteryPeriodic.uavcan` 为例，构建后实测生成的头（`build/<board>/.../ardupilot.equipment.power.BatteryPeriodic.h`）头部有：

```c
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_MAX_SIZE   125
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_SIGNATURE  (0xF012494E97358D2ULL)
#define ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_ID        20011
...
BROADCAST_MESSAGE_CXX_IFACE(
    ardupilot_equipment_power_BatteryPeriodic,
    ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_ID,        // ← DTID 从文件名来
    ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_SIGNATURE, // ← 由全名+字段算出
    ARDUPILOT_EQUIPMENT_POWER_BATTERYPERIODIC_MAX_SIZE);
```

- **`_ID`（20011）**：直接来自文件名前缀，是这条消息在总线上的类型号，全总线唯一、固定。
- **`_SIGNATURE`**：64 位数据结构 CRC，由 **消息全名 + 字段布局** 计算而来（见 §3）。用于多帧传输 CRC 和版本兼容校验，不放进 CAN ID。
- **`_MAX_SIZE`（125）**：按字段 bit-packing 算出的最大序列化字节数。

标准消息同理：`uavcan/equipment/actuator/1010.ArrayCommand.uavcan` → `UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID 1010`、`_SIGNATURE (0xD8A7486238EC3AF3ULL)`。

### 2.2 第二步：libcanard 把 DTID 编进 29 位 CAN ID

飞控发消息时构造 `CanardTransfer{ data_type_id=20011, priority, payload }`，`modules/DroneCAN/libcanard/canard.c:217` 组 CAN ID：

```c
can_id = ((uint32_t) transfer_object->priority << 24U)        // bit 24-28: priority
       | ((uint32_t) transfer_object->data_type_id << 8U)    // bit 8-23:  DTID ← 这里
       | (uint32_t) canardGetLocalNodeID(ins);                // bit 0-6:   source NodeID
```

29 位扩展帧 ID 布局（消息广播帧）：

```
 31  29 28      24 23                  8 7   0
  ┌──┬──────────┬──────────────────────┬──┬─────┐
  │  │ priority │    data_type_id      │0 │ src │
  │  │  (5bit)  │       (16bit)        │  │node │
  └──┴──────────┴──────────────────────┴──┴─────┘
```

接收侧对称：CAN 控制器收帧 → libcanard 从 CAN ID 的 bit8–23 抠出 `data_type_id` → 查订阅表（`should_accept`，`handler_list.h:59`）决定收不收 → 命中后用对应消息的 `_decode` 反序列化。**所以 DTID 就是 CAN ID 里"识别这是什么消息"的字段，它源自 `.uavcan` 文件名。**

### 2.3 signature 的作用（与 DTID 互补）

DTID 解决"这是什么消息"，signature 解决"版本对不对得上"：

- 多帧传输（payload > 8B）时，libcanard 把 `data_type_signature` 参与 transfer CRC 计算（`canard.c`，`uav_can.md` §4.2）；
- 接收方在 `should_accept` 时回填 signature 供校验（`handler_list.h:59` 的 `signature = entry->signature`）；
- **字段一改，signature 就变，旧版本节点算出的 CRC 对不上，帧被丢弃**——这是 DroneCAN 的版本兼容机制。

所以多方对齐要同时保证 **DTID 一致 + signature 一致**。下一节解释为什么路径会影响 signature。

---

## 3. "命名空间路径"到底怎么影响对齐（核心答疑）

> 先纠正一个常见误解：**对齐的真正红线不是"路径一样"，而是"signature 一样"。** 但 signature 的计算输入里包含 full_name，而 full_name 由路径决定，所以路径是影响 signature 的因素之一。本节把这条因果链彻底讲清。

### 3.1 全名怎么从路径来——只认"从命名空间根往下的相对结构"

决定消息全名的是 `parser.py:418` 的 `_namespace_from_filename()`，算法（`parser.py:419-429`）：

1. 取你**传给 dsdlc 的那个目录**的最后一个分量当根命名空间（如 `myco/` → 根 `myco`）；
2. 取文件相对该根目录的子路径，`/` 换成 `.`；
3. 拼成 `full_name = 根命名空间 + 子路径 + '.' + 消息名`（`parser.py:457`）。

关键结论：**它只认"从命名空间根目录往下的相对结构"，不认绝对路径，也不认仓库结构。**

实测（飞控构建把 `libraries/AP_DroneCAN/dsdl/` 下的子目录传给 dsdlc，`wscript:743` glob 的是 `[a-z]*` 子目录）：

| 文件在飞控仓库的绝对路径 | 传给 dsdlc 的目录 | 生成的 `full_name` | 生成头文件名 |
|---|---|---|---|
| `…/libraries/AP_DroneCAN/dsdl/ardupilot/equipment/power/20011.BatteryPeriodic.uavcan` | `…/libraries/AP_DroneCAN/dsdl/ardupilot` | `ardupilot.equipment.power.BatteryPeriodic` | `ardupilot.equipment.power.BatteryPeriodic.h` |

对端在自己仓库怎么做才能得到同样全名？**只要把 `ardupilot/equipment/power/` 这个相对子结构原样保留，再把 `ardupilot/` 这个根目录传给他的 dsdlc 即可**——他放在 `<他的仓库>/任意目录/ardupilot/equipment/power/20011.BatteryPeriodic.uavcan` 都行，绝对路径、上层目录名完全不必和飞控一样。`parser.py:419` 会先 `os.path.abspath` 再按"是否 startswith 传进来的根目录"匹配，匹配上就取相对部分。

> 所以你之前担心的"大家原来的工程目录不一样"——**不影响**。双方工程目录结构可以完全不同，只要"从命名空间根 `myco/`（或 `ardupilot/`）往下的子目录层级"一致即可。

### 3.2 顺带澄清：哪些是源码、哪些是编译产物

- **源码 DSDL 定义**（你和对端真正需要交换的）：
  - `modules/DroneCAN/DSDL/` —— 标准 + 各厂商 DSDL，子模块（勿手改）；
  - `libraries/AP_DroneCAN/dsdl/` —— ArduPilot 给你预留的厂商自定义槽位，README 就一句 "This directory can be used for vendor specific DSDL for ArduPilot"，是源码、随你的代码提交。
- **编译产物**（`build/<board>/.../dsdlc_generated/`）—— dsdlc 生成的 `*.h/*.c`，飞控构建时自动产出，**对端不需要、也不该拿这个**，对端拿 `.uavcan` 源文件自己跑 dsdlc 生成。

### 3.3 signature 由全名计算——这就是路径影响对齐的根因

`parser.py:275` 这一行是决定性的：

```python
txt.write(self.full_name + '\n')   # full_name 是 signature 计算输入的第一行
```

完整链路（`parser.py:296-325`）：

```
get_data_type_signature()                           # :309
  └─ Signature(get_dsdl_signature())               # :316
       └─ get_dsdl_signature() = compute_signature( # :296,303
              get_dsdl_signature_source_definition())
            └─ source_definition 第一行 = full_name  # :275  ← 路径从这里进来
               后续行 = 字段定义
```

即 **signature = CRC64( full_name + 字段布局 )**。后果：

- 同样字段，放在不同层级路径下 → full_name 不同 → **signature 必然不同**；
- signature 不同 → 多帧传输时双方算的 transfer CRC 对不上 → 对端 `should_accept` 校验失败 → **帧被丢弃**，即便 DTID 一样也不通。

**所以"signature 一致就行"这个判断是对的——但 signature 一致的充分条件之一就是 full_name 一致，而 full_name 一致要求"命名空间根往下的子结构"一致。** 两者不是矛盾，是因果：路径对齐是因，signature 一致是果。

### 3.4 三种达成 signature 一致的方式 + OVERRIDE_SIGNATURE 怎么处理

| 方式 | 做法 | 代价 | 适用场景 | 推荐度 |
|------|------|------|----------|:--:|
| **A. 相对子结构对齐** | 双方都保留 `myco/equipment/actuator/` 子结构，把根 `myco/` 传给各自 dsdlc | 无，最自然 | **新建消息** | ★★★ |
| **B. OVERRIDE_SIGNATURE 写死** | `.uavcan` 里写 `OVERRIDE_SIGNATURE 0x...`（`parser.py:663-664` 读取，`:301-302` 生效，绕过 full_name 计算） | 改字段后 signature 不自动更新，需手动同步，易出错 | **对接已量产、固件烧死的老设备** | ★ 仅兜底 |
| **C. 不管路径只比对 signature** | 各自随便放，只比对签名值 | 路径不一致时 signature 根本不会一致 | — | ✗ 不成立 |

#### himark 为什么用 B（理解了就知道自己该不该用）

`com/himark/servo/2018.ServoCmd.uavcan` 全文只有 4 行有效内容，注释自己交代了原因：

```python
# it is not recommended that anyone implement this message.
# It is here as a placeholder for existing Himark servos

# signature from original file location
OVERRIDE_SIGNATURE 0x5D09E48551CE9194

uint10[<=17] cmd
```

场景是：himark 舵机**已经量产、固件已烧死**，那批舵机固件是按 himark 原始仓库里的全名/字段算的 signature。ArduPilot 把这条消息挪进自己仓库后，目录路径变了 → 全名变了 → 重算的 signature 和老舵机对不上 → 老舵机收到的帧 CRC 校验失败。所以用 `OVERRIDE_SIGNATURE` 把"老 signature"写死，强行让 ArduPilot 算出的 signature 和那批烧死的固件一致。

**判据：你是在对接"对方固件已经动不了"的存量设备吗？**

- 是（存量老设备、固件不能改）→ 必须 B，把对方固件里那个 signature 抄过来写死；
- 否（双方都在开发、固件都能改）→ 用 A，不要碰 OVERRIDE_SIGNATURE。

#### 用了 OVERRIDE_SIGNATURE（方式 B）时怎么处理对齐

1. **signature 来源**：从对方现存固件/对方跑出的 dsdlc 产物里**抄**那个值，写进双方各自 `.uavcan` 的 `OVERRIDE_SIGNATURE` 行。**必须一字不差。**
2. **DTID 仍要一致**：OVERRIDE_SIGNATURE 只覆盖 signature，DTID 还是来自文件名前缀，这条照旧对齐。
3. **字段结构仍要一致**：OVERRIDE_SIGNATURE 只让 signature 不随字段变，但收方 decode 还是按字段 bit-packing 解析——**字段不一致会解出垃圾值**。所以 `uint10[<=17] cmd` 这种字段定义双方必须一致，不能因为"signature 反正写死了"就乱改字段。
4. **危险点**：正因为 signature 被写死、不再随字段自动更新，**改了字段不会被发现**——双方字段早就不一致了，signature 还显示"一致"，给假象。所以用 B 必须在文件头注释里写清"字段不得改，改了要同步更新 OVERRIDE_SIGNATURE 并重新对齐"。
5. **对齐表填法**：signature 行不再"各自跑 dsdlc 算出来比对"，而是"双方 .uavcan 里 OVERRIDE_SIGNATURE 值必须相同"。自检命令改成 `grep OVERRIDE_SIGNATURE *.uavcan` 互相比对，而不是 `show_data_type_info.py`（后者对 B 方式仍会显示写死的值，能比对，但更要看 .uavcan 源行）。

#### 不用 OVERRIDE_SIGNATURE（方式 A，绝大多数情况）

什么都不写，让 dsdlc 自动算。对齐表 signature 行就是"双方各跑 `show_data_type_info.py -c <myco根>` 比对"，一致即对齐。改了字段 signature 自动更新，双方一比对就知道字段有没有同步——这就是 A 比 B 安全的地方。

### 3.5 次要原因：符号名一致，省踩坑

即便单帧消息不依赖 signature 的多帧 CRC，双方生成的 C 符号名（`myco_equipment_actuator_MyCmd`）也得对得上，否则代码片段粘进去编译不过。路径相对结构一致 → full_name 一致 → 符号名一致。

### 3.5b 消息全名（full_name）到底起什么作用（三处）

| # | 作用 | 阶段 | 代码位置 | 说明 |
|---|------|------|----------|------|
| 1 | **算 signature 的第一行输入** | 编译期 | `parser.py:275` `txt.write(self.full_name + '\n')` | full_name 进 CRC64 → signature → 多帧传输校验。**这是全名"影响运行时"的唯一通道**（通过 signature 间接起作用，全名字符串本身不上总线） |
| 2 | **跨文件类型引用解析** | 编译期 | `parser.py:461-490` `_locate_compound_type_definition` | 一个 DSDL 里引用另一个类型（如 `ArrayCommand` 里的 `Command[<=15]`）时，dsdlc 靠 full_name 跨目录找到被引用类型的定义（`:489` 按 `full_typename == fn_full_typename` 匹配）。这就是 `actuator/` 目录里同时有 `1010.ArrayCommand.uavcan` 和 `Command.uavcan` 的原因 |
| 3 | **生成符号名/头文件名** | 编译期 | `dronecan_dsdlc.py:86` 用 `full_name` 做 key | 决定 C 符号 `myco_equipment_actuator_MyCmd` 和头文件名 `myco.equipment.actuator.MyCmd.h`，影响代码可粘连性 |

> **关键结论：全名字符串本身不会上 CAN 总线。** 总线上跑的只有 DTID（在 29 位 CAN ID 里）和 signature（参与多帧 transfer CRC）。全名只在**编译期**起作用，作用 1 通过 signature 间接影响运行时收发校验，作用 2/3 只影响编译产物。所以"全名对不对"之所以重要，是因为作用 1 会传导到 signature——一旦 signature 对不上，运行时帧被丢弃。

### 3.6 严格结论

> **协议层（CAN 总线）硬红线 = DTID 一致 + signature 一致。** 满足这两条，绝对路径、仓库结构、实现语言都可以不同，各编各的运行时能通。
>
> **达成 signature 一致的推荐手段 = 命名空间根往下的相对子结构一致**（方式 A）。交付时告诉对端"把 `myco/equipment/actuator/MyCmd.uavcan` 保留这个相对子结构，把 `myco/` 目录传给你的 dsdlc"即可，不必关心他放哪个仓库、哪层目录。
>
> **对齐自检**：双方各跑 `show_data_type_info.py -c <各自的 myco 根目录>`，比对输出的 signature 列。只要 signature 列一致，即确认对齐成功——这才是最终判据。

---

## 4. 怎么自定义一条 DSDL 消息（step by step）

### Step 1：申请命名空间 + DTID

- 命名空间：用你的厂商/项目名，如 `myco`。
- DTID 范围（`modules/DroneCAN/DSDL/README.md`）：
  - 厂商私有**广播消息**：`[20000, 21000)`
  - 厂商私有**服务消息**：`[200, 256)`
- 查重：`python3 modules/DroneCAN/DSDL/list_ids.py` 列出当前已占号，避开冲突。这个脚本（`:11`）解析 `['uavcan','com','ardupilot','dronecan']` 四个命名空间，按 DTID 排序列出所有消息。注意它默认不含你的自定义目录，需手动确认。

### Step 2：建目录 + 写文件

在 `libraries/AP_DroneCAN/dsdl/` 下建你的命名空间目录。注意（见 §3.1）飞控构建把 `libraries/AP_DroneCAN/dsdl/` 下的 `[a-z]*` 子目录传给 dsdlc，所以 `myco/` 就是根命名空间，往下子结构决定全名：

```
libraries/AP_DroneCAN/dsdl/
└── myco/                        ← 根命名空间（dsdlc 从这往下数路径）
    └── equipment/
        └── actuator/
            ├── 20050.MyCmd.uavcan        # 全名 = myco.equipment.actuator.MyCmd
            └── 20051.MyStatus.uavcan     # 全名 = myco.equipment.actuator.MyStatus
```

> 交付给对端时，告诉对方"保留 `myco/equipment/actuator/` 这个相对子结构，把 `myco/` 目录传给你的 dsdlc"即可。对端放自己仓库的哪个绝对路径都无所谓（§3.1 已验证）。

`.uavcan` 文件格式（参照 `modules/DroneCAN/DSDL/ardupilot/equipment/power/20011.BatteryPeriodic.uavcan`）：

```python
# MyCmd - 自定义舵机/作动器命令示例
# 注释行以 # 开头
# 字段格式：类型  名称   # [单位] : 描述

uint8[<=20] device_name     #      : 设备名，0 terminated
float16 position            # [rad] : 目标位置
float16 max_velocity        # [rad/s] : 最大速度，0 表示不限制

# 常量/枚举（编译成宏）
uint8 MODE_IDLE   = 0
uint8 MODE_ACTIVE = 1
uint8 MODE_HOLD   = 2

uint8 mode
```

字段类型速查：

| 类型 | 说明 |
|------|------|
| `uint8/16/32/64` `int8/16/32/64` | 定长整数 |
| `float16` `float32` | 浮点；`float16` 是 2 字节，省带宽（标准 actuator.Command 就用 float16） |
| `bool` | 布尔 |
| `T[<=N]` | 动态数组，最多 N 个；消息变长，可能多帧拆分 |
| `T[N]` | 定长数组 |
| `voided` / `void3` | 填充位，占位不传值 |

### Step 3：编译验证

```sh
./waf configure --board <board>     # 换板或改选项才需重配
./waf plane                          # 自动跑 dronecangen
```

检查生成：

```sh
ls build/<board>/modules/DroneCAN/libcanard/dsdlc_generated/include/ | grep myco
# 应出现 myco.equipment.actuator.MyCmd.h
grep "_ID\|_SIGNATURE\|_MAX_SIZE" \
  build/<board>/modules/DroneCAN/libcanard/dsdlc_generated/include/myco.equipment.actuator.MyCmd.h
```

也可单测编译器（不构建整个固件）：

```sh
python3 modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py \
    -O/tmp/dsdl_out \
    modules/DroneCAN/DSDL/uavcan \
    libraries/AP_DroneCAN/dsdl/myco
```

### Step 4：飞控侧使用

参照 `libraries/AP_DroneCAN/AP_DroneCAN.cpp` 里 hobbywing/himark 私有消息的用法（`uav_can.md` §7.5）：

- 发送：声明 `Publisher<myco_equipment_actuator_MyCmd>`，填结构体字段，`broadcast(msg)`；
- 接收：`subscribe_msgs()` 里 `allocate_sub_arg_callback` 注册回调，回调里 `myco_equipment_actuator_MyCmd_decode` 反序列化。

机制与标准消息完全一致，DTID 和 signature 已在生成头里，libcanard 自动处理 CAN ID 组装与五层过滤（`uav_can.md` §4、§5）。

---

## 5. 对外沟通要交付什么（光给 DSDL 不够）

DSDL 只定义"消息长什么样"，不含波特率、周期、收发角色、NodeID。完整交付清单见 `uav_can.md` §8 最小配置清单，这里按"交付物"重列：

| # | 交付物 | 必要性 | DSDL 里有没有 | 说明 |
|---|--------|:--:|:--:|------|
| 1 | `.uavcan` 源文件 | ✅必须 | 是 | 消息字段定义，对端跑 dsdlc 生成编解码代码 |
| 2 | 命名空间目录路径 | ✅必须 | 否（隐式） | 决定全名 → signature → 能否对齐。见 §3 |
| 3 | DTID | ✅必须 | 是（文件名） | 编进 CAN ID，总线识别用。虽在文件名里，要明确声明归属私有命名空间 |
| 4 | signature | 自动 | 是（生成） | dsdlc 算出，写进生成头。要对端知道"字段一改这个值变" |
| 5 | 收发角色与方向 | ✅必须 | 否 | 谁发谁收，如"飞控广播 MyCmd、外设回传 MyStatus" |
| 6 | 发送周期/频率 | ✅推荐 | 否 | DroneCAN 周期是应用层软件定时，不在 DSDL（`uav_can.md` §3） |
| 7 | 波特率 | ✅必须 | 否 | 默认 1Mbps，全网一致（`uav_can.md` §2） |
| 8 | NodeID 方案 | ✅必须 | 否 | 飞控默认 10；外设静态配号 or DNA 动态分配（`uav_can.md` §6） |
| 9 | priority | 推荐 | 否 | 默认 MEDIUM，发送方单边设定，不需对端配（`uav_can.md` §6.2） |
| 10 | 字段语义/单位/取值范围 | ✅必须 | 部分（注释） | float16 量化精度、PWM 1000–2000 还是归一化 [-1,1] 等 |
| 11 | 版本兼容约定 | 推荐 | 否 | 私有消息改字段使 signature 变，约定加字段用新 ID 还是向后兼容 |

---

## 6. 多方对齐清单表（各方填写即可对齐）

> 用法：飞控方（A）和外设方（B）/ 其他节点方各自填一张，逐行比对。**打 ★ 的是协议层硬红线，不一致则总线不通；其余是工程约定。**

### 6.1 消息定义对齐表

| 字段 | 飞控方(A)填写 | 外设方(B)填写 | 一致性要求 |
|------|---------------|---------------|-----------|
| ★ 消息全名 | `myco.equipment.actuator.MyCmd` | _________ | **必须一致**（决定 signature，见 §3.5b） |
| ★ DTID | 20050 | _________ | **必须一致**（编进 CAN ID） |
| ★ data_type_signature | `0x____________` | `0x____________` | **必须一致**。填法分两情况：**方式 A**（自动算）= 各跑 `show_data_type_info.py` 比对；**方式 B**（对接老固件）= 抄对方现存 signature 写进双方 `.uavcan` 的 `OVERRIDE_SIGNATURE` 行（见 §3.4） |
| signature 来源方式 | A 自动 / B OVERRIDE_SIGNATURE | A 自动 / B OVERRIDE_SIGNATURE | 双方必须同方式 |
| ★ 字段结构 | 见 .uavcan | 见 .uavcan | **必须一致**（字段决定 decode，方式 B 下 signature 不再反映字段变化，更要靠人工比对字段） |
| 命名空间相对子结构 | `myco/equipment/actuator/` | `myco/equipment/actuator/` | 方式 A **必须一致**（决定全名→signature）；方式 B 可不严格一致（signature 已写死）但建议一致 |
| 消息/服务类型 | 广播消息 | _________ | 一致（广播 vs 服务，CAN ID 布局不同） |
| `_MAX_SIZE` | ____ B | ____ B | 一致（dsdlc 算出） |

### 6.2 运行参数对齐表

| 参数 | 飞控方(A)填写 | 外设方(B)填写 | 一致性要求 |
|------|---------------|---------------|-----------|
| ★ 波特率 | `CAN_D1_BITRATE=1000000` (1Mbps) | _________ bps | **全网必须一致** |
| ★ 飞控 NodeID | `CAN_D1_NODE=10` | — | 全网唯一 |
| ★ 外设 NodeID | — | _________（静态 / DNA 分配） | 全网唯一，1–125 |
| 发送方 | 飞控 | _________ | 谁发谁收要明确 |
| 接收方 | _________ | 飞控 | 同上 |
| 发送周期 | `CAN_D1_SRV_RT=50` Hz | _________ Hz | 不强制同值，但要满足收方控制环需求 |
| 回传周期 | _________ Hz | _________ Hz（建议 5–10） | 监控用，不强制 |
| priority | HIGH（代码硬编码） | _________ | 不需对端配，随帧走；约定了更合理 |
| timeout_ms | 2（周期消息） | _________ | TX 队列超时丢弃，非周期 |

### 6.3 字段语义对齐表

| 字段名 | 类型 | 单位 | 取值范围 | 量化说明 | A方语义 | B方语义 |
|--------|------|------|----------|----------|---------|---------|
| position | float16 | rad | [-π, π] | float16 精度 ~3 位有效 | _________ | _________ |
| mode | uint8 | — | 0/1/2 | 枚举见常量 | _________ | _________ |
| device_name | uint8[<=20] | — | ASCII | 0 terminated | _________ | _________ |

### 6.4 版本管理

| 项 | 约定 |
|----|------|
| 当前版本号 | v1.0 |
| signature 值 | `0x____________` |
| signature 来源方式 | ☐ A 自动计算（新建消息） / ☐ B OVERRIDE_SIGNATURE（对接老固件，见 §3.4） |
| 字段变更策略 | ☐ 加字段用新 DTID（推荐，老消息不动） / ☐ 原地改字段（双方同步升级） |
| 兼容性测试 | 方式 A：双方跑 `show_data_type_info.py` 比对 signature；方式 B：`grep OVERRIDE_SIGNATURE *.uavcan` 比对源行 + 人工比对字段 |

对齐自检命令（分方式）：

**方式 A（自动计算，新建消息）**——双方各跑，比对 signature 列：

```sh
# 飞控方（在 ardupilot 仓库根）
python3 modules/DroneCAN/libcanard/show_data_type_info.py -c libraries/AP_DroneCAN/dsdl/myco

# 外设方（在自己仓库，把 dsdl 目录路径替换为自己的）
python3 <path>/show_data_type_info.py -c <自己的 dsdl/myco 目录>
```

输出列出 `Full Data Type Name | DDTID | Type Signature | Max Bit Len`，逐行比对 signature。签名一致 = 全名+字段都对齐了。`show_data_type_info.py` 支持 `-c`（`:17`），按目录路径算全名（§3.1）。

**方式 B（OVERRIDE_SIGNATURE，对接老固件）**——比对 `.uavcan` 源行 + 人工核对字段：

```sh
# 双方各自
grep -r "OVERRIDE_SIGNATURE" <各自的 dsdl/myco 目录>/*.uavcan
# 输出的 0x... 值必须一字不差相同
```

⚠️ 方式 B 下 signature 不再随字段自动更新，`show_data_type_info.py` 显示的也是写死的值，**无法发现字段不一致**。必须额外人工比对 `.uavcan` 字段定义。

---

## 附：关键代码位置索引

| 主题 | 文件:行号 |
|------|-----------|
| DSDL 源定义根目录 | `modules/DroneCAN/DSDL/` |
| ArduPilot 厂商私有 DSDL | `modules/DroneCAN/DSDL/ardupilot/` |
| 自定义 DSDL 槽位 | `libraries/AP_DroneCAN/dsdl/` |
| dsdlc 编译器 | `modules/DroneCAN/dronecan_dsdlc/dronecan_dsdlc.py` |
| 全名解析（full_name 作 key） | `dronecan_dsdlc.py:81,86` |
| 生成头里的 ID/SIGNATURE/MAX_SIZE | `build/<board>/.../dsdlc_generated/include/*.h`（如 `ardupilot.equipment.power.BatteryPeriodic.h:7-9`） |
| 构建注入 DSDL 目录（glob） | `wscript:743`（带CAN）/ `wscript:755`（AP_Periph） |
| dronecangen 任务定义 | `Tools/ardupilotwaf/dronecangen.py:51-79` |
| dsdlc 目录配置 | `Tools/ardupilotwaf/dronecangen.py:86` |
| CAN ID 组装（DTID<<8） | `modules/DroneCAN/libcanard/canard.c:217` |
| CAN ID 位域提取宏 | `modules/DroneCAN/libcanard/canard.c:43-49` |
| ID 分配规则说明 | `modules/DroneCAN/DSDL/README.md` |
| 厂商私有 ID 范围说明 | `modules/DroneCAN/DSDL/ardupilot/README.md` |
| 列出所有消息按 DTID 排序 | `modules/DroneCAN/DSDL/list_ids.py` |
| signature/DTID 查询工具 | `modules/DroneCAN/libcanard/show_data_type_info.py` |
