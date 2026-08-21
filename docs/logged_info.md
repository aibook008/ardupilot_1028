# ArduPilot 日志系统分析

本文档基于 `logs/00000001.BIN` 日志文件分析，记录日志消息类型与代码的对应关系。

## 概述

- **定义的消息类型总数**：197 种
- **实际出现的消息类型**：80 种
- **总记录数**：约 60 万条

## 日志消息按频率分类

### 高频消息（>10000 条）

| 消息 | 数量 | 代码位置 | 说明 |
|---|---|---|---|
| IMU | 114519 | `libraries/AP_InertialSensor/AP_InertialSensor_Logging.cpp` | 惯性测量单元原始数据 |
| ESCX | 76348 | `libraries/AP_Motors/` | 电调扩展数据 |
| ESC | 76348 | `libraries/AP_Motors/` | 电调数据 |
| SIM2 | 57260 | `libraries/AP_Logger/LogStructure.h` | 仿真数据 |
| RATE | 23155 | `libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp` | 角速率控制 |
| ANG | 23155 | `libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp` | 角度控制 |

### 中频消息（1000-10000 条）

| 消息 | 数量 | 代码位置 | 说明 |
|---|---|---|---|
| VIBE | 9542 | `libraries/AP_InertialSensor/` | 振动数据 |
| MAG | 5724 | `libraries/AP_Compass/LogStructure.h` | 磁力计 |
| RCOU/RCIN | 4771 | `ArduPlane/Log.cpp` | RC 输出/输入 |
| NTUN/CTUN | 4771 | `ArduPlane/Log.cpp` | 导航/控制调优 |
| AETR | 4771 | `ArduPlane/Log.cpp` | 副翼/升降/方向舵/油门 |
| XKF1-5, XKQ, XKFS | 3816 | `libraries/AP_NavEKF3/` | EKF3 状态估计 |
| BARO | 3816 | `libraries/AP_Baro/LogStructure.h` | 气压计 |
| QPOS | 2638 | `ArduPlane/quadplane.cpp` | QuadPlane 位置 |
| QTUN | 1790 | `ArduPlane/quadplane.cpp` | QuadPlane 垂直调优 |
| TILT | 1790 | `ArduPlane/tiltrotor.cpp` | 倾转旋翼 |
| PARM | 1596 | `libraries/AP_Logger/` | 参数变更 |
| PSCN/PSCE/PSCD | 1567-1728 | `libraries/AC_PosControl/` | 位置控制 |
| TECS/TEC2/TEC3 | 1010 | `libraries/AP_TECS/` | 总能量控制系统 |
| STAT | 959 | `ArduPlane/Log.cpp:209` | 系统状态 |
| GPS/GPA | 954 | `libraries/AP_GPS/LogStructure.h` | GPS 数据 |
| ATSC | 826 | `libraries/AP_AHRS/LogStructure.h` | AHRS 状态 |
| PIQP/PIQR/PIQY/PIQA | 773 | `ArduPlane/quadplane.cpp` | QuadPlane PID 控制 |
| FILE | 764 | `libraries/AP_Logger/` | 文件传输 |
| XKTV | 752 | `libraries/AP_NavEKF3/` | EKF 电视觉 |
| MOTB | 748 | `libraries/AP_Motors/` | 电机电池 |
| CTRL | 748 | `libraries/APM_Control/` | 控制输出 |
| PIDN/PIDE | 678 | `libraries/APM_Control/` | 导航/高度误差 PID |
| MAV | 570 | `libraries/GCS_MAVLink/` | MAVLink 消息 |

### 低频消息（<500 条）

| 消息 | 数量 | 代码位置 | 说明 |
|---|---|---|---|
| XKV1/XKV2 | 382 | `libraries/AP_NavEKF3/` | EKF 方差 |
| UART | 382 | `libraries/AP_Logger/` | UART 统计 |
| FMTU/FMT | 202/197 | `libraries/AP_Logger/` | 格式定义 |
| TERR | 191 | `libraries/AP_Terrain/LogStructure.h` | 地形数据 |
| DSF | 187 | `libraries/AP_Logger/` | 动态存储格式 |
| XKT | 76 | `libraries/AP_NavEKF3/` | EKF 时间常数 |
| MSG | 40 | `libraries/AP_Logger/` | 文本消息 |
| PM | 38 | `libraries/AP_Logger/` | 性能监控 |
| UNIT | 37 | `libraries/AP_Logger/` | 单位定义 |
| ORGN | 20 | `libraries/AP_AHRS/` | 原点位置 |
| LAND | 20 | `ArduPlane/Log.cpp` | 着陆数据 |
| MAVC | 19 | `libraries/GCS_MAVLink/` | MAVLink 命令 |
| MULT | 15 | `libraries/AP_Logger/` | 乘数定义 |
| CMD | 15 | `ArduPlane/Log.cpp` | 当前命令 |
| **MODE** | **5** | `ArduPlane/Log.cpp:520`, `ArduPlane/system.cpp:324` | **飞行模式切换** |
| MISE | 4 | `ArduPlane/Log.cpp` | 任务项 |
| EV | 2 | `libraries/AP_Logger/` | 事件记录 |
| VER | 1 | `libraries/AP_Logger/` | 版本信息 |
| ARM | 1 | `libraries/AP_Logger/` | 解锁状态 |

## MODE 消息完整调用链

MODE 消息记录飞行模式切换事件，只在模式变化时写入，不是周期性记录。

### 调用流程

```
用户切换模式（遥控器/GCS/任务）
  ↓
Plane::set_mode(new_mode, reason)
  位置: ArduPlane/system.cpp:280-333
  ↓
logger.Write_Mode(control_mode->mode_number(), control_mode_reason)
  位置: ArduPlane/system.cpp:324
  ↓
AP_Logger::Write_Mode(mode, reason)
  位置: libraries/AP_Logger/AP_Logger.cpp:890-893
  ↓
FOR_EACH_BACKEND(Write_Mode(mode, reason))
  遍历所有已注册的日志后端
  ↓
AP_Logger_Backend::Write_Mode(mode, reason)
  位置: libraries/AP_Logger/LogFile.cpp:442-453
  ↓
构造 log_Mode 结构体:
  struct PACKED log_Mode {
      LOG_PACKET_HEADER;
      uint64_t time_us;        // 微秒时间戳
      uint8_t mode;            // 模式编号 (如 10=AUTO, 15=GUIDED)
      uint8_t mode_num;        // 模式编号（冗余字段）
      uint8_t mode_reason;     // 切换原因 (如 2=GCS_COMMAND)
  };
  ↓
WriteCriticalBlock(&pkt, sizeof(pkt))
  is_critical=true，保证写入
  ↓
WritePrioritisedBlock(pBuffer, size, true)
  位置: libraries/AP_Logger/AP_Logger_File.cpp:440
  ↓
写入到各后端:
  - AP_Logger_File (文件系统 /logs/00000001.BIN)
  - AP_Logger_Block (DataFlash)
  - AP_Logger_MAVLink (MAVLink 传输)
```

### MODE 消息字段

**代码位置**：`libraries/AP_Logger/LogFile.cpp:442-453`（结构体构造）
**写入函数**：`AP_Logger_Backend::Write_Mode(mode, reason)`
**调用位置**：`ArduPlane/system.cpp:324`（`Plane::set_mode()` 中调用 `logger.Write_Mode()`）

| 字段 | 类型 | 说明 |
|---|---|---|
| TimeUS | uint64_t | 系统启动以来的微秒数 |
| Mode | uint8_t | 当前模式编号 |
| ModeNum | uint8_t | 同 Mode（历史冗余） |
| Rsn | uint8_t | ModeReason 枚举值 |

### 模式编号定义

位置：`ArduPlane/mode.h:33-65`

```cpp
enum Number : uint8_t {
    MANUAL        = 0,
    CIRCLE        = 1,
    STABILIZE     = 2,
    TRAINING      = 3,
    ACRO          = 4,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    CRUISE        = 7,
    AUTOTUNE      = 8,
    AUTO          = 10,
    RTL           = 11,
    LOITER        = 12,
    TAKEOFF       = 13,
    AVOID_ADSB    = 14,
    GUIDED        = 15,
    INITIALISING  = 16,
    // QuadPlane 模式 17-25
};
```

### 切换原因定义

位置：`libraries/AP_Vehicle/ModeReason.h:19-75`

```cpp
enum class ModeReason : uint8_t {
  UNKNOWN = 0,
  RC_COMMAND = 1,
  GCS_COMMAND = 2,
  RADIO_FAILSAFE = 3,
  BATTERY_FAILSAFE = 4,
  // ... 更多原因
};
```

## 日志系统架构

### 消息定义层次

1. **消息 ID 枚举**
   - 位置：`libraries/AP_Logger/LogStructure.h:1289-1372`
   - 定义：`LOG_xxx_MSG` 常量（如 `LOG_MODE_MSG = 122`）

2. **结构体定义**
   - 位置：各库的 `LogStructure.h` 或 `*_Logging.cpp`
   - 定义：`struct PACKED log_Xxx`（字段类型和顺序）

3. **格式注册**
   - 位置：各库的 `LogStructure.h` 格式表数组
   - 格式：`{ LOG_xxx_MSG, sizeof(log_xxx), "NAME", "format", "labels", "units", "mults" }`
   - 示例：`{ LOG_MODE_MSG, sizeof(log_Mode), "MODE", "QMBB", "TimeUS,Mode,ModeNum,Rsn", "s---", "F---" }`

4. **写入函数**
   - 位置：各库的 `*_Logging.cpp` 或载具的 `Log.cpp`
   - 函数：`void Xxx::Log_Write_Yyy()`
   - 调用：通过调度表周期性调用（如 `SCHED_TASK(Log_Write_xxx, freq, ...)`）

### 写入优先级

- **Critical 消息**（如 MODE、PARM）：使用 `WriteCriticalBlock()`，缓冲区满时保留空间，保证写入
- **普通消息**（如 CTUN、NTUN）：使用 `WriteBlock()`，缓冲区满时可能被丢弃

### 日志后端

- **AP_Logger_File**：文件系统（SD 卡）
- **AP_Logger_Block**：DataFlash（环形缓冲区）
- **AP_Logger_MAVLink**：通过 MAVLink 传输到 GCS

## 代码位置总结

| 组件 | 位置 | 作用 |
|---|---|---|
| 消息 ID 枚举 | `libraries/AP_Logger/LogStructure.h:1289-1372` | 定义 `LOG_xxx_MSG` 常量 |
| 结构体定义 | 各库的 `LogStructure.h` 或 `*_Logging.cpp` | 定义 `struct PACKED log_Xxx` |
| 格式注册 | 各库的 `LogStructure.h` 格式表数组 | 提供字段名、类型、单位信息 |
| 写入函数 | 各库的 `*_Logging.cpp` 或载具的 `Log.cpp` | `void Xxx::Log_Write_Yyy()` |
| 调度调用 | 载具的 `ArduPlane.cpp` 调度表 | `SCHED_TASK(Log_Write_xxx, freq, ...)` |

## 关键发现

1. **MODE 消息只出现 5 次**：只在模式切换时记录，不是周期性记录
2. **IMU 最高频（11.4 万条）**：50Hz 采样，记录原始传感器数据
3. **EKF 相关消息很多**：XKF1-5、XKQ、XKFS 等共 7 种，反映 EKF 是核心算法
4. **QuadPlane 专用消息**：QPOS、QTUN、PIQP/PIQR/PIQY/PIQA，日志包含倾转旋翼数据
5. **PID 消息丰富**：PIDR/PIDP/PIDY（姿态）+ PIDN/PIDE（导航）+ PIQP/PIQR/PIQY/PIQA（QuadPlane）

## 日志查看工具

```bash
# 列出所有消息类型
mavlogdump.py --type=FMT logs/00000001.BIN

# 查看特定消息
mavlogdump.py --type=MODE logs/00000001.BIN

# 统计消息数量
mavlogdump.py logs/00000001.BIN | awk -F'{' '{print $1}' | awk '{print $NF}' | sort | uniq -c | sort -rn
```

## VTOL 核心日志消息详解

### TILT - 倾转旋翼角度

**代码位置**：`ArduPlane/tiltrotor.cpp:404-431`
**写入函数**：`Tiltrotor::write_log()`（定义于 `tiltrotor.cpp:404`）
**调用位置**：`ArduPlane/quadplane.cpp:3702`（在 `QuadPlane::Log_Write_QControl_Tuning()` 末尾调用，与 QTUN 同一周期）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| Tilt | float | deg | 当前倾转角度（current_tilt × 90°） |
| FL | float | deg | 左侧倾转电机角度（仅矢量偏航模式有效） |
| FR | float | deg | 右侧倾转电机角度（仅矢量偏航模式有效） |

**日志示例**：
```
TILT {TimeUS : 84286272, Tilt : 89.87, FL : 89.87, FR : 89.87}
```

**说明**：
- `Tilt` 范围 0-90°，0°=垂直向上（VTOL），90°=水平向前（固定翼）
- `FL`/`FR` 仅在 `Q_TILT_TYPE=2`（矢量偏航）时有意义，从舵机输出反算
- 非矢量偏航模式下 FL/FR 为 NaN

---

### QPOS - QuadPlane 位置控制状态

**代码位置**：`ArduPlane/quadplane.cpp:2245-2264`
**写入函数**：`QuadPlane::log_QPOS()`
**调用位置**：
1. **状态切换时**（`PosControlState::set_state()`，2304/2308 行）：状态变化前后各写一次，捕捉状态转换
2. **周期性记录**（`QuadPlane::vtol_position_controller()`，2884 行）：25Hz（每 40ms 一次）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| State | uint8_t | - | 位置控制状态（见下表） |
| Dist | float | m | 到下一航点的距离 |
| TSpd | float | m/s | 目标速度 |
| TAcc | float | m/s² | 目标加速度 |
| OShoot | uint8_t | - | 是否超过降落点或航向偏差>60° |

**位置控制状态枚举**（`QuadPlane::PosControlState`）：

| 值 | 状态名 | 说明 |
|----|--------|------|
| 0 | QPOS_NONE | 未激活 |
| 1 | QPOS_APPROACH | 进近阶段（固定翼方式飞向降落点） |
| 2 | QPOS_AIRBRAKE | 空气刹车（VTOL 电机启动减速） |
| 3 | QPOS_POSITION1 | 定位阶段 1（VTOL 控制器接管） |
| 4 | QPOS_POSITION2 | 定位阶段 2 |
| 5 | QPOS_LAND_DESCEND | 下降阶段 |
| 6 | QPOS_LAND_ABORT | 降落中止 |
| 7 | QPOS_LAND_FINAL | 最终降落 |
| 8 | QPOS_LAND_COMPLETE | 降落完成 |

**日志示例**：
```
QPOS {TimeUS : 141837409, State : 1, Dist : 769.47, TSpd : 0.0, TAcc : 0.0, OShoot : 0}
```

---

### QTUN - QuadPlane 控制调优

**代码位置**：`ArduPlane/quadplane.cpp:3680-3696`
**写入函数**：`QuadPlane::Log_Write_QControl_Tuning()`（定义于 `quadplane.cpp:3637`）
**调用位置**：`ArduPlane/quadplane.cpp:1819`（在 `QuadPlane::update()` 中调用，条件：电机活跃或最近 250ms 内活跃，且距上次记录 >40ms，即 25Hz）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| ThI | float | - | 油门输入（来自姿态控制器） |
| ABst | float | - | 角度增强（用于补偿倾转导致的升力损失） |
| ThO | float | - | 油门输出（到电机） |
| ThH | float | - | 悬停油门（估计值） |
| DAlt | float | m | 期望高度 |
| Alt | float | m | 实际高度（惯性导航） |
| BAlt | float | m | 气压计高度 |
| DCRt | float | m/s | 期望爬升率 |
| CRt | float | m/s | 实际爬升率 |
| TMix | float | - | 油门混合（VTOL 与固定翼的混合比例） |
| Trn | uint8_t | - | 过渡状态（0=完成, 1=等待空速, 2=计时中） |
| Ast | uint8_t | - | 辅助标志位（见下表） |

**辅助标志位**（`log_assistance_flags`）：

这是一个 C++ 枚举类，用于记录 VTOL 辅助飞行时的状态标志，主要用于日志记录。

```cpp
enum class log_assistance_flags {
    in_assisted_flight = 1U<<0,  // 第0位 = 1 (0x01)
    forced             = 1U<<1,  // 第1位 = 2 (0x02)
    speed              = 1U<<2,  // 第2位 = 4 (0x04)
    alt                = 1U<<3,  // 第3位 = 8 (0x08)
    angle              = 1U<<4,  // 第4位 = 16 (0x10)
    fw_force           = 1U<<5,  // 第5位 = 32 (0x20)
    spin_recovery      = 1U<<6,  // 第6位 = 64 (0x40)
};
```

`1U<<n` 表示把 1 左移 n 位，每个标志占一个 bit，可以组合成一个字节。

| 位 | 标志名 | 含义 |
|----|--------|------|
| 0 | in_assisted_flight | VTOL 辅助飞行激活中 |
| 1 | forced | 强制辅助（参数设置强制开启） |
| 2 | speed | 因空速过低触发辅助 |
| 3 | alt | 因高度过低触发辅助 |
| 4 | angle | 因姿态误差过大触发辅助 |
| 5 | fw_force | 强制使用固定翼控制器 |
| 6 | spin_recovery | 尾旋恢复 |

**日志示例**：
```
QTUN {TimeUS : 84286272, ThI : 0.0, ABst : 0.0, ThO : 0.0, ThH : 0.35, DAlt : -0.03, Alt : -0.03, BAlt : -0.08, DCRt : 0.0, CRt : 0.0, TMix : 0.5, Trn : 0, Ast : 0}
```

---

### PIQR/PIQP/PIQY - QuadPlane 角速率 PID

**代码位置**：`ArduPlane/Log.cpp:32-34`
**写入函数**：`Plane::Log_Write_Attitude()`（定义于 `Log.cpp:6`，通过 `logger.Write_PID()` 写入）
**调用位置**：`ArduPlane/ArduPlane.cpp:286`（在 `Plane::update_logging25()` 中调用，25Hz）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| Tar | float | deg/s | 目标角速度 |
| Act | float | deg/s | 实际角速度（来自陀螺仪） |
| Err | float | deg/s | 误差（Tar - Act） |
| P | float | - | P 项输出 |
| I | float | - | I 项输出 |
| D | float | - | D 项输出 |
| FF | float | - | 前馈输出 |
| DFF | float | - | 微分前馈输出 |
| Dmod | float | - | D 项调制系数 |
| SRate | float | Hz | 采样率 |
| Flags | uint8_t | - | 标志位（0=正常, 1=限制） |

**日志示例**：
```
PIQR {TimeUS : 84342916, Tar : -0.017, Act : 0.0003, Err : -0.017, P : -0.002, I : 0.0, D : -0.0007, FF : 0.0, DFF : 0.0, Dmod : 1.0, SRate : 0.004, Flags : 1}
```

**说明**：
- PIQR：滚转角速率 PID
- PIQP：俯仰角速率 PID
- PIQY：偏航角速率 PID
- 这些是 VTOL 模式下的角速率内环 PID，输出到电机混控

---

### MOTB - 电机电池状态

**代码位置**：`libraries/AP_Motors/AP_MotorsMulticopter.cpp:393-403`
**写入函数**：`AP_MotorsMulticopter::Log_Write()`（定义于 `AP_MotorsMulticopter.cpp:391`）
**调用位置**：`ArduPlane/quadplane.cpp:1813`（在 `QuadPlane::update()` 中调用，条件：距上次记录 >100ms，即 10Hz，仅当电机活跃时）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| LiftMax | float | - | 最大升力系数（归一化） |
| BatVolt | float | - | 电池电压系数（归一化） |
| ThLimit | float | - | 油门限制系数 |
| ThrAvMx | float | - | 可用最大油门 |
| ThrOut | float | - | 实际油门输出 |
| FailFlags | uint8_t | - | 故障标志 |

**日志示例**：
```
MOTB {TimeUS : 84286272, LiftMax : 1.0, BatVolt : 1.0, ThLimit : 1.0, ThrAvMx : 0.0, ThrOut : 0.0, FailFlags : 2}
```

---

### TECS - 总能量控制系统

**代码位置**：`libraries/AP_TECS/AP_TECS.cpp:1334`
**写入函数**：直接在 `AP_TECS::update_pitch_throttle()` 函数内调用 `AP::logger().WriteStreaming()`
**调用位置**：`ArduPlane/ArduPlane.cpp:659`（在 `Plane::calc_throttle()` 中调用 `TECS_controller.update_pitch_throttle()`，50Hz）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| h | float | m | 实际高度 |
| dh | float | m/s | 实际爬升率 |
| hin | float | m | 期望高度输入 |
| hdem | float | m | 期望高度 |
| dhdem | float | m/s | 期望爬升率 |
| spdem | float | m/s | 期望空速 |
| sp | float | m/s | 实际空速 |
| dsp | float | m/s² | 实际加速度 |
| th | float | - | 油门输出（0-1） |
| ph | float | rad | 俯仰角输出 |
| pmin | float | rad | 俯仰角下限 |
| pmax | float | rad | 俯仰角上限 |
| dspdem | float | m/s² | 期望加速度 |
| f | uint8_t | - | 标志位 |

**日志示例**：
```
TECS {TimeUS : 84742756, h : -0.028, dh : -0.002, hin : 19.98, hdem : -0.027, dhdem : 0.033, spdem : 3.0, sp : 3.0, dsp : 0.071, th : 0.45, ph : 0.015, pmin : -0.524, pmax : 0.262, dspdem : 0.0, f : 128}
```

**说明**：
- TECS 是固定翼模式下的能量管理器，控制油门和俯仰角
- 在 VTOL 模式下 TECS 仍然运行，但输出不被使用
- 过渡阶段 TECS 和 VTOL 控制器并行运行

---

### CTUN - 控制调优（固定翼）

**代码位置**：`ArduPlane/Log.cpp:93-121`
**写入函数**：`Plane::Log_Write_Control_Tuning()`（定义于 `Log.cpp:94`）
**调用位置**：`ArduPlane/ArduPlane.cpp:290`（在 `Plane::update_logging25()` 中调用，25Hz）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| NavRoll | int16_t | cdeg | 导航期望滚转角 |
| Roll | int16_t | cdeg | 实际滚转角 |
| NavPitch | int16_t | cdeg | 导航期望俯仰角 |
| Pitch | int16_t | cdeg | 实际俯仰角 |
| ThO | float | % | 油门输出 |
| RdO | float | - | 方向舵输出 |
| ThD | float | % | 油门期望（来自 TECS） |
| As | float | m/s | 估计空速 |
| AsT | uint8_t | - | 空速估计状态 |
| SAs | float | m/s | 合成空速 |
| E2T | float | - | EAS 到 TAS 转换系数 |
| GU | int32_t | - | 地速欠冲 |

**日志示例**：
```
CTUN {TimeUS : 48586391, NavRoll : -25.0, Roll : 0.06, NavPitch : 0.0, Pitch : 0.04, ThO : 0.0, RdO : -2250.0, ThD : 0.0, As : 1.05, AsT : 1, SAs : 1.00, E2T : 1.03, GU : 0}
```

---

### RATE - 角速率控制输出

**代码位置**：`libraries/AC_AttitudeControl/AC_AttitudeControl_Logging.cpp:31-50`
**写入函数**：`AC_AttitudeControl::Write_Rate()`（定义于 `AC_AttitudeControl_Logging.cpp:31`）
**调用位置**：`ArduPlane/quadplane.cpp:1806`（在 `QuadPlane::update()` 中调用，主循环频率）

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| TimeUS | uint64_t | us | 系统启动以来的微秒数 |
| RDes | float | deg/s | 期望滚转角速度 |
| R | float | deg/s | 实际滚转角速度 |
| ROut | float | - | 滚转控制输出（-1 到 +1） |
| PDes | float | deg/s | 期望俯仰角速度 |
| P | float | deg/s | 实际俯仰角速度 |
| POut | float | - | 俯仰控制输出 |
| YDes | float | deg/s | 期望偏航角速度 |
| Y | float | deg/s | 实际偏航角速度 |
| YOut | float | - | 偏航控制输出 |
| ADes | float | m/s² | 期望垂直加速度 |
| A | float | m/s² | 实际垂直加速度 |
| AOut | float | - | 垂直加速度控制输出 |
| AOutSlew | float | - | 垂直输出渐变率 |

**日志示例**：
```
RATE {TimeUS : 84286272, RDes : 0.0, R : 0.0, ROut : 0.0, PDes : 0.0, P : 0.0, POut : 0.0, YDes : 0.0, Y : 0.0, YOut : 0.0, ADes : 0.0, A : 0.0, AOut : 0.0, AOutSlew : 0.0}
```

**说明**：
- 这是 VTOL 姿态控制的最外层输出
- `ROut`/`POut`/`YOut` 是归一化的控制指令（-1 到 +1），送到电机混控
- 与 PIQR/PIQP/PIQY 的关系：RATE 是期望值，PIQx 是 PID 内部状态

---

## 相关文档

- `CLAUDE.md`：项目整体架构和构建命令
- `docs/redundancy_feasibility.md`：多余度飞控调研
- `docs/vtol_tiltrotor_architecture.md`：倾转旋翼架构
- `docs/code_details.md`：代码细节分析（in_vtol_mode、use_fw_attitude_controllers 等）
