# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

ArduPilot 是开源飞控固件，一套代码支持多种载具。当前分支常见于 Plane 开发（如 `qhz/plane_4.6.3`），PR 通常基于 `main`（上游默认分支名为 `master`）。

本仓库正在进行**多余度飞控系统调研**（主从接管 → 任务对齐 → 三余度表决），调研结论与参考文档索引见 `docs/redundancy_feasibility.md`，涉及 DroneCAN 自定义 DSDL、接管 arming 钩子、EKF lane 切换等主题时应先读该文档。

## 常用命令

构建系统为 waf，必须从仓库根目录调用，**不要用 sudo**。`./waf` 是 `modules/waf/waf-light` 的包装。

```sh
./waf configure --board <board>   # 配置目标板，只需在换板/换选项时重新执行
./waf list_boards                 # 列出所有支持的板子
./waf plane                       # 构建 ArduPlane（其他目标: copter/heli/rover/sub/antennatracker/AP_Periph/blimp）
./waf --targets bin/arduplane     # 只构建单个二进制
./waf --targets tests/test_math   # 构建并运行单个单元测试
./waf check                       # 构建全部程序并运行增量测试（只跑新构建/失败过的）
./waf check-all                   # 构建并运行所有单元测试（等价于 ./waf check --alltests）
./waf --targets bin/arducopter --upload   # 构建并上传到连接的飞控板
```

产物位于 `build/<board>/bin/` 与 `build/<board>/tests/`。板子分三类：ChibiOS MCU 板（如 CubeBlack、fmuv3）、Linux SBC 板（如 navio2，可用 `--rsync-dest` 配置上传目标）、`sitl`（软件在环模拟器，`--debug` 加调试符号）。

常用 configure 选项：`--enable-scripting`、`--enable-dds`（ROS2/DDS）、`--default-parameters`（内嵌默认参数）、`--extra-hwdef`（自定义板级 hwdef.dat）。`build_options.py` 中每个选项都有对应的 `--enable-xxx`/`--disable-xxx` 开关，决定哪些库参与编译（库内代码统一用 `AP_xxx_ENABLED` 宏裁剪）。

单元测试基于 gtest，测试源码放在各库的 `libraries/<LIB>/tests/`（还有 `examples/` 和 `benchmarks/`，后者需 `configure --enable-benchmarks`）。顶层 `tests/` 只有公共头文件 `AP_gtest.h`。

SITL 仿真：

```sh
./Tools/autotest/sim_vehicle.py -v ArduPlane        # 编译 SITL 并启动仿真+MAVProxy
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane   # -f 指定 frame
./Tools/autotest/autotest.py run.Plane              # 完整自动化测试（各载具测试定义在 Tools/autotest/<vehicle>.py）
```

## 代码风格与提交规范

- C++ 用 astyle 格式化：`Tools/CodeStyle/ardupilot-astyle.sh <files>`（配置在 `Tools/CodeStyle/astylerc`：Linux 风格、4 空格缩进）。
- Python 用 black + isort（见 `pyproject.toml`），仓库有 `.pre-commit-config.yaml`。
- **提交信息格式**：`子系统: 描述`，例如 `Plane: fix VTOL transition`、`AP_Baro: add temperature compensation`。Bug 报告/PR 标题同样遵循此约定。
- 许可证为 GPLv3。

## 工具使用注意事项

### Write 工具

调用 Write 工具时**必须同时提供** `file_path` 和 `content` 两个参数，缺一不可：

```xml
<Write>
<file_path>/absolute/path/to/file.md</file_path>
<content>文件内容...</content>
</Write>
```

**常见错误**：
- 忘记传递 `file_path` 或 `content` 参数 → 报错 "The required parameter is missing"
- 使用相对路径 → 必须使用绝对路径
- 写入已存在的文件前未先 Read → 报错 "File has not been read yet"

**正确流程**：
1. 新文件：直接 Write，提供绝对_path 和 content
2. 已存在文件：先 Read，再 Write（或 Edit）

### 代码位置引用规则

写文档或回复中引用代码位置（文件名:行号）时，**必须遵守**：

**禁止编造行号**：
- 不得凭记忆或推断写行号
- 不得假设"大概在这个位置"
- 每个行号都必须通过 Read 工具实际验证

**正确流程**：
1. 用 `grep -n "函数名\|变量名" 文件路径` 查找位置
2. 用 `Read` 读取相关代码段，确认实际行号
3. 写入文档前，再次确认行号与代码匹配

**示例**：
```bash
# 错误：凭记忆写 "quadplane.cpp:682"
# 正确：先 grep 查找
grep -n "bool QuadPlane::setup" ArduPlane/quadplane.cpp
# 输出：682:bool QuadPlane::setup(void)
# 然后 Read 确认
Read ArduPlane/quadplane.cpp offset=682 limit=10
# 确认函数确实从 682 行开始
```

**违规后果**：
- 行号错误会误导后续分析
- 破坏文档可信度
- 增加调试成本

### 文档编写规范

**文档位置**：
- 技术调研、设计方案、分析文档 → `docs/` 目录
- 临时笔记、会话产出 → 仓库根目录或 `docs/`
- 不要把文档写进 `libraries/`、`ArduPlane/` 等源码目录

**文档命名**：
- 使用小写字母 + 下划线：`qp_discussion.md`、`vtol_hitl.md`
- 中文文档可用中文名：`控制律设计方案.docx`（已有）
- 避免空格和特殊字符

**内容规范**：
1. **代码引用必须验证**：涉及文件:行号时，必须先用工具确认
2. **交叉引用**：引用其他文档时用相对路径，如 `见 docs/redundancy_feasibility.md`
3. **区分概念与实现**：概念性讨论可以不引用行号；具体代码分析必须验证行号
4. **表格对齐**：用 Markdown 表格时注意列对齐，便于阅读
5. **代码块标注语言**：` ```cpp `、` ```python `、` ```bash ` 等

**更新已有文档时**：
1. 先 Read 全文，了解现有结构
2. 用 Edit 做局部修改，避免整文件覆盖导致丢失内容
3. 修改调用链/行号时，必须重新验证相关代码
4. 在文档开头或相关章节注明"与代码版本对应关系"（如 Plane 4.6.x）

**常见文档类型**：
| 类型 | 示例 | 重点 |
|------|------|------|
| 代码走读 | `code_walkthrough_*.md` | 调用链 + 验证过的行号 |
| 技术调研 | `redundancy_feasibility.md` | 方案对比 + 可行性 |
| 设计讨论 | `qp_discussion.md` | 概念 + 数学 + 与现有实现对比 |
| 联仿指南 | `APM-MATLAB联合仿真.md` | 步骤 + 协议 + 文件索引 |

## 架构总览

### 顶层目录分工

- **载具目录**（`ArduPlane/`、`ArduCopter/`、`Rover/`、`ArduSub/`、`AntennaTracker/`、`Blimp/`、`Tools/AP_Periph/`）：每个载具是一个"sketch"——继承公共基类，定义自己的任务调度表、飞行模式和参数结构。最终编译为 `arduplane`、`arducopter` 等独立二进制。
- **`libraries/`**：~150 个共享库，所有载具逻辑的核心。库名即命名空间前缀。
- **`Tools/`**：构建脚本（`Tools/ardupilotwaf/`）、SITL 自动化测试（`Tools/autotest/`）、Replay 日志回放工具、bootloader、AP_Periph 外设固件等。
- **`modules/`**：git 子模块（mavlink、DroneCAN DSDL、waf、ChibiOS 等），勿手动改动；同步用 `./waf submodulesync`。

### 单进程架构（与 PX4 的关键区别）

ArduPilot 不是微内核/uORB 消息总线架构。**整个载具固件是单进程、单主循环**：

1. 每个 HAL 后端提供一个 `main()`（通过 `AP_HAL_MAIN_CALLBACKS` 宏生成），回调到 `AP_Vehicle::setup()` / `AP_Vehicle::loop()`。
2. 载具类（如 `ArduPlane/ArduPlane.cpp` 中的 `Plane`）在初始化时构建 **`AP_Scheduler::Task` 调度表**（`SCHED_TASK_CLASS(类, 实例, 函数, 频率Hz, 预算微秒, 优先级)` 宏），主循环按优先级和剩余时间预算执行任务；`LOOP_RATE` 表示与主循环同频的任务。
3. 库之间通过**直接对象引用和共享单例**通信（`AP::ahrs()`、`AP::gps()`、`AP::baro()` 等 `get_singleton()`），没有发布/订阅中间件。

### 关键库（理解代码必读）

- **`AP_HAL`**：硬件抽象层接口；后端实现分目录：`AP_HAL_ChibiOS`（STM32 MCU，主流飞控硬件）、`AP_HAL_Linux`、`AP_HAL_SITL`（仿真）、`AP_HAL_Empty`、`AP_HAL_ESP32`、`AP_HAL_QURT`。板级硬件定义在 `libraries/AP_HAL_ChibiOS/hwdef/` 下的 `hwdef.dat`。
- **`AP_Vehicle`**：所有载具的公共基类，负责通用初始化顺序（`init_ardupilot()` 由载具实现）、模式切换（`set_mode()`/`ModeReason`）、看门狗与崩溃转储钩子。
- **`AP_Param`**：参数系统。参数声明在各库/载具的 `var_info[]` 表中（`AP_Int8`/`AP_Float`/...），通过键值名暴露给 GCS，运行时可调并持久化到 `StorageManager`（FRAM/flash）。参数名有长度限制（历史原因 16 字符内）。
- **`AP_AHRS` / `AP_NavEKF2` / `AP_NavEKF3`**：姿态与状态估计。EKF 支持多核（每个 IMU/GPS 组合一个 lane），lane 间打分与切换由 `AP_AHRS` 管理——这是 ArduPilot 传感器冗余的核心机制。`AP_InertialSensor` 管理多 IMU 实例与加速度计/陀螺一致性检查。
- **`GCS_MAVLink`**：MAVLink 协议栈，载具继承实现 `GCS_MAVLINK_Channel`。mavlink 消息代码由构建时 `mavgen` 从 `modules/mavlink/message_definitions` 生成，勿手改。
- **`AP_DroneCAN` / `AP_CANManager`**：DroneCAN（CAN）外设支持；`Tools/AP_Periph` 是可刷到 CAN 外设（GPS、电调等）上的独立固件。DroneCAN C 代码由构建时 dronecangen 生成。
- **飞行模式**：每个模式一个类（如 `ArduPlane/mode_auto.cpp` 的 `ModeAuto`），继承载具的 `Mode` 基类（`mode.h`），实现 `_enter/_run/_exit` 等钩子。
- **故障安全**：载具目录下的 `failsafe.cpp`、`ekf_check.cpp`、`AP_AdvancedFailsafe`（Plane 的 AFS 任务级故障安全）是理解冗余/降级行为的关键入口。

### 构建系统（waf）要点

- 顶层 `wscript` 定义命令和选项；每个载具目录有自己的 `wscript`，通过 `bld.ap_stlib(ap_vehicle=..., ap_libraries=[...])` 声明该载具链接哪些库。
- 每个库目录可有 `wscript`（声明源文件、tests/examples/benchmarks 子目录）。
- 库是否参与编译由 `Tools/ardupilotwaf/build_options.py` 中的 feature 开关控制，反映为 `AP_FEATURE_xxx` → 库内 `AP_xxx_ENABLED` 宏。裁剪代码时要同时注意 `*_config.h` 中的默认值。
- 程序组：`bin`（载具主程序，默认构建组）、`tools`、`examples`、`tests`、`benchmarks`；产品组名可直接当命令用（`./waf plane`）。

### 测试体系

- **单元测试**：gtest，`libraries/<LIB>/tests/test_*.cpp`，用 `./waf --targets tests/test_xxx` 运行单个测试，`AP_gtest.h` 提供公共夹具。
- **SITL autotest**：Python 驱动的端到端测试（`Tools/autotest/`），`autotest.py` 编排：编译 SITL → 启动仿真器（jsbsim 等）→ 通过 MAVLink 注入故障/任务并断言行为。新增飞控行为通常需要在此加测试。
- **Replay**：`Tools/Replay` 可回放真实 `.bin` 数据日志重新跑 EKF/滤波器，用于离线验证估计器改动。
