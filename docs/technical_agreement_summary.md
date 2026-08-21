# 倾转旋翼验证机飞控系统采购技术协议书摘要

> 原文：`/home/codyqin/Downloads/飞控开发技术协议书20260807.docx`  
> 关联文档：[风险分析](technical_agreement_risk_analysis.md) | [VTOL架构通读](vtol_tiltrotor_architecture.md)

---

## 1. 项目概况

**合同双方**：
- 甲方：中汽创智科技有限公司（南京）
- 乙方：武汉玄天飞车航空科技有限公司（武汉）

**项目目标**：开发倾转旋翼验证机飞控系统（约300kg级）

**阶段划分**：
| 阶段 | 截止 | 主要交付 |
|------|------|----------|
| 一期 | 2026-08-31 | 飞控系统设计、选型报告、特性分析 |
| 二期 | 2026-09-30 | 仿真测试、缩比机试飞、代码初版 |
| 三期 | 2026-10-31 | 垂起首飞、全模态控制律 |
| 四期 | 2026-12-31 | 全模态飞行、全部交付物 |

**职责分工**：
- 甲方：总体设计、气动仿真（R）；飞控设计、控制律（S）
- 乙方：飞控设计、控制律、动力学建模、软件开发、测试（R）

---

## 2. 飞控计算机硬件需求（§5.1.1.1, §5.1.5.1）

| 需求 | APM实现 | 状态 |
|------|---------|------|
| 主CPU ≥480MHz | CubeOrange STM32H7@480MHz, Pixhawk6X STM32H755@480MHz | ✅ |
| 从CPU ≥24MHz | STM32F103@72MHz协处理器(IOMCU) | ✅ |
| MEMS惯导集成 | 三冗余IMU (ICM-20689, BMI055, ICM-42688) | ✅ |
| 动静压采集 | 内置气压计(MS5611/ICP-20100) + 外部空速管 | ✅ |
| ≥2路隔离CAN | `libraries/AP_DroneCAN/` (libcanard), 双CAN口 | ✅ |
| ≥5路UART | 典型H7板6路UART | ✅ |
| ≥4 DI/DO或16 PWM | `libraries/SRV_Channel/SRV_Channel.h:47` (Aux_servo_function_t) | ✅ |
| 数据记录≥8GB | `libraries/AP_Logger/` + microSD | ✅ |
| 总重≤2kg | 飞控板<500g | ✅ |
| 工作温度-40~+60°C | ⚠️ 标准板-20~+60°C，需定制工业级 | ⚠️ |

**板级定义**：`libraries/AP_HAL_ChibiOS/hwdef/`（366个板子目录）

---

## 3. 飞行控制功能需求（§5.1.3）

### 3.1 旋翼模态（§5.1.3.3）

| 功能 | APM实现 | 代码路径 |
|------|---------|----------|
| 增稳控制 | QSTABILIZE模式 | `ArduPlane/mode_qstabilize.cpp` |
| 定高控制 | QHOVER模式 | `ArduPlane/mode_qhover.cpp` |
| 定点控制 | QLOITER模式 | `ArduPlane/mode_qloiter.cpp` |
| 自动航线 | AUTO模式 | `ArduPlane/mode_auto.cpp` + `libraries/AP_Mission/` |

**详细通读**：[VTOL架构文档](vtol_tiltrotor_architecture.md)

### 3.2 转换模态（§5.1.3.4）

| 功能 | APM实现 | 代码路径 | 状态 |
|------|---------|----------|------|
| 冗余操纵分配 | tilt_compensate_angle() | `ArduPlane/tiltrotor.cpp:451-497` | ⚠️ 需验证300kg适用性 |
| 转换姿态控制 | SLT_Transition三阶段状态机 | `ArduPlane/quadplane.cpp:1466-1671` | ⚠️ 需补充中断回退 |
| 转换进入条件 | Q_ASSIST_SPEED参数 | `ArduPlane/quadplane.cpp` | ✅ |
| 转换中断判断 | **缺失** | — | ❌ 需新增TRANSITION_ABORT |
| 安全性处理 | Q_TRANS_FAIL_TMO超时降级 | `quadplane.cpp:1466-1671` | ⚠️ 需扩展 |

**三阶段状态机**：
1. `TRANSITION_AIRSPEED_WAIT`：等待空速达到AIRSPEED_MIN
2. `TRANSITION_TIMER`：逐步衰减多旋翼推力
3. `TRANSITION_DONE`：固定翼完全接管

### 3.3 固定翼模态（§5.1.3.5）

| 功能 | APM实现 | 代码路径 |
|------|---------|----------|
| 姿态控制 | FBWA/STABILIZE | `ArduPlane/mode_fbwa.cpp` |
| 外环控制 | CRUISE/LOITER/AUTO | `mode_cruise.cpp`, `mode_loiter.cpp`, `mode_auto.cpp` |

### 3.4 CAAC合规（§5.1.3.6）⚠️ 最大合规风险

| 功能 | APM现状 | 工作量 |
|------|---------|--------|
| 实名登记+UOM激活校验 | ❌ 无 | 需从零开发 |
| 运行识别信息报送 | ❌ 无 | 需从零开发 |
| 广播式Remote ID (BLE/WiFi) | ❌ 无 | 需集成外部模块 |
| 网络式Remote ID (4G/5G→UOM) | ❌ 无 | 需开发蜂窝通信 |
| 识别失效处置 | ⚠️ 可复用failsafe框架 | 需定制 |

**参考标准**：GB 46761-2025（实名登记）、GB 46750-2025（运行识别）

### 3.5 安全管理（§5.1.3.7）

| 功能 | APM实现 | 代码路径 | 状态 |
|------|---------|----------|------|
| 低电量保护 | battery_failsafe() | `ArduPlane/failsafe.cpp:28-44` | ✅ |
| 电子围栏 | AC_Fence库 | `libraries/AC_Fence/` | ✅ |
| GNSS丢失保护 | gps_failsafe() | `ArduPlane/failsafe.cpp` | ✅ |
| RC中断保护 | RC failsafe→RTL | `ArduPlane/system.cpp:353-418` | ✅ |
| 高度异常保护 | **无独立模块** | — | ❌ 需新增 |
| 失速保护 | **无独立模块** | — | ❌ 需新增 |
| 姿态异常保护 | ekf_check() | `ArduPlane/ekf_check.cpp` | ⚠️ 部分 |
| 单电机失效 | **无独立模块** | — | ❌ 需新增 |
| EKF检查 | ekf_check() | `ArduPlane/ekf_check.cpp` | ✅ |
| 高级故障安全 | AP_AdvancedFailsafe | `libraries/AP_AdvancedFailsafe/` | ✅ |

**详细通读**：[故障安全代码通读](code_walkthrough_failsafe.md)

### 3.6 航线与遥控（§5.1.3.7）

| 功能 | APM实现 | 代码路径 |
|------|---------|----------|
| 航线管理 | AP_Mission | `libraries/AP_Mission/AP_Mission.h` |
| RC输入解析 | radio.cpp | `ArduPlane/radio.cpp` |
| 参数系统 | AP_Param | `libraries/AP_Param/AP_Param.h:178` |
| 日志记录 | AP_Logger | `libraries/AP_Logger/AP_Logger.h` |
| GCS/MAVLink | GCS_MAVLINK | `libraries/GCS_MAVLink/GCS.h:191` |

---

## 4. 二次开发需求（§5.1.3.2）

| 需求 | APM实现 | 状态 |
|------|---------|------|
| 硬件抽象层(HAL) | `libraries/AP_HAL/` (接口), `AP_HAL_ChibiOS/` (实现) | ✅ |
| 飞行状态API | `AP::ahrs()`, `AP::gps()`, `AP::baro()` 等单例 | ✅ |
| 控制指令注入 | GUIDED模式 + `AP_Vehicle::set_mode()` | ⚠️ 部分 |
| 调度器 | `libraries/AP_Scheduler/AP_Scheduler.h:87` (Task结构) | ✅ |
| API文档 | **无中文文档** | ❌ 需编写 |
| 示例工程 | **无** | ❌ 需创建 |

**调度器任务表**：`ArduPlane/ArduPlane.cpp:56` (scheduler_tasks[])

---

## 5. 动力学建模（§5.1.2）

| 需求 | APM实现 | 说明 |
|------|---------|------|
| 全包线建模 | 需甲方提供气动数据 | — |
| 操稳特性分析 | SITL内置模型 | `libraries/SITL/SIM_Plane.cpp:112` (liftCoeff) |
| 倾转策略 | 四种倾转类型 | `ArduPlane/tiltrotor.cpp` |

**内置模型局限**：`SIM_Plane.cpp` 使用sigmoid静态失速模型，不支持桨-翼干扰、动态失速。高保真建模需使用MATLAB/JSBSim。

---

## 6. 仿真验证（§5.1.4）

| 需求 | APM实现 | 状态 |
|------|---------|------|
| 全流程仿真 | SITL + JSBSim/MATLAB | ✅ |
| 控制参数拉偏 | 可改PID参数 | ✅ |
| 升阻/舵效拉偏 | ⚠️ 内置模型硬编码，需改源码或用MATLAB | ⚠️ |
| 重心/重量拉偏 | SIM_MASS/SIM_CG参数 | ✅ |
| 风扰动 | SIM_WIND_*参数 | ✅ |
| 传感器扰动 | SIM_GPS_*/SIM_IMU_*噪声参数 | ✅ |
| 半物理仿真 | HIL via DroneCAN | ✅ |

**VTOL仿真frames**：`Tools/autotest/pysim/vehicleinfo.py:252`
- quadplane（标准四旋翼）
- quadplane-tilt（倾转旋翼）
- quadplane-tri（前三旋翼）

---

## 7. 性能指标（§5.1.5）

| 模态 | 指标 | 要求 |
|------|------|------|
| 旋翼 | 姿态误差 | <1° |
| 旋翼 | 速度误差 | <0.5m/s |
| 旋翼 | 位置误差 | <1m |
| 旋翼 | 自主起降偏移 | ≤2m |
| 转换 | 姿态波动 | <5° |
| 转换 | 高度波动 | <10m |
| 固定翼 | 高度误差 | <5m |
| 固定翼 | 速度误差 | <2m/s |
| 固定翼 | 侧偏距 | <5m |

---

## 8. 地面站（§5.2）

- Mission Planner/QGC可复用但需深度定制
- 要求源码交付
- 支持Windows/Ubuntu双平台
- 支持多模式任务规划、实时曲线、航线编辑

---

## 9. 试飞验证（§5.3）

| 阶段 | 内容 |
|------|------|
| 缩比机 | HIL + 全模态试飞 |
| 验证机桌面 | 联调、接口验证、拷机测试(≥1小时) |
| 验证机装机 | 全机系统联试 |
| 试飞科目 | 旋翼→转换→固定翼→全模态自动飞行 |

---

## 10. 交付物清单

**硬件**（第二期交付）：
- 飞控计算机1架（含线缆、GNSS天线、空速管）
- 主备数据链机载端1套
- GNSS天线1块、地面天线4根
- 半实物仿真系统1套
- 遥控器1台
- 低压适配模块及线缆1套

**文档**（≥17件）：
- 选型报告（飞控、航电）
- 通信系统集成测试报告
- 地面站用户手册
- 地面测试大纲/报告
- 特性分析报告
- 控制律设计报告（多旋翼、全模态）
- 飞控系统设计报告
- 仿真测试报告
- 飞行试验大纲/报告
- 飞控计算机用户手册
- 飞控基础软件二次开发说明
- 缩比机HIL/试飞报告

**软件及代码**（≥4件）：
- 地面站软件源代码
- 机载软件初版源代码（第二期）
- 仿真源代码
- 机载软件终版源代码（第四期）

---

## 11. 核心代码通读文档索引

| 模块 | 文档 | 主要内容 |
|------|------|----------|
| QuadPlane/VTOL | [code_walkthrough_quadplane.md](code_walkthrough_quadplane.md) | Tiltrotor、Transition、Tailsitter |
| 故障安全 | [code_walkthrough_failsafe.md](code_walkthrough_failsafe.md) | Failsafe、EKF Check、Arming |
| 调度器 | [code_walkthrough_scheduler.md](code_walkthrough_scheduler.md) | AP_Scheduler、任务表、频率分频 |
| 任务/导航 | [code_walkthrough_mission.md](code_walkthrough_mission.md) | AP_Mission、飞行模式 |
| EKF/传感器 | [code_walkthrough_ekf.md](code_walkthrough_ekf.md) | AP_NavEKF3、AP_AHRS、多lane |
| 舵机/输出 | [code_walkthrough_servo.md](code_walkthrough_servo.md) | SRV_Channel、DroneCAN输出 |

---

**风险评估**：详见 [technical_agreement_risk_analysis.md](technical_agreement_risk_analysis.md)
