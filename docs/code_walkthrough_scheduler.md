# AP_Scheduler 调度器模块代码通读

> 适用分支：`qhz/plane_4.6.3`
> 关联文档：[技术协议摘要](technical_agreement_summary.md) §4 二次开发需求 | [余度调研](redundancy_feasibility.md) §2.5

---

## 1. 核心概念

ArduPilot 是**单进程单主循环**架构，所有任务在同一循环中执行，不同频率通过 tick 整数分频实现。

与 PX4 的 uORB 多队列模型的关键区别：
- APM：所有任务在同一主循环，分频靠 tick 计数
- PX4：不同频率模块在不同 work queue

---

## 2. Task 结构（`libraries/AP_Scheduler/AP_Scheduler.h:87`）

```cpp
struct Task {
    task_fn_t function;        // 函数指针
    const char *name;          // 任务名
    float rate_hz;             // 目标频率
    uint16_t max_time_micros;  // 最大执行时间预算
    uint8_t priority;          // 优先级（0最高）
};
```

**特殊频率值**（`AP_Scheduler.h:40`）：
- `LOOP_RATE = 0`：每 tick 都跑，与主循环同频
- 正整数：主循环的整数分频比

---

## 3. 主循环入口（`AP_Scheduler.cpp:342-347`）

```cpp
void AP_Scheduler::loop() {
    AP::ins().wait_for_sample();   // 阻塞等 INS 新采样
    ...
    tick();                        // 计数器 +1
    run(time_available);           // 扫描任务表执行
}
```

**关键**：主循环节奏由传感器驱动，`wait_for_sample()` 是天然阻塞点。

---

## 4. 频率分频机制（`AP_Scheduler.cpp:226-235`）

```cpp
const uint16_t dt = _tick_counter - _last_run[i];
uint32_t interval_ticks = _loop_rate_hz / task.rate_hz;  // 整数除法
if (dt < interval_ticks) continue;  // 未到期，跳过
```

**示例**（Copter 400Hz 主循环）：

| 任务 | 声明 rate_hz | interval_ticks | 实际频率 |
|------|-------------|----------------|---------|
| FAST_TASK | LOOP_RATE | 1 | 400Hz |
| rc_loop | 250 | 400/250=1 | 400Hz（截断） |
| GPS update | 50 | 8 | 50Hz |
| update_batt_compass | 10 | 40 | 10Hz |
| three_hz_loop | 3 | 133 | 3.0075Hz |
| one_hz_loop | 1 | 400 | 1Hz |

---

## 5. SCHED_LOOP_RATE 参数（`AP_Scheduler.cpp:68`）

```cpp
AP_GROUPINFO("LOOP_RATE", 1, AP_Scheduler, _loop_rate_hz, SCHEDULER_DEFAULT_LOOP_RATE)
```

- Copter/Sub 默认：400Hz
- Plane/Rover 默认：50Hz
- 范围：50-2000Hz
- **重启生效**

---

## 6. FAST_TASK 优先级（`AP_Scheduler.h:99-103`）

```cpp
enum FastTaskPriorities {
    FAST_TASK_PRI0 = 0,
    FAST_TASK_PRI1 = 1,
    FAST_TASK_PRI2 = 2,
    MAX_FAST_TASK_PRIORITIES = 3
};
```

- 优先级槽位 0-2 保留给 FAST_TASK
- 每 tick 必跑，不分频，不受时间预算门控
- 普通任务优先级从 3 开始

---

## 7. Plane 任务表（`ArduPlane/ArduPlane.cpp:56`）

```cpp
const AP_Scheduler::Task Plane::scheduler_tasks[] = {
    // FAST_TASK 区（优先级 0-2）
    SCHED_TASK(ahrs_update,           LOOP_RATE,  50, 0),   // :58
    SCHED_TASK(update_control_mode,   LOOP_RATE,  50, 1),   // :59
    SCHED_TASK(stabilize,             LOOP_RATE,  50, 2),   // :60
    SCHED_TASK(set_servos,            LOOP_RATE,  50, 2),   // :61
    
    // 普通任务（优先级 3+）
    SCHED_TASK(read_radio,            50,   100, 6),        // :62
    SCHED_TASK(read_mode_switch,      7,    100, 27),       // :66
    SCHED_TASK(update_GPS_50Hz,       50,   300, 30),       // :67
    SCHED_TASK(update_GPS_10Hz,       10,   400, 33),       // :68
    SCHED_TASK(navigate,              10,   150, 36),       // :69
    SCHED_TASK(update_compass,        10,   200, 39),       // :70
    SCHED_TASK(ekf_check,             10,   75,  54),       // :77
    SCHED_TASK(check_long_failsafe,   3,    400, 96),       // :95
    SCHED_TASK(one_second_loop,       1,    400, 90),       // :93
    SCHED_TASK(Log_Write_FullRate,    400,  300, 117),      // :113
};
```

---

## 8. 时间预算管理（`AP_Scheduler.cpp:249-253`）

同一 tick 内按优先级升序执行：
```cpp
time_available = 循环周期 - 已耗时
if (task.max_time_micros > time_available) {
    continue;  // 预算不足，跳过本 tick
}
```

**自适应扩容**（:400-403）：
- 任务持续超时 → `extra_loop_us` 每 tick +100µs
- 上限 5ms
- 主循环整体变慢以保低频任务不丢拍

---

## 9. 双表合并（`AP_Scheduler.cpp:190-223`）

载具任务表（`ArduPlane.cpp:56`）与 `AP_Vehicle` 公共任务表按优先级交错合并。

公共任务包括：
- GCS 消息处理
- 日志写入
- 参数保存
- 传感器校准

---

## 10. 主循环之外的线程

| 线程 | 代码位置 | 职责 |
|------|---------|------|
| INS 采样 | 后端驱动 | 传感器采样 |
| DroneCAN | `AP_DroneCAN.cpp:503` | CAN 收发 |
| RC 输入 | HAL 层 | 遥控信号 |
| UART | 中断驱动 | 串口通信 |

**关键**：这些线程不受主循环阻塞影响，心跳/看门狗独立运行。

---

## 11. 对屏障同步的意义（余度调研 §3.5）

APM 单循环分频模型是屏障同步的天然优势：

1. **挂载点唯一**：`AP_Scheduler::loop()` 头部
2. **该点本就阻塞**：`wait_for_sample()` 已是等待状态
3. **统一触发**：屏障放行后所有频率任务被同一信号触发
4. **不存在 PX4 问题**："屏障挂哪个 work queue"

**屏障设计方案**（`redundancy_feasibility.md` §3.5.3）：
- 在 Task 结构加 `bool in_barrier` 标志
- 仅表决相关任务（导航/估计）入屏障
- GCS/日志留屏障外
- 10Hz 屏障周期：`interval_ticks = 400/10 = 40`

---

## 12. 看门狗集成

**主循环喂狗**（`HAL_ChibiOS_Class.cpp:316`）：
每迭代调用 `stm32_watchdog_pat()`

**监控线程**（`Scheduler.cpp:414`）：
- 100ms 周期检查主循环是否卡住
- 200ms 延迟 → 日志警告
- 500ms 延迟 → 内部错误，尝试释放 mutex
- 1800ms 延迟 → 触发 hardfault 获取 crash dump

---

## 13. 二次开发接口

添加自定义任务：
```cpp
// 在载具任务表中添加一行
SCHED_TASK_CLASS(MyModule, &my_module, update, 10, 100, <优先级>)
```

参数说明：
- 类名、实例指针、方法名
- 频率 Hz（整数分频）
- 最大时间 µs
- 优先级（≥3）

**示例**：添加 10Hz 冗余管理任务
```cpp
SCHED_TASK_CLASS(Redundancy, &redundancy, update, 10, 100, 35)
```
