# 故障安全模块代码通读

> 适用分支：`qhz/plane_4.6.3`
> 关联文档：[技术协议摘要](technical_agreement_summary.md) §3.5 安全管理

---

## 1. 模块总览

ArduPilot Plane 的故障安全系统分布在多个层级：

| 层级 | 代码位置 | 职责 |
|------|---------|------|
| 载具层 | `ArduPlane/failsafe.cpp` | Plane 特有故障处理 |
| 事件层 | `ArduPlane/events.cpp` | 故障事件分发 |
| 系统层 | `ArduPlane/system.cpp` | 长/短故障检查 |
| EKF检查 | `ArduPlane/ekf_check.cpp` | EKF 健康监控 |
| Arming | `ArduPlane/AP_Arming.cpp` | 解锁/加锁 |
| 高级FS | `libraries/AP_AdvancedFailsafe/` | 任务级故障安全 |
| 围栏 | `libraries/AC_Fence/` | 电子围栏 |

---

## 2. Failsafe 结构（`ArduPlane/failsafe.h`）

```cpp
struct failsafe_t {
    bool rc_failsafe;          // RC 信号丢失
    bool gcs_failsafe;         // GCS 心跳丢失
    uint32_t last_heartbeat_ms;
    uint32_t last_valid_rc_ms;
};
```

---

## 3. 电池故障安全（`ArduPlane/events.cpp:243-319`）

`Plane::handle_battery_failsafe(const char *type_str, const int8_t action)`

触发电路：
```
AP_BattMonitor 检测到电压低
  → 回调 handle_battery_failsafe()
    → 根据 Failsafe_Action 分支处理
```

动作分支（`events.cpp:247-319`）：

| 动作 | 行号 | 行为 |
|------|------|------|
| `Loiter_alt_QLand` | :247-252 | QuadPlane → mode_loiter_qland，否则 QLand |
| `QLand` | :254-259 | QuadPlane → mode_qland，否则 Land |
| `Land` | :261-280 | AUTO 模式跳着陆序列，否则 RTL |
| `RTL` | :282-301 | 切 mode_rtl，原因 BATTERY_FAILSAFE |
| `Terminate` | :303-311 | AFS gcs_terminate 或直接 disarm |
| `Parachute` | :313-316 | 释放降落伞 |
| `None` | :319 | 不执行任何动作 |

**实例化**（`Plane.h:444-447`）：
```cpp
AP_BattMonitor battery{MASK_LOG_CURRENT,
    FUNCTOR_BIND_MEMBER(&Plane::handle_battery_failsafe, void, const char*, const int8_t),
    _failsafe_priorities};
```

`_failsafe_priorities`（`Plane.h:1255`）决定多电池时谁的故障优先。

---

## 4. RC 短故障安全（`ArduPlane/events.cpp:20-105`）

`Plane::failsafe_short_on_event()`

检查频率：50Hz（`ArduPlane.cpp:63` → `check_short_failsafe`）

按当前模式分支处理：

| 模式组 | 行号 | 动作 |
|--------|------|------|
| Manual/STABILIZE/ACRO/FBW/CRUISE | :28-47 | 切 FBWA/FBWB/CIRCLE |
| QuadPlane VTOL 模式 | :49-64 | 切 RTL/QRTL/QLAND |
| AUTO/GUIDED/LOITER | :67-88 | 仅当 fs_action_short != BESTGUESS |
| CIRCLE/TAKEOFF/RTL/QLAND | :89-98 | 无短故障动作 |

恢复处理（`events.cpp:217-241`）：RC 信号恢复时恢复保存的模式。

---

## 5. RC/GCS 长故障安全（`ArduPlane/events.cpp:107-215`）

`Plane::failsafe_long_on_event()`

检查频率：3Hz（`ArduPlane.cpp:95` → `check_long_failsafe`）

触发条件（`system.cpp:353-398`）：
- RC 故障 + 超时 > `fs_timeout_long`
- GCS 心跳丢失（AUTO 模式）
- RSSI 低于阈值

按模式分支：

| 模式组 | 行号 | 动作 |
|--------|------|------|
| 手动/飞行员模式 | :116-150 | 降落伞/滑翔(FBWA)/AUTO/RTL |
| QuadPlane VTOL | :152-168 | RTL/QRTL/QLAND |
| AUTO/GUIDED | :170-199 | 降落伞/滑翔/AUTO/RTL |

---

## 6. RC 输入故障检测（`ArduPlane/radio.cpp:244-299`）

`Plane::control_failsafe()`

每次读取 RC 输入时调用：
- RC 故障激活时：控制输入设为 trim 值，油门设为 min
- QuadPlane VTOL 模式下油门设为半油门
- 连续10次低油门读数 → `failsafe.rc_failsafe = true`

---

## 7. EKF 检查（`ArduPlane/ekf_check.cpp`）

检查频率：10Hz（任务表 `ArduPlane.cpp:77`）

检查内容：
- EKF 速度估计方差
- EKF 位置估计方差
- EKF 高度估计方差
- 罗盘方差

超阈值时触发：
- 降级到更保守的飞行模式
- 或触发 RTL
- 或触发 QLAND（QuadPlane）

---

## 8. AP_Arming（`ArduPlane/AP_Arming.cpp`）

### 8.1 arm() 方法（`libraries/AP_Arming/AP_Arming.h:98-101`）

```cpp
bool arm(Method method, bool do_arming_checks = true);
```

- `do_arming_checks = true`：执行全部 pre-arm 检查
- `do_arming_checks = false`：跳过检查，强制解锁（BlackBox/冗余接管用）

### 8.2 BlackBox 先例（`AP_Arming.cpp:411-429`）

Plane 的 BlackBox 模式在飞行中自动 `arm(Method::BLACKBOX, false)` / `disarm`——官方支持不经预检的运行时强制接管。冗余接管可模仿此模式，新增 `Method::REDUNDANCY`。

### 8.3 三个 armed 布尔量的收口

| 布尔量 | 位置 | 说明 |
|--------|------|------|
| `AP_Arming::armed` | 基类 | 唯一权威 |
| `AP_Motors::_armed` | 电机类 | 跟随 AP_Arming |
| `hal.util->soft_armed` | HAL层 | 跟随 AP_Arming |

载具 `AP_Arming` 子类的 arm/disarm 是唯一收口点。

---

## 9. 高级故障安全 AP_AdvancedFailsafe（`libraries/AP_AdvancedFailsafe/`）

Plane 子类：`ArduPlane/afs_plane.h:27-45`

关键方法：
- `should_crash_vehicle()`（:402-425）：`_terminate` + `TERMINATE(42)` 或 `LAND(43)`
- `gcs_terminate()`（:429-451）：GCS 发起的终止
- `max_range_update()`（:456-490）：超出 `_max_range_km` 终止

---

## 10. 电子围栏 AC_Fence（`libraries/AC_Fence/`）

支持类型：
- 多边形围栏（水平边界）
- 圆柱围栏（高度+半径）
- 高度上限/下限

触发时动作：RTL 或 LAND，取决于参数配置。

---

## 11. 协议要求 vs APM 现状

| 协议要求 | APM 实现 | 差距 |
|---------|---------|------|
| 低电量保护 | ✅ 完整 | — |
| 电子围栏 | ✅ 完整 | — |
| GNSS丢失 | ✅ 完整 | — |
| RC中断 | ✅ 完整 | — |
| 高度异常保护 | ❌ 无独立模块 | 需新增 `altitude_failsafe()` |
| 失速保护 | ❌ 无独立模块 | 需新增 `stall_failsafe()` |
| 单电机失效 | ❌ 无（Copter有throttle_boost） | 需移植或新增 |
| 姿态异常 | ⚠️ ekf_check部分覆盖 | 需扩展 |
| 电压低/转速低 | ✅ 电池+电机监控 | — |
