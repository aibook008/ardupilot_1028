# DF-200 单电机失效 SITL 测试规程

> 创建：2026-10-15 / 更新：2026-10-16（脚本 v4 验证通过）
> 环境：SITL（quadplane-df200）+ MAVProxy 故障注入（QGC 可选监控）
> 脚本：`scripts/motor_failure_test_param.lua` v4（注入）、`scripts/df200_monitor.lua`（监控）
> 关联：[failure_analysis.md](failure_analysis.md) §3.5.1（机制核实）、§5 P1 行动项

---

## 1. 背景

ArduPilot 内置电机失效检测+混控补偿机制（`AP_MotorsMatrix::check_for_failed_motor()`，`AP_MotorsMatrix.cpp:479`），但触发钩子 `thrust_loss_check()` 仅 Copter 有，Plane/QuadPlane 缺 `set_thrust_boost(true)` 调用方。本测试目标：

1. 验证 8 桨停 1 桨后姿态控制器可硬顶（协议 §5.1.5 可飞性基线）
2. 记录失效注入后的姿态/高度/电机输出数据，作为后续移植 thrust_boost 触发的对照

---

## 2. 注入脚本（v4，已验证）

`scripts/motor_failure_test_param.lua`：
- 参数触发（无需 RC 开关）：`SCR_USER1` 1=注入 / 0=恢复
- `MOT_STOP_BITMASK`：bitmask 选电机（bit0=电机1）
- Lua 绑定注意：`SRV_Channels:find_channel(fn)` 返回**第一个值是通道号**（数字=找到），探测结论见 §7 已知问题
- 15ms 超时自恢复安全机制

---

## 3. 操作流程

```bash
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-df200 \
    --out udpout:127.0.0.1:14552 --console --map
```

```
param show MOT_STOP_BITMASK          # = 2（停电机2）
arm throttle
# 地面先验证：SCR_USER1 1/0，C6 应 1000/恢复
mode qhover
takeoff 10
# 悬停稳定 ≥10s
param set SCR_USER1 1                # ★ 飞行中注入
# 观察 10-20s
param set SCR_USER1 0
mode qland
```

---

## 4. 验证结果（log 00000030.BIN，2026-10-16）

### 4.1 飞行中注入 ✅ 生效

时间线：GUIDED 起飞爬升至 ~10m 悬停（209–215s）→ 稳定悬停 → **注入 227.7s**（SCR_USER1=1）→ 悬停观察 → QLAND（245.2s）→ 落地（264s）。

**电机输出（注入前 216–227s vs 注入中 228–244s）：**

| 通道 | 注入前均值 | 注入中均值 | 注入中 min/max | 说明 |
|------|-----------|-----------|---------------|------|
| C6（电机2） | 1684 | **1000 钉死** | 1000/1000 | ✅ 停转成功 |
| C8（RR vert） | 1683 | **1744** | 1479–1950 | 补偿抬升 +60 |
| C10 | 1615 | **1806** | 1538–1950 | 补偿抬升 +191（最大） |
| C9 | 1617 | 1656 | 1150–1932 | +39 |
| C5/C7/C11/C12 | ~1684/1616 | 1669/1615 | — | 基本不变 |

剩余电机出现明确的差动补偿重分配（C8/C9/C10 抬升、对侧通道不变），姿态控制器在顶住失效电机。

### 4.2 飞行状态响应（8 桨冗余裕度大，扰动小）

| 指标 | 数值 |
|------|------|
| 姿态最大偏差 | Roll 0.19° / Pitch 0.29°（注入前基线 max 0.22°/0.19°——同量级） |
| 最大下沉率 | 0.31 m/s（228.06s，注入后 0.4s） |
| 高度跌落 | 9.95 → 9.47m（-0.48m，~2s 内定高回路拉回） |
| 定高恢复 | ~2s 回 DAlt |
| Failsafe | 无 EKF/姿态 failsafe 触发 |

**结论**：SITL 无风+8 桨推力裕度下，停 1 桨（电机2=FL tilt 40in）姿态几乎无感、掉高 <0.5m。协议 §5.1.5 可飞性验证通过（此工况）。

### 4.3 历史排查记录（供参考）

- log 28：脚本 v1 的 `find_channel` 出参绑定误用 → 注入无效（静默空转）
- log 29：v4 地面注入验证（C6=1000 钉死 41s，182–224s）
- log 30：v4 飞行中注入验证（本节）

---

## 5. 后续测试矩阵（待做）

| # | 工况 | bitmask | 预期 |
|---|------|---------|------|
| 1 | 停对侧电机（RR tilt M4） | 8 | 姿态偏差应仍小（对称位） |
| 2 | 停外桨（M5–M8，60in） | 16/32/64/128 | 推力损失更大，掉高更明显 |
| 3 | 停同侧两桨 | 3（M1+M2） | 接近推力裕度边界 |
| 4 | FW 巡航中停升力桨 | — | 无影响（SHUTDOWN 正常） |
| 5 | 转换中停桨 | — | 需谨慎，Q_ASSIST 兜底 |

判据记录：姿态最大偏差、掉高、恢复时间、failsafe 触发。

---

## 6. 观察判据

| 观察项 | 预期 |
|--------|------|
| QGC/MAVProxy 姿态球 | 抖动后回稳 |
| 高度 | 掉高后定高恢复 |
| Messages | 无 EKF/姿态 failsafe |
| DF200 monitor | 恒报 MotorOK（`_thrust_boost` 恒 false，Plane 缺触发钩子，§3.5.1） |
| 日志 RCOU | 目标通道 1000 钉死，对侧电机抬升 |

---

## 7. 已知问题与坑

1. **Lua `find_channel` 绑定返回顺序**：第一个返回值是通道号（数字），不是 bool。v3 及之前误判导致静默 NOT FOUND。探测脚本定位（`fn=37 -> ret=8` = SERVO9 规律）。
2. **`param:add_table` 必须在脚本里创建参数**：`MOT_STOP_BITMASK` 由脚本创建（KEY=75），首次运行后需重启才 init 成功。
3. **脚本返回值**：Lua 脚本必须返回 0 个或 2 个值（`return update, 1000`），返回 `nil` 会报 "bad result count"（`lua_scripts.cpp:391`）。
4. **手动飞降落**：`land` 命令需要任务含 DO_LAND_START，手动飞用 `mode qland`。
5. **QGC 陈旧报错**：v3 时代的 "NOT FOUND" ERROR 消息可能在 QGC Messages 面板残留，v4 文本不同（`motor 2 -> func 34 -> chan 5`），重启后消失。
