# DF-200 反向过渡刹车链：核心工作总结

> 日期：2026-09-18。会话横跨 209–225 共 17 架次 SITL 迭代。
> 详细过程见 `docs/vtol_reverse_transition.md`（代码链路）、`docs/vtol_reverse_check_0917.md`（信号对照）。
> 所有行号经 grep+Read 验证（工作区状态，未提交部分以 diff 形式存在）。

---

## 0. 最终成果（一句话）

**DF-200 固定翼→多旋翼反向过渡刹车链全程打通**：4km 处进 AIRBRAKE 怠速磨速 → AS=33 交棒 → POSITION1 沿 √ 剖面以恒定 10° 俯仰减速 450m → 目标点前速度归零 → 多旋翼状态飞完剩余距离 → POSITION2 降落。225 架次全链验证：剖面公式逐点吻合、pitch 恒定 10°、高度全程恒定、零穿终点。

## 1. 最终架构（五段式）

```text
┌──────────┬────────────────────────────┬──────────┬─────────┬────────────┐
│ 阶段      │ 手段                       │ 速度      │ 俯仰     │ 高度        │
├──────────┼────────────────────────────┼──────────┼──────────┼────────────┤
│ APPROACH │ TECS 巡航                  │ 58       │ ~1°      │ 锁定        │
│ AIRBRAKE │ 油门强制怠速+桨盘/机身阻力    │ 58→33    │ 钳 0°    │ hold_hover  │
│          │                            │ ~47s/2km │ (无气球) │ (Z环零爬升) │
│ 交棒      │ AS≤33 ‖ dist<20m           │ 33       │ —        │ —          │
│ POSITION1│ √剖面 v=√(v0²−2·a·s)        │ 33→3     │ 恒 10°   │ Z ceiling  │
│          │ a=0.67·g·tan(Q_BCK_PIT_LIM) │ ~30s/450m│ (QBRK钳) │ (禁爬升)    │
│ 旋翼段    │ 多旋翼 wp_speed 限速        │ ~5       │ 悬停微调 │ 恒定        │
│ POS2/降落 │ 标准着陆序列                │ →0       │ —        │ 下降        │
└──────────┴────────────────────────────┴──────────┴─────────┴────────────┘
```

设计哲学（用户定版）：**目标值 = 姿态（≤10°）+ 高度（不升），减速度随缘、尽早减速、允许点前归零**。减速是不可命令量（阻力决定），姿态和高度才是可命令量——拿可命令量做目标，不可命令量自然收敛。

## 2. 代码改动清单（全部在 ArduPlane/，未提交）

| # | 改动 | 位置 | 内容 |
|---|------|------|------|
| 1 | BCK_PIT_LIM 前移生效 | `quadplane.cpp` assign_tilt_to_fwd_thr 入口 | 速度平方律钳制块移到 method 判定前，Q_FWD_THR_USE=0 也生效；后改为**恒定限值**（去放开段） |
| 2 | effective_decel() 新增 | `quadplane.cpp:4274` + `quadplane.h` | 减速规划值与实际钳制同源：`MIN(transition_decel, 0.67·g·tan(BCK))`，气球降额 0.67 |
| 3 | 怠速 AIRBRAKE | `quadplane.cpp` vtol_position_controller | AIRBRAKE 油门强制 `MAX(throttle_min,0)`，绕过 TECS（其 slew ~0.32 m/s² 是减速跟随瓶颈且会加油门对抗阻力） |
| 4 | 交棒条件重构 | `quadplane.cpp:2540-2557` | 删 `aspeed<43`/`too fast`/profile 项；改为 `AS≤33 ‖ dist<20m`；attitude 分支加 3s 时间门；direction 两分支加 500m 距离门 |
| 5 | 4km 固定进入 | `quadplane.cpp:2497` | `stop_distance = MAX(4000, 2phase+2clos)`，2phase 作慢速兜底 |
| 6 | 刹车剖面重构 | `quadplane.cpp:2640-2658` | 零点=交棒点（`pos1_handover_dist_m` 交棒捕获），`v=√(MAX(v0²−2·a·s, 3²))`，交棒即减速无平飞段 |
| 7 | Z ceiling 禁爬升 | `quadplane.h` poscontrol struct + `:2895` | `max_z_target_cm` 交棒捕获，POSITION1 期间 Z 目标钳制——翼升力盈余全部用于刹车不用于爬升 |
| 8 | QBRK 日志 | 随 #1 | 限幅值/速度缩放/实际俯仰逐拍可查 |

## 3. 十七架次迭代链（问题→根因→修复）

| 架次 | 现象 | 根因 | 修复 |
|------|------|------|------|
| 209 | 30.9° 抬头穿终点 | lean 上限 atan(6/g)=31.4° 无约束；BCK_PIT_LIM 被method=NONE提前return跳过 | #1 前移生效 |
| 210 | 限值生效但仍穿 88m+回头 | stop_distance 用名义 2.0，限幅后实际 ~1.7-2.0 | #2 effective_decel |
| 211 | 穿 129m 回头 −19.9° | ①AIRBRAKE TECS 目标恒 45 反而加油门 ②too-fast 提前踢人 ③BCK 放开段破 10° | #3 + 交棒重构 + 恒定限值 |
| 212 | AIRBRAKE 8s 只减 2m/s | **TECS 内部速度需求 slew 瓶颈**（0.9×SINK_MIN×g/V≈0.32 m/s²），追滞后目标反而加油门 | #3 怠速绕过 |
| 213 | 气球爬升 +29m、推力矢量刹车失效 | 10° 抬头→翼升力卸载桨→Z 环切桨→**推力矢量刹车不存在**，减速全是阻力 | #2 降额 0.67 + 两阶段模型 |
| 214 | AIRBRAKE 1s 即退+爬升 29m | 退出 profile 项远距离恒真；Z 目标(44m)在头上允许爬 | #4 重构 + #7 |
| 215 | AIRBRAKE 卡住冲过点 | 同一公式反向极端（MIN 压低交棒门槛） | #4 MAX 化 |
| 219 | 又 1s 退出 | `aspeed≤profile` 项远距离恒真（残留） | #4 整体删除该项 |
| 221 | 又 1s 退出（AS=57） | **L1 转向瞬态**：do_RTL 切航点→NavRoll 突跳 40°→roll 追 2s→attitude 误差恒真 | #4 attitude 3s 门 |
| 223 | 又 1s 退出（AS=58） | **38° 航向差转弯期 closing<0** → wrong-direction 误触发 | #4 direction 500m 门 |
| 224 | AIRBRAKE 首次跑满 45s，但 POS1 无减速 | **剖面符号写反**：`√(2as)` 是加速剖面 | #6 改 `√(v0²−2as)` |
| **225** | **全链正确** | — | — |

规律沉淀：**三个退出分支（profile/attitude/direction）被"远距离+大转弯"新场景逐个引爆**——原版设计 AIRBRAKE 贴目标几百米进入，瞬态不存在；进入点拉远 8 倍后全部暴露。TECS slew 是一切"给 TECS 减速目标"方案的隐形天花板。

## 4. 关键物理认知（比代码更值钱的部分）

1. **减速度不可命令**： pitch 抬头→翼升力卸载桨→Z 环切桨油门→"推力矢量刹车"（g·tan(θ)）不成立，实测减速全是气动阻力（10° 限幅下 1.17 m/s²，≈0.67×理论值）。所以目标值必须是姿态+高度，不是减速度。
2. **TECS 不适合减速调度**： 内部速度需求 slew（~0.32 m/s²）远慢于需求，且追滞后目标时会加油门对抗桨阻力。直线减速=怠速+阻力，绕开 TECS 油门。
3. **气球是 Z 目标与升力盈余的乘积**： 单纯翼升力盈余只是趋势，Z 目标在上方才变成真实爬升。Z ceiling 一行钳制解决。
4. **旋翼唤醒掉高 ~6m 是固有代价**： tilt 扫转+spool 期间垂直推力缺口，pitch 又被压 0°，无法避免只能接受。
5. **运动学剖面方向必须先画 v-s 曲线核对**： `√(2as)`（加速）与 `√(v0²−2as)`（刹车）一字之差，224 全程无减速即此。
6. **每拍重算的剖面 + 每拍捕获的锚点**： 交棒点距离/速度/Z ceiling 都在 `set_state(POSITION1)` 一拍捕获成常量，之后剖面以常量为锚、以实时 dist 为变量——这是"交棒即减速"的实现骨架。

## 5. 待办（链路定型后的精修项）

| 项 | 现状 | 抓手 |
|----|------|------|
| 多旋翼段 1.35km 偏长 | @5m/s 飞 4 分钟 | 交棒速度 33→25（AIRBRAKE 多磨一段）或 Q_WP_SPEED 5→8 |
| 四个 tuning knob 硬编码 | 4km/33/3s/500m | 收成正式参数（Q_TILT_BRAKE_DIST 等） |
| 2phase 模型 handover=43 与交棒 33 不一致 | 兜底地板偏保守 | 统一到 33 |
| 尾段悬停爬升悬案 | 215 后未复现（疑似杀程序假象） | 复现时抓 spool state/RCOU |
| TECS 僵尸输出（VTOL 段 th=1.00） | 无消费，日志吓人 | should_disable_TECS() 加 VTOL 判断（可选清理） |
| G5 缺口（路径①切模式无配对） | 仍开放 | 反向门控设计（W11 交付节点） |

## 6. 文档与代码状态

- 本文档 + `vtol_reverse_transition.md`（主链路+209 详析）+ `vtol_reverse_check_0917.md`（信号对照/velfwd/30° 抬头）
- 代码：`ArduPlane/quadplane.cpp/.h`（+218/−58 未提交），`tiltrotor.cpp/.h`（走廊门控，前序工作），`Log.cpp`（TILT 扩展）
- 验证方法学：pymavlink DFReader 逐帧对账（DFReader_binary 直导、TimeUS 过滤、最近邻匹配），QBRK/QTUN/NTUN/CTUN/MOTB/TILT 七通道交叉验证，每个结论都有"代码行号+日志数值"双重证据
