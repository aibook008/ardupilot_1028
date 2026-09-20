# JT-20 倾转过渡功率分析

| 项 | 内容 |
|----|------|
| 代码/模型 | Plane 4.6.x / `qhz/plane_4.6.3`，`Tools/autotest/models/jt20.json` |
| 脚本 | `docs/scripts/jt20_power_from_log.py` |
| 产出 CSV | `docs/scripts/out/` |
| 对比日志 | `00000023.BIN`（无 MOTT，RCOU 反推）vs `00000024.BIN`（有 **MOTT**） |

## 1. 目的

把「电机谱 / 推力分配」和「倾转过程功率」放到同一套 0~1 油门语义上，为后续功率走廊、电池与散热分析做底：

1. 集体油门 `QTUN.ThI` vs 各电机混控后油门  
2. 倾转 WAIT / TIMER / DONE 各阶段功率  
3. 旧算法（RCOU 反推）与新日志 **MOTT**（转 PWM 前真值）的差异  

## 2. 功率模型（与 SITL 盘模型一致）

来自 `Frame::init` + `SIM_Motor::calc_thrust`（见 `docs/vtol_jt20.md` §9.2）。当前 json 标定：

| 量 | 值 |
|----|-----|
| mass（标定） | 10.255 kg |
| refCurrent × refVoltage | 45 × 25.2 → **1134 W** |
| hoverThrOut | 0.35 |
| propExpo | 0.65 |
| velocity_max | **38.12 m/s** |
| A_eff（每桨） | **0.0404 m²** |
| power_factor = P_hover/T_hover | **11.276 W/N** |

对每个电机归一化推力 \(u\in[0,1]\)（满电、悬停来流≈0）：

```text
v_out = velocity_max * sqrt((1-expo)*u + expo*u^2)
T     = 0.5 * ρ * A_eff * v_out^2          # N
P     = power_factor * T                   # W  （诱导功率近似）
```

总功率 \(P_{\mathrm{tot}}=\sum P_i\)；内四 \(P_{\mathrm{in}}=\sum_{1..4} P_i\)，外四 \(P_{\mathrm{out}}=\sum_{5..8} P_i\)。

> 这是 **SITL 气动功率代理**，不是 ESC 电功率。SITL 的 `ESC.Curr` 不可信；实机应用 `Volt×Curr`。

## 3. 两种电机油门来源

| 来源 | 含义 | 适用 |
|------|------|------|
| **MOTT.Mot1…Mot8** | `_thrust_rpyt_out[i]`：混控 + `tilt_compensate` 之后、**转 PWM 之前**，与 `ThI` 同量纲 0~1 | `00000024` 及之后（推荐） |
| **RCOU 反推** | 由 C5…C12 + `Q_M_SPIN_*/THST_EXPO` 反解 thrust | 旧 log（如 `00000023`） |

MOTT 写入点：`AP_MotorsMatrix::output_to_motors()`，armed 时约 50 Hz。

RCOU 反推（旧 log）：

```text
actuator = (PWM-1000)/1000
y = (actuator - spin_min)/(spin_max - spin_min)   # 夹在 spin 区间
反解 y = (1-e)*u + e*u^2  →  u
```

## 4. 阶段功率对比（脚本汇总）

命令：

```bash
python3 docs/scripts/jt20_power_from_log.py \
  logs/00000023.BIN logs/00000024.BIN \
  --out-dir docs/scripts/out
```

### 4.1 分阶段均值

**00000023（RCOU 反推）**

| phase | n | ThI | u_in | u_out | T_tot(N) | P_tot(W) | P_in | P_out | Pmax |
|-------|---|-----|------|-------|----------|----------|------|-------|------|
| WAIT/VTOL | 1517 | 0.24 | 0.44 | 0.45 | 109 | 1229 | 613 | 616 | 3196 |
| WAIT_tilt | 200 | 0.18 | 0.50 | 0.47 | 115 | 1299 | 698 | 601 | 2808 |
| TIMER | 50 | 0.22 | 0.56 | 0.37 | 113 | 1273 | 828 | 445 | 2590 |
| DONE/FW | 1478 | 0.37 | 0.15 | 0.00 | 13 | 145 | 144 | 0 | 1877 |

**00000024（MOTT 真值）**

| phase | n | ThI | u_in | u_out | T_tot(N) | P_tot(W) | P_in | P_out | Pmax |
|-------|---|-----|------|-------|----------|----------|------|-------|------|
| WAIT/VTOL | 358 | 0.55 | 0.59 | 0.59 | 127 | **1432** | 715 | 716 | 1632 |
| WAIT_tilt | 259 | 0.53 | 0.69 | 0.54 | 136 | **1537** | 907 | 630 | 3010 |
| TIMER | 99 | 0.28 | 0.51 | 0.22 | 85 | **953** | 745 | 208 | 2033 |
| DONE/FW | 502 | 0.33 | 0.01 | 0.00 | 2 | **23** | 23 | 0 | 1668 |

说明：`00000023` 的 WAIT/VTOL `ThI` 均值被起飞前大量零油门样本拉低；看倾转过程应以 **对齐后→FW done** 窗口为准（下一节）。

### 4.2 倾转窗口细看（对齐 → FW done）

#### 00000024 / MOTT（可信）

| 阶段 | 典型 t | ThI | Tilt | u_in | u_out | P_in(W) | P_out(W) | P_tot(W) |
|------|--------|-----|------|------|-------|---------|----------|----------|
| 悬停对齐 | ~67.5 | 0.55 | 0 | 0.59 | 0.59 | ~700 | ~700 | **~1400** |
| WAIT 倾转爬升 | 68.0–68.5 | 0.35–0.42 | 19→40 | 0.48–0.51 | 0.37–0.48 | 510–565 | 360–520 | 870–1090 |
| WAIT @45° | 69.5 | 0.56 | 45 | **0.83** | 0.59 | **~1200** | ~700 | **~1900** |
| WAIT 稳态 | 70–71.5 | 0.45 | 45 | 0.67–0.78 | 0.48–0.55 | 860–1090 | 510–640 | 1370–1730 |
| TIMER 中 | 72.0 | 0.41 | 61 | **0.90** | 0.44 | **~1370** | ~450 | ~1830 |
| TIMER 末 | 72.5 | 0.37 | 82 | **0.99** | 0.15 | **~1600** | ~110 | ~1710 |
| DONE | ≥72.8 | — | 90 | **0** | **0** | 0 | 0 | 0※ |

※ DONE 后 MOTT≈0：VTOL 电机 spool 关闭，`_thrust_rpyt_out` 不再表示前飞推力。前飞功率应改看 **`TECS.th` / `CTUN.ThO` / `AETR.Thr`**（或后续给 `output_motor_mask` 另加日志）。

#### 00000023 / RCOU 反推（对照）

| 阶段 | ThI | Tilt | u_in | u_out | P_tot 量级 |
|------|-----|------|------|-------|------------|
| 对齐悬停 | 0.55→0.36 | 0 | ~0.67–0.82 | ~0.67–0.82 | 1.7–2.3 kW |
| WAIT @45° | 0.58 | 45 | **0.96** | 0.84 | **~2.7 kW** |
| TIMER 末 | 0.37 | 87 | **1.00** | 0.22 | ~1.8 kW |
| DONE 初 | 0.37 | 90 | 0.85→降 | 0 | 从 ~1.2 kW 衰减 |

RCOU 反推在倾转段往往把内四估得更满、总功率偏高（PWM 饱和 + expo 反解），**趋势与 MOTT 一致，绝对值以 MOTT 为准**。

## 5. 物理解读（倾转功率谱）

1. **垂直悬停**：内外功率接近均分，总功率 ~1.4 kW（MOTT）。  
2. **WAIT 倾转到 45°**：  
   - 集体 `ThI` 可能先掉再抬；  
   - **内四功率明显上升**（`1/cosθ` + 姿态差动）；  
   - 外四仍承担大部分垂向，功率略降或持平；  
   - **峰值总功率出现在倾转过程，而不是纯悬停**（MOTT 峰值约 3.0 kW）。  
3. **TIMER**：外四功率塌缩（垂向辅助按时间撤）；内四短期打满维持前向/垂向分量，总功率从 ~1.8 kW 降到 FW 交接。  
4. **DONE/巡航掉高窗口**：MOTT 归零 ≠ 整机无功率；TECS 前飞油门在重建（见 `throttle_calculation.md` §2.6.1），BARO 掉高 ~5 m 与「垂起分量消失、前飞未接稳」一致。

```text
功率重心迁移（示意）:

悬停:   P_in ≈ P_out
WAITθ↑: P_in ↗  P_out →
TIMER:  P_in 高位维持后降   P_out ↘↘
DONE:   MOTT=0  →  功率看 TECS/前飞通道
```

## 6. 与「电机谱」怎么联合

| 量 | 用途 |
|----|------|
| `ThI` | 集体需求基线 |
| `MOTT.Mot1..4` / `u_in` | 倾转电机谱（补偿后） |
| `MOTT.Mot5..8` / `u_out` | 固定垂起电机谱 |
| `P_in/P_out/P_tot` | 阶段功率与峰值 |
| `TILT` + `Trn` | 对齐功率曲线的飞行阶段 |
| `BARO`/`AHR2` | FW 段高度（勿用停录后的 QTUN.Alt） |

后续可做：

- \(P(\theta)\)、\(P(V)\) 走廊  
- 电池：\(\int P_{\mathrm{tot}}\,dt\) 粗算能耗（SITL 代理）  
- 对比提高 `AIRSPEED_MIN` / `Q_TRANSITION_MS` 后 TIMER→DONE 功率塌陷与掉高是否减轻  

## 7. 脚本用法

```bash
# 依赖: pymavlink, numpy
python3 docs/scripts/jt20_power_from_log.py logs/00000024.BIN --out-dir docs/scripts/out

# 新旧对比
python3 docs/scripts/jt20_power_from_log.py \
  logs/00000023.BIN logs/00000024.BIN \
  --out-dir docs/scripts/out
```

产出：

| 文件 | 内容 |
|------|------|
| `docs/scripts/jt20_power_from_log.py` | 分析脚本 |
| `docs/scripts/out/*_power_timeseries.csv` | 逐时刻 u/T/P |
| `docs/scripts/out/jt20_power_phase_compare.csv` | 分阶段对比 |

## 8. 结论

1. **MOTT 已能全程（armed）记录转 PWM 前各电机 0~1 推力**，应作为功率分析主数据；RCOU 反推仅兼容旧 log。  
2. 倾转过程中功率峰值出现在 **WAIT 倾转 + TIMER 前段**，内四占比上升、外四下降，与 `tilt_compensate` / TIMER 衰减逻辑一致。  
3. **DONE 后 MOTT≈0 正常**；巡航功率与掉高分析要接 TECS/BARO，不能只用 MOTT。  
4. 下一步建议：用同一任务，在提高 `AIRSPEED_MIN` 或拉长 `Q_TRANSITION_MS` 后再跑一局，对比 TIMER→DONE 的 `P_out` 塌陷速度与 BARO 谷底掉高。

## 9. 相关文档

- `docs/throttle_calculation.md` §2.5–2.6.1（油门状态机、掉高与 TECS 接棒）  
- `docs/vtol_jt20.md` §9.2（盘模型数值）  
- `docs/throttle_calculation.md` §2.5.6（MOTT 字段说明）  
