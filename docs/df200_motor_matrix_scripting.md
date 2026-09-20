# DF-200 / QuadPlane 脚本混控矩阵：加载原理与选型

> 对应参数：`Tools/autotest/default_params/quadplane-df200.parm`  
> 对应脚本：`scripts/df200_motor_matrix.lua`（动态）  
> 参考脚本：`scripts/jt20_motor_matrix.lua.off`（静态写法）  
> 相关：`docs/allocation_comparison_apm_px4.md`、`docs/scripts/gen_motor_matrix.py`、`docs/scripts/verify_motor_matrix.py`

---

## 1. 结论先看

| 问题 | 答案 |
|------|------|
| 混控每拍在哪算？ | **C++**（`AP_MotorsMatrix`），不是 Lua |
| Lua 干什么？ | 把系数表 **写进** C++ 数组；动态类还可按空速 **改表** |
| `Q_FRAME_CLASS=17` 稳不稳？ | 混控路径稳；风险在 **脚本生命周期 / SD 路径** |
| 静态（15）只要 load 成功？ | **对**——启动 `init` 成功后即可，不再依赖脚本循环 |
| 还要空速 fade 又要实飞可靠？ | **把动态逻辑迁到 C++** 更合适 |

---

## 2. 参数打开了什么

`quadplane-df200.parm`（要点）：

```text
Q_ENABLE         1
Q_FRAME_CLASS    17          # MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX
Q_FRAME_TYPE     0
SCR_ENABLE       1
```

机架类枚举（`libraries/AP_Motors/AP_Motors_Class.h`）：

| 值 | 名称 | 含义 |
|----|------|------|
| 1…14 | Quad/Hexa/… | 官方 `setup_*_matrix` 写死表 |
| **15** | `MOTOR_FRAME_SCRIPTING_MATRIX` | **静态**脚本矩阵：`add_motor_raw` 一次，`init` 后不可再改 |
| 16 | `MOTOR_FRAME_6DOF_SCRIPTING` | 6DoF 脚本 |
| **17** | `MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX` | **动态**脚本矩阵：可反复 `load_factors` |

DF-200 用 17，是因为要做空速衰减；15 的 `add_motor_raw` 在 `initialised_ok()` 后会被直接忽略。

---

## 3. 对象怎么创建出来

`ArduPlane/quadplane.cpp` 在 `Q_FRAME_CLASS=17` 时：

```cpp
case AP_Motors::MOTOR_FRAME_DYNAMIC_SCRIPTING_MATRIX:
    motors = NEW_NOTHROW AP_MotorsMatrix_Scripting_Dynamic(...);
```

- 混控对象是 **C++** `AP_MotorsMatrix_Scripting_Dynamic`
- 对 Dynamic：`motors->init(frame_class, frame_type)` 基本是空操作，**等 Lua 调专用 `init(N)`**
- Lua 绑定名：`Motors_dynamic`（见 `bindings.desc`）

这是上游官方机架类，不是私货钩子。

---

## 4. 脚本从哪加载

独立线程 `AP_Scripting`（通常低优先级）扫描：

| 环境 | 目录 |
|------|------|
| SITL / Linux | `./scripts/`（仓库根下 `scripts/df200_motor_matrix.lua`） |
| 实机 ChibiOS | `/APM/scripts/`（SD 卡） |
| 可选 | `@ROMFS/scripts`（嵌入 ROMFS 的脚本） |

由 `SCRIPTING_DIRECTORY`（`lua_common_defs.h`）与 `SCR_DIR_DISABLE` 控制。

**实机务必把脚本放到 `APM/scripts/`**，只放在开发机 `scripts/` 不够。

---

## 5. 两层分工（关键）

```
┌─────────────────────────────────────────────────────────┐
│ 主循环 LOOP_RATE（姿态 / 混控 / PWM）                      │
│   AC_AttitudeControl → AP_MotorsMatrix::output_*         │
│   thrust_i = thr + R*roll_i + P*pitch_i + Y*yaw_i        │
│   读的是 C++ 内存里的 _roll/_pitch/_yaw_factor[]          │
│   ★ 这里不调用 Lua                                        │
└─────────────────────────────────────────────────────────┘
                          ▲
                          │ load_factors() → memcpy
┌─────────────────────────────────────────────────────────┐
│ Scripting 线程                                           │
│   df200_motor_matrix.lua                                 │
│   return update, 100   → 约每 100 ms 跑一次               │
│   读空速 → lerp scale → Motors_dynamic:load_factors(...) │
│   ★ 只改表，不算混控                                      │
└─────────────────────────────────────────────────────────┘
```

`Scripting_Dynamic::output_to_motors()` 仅加信号量后调用基类——**混控不经过 Lua VM**。

---

## 6. 动态脚本（CLASS=17）启动序列

`scripts/df200_motor_matrix.lua`：

1. `Motors_dynamic:add_motor(i, order)` × 8  
2. `apply_factors(1,1,1)` → `load_factors(BASE)`  
3. `Motors_dynamic:init(8)` → `had_table` 已置位且电机数匹配 → `set_initialised_ok(true)`  
4. GCS：`DF-200 dyn: AS fade; R→… P→… Y→…`  
5. 之后 `update()`：AS∈[15,25] 线性衰减 R/P/Y 到 `*_FLOOR`（当前多为 0）

BASE 系数约定（与 `setup_quad_matrix` / `add_motor` 相同）：

```text
angle = atan2(Y, X)
roll  = cos(angle + 90°) = -sin(angle)
pitch = cos(angle)
yaw   = ±1   （旋向；与力臂长度无关）
```

可用 `docs/scripts/gen_motor_matrix.py --vehicle df200 --format lua` 生成/核对。

---

## 7. 静态脚本（CLASS=15）

典型写法（JT20 off 脚本思路）：

```lua
MotorsMatrix:add_motor_raw(i, roll, pitch, yaw, order)
assert(MotorsMatrix:init(8), "Failed to init …")
```

### 「只要 load 成功就可以了吗？」

**对。** 静态路径：

- 成功标志：`init(N)` 返回 true + 脚本自报成功日志 + 预臂通过  
- `init` 之后 **不能再改** 矩阵（`add_motor_raw` 被拒）  
- 脚本线程之后即使挂掉：系数已在 C++ 数组里，**混控继续用那张静态表**

不适合还要空速 / 倾转过程中改 RPY 增益的需求。

---

## 8. 「会不会掉」——风险边界

### 相对稳的部分

- 系数进入 C++ 后，混控与官方 Matrix 同路径（Dynamic 另有 `_sem` 防改表与出 PWM 打架）  
- 脚本在 **init 成功后** 被 `remove_script`：**不会清空** `_roll/_pitch/_yaw_factor`  
  → 保留**最后一次** `load_factors` 的结果  
- `AP_Scripting::arming_checks`：线程失败、OOM、Lua error、CRC 不符可拦解锁  

### 真正风险

| 风险 | 静态 15 | 动态 17 |
|------|---------|---------|
| 起飞前脚本未成功 init | 混控未就绪 | 同左 |
| SD / 路径错误 | 同左 | 同左 |
| 运行中脚本报错 / overtime | 表已固定，影响小 | **fade 停更**，表冻结在最后一次 scale |
| 依赖脚本持续调度 | 否 | 是（改表） |

**不是**「每拍用 Lua 算混控导致抖动」。  
**是**「配置与生命周期依赖脚本环境」。

---

## 9. 选型建议

| 需求 | 建议 |
|------|------|
| 矩阵固定、要简单可靠 | `Q_FRAME_CLASS=15`，启动看到 init/load 成功即可 |
| SITL 调空速 fade | 可继续 `17` + Lua |
| 实飞 / 定型仍要 AS fade | **把 BASE + `lerp_scale` 迁到 C++**（或固件内按空速改 factor） |
| 完全不依赖脚本 | C++ `setup_df200_matrix`（或等价）+ 可选参数化 AS_LO/HI/FLOOR |

迁 C++ 时逻辑对齐现有 Lua：

- `AS_LO=15`, `AS_HI=25`  
- `scale = lerp(1 → FLOOR)`  
- `factor[i] = BASE[i] * scale`（R/P/Y 可分开 floor）  

生成 BASE：`docs/scripts/gen_motor_matrix.py`  
核对对应：`docs/scripts/verify_motor_matrix.py`

---

## 10. 实飞检查清单（若暂用 Lua）

1. `SCR_ENABLE=1`，脚本在 **`APM/scripts/df200_motor_matrix.lua`**  
2. 设置并核对 `SCR_LD_CHECKSUM` / `SCR_RUN_CHECKSUM`（或 DEBUG 保存 CRC）  
3. 上电 GCS 出现 `DF-200 dyn: AS fade...`；没有则不要飞  
4. 过渡段观察 `RSCALE` / `PSCALE` / `YSCALE` / `AS_MIX`；停更说明脚本已挂或未调度  
5. `ARMING_CHECK` 实飞勿长期关（SITL 调试除外）  

---

## 11. 相关代码索引

| 位置 | 内容 |
|------|------|
| `ArduPlane/quadplane.cpp` | `Q_FRAME_CLASS=17` → `AP_MotorsMatrix_Scripting_Dynamic` |
| `libraries/AP_Motors/AP_MotorsMatrix_Scripting_Dynamic.*` | `load_factors` / `init` / 带锁 `output_to_motors` |
| `libraries/AP_Motors/AP_MotorsMatrix.cpp` | 混控公式、`add_motor`、`normalise_rpy_factors`、CLASS=15 的 scripting `init` |
| `libraries/AP_Scripting/AP_Scripting.cpp` | 线程、`arming_checks`、CRC |
| `libraries/AP_Scripting/lua_scripts.cpp` | 加载、超时、`remove_script` |
| `libraries/AP_Scripting/lua_common_defs.h` | `SCRIPTING_DIRECTORY` |
| `scripts/df200_motor_matrix.lua` | DF-200 动态 BASE + AS fade |

---

## 12. 和「写死进代码」的关系

别人说「得写死、脚本容易掉」：

- **说对的部分**：实飞依赖 SD/脚本线程，动态 fade 在脚本挂后会停更  
- **说偏的部分**：并不是混控在 Lua 里跑；也不是官方不支持脚本矩阵（15/17 都是正式 `FRAME_CLASS`）

**矩阵固定 → 静态 15 或 C++ 表，看到 load/init 成功即可。**  
**还要动态 → 定型阶段用 C++ 实现同一套 fade，比长期绑 Lua 更合适。**
