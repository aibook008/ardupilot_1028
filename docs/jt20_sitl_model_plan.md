# JT-20 SITL 物理模型（jt20.json）与参数草稿

> 对应 log 分析：`00000013.BIN` 起，QLOITER 悬停可保持，但**起飞大油门低头瞬态大、水平被顶出 5 m/s**。
> 根因：SITL 仍用默认小多旋翼模型（3kg×1.5、0.35m 力臂），与 JT-20 真实质量/力臂不符。
> 本文件：给出 `Tools/autotest/models/jt20.json` 与参数草稿，让 SITL 按真实 JT-20 建模。

---

## 1. 为什么建 JSON 模型

`Frame::init()` 无 `xxx.json` 时用 `default_model`（`SIM_Frame.h`）：

| 量 | 默认 | JT-20 实际 |
|----|------|-----------|
| mass | 3.0（QuadPlane 再 ×1.5 → **4.5**） | **10.255**（SDF 总和） |
| diagonal_size | 0.35 m | **1.99 m** |
| num_motors | 4 | **8** |
| 电机位置 | `angle*0.35` 量级 | SDF 实测臂长 ~0.4–1.0 m |

后果：SITL 力臂/质量/惯量都比真机小得多 → 同样油门下 pitch 瞬态被放大、yaw/roll 权威不足。

---

## 2. `Tools/autotest/models/jt20.json`

已创建。要点：

- `mass` **10.255**（SDF 全链路总和，含 10kg 机身）
- `diagonal_size` **1.99**
- `num_motors` **8**
- `moment_inertia` `[2.168, 1.242, 3.016]`（SDF base_link ixx/iyy/izz，近似整机）
- 每个 `motorN_position` = SDF rotor 坐标换算到 APM 轴（X前 Y右 Z下），`motorN_yaw` = +1(CW)/−1(CCW)，与 Lua / `SIM_Frame.cpp` 一致
- `hoverThrOut 0.55`（初值，需从 `CTUN.ThO` 校准）
- `disc_area 1.45`（按 8×0.48m 桨估算）
- `refVoltage/maxVoltage 25.2`（6S）

字段名与 `load_frame_params()` 一致：`mass / diagonal_size / num_motors / moment_inertia / motorN_position / motorN_vector / motorN_yaw / ...`。

---

## 3. 如何使用（关键）

frame 字符串里带 json 路径，SITL 才加载：

```bash
./Tools/autotest/sim_vehicle.py -v ArduPlane \
  -f quadplane-jt20:Tools/autotest/models/jt20.json -w
```

不带 json 时仍是默认 4.5kg 模型（保持现状）。`vehicleinfo.py` 里 `quadplane-jt20` 的默认 frame 目前没有带 json；建议只在测试命令里显式指定（避免影响其它用例）。

## 4. 参数草稿（quadplane-jt20.parm 已更新）

关键更动：

```text
Q_RTL_MODE 3        # RTL 时始终用 QRTL（垂起返航），不再转固定翼
```

其余保持上一版已验证参数：

```text
Q_ENABLE         1
Q_FRAME_CLASS    15        # Lua 自定义混控
Q_FRAME_TYPE     0
Q_TILT_TYPE      0         # 无差动（先做基本悬停）
Q_TILT_YAW_ANGLE 0
Q_TILT_MASK      15        # M1-M4 倾转
Q_OPTIONS        2097152   # DISARMED_TILT_UP
SCR_ENABLE       1
SERVO5..12       33..40    # Motor1-8
SERVO13/14       41/41     # 统一倾转
```

## 5. 建议的调参/实验顺序

1. **先验证 json 加载**：启动后日志应显示 `Loaded model params from jt20.json`。
2. **检查质量/推力**：日志 `QTUN.ThO`（悬停油门）、`CTUN`、`SIM2`；若悬停油门不在合理区间，调 `hoverThrOut`。
3. **QLOITER 起飞**：油门**缓推**，观察起飞瞬间 pitch 瞬态是否明显减小（对比 `00000013` 的 −31°）。
4. **若仍低头/抬头**：再考虑调 `Q_A_RAT_PIT_*` 增益或 `Q_TILT_MAX`。
5. 悬停稳了再开差动：`Q_TILT_TYPE=2`、`Q_TILT_YAW_ANGLE=10`、SERVO13/14=75/76。

## 6. 待办/注意

- `hoverThrOut`、`refCurrent/refSpd`、`disc_area` 都是初值，应按真机试飞/电机选型重新标定。
- `moment_inertia` 只用 base_link 近似；若后续做精细仿真，应把各 link 惯量平移到整机 CG 再合成。
- `jt20_motors[]`（`SIM_Frame.cpp`）里的 `Motor` 角度只用于「无 json」分支；json 加载后 `setup_params` 会用 json 的 `motorN_position`（非零则覆盖），但 **倾转 `pitch_servo` 只存在于 `jt20_motors[]` 构造**——所以内四倾转仍靠 `SIM_Frame.cpp` 的 `pitch_servo` 字段，json 里不要写 `motorN_position` 为零、否则角度重建可能与 SITL 混控不一致（当前 json 的 position 与 `jt20_motors` 几何一致，没问题）。
