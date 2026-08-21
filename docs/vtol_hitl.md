# ArduPilot 倾转旋翼 HITL（SimOnHW）

> Plane 4.6.x。工具：QGC + Pixhawk。  
> **HITL = 真 MCU 跑固件 + 传感器/物理全部来自板内 SITL 模型（不用板载 IMU）。**  
> 与桌面 SITL **共用** `libraries/SITL` 动力学；差别在「模型怎么被选中、谁每拍驱动它」。

---

## 1. 一句话对比

| | 桌面 SITL | HITL（SimOnHW） |
|--|-----------|-----------------|
| 命令/入口 | `sim_vehicle.py -v ArduPlane -f quadplane-tilttrivec -w` | 编译烧录 `*-SimOnHardWare` + QGC 加载 parm |
| 模型怎么选 | **运行时** `--model <frame>` | **编译期** `AP_SIM_FRAME_CLASS` + `AP_SIM_FRAME_STRING` |
| 模型实现 | 同：`SITL::QuadPlane` 等 | 同 |
| 传感器 | SITL 后端 | SITL 后端（板载 IMU 不探测） |
| 谁每拍步进 | `AP_HAL_SITL` 主循环 | `SIMState::update()` + `RCOutput` 写 `pwm_output[]` |

所以：SITL 一条命令就能换机架；HITL 必须在 **hwdef 里写死** 同一套 frame 字符串，并打开 `SIM_ENABLED`。

---

## 2. 为什么 SITL 能直接：

```sh
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane-tilttrivec -w
```

### 2.1 脚本层（运行时选型）

1. `-v ArduPlane` → 编/跑 `bin/arduplane`，vehicle 配置在 `Tools/autotest/pysim/vehicleinfo.py`。
2. `-f quadplane-tilttrivec` → `options_for_frame()`：
   - 在 `frames` 表里命中该 key；
   - 表项**没有**单独 `"model"` 字段时：`ret["model"] = frame`，即 **`model = "quadplane-tilttrivec"`**；
   - 同时挂上 `default_params/quadplane.parm` + `quadplane-tilttrivec.parm`。
3. `-w` → 擦 EEPROM 类状态，用本次 defaults 干净启动。
4. `sim_vehicle.py` 最终给二进制传：`--model quadplane-tilttrivec`（以及 defaults 路径等）。

### 2.2 二进制层（模型表 + 前缀匹配）

`libraries/AP_HAL_SITL/SITL_cmdline.cpp`：

```cpp
// 注册表（节选）
{ "quadplane", QuadPlane::create },
{ "firefly",   QuadPlane::create },
{ "plane",     Plane::create },
// ...

// 创建：前缀不敏感匹配
strncasecmp(model_constructors[i].name, model_str, strlen(name)) == 0
→ sitl_model = QuadPlane::create("quadplane-tilttrivec");
```

- `"quadplane-tilttrivec"` **前缀**匹配 `"quadplane"` → 构造函数是 **`QuadPlane::create`**。
- 完整字符串仍传给 `QuadPlane(frame_str)`，供内部 `strstr` 区分 tilttri / tilthvec / tilt 等。

因此：**不用改代码**——frame 名在 vehicleinfo 有表项、且以 `quadplane` 开头，就会走到 `SIM_QuadPlane`。

### 2.3 与 HITL 的差别（为什么 HITL 不能只敲同一条命令）

| 桌面 SITL 已具备 | HITL 默认没有 |
|------------------|---------------|
| `AP_HAL_SITL` + 命令行 `--model` | 真机是 `AP_HAL_ChibiOS`，**没有**这套 model 表解析 |
| 任意 `-f` 换模型 | 板内只有 `SIMState`：`create(AP_SIM_FRAME_CLASS, AP_SIM_FRAME_STRING)`，**宏写死** |
| 自动加载 frame 的 parm | 需 QGC **手动**加载同一套 parm |
| 默认板型 `sitl` 已链 SITL 库 | 真机板必须 `include SimOnHW.inc`（`SIM_ENABLED 1`）才会链 SITL、挂仿真传感器 |

**HITL 要额外增加的（对应上面缺口）：**

1. **hwdef**：`include SimOnHW.inc`（打开仿真栈、不用真 IMU）。  
2. **hwdef**：`define AP_SIM_FRAME_CLASS QuadPlane`  
3. **hwdef**：`define AP_SIM_FRAME_STRING "quadplane-tilttrivec"`（或你要的 `-f` 同名字符串）  
4. **hwdef**：`AUTOBUILD_TARGETS Plane`（SimOnHW 示例默认常是 Copter）  
5. **烧录后参数**：与 SITL 相同的 `quadplane.parm` + 变体 parm（QGC）。  
6. **板型**：如 `Pixhawk6X-SimOnHardWare`（基于 `Pixhawk6X` + 上述宏），不是量产 `Pixhawk6X` 固件名。

加这些之后，板子上跑的仍是 **`QuadPlane::create("quadplane-tilttrivec")` 同一路径**，只是入口从 cmdline 换成编译宏。

---

## 3. SITL 模型初始化与核心调用链

### 3.1 桌面 SITL：从进程启动到第一拍

```
arduplane --model quadplane-tilttrivec ...
  └─ SITL_State 解析 --model
       └─ model_constructors[] 前缀匹配 "quadplane"
            └─ sitl_model = QuadPlane::create(model_str)   // NEW QuadPlane(frame_str)
                 ├─ Plane(frame_str) 基类
                 ├─ strstr 定 frame_type / motor_offset / thrust_scale
                 ├─ frame = Frame::find_frame(frame_type)
                 ├─ frame->motor_offset = …
                 └─ frame->init(frame_str, &battery)
  主循环中 sitl_model->update(input) / update_model …
```

核心符号：

| 步骤 | 函数/表 | 文件 |
|------|---------|------|
| 模型注册 | `model_constructors[]` | `AP_HAL_SITL/SITL_cmdline.cpp` |
| 创建 | `QuadPlane::create` → `QuadPlane::QuadPlane` | `SITL/SIM_QuadPlane.cpp` |
| 查电机布局 | `Frame::find_frame` | `SITL/SIM_Frame.cpp` |
| 气动/电机参数 | `Frame::init` | `SITL/SIM_Frame.cpp` |
| 每拍动力学 | `Aircraft::update` / `QuadPlane::update` | `SIM_Aircraft` / `SIM_QuadPlane` |
| 每拍旋翼力 | `Frame::calculate_forces` → `Motor::calculate_forces` | `SIM_Frame` / `SIM_Motor` |

### 3.2 SimOnHW：从开机到第一拍

```
固件启动（ChibiOS）
  └─ 传感器 detect：AP_SIM_INS_ENABLED → 只挂 AP_InertialSensor_SITL 后 return
  └─ 首次 SIMState::update()                    [AP_HAL/SIMState.cpp]
       └─ sitl_model = SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING)
            // 等价于 compile-time 的 QuadPlane::create("quadplane-tilttrivec")
            └─ 之后同桌面：QuadPlane 构造 → find_frame → init

每拍：
  控制律 → RCOutput::write
       └─ hal.simstate->pwm_output[chan] = period_us
  SIMState::update()
       └─ fdm_input_local()
            ├─ _simulator_servos(input)      // pwm_output → sitl_input.servos
            ├─ sitl_model->update_model(input)
            │    └─ QuadPlane::update(input)
            └─ fill_fdm(_sitl->state)
  AP_InertialSensor_SITL / Baro_SITL / GPS SITL 读 sitl->state
```

核心符号：

| 步骤 | 函数 | 文件 |
|------|------|------|
| 模型创建（一次） | `SIMState::update` 内 `create` | `AP_HAL/SIMState.cpp` |
| PWM 进仿真 | `RCOutput` 写 `pwm_output` | `AP_HAL_ChibiOS/RCOutput.cpp` |
| 步进 | `SIMState::fdm_input_local` | `AP_HAL/SIMState.cpp` |
| 开关 | `SimOnHW.inc` → `SIM_ENABLED` | `hwdef/include/SimOnHW.inc` |

### 3.3 `QuadPlane` 构造：机架如何分支

`SIM_QuadPlane.cpp` 中 `QuadPlane::QuadPlane(const char *frame_str)`：

1. 默认 `frame_type = "x"`，`motor_offset = 4`（通道 0–3 留给副翼/升降/油门/方向一类固定翼输出）。  
2. `strstr(frame_str, …)` 改类型，例如：

| frame_str 含 | frame_type | 其它 |
|--------------|------------|------|
| `-tilttrivec` | `tilttrivec` | 常 `thrust_scale=0`（无独立前推桨） |
| `-tilthvec` | `tilthvec` | H 四倾转矢量 |
| `-tilttri` | `tilttri` | 三倾转 |
| `-tilt` | `tilt` | 四轴倾转 |
| `firefly` | `firefly` | `motor_offset=2` |
| `cl84` | `tilttri` | 前电机 retract |

3. `frame = Frame::find_frame(frame_type)` — 在 `supported_frames[]` **前缀**匹配。  
4. `frame->motor_offset = motor_offset`。  
5. `frame->init(frame_str, &battery)`。  
6. `mass = frame->get_mass() * 1.5`。

**`quadplane-tilttrivec` 路径：**

```
create("quadplane-tilttrivec")
  → frame_type = "tilttrivec"
  → find_frame("tilttrivec") → tilttri_vectored_motors[3]
  → motor_offset = 4
  → init → 各 Motor::setup_params
```

### 3.4 不同 motor setup 如何初始化

**布局差异** = 静态 `Motor[]` 表不同；**init 流程相同**。

`Motor` 两种构造（`SIM_Motor.h`）：

```cpp
// 固定朝上
Motor(servo, angle_deg, yaw_factor, display_order)

// 可倾转
Motor(servo, angle, yaw_factor, order,
      roll_servo, roll_min, roll_max,
      pitch_servo, pitch_min, pitch_max)
```

- `servo`：相对 `motor_offset` 的电机 PWM 下标。  
- `pitch_servo` / `roll_servo`：倾转舵机下标（同样 + `motor_offset`）；`-1` 表示不倾转。  
- 默认 `thrust_vector = (0,0,-1)`（机体 NED 向上为负 Z）。

倾转相关 `supported_frames` 名（`SIM_Frame.cpp`）：`tilthvec`、`tilt`、`tilttri`、`tilttrivec`、`firefly` 等。

`Frame::init`（所有 setup 共用）：

1. `model = default_model`（可被 `frame:json` 覆盖）。  
2. 由悬停参考算桨盘等效面积、最大速度、功率因子、阻力等。  
3. `battery->setup`。  
4. **for each motor** → `motors[i].setup_params(...)`（PWM、位置、推力方向、偏航因子…）。  
5. 缺省惯量则按质量与对角线估算。

### 3.5 每拍力与倾转（核心函数）

```
QuadPlane::update(input)
  ├─ Plane::calculate_forces     // 机翼/舵面
  ├─ frame->calculate_forces     // 旋翼
  │    └─ for i: Motor::calculate_forces(input, motor_offset, …)
  │         ├─ pwm = input.servos[motor_offset + servo]
  │         ├─ 算轴向推力
  │         ├─ 若 pitch/roll_servo>=0：
  │         │    读 input.servos[servo_idx + motor_offset]
  │         │    映射到 min..max 度，旋转 thrust / 反扭矩
  │         └─ 累加力/力矩
  ├─ 合力 → update_dynamics / update_position
  └─ update_mag_field_bf
```

飞控倾转 `SERVOx_FUNCTION`（如 75/76）必须与表中 `pitch_servo+motor_offset` 一致，否则仿真倾转角对不上（见 `docs/tilt_servo_mapping_summary.md`）。

---

## 4. 已有倾转 frame（`-f` / `AP_SIM_FRAME_STRING`）

| `-f` / STRING | 内部 frame_type | 说明 |
|---------------|-----------------|------|
| `quadplane-tilttrivec` | `tilttrivec` | 三倾转矢量（你举的例子） |
| `quadplane-tilthvec` | `tilthvec` | H 四倾转矢量 |
| `quadplane-tilt` | `tilt` | 四轴倾转 |
| `quadplane-tilttri` | `tilttri` | 三倾转 |
| `quadplane-cl84` | `tilttri`+retract | 可收放 |
| `firefly` | `firefly` | Y6 前倾转 |
| `quadplane` | `x` | 普通 X 四旋翼 VTOL（电机不倾转） |

---

## 5. HITL 实操（Pixhawk + QGC）

### 5.1 hwdef 要比 SITL **多写的内容**

目录示例：`libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X-SimOnHardWare/hwdef.dat`

```
include ../Pixhawk6X/hwdef.dat
include ../include/SimOnHW.inc          # ← SITL 桌面不需要：打开板内仿真

define CHIBIOS_SHORT_BOARD_NAME "Pixhawk6X-SimHW"

undef AUTOBUILD_TARGETS
AUTOBUILD_TARGETS Plane                 # ← 示例 SimOnHW 常默认 Copter

define AP_SIM_FRAME_CLASS QuadPlane     # ← 对应 SITL 的 QuadPlane::create
define AP_SIM_FRAME_STRING "quadplane-tilttrivec"  # ← 对应 -f 字符串

define AP_BOOTLOADER_FLASHING_ENABLED 0
```

换机架：只改 `AP_SIM_FRAME_STRING` 为上表中的 `-f` 名，**与 SITL 字符串保持一致**。

### 5.2 编译烧录

```sh
./waf configure --board Pixhawk6X-SimOnHardWare
./waf plane
./waf plane --upload
```

量产非仿真固件：`--board Pixhawk6X`（无 `SimOnHW`，**不能**当 HITL）。

### 5.3 参数（QGC）

SITL 靠 vehicleinfo **自动**加载 parm；HITL 需在 QGC 加载同等文件，例如：

- `Tools/autotest/default_params/quadplane.parm`
- `Tools/autotest/default_params/quadplane-tilttrivec.parm`

并确认：`Q_ENABLE`、`Q_FRAME_*`、`Q_TILT_*`、电机/`k_tiltMotor*` 的 `SERVOx_FUNCTION`、`GPS1_TYPE=100`、`AHRS_EKF_TYPE`（SimOnHW 常用 10）、`SIM_RATE_HZ` 等。

### 5.4 检查

1. QGC：GPS fix、姿态合理（仿真数据）。  
2. `QHOVER` 解锁、油门响应。  
3. 倾转通道随模式变化（与 tiltrivec 构型一致）。  
4. 日志可与桌面 `-f quadplane-tilttrivec` 对照。

---

## 6. 文件索引

| 内容 | 路径 |
|------|------|
| SITL 模型表 / `--model` | `libraries/AP_HAL_SITL/SITL_cmdline.cpp` |
| frame→model/parm | `Tools/autotest/pysim/vehicleinfo.py` |
| SimOnHW 创建模型 | `libraries/AP_HAL/SIMState.cpp` |
| QuadPlane 构造/update | `libraries/SITL/SIM_QuadPlane.cpp` |
| 机架表 / init / calculate_forces | `libraries/SITL/SIM_Frame.cpp` |
| 单电机与倾转 | `libraries/SITL/SIM_Motor.h` / `.cpp` |
| PWM→仿真 | `libraries/AP_HAL_ChibiOS/RCOutput.cpp` |
| SimOnHW 开关 | `hwdef/include/SimOnHW.inc` |
| 倾转舵机映射 | `docs/tilt_servo_mapping_summary.md` |

行号随分支变化，用前 `grep` 核对。

---

## 7. 总结

- **SITL 一条命令可用**：vehicleinfo 提供 frame+parm；`--model` 前缀命中 `quadplane` → `QuadPlane::create(完整 frame 字符串)` → 内部 `strstr` 选 motor 表。  
- **HITL 要增加**：`SimOnHW.inc`、编译期 `AP_SIM_FRAME_CLASS/STRING`（= 你的 `-f`）、Plane 目标、QGC 手动 parm；模型代码与 SITL **同一套**。  
- **初始化核心**：`create` → `QuadPlane` 构造 → `Frame::find_frame` → `Frame::init` / `Motor::setup_params`。  
- **运行核心**：`pwm` → `QuadPlane::update` → `Frame::calculate_forces` / `Motor::calculate_forces`（含倾转）→ `sitl->state` → 仿真传感器后端。

---

## 8. HITL 调试常见问题

记录实际搭建 Pixhawk6X-SimOnHardWare HITL 过程中遇到的问题和解决方法。

### 8.1 工具链版本问题

**现象**：
```
ChibiOS build requires g++ version 10.2.1 or later, found 9.3.1
```

**原因**：系统默认的 `gcc-arm-none-eabi` 是 9.3.1 版本，ArduPilot 4.6.x 要求 GCC 10.2.1+。

**解决**：
1. 下载安装 Arm GNU Toolchain 13.3：
   ```bash
   cd /opt
   sudo wget https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
   sudo tar -xf arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi.tar.xz
   sudo mv arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi gcc-arm-none-eabi-13.3
   ```
2. 使用项目根目录的 `ap_compile.sh` 脚本编译（它会自动设置 `PATH=/opt/gcc-arm-none-eabi-13.3/bin:$PATH`）。
3. 或者临时：`export PATH=/opt/gcc-arm-none-eabi-13.3/bin:$PATH`。

### 8.2 Pixhawk6X 以太网导致 HAL_USE_MAC 重复定义

**现象**：
```
./hwdef.h:120: error: "HAL_USE_MAC" redefined [-Werror]
./hwdef.h:50:  #define HAL_USE_MAC TRUE
```

**原因**：Pixhawk6X 的 hwdef 定义了 ETH1 引脚，hwdef 脚本自动生成 `HAL_USE_MAC TRUE`。SimOnHW 不需要以太网，尝试 `undef HAL_USE_MAC` + `define FALSE` 无效（hwdef 处理器在 include 后追加定义，无法覆盖）。

**解决**：创建不带 ETH 引脚的板子变体：
1. 新建目录 `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X-NoETH/`
2. 从 `Pixhawk6X/hwdef.dat` 复制并删除所有 `ETH` 相关行（约第 88–102 行）
3. SimOnHW 的 hwdef.dat 改为：
   ```
   include ../Pixhawk6X-NoETH/hwdef.dat
   include ../include/SimOnHW.inc
   ```

### 8.3 defaults.parm 没有被嵌入固件

**现象**：烧录后参数没有变化，QGC 看不到 `Q_FRAME_TYPE`、`GPS1_TYPE=100` 等。

**原因**：hwdef 脚本从**第一个被 include 的 hwdef 所在目录**查找 `defaults.parm`（见 `chibios_hwdef.py` 的 `write_processed_defaults_file`）。我们 include 的第一个文件是 `Pixhawk6X-NoETH/hwdef.dat`，而 defaults.parm 只放在 `Pixhawk6X-SimOnHardWare/` 目录。

**验证方法**：
```bash
# 配置后检查是否生成 processed_defaults.parm
ls -la build/Pixhawk6X-SimOnHardWare/processed_defaults.parm
```
如果文件不存在，说明 defaults 没被加载。

**解决**：把 defaults.parm 同时放到 `Pixhawk6X-NoETH/` 目录（或确保 hwdef.dat 第一行 include 的目录里有 defaults.parm）。重新 configure 后日志应显示：
```
Default parameters path from hwdef: .../defaults.parm
```

### 8.4 defaults.parm 应该包含什么

对于 QuadPlane 倾转旋翼 HITL，defaults.parm 必须包含：

```ini
# 仿真核心
AHRS_EKF_TYPE 10
GPS1_TYPE 100
BATT_MONITOR 0
SIM_RATE_HZ 400
SCHED_LOOP_RATE 400
SIM_MAG1_DEVID 97539
SIM_BARO_RND 0

# IMU 校准（让 INS 认为已校准）
INS_ACCOFFS_X 0.001
INS_ACCOFFS_Y 0.001
INS_ACCOFFS_Z 0.001
INS_ACCSCAL_X 1.001
INS_ACCSCAL_Y 1.001
INS_ACCSCAL_Z 1.001
# 同理 INS_ACC2*

# QuadPlane（来自 quadplane-tilthvec.parm）
Q_ENABLE 1
Q_FRAME_CLASS 1
Q_FRAME_TYPE 3
Q_TILT_TYPE 0
Q_TILT_YAW_ANGLE 10
Q_TILT_MASK 15

# 倾转舵机
SERVO12_FUNCTION 41
SERVO13_FUNCTION 41
SERVO12_MIN 1000
SERVO12_MAX 2000
SERVO13_MIN 1000
SERVO13_MAX 2000
```

注意：烧录后 defaults 只在参数表为空时自动加载；若飞控已有旧参数，必须在 QGC 里 **Tools → Reset to defaults** 后重启。

### 8.5 PreArm: Hardware safety switch

**现象**：解锁时一直提示 "Hardware safety switch"。

**原因**：Pixhawk6X 板上有物理安全按钮，初始状态是 DISARMED。

**解决**：
- 方法 A：按一下飞控板上的安全按钮（LED 常亮表示已解除）。
- 方法 B：QGC 参数设置 `BRD_SAFETY_DEFLT 0` 并重启，禁用安全开关。

### 8.6 PreArm: Throttle (RC3) is not neutral

**现象**：解锁时提示油门不在中位/低位。

**原因**：ArduPilot 解锁前要求油门在最低位（`control_in = 0`），HITL 无真实 RC 接收机时通道值异常。

**解决（HITL 推荐）**：设置 `ARMING_CHECK = 0` 关闭所有解锁检查，重启。这样可以直接在 AUTO 模式下解锁启动任务，不需要 RC。

如需保留部分检查，可只关闭 RC 检查位，但直接设 0 最简单。

### 8.7 PreArm: Waiting for RC

**现象**：提示等待 RC 接收机。

**原因**：HITL 通常没有连接 RC 接收机。

**解决**：
- 方法 A（推荐用于 AUTO）：`ARMING_CHECK = 0`，AUTO 任务飞行不需要 RC。
- 方法 B：QGC → 应用设置 → General → 勾选 **Virtual Joystick**，重启 QGC，用虚拟摇杆。
- 方法 C：用 MAVProxy/Mavlink 发 RC_CHANNELS_OVERRIDE。

### 8.8 QGC 自动恢复旧参数

**现象**：Reset to defaults 后参数又变回旧值。

**原因**：QGC 可能开启了"Auto load parameters"功能，自动从本地恢复。

**解决**：QGC → 应用设置 → 关闭 `Auto load parameters`，再 Reset to defaults。

### 8.9 HITL 解锁飞行的推荐参数

为了顺畅跑 HITL，建议额外设置：

| 参数 | 值 | 说明 |
|------|-----|------|
| `ARMING_CHECK` | 0 | 关闭所有解锁检查 |
| `FS_THR_ENABLE` | 0 | 关闭油门失效保护 |
| `FS_GCS_ENABLE` | 0 | 关闭 GCS 失效保护 |
| `FENCE_ENABLE` | 0 | 关闭电子围栏（仿真 home 在堪培拉） |
| `BRD_SAFETY_DEFLT` | 0 | 禁用安全开关 |

这些参数也可以直接写入 defaults.parm 里。

### 8.10 用 AUTO 模式测试 HITL

参数正确后：
1. QGC → Plan（计划）→ 创建简单任务：TAKEOFF 30m → WAYPOINT → LAND → 上传。
2. 飞行模式切到 **AUTO**。
3. QGC 主界面长按 **Arm**，确认启动任务。
4. 观察：
   - QGC 地图上飞机位置移动（仿真位置）
   - SERVO_OUTPUT_RAW 里电机 PWM 和 SERVO12/13 倾转舵机值
   - 倾转过渡时 SERVO12/13 从 1000（垂直）向 2000（水平）变化
5. 飞完下载 .BIN 日志，查看 `ATT`、`GPS`、`SIM`、`QTUN`、`TILT` 消息。

### 8.11 快速验证清单

烧录新固件并 Reset to defaults 后，按顺序检查：

- [ ] `GPS1_TYPE = 100`
- [ ] `AHRS_EKF_TYPE = 10`
- [ ] `SIM_RATE_HZ = 400`
- [ ] `Q_ENABLE = 1`
- [ ] `Q_FRAME_TYPE = 3`
- [ ] `Q_TILT_TYPE = 0`
- [ ] `Q_TILT_MASK = 15`
- [ ] `SERVO12_FUNCTION = 41`
- [ ] `SERVO13_FUNCTION = 41`
- [ ] QGC 显示 GPS 3D fix
- [ ] QGC 姿态仪表有响应
- [ ] 设置 `ARMING_CHECK = 0` 并重启
- [ ] AUTO 模式可以 arm
