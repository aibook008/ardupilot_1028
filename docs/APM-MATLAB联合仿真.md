# ArduPilot 与 MATLAB/Simulink 联合仿真指南

> 基于本仓库（Plane 4.6.x 分支）代码整理。所有代码引用均给出文件位置，可直接对照阅读。

## 1. 联合仿真的核心思路

**用 MATLAB/Simulink 替换 SITL 内置的物理模型，但保留完整的 ArduPilot 飞控固件。**

普通 SITL 的结构是：

```
┌─────────────────────── arduplane / arducopter 进程 ───────────────────────┐
│  真实飞控固件（姿态估计 EKF、控制器、模式逻辑、MAVLink……）                 │
│              ▲ 传感器                       ▼ 舵机输出(PWM)                │
│  ┌───────────────────────────────────────────────────────────┐            │
│  │ libraries/SITL 内置物理模型（SIM_Plane.cpp 等）            │            │
│  └───────────────────────────────────────────────────────────┘            │
└────────────────────────────────────────────────────────────────────────────┘
```

联合仿真就是把中间那块"内置物理模型"换成 MATLAB：

```
┌──── arduplane 进程（model=JSON）────┐          ┌──────── MATLAB / Simulink ────────┐
│ 真实飞控固件                         │  UDP     │  SITL_connector.m（收发协议层）    │
│   ▲ JSON 传感器报文     PWM ▼       │◄────────►│         ▲            ▼             │
│ libraries/SITL/SIM_JSON.cpp         │  9002    │  你自己的动力学模型（state 结构体） │
└─────────────────────────────────────┘          └────────────────────────────────────┘
        ▲
        │ MAVLink (TCP 5760)
   MAVProxy / QGroundControl
```

**关键点：**
- 飞控侧跑的是**真实固件**，不是 MATLAB 里重写的控制器。EKF、PID、模式切换、故障保护全部是真的——这正是联合仿真的价值：验证"真实控制律 + 自定义动力学模型"的闭环行为。
- MATLAB 侧只负责动力学：接收舵机 PWM，积分一步，返回传感器状态（IMU、位置、速度、姿态）。
- 两者通过 **UDP + JSON 文本协议**通信，SITL 端实现在 `libraries/SITL/SIM_JSON.cpp`，模型名注册为 `"JSON"`（见 `libraries/AP_HAL_SITL/SITL_cmdline.cpp:179`）。

## 2. 通信协议（必须掌握的细节）

协议权威文档：`libraries/SITL/examples/JSON/readme.md`。

### 2.1 SITL → MATLAB：舵机输出（二进制，UDP 发到 9002 端口）

```c
uint16 magic = 18458;   // 固定魔数，用于校验包来源/协议版本
uint16 frame_rate;      // 建议的物理步长频率 (Hz)，由参数 SIM_RATE_HZ 控制
uint32 frame_count;     // 帧序号（回绕/回退可检测 SITL 重启、丢帧、重复帧）
uint16 pwm[16];         // 16 路舵机输出，微秒（典型 1000~2000）
```

- 设置参数 `SERVO_32_ENABLE=1` 后变为 32 通道包，magic = 29569。
- 若 10 秒没收到 MATLAB 的回包，SITL 会**重发当前帧**（frame_count 不递增），因此 MATLAB 端要做重复帧判断。

### 2.2 MATLAB → SITL：传感器状态（JSON 文本，UDP 回包）

一行 JSON，前后各带 `\n`。**必填字段**：

```json
{"timestamp":2500,
 "imu":{"gyro":[0,0,0],"accel_body":[0,0,-9.8]},
 "position":[0,0,0],
 "velocity":[0,0,0],
 "attitude":[0,0,0]}
```

| 字段 | 含义 | 单位/坐标系 |
|---|---|---|
| `timestamp` | 物理仿真时间 | s |
| `imu.gyro` | 陀螺仪 | rad/s，机体系 |
| `imu.accel_body` | 加速度计 | m/s²，机体系 |
| `position` | 相对起飞原点的位移 | m，NED |
| `velocity` | 速度 | m/s，NED |
| `attitude` 或 `quaternion` | 姿态（二者必须有其一） | rad / 四元数 |

可选字段：`rng_1`~`rng_6`（测距仪）、`velocity_wind`（NED 风矢量）、`windvane`（风向风速）、`airspeed`（空速，直接给则不再由速度推算）、`rc`（最多 12 路遥控输入）、`battery`（电压电流）。

**注意**：MATLAB 只给位置，ArduPilot 内部自行把 NED 位移换算成经纬度，并在内部生成 GPS、气压计、磁力计数据（磁场由位置姿态计算，见 `SIM_JSON.cpp:519` 的 `update_mag_field_bf()`）。所以 MATLAB 模型**不需要**实现 GPS/罗盘/气压计。

### 2.3 时间同步（lockstep）

- 协议是**锁步**的：SITL 发一帧 → 等 MATLAB 回一帧 → 固件前进一个步长。SITL 端逻辑见 `SIM_JSON.cpp` 的 `update()`：先 `output_servos()` 再阻塞 `recv_fdm()`。
- 因此 MATLAB 里**打断点也能正常调试**，仿真会停在那里等。
- 时间步长由 ArduPilot 参数 `SIM_RATE_HZ` 决定（须 ≥ 载具主循环频率：Copter/QuadPlane 默认 400Hz，Plane/Rover 默认 50Hz）。MATLAB 收到 `frame_rate` 后用 `delta_t = min(1/frame_rate, max_timestep)` 积分。
- 实测 MATLAB 连接器可达 >1500 fps：400Hz 步长下约 4~5 倍加速；Plane 步长 50Hz 可开到更高加速比。速度开太高 GCS 会跟不上，注意节制。

## 3. 实操步骤

### 3.1 构建 SITL

```sh
./waf configure --board sitl
./waf plane        # 或 copter/rover，取决于你要仿真的载具
```

### 3.2 启动 SITL（JSON 模型）

方式一，通过 `sim_vehicle.py`（会顺带起 MAVProxy，GCS 在 TCP 5760）：

```sh
./Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON
# 指定 MATLAB 所在 IP（跨机器时）：
./Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON:192.168.1.50
```

方式二，直接跑二进制（无 MAVProxy，需自己连 5760）：

```sh
./build/sitl/bin/arduplane --model JSON --speedup 1 --home -35.363261,149.165230,584,353
```

`--model` 的匹配是前缀不敏感的（`SITL_cmdline.cpp:567` 用 `strncasecmp`），所以 `JSON:ip` 能正确匹配到 `"JSON"` 构造函数，冒号后是 MATLAB 主机地址。

启动后 SITL 会打印 `JSON control interface set to ...:9002` 并开始向 9002 端口发舵机包（MATLAB 未连接时每 10 秒重发一次并提示）。

### 3.3 运行 MATLAB 连接器

仓库自带官方示例：`libraries/SITL/examples/JSON/MATLAB/`

```
MATLAB/
├── SITL_connector.m          # 协议层：UDP 收发、帧计数、复位检测
├── AP_Conector.slx           # Simulink 收发模块（输入/输出 block）
├── AP_send.m / AP_receve.m   # Simulink 用的 S-function 级收发
├── tcp_udp_ip_2.0.6/         # pnet UDP 工具箱（比 Instrument Control Toolbox 快 10 倍）
├── Copter/                   # 多旋翼动力学示例（Copter.m 参数 + SIM_multicopter.m）
├── Rover/                    # 小车 Simulink 模型 Rover.slx
└── Heli/                     # 直升机 Simulink 模型 heli.slx
```

MATLAB 侧用法（纯脚本形式）：

```matlab
state = ...;                 % 你的载具状态结构体，至少含 gyro/attitude/accel/velocity/position
init_function    = @init;    % state = init(state)，初始化 & SITL 重启时复位
physics_function = @physics_step;  % state = physics_step(pwm_in, state)，积分一步
SITL_connector(state, init_function, physics_function, max_timestep)
```

`SITL_connector.m` 的主循环逻辑（对照 `libraries/SITL/examples/JSON/MATLAB/SITL_connector.m`）：

1. 监听 UDP 9002，收到包后校验 magic=18458；
2. 对比 `frame_count`：变小 → SITL 重启，重新调用 `init_function` 复位物理模型；相等 → 重复帧，跳过；
3. `delta_t = min(1/frame_rate, max_timestep)`，调用 `physics_function` 积分一步；
4. 把新状态打包成 JSON（`jsonencode`），`sprintf('\n%s\n', ...)` 后 UDP 回包给 SITL。

Simulink 形式：打开 `AP_Conector.slx`，接收模块输出 16 路 PWM 数组，发送模块接收载具状态；还有一个 reset 信号在 SITL 重启时拉高，可用于复位 Simulink 模型的初始条件。注意 Simulink 的步长**不能**由 ArduPilot 的 `SIM_RATE_HZ` 动态调整，需在 Simulink 里设置（支持变步长）。

### 3.4 验证链路

1. SITL 终端首次收到数据时会打印 `JSON received:` 列出成功解析的字段——用它核对你的 MATLAB 发的字段是否齐全（`SIM_JSON.cpp:310`）。
2. MATLAB 侧每 1000 帧打印 fps 与实时率百分比。
3. MAVProxy（或 QGC 连 `tcp:127.0.0.1:5760`）里 `mode GUIDED` / `arm throttle` 起飞，观察姿态、位置是否按你的模型响应。

## 4. 典型应用场景

1. **高保真动力学验证**：把 CFD 数据、风洞辨识出的气动模型、柔性体/吊挂载荷模型放进 MATLAB，固件原样跑闭环。
2. **参数整定/批量扫描**：MATLAB 里批量改动力学参数（质量、惯量、气动导数），循环启动 SITL 跑同一任务，对比日志（SITL 的 `.tlog`/`.bin` 可用 MATLAB 的 flightlog 工具或 MAVLink 解析读取）。
3. **传感器注入实验**：通过可选字段注入自定义测距仪、空速、风场，验证滤波器和故障保护逻辑。
4. **控制算法对比**：MATLAB 里实现被控对象，ArduPilot 做控制器，与 MATLAB 自己设计的控制器做对照。

## 5. 已知限制

- JSON 接口**不支持直接给 GPS/磁力计/气压计原始数据**；它们由 ArduPilot 从 position/attitude 内部生成。若要注入 GPS 欺骗、磁干扰等，需走别的途径（如 SITL 的 fakepos、或改 SITL 参数 `SIM_GPS_*`）。
- `velocity_wind` 提供后，若同时未直接给 `airspeed`，空速由速度减风矢量推算；JSON 模型目前风模型能力有限（见 `SIM_JSON.cpp:362` 附近注释）。
- MATLAB 单解释执行时帧率受限；物理模型写成向量化代码或编译为 MEX/Coder 可显著提升加速比。
- pnet 工具箱是老代码（含预编译 mexa64/mexw64），新系统上若 mex 失效需按 `readme.md` 重新编译。

## 6. 另一种思路：MATLAB 通过 MAVLink 连 SITL

如果你的目的不是替换物理模型，而是**把 MATLAB 当 GCS/上层决策器**（发航点、读遥测、做轨迹规划），则不需要 JSON 接口：

- 正常启动 SITL（内置模型）+ MAVProxy；
- MATLAB 通过 MAVLink 协议连 `tcp:127.0.0.1:5760`（MAVProxy 支持多路转发，可 `output add 127.0.0.1:14550`）。MathWorks File Exchange 有现成的 MAVLink 读写库，或用 `mavgen` 生成的 C/Python 桥接。
- 适合：编队上层规划、视觉处理结果注入（配合 `SIM_VICON` 等）、参数扫描脚本化。

两种思路可以组合：MATLAB 既是物理模型（JSON/UDP）又是上层规划器（MAVLink/TCP）。

## 7. 参考文件索引

| 文件 | 作用 |
|---|---|
| `libraries/SITL/SIM_JSON.h/.cpp` | SITL 端 JSON 协议实现（端口、包格式、字段解析） |
| `libraries/SITL/examples/JSON/readme.md` | 协议权威说明 |
| `libraries/SITL/examples/JSON/MATLAB/readme.md` | MATLAB/Simulink 连接器说明 |
| `libraries/SITL/examples/JSON/MATLAB/SITL_connector.m` | MATLAB 协议层参考实现 |
| `libraries/SITL/examples/JSON/MATLAB/AP_Conector.slx` | Simulink 收发模块 |
| `libraries/AP_HAL_SITL/SITL_cmdline.cpp:179` | `JSON` 模型注册、`--model` 解析 |
| `Tools/autotest/sim_vehicle.py` | 启动脚本（`--model`、`--sim-address` 等选项） |
