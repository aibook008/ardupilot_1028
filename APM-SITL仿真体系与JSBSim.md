# ArduPilot SITL 仿真体系与 JSBSim 集成

> 基于本仓库（Plane 4.6.x 分支）代码整理，所有引用均给出文件位置。
> MATLAB 联合仿真见姊妹文档 `APM-MATLAB联合仿真.md`。

## 1. SITL 体系总览

ArduPilot 自带一套完整的 SITL（Software In The Loop）仿真体系，物理模型全部实现在
`libraries/SITL/`，编译进 SITL 固件（`./waf configure --board sitl`）。按动力学来源分两类：

### 1.1 内置物理模型（开箱即用，无外部依赖）

| 模型名 | 载具 | 实现文件 |
|---|---|---|
| `plane` | 固定翼 | `SIM_Plane.cpp` |
| `glider` | 滑翔机 | `SIM_Glider.cpp` |
| `quadplane` / `firefly` | 垂起复合翼 | `SIM_QuadPlane.cpp` |
| `quad`/`x`/`hexa`/`octa`/`tri`/`y6`/`deca` 等 | 多旋翼各构型 | `SIM_Multicopter.cpp` |
| `heli` / `heli-dual` / `heli-compound` | 直升机 | `SIM_Helicopter.cpp` |
| `rover` / `balancebot` / `sailboat` / `motorboat` | 地面/水面 | `SIM_Rover.cpp` 等 |
| `vectored` / `vectored_6dof` | 水下 | `SIM_Submarine.cpp` |
| `blimp` / `balloon` / `stratoblimp` | 轻于空气载具 | `SIM_Blimp.cpp` 等 |
| `tracker` | 天线追踪器 | `SIM_Tracker.cpp` |
| `calibration` / `novehicle` | 传感器标定/纯传感器测试 | `SIM_Calibration.cpp` / `SIM_NoVehicle.h` |

内置模型同时模拟完整传感器链：IMU 噪声与偏置、GPS（`SIM_GPS_*`，含多种真实协议）、
气压计、磁力计（地磁模型）、空速计、测距仪等——EKF 在 SITL 中是真实工作的。

### 1.2 桥接模型（对接外部物理引擎）

SITL 本身不含动力学，把舵机输出转发给外部引擎、收回传感器状态：

| 模型名 | 外部引擎 | 通信方式 |
|---|---|---|
| `jsbsim` | JSBSim 六自由度飞行动力学 | TCP 控制台 + UDP fgFDM，**由 SITL fork 子进程**（本文重点，见第 3 章） |
| `JSON` | 任意（MATLAB/pybullet/自定义） | UDP 二进制下行 + JSON 文本上行（锁步） |
| `gazebo` | Gazebo | `SIM_Gazebo.cpp` |
| `airsim` | AirSim | `SIM_AirSim.cpp` |
| `webots` / `webots-python` | Webots | `SIM_Webots.cpp` |
| `morse` | MORSE | `SIM_Morse.cpp` |
| `scrimmage` | SCRIMMAGE 多智能体 | `SIM_Scrimmage.cpp` |
| `xplane` | X-Plane | `SIM_XPlane.cpp` |
| `flightaxis` | FlightAxis/RealFlight | `SIM_FlightAxis.cpp` |
| `silentwings` | Silent Wings 滑翔机 | `SIM_SilentWings.cpp` |
| `crrcsim` / `last_letter` | CRRCSim / last_letter | `SIM_CRRCSim.cpp` / `SIM_last_letter.cpp` |

### 1.3 模型注册与匹配

所有模型注册在 `libraries/AP_HAL_SITL/SITL_cmdline.cpp:126` 的 `model_constructors[]`。
`--model` 按**前缀不敏感**匹配（`SITL_cmdline.cpp:567` 用 `strncasecmp`），所以
`jsbsim:elevon` 匹配 `"jsbsim"`、`JSON:192.168.1.50` 匹配 `"JSON"`，冒号后内容传给构造函数。

### 1.4 启动入口

```sh
./Tools/autotest/sim_vehicle.py -v ArduPlane              # 默认内置 plane 模型 + MAVProxy
./Tools/autotest/sim_vehicle.py -v ArduPlane -f jsbsim    # JSBSim 后端
./Tools/autotest/sim_vehicle.py -v ArduPlane --model JSON # MATLAB 等 JSON 后端
```

`sim_vehicle.py` 的 frame→model 解析在 `Tools/autotest/pysim/vehicleinfo.py` 的
`options_for_frame()`：frame 表里有 `"model"` 键就用它，否则 `model = frame`；
`--model` 命令行参数最终覆盖一切（`sim_vehicle.py:772` 传给二进制的 `--model`）。
ArduPlane 的默认 frame 是 `plane`（`vehicleinfo.py:252`）。

`./Tools/autotest/autotest.py run.Plane` 的 Plane 回归测试默认用 `plane-elevrev`
（内置模型，`arduplane.py:77`）；`jsbsim` 是独立 frame，需要机器上装有 JSBSim
才能跑，autotest CI 服务器上就是用它做高保真验证的。

## 2. 内置物理模型的使用与 VTOL

### 2.1 基本用法

```sh
./Tools/autotest/sim_vehicle.py -v ArduPlane                 # 默认内置 plane 模型
./Tools/autotest/sim_vehicle.py -v ArduPlane -f quadplane    # 标准四旋翼复合翼 VTOL
./Tools/autotest/sim_vehicle.py -v ArduPlane --list-frames   # 列出所有可用 frame
```

`-f <frame>` 经 `Tools/autotest/pysim/vehicleinfo.py` 映射成两样东西：

1. `--model <frame>`：决定 SITL 用哪个物理模型类（`SIM_Plane`/`SIM_QuadPlane`/...）；
2. `default_params_filename`：决定加载哪套载具构型参数（`Tools/autotest/default_params/*.parm`）。

旋翼构型后缀（`-hexa`/`-octa`/`-y6`/`-tri`/`-tilt...`）由模型构造函数里的 `strstr`
解析，可自由组合，如 `--model quadplane-hexa`（`SIM_QuadPlane.cpp:33`）。

### 2.2 VTOL（quadplane）frame 一览

`vehicleinfo.py:252` 起的 ArduPlane frame 表中 VTOL 相关：

| frame | 构型 | 参数文件 |
|---|---|---|
| `quadplane` | 标准 X 型四旋翼 + 固定翼 | `quadplane.parm` |
| `quadplane-tri` | 前三旋翼 | `quadplane-tri.parm` |
| `quadplane-tilt` | 倾转旋翼 | `quadplane-tilt.parm` |
| `quadplane-tilttri` | 三倾转旋翼 | `quadplane-tilttri.parm` |
| `quadplane-tilttrivec` | 三倾转+矢量 | `quadplane-tilttrivec.parm` |
| `quadplane-tilthvec` | H 型倾转矢量 | `quadplane-tilthvec.parm` |
| `quadplane-cl84` | CL-84 风格（可收放旋翼，`SERVO_RETRACT`，7 秒收放） | `quadplane-cl84.parm` |
| `quadplane-copter_tailsitter` | 多旋翼式尾坐 | `quadplane-copter_tailsitter.parm` |
| `plane-tailsitter` | 常规尾坐 | `plane-tailsitter.parm` |
| `firefly` | 升降副翼 VTOL（elevon + 推/拉电机） | `firefly.parm` |
| `quadplane-ice` / `quadplane-can` | 油动 / DroneCAN 外设变体 | 组合 parm |

**关键：SITL frame 必须与飞控参数里的构型匹配。** 各 parm 文件已预置
`Q_ENABLE`、`Q_FRAME_CLASS/Q_FRAME_TYPE`、`SERVOx_FUNCTION` 等，选对 `-f` 就
无需手动改参数。例如 `quadplane.parm` 含 `Q_ENABLE 1`、`Q_ASSIST_SPEED 18`。

### 2.3 内置 VTOL 模型的工作原理

`SIM_QuadPlane` 继承 `SIM_Plane`（`libraries/SITL/SIM_QuadPlane.cpp`），
一个模型里同时算两套力，每步相加后统一积分：

1. **固定翼气动力**（继承自 Plane）：机翼升力、舵面力矩、阻力——前 4 路舵机走这里；
2. **旋翼推力**（`frame->calculate_forces()`）：VTOL 电机从 **servo 4 之后**开始
   （`motor_offset = 4`，`SIM_QuadPlane.cpp:101`）；倾转变体中前向电机
   `thrust_scale = 0`（靠倾转提供推力）；
3. copter tailsitter 把旋翼坐标系旋转 90°（`ROTATION_PITCH_270`）；
4. 质量按旋翼架质量 ×1.5 估算（计入机身）；支持 lock-step。

### 2.4 典型验证流程

```
# MAVProxy 中：
mode QHOVER; arm throttle; rc 3 1800   → 垂直升起
mode FBWA（或 QLOITER→FBWA）           → 观察自动前飞转换
mode QLAND                             → 返航垂直降落
```

完整回归测试：`./Tools/autotest/autotest.py fly.QuadPlane`（用例定义在
`Tools/autotest/quadplane.py`，含 QAUTOTUNE、Weathervane、QAssist、Tailsitter、
CopterTailsitter、ICEngine、Mission 等）。

### 2.5 内置气动模型的保真度与局限

`SIM_Plane.cpp` 用解析气动模型：

- 升力系数 `liftCoeff()`（`SIM_Plane.cpp:112`）：sigmoid 混合**线性段**
  （小攻角，`c_lift_0 + c_lift_a*α`）与**平板段**（失速后，`2·sin²α·cosα`），
  以 `alpha_stall` 为过渡点——能表现失速现象，但是**静态、无滞后**的近似；
- 阻力 `dragCoeff()`：寄生阻力 + 诱导阻力（升致阻力，`SIM_Plane.cpp:134`）；
- 旋翼：只算推力/反扭矩与简单桨盘模型，无桨盘动力学、无桨-翼相互作用。

因此内置模型适合验证**控制逻辑、模式转换流程、参数整定**；不适合研究
转换段复杂气动耦合（桨-翼干扰、动态失速、尾流交互）——这类课题需要
JSBSim（第 3 章）或 JSON 接口接 MATLAB（见姊妹文档）。

#### 桨-翼干扰（propeller-wing interaction）

固定翼飞机推进桨的滑流（prop slipstream）直接冲刷机翼，产生两类效应：

1. **动压增益**：滑流使机翼局部来流速度增大（滑流速度 ≈ 螺旋桨诱导速度叠加），
   升力与操纵面效率随之改变——低速时尤其明显，机翼"感觉"到的空速比真实
   飞行速度更高；
2. **桨滑流扭转（slipstream swirl）**：螺旋桨旋转使滑流带角动量，打在机翼/尾翼上
   产生非对称侧力和偏航/滚转力矩（典型现象：大功率时飞机偏航）。

对 VTOL 更复杂：旋翼既在垂起模式提供升力，又在转换段与机翼同时工作——
旋翼尾流吹过机翼改变其有效攻角和局部升力分布，机翼又反过来扰动旋翼入流
（影响旋翼拉力与振动）。转换段"谁主导升力"的切换瞬间，气动力是强耦合、
非线性、非定常的。

内置 `SIM_QuadPlane` 把旋翼推力和机翼升力**简单相加**，完全忽略这类耦合；
JSBSim 可用 `aerosurfaces`/外部气动数据表建模部分效应；CFD 级精度只能靠
外部模型（MATLAB/Simulink 接 JSON）。

#### 动态失速（dynamic stall）

**静态失速**：攻角 α 超过临界值（失速攻角）后上翼面气流分离，升力骤降、阻力剧增。
**动态失速**：攻角随时间快速变化时（旋翼桨叶旋转、VTOL 转换段机翼攻角快速扫过、
尾坐俯仰机动），分离的发生和恢复都**滞后于**静态失速点：

1. 攻角动态增大时，前缘会形成**动态失速涡（dynamic stall vortex）**，涡在
   上翼面停留期间升力反而短暂超过静态最大值（升力过冲）；
2. 涡脱落后升力突然崩溃，并伴随大幅俯仰力矩变化（低头力矩脉冲）；
3. 攻角回落后气流**再附着有延迟**，回线（升力-攻角迟滞环）不可逆。

对 VTOL 的影响：转换段机翼攻角在短时间内扫过大范围，若按静态气动表设计
控制律，会低估/高估某些瞬间的可用升力与俯仰力矩，导致高度掉/姿态超调；
旋翼桨叶在高入流条件下也会进入动态失速，产生周期性载荷与振动。

内置模型的 `liftCoeff()` 是**静态** sigmoid，无迟滞、无涡升力过冲，
无法复现动态失速；要研究需高保真模型（JSBSim 动态气动表、
MATLAB/Simulink 的 Leishman-Beddoes 类动态失速模型等，走 JSON 接口）。

## 3. JSBSim 集成机制（重点）

JSBSim 是高保真六自由度飞行动力学库，ArduPlane 的 CI 回归测试用它。
SITL 端连接器：`libraries/SITL/SIM_JSBSim.h/.cpp`（类 `SITL::JSBSim`）。

### 3.1 进程模型：SITL 自动拉起 JSBSim

与 Gazebo/AirSim 等"自己先起外部进程"不同，JSBSim 模式下 **SITL 用 `fork()+execlp()`
把 JSBSim 作为子进程拉起**（`SIM_JSBSim.cpp:180` 的 `start_JSBSim()`）：

1. 绑定 FDM 接收端口（UDP 5504）；
2. fork 子进程，`chdir(autotest_dir)` 后执行：
   ```
   JSBSim --suspend --simulation-rate=<rate_hz> --nice=1e-8 \
          --logdirectivefile=jsbsim_fgout_<instance>.xml \
          --script=jsbsim_start_<instance>.xml
   ```
   `--suspend` 使 JSBSim 启动后停在控制台等待指令，不会自己跑。
3. 父进程从管道读子进程 stdout，等待 `"JSBSim Execution beginning"` 字符串确认启动成功；
4. 连接 JSBSim 控制台 TCP 端口，发送 `info`、`resume`、`iterate 1`、
   `set atmosphere/turb-type 4`（MIL-SPEC Dryden 湍流模型）解锁仿真。

前提：`JSBSim` 可执行文件在 `PATH` 里（安装见 2.5），否则 SITL 直接 panic 退出。

### 3.2 通信架构（每个仿真步）

```
 arduplane (SITL::JSBSim)                          JSBSim 子进程
 ─────────────────────────                         ──────────────────────
 ① send_servos():
    TCP <control_port>  ───文本指令──────────────►  控制台 <input port>
    set fcs/aileron-cmd-norm ...                      解析并执行
    set atmosphere/wind-mag-fps ...
    iterate 1                    （JSBSim 积分一个 dt）
 ② recv_fdm():
    UDP <fdm_port>      ◄──二进制 FGNetFDM────────  FLIGHTGEAR 输出插件
    （阻塞等待，超时 100ms 就重发舵机指令）
```

- **控制通道**（TCP，默认端口 `5505 + instance*10`，`SIM_JSBSim.cpp:65`）：
  纯文本的 JSBSim 控制台协议。每步发送：
  ```
  set fcs/aileron-cmd-norm <ch1>
  set fcs/elevator-cmd-norm <ch2>
  set fcs/rudder-cmd-norm <ch3>
  set fcs/throttle-cmd-norm <ch4>
  set atmosphere/psiw-rad <风向>
  set atmosphere/wind-mag-fps <风速>
  set atmosphere/turbulence/milspec/windspeed_at_20ft_AGL-fps <风速/3>
  set atmosphere/turbulence/milspec/severity <湍流强度>
  iterate 1
  ```
  舵机经 `filtered_servo_angle/range()` 归一化到 -1..1（`SIM_JSBSim.cpp:342`）。
  帧型 `elevon`/`vtail`（`jsbsim:elevon`、`jsbsim:vtail`）会在发送前做混控
  （`SIM_JSBSim.cpp:346-360`），注意混控后的舵面映射需与 JSBSim 飞机模型的
  FCS 定义一致，且仓库自带的飞机模型只有 Rascal（常规布局）。
- **状态通道**（UDP，默认端口 `5504 + instance*10`）：
  JSBSim 用 FlightGear 输出插件发 **FGNetFDM** 二进制包（约 292 字节，
  结构体定义在 `SIM_JSBSim.h:94`），包含经纬度、高度、姿态/角速率、速度、
  机体加速度（飞行员位置）、发动机 RPM、起落架、舵面位置等。
  JSBSim 发小端，`FGNetFDM::ByteSwap()` 做大端转换（含 3 个 double 的特殊处理）。
  用 `cur_time` 去重：`while (fdm.cur_time == time_now_us)` 循环跳过旧包。

### 3.3 锁步时序

`update()`（`SIM_JSBSim.cpp:466`）：

```
send_servos(input)      // TCP 指令，末尾 iterate 1 让 JSBSim 走一步
recv_fdm(input)         // 阻塞等 UDP 状态包（100ms 超时则重发舵机）
adjust_frame_time(rate_hz)
sync_frame_time()       // 按 --speedup 控制墙钟节奏
```

- 步长：JSBSim 脚本里 `dt = 1/rate_hz`（`create_templates()` 生成，
  `SIM_JSBSim.cpp:108`），`rate_hz` 由 SITL 参数 `SIM_RATE_HZ` 控制
  （`SIM_Aircraft.h:250` 默认 1200 Hz）。注意这是 SITL 自己的参数，
  **不是** `plane-jsbsim.parm` 里的内容。
- 是锁步的：SITL 不收到新状态包不会前进；MATLAB 连接器一样支持断点调试，
  JSBSim 也会跟着停住。

### 3.4 模板文件（SITL 运行时自动生成）

`create_templates()`（`SIM_JSBSim.cpp:75`）在 `autotest_dir`（即
`Tools/autotest/`，由 `--autotest-dir` 传入）生成三个文件：

| 生成文件 | 作用 | 参考模板 |
|---|---|---|
| `jsbsim_start_<i>.xml` | runscript：载机、`<input port>`、启动发动机、配平、`dt=1/rate_hz` | `Tools/autotest/jsb_sim/rascal_test_template.xml` |
| `jsbsim_fgout_<i>.xml` | FLIGHTGEAR/UDP 输出配置，端口 `5504+10i` | `Tools/autotest/jsb_sim/fgout_template.xml` |
| `aircraft/<model>/reset.xml` | 初始条件（由 SITL `--home` 换算经纬度/航向，高度 1.3m，俯仰 13°） | `Tools/autotest/aircraft/Rascal/reset_template.xml` |

runscript 里两个关键 event：
- `start engine`（sim-time ≤ 0.01s）：`propulsion/engine[0]/set-running = 1`；
- `Trim`：`simulation/do_simple_trim = 2`，地面配平。

飞机模型目录：`Tools/autotest/aircraft/`，仓库自带 **Rascal**（大型固定翼，
气动定义在 `aircraft/Rascal/Rascal.xml`）。模型名默认 `Rascal`，可用
`jsbsim:<模型名>` 覆盖（`SIM_JSBSim.cpp:61`），但换模型需在
`aircraft/<模型名>/` 放好 JSBSim 标准格式的模型文件。

### 3.5 安装与运行

安装（仓库自带脚本 `Tools/scripts/build-jsbsim.sh`，克隆编译到 `~/jsbsim`）：

```sh
./Tools/scripts/build-jsbsim.sh
export PATH=$PATH:~/jsbsim/build/src   # 建议写进 ~/.bashrc
```

运行：

```sh
./Tools/autotest/sim_vehicle.py -v ArduPlane -f jsbsim
# 等价手动形式：
./build/sitl/bin/arduplane --model jsbsim --autotest-dir=$(pwd)/Tools/autotest \
    --speedup 1 --home -35.363261,149.165230,584,353
```

frame 配置在 `Tools/autotest/pysim/vehicleinfo.py:361`：默认参数文件为
`default_params/plane-jsbsim.parm`（EKF2、舵机反向、PID 初值等，与 Rascal
匹配），`"external": True` 标记（autotest 遍历时跳过，`arduplane.py:4278`）。

### 3.6 JSBSim 与内置模型/JSON 的选择

| 需求 | 选择 |
|---|---|
| 快速验证飞控逻辑、无外部依赖 | 内置 `plane`/`quad` 等 |
| 高保真气动（失速后特性、真实气动导数）、回归测试 | `jsbsim`（Rascal） |
| 自定义动力学（MATLAB/Simulink/自研模型） | `JSON`（见姊妹文档） |
| 带 3D 可视化的机器人/传感器环境 | `gazebo` / `airsim` / `webots` |

JSBSim 的代价：需单独安装；控制通道是文本 TCP，每步多一次进程间往返；
飞机模型制作门槛高（XML 气动定义）。

## 4. 参考文件索引

| 文件 | 作用 |
|---|---|
| `libraries/SITL/SIM_JSBSim.h/.cpp` | JSBSim 连接器（进程管理、控制台协议、fgFDM 解析） |
| `libraries/SITL/SIM_Aircraft.h/.cpp` | 模型基类（rate_hz、伺服滤波、时间同步） |
| `libraries/SITL/SIM_Plane.cpp` | 内置固定翼气动模型（`liftCoeff`/`dragCoeff`，静态失速） |
| `libraries/SITL/SIM_QuadPlane.cpp` | 内置 VTOL 模型（机翼+旋翼合力，构型后缀解析） |
| `libraries/AP_HAL_SITL/SITL_cmdline.cpp` | `--model` 解析与模型注册表 |
| `Tools/autotest/pysim/vehicleinfo.py` | frame→model/参数文件映射（ArduPlane frames 在 :252，`jsbsim` 在 :361） |
| `Tools/autotest/default_params/quadplane*.parm` | 各 VTOL 构型默认参数 |
| `Tools/autotest/quadplane.py` | QuadPlane 回归测试用例（fly.QuadPlane） |
| `Tools/autotest/jsb_sim/*.xml` | JSBSim 脚本模板（参考用，运行时实际生成同名变体） |
| `Tools/autotest/aircraft/Rascal/` | 自带 Rascal 飞机模型 |
| `Tools/autotest/default_params/plane-jsbsim.parm` | JSBSim frame 默认参数 |
| `Tools/scripts/build-jsbsim.sh` | JSBSim 一键安装脚本 |
