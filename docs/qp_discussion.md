# QP控制分配在飞行器作动面求解中的应用

## 1. 问题本质：过驱动系统的控制分配

飞行器通常有**多个执行器**（旋翼、舵面等），但只需要控制**6个自由度**（3个力+3个力矩）。这就是**过驱动系统**：

```
执行器数量 > 控制自由度
例如：8个旋翼 + 6个副翼 + 6个V尾舵面 = 20个执行器
      需要控制：3个力 + 3个力矩 = 6个自由度
```

**核心问题**：给定6维虚拟控制指令（期望的力和力矩），如何分配到20个执行器？

**解不唯一**：有无穷多种分配方式都能满足控制需求，需要**优化**来选择"最好"的那个。

## 2. 数学建模

### 2.1 基本关系

执行器产生的力和力矩与控制输入的关系：

```
F = B * u
```

其中：
- `F` ∈ R^6：虚拟控制向量（3个力 + 3个力矩）
- `B` ∈ R^(6×20)：控制效能矩阵（每个执行器对6个自由度的贡献）
- `u` ∈ R^20：执行器控制输入（旋翼转速、舵面偏角等）

### 2.2 控制分配问题

**已知**：期望的虚拟控制指令 `F_des` ∈ R^6

**求解**：执行器指令 `u` ∈ R^20，使得：

```
B * u ≈ F_des  （满足控制需求）
```

同时优化某些目标（如能耗最小、磨损均衡等）。

## 3. QP问题建模

### 3.1 标准QP形式

二次规划（Quadratic Programming）的标准形式：

```
minimize:    (1/2) * x^T * H * x + f^T * x
subject to:  A * x ≤ b
             Aeq * x = beq
             lb ≤ x ≤ ub
```

### 3.2 控制分配的QP建模

**决策变量**：`x = u` ∈ R^20（执行器指令）

**目标函数**：多目标加权

```
J = J1 + J2 + J3

J1 = ||B*u - F_des||^2_W1      // 跟踪误差最小
J2 = ||u||^2_W2                 // 控制能耗最小
J3 = ||u - u_prev||^2_W3        // 变化率最小（平滑性）
```

展开为标准QP形式：

```
H = 2 * (B^T * W1 * B + W2 + W3)
f = -2 * B^T * W1 * F_des - 2 * W3 * u_prev
```

**约束条件**：

1. **等式约束**（可选）：精确满足某些控制需求
   ```
   Aeq * u = beq
   例如：某些关键自由度必须精确满足
   ```

2. **不等式约束**：执行器物理限制
   ```
   u_min ≤ u ≤ u_max          // 位置限制
   u_dot_min ≤ (u - u_prev)/dt ≤ u_dot_max  // 速率限制
   ```

### 3.3 具体例子

假设4个旋翼控制3个自由度（升力、滚转、俯仰）：

```
控制效能矩阵 B：
        旋翼1  旋翼2  旋翼3  旋翼4
升力    [ 1      1      1      1   ]
滚转    [-1     -1      1      1   ]  × 力臂
俯仰    [-1      1     -1      1   ]  × 力臂

期望控制：F_des = [100 N, 5 Nm, 3 Nm]^T

约束：0 ≤ 旋翼推力 ≤ 50 N

求解：u = [u1, u2, u3, u4]^T
```

## 4. 求解方法

### 4.1 常用QP求解器

| 求解器 | 特点 | 适用场景 |
|--------|------|----------|
| **OSQP** | 开源、高效、支持嵌入式 | 实时控制、嵌入式系统 |
| **qpOASES** | 开源、专为MPC设计 | 模型预测控制 |
| **quadprog** (MATLAB) | MATLAB内置 | 离线仿真、原型设计 |
| **Gurobi/CPLEX** | 商业求解器、极快 | 大规模问题 |

### 4.2 嵌入式实现（OSQP为例）

```c
// 1. 定义问题维度
int n = 20;  // 决策变量数（执行器数）
int m = 40;  // 约束数

// 2. 构建QP矩阵
csc H = {...};  // Hessian矩阵（稀疏格式）
float* f = ...; // 线性项

csc A = {...};  // 约束矩阵
float* l = ...; // 约束下界
float* u = ...; // 约束上界

// 3. 初始化求解器
OSQPWorkspace *work = osqp_setup(&settings);

// 4. 求解
osqp_solve(work);

// 5. 获取解
float* u_opt = work->solution->x;
```

### 4.3 实时性考虑

| 方面 | 要求 | 解决方案 |
|------|------|----------|
| **求解时间** | < 1ms（400Hz控制） | 使用高效求解器、稀疏矩阵 |
| **内存占用** | < 100KB（嵌入式） | 稀疏存储、预分配 |
| **数值稳定性** | 保证收敛 | 条件数控制、正则化 |
| **热启动** | 加速求解 | 用上一次的解作为初始猜测 |

## 5. 工程实现架构

### 5.1 分层控制分配

```
┌─────────────────────────────────────┐
│  高层控制器（PID/INDI）              │
│  输出：期望的 F_des = [Fx,Fy,Fz,Mx,My,Mz] │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  控制分配层（QP求解器）              │
│  输入：F_des, B矩阵, 约束          │
│  输出：u = [旋翼1, 旋翼2, ..., 舵面N] │
└──────────────┬─────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│  执行器驱动层                        │
│  将 u 转换为 PWM/转速指令            │
└─────────────────────────────────────┘
```

### 5.2 控制效能矩阵B的获取

**方法1：解析计算**
```
旋翼：基于动量理论 + 几何关系
舵面：基于气动力导数 + 动压
```

**方法2：在线辨识**
```
执行器激励 → 测量响应 → 最小二乘辨识 → 更新B矩阵
```

**方法3：查表插值**
```
离线计算不同飞行状态的B矩阵
在线根据空速、迎角插值
```

### 5.3 动态权重调整

```python
# 根据飞行模态调整权重
if flight_mode == HOVER:
    W1_rotor = 1.0    # 优先使用旋翼
    W1_surface = 0.1  # 舵面权重低
elif flight_mode == CRUISE:
    W1_rotor = 0.1    # 旋翼权重低
    W1_surface = 1.0  # 优先使用舵面
else:  # TRANSITION
    ratio = transition_progress
    W1_rotor = 1.0 - ratio
    W1_surface = ratio
```

## 6. 与ArduPilot现有方案的对比

### 6.1 ArduPilot的固定混合

ArduPilot 使用固定混合矩阵，在 `AP_MotorsMatrix` 中实现：

```cpp
// 简化的混合逻辑
for (uint8_t i = 0; i < num_motors; i++) {
    thrust[i] = roll_thrust * roll_factor[i]
              + pitch_thrust * pitch_factor[i]
              + yaw_thrust * yaw_factor[i]
              + throttle_thrust;
}
```

**特点**：
- 固定混合矩阵（`roll_factor`, `pitch_factor`等）
- 无优化目标
- 简单限幅处理约束

### 6.2 QP控制分配的优势

| 方面 | ArduPilot固定混合 | QP控制分配 |
|------|------------------|-----------|
| **冗余利用** | 无法处理过驱动 | 显式优化 |
| **约束处理** | 简单限幅 | 严格的数学约束 |
| **多目标** | 无 | 能耗、磨损、优先级 |
| **容错** | 被动降级 | 主动重构 |
| **适应性** | 固定参数 | 实时优化 |

## 7. 代码示例（Python）

```python
import numpy as np
import osqp
import scipy.sparse as sparse

class ControlAllocator:
    def __init__(self, num_actuators, num_dof):
        self.n = num_actuators  # 执行器数量
        self.m = num_dof        # 控制自由度
        
        # 控制效能矩阵（需要实时更新）
        self.B = np.zeros((self.m, self.n))
        
        # 权重矩阵
        self.W_tracking = np.eye(self.m)
        self.W_energy = np.eye(self.n)
        
        # 执行器约束
        self.u_min = np.zeros(self.n)
        self.u_max = np.ones(self.n) * 100  # 例如最大100%
        
        # 初始化QP求解器
        self.setup_qp()
    
    def setup_qp(self):
        """初始化OSQP求解器"""
        # Hessian矩阵（需要动态更新）
        H = sparse.csc_matrix((self.n, self.n))
        
        # 约束矩阵（上下界约束）
        I = sparse.eye(self.n, format='csc')
        A = I
        l = self.u_min
        u = self.u_max
        
        # 设置求解器
        self.qp_solver = osqp.OSQP()
        self.qp_solver.setup(P=H, q=np.zeros(self.n), A=A, l=l, u=u,
                            verbose=False, warm_start=True)
    
    def allocate(self, F_des, u_prev=None):
        """
        执行控制分配
        
        参数：
            F_des: 期望的虚拟控制 [m]
            u_prev: 上一时刻的执行器指令 [n]
        
        返回：
            u_opt: 最优执行器指令 [n]
        """
        # 更新Hessian矩阵
        # H = 2 * (B^T * W1 * B + W2)
        B = self.B
        W1 = self.W_tracking
        W2 = self.W_energy
        
        H = 2 * (B.T @ W1 @ B + W2)
        H = sparse.csc_matrix(H)
        
        # 更新线性项
        # f = -2 * B^T * W1 * F_des
        f = -2 * B.T @ W1 @ F_des
        
        # 更新求解器
        self.qp_solver.update(Px=H.data, q=f)
        
        # 求解
        result = self.qp_solver.solve()
        
        if result.info.status == 'solved':
            return result.x
        else:
            # 求解失败，返回上一时刻的值或零
            return u_prev if u_prev is not None else np.zeros(self.n)
    
    def update_effectiveness_matrix(self, B_new):
        """更新控制效能矩阵（根据飞行状态）"""
        self.B = B_new

# 使用示例
allocator = ControlAllocator(num_actuators=20, num_dof=6)

# 设置控制效能矩阵（需要根据实际构型计算）
B = np.array([
    # Fx  Fy  Fz  Mx  My  Mz
    [1,   0,  0,  0,  1, -1],  # 旋翼1
    [1,   0,  0,  0,  1,  1],  # 旋翼2
    [1,   0,  0,  0, -1, -1],  # 旋翼3
    [1,   0,  0,  0, -1,  1],  # 旋翼4
    # ... 其他执行器
])
allocator.update_effectiveness_matrix(B)

# 期望控制：升力100N，滚转5Nm，俯仰3Nm
F_des = np.array([0, 0, 100, 5, 3, 0])

# 执行分配
u_opt = allocator.allocate(F_des)
print(f"最优执行器指令: {u_opt}")
```

## 8. 关键挑战与解决方案

| 挑战 | 原因 | 解决方案 |
|------|------|----------|
| **实时性** | QP求解耗时 | 使用OSQP、稀疏矩阵、热启动 |
| **数值稳定性** | 条件数差 | 正则化、缩放 |
| **B矩阵准确性** | 气动不确定性 | 在线辨识、鲁棒优化 |
| **约束冲突** | 不可行问题 | 软约束、优先级 |
| **模式切换** | 权重突变 | 平滑过渡、渐进调整 |

## 9. 总结

**QP控制分配的核心思路**：

1. **建模**：将控制分配问题转化为带约束的二次优化问题
2. **目标**：多目标加权（跟踪误差 + 能耗 + 平滑性）
3. **约束**：执行器物理限制（位置、速率）
4. **求解**：使用高效QP求解器（OSQP等）
5. **更新**：根据飞行状态实时更新控制效能矩阵和权重

**优势**：
- 充分利用冗余执行器
- 显式处理约束
- 多目标优化
- 支持容错控制

**适用场景**：
- 过驱动飞行器（多旋翼、分布式推进）
- 需要精确控制分配的场景
- 需要容错能力的系统
- VTOL过渡阶段的复杂分配

## 10. 与INDI控制器的结合

INDI（增量非线性动态逆）控制器输出的是**虚拟控制指令**（期望的角加速度或力/力矩），需要控制分配层将其转换为执行器指令。

**结合架构**：

```
INDI控制器
    ↓ 输出：Δω_desired（期望角加速度增量）
    ↓
控制效能矩阵 B
    ↓
QP求解器
    ↓ 输出：Δu（执行器指令增量）
    ↓
执行器驱动
```

**优势**：
- INDI提供鲁棒的内环控制
- QP处理过驱动和约束
- 两者结合实现全包线自适应控制

**关键接口**：
- INDI输出 → QP输入：虚拟控制指令 `F_des`
- QP输出 → 执行器：最优分配 `u_opt`
- 反馈回路：控制效能矩阵 `B` 根据飞行状态实时更新

## 11. 倾转电机：分配矩阵随角度变化的处理

倾转旋翼的推力方向随倾转角 θ 变化，控制效能矩阵 B 变成 B(θ)。ArduPilot 与 QP 方案处理方式完全不同。

### 11.1 为什么倾转会改变分配矩阵

θ 定义：0° = 垂直向上（悬停），90° = 水平向前（巡航）。绕 y 轴倾转时：

```
θ = 0°:   T → [0, 0, -T]              只产生升力
θ = 45°:  T → [T·sinθ, 0, -T·cosθ]    升力 + 前推
θ = 90°:  T → [T, 0, 0]               只产生前推

力矩：M = r × T(θ)
```

同一电机、同样油门，在不同 θ 下对 Fx/Fz/Mx/My/Mz 的贡献不同 → **B 是 θ 的函数**。

### 11.2 ArduPilot：固定混合 + 倾转后补偿

**架构**（混控之后做几何修正）：

```
姿态/油门 PID
    ↓  roll, pitch, yaw, throttle
output_armed_stabilizing()          ← 固定 roll_factor/pitch_factor/yaw_factor
    ↓  各电机 thrust[i]
thrust_compensation() 回调
    ↓
tilt_compensate()                   ← 按 current_tilt 做几何修正
    ↓
output_to_motors()
```

回调注册（`ArduPlane/tiltrotor.cpp`，`set_thrust_compensation_callback`）：

```cpp
motors->set_thrust_compensation_callback(
    FUNCTOR_BIND_MEMBER(&Tiltrotor::tilt_compensate, void, float *, uint8_t));
```

**`tilt_compensate()` 逻辑**（`tiltrotor.cpp` 中 `Tiltrotor::tilt_compensate`）：

```cpp
if (current_tilt <= 0) return;  // 未倾转，不补偿

if (quadplane.in_vtol_mode()) {
    // VTOL 模式：非倾转电机乘 cosθ
    tilt_factor = cosf(radians(current_tilt*90));
    tilt_compensate_angle(thrust, num_motors, tilt_factor, 1);
} else {
    // 固定翼模式：倾转电机乘 1/cosθ，补偿垂直分量损失
    inv_tilt_factor = 1.0 / cosf(radians(current_tilt*90));
    tilt_compensate_angle(thrust, num_motors, 1, inv_tilt_factor);
}
```

**`tilt_compensate_angle()` 三件事**（`tiltrotor.cpp` 中 `Tiltrotor::tilt_compensate_angle`）：

1. **推力缩放（1/cosθ）**  
   倾转电机 `thrust[i] *= 1/cosθ`，保证垂直升力分量不随倾转丢失。θ=45° 时乘约 1.414。

2. **滚转控制衰减**  
   ```cpp
   thrust[i] = current_tilt * avg_tilt_thrust + thrust[i] * (1-current_tilt);
   ```  
   θ→90° 时，差动推力对滚转的贡献逐渐撤掉。

3. **偏航矢量**  
   ```cpp
   sin_tilt = sinf(radians(current_tilt*90));
   diff_thrust = roll_factor[i] * (yaw + yaw_ff) * sin_tilt * yaw_gain;
   thrust[i] += diff_thrust;
   ```  
   θ 越大，左右差动越能产生偏航力矩。

若有电机饱和（>1.0），整体等比缩小。

**本质**：

```
混合矩阵（roll_factor 等）= 常数（按悬停构型算好）
倾转影响 = 混控之后的后处理修正
```

| 优点 | 缺点 |
|------|------|
| 计算量 O(n)、实现简单 | 非最优，只是几何近似 |
| 工程验证充分 | 滚转/偏航切换是启发式线性插值 |
| | 复杂构型（多倾转轴、V尾倾转）难推广 |
| | 饱和用简单缩放，非约束最优 |

### 11.3 QP 方案：B(θ) 实时更新再求解

**控制效能矩阵显式依赖 θ**。第 i 个倾转旋翼：

```
推力方向：d_i(θ) = [sinθ, 0, -cosθ]^T

力：  F_i = T_i · d_i(θ)
力矩：M_i = r_i × F_i + 反扭矩项

B 的第 i 列：B[:, i] = ∂(F,M)/∂u_i |_{θ}
```

θ 变 → 对应列变 → **必须用新 B 重新解 QP**。

**每控制周期流程**：

```
① 读取当前倾转角 θ（或各电机独立 θ_i）
② 根据 θ 重算 B(θ)：
     for each 倾转电机 i:
         B[:, i] = f(θ_i, r_i, Ω_i, ρ, ...)
     for each 固定旋翼/舵面:
         B[:, j] = f(空速, 迎角, ...)   // 舵面还依赖动压
③ 用新 B 组装 QP：
     H = 2(Bᵀ W₁ B + W₂ + W₃)
     f = -2 Bᵀ W₁ F_des - 2 W₃ u_prev
④ 求解 u*
⑤ 输出到执行器（倾转角指令 θ_cmd 可由上层单独生成）
```

**数值直觉**（单倾转电机，简化）：

```
θ=0°:   B_col ≈ [0, 0, -1, ...]ᵀ     // 主要贡献 Fz
θ=45°:  B_col ≈ [0.707, 0, -0.707, ...]ᵀ
θ=90°:  B_col ≈ [1, 0, 0, ...]ᵀ      // 主要贡献 Fx
```

同一条悬停指令 `F_des = [0,0,mg,0,0,0]`：
- θ=0°：QP 给该电机较大 u，专门产升力
- θ=45°：该电机 u 增大，或把升力分给其他未倾转电机
- θ=90°：该电机对 Fz≈0，升力任务自动分给剩余垂直旋翼/机翼

**不需要手写 1/cosθ 或滚转衰减**——B 变了，最优解自然变。

### 11.4 对比总结

| 维度 | ArduPilot | QP |
|------|-----------|-----|
| B/混合矩阵 | **固定**（悬停几何） | **每周期更新 B(θ)** |
| 倾转处理位置 | 混控**之后**补偿 | 混控**之中**（B 已含 θ） |
| 1/cosθ | 显式后乘 | 隐含在 B 的 Fz 行，优化器自动加大 u |
| 滚转/偏航切换 | 启发式线性衰减 | B 的 Mx/Mz 列自然变化，QP 自动重分配 |
| 多目标 | 无 | 可优化能耗、饱和裕度 |
| 容错 | 难 | 故障列置零，重新求解 |
| 计算量 | O(n) | 约 O(n³)，需 OSQP 等高效求解器 |

```
简单 ←————————————————————————→ 最优
  │                              │
ArduPilot 后补偿              QP + B(θ)
· 固定混合矩阵                · 每周期更新 B
· 1/cosθ + 启发式             · 约束内最优
· μs 级                       · ms 级（OSQP）
· 构型扩展难                  · 构型扩展易
```

### 11.5 工程折中

既要 QP 优点、又不想每拍全量重构 B 时：

1. **B 表离线计算**：θ = 0°, 10°, …, 90° 预计算 B(θ)，在线线性插值
2. **热启动**：上一拍 u* 作 OSQP 初值，通常 1–2 次迭代即可
3. **分层**：旋翼组用 QP；气动舵面仍固定混合（舵面 B 随空速变，可低频更新）
4. **θ 慢变**：倾转速率有限（如 30°/s），B 不必 400Hz 更新，50–100Hz 足够

把“倾转导致分配矩阵变化”变成**可调度的参数更新**，而不是每拍从零组装。

### 11.6 相关代码索引（ArduPilot）

引用前请用工具核对当前分支行号：

| 内容 | 文件 | 符号 |
|------|------|------|
| 补偿回调注册 | `ArduPlane/tiltrotor.cpp` | `set_thrust_compensation_callback` |
| 倾转补偿入口 | `ArduPlane/tiltrotor.cpp` | `Tiltrotor::tilt_compensate` |
| 角度相关修正 | `ArduPlane/tiltrotor.cpp` | `Tiltrotor::tilt_compensate_angle` |
| 混控后调用补偿 | `libraries/AP_Motors/AP_MotorsMulticopter.cpp` | `output()` → `thrust_compensation()` |
| 矩阵类回调触发 | `libraries/AP_Motors/AP_MotorsMatrix.cpp` | `_thrust_compensation_callback` |
