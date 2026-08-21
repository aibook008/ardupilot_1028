# EKF/传感器模块代码通读

> 适用分支：`qhz/plane_4.6.3`
> 关联文档：[技术协议摘要](technical_agreement_summary.md) §2 硬件需求 | [余度调研](redundancy_feasibility.md)

---

## 1. 模块总览

| 模块 | 代码位置 | 职责 |
|------|---------|------|
| AP_NavEKF3 | `libraries/AP_NavEKF3/` | EKF3 导航滤波器 |
| AP_AHRS | `libraries/AP_AHRS/` | 姿态航向参考系统 |
| AP_InertialSensor | `libraries/AP_InertialSensor/` | IMU 传感器管理 |
| AP_GPS | `libraries/AP_GPS/` | GPS 传感器管理 |
| AP_Baro | `libraries/AP_Baro/` | 气压计管理 |
| AP_Compass | `libraries/AP_Compass/` | 磁力计管理 |

---

## 2. AP_NavEKF3 导航滤波器

### 2.1 EKF3 架构（`AP_NavEKF3.h`）

EKF3 是 ArduPilot 的主力导航滤波器，支持多核并行运行。

**核心类**：
```cpp
class NavEKF3 {
    NavEKF3_core *core;        // 核心实例数组
    uint8_t _core_count;       // 核心数量（1-3）
    uint8_t _primary;          // 当前主核心
};
```

**多核机制**（`AP_NavEKF3.h:45-50`）：
- 每个核心独立运行 EKF
- 使用不同传感器组合
- 自动选择最优核心

### 2.2 EKF 核心（`AP_NavEKF3_core.h`）

每个核心包含：
- 状态向量（17 维）：位置(3) + 速度(3) + 姿态(4) + IMU 偏差(6) + 风速(2)
- 协方差矩阵（17×17）
- 传感器融合逻辑

**状态估计流程**：
```
IMU 预测 → GPS 校正 → 气压计校正 → 磁力计校正 → 输出
```

### 2.3 关键方法

**初始化**（`AP_NavEKF3.cpp:InitEKF()`）：
```cpp
bool NavEKF3::InitEKF(void) {
    for (uint8_t i=0; i<_core_count; i++) {
        core[i]->InitialiseFilter();
    }
    return true;
}
```

**状态更新**（`AP_NavEKF3.cpp:UpdateFilter()`）：
```cpp
void NavEKF3::UpdateFilter(void) {
    for (uint8_t i=0; i<_core_count; i++) {
        core[i]->UpdateFilter();
    }
    SelectCore();  // 选择最优核心
}
```

**获取姿态**（`AP_NavEKF3.cpp:getRotationBodyToNED()`）：
```cpp
bool NavEKF3::getRotationBodyToNED(Matrix3f &ret) const {
    return core[_primary]->getRotationBodyToNED(ret);
}
```

### 2.4 传感器融合

**IMU 融合**（`AP_NavEKF3_core.cpp:FuseVelPosNED()`）：
- 高频预测（200-400Hz）
- 积分加速度/角速度
- 更新状态向量

**GPS 融合**（`AP_NavEKF3_core.cpp:FuseGPS()`）：
- 低频校正（5-10Hz）
- 位置/速度观测
- 卡尔曼增益计算

**气压计融合**（`AP_NavEKF3_core.cpp:FuseBaro()`）：
- 高度校正
- 与 GPS 高度互补

**磁力计融合**（`AP_NavEKF3_core.cpp:FuseMag()`）：
- 航向校正
- 地磁场模型

---

## 3. AP_AHRS 姿态航向参考系统

### 3.1 类层次（`AP_AHRS.h`）

```cpp
class AP_AHRS {
    AP_NavEKF3 _ekf3;              // EKF3 实例
    AP_InertialSensor &_ins;       // IMU 引用
    AP_GPS &_gps;                  // GPS 引用
    AP_Baro &_baro;                // 气压计引用
};
```

**单例访问**（`AP_AHRS.h:50`）：
```cpp
static AP_AHRS *get_singleton() { return _singleton; }
```

### 3.2 姿态获取

**欧拉角**（`AP_AHRS.cpp:get_euler()`）：
```cpp
void AP_AHRS::get_euler(float &roll, float &pitch, float &yaw) const {
    _ekf3.getEulerAngles(roll, pitch, yaw);
}
```

**四元数**（`AP_AHRS.cpp:get_quaternion()`）：
```cpp
bool AP_AHRS::get_quaternion(Quaternion &quat) const {
    return _ekf3.getQuaternion(quat);
}
```

### 3.3 位置/速度获取

**位置**（`AP_AHRS.cpp:get_position()`）：
```cpp
bool AP_AHRS::get_position(Location &loc) const {
    return _ekf3.getLLH(loc.lat, loc.lng, loc.alt);
}
```

**速度**（`AP_AHRS.cpp:get_velocity()`）：
```cpp
bool AP_AHRS::get_velocity(Vector3f &vel) const {
    return _ekf3.getVelNED(vel);
}
```

### 3.4 健康检查

**EKF 健康**（`AP_AHRS.cpp:ekf_status()`）：
```cpp
bool AP_AHRS::ekf_status(void) const {
    return _ekf3.healthy();
}
```

**传感器健康**：
```cpp
bool AP_AHRS::gyro_health(uint8_t instance) const;
bool AP_AHRS::accel_health(uint8_t instance) const;
bool AP_AHRS::compass_health(uint8_t instance) const;
```

---

## 4. AP_InertialSensor IMU 管理

### 4.1 多 IMU 支持（`AP_InertialSensor.h`）

```cpp
class AP_InertialSensor {
    uint8_t _gyro_count;       // 陀螺仪数量
    uint8_t _accel_count;      // 加速度计数量
    uint8_t _primary_gyro;     // 主陀螺仪
    uint8_t _primary_accel;    // 主加速度计
};
```

**典型配置**：
- CubeOrange：3× IMU（ICM-20689, BMI055, ICM-42688）
- Pixhawk6X：3× IMU + 2× IMU（共 5 个）

### 4.2 采样与滤波

**采样率**（`AP_InertialSensor.cpp:sample_rate()`）：
- 后端驱动：800-1000Hz
- 前端输出：200-400Hz（可配置）

**数字低通滤波**（`AP_InertialSensor_Backend.cpp:apply_filters()`）：
```cpp
void AP_InertialSensor_Backend::apply_filters() {
    // 陀螺仪低通滤波
    _gyro_filter.apply(gyro);
    // 加速度计低通滤波
    _accel_filter.apply(accel);
}
```

### 4.3 冗余管理

**健康检查**（`AP_InertialSensor.cpp:healthy()`）：
```cpp
bool AP_InertialSensor::healthy(uint8_t instance) const {
    return _backend[instance]->healthy();
}
```

**自动切换**（`AP_InertialSensor.cpp:select_primary()`）：
- 持续监控所有 IMU
- 主 IMU 故障时自动切换
- 平滑过渡，无扰动

---

## 5. AP_GPS GPS 管理

### 5.1 多 GPS 支持（`AP_GPS.h`）

```cpp
class AP_GPS {
    uint8_t _num_instances;    // GPS 数量
    uint8_t _primary;          // 主 GPS
    GPS_State state[GPS_MAX_INSTANCES];
};
```

**典型配置**：
- 单 GPS：普通应用
- 双 GPS：冗余 + RTK
- 三 GPS：高可靠性

### 5.2 RTK 支持

**RTK 状态**（`AP_GPS.h:GPS_Status`）：
```cpp
enum GPS_Status {
    NO_GPS = 0,
    NO_FIX = 1,
    GPS_OK_FIX_2D = 2,
    GPS_OK_FIX_3D = 3,
    GPS_OK_FIX_3D_DGPS = 4,
    GPS_OK_FIX_3D_RTK_FLOAT = 5,
    GPS_OK_FIX_3D_RTK_FIXED = 6,
};
```

**RTK 校正**（`AP_GPS.cpp:handle_rtcm_data()`）：
```cpp
void AP_GPS::handle_rtcm_data(const uint8_t *data, uint16_t len) {
    // 发送 RTCM 数据到 GPS
    _backend[_primary]->inject_data(data, len);
}
```

### 5.3 融合策略

**主 GPS 选择**（`AP_GPS.cpp:select_primary()`）：
- 优先选择 RTK Fixed
- 其次 RTK Float
- 其次 3D Fix
- 最后 2D Fix

---

## 6. AP_Baro 气压计管理

### 6.1 多气压计（`AP_Baro.h`）

```cpp
class AP_Baro {
    uint8_t _num_sensors;      // 气压计数量
    uint8_t _primary;          // 主气压计
};
```

### 6.2 高度计算

**气压高度**（`AP_Baro.cpp:get_altitude()`）：
```cpp
float AP_Baro::get_altitude() const {
    return _alt_filter.apply(_altitude);
}
```

**温度补偿**（`AP_Baro.cpp:temperature_calibration()`）：
- 开机自动校准
- 消除温度漂移

---

## 7. AP_Compass 磁力计管理

### 7.1 多磁力计（`AP_Compass.h`）

```cpp
class Compass {
    uint8_t _compass_count;    // 磁力计数量
    uint8_t _primary;          // 主磁力计
};
```

### 7.2 校准

**自动校准**（`AP_Compass.cpp:auto_calibration()`）：
- 飞行中自动校准
- 消除硬铁/软铁干扰

**手动校准**（`AP_Compass.cpp:calibrate()`）：
- 地面手动校准
- 旋转飞机 360°

---

## 8. 传感器融合架构

### 8.1 数据流

```
IMU (400Hz) ──→ EKF3 预测 ──→ 状态估计
  ↓
GPS (10Hz) ───→ EKF3 校正
  ↓
气压计 (50Hz) → EKF3 校正
  ↓
磁力计 (10Hz) → EKF3 校正
  ↓
AHRS 输出 (400Hz)
```

### 8.2 冗余策略

**传感器级冗余**：
- 多个同类传感器
- 自动健康检查
- 故障自动切换

**估计器级冗余**：
- EKF3 多核并行
- 不同传感器组合
- 最优核心选择

**融合级冗余**：
- 多源信息融合
- 卡尔曼滤波器
- 协方差监控

---

## 9. 关键参数

### 9.1 EKF 参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `EK3_ENABLE` | 启用 EKF3 | 1 |
| `EK3_CHECK_SCALE` | 健康检查缩放 | 1.0 |
| `EK3_GPS_TYPE` | GPS 融合类型 | 0 |
| `EK3_MAG_MASK` | 磁力计使用掩码 | 0 |

### 9.2 传感器参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `INS_GYRO_FILTER` | 陀螺仪滤波 | 20Hz |
| `INS_ACCEL_FILTER` | 加速度计滤波 | 20Hz |
| `GPS_TYPE` | GPS 类型 | 1 |
| `COMPASS_USE` | 使用磁力计 | 1 |

---

## 10. 故障检测与处理

### 10.1 EKF 故障

**检测条件**（`AP_NavEKF3.cpp:healthy()`）：
- 协方差发散
- 残差过大
- 传感器不一致

**处理**（`ArduPlane/ekf_check.cpp`）：
- 触发 EKF failsafe
- 降级到保守模式
- 通知飞手

### 10.2 传感器故障

**IMU 故障**：
- 检测：健康标志、数据冻结
- 处理：切换到备用 IMU

**GPS 故障**：
- 检测：Fix 丢失、精度下降
- 处理：切换到备用 GPS、降级到航位推算

**气压计故障**：
- 检测：数据异常、与 GPS 不一致
- 处理：切换到备用气压计、使用 GPS 高度

---

## 11. 性能指标

| 指标 | 要求 | APM 实现 |
|------|------|---------|
| 姿态精度 | <1° | EKF3 典型 0.1-0.3° |
| 位置精度 | <1m | RTK Fixed <0.1m |
| 速度精度 | <0.5m/s | GPS 典型 0.1m/s |
| 更新率 | ≥100Hz | EKF3 输出 400Hz |

---

## 12. 余度系统扩展

### 12.1 当前状态

- **传感器级**：完整（多 IMU/GPS/Baro/Compass）
- **估计器级**：完整（EKF3 多核）
- **板级**：无（需外部实现）

### 12.2 板级余度方案（`redundancy_feasibility.md`）

**屏障同步**：
- 10Hz 对齐周期
- EKF 状态交换
- 多数表决

**故障切换**：
- 心跳监控
- 自动接管
- 无缝过渡

---

## 13. 二次开发接口

### 13.1 获取姿态

```cpp
AP_AHRS *ahrs = AP_AHRS::get_singleton();
float roll, pitch, yaw;
ahrs->get_euler(roll, pitch, yaw);
```

### 13.2 获取位置

```cpp
Location loc;
if (ahrs->get_position(loc)) {
    float lat = loc.lat * 1e-7f;
    float lng = loc.lng * 1e-7f;
    float alt = loc.alt * 1e-2f;
}
```

### 13.3 获取速度

```cpp
Vector3f vel;
if (ahrs->get_velocity(vel)) {
    float vx = vel.x;  // 北速度 (m/s)
    float vy = vel.y;  // 东速度 (m/s)
    float vz = vel.z;  // 下速度 (m/s)
}
```

### 13.4 健康检查

```cpp
if (!ahrs->ekf_status()) {
    // EKF 故障处理
}
```
