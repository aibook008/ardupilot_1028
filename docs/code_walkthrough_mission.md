# 任务/导航模块代码通读

> 适用分支：`qhz/plane_4.6.3`
> 关联文档：[技术协议摘要](technical_agreement_summary.md) §3.6 航线与遥控

---

## 1. 模块总览

| 模块 | 代码位置 | 职责 |
|------|---------|------|
| AP_Mission | `libraries/AP_Mission/` | 航点管理、任务执行 |
| 飞行模式 | `ArduPlane/mode_*.cpp` | 各模式控制逻辑 |
| AP_Navigation | `libraries/AP_Navigation/` | 纯追踪算法 |
| AP_Rally | `libraries/AP_Rally/` | 集结点管理 |
| AP_Landing | `ArduPlane/AP_Landing.cpp` | 着陆逻辑 |

---

## 2. AP_Mission 航点管理

### 2.1 类定义（`libraries/AP_Mission/AP_Mission.h`）

核心成员：
- `_cmd_list`：航点命令列表
- `_nav_cmd`：当前导航命令
- `_cmd_total`：总航点数
- `_cmd_index`：当前航点索引

### 2.2 航点结构（`AP_Mission.h`）

```cpp
struct Mission_Command {
    uint16_t id;              // 航点ID
    MAV_CMD command;          // MAVLink 命令类型
    Content p1;               // 参数1-7
    Location location;        // 经纬度/高度
};
```

支持的命令类型：
- `MAV_CMD_NAV_WAYPOINT`：普通航点
- `MAV_CMD_NAV_LOITER_UNLIM`：无限盘旋
- `MAV_CMD_NAV_LOITER_TIME`：定时盘旋
- `MAV_CMD_NAV_LAND`：着陆
- `MAV_CMD_NAV_TAKEOFF`：起飞
- `MAV_CMD_NAV_RETURN_TO_LAUNCH`：返航
- `MAV_CMD_NAV_VTOL_TAKEOFF`：VTOL起飞
- `MAV_CMD_NAV_VTOL_LAND`：VTOL着陆
- `MAV_CMD_DO_*`：动作命令（改变参数、触发相机等）

### 2.3 关键方法

**读取航点**（`AP_Mission.cpp`）：
```cpp
bool read_cmd_from_storage(Mission_Command &cmd, uint16_t index);
```
从 EEPROM 读取指定索引的航点。

**写入航点**：
```cpp
bool write_cmd_to_storage(const Mission_Command &cmd, uint16_t index);
```
写入 EEPROM，持久化保存。

**任务执行**：
```cpp
bool start_or_resume();        // 启动或恢复任务
bool advance_current_nav_cmd(); // 推进到下一航点
bool verify_nav_cmd(const Mission_Command &cmd); // 验证航点完成
```

---

## 3. AUTO 模式（`ArduPlane/mode_auto.cpp`）

### 3.1 进入 AUTO（`mode_auto.cpp:enter()`）

```cpp
bool ModeAuto::enter() {
    if (!plane.mission.start_or_resume()) {
        return false;
    }
    plane.auto_state.wp_proportion = 0;
    return true;
}
```

启动任务执行，重置航点比例。

### 3.2 AUTO 运行（`mode_auto.cpp:update()`）

每主循环调用：
1. 获取当前导航命令
2. 根据命令类型分发处理
3. 更新航点比例（0.0~1.0）
4. 调用导航控制器

### 3.3 命令处理（`mode_auto.cpp:handle_nav_cmd()`）

```cpp
switch (cmd.id) {
    case MAV_CMD_NAV_WAYPOINT:
        plane.nav_controller->update_waypoint(prev_WP, next_WP);
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
        plane.nav_controller->update_loiter(cmd.location, cmd.p1);
        break;
    case MAV_CMD_NAV_LAND:
        plane.landing.do_land(cmd);
        break;
    // ... 其他命令
}
```

---

## 4. 导航控制器（`libraries/AP_Navigation/`）

### 4.1 纯追踪算法（L1 Navigation）

`AP_Navigation` 实现 L1 纯追踪算法：

```cpp
void update_waypoint(const Location &prev_WP, const Location &next_WP);
```

计算：
- 航向误差（bearing error）
- 横向误差（crosstrack error）
- L1 距离（前瞻距离）
- 滚转指令（roll command）

### 4.2 参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `NAV_L1_PERIOD` | L1 追踪周期 | 25m |
| `NAV_L1_DAMPING` | L1 阻尼 | 0.75 |
| `NAVL1_XTRACK` | 横向追踪增益 | 1.0 |

---

## 5. 其他飞行模式

### 5.1 GUIDED 模式（`ArduPlane/mode_guided.cpp`）

GCS 实时指令控制：
- 位置指令：飞往指定经纬度/高度
- 速度指令：保持指定速度/航向
- 航向指令：保持指定航向

### 5.2 LOITER 模式（`ArduPlane/mode_loiter.cpp`）

以当前位置为中心盘旋：
- 盘旋半径：`LOITER_RADIUS`
- 盘旋方向：`LOITER_DIR`（1=顺时针，-1=逆时针）

### 5.3 RTL 模式（`ArduPlane/mode_rtl.cpp`）

返航到起飞点：
1. 爬升到 `RTL_ALTITUDE`
2. 飞往起飞点
3. 在起飞点盘旋或着陆

### 5.4 CRUISE 模式（`ArduPlane/mode_cruise.cpp`）

定高定速巡航：
- 空速：`TRIM_ARSPD_CM`
- 高度：当前高度
- 航向：当前航向

---

## 6. AP_Rally 集结点（`libraries/AP_Rally/`）

备降点管理，RTL 时可飞往最近的集结点而非起飞点。

```cpp
bool get_nearest_rally_point(Location &ret);
```

---

## 7. AP_Landing 着陆逻辑（`ArduPlane/AP_Landing.cpp`）

### 7.1 着陆阶段

1. **Approach**：进近阶段，对准跑道
2. **Glide slope**：下滑道，控制高度/速度
3. **Flare**：拉平，减小下降率
4. **Touchdown**：触地

### 7.2 关键参数

| 参数 | 说明 |
|------|------|
| `LAND_SLOPE_PITCH` | 下滑道俯仰角 |
| `LAND_FLARE_SEC` | 拉平时间 |
| `LAND_FLARE_ALT` | 拉平高度 |
| `LAND_PITCH_DEG` | 着陆俯仰角 |

---

## 8. MAVLink 任务协议（`libraries/GCS_MAVLink/`）

### 8.1 任务上传（GCS → 飞控）

```
GCS: MISSION_COUNT (n items)
飞控: MISSION_REQUEST (index 0)
GCS: MISSION_ITEM (index 0)
飞控: MISSION_REQUEST (index 1)
...
飞控: MISSION_ACK
```

处理器：`GCS_MAVLINK.cpp:handle_mission_count()`

### 8.2 任务下载（飞控 → GCS）

```
GCS: MISSION_REQUEST_LIST
飞控: MISSION_COUNT (n items)
GCS: MISSION_REQUEST (index 0)
飞控: MISSION_ITEM (index 0)
...
```

处理器：`GCS_MAVLINK.cpp:handle_mission_request_list()`

---

## 9. QuadPlane VTOL 任务命令

### 9.1 VTOL 起飞（`MAV_CMD_NAV_VTOL_TAKEOFF`）

1. 多旋翼垂直到 `Q_ALT`
2. 过渡到固定翼
3. 继续爬升到航点高度

### 9.2 VTOL 着陆（`MAV_CMD_NAV_VTOL_LAND`）

1. 固定翼飞往着陆点
2. 过渡到多旋翼
3. 垂直下降到地面

### 9.3 过渡命令

- `MAV_CMD_DO_VTOL_TRANSITION`：强制过渡（多旋翼↔固定翼）
- `MAV_CMD_NAV_LOITER_TO_ALT`：盘旋爬升/下降到指定高度

---

## 10. 任务持久化

航点存储在 EEPROM（通过 `AP_Param`）：
- 最大航点数：`MIS_TOTAL`（默认 200）
- 存储格式：每个航点约 40 字节
- 掉电保持：EEPROM 非易失

---

## 11. 任务执行状态

`AP_Mission` 维护状态机：

```cpp
enum mission_state {
    MISSION_STOPPED,      // 任务停止
    MISSION_RUNNING,      // 任务执行中
    MISSION_COMPLETE,     // 任务完成
};
```

状态转换：
- `start_or_resume()`：STOPPED → RUNNING
- `advance_current_nav_cmd()`：推进到下一航点
- `verify_nav_cmd()`：验证航点完成条件
- 所有航点完成：RUNNING → COMPLETE

---

## 12. 协议要求覆盖

| 协议要求 | APM 实现 | 状态 |
|---------|---------|------|
| 航点设置 | `AP_Mission` + MAVLink | ✅ |
| 航线规划 | GCS 软件（Mission Planner/QGC） | ✅ |
| 航点查询修改 | MAVLink 任务协议 | ✅ |
| 自动起飞 | `MAV_CMD_NAV_TAKEOFF` / `VTOL_TAKEOFF` | ✅ |
| 自动着陆 | `MAV_CMD_NAV_LAND` / `VTOL_LAND` | ✅ |
| 返航 | RTL 模式 + Rally 点 | ✅ |
