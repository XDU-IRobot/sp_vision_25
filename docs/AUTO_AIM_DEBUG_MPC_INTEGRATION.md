# auto_aim_debug_mpc程序接入CBoard分析报告
# Analysis Report: Integrating auto_aim_debug_mpc with CBoard

**日期**: 2025-12-09
**程序**: `src/auto_aim_debug_mpc.cpp`
**问题**: 程序使用Gimbal直连云台，无法通过CBoard（CAN/串口）通信

---

## 📋 当前状况分析

### 1. 程序架构

**文件**: `src/auto_aim_debug_mpc.cpp`

**当前通信方式**:
```cpp
io::Gimbal gimbal(config_path);  // Line 40 - 直连云台

// 使用方式：
auto gs = gimbal.state();                    // Line 58 - 获取状态
auto q = gimbal.q(t);                       // Line 112 - 获取四元数
gimbal.send(control, fire, yaw, yaw_vel,   // Line 61-63 - 发送MPC指令
            yaw_acc, pitch, pitch_vel, pitch_acc);
```

**Gimbal接口特点**:
- ✅ 支持完整的MPC指令（位置、速度、加速度）
- ✅ 直接读取云台状态（yaw, pitch, vel）
- ❌ 绕过CBoard，无法使用CAN/串口通信
- ❌ 无法与电控板通信

### 2. 标准程序架构

**文件**: `src/sentry.cpp`

**标准通信方式**:
```cpp
io::CBoard cboard(config_path);  // Line 46 - 通过CBoard

// 使用方式：
auto q = cboard.imu_at(timestamp - 1ms);    // Line 67 - 获取IMU
auto bs = cboard.bullet_speed;              // Line 93 - 获取弹速
cboard.send(command);                       // Line 98 - 发送简单指令
```

**CBoard接口特点**:
- ✅ 支持CAN和串口通信
- ✅ 与电控板通信
- ✅ 获取IMU四元数
- ❌ Command结构只支持位置（yaw, pitch）
- ❌ 不支持速度和加速度

### 3. 关键差异对比

| 特性 | Gimbal (直连) | CBoard (电控) |
|------|--------------|--------------|
| **通信方式** | 直连云台串口 | CAN/串口→电控→云台 |
| **协议** | SCM/SP_CRC16 | 新CAN/旧CAN/串口SCM |
| **发送数据** | yaw, pitch, vel, acc | yaw, pitch (仅位置) |
| **状态读取** | state() → 完整状态 | 仅IMU四元数和弹速 |
| **适用场景** | 直连云台调试 | 实际比赛（通过电控） |

---

## ⚠️ 核心问题

### 问题1：Command结构不支持速度和加速度

**当前定义** (`io/command.hpp:6-13`):
```cpp
struct Command {
  bool control;
  bool shoot;
  double yaw;      // 只有位置
  double pitch;    // 只有位置
  double horizon_distance = 0;  // 无人机专用
};
```

**MPC需要**:
```cpp
planner.plan() 输出:
  - yaw, yaw_vel, yaw_acc       // 位置、速度、加速度
  - pitch, pitch_vel, pitch_acc
  - control, fire
```

**问题**：无法通过现有Command传递速度和加速度！

### 问题2：CBoard没有公开MPC发送接口

**当前实现**:
- `CBoard::send(Command command)` - 只能发送位置
- `CBoard::send_scm(...)` - 私有函数，支持速度但不支持加速度

**SCM帧格式** (`io/cboard.cpp:647-659`):
```cpp
struct AimbotFrame_SCM_TX_t {
  uint8_t SOF;
  uint8_t ID;
  uint8_t Aimbotstate;
  uint8_t AimbotTarget;
  float Pitch;              // ✅ 位置
  float Yaw;                // ✅ 位置
  float TargetPitchSpeed;   // ✅ 速度
  float TargetYawSpeed;     // ✅ 速度
  float SystemTimer;
  uint8_t _EOF;
};
```

**发现**：SCM协议**已经支持速度**，但没有加速度字段！

### 问题3：CBoard无法读取云台完整状态

**Gimbal提供**:
```cpp
struct GimbalState {
  float yaw, yaw_vel;
  float pitch, pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
  // ...
};
```

**CBoard提供**:
```cpp
cboard.bullet_speed   // ✅ 弹速
cboard.mode           // ✅ 模式
cboard.shoot_mode     // ✅ 射击模式
cboard.imu_at(t)      // ✅ IMU四元数
// ❌ 没有云台角速度！
```

**问题**：MPC需要云台当前角速度用于闭环控制，CBoard无法提供！

---

## 🎯 解决方案

### 方案1：扩展Command结构 ⭐⭐⭐ (推荐)

**优点**:
- 统一接口，所有程序都可以用
- 向后兼容（vel/acc默认为0）
- 符合系统架构设计

**缺点**:
- 需要修改多个文件
- SCM协议不支持加速度（需要扩展或忽略）

**实现步骤**:

#### 步骤1：扩展Command结构

```cpp
// io/command.hpp
struct Command {
  bool control;
  bool shoot;
  double yaw;
  double pitch;

  // MPC扩展字段（可选，默认为0）
  double yaw_vel = 0.0;
  double yaw_acc = 0.0;
  double pitch_vel = 0.0;
  double pitch_acc = 0.0;

  double horizon_distance = 0;
};
```

#### 步骤2：修改CBoard::send()

```cpp
void CBoard::send(Command command) {
  if (use_serial_ && serial_protocol_scm_) {
    // SCM协议：支持速度，忽略加速度
    send_scm(command.control, command.shoot,
             command.yaw, command.yaw_vel, 0.0f,  // 忽略yaw_acc
             command.pitch, command.pitch_vel, 0.0f);  // 忽略pitch_acc
  } else {
    // 其他协议：仅支持位置
    // 现有代码不变
  }
}
```

#### 步骤3：修改auto_aim_debug_mpc

```cpp
// 替换 io::Gimbal 为 io::CBoard
io::CBoard cboard(config_path);

// 获取IMU
auto q = cboard.imu_at(t);

// 发送MPC指令
io::Command command;
command.control = plan.control;
command.shoot = plan.fire;
command.yaw = plan.yaw;
command.pitch = plan.pitch;
command.yaw_vel = plan.yaw_vel;
command.yaw_acc = plan.yaw_acc;
command.pitch_vel = plan.pitch_vel;
command.pitch_acc = plan.pitch_acc;
cboard.send(command);
```

**问题**：无法获取云台角速度用于Planner！

---

### 方案2：扩展SCM协议添加加速度 ⭐⭐

**修改SCM帧格式**:
```cpp
struct AimbotFrame_SCM_TX_Extended_t {
  uint8_t SOF;
  uint8_t ID;
  uint8_t Aimbotstate;
  uint8_t AimbotTarget;
  float Pitch;              // 位置
  float Yaw;                // 位置
  float TargetPitchSpeed;   // 速度
  float TargetYawSpeed;     // 速度
  float TargetPitchAcc;     // ⭐ 加速度（新增）
  float TargetYawAcc;       // ⭐ 加速度（新增）
  float SystemTimer;
  uint8_t _EOF;
};
// 帧长度：25 → 33字节
```

**优点**:
- 完整支持MPC
- 协议清晰

**缺点**:
- **需要修改电控固件**
- 帧长度增加8字节
- 与现有SCM不兼容

---

### 方案3：简化方案 - 使用Planner内部状态 ⭐

**思路**：Planner本身维护云台状态估计，不需要从外部读取

**修改auto_aim_debug_mpc**:
```cpp
io::CBoard cboard(config_path);

// Planner不需要真实的云台状态，只需要target即可
auto plan = planner.plan(target, cboard.bullet_speed);

// 发送位置指令（忽略速度加速度）
io::Command command;
command.control = plan.control;
command.shoot = plan.fire;
command.yaw = plan.yaw;
command.pitch = plan.pitch;
// 注意：vel和acc被忽略，由电控的PID控制器处理
cboard.send(command);
```

**优点**:
- 不修改Command和CBoard
- 最小改动

**缺点**:
- **MPC失去闭环反馈**
- 性能下降（无法利用MPC优势）
- 仅适合调试

---

### 方案4：混合方案 ⭐⭐⭐⭐ (最佳)

**思路**：
1. 修改Command支持速度
2. 使用SCM协议发送速度（已支持）
3. 忽略加速度（由电控PID处理）
4. 从IMU四元数微分估计角速度

**实现**:

#### 步骤1：扩展Command（仅速度）
```cpp
struct Command {
  bool control;
  bool shoot;
  double yaw;
  double pitch;

  // 仅添加速度（SCM支持）
  double yaw_vel = 0.0;
  double pitch_vel = 0.0;

  double horizon_distance = 0;
};
```

#### 步骤2：修改CBoard添加角速度估计
```cpp
// cboard.hpp
class CBoard {
public:
  // 新增：估计云台角速度（从IMU微分）
  Eigen::Vector2d estimate_gimbal_velocity(
    std::chrono::steady_clock::time_point t);

private:
  Eigen::Quaterniond last_q_;
  std::chrono::steady_clock::time_point last_t_;
};

// cboard.cpp
Eigen::Vector2d CBoard::estimate_gimbal_velocity(
  std::chrono::steady_clock::time_point t)
{
  auto q_curr = imu_at(t);

  if (last_t_.time_since_epoch().count() == 0) {
    // 首次调用，无法计算
    last_q_ = q_curr;
    last_t_ = t;
    return Eigen::Vector2d::Zero();
  }

  // 计算角度变化
  auto R_curr = q_curr.toRotationMatrix();
  auto R_last = last_q_.toRotationMatrix();

  auto euler_curr = R_curr.eulerAngles(2, 1, 0);  // ZYX: yaw, pitch, roll
  auto euler_last = R_last.eulerAngles(2, 1, 0);

  double dt = std::chrono::duration<double>(t - last_t_).count();

  // 角速度 = 角度差 / 时间差
  double yaw_vel = (euler_curr[0] - euler_last[0]) / dt;
  double pitch_vel = (euler_curr[1] - euler_last[1]) / dt;

  last_q_ = q_curr;
  last_t_ = t;

  return Eigen::Vector2d(yaw_vel, pitch_vel);
}
```

#### 步骤3：修改auto_aim_debug_mpc
```cpp
io::CBoard cboard(config_path);

while (!quit) {
  auto target = target_queue.front();

  // 估计云台角速度
  auto vel = cboard.estimate_gimbal_velocity(std::chrono::steady_clock::now());

  // 构造GimbalState（仅用于Planner）
  auto_aim::GimbalState gs;
  gs.yaw_vel = vel[0];
  gs.pitch_vel = vel[1];
  gs.bullet_speed = cboard.bullet_speed;

  auto plan = planner.plan(target, gs.bullet_speed);

  // 发送位置+速度（忽略加速度）
  io::Command command;
  command.control = plan.control;
  command.shoot = plan.fire;
  command.yaw = plan.yaw;
  command.pitch = plan.pitch;
  command.yaw_vel = plan.yaw_vel;
  command.pitch_vel = plan.pitch_vel;

  cboard.send(command);
}
```

**优点**:
- ✅ 支持MPC速度控制
- ✅ 使用现有SCM协议
- ✅ 不修改电控固件
- ✅ 提供云台状态估计
- ✅ 向后兼容

**缺点**:
- 角速度估计有噪声（需要滤波）
- 没有加速度（由电控处理）

---

## 📊 方案对比

| 方案 | 修改量 | 兼容性 | MPC效果 | 推荐度 |
|------|--------|--------|---------|--------|
| 方案1 | 中等 | 好 | 中等 | ⭐⭐⭐ |
| 方案2 | 大 | 差（需改固件） | 最好 | ⭐⭐ |
| 方案3 | 最小 | 最好 | 差 | ⭐ |
| **方案4** | **中等** | **最好** | **好** | **⭐⭐⭐⭐** |

---

## 🎯 推荐实施方案

### 采用方案4：混合方案

**理由**:
1. 不需要修改电控固件
2. 利用现有SCM协议的速度支持
3. 通过IMU微分估计角速度
4. 保持向后兼容

**实施步骤**:
1. ✅ 扩展Command添加vel字段
2. ✅ 添加CBoard::estimate_gimbal_velocity()
3. ✅ 修改CBoard::send()支持速度
4. ✅ 修改auto_aim_debug_mpc使用CBoard

---

## 📝 实施清单

- [ ] 修改 `io/command.hpp` - 添加yaw_vel, pitch_vel
- [ ] 修改 `io/cboard.hpp` - 添加estimate_gimbal_velocity()
- [ ] 修改 `io/cboard.cpp` - 实现速度估计和发送
- [ ] 修改 `src/auto_aim_debug_mpc.cpp` - 替换Gimbal为CBoard
- [ ] 测试编译
- [ ] 实测验证

---

**是否开始实施？**
