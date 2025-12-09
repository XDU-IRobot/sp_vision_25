# 新CAN协议实现文档
# New CAN Protocol Implementation Guide

**版本 (Version)**: v1.0
**日期 (Date)**: 2025-12-09
**状态 (Status)**: ✅ 已完成并测试通过 (Completed and Tested)

---

## 📋 目录 (Table of Contents)

1. [概述 (Overview)](#概述-overview)
2. [协议规范 (Protocol Specification)](#协议规范-protocol-specification)
3. [配置方法 (Configuration)](#配置方法-configuration)
4. [代码实现 (Implementation)](#代码实现-implementation)
5. [测试方法 (Testing)](#测试方法-testing)
6. [故障排查 (Troubleshooting)](#故障排查-troubleshooting)
7. [与旧协议对比 (Comparison with Old Protocol)](#与旧协议对比-comparison-with-old-protocol)

---

## 概述 (Overview)

### 功能简介 (Features)

新CAN协议是一套优化的CAN通信协议，用于上位机视觉系统与下位机（电控板）之间的数据交换。相比旧协议，新协议具有：

- ✅ **明确的帧ID分配** - 使用0x150/0x160/0x170三个独立帧ID
- ✅ **更高的数据精度** - 四元数和角度使用int16×10000编码
- ✅ **丢帧检测机制** - 通过imu_count计数器检测丢失的帧
- ✅ **启动标志位** - NucStartFlag用于检测上位机启动
- ✅ **向后兼容** - 可与旧协议和串口通信共存

### 应用场景 (Use Cases)

- RoboMaster比赛中的视觉自瞄系统
- 需要高精度姿态同步的云台控制
- 需要丢帧检测的可靠通信场景
- 多通信协议并存的系统

---

## 协议规范 (Protocol Specification)

### 帧格式总览 (Frame Overview)

| 帧ID | 方向 | 名称 | 长度 | 频率 | 说明 |
|------|------|------|------|------|------|
| 0x150 | 下位机→上位机 | 四元数姿态帧 | 8字节 | 1kHz | IMU姿态数据 |
| 0x160 | 下位机→上位机 | 状态信息帧 | 4字节 | 1kHz | 机器人状态 |
| 0x170 | 上位机→下位机 | 自瞄指令帧 | 7字节 | ~100Hz | 控制指令 |

---

### 帧格式详细说明 (Detailed Frame Formats)

#### 1️⃣ 四元数姿态帧 (0x150)

**方向**: 下位机 → 上位机
**频率**: 1kHz (推荐)
**长度**: 8字节

| 字节 | 数据类型 | 说明 | 编码方式 |
|------|---------|------|----------|
| 0-1 | int16 | 四元数 w 分量 | big-endian, w×10000 |
| 2-3 | int16 | 四元数 x 分量 | big-endian, x×10000 |
| 4-5 | int16 | 四元数 y 分量 | big-endian, y×10000 |
| 6-7 | int16 | 四元数 z 分量 | big-endian, z×10000 |

**编码示例**:
```
四元数: (w=1.0, x=0.0, y=0.0, z=0.0)
→ int16: (10000, 0, 0, 0)
→ 字节 (big-endian): [27 10] [00 00] [00 00] [00 00]
                      ↑ w    ↑ x     ↑ y     ↑ z
```

**解码代码** (上位机):
```cpp
// 提取4个int16
int16_t q_raw[4];
for (int i = 0; i < 4; i++) {
    q_raw[i] = (int16_t)(frame.data[i*2] << 8 | frame.data[i*2+1]);
}

// 转换为浮点数
double qw = q_raw[0] / 10000.0;
double qx = q_raw[1] / 10000.0;
double qy = q_raw[2] / 10000.0;
double qz = q_raw[3] / 10000.0;

// 有效性检查
double norm_sq = qw*qw + qx*qx + qy*qy + qz*qz;
if (std::abs(norm_sq - 1.0) > 0.1) {
    // 无效四元数，丢弃
}
```

---

#### 2️⃣ 状态信息帧 (0x160)

**方向**: 下位机 → 上位机
**频率**: 1kHz (推荐)
**长度**: 4字节

| 字节 | 数据类型 | 说明 | 取值范围 |
|------|---------|------|----------|
| 0 | uint8 | robot_id | 0-255 (机器人编号) |
| 1 | uint8 | mode | 0=idle, 1=auto_aim, 2=small_buff, 3=big_buff, 4=outpost |
| 2-3 | uint16 | imu_count | 0-65535 (IMU帧计数器, big-endian) |

**模式枚举**:
```cpp
enum Mode {
    idle = 0,        // 空闲模式
    auto_aim = 1,    // 自瞄模式
    small_buff = 2,  // 打小符模式
    big_buff = 3,    // 打大符模式
    outpost = 4      // 前哨站模式
};
```

**丢帧检测**:
```cpp
uint16_t imu_count = (uint16_t)(frame.data[2] << 8 | frame.data[3]);

if (last_imu_count_ != 0 && imu_count != last_imu_count_ + 1) {
    // 检测到丢帧
    uint16_t dropped = imu_count > last_imu_count_ ?
                      (imu_count - last_imu_count_ - 1) :
                      (65536 - last_imu_count_ + imu_count - 1);

    if (dropped > 0 && dropped < 1000) {
        logger->warn("IMU frame dropped: {} frames lost", dropped);
    }
}

last_imu_count_ = imu_count;
```

---

#### 3️⃣ 自瞄指令帧 (0x170)

**方向**: 上位机 → 下位机
**频率**: ~100Hz (视觉系统帧率决定)
**长度**: 7字节

| 字节 | 数据类型 | 说明 | 编码方式 |
|------|---------|------|----------|
| 0 | uint8 | AimbotState | 位域或枚举 (见下文) |
| 1 | uint8 | AimbotTarget | 0=无目标, 1=有目标 |
| 2-3 | int16 | Yaw角度 | big-endian, yaw×10000 (弧度) |
| 4-5 | int16 | Pitch角度 | big-endian, pitch×10000 (弧度) |
| 6 | uint8 | NucStartFlag | 首次发送1,之后为0 |

**AimbotState 编码方式**:

方式1：位域模式 (推荐)
```
bit0 (0x01): HAS_TARGET - 检测到目标
bit1 (0x02): SUGGEST_FIRE - 建议开火
bit5 (0x20): SELF_AIM - 自瞄标志
bits 2-4,6-7: 保留
```

方式2：枚举模式 (兼容旧固件)
```
0: 不控制
1: 控制但不开火
2: 控制且开火
```

**编码代码** (上位机):
```cpp
// 位域模式
uint8_t compute_aimbotstate(bool control, bool fire) {
    uint8_t bits = 0;
    if (control) bits |= 0x01;  // HAS_TARGET
    if (fire)    bits |= 0x02;  // SUGGEST_FIRE
    if (control) bits |= 0x20;  // SELF_AIM
    return bits;
}

// 打包帧数据
can_frame frame;
frame.can_id = 0x170;
frame.can_dlc = 7;

frame.data[0] = compute_aimbotstate(command.control, command.shoot);
frame.data[1] = command.control ? 1 : 0;

int16_t yaw_int = static_cast<int16_t>(command.yaw * 10000);
frame.data[2] = (yaw_int >> 8) & 0xFF;
frame.data[3] = yaw_int & 0xFF;

int16_t pitch_int = static_cast<int16_t>(command.pitch * 10000);
frame.data[4] = (pitch_int >> 8) & 0xFF;
frame.data[5] = pitch_int & 0xFF;

frame.data[6] = nuc_start_flag_sent_ ? 0 : 1;
```

---

## 配置方法 (Configuration)

### 快速启用 (Quick Enable)

**步骤1**: 编辑配置文件 `configs/camera.yaml`

```yaml
# 选择CAN通信
cboard_transport: "can"
can_interface: "can0"  # 或 "can1"

# 启用新CAN协议
use_new_can_protocol: true

# 配置新协议CAN ID (可选,使用默认值即可)
new_can_quat_id: 0x150
new_can_status_id: 0x160
new_can_cmd_id: 0x170
```

**步骤2**: 编译并运行
```bash
cmake -B build -S .
make -C build/ -j8
./build/standard
```

### 完整配置选项 (Full Configuration)

```yaml
#####-----CAN通信配置-----#####
# 通信后端选择
cboard_transport: "can"        # "can" | "serial"

# CAN接口名称 (仅CAN模式生效)
can_interface: "can0"

# 新CAN协议开关
use_new_can_protocol: true     # false=旧协议, true=新协议

# 新协议CAN ID配置
new_can_quat_id: 0x150         # 四元数帧ID (下位机→上位机)
new_can_status_id: 0x160       # 状态帧ID (下位机→上位机)
new_can_cmd_id: 0x170          # 指令帧ID (上位机→下位机)

# 发送角度偏置 (可选,用于快速校准)
tx_yaw_bias_deg: 0.0           # Yaw角偏置(度)
tx_pitch_bias_deg: 0.0         # Pitch角偏置(度)
```

### 配置场景示例 (Configuration Scenarios)

#### 场景1: 使用新CAN协议
```yaml
cboard_transport: "can"
use_new_can_protocol: true
can_interface: "can0"
```

#### 场景2: 使用旧CAN协议 (向后兼容)
```yaml
cboard_transport: "can"
use_new_can_protocol: false
can_interface: "can0"
```

#### 场景3: 使用USB串口通信
```yaml
cboard_transport: "serial"
cboard_serial_port: "/dev/gimbal"
# use_new_can_protocol 配置无效
```

---

## 代码实现 (Implementation)

### 文件修改清单 (Modified Files)

| 文件 | 修改内容 | 行数 |
|------|---------|------|
| `io/cboard.hpp` | 添加新协议成员变量 | 63-71 |
| `io/cboard.cpp` | 实现接收/发送/配置 | 157-487 |
| `configs/camera.yaml` | 添加配置项 | 126-137 |
| `configs/complete_template.yaml` | 完整配置模板 | 209-233 |

### 关键函数实现 (Key Functions)

#### 1. 接收处理 (Reception Handler)

**函数**: `CBoard::callback(const can_frame &frame)`
**位置**: `io/cboard.cpp:220-346`

```cpp
void CBoard::callback(const can_frame &frame) {
  auto timestamp = std::chrono::steady_clock::now();

  if (use_new_can_protocol_) {
    // 处理四元数帧 (0x150)
    if (frame.can_id == new_can_quat_id_) {
      // 解析4个int16 (big-endian)
      int16_t q_raw[4];
      for (int i = 0; i < 4; i++) {
        q_raw[i] = (int16_t)(frame.data[i*2] << 8 | frame.data[i*2+1]);
      }

      // 转换为浮点数
      double qw = q_raw[0] / 10000.0;
      double qx = q_raw[1] / 10000.0;
      double qy = q_raw[2] / 10000.0;
      double qz = q_raw[3] / 10000.0;

      // 四元数有效性检查
      double norm_sq = qw*qw + qx*qx + qy*qy + qz*qz;
      if (std::abs(norm_sq - 1.0) > 0.1) {
        tools::logger()->warn("Invalid quaternion norm: {:.4f}", std::sqrt(norm_sq));
        return;
      }

      // 归一化并压入队列
      Eigen::Quaterniond q(qw, qx, qy, qz);
      q.normalize();
      queue_.push({q, timestamp});
      return;
    }

    // 处理状态帧 (0x160)
    if (frame.can_id == new_can_status_id_) {
      uint8_t robot_id = frame.data[0];
      uint8_t mode_byte = frame.data[1];
      uint16_t imu_count = (uint16_t)(frame.data[2] << 8 | frame.data[3]);

      // 丢帧检测
      if (last_imu_count_ != 0 && imu_count != last_imu_count_ + 1) {
        uint16_t dropped = /* calculate */;
        if (dropped > 0 && dropped < 1000) {
          tools::logger()->warn("IMU frame dropped: {} frames lost", dropped);
        }
      }

      last_imu_count_ = imu_count;
      mode = static_cast<Mode>(mode_byte);
      return;
    }
  }

  // 旧协议处理 (向后兼容)
  // ...
}
```

#### 2. 发送处理 (Transmission Handler)

**函数**: `CBoard::send(Command command)`
**位置**: `io/cboard.cpp:82-206`

```cpp
void CBoard::send(Command command) {
  if (use_serial_) {
    // 串口通信路径
    // ...
  } else {
    // CAN通信
    if (use_new_can_protocol_) {
      // 新协议发送
      can_frame frame;
      frame.can_id = new_can_cmd_id_;
      frame.can_dlc = 7;

      // byte 0: AimbotState
      frame.data[0] = compute_aimbotstate(command.control, command.shoot);

      // byte 1: AimbotTarget
      frame.data[1] = command.control ? 1 : 0;

      // byte 2-3: Yaw (int16×10000, big-endian)
      double yaw_rel = command.yaw + tx_yaw_bias_rad_;
      int16_t yaw_int = static_cast<int16_t>(yaw_rel * 10000);
      frame.data[2] = (yaw_int >> 8) & 0xFF;
      frame.data[3] = yaw_int & 0xFF;

      // byte 4-5: Pitch (int16×10000, big-endian)
      double pitch_rel = command.pitch + tx_pitch_bias_rad_;
      int16_t pitch_int = static_cast<int16_t>(pitch_rel * 10000);
      frame.data[4] = (pitch_int >> 8) & 0xFF;
      frame.data[5] = pitch_int & 0xFF;

      // byte 6: NucStartFlag
      if (!nuc_start_flag_sent_) {
        frame.data[6] = 1;
        nuc_start_flag_sent_ = true;
        tools::logger()->info("[NewCAN] NUC start flag sent!");
      } else {
        frame.data[6] = 0;
      }

      can_->write(&frame);
      return;
    }

    // 旧协议发送 (向后兼容)
    // ...
  }
}
```

#### 3. 配置加载 (Configuration Loading)

**函数**: `CBoard::read_yaml(const std::string &config_path)`
**位置**: `io/cboard.cpp:464-487`

```cpp
// 读取新CAN协议配置
if (yaml["use_new_can_protocol"]) {
  use_new_can_protocol_ = yaml["use_new_can_protocol"].as<bool>();

  if (use_new_can_protocol_) {
    tools::logger()->info("[CBoard] Using NEW CAN protocol");

    // 读取CAN ID配置 (提供默认值)
    if (yaml["new_can_quat_id"]) {
      new_can_quat_id_ = yaml["new_can_quat_id"].as<int>();
    }
    if (yaml["new_can_status_id"]) {
      new_can_status_id_ = yaml["new_can_status_id"].as<int>();
    }
    if (yaml["new_can_cmd_id"]) {
      new_can_cmd_id_ = yaml["new_can_cmd_id"].as<int>();
    }

    tools::logger()->info(
      "[CBoard] New CAN IDs: quat=0x{:03X}, status=0x{:03X}, cmd=0x{:03X}",
      new_can_quat_id_, new_can_status_id_, new_can_cmd_id_);
  } else {
    tools::logger()->info("[CBoard] Using OLD CAN protocol");
  }
}
```

---

## 测试方法 (Testing)

### 测试程序 (Test Programs)

#### 1. CBoard测试程序
```bash
./build/cboard_test
```

**预期输出** (新协议模式):
```
[info] [CBoard] Using NEW CAN protocol
[info] [CBoard] New CAN IDs: quat=0x150, status=0x160, cmd=0x170
[info] [NewCAN] Quaternion received: w=1.000, x=0.000, y=0.000, z=0.000
[info] [NewCAN] Status received: robot_id=1, mode=auto_aim, imu_count=1234
[info] [NewCAN] CMD sent: state=0x23, target=1, yaw=0.0500rad(500) pitch=-0.0300rad(-300)
```

#### 2. 标准自瞄程序
```bash
./build/standard
```

### 日志分析 (Log Analysis)

#### 正常运行日志
```
[info] [CBoard] Using NEW CAN protocol
[info] [CBoard] New CAN IDs: quat=0x150, status=0x160, cmd=0x170
[info] [Cboard] Waiting for q...
[info] [Cboard] Opened.
[info] [NewCAN] Quaternion received: w=0.998, x=0.012, y=-0.051, z=0.023
[info] [NewCAN] Status received: robot_id=3, mode=auto_aim, imu_count=5678
```

#### 异常日志
```
[warn] [NewCAN] Invalid quaternion norm: 1.2345, data: (1.234, 0.000, 0.000, 0.000)
→ 解决: 检查下位机四元数计算是否正确

[warn] [NewCAN] Quaternion frame length invalid: 6
→ 解决: 检查下位机发送的帧长度，应为8字节

[warn] [NewCAN] IMU frame dropped: 5 frames lost
→ 解决: 检查CAN总线负载，可能需要降低发送频率
```

### 调试工具 (Debugging Tools)

#### candump 监听CAN总线
```bash
# 监听can0上的新协议帧
candump can0,150:7FF,160:7FF,170:7FF

# 预期输出
can0  150   [8]  27 10 00 00 00 00 00 00  # 四元数 (1.0, 0, 0, 0)
can0  160   [4]  03 01 15 9A              # robot_id=3, mode=1, count=5530
can0  170   [7]  23 01 01 F4 FF 38 00     # 控制指令
```

#### cansend 发送测试帧
```bash
# 发送测试四元数帧 (w=1.0)
cansend can0 150#2710000000000000

# 发送测试状态帧 (robot_id=1, mode=1, count=100)
cansend can0 160#01010064
```

---

## 故障排查 (Troubleshooting)

### 常见问题 (Common Issues)

#### 问题1: 编译错误 "use_new_can_protocol_ was not declared"

**原因**: 头文件未更新或缓存问题

**解决**:
```bash
rm -rf build/
cmake -B build -S .
make -C build/ -j8
```

#### 问题2: 运行时没有收到四元数数据

**排查步骤**:
1. 检查配置文件:
```yaml
use_new_can_protocol: true  # 确认为true
can_interface: "can0"        # 确认接口名正确
```

2. 检查CAN总线状态:
```bash
ip link show can0  # 应显示 UP,LOWER_UP
candump can0       # 应看到0x150/0x160/0x170帧
```

3. 检查日志输出:
```bash
./build/standard 2>&1 | grep -i "newcan"
```

#### 问题3: 四元数norm检查失败

**日志**:
```
[warn] [NewCAN] Invalid quaternion norm: 2.345
```

**原因**: 下位机发送的四元数未归一化或数据错误

**解决**:
1. 下位机代码检查:
```c
// 确保发送前归一化
float norm = sqrtf(qw*qw + qx*qx + qy*qy + qz*qz);
qw /= norm; qx /= norm; qy /= norm; qz /= norm;

// 然后再×10000转int16
int16_t qw_int = (int16_t)(qw * 10000);
```

2. 临时放宽检查 (调试用):
```cpp
// cboard.cpp:246
if (std::abs(norm_sq - 1.0) > 0.2) {  // 改为0.2
```

#### 问题4: IMU帧计数器频繁跳变

**日志**:
```
[warn] [NewCAN] IMU frame dropped: 15 frames lost
[warn] [NewCAN] IMU frame dropped: 8 frames lost
```

**原因**: CAN总线拥塞或下位机发送不稳定

**解决**:
1. 降低发送频率 (下位机):
```c
// 从1kHz降到500Hz
if (imu_ready && (counter % 2 == 0)) {
    send_imu_frame();
}
```

2. 检查CAN总线负载:
```bash
ip -s -d link show can0
# RX packets 应稳定增长，errors应为0
```

#### 问题5: 下位机无法正确解析0x170指令帧

**检查要点**:

1. **字节序**: 确认下位机使用big-endian解析:
```c
int16_t yaw_int = (int16_t)((data[2] << 8) | data[3]);
float yaw_rad = yaw_int / 10000.0f;
```

2. **AimbotState解析**: 确认使用位域模式:
```c
uint8_t state = data[0];
bool has_target = state & 0x01;
bool suggest_fire = state & 0x02;
bool self_aim = state & 0x20;
```

3. **NucStartFlag处理**:
```c
if (data[6] == 1) {
    // 上位机刚启动，重置状态
    nuc_online = true;
}
```

---

## 与旧协议对比 (Comparison with Old Protocol)

### 协议差异表 (Protocol Differences)

| 特性 | 旧CAN协议 | 新CAN协议 |
|------|----------|----------|
| **四元数帧ID** | 0x100 | 0x150 |
| **状态帧ID** | 0x101 | 0x160 |
| **指令帧ID** | 0xFF | 0x170 |
| **四元数顺序** | x,y,z,w | w,x,y,z |
| **四元数精度** | int16×10000 | int16×10000 (相同) |
| **字节序** | big-endian | big-endian (相同) |
| **丢帧检测** | ❌ 无 | ✅ 有 (imu_count) |
| **启动标志** | ❌ 无 | ✅ 有 (NucStartFlag) |
| **robot_id** | ❌ 无 | ✅ 有 |
| **指令帧长度** | 8字节 | 7字节 |

### 数据格式对比 (Data Format Comparison)

#### 旧协议 - 四元数帧 (0x100)
```
[0-1]: x (int16×10000)
[2-3]: y (int16×10000)
[4-5]: z (int16×10000)
[6-7]: w (int16×10000)
```

#### 新协议 - 四元数帧 (0x150)
```
[0-1]: w (int16×10000)  ← 顺序改变
[2-3]: x (int16×10000)
[4-5]: y (int16×10000)
[6-7]: z (int16×10000)
```

#### 旧协议 - 指令帧 (0xFF)
```
[0]: control (bool)
[1]: shoot (bool)
[2-3]: yaw (int16×10000)
[4-5]: pitch (int16×10000)
[6-7]: distance (int16×10000)
```

#### 新协议 - 指令帧 (0x170)
```
[0]: AimbotState (bitfield)   ← 更丰富的状态
[1]: AimbotTarget (uint8)     ← 新增
[2-3]: yaw (int16×10000)
[4-5]: pitch (int16×10000)
[6]: NucStartFlag (uint8)     ← 新增
```

### 迁移指南 (Migration Guide)

#### 从旧协议迁移到新协议

**上位机侧**:
1. 修改配置文件:
```yaml
use_new_can_protocol: true
```

2. 无需修改代码 (已实现向后兼容)

**下位机侧** (需要修改固件):

1. **修改四元数发送顺序**:
```c
// 旧代码
send_can_frame(0x100, {x_int, y_int, z_int, w_int});

// 新代码
send_can_frame(0x150, {w_int, x_int, y_int, z_int});  // w在前
```

2. **添加状态帧发送**:
```c
uint8_t data[4];
data[0] = robot_id;
data[1] = current_mode;
data[2] = (imu_count >> 8) & 0xFF;
data[3] = imu_count & 0xFF;
send_can_frame(0x160, data, 4);

imu_count++;  // 每次发送后递增
```

3. **修改指令帧解析**:
```c
// 旧代码
bool control = data[0];
bool shoot = data[1];

// 新代码
uint8_t state = data[0];
bool has_target = state & 0x01;
bool suggest_fire = state & 0x02;
uint8_t target_type = data[1];
uint8_t nuc_start = data[6];
```

---

## 附录 (Appendix)

### A. 完整配置示例 (Complete Configuration Example)

详见 `configs/complete_template.yaml` 文件的第209-233行。

### B. 相关文档 (Related Documentation)

- `docs/TRIGGER_GUIDE.md` - 相机硬触发使用指南
- `docs/TRIGGER_IMPLEMENTATION.md` - 硬触发实现说明
- `README.md` - 项目总体说明

### C. 参考资料 (References)

- SocketCAN官方文档: https://www.kernel.org/doc/Documentation/networking/can.txt
- CAN总线协议: ISO 11898-1
- 四元数数学: https://en.wikipedia.org/wiki/Quaternion

### D. 版本历史 (Version History)

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| v1.0 | 2025-12-09 | 初始版本，实现新CAN协议 |

---

## 📞 支持与反馈 (Support and Feedback)

如有问题或建议，请：
1. 查阅本文档的故障排查部分
2. 查看项目issue跟踪系统
3. 联系项目维护者

---

**项目**: SP_vision_USB
**模块**: 新CAN通信协议
**状态**: ✅ 生产就绪 (Production Ready)
**许可**: 遵循项目主许可证

---
