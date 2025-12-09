# 大恒相机硬触发使用指南

## 📖 概述

硬触发功能允许通过外部GPIO信号触发相机曝光，实现与其他设备的精确同步。适用场景：
- 与云台IMU同步采集
- 与发射机构同步触发
- 多相机同步采集
- 外部编码器同步

---

## ⚙️ 配置参数

在 `configs/daheng.yaml` 中配置以下参数：

```yaml
#####-----硬触发配置-----#####
trigger_enable: false        # 是否启用硬触发
trigger_source: 2            # 触发源：0=Line0, 1=Line1, 2=Line2
trigger_activation: 0        # 触发沿：0=上升沿, 1=下降沿
```

### 参数详解

#### `trigger_enable`
- **类型**: `bool`
- **默认值**: `false`
- **说明**:
  - `false`: 连续采集模式（软件自由采集）
  - `true`: 硬触发模式（等待外部信号）

#### `trigger_source`
- **类型**: `int`
- **可选值**: `0` / `1` / `2`
- **默认值**: `2`
- **说明**: 选择触发信号输入的GPIO线
  - `0` → Line0 (GPIO 1，相机接口Pin 1)
  - `1` → Line1 (GPIO 2，相机接口Pin 2)
  - `2` → Line2 (GPIO 3，相机接口Pin 3)

#### `trigger_activation`
- **类型**: `int`
- **可选值**: `0` / `1`
- **默认值**: `0`
- **说明**: 选择触发信号的有效边沿
  - `0` → 上升沿触发 (低电平→高电平)
  - `1` → 下降沿触发 (高电平→低电平)

---

## 🔌 硬件连接

### 大恒相机GPIO接口定义

以大恒 MER-xxx 系列相机为例，GPIO接口通常为6针Hirose连接器：

```
接口引脚定义：
Pin 1: Line0 (GPIO 1) - 可配置为输入/输出
Pin 2: Line1 (GPIO 2) - 可配置为输入/输出
Pin 3: Line2 (GPIO 3) - 可配置为输入/输出
Pin 4: GND         - 地线
Pin 5: 12V Out     - 12V电源输出
Pin 6: Opto GND    - 光耦地
```

### 连接方式

#### 方式1：TTL电平触发（推荐）
```
触发源设备 ──┐
             │
         [电阻]  (可选，限流保护)
             │
         Pin X (Line0/1/2) ──┐
                             │── 相机
         Pin 4 (GND) ────────┘
```

#### 方式2：光耦隔离触发（工业场景）
```
触发源设备 ──┐
             │
         [光耦] ──── Pin X (Line0/1/2)
             │
         [电阻] ──── Pin 6 (Opto GND)
```

### 电气规格
- **电平范围**: 0V ~ 5V (TTL)
- **电平阈值**:
  - 低电平: < 0.8V
  - 高电平: > 2.4V
- **最大输入频率**: ~5kHz (依相机型号)
- **触发脉冲宽度**: ≥ 1μs

---

## 🚀 使用示例

### 示例1：云台IMU同步触发

**场景**: 云台每次运动到指定位置时触发相机采集

**配置**:
```yaml
trigger_enable: true
trigger_source: 2         # 使用Line2
trigger_activation: 0     # 上升沿触发
```

**硬件连接**:
```
云台控制板 GPIO Output → 相机 Line2 (Pin 3)
云台控制板 GND         → 相机 GND (Pin 4)
```

**工作流程**:
1. 相机启动后进入等待触发状态
2. 云台到达目标位置，GPIO输出高电平
3. 相机检测到上升沿，立即曝光
4. 相机采集完成，等待下一次触发

---

### 示例2：多相机同步采集

**场景**: 两台相机同时采集，确保时间同步

**配置** (两台相机均配置):
```yaml
trigger_enable: true
trigger_source: 0         # 使用Line0
trigger_activation: 0     # 上升沿触发
```

**硬件连接**:
```
                    ┌─→ 相机1 Line0 (Pin 1)
触发源 (定时器) ────┤
                    └─→ 相机2 Line0 (Pin 1)

触发源 GND ─────┬─→ 相机1 GND (Pin 4)
               └─→ 相机2 GND (Pin 4)
```

**触发源实现** (Arduino示例):
```cpp
// 100Hz同步触发
void setup() {
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
}

void loop() {
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);  // 10μs脉冲
    digitalWrite(TRIGGER_PIN, LOW);
    delay(10);              // 100Hz = 10ms周期
}
```

---

### 示例3：编码器同步触发

**场景**: 云台旋转时，每隔一定角度触发采集

**配置**:
```yaml
trigger_enable: true
trigger_source: 1         # 使用Line1
trigger_activation: 0     # 上升沿触发
```

**硬件连接**:
```
编码器 Phase A → 相机 Line1 (Pin 2)
编码器 GND    → 相机 GND (Pin 4)
```

---

## 🐛 故障排查

### 问题1：启用硬触发后无图像输出

**可能原因**:
- 触发信号未连接或连接错误
- 触发信号电平不符合TTL标准
- 触发沿配置错误

**排查步骤**:
1. 检查硬件连接是否牢固
2. 用示波器测量触发信号电平
3. 尝试切换 `trigger_activation` (0↔1)
4. 先用连续模式测试相机是否正常 (`trigger_enable: false`)

---

### 问题2：触发后帧率异常

**可能原因**:
- 触发频率超过相机最大采集速度
- 曝光时间过长，来不及响应触发

**解决方案**:
```yaml
camera_settings:
  exposure: 2100          # 减小曝光时间
  fps: 168                # 确认相机支持的最大帧率
```

**最大触发频率计算**:
```
最大频率 = 1 / (曝光时间 + 传输时间)
例如: exposure=2.1ms, 传输=1ms
最大频率 ≈ 322 Hz
```

---

### 问题3：触发不稳定，偶尔丢帧

**可能原因**:
- 触发脉冲宽度不足
- 信号抖动或噪声干扰

**解决方案**:
1. 增加触发脉冲宽度至 ≥ 10μs
2. 添加RC滤波电路去除毛刺
3. 使用光耦隔离 (工业环境)

**滤波电路示例**:
```
触发信号 ──[100Ω]──┬──→ 相机 LineX
                   │
                  [0.1μF]
                   │
                  GND
```

---

### 问题4：日志显示"Failed to set trigger mode ON"

**可能原因**:
- 相机固件版本不支持硬触发
- 驱动库版本过旧

**解决方案**:
1. 检查相机型号是否支持硬触发功能
2. 更新大恒相机驱动到最新版本
3. 查看 `/home/anna/OPEN/SP_vision_USB/io/driver/` 中的SDK版本

---

## 📊 性能参考

| 场景 | 触发频率 | 曝光时间 | 延迟 | 备注 |
|------|---------|---------|------|------|
| 云台同步 | 20-100 Hz | 2-5 ms | < 1 ms | 推荐 |
| 多相机同步 | 30-60 Hz | 2-10 ms | < 0.5 ms | 同步精度高 |
| 高速采集 | 200-300 Hz | < 2 ms | < 1 ms | 需短曝光 |
| 低速监控 | 1-10 Hz | 5-20 ms | < 2 ms | 可长曝光 |

---

## 📝 日志解读

启用硬触发后，日志输出示例：

```bash
[info] Daheng external trigger enabled: source=2 (Line2), activation=0 (RisingEdge)
[info] Daheng camera configured successfully
```

如果配置失败，会有警告：
```bash
[warn] Failed to enable trigger mode ON
[warn] Failed to set trigger source to Line2
[warn] Failed to set Line2 mode to INPUT
```

---

## 🔧 高级应用

### 自适应触发频率

可以根据目标速度动态调整触发频率，提高采集效率：

```cpp
// 伪代码
if (target_speed > HIGH_SPEED_THRESHOLD) {
    trigger_freq = 200;  // 高速目标，高频采集
} else {
    trigger_freq = 50;   // 低速目标，节省带宽
}
```

### 触发延迟补偿

如果发现触发信号与实际需要采集的时刻有固定延迟，可以在上位机提前或延后发送触发信号。

---

## 📚 参考资料

- 大恒相机SDK文档: `/opt/Daheng/SDK/doc/`
- GxIAPI.h 头文件: `/opt/Daheng/SDK/inc/GxIAPI.h`
- 项目代码: `io/daheng.cpp:330-433`

---

## ✅ 测试清单

在实际使用前，建议完成以下测试：

- [ ] 连续模式正常工作 (`trigger_enable: false`)
- [ ] 硬触发模式日志无错误 (`trigger_enable: true`)
- [ ] 手动短接GPIO测试触发响应
- [ ] 实际触发源连接测试
- [ ] 触发频率压力测试
- [ ] 长时间稳定性测试 (>1小时)

---

**更新日期**: 2025-12-09
**作者**: Claude Code
**版本**: v1.0
