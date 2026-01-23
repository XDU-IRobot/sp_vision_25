#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "tools/thread_safe_queue.hpp"
#include "io/command.hpp"

namespace io {
struct __attribute__((packed)) GimbalToVision {
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];   // wxyz顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count; // 子弹累计发送次数
  uint16_t crc16;
};

static_assert(sizeof(GimbalToVision) <= 64);

struct __attribute__((packed)) VisionToGimbal {
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode; // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  uint16_t crc16;
};

static_assert(sizeof(VisionToGimbal) <= 64);

enum class GimbalMode {
  IDLE,       // 空闲
  AUTO_AIM,   // 自瞄
  SMALL_BUFF, // 小符
  BIG_BUFF    // 大符
};

struct GimbalState {
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

class Gimbal {
public:
  Gimbal(const std::string &config_path);

  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
            float pitch, float pitch_vel, float pitch_acc);

  void send(io::VisionToGimbal VisionToGimbal);

  void send_command_scm(io::Command command);

private:
  serial::Serial serial_;

  std::thread thread_;
  std::atomic<bool> quit_ = false;
  mutable std::mutex mutex_;

  GimbalToVision rx_data_;
  VisionToGimbal tx_data_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_;
  tools::ThreadSafeQueue<
      std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
      queue_{1000};

  bool read(uint8_t *buffer, size_t size);
  void read_thread();
  void reconnect();

  // --- SCM 协议支持 ---
  bool use_scm_ = false;          // 是否启用你提供的 SCM 协议
  uint8_t scm_sof_ = 0x55;        // SOF 默认 0x55（可配置）
  uint8_t scm_eof_ = 0xFF;        // EOF 默认 0xFF（可配置）
  uint8_t scm_rx_id_ = 0x03;      // 电控→自瞄 帧 ID（可配置）
  uint8_t scm_tx_id_ = 0x02;      // 自瞄→电控 帧 ID（可配置）
  bool scm_angles_in_deg_ = true; // 角度单位是否使用度
  std::chrono::steady_clock::time_point start_tp_; // 发送中的 SystemTimer 基准

  void send_scm(bool control, bool fire, float yaw, float yaw_vel,
                float yaw_acc, float pitch, float pitch_vel, float pitch_acc);
  
  bool parse_scm_rx();
};

// 你提供的 SCM 协议结构体定义（packed）。为避免与宏 EOF 冲突，先取消其定义。
#ifdef EOF
#undef EOF
#endif

/*uav 发送数据包结构*/
typedef struct __attribute__((packed)) {
  // 包头
  uint8_t SOF;
  uint8_t ID;
  // 自瞄状态
  uint8_t AimbotState;
  uint8_t AimbotTarget;
  // 自瞄数据
  float Pitch;
  float Yaw;
  // 自瞄目标角速度
  float TargetPitchSpeed;
  float TargetYawSpeed;
  // 时间戳
  float SystemTimer;
  // 包尾
  uint8_t EOF;
  // 处理后数据
  float PitchRelativeAngle;
  float YawRelativeAngle;
} AimbotFrame_SCM_t;

/*步兵 发送数据包结构*/
typedef struct __attribute__((packed)) {
  // 包头
  uint8_t SOF;
  uint8_t ID;
  // 自瞄状态
  uint8_t AimbotState;
  uint8_t AimbotTarget;
  float Pitch;
  float Yaw;
  float SystemTimer;
  uint8_t _EOF;
}AimbotFrame_SCM_t_bot3;

typedef struct __attribute__((packed)) {
  // 包头
  uint8_t SOF;
  uint8_t ID;
  // IMU数据
  uint32_t TimeStamp;
  float q0;
  float q1;
  float q2;
  float q3;
  uint8_t robot_id;
  uint8_t mode;
  // 包尾
  uint8_t EOF;
} Gimaballmurname_SCM_t;

} // namespace io

#endif // IO__GIMBAL_HPP