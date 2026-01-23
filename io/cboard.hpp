#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"
#include <memory>
#include <serial/serial.h>

namespace io {
enum Mode { idle, auto_aim, small_buff, big_buff, outpost };
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff",
                                        "big_buff", "outpost"};

enum ShootMode { left_shoot, right_shoot, both_shoot };
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot",
                                              "both_shoot"};

class CBoard {
public:
  struct IMUData {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
    uint16_t imu_count;      // 完整的IMU计数器：0-9999循环
    uint8_t cycle_count;     // IMU周期计数：1-10循环
  };

  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;

  CBoard(const std::string &config_path);
  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // 🆕 环形数组直接查询接口（推荐使用）
  struct IMUQueryResult {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
    bool valid;
  };
  IMUQueryResult get_imu_from_ring_buffer(uint16_t target_imu_count) const;

  void send(Command command);

private:
  tools::ThreadSafeQueue<IMUData> queue_;

  // 🆕 环形数组：用于高效存储和查询IMU数据
  static constexpr size_t IMU_RING_BUFFER_SIZE = 10;

  struct IMUFrame {
    uint16_t imu_count;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
    std::atomic<bool> valid;

    IMUFrame() : imu_count(0), q(1, 0, 0, 0), valid(false) {}
  };

  IMUFrame imu_ring_buffer_[IMU_RING_BUFFER_SIZE];

  std::unique_ptr<SocketCAN> can_;
  serial::Serial serial_;
  std::thread serial_thread_;
  std::atomic<bool> serial_quit_{false};
  bool use_serial_ = false;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;

  // ===== 新CAN协议配置 =====
  bool use_new_can_protocol_ = false;
  int new_can_quat_id_ = 0x150;
  int new_can_cmd_id_ = 0x170;
  uint8_t robot_id_ = 0;
  uint16_t imu_count_ = 0;
  uint16_t last_imu_count_ = 0;
  std::atomic<bool> mcu_online_{false};

  // 🆕 调试开关配置
  bool debug_rx_ = false;
  bool debug_tx_ = false;

  static constexpr uint8_t AIMBOT_BIT_HAS_TARGET = 0x01;
  static constexpr uint8_t AIMBOT_BIT_SUGGEST_FIRE = 0x02;
  static constexpr uint8_t AIMBOT_BIT_SELF_AIM = 0x20;

  bool gimbal_pitch_from_x_ = false;
  int yaw_sign_ = 1;
  int pitch_sign_ = 1;
  bool normalize_abs_angles_ = true;

  bool tx_has_last_ = false;
  float tx_last_yaw_ = 0.f;
  float tx_last_pitch_ = 0.f;
  std::chrono::steady_clock::time_point tx_last_tp_{};

  uint8_t compute_aimbotstate(bool control, bool fire);

  void callback(const can_frame &frame);

  std::string read_yaml(const std::string &config_path);
};

} // namespace io

#endif // IO__CBOARD_HPP