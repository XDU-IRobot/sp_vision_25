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

  // 🆕 启动相机触发信号（在程序完全初始化后调用）
  void start_camera_trigger();

#ifdef AMENT_CMAKE_FOUND
  // 🆕 设置ROS2节点用于实时发布TF（IMU数据到达时立即发布）
  void set_ros2_tf_publisher(
    std::shared_ptr<void> node,  // 类型擦除，避免强依赖
    const Eigen::Matrix3d & R_gimbal2imubody);

  // 🆕 将 steady_clock 时间戳转换为 ROS 时间（使用与TF相同的时间基准）
  // 用于确保 Marker 和 TF 使用同一时间戳
  std::shared_ptr<void> convert_to_ros_time(std::chrono::steady_clock::time_point timestamp);
#endif

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

  int new_can_quat_id_;
  int new_can_cmd_id_;
  uint8_t robot_id_ = 0;
  uint16_t imu_count_ = 0;
  uint16_t last_imu_count_ = 0;
  std::atomic<bool> mcu_online_{false};


  bool debug_rx_=true;
  bool debug_tx_;

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

#ifdef AMENT_CMAKE_FOUND
  // ROS2 TF发布相关
  std::shared_ptr<void> ros_node_;  // 类型擦除，避免强依赖
  std::shared_ptr<void> tf_broadcaster_;
  Eigen::Matrix3d R_gimbal2imubody_;  // 用于计算R_gimbal2world
  std::chrono::steady_clock::time_point ros_time_base_;  // steady_clock时间基准
  std::shared_ptr<void> ros_time_start_;  // ROS时间基准 (类型擦除)
#endif
};

} // namespace io

#endif // IO__CBOARD_HPP