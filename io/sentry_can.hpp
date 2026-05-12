#ifndef IO__SENTRY_CAN_HPP
#define IO__SENTRY_CAN_HPP

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/socketcan.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

class SentryCan {
public:
  explicit SentryCan(const std::string &config_path);
  ~SentryCan();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
            float pitch, float pitch_vel, float pitch_acc);
  void send(VisionToGimbal vision_to_gimbal);
  void send_command_scm(io::Command command);
  void send_command_scm_can(io::Command command);

private:
  static constexpr uint8_t AIMBOT_BIT_HAS_TARGET = 0x01;
  static constexpr uint8_t AIMBOT_BIT_SUGGEST_FIRE = 0x02;
  static constexpr uint8_t AIMBOT_BIT_SELF_AIM = 0x20;

  void callback(const can_frame &frame);
  void parse_pose_frame(const can_frame &frame);
  void load_config(const std::string &config_path);
  uint8_t compute_aimbot_state(bool control, bool fire) const;

  std::unique_ptr<SocketCAN> can_;
  std::string can_interface_ = "can0";
  canid_t pose_can_id_ = 0x150;
  canid_t cmd_can_id_ = 0x170;
  bool debug_rx_ = false;
  bool debug_tx_ = false;
  bool close_send_ = false;
  bool tx_angles_in_deg_ = true;

  mutable std::mutex mutex_;
  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_{};
  double roll_ = 0.0;
  uint8_t enemy_color_bit_ = 0;
  uint16_t imu_count_ = 0;

  tools::ThreadSafeQueue<
      std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
      queue_{1000};
};

} // namespace io

#endif // IO__SENTRY_CAN_HPP
