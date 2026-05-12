#include "sentry_can.hpp"

#include <cmath>
#include <functional>
#include <stdexcept>

#include "tools/float16.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io {

namespace {
constexpr double kPi = 3.14159265358979323846;
enum class RxMode : uint8_t { idle, auto_aim, small_buff, big_buff, outpost };

static inline double rad2deg(double r) { return r * 180.0 / kPi; }

f16tools::f16 be_to_f16(const uint8_t *p) {
  return static_cast<f16tools::f16>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}

GimbalMode to_gimbal_mode(RxMode mode) {
  switch (mode) {
  case RxMode::auto_aim:
    return GimbalMode::AUTO_AIM;
  case RxMode::small_buff:
    return GimbalMode::SMALL_BUFF;
  case RxMode::big_buff:
    return GimbalMode::BIG_BUFF;
  case RxMode::outpost:
  case RxMode::idle:
  default:
    return GimbalMode::IDLE;
  }
}
} // namespace

SentryCan::SentryCan(const std::string &config_path) {
  load_config(config_path);

  can_ = std::make_unique<SocketCAN>(
      can_interface_, std::bind(&SentryCan::callback, this, std::placeholders::_1));

  tools::logger()->info("[SentryCan] Opened CAN {} (pose=0x{:03X}, cmd=0x{:03X})",
                        can_interface_, pose_can_id_, cmd_can_id_);
  tools::logger()->info("[SentryCan] Waiting for q...");
  queue_.pop();
  tools::logger()->info("[SentryCan] First q received.");
}

SentryCan::~SentryCan() = default;

GimbalMode SentryCan::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState SentryCan::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string SentryCan::str(GimbalMode mode) const {
  switch (mode) {
  case GimbalMode::IDLE:
    return "IDLE";
  case GimbalMode::AUTO_AIM:
    return "AUTO_AIM";
  case GimbalMode::SMALL_BUFF:
    return "SMALL_BUFF";
  case GimbalMode::BIG_BUFF:
    return "BIG_BUFF";
  default:
    return "INVALID";
  }
}

Eigen::Quaterniond SentryCan::q(std::chrono::steady_clock::time_point t) {
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_b, t_a);
    auto t_ac = tools::delta_time(t, t_a);

    if (std::abs(t_ab) < 1e-9) {
      return q_b.normalized();
    }

    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a)
      return q_c;
    if (!(t_a < t && t <= t_b))
      continue;

    return q_c;
  }
}

void SentryCan::send(VisionToGimbal vision_to_gimbal) {
  bool control = vision_to_gimbal.mode != 0;
  bool fire = vision_to_gimbal.mode == 2;
  send(control, fire, vision_to_gimbal.yaw, vision_to_gimbal.yaw_vel,
       vision_to_gimbal.yaw_acc, vision_to_gimbal.pitch,
       vision_to_gimbal.pitch_vel, vision_to_gimbal.pitch_acc);
}

void SentryCan::send(bool control, bool fire, float yaw, float, float, float pitch, float, float) {
  if (!can_ || close_send_) {
    return;
  }

  can_frame frame{};
  frame.can_id = cmd_can_id_;
  frame.can_dlc = 7;
  frame.data[0] = compute_aimbot_state(control, fire);
  frame.data[1] = control ? 1 : 0;
  frame.data[6] = 1;

  if (std::isnan(yaw) || std::isnan(pitch)) {
    frame.data[0] = 0;
    frame.data[1] = 0;
    yaw = 0.0f;
    pitch = 0.0f;
  }

  double yaw_rel = tx_angles_in_deg_ ? rad2deg(yaw) : static_cast<double>(yaw);
  double pitch_rel = tx_angles_in_deg_ ? rad2deg(pitch) : static_cast<double>(pitch);

  yaw_rel = -yaw_rel;
  pitch_rel = -pitch_rel;

  f16tools::f16 yaw_f16 = f16tools::f64_to_f16(yaw_rel);
  f16tools::f16 pitch_f16 = f16tools::f64_to_f16(pitch_rel);

  frame.data[2] = (yaw_f16 >> 8) & 0xFF;
  frame.data[3] = yaw_f16 & 0xFF;
  frame.data[4] = (pitch_f16 >> 8) & 0xFF;
  frame.data[5] = pitch_f16 & 0xFF;

  try {
    can_->write(&frame);
    if (debug_tx_) {
      tools::logger()->info(
          "[TX][SentryCan] id=0x{:03X} state=0x{:02X} target={} yaw={} pitch={}",
          frame.can_id, frame.data[0], frame.data[1], yaw_rel, pitch_rel);
    }
  } catch (const std::exception &e) {
    tools::logger()->warn("[SentryCan] write failed: {}", e.what());
  }
}

void SentryCan::send_command_scm(io::Command command) {
  send(command.control, command.shoot, static_cast<float>(command.yaw), 0.0f, 0.0f,
       static_cast<float>(command.pitch), 0.0f, 0.0f);
}

void SentryCan::send_command_scm_can(io::Command command) { send_command_scm(command); }

void SentryCan::callback(const can_frame &frame) {
  if ((frame.can_id & CAN_SFF_MASK) == pose_can_id_) {
    parse_pose_frame(frame);
  }
}

void SentryCan::parse_pose_frame(const can_frame &frame) {
  if (frame.can_dlc < 8) {
    tools::logger()->warn("[SentryCan] Pose frame length invalid: {}", frame.can_dlc);
    return;
  }

  auto timestamp = std::chrono::steady_clock::now();

  double yaw = f16tools::f16_to_f64(be_to_f16(&frame.data[0]));
  double pitch = f16tools::f16_to_f64(be_to_f16(&frame.data[2]));
  double roll = f16tools::f16_to_f64(be_to_f16(&frame.data[4]));

  uint8_t packed = frame.data[6];
  uint8_t id_bit = (packed >> 7) & 0x1;
  uint8_t mode_bits = (packed >> 4) & 0x7;
  uint8_t imu_bits = packed & 0xF;
  uint16_t imu_count_low = imu_bits % 10;
  double bullet_speed = static_cast<double>(frame.data[7]) * 32.0 / 255.0;

  RxMode rx_mode = RxMode::idle;
  if (mode_bits <= static_cast<uint8_t>(RxMode::outpost)) {
    rx_mode = static_cast<RxMode>(mode_bits);
  }
  GimbalMode gimbal_mode = to_gimbal_mode(rx_mode);

  Eigen::Vector3d p_zero(0.0, 0.0, 0.0);

  Eigen::AngleAxisd Rx(kPi * 0.5, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry(-kPi * 0.5, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz(0.0, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R_imu_to_odom = Rz.toRotationMatrix() * Ry * Rx;

  Eigen::Isometry3d T_imu_to_odom = Eigen::Isometry3d::Identity();
  T_imu_to_odom.rotate(R_imu_to_odom);
  T_imu_to_odom.pretranslate(p_zero);

  Eigen::AngleAxisd Rx_gimbal(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd Ry_gimbal(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd Rz_gimbal(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R_gimbal_to_imu =
      Rz_gimbal.toRotationMatrix() * Ry_gimbal * Rx_gimbal;

  Eigen::Isometry3d T_gimbal_to_imu = Eigen::Isometry3d::Identity();
  T_gimbal_to_imu.rotate(R_gimbal_to_imu);
  T_gimbal_to_imu.pretranslate(p_zero);

  Rx = Eigen::AngleAxisd(-kPi * 0.5, Eigen::Vector3d::UnitX());
  Ry = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY());
  Rz = Eigen::AngleAxisd(-kPi * 0.5, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d R_camera_fake_to_gimbal = Rz.toRotationMatrix() * Ry * Rx;

  Eigen::Isometry3d T_camera_fake_to_gimbal = Eigen::Isometry3d::Identity();
  T_camera_fake_to_gimbal.rotate(R_camera_fake_to_gimbal);
  T_camera_fake_to_gimbal.pretranslate(p_zero);

  auto T_camera_fake_to_odom =
      T_imu_to_odom * T_gimbal_to_imu * T_camera_fake_to_gimbal;
  Eigen::Matrix3d rotate_matrix = T_camera_fake_to_odom.linear().matrix();

  double r13 = rotate_matrix(0, 2);
  double r21 = rotate_matrix(1, 0);
  double r22 = rotate_matrix(1, 1);
  double r23 = rotate_matrix(1, 2);
  double r33 = rotate_matrix(2, 2);

  Eigen::Vector3d euler_angle;
  euler_angle.z() = std::atan2(r21, r22);
  euler_angle.x() = std::atan2(-r23, std::sqrt(r21 * r21 + r22 * r22));
  euler_angle.y() = std::atan2(r13, r33);

  Eigen::Quaterniond q(rotate_matrix);
  q.normalize();
  queue_.push({q, timestamp});

  {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = gimbal_mode;
    state_.yaw = static_cast<float>(euler_angle.y());
    state_.yaw_vel = 0.0f;
    state_.pitch = static_cast<float>(euler_angle.x());
    state_.pitch_vel = 0.0f;
    state_.bullet_speed = static_cast<float>(bullet_speed);
    state_.bullet_count = imu_count_low;
    roll_ = euler_angle.z();
    enemy_color_bit_ = id_bit;
    imu_count_ = imu_count_low;
  }

  if (debug_rx_) {
    tools::logger()->info(
        "[RX][SentryCan] id=0x{:03X} color_bit={} mode={} imu_count={} bullet_speed={:.2f} "
        "imu_ypr({:.4f},{:.4f},{:.4f}) gimbal_ypr({:.4f},{:.4f},{:.4f})",
        frame.can_id, static_cast<int>(id_bit), mode_bits, imu_count_low, bullet_speed, yaw,
        pitch, roll, euler_angle.y(), euler_angle.x(), euler_angle.z());
  }
}

void SentryCan::load_config(const std::string &config_path) {
  auto yaml = tools::load(config_path);

  try {
    if (yaml["sentry_can_interface"]) {
      can_interface_ = yaml["sentry_can_interface"].as<std::string>();
    } else if (yaml["gimbal_can_interface"]) {
      can_interface_ = yaml["gimbal_can_interface"].as<std::string>();
    } else if (yaml["can_interface"]) {
      can_interface_ = yaml["can_interface"].as<std::string>();
    }

    if (yaml["sentry_can_pose_id"]) {
      pose_can_id_ = static_cast<canid_t>(yaml["sentry_can_pose_id"].as<uint32_t>());
    } else if (yaml["new_can_quat_id"]) {
      pose_can_id_ = static_cast<canid_t>(yaml["new_can_quat_id"].as<uint32_t>());
    } else if (yaml["gimbal_can_rx_id"]) {
      pose_can_id_ = static_cast<canid_t>(yaml["gimbal_can_rx_id"].as<uint32_t>());
    }

    if (yaml["sentry_can_cmd_id"]) {
      cmd_can_id_ = static_cast<canid_t>(yaml["sentry_can_cmd_id"].as<uint32_t>());
    } else if (yaml["new_can_cmd_id"]) {
      cmd_can_id_ = static_cast<canid_t>(yaml["new_can_cmd_id"].as<uint32_t>());
    } else if (yaml["gimbal_can_tx_id"]) {
      cmd_can_id_ = static_cast<canid_t>(yaml["gimbal_can_tx_id"].as<uint32_t>());
    }

    if (yaml["debug_rx"]) {
      debug_rx_ = yaml["debug_rx"].as<bool>();
    }
    if (yaml["debug_tx"]) {
      debug_tx_ = yaml["debug_tx"].as<bool>();
    }
    if (yaml["sentry_can_close_send"]) {
      close_send_ = yaml["sentry_can_close_send"].as<bool>();
    }
    if (yaml["sentry_can_tx_angles_in_deg"]) {
      tx_angles_in_deg_ = yaml["sentry_can_tx_angles_in_deg"].as<bool>();
    } else if (yaml["scm_angles_in_deg"]) {
      tx_angles_in_deg_ = yaml["scm_angles_in_deg"].as<bool>();
    }
  } catch (const std::exception &e) {
    tools::logger()->warn("[SentryCan] Failed to read optional config: {}", e.what());
  }
}

uint8_t SentryCan::compute_aimbot_state(bool control, bool fire) const {
  uint8_t bits = 0;
  if (control)
    bits |= AIMBOT_BIT_HAS_TARGET;
  if (fire)
    bits |= AIMBOT_BIT_SUGGEST_FIRE;
  if (control)
    bits |= AIMBOT_BIT_SELF_AIM;
  return bits;
}

} // namespace io
