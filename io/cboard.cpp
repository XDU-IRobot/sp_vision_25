#include "cboard.hpp"

#include <cstdint>
#include <openvino/core/type/element_type.hpp>

#include "openvino/core/visibility.hpp"
#include "tools/crc.hpp"
#include "tools/float16.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

// ROS2 TF2 支持头文件（需要在 namespace 之前包含）
#ifdef AMENT_CMAKE_FOUND
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#endif

namespace io
{
CBoard::CBoard(const std::string & config_path)
: mode(Mode::idle),
  bullet_speed(0),
  queue_(5000)  // 注意: callback的运行会早于Cboard构造函数的完成
{
  auto transport = read_yaml(config_path);


    can_ = std::make_unique<SocketCAN>(
    transport, std::bind(&CBoard::callback, this, std::placeholders::_1));

  tools::logger()->info("[Cboard] Waiting for q...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[Cboard] Opened.");
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  if (data_behind_.timestamp < timestamp) data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp) break;
    data_ahead_ = data_behind_;
  }

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;
  auto t_c = timestamp;
  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = t_c - t_a;

  // 四元数插值
  auto k = t_ac / t_ab;
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command)
{
 new_can_cmd_id_=0x170;
    can_frame frame;
    frame.can_id = new_can_cmd_id_;
    frame.can_dlc = 7;

    frame.data[6] = 1;  // start标志一直为1

    frame.data[0] = compute_aimbotstate(command.control, command.shoot);
    frame.data[1] = command.control ? 1 : 0;

    double yaw_rel = static_cast<double>(command.yaw);
    double pitch_rel = static_cast<double>(command.pitch);
    f16tools::f16 yaw_int = f16tools::f64_to_f16(yaw_rel);
    f16tools::f16 pitch_int = f16tools::f64_to_f16(pitch_rel);

    frame.data[2] = (yaw_int >> 8) & 0xFF;
    frame.data[3] = yaw_int & 0xFF;
    frame.data[4] = (pitch_int >> 8) & 0xFF;
    frame.data[5] = pitch_int & 0xFF;

    try {
      can_->write(&frame);

      if (debug_tx_) {
        tools::logger()->info(
          "[TX][NewCAN] id=0x{:03X} state=0x{:02X} target={} yaw={} pitch={}", frame.can_id,
          frame.data[0], frame.data[1], yaw_int ,pitch_int);
      }
    } catch (const std::exception & e) {
      tools::logger()->warn("[NewCAN] write failed: {}", e.what());
  }
  return;
}

CBoard::~CBoard() {}

void CBoard::callback(const can_frame & frame)
{
  auto timestamp = std::chrono::steady_clock::now();
    // 下位机(0x150)
    if (frame.can_id == new_can_quat_id_) {
      if (frame.can_dlc < 8) {
        tools::logger()->warn("[NewCAN] Quaternion frame length invalid: {}", frame.can_dlc);
        return;
      }

      // 解析4个f16四元数分量
      auto be_to_f16 = [](const uint8_t * p) -> f16tools::f16 {
        return static_cast<f16tools::f16>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
      };

      f16tools::f16 q_w16 = be_to_f16(&frame.data[0]);
      f16tools::f16 q_x16 = be_to_f16(&frame.data[2]);
      f16tools::f16 q_y16 = be_to_f16(&frame.data[4]);

      double qw = f16tools::f16_to_f64(q_w16);
      double qx = f16tools::f16_to_f64(q_x16);
      double qy = f16tools::f16_to_f64(q_y16);

      //  解析 byte6 的打包字段
      uint8_t packed = frame.data[6];
      uint8_t z_sign = (packed >> 7) & 0x1;
      uint8_t id_bit = (packed >> 6) & 0x1;
      uint8_t mode_bits = 1;
      uint8_t imu_bits = packed & 0xF;
      //  根据归一化约束和符号位恢复 z
      double qz_abs = std::sqrt(std::max(0.0, 1.0 - qw * qw - qx * qx - qy * qy));
      double qz = (z_sign == 1) ? qz_abs : -qz_abs;
      // 恢复 robot_id
      robot_id_ = id_bit ? (100 + mode_bits) : mode_bits;

      // 恢复 mode
      mode = static_cast<Mode>(mode_bits);

      // 恢复 imu_count（低4位）/*
      uint16_t imu_count_low = imu_bits%10;

      //  解析 byte7 的子弹速度
      bullet_speed = static_cast<double>(frame.data[7]) * 32.0 / 255.0;
      Eigen::Quaterniond q(qw,qx,qy,qz);
      imu_ring_buffer_[imu_count_low].q = q;
      imu_ring_buffer_[imu_count_low].timestamp = timestamp;
      imu_ring_buffer_[imu_count_low].imu_count = imu_count_low;
      imu_ring_buffer_[imu_count_low].valid.store(true, std::memory_order_release);

      // 推入队列（兼容旧接口）
      IMUData imu_data;
      imu_data.q = q;
      imu_data.timestamp = timestamp;
      imu_data.imu_count = imu_count_low;
      imu_data.cycle_count = imu_count_low % 10;
      queue_.push(imu_data);

      mcu_online_.store(true, std::memory_order_relaxed);

      if (debug_rx_) {
        tools::logger()->info(
          "[RX][NewCAN][Quat] id=0x{:03X} robot_id={} mode={} imu_count={} bullet_speed={:.2f} "
          "q({:.4f},{:.4f},{:.4f},{:.4f})",
          frame.can_id, robot_id_, MODES[mode], imu_count_low, bullet_speed, q.w(), q.x(), q.y(),
          q.z());
      }
      return;
    }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  // 传输后端选择
  std::string transport = "can";
  // 读取新CAN协议配置
      tools::logger()->info("[CBoard] Using NEW CAN protocol");

      // 读取新协议的CAN ID配置（提供默认值）
      if (yaml["new_can_quat_id"]) {
        new_can_quat_id_ = yaml["new_can_quat_id"].as<int>();
      }
      if (yaml["new_can_cmd_id"]) {
        new_can_cmd_id_ = yaml["new_can_cmd_id"].as<int>();
      }

      tools::logger()->info(
        "[CBoard] New CAN IDs: quat=0x{:03X}, cmd=0x{:03X}", new_can_quat_id_, new_can_cmd_id_);
  

  // 读取调试开关配置（CAN模式）
  if (yaml["debug_rx"]) {
    debug_rx_ = yaml["debug_rx"].as<bool>();
  }
  if (yaml["debug_tx"]) {
    debug_tx_ = yaml["debug_tx"].as<bool>();
  }
  return yaml["can_interface"].as<std::string>();
}

// 🆕 环形数组直接查询接口
CBoard::IMUQueryResult CBoard::get_imu_from_ring_buffer(uint16_t target_imu_count) const
{
  size_t index = target_imu_count % IMU_RING_BUFFER_SIZE;

  bool is_valid = imu_ring_buffer_[index].valid.load(std::memory_order_acquire);

  if (is_valid && imu_ring_buffer_[index].imu_count == target_imu_count) {
    return {imu_ring_buffer_[index].q, imu_ring_buffer_[index].timestamp, true};
  } else {
    return {Eigen::Quaterniond(1, 0, 0, 0), std::chrono::steady_clock::time_point(), false};
  }
}
uint8_t CBoard::compute_aimbotstate(bool control, bool fire)
{
  uint8_t bits = 0;
  if (control) bits |= AIMBOT_BIT_HAS_TARGET;
  if (fire) bits |= AIMBOT_BIT_SUGGEST_FIRE;
  if (control) bits |= AIMBOT_BIT_SELF_AIM;
  return bits;
}

}  // namespace io