#include "cboard.hpp"

#include <cstdint>

// OpenVINO头文件（如果需要的话，目前未使用）
// #include <openvino/core/type/element_type.hpp>
// #include "openvino/core/visibility.hpp"

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
  shoot_mode(ShootMode::left_shoot),
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
  new_can_cmd_id_ = 0x170;
  can_frame frame;
  frame.can_id = new_can_cmd_id_;
  frame.can_dlc = 7;  // 帧长度为7字节

  // Byte 0: AimbotState (位域)
  uint8_t aimbot_state = compute_aimbotstate(command.control, command.shoot);
  frame.data[0] = aimbot_state;

  // Byte 1: AimbotTarget (0/1 表示是否检测到目标/被控制)
  frame.data[1] = command.control ? 1 : 0;

  // 将yaw, pitch转换为f16格式（与电控侧保持一致：double -> float -> f16）
  // 坐标系转换：电控 pitch 向上为正，视觉 pitch 向下为正，需要取反
  float yaw_f32 = static_cast<float>(command.yaw);
  float pitch_f32 = static_cast<float>(-command.pitch);
  f16tools::f16 yaw_f16 = f16tools::f32_to_f16(yaw_f32);
  f16tools::f16 pitch_f16 = f16tools::f32_to_f16(pitch_f32);

  // Byte 2-3: yaw (f16, 大端序)
  frame.data[2] = (yaw_f16 >> 8) & 0xFF;
  frame.data[3] = yaw_f16 & 0xFF;

  // Byte 4-5: pitch (f16, 大端序)
  frame.data[4] = (pitch_f16 >> 8) & 0xFF;
  frame.data[5] = pitch_f16 & 0xFF;

  // Byte 6: NucStartFlag (固定为1表示NUC在线)
  frame.data[6] = 1;

  try {
    can_->write(&frame);

    if (debug_tx_) {
      // 转换为角度制显示
      double yaw_deg = command.yaw * 180.0 / M_PI;
      double pitch_deg = command.pitch * 180.0 / M_PI;
      tools::logger()->info(
        "[TX][NewCAN] id=0x{:03X} aimbot_state=0x{:02X} target={} yaw={:.2f}° pitch={:.2f}° nuc_flag={}",
        frame.can_id, aimbot_state, frame.data[1], yaw_deg, pitch_deg, frame.data[6]);
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

      // 解析欧拉角 (yaw, pitch, roll)
      auto be_to_f16 = [](const uint8_t * p) -> f16tools::f16 {
        return static_cast<f16tools::f16>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
      };

      f16tools::f16 yaw16 = be_to_f16(&frame.data[0]);
      f16tools::f16 pitch16 = be_to_f16(&frame.data[2]);
      f16tools::f16 roll16 = be_to_f16(&frame.data[4]);

      // f16 -> f32 -> double（与电控发送路径对称）
      float yaw_f32 = f16tools::f16_to_f32(yaw16);
      float pitch_f32 = f16tools::f16_to_f32(pitch16);
      float roll_f32 = f16tools::f16_to_f32(roll16);

      double yaw = static_cast<double>(yaw_f32);
      double pitch = static_cast<double>(pitch_f32);
      double roll = static_cast<double>(roll_f32);

      // 将欧拉角转换为四元数 (ZYX顺序: yaw-pitch-roll)
      Eigen::Quaterniond q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

      //  解析 byte6 的打包字段
      uint8_t packed = frame.data[6];
      uint8_t id_bit = (packed >> 6) & 0x1;      // 第6位：颜色标识
      uint8_t mode_bits = (packed >> 4) & 0x3;   // 第4-5位：模式
      uint8_t imu_bits = packed & 0xF;           // 低4位：IMU计数

      // 恢复 robot_id
      robot_id_ = id_bit ? (100 + mode_bits) : mode_bits;

      // 恢复 mode
      mode = static_cast<Mode>(mode_bits);

      // 恢复 imu_count（低4位）/*
      uint16_t imu_count_low = imu_bits%10;

      //  解析 byte7 的子弹速度
      bullet_speed = static_cast<double>(frame.data[7]) * 32.0 / 255.0;

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
        // 转换为角度制显示
        double yaw_deg = yaw * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        double roll_deg = roll * 180.0 / M_PI;
        tools::logger()->info(
          "[RX][NewCAN][Euler] id=0x{:03X} robot_id={} mode={} imu_count={} bullet_speed={:.2f} "
          "yaw={:.2f}° pitch={:.2f}° roll={:.2f}°",
          frame.can_id, robot_id_, MODES[mode], imu_count_low, bullet_speed, yaw_deg, pitch_deg, roll_deg);
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