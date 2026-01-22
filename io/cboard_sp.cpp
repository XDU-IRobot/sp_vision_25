#include "cboard_sp.hpp"

#include <sys/types.h>

#include <cstdint>

#include "tools/float16.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
namespace io
{

CBoard::CBoard(const std::string & config_path) : mode(Mode::idle), bullet_speed(0)
{
  tools::logger()->info("[CBoard_SP] Initializing...");

  // 从配置文件读取 CAN ID
  auto yaml = tools::load(config_path);
  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");
  auto transport = read_yaml(config_path);
  tools::logger()->info(
    "[CBoard_SP] Initialized with quaternion_canid: {}, send_canid: {}", quaternion_canid_,
    send_canid_);

  can_ = std::make_unique<SocketCAN>(
    transport, std::bind(&CBoard::callback, this, std::placeholders::_1));
}

Eigen::Quaterniond CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  //   if (data_behind_.timestamp < timestamp)
  //   data_ahead_ = data_behind_;

  // while (true) {
  //   queue_.pop(data_behind_);
  //   if (data_behind_.timestamp > timestamp)
  //     break;
  //   data_ahead_ = data_behind_;
  // }

  // Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  // Eigen::Quaterniond q_b = data_behind_.q.normalized();
  // auto t_a = data_ahead_.timestamp;
  // auto t_b = data_behind_.timestamp;
  // auto t_c = timestamp;
  // std::chrono::duration<double> t_ab = t_b - t_a;
  // std::chrono::duration<double> t_ac = t_c - t_a;

  // // 四元数插值
  // auto k = t_ac / t_ab;
  // Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  // return q_c;
  return Eigen::Quaterniond(1, 0, 0, 0);
}
Eigen::Quaterniond CBoard::imu_by_count(uint16_t target_count)
{
  const uint8_t low4 = static_cast<uint8_t>(target_count & 0x0F);
  const uint8_t slot = low4 % IMU_RING_BUFFER_SIZE;

  if (
    imu_ring_buffer_[slot].valid.load(std::memory_order_acquire) &&
    imu_ring_buffer_[slot].imu_count_short == low4) {
    return imu_ring_buffer_[slot].q;
  } else {
    return Eigen::Quaterniond(1, 0, 0, 0);
  }
}

void CBoard::send(Command command) const
{
  can_frame frame;
  frame.can_id = send_canid_;
  frame.can_dlc = 7;  // 7字节数据

  // 🆕 start字段一直为1，表示NUC在线
  frame.data[6] = 1;
  // 数据打包：
  // byte 0: AimbotState (u8)
  // byte 1: AimbotTarget (u8)
  // byte 2-3: Yaw (f16)
  // byte 4-5: Pitch (f16)
  // byte 6: NucStartFlag (u8) - 一直为1

  // AimbotState: 使用compute_aimbotstate计算
  frame.data[0] = compute_aimbotstate(command.control, command.shoot);

  // AimbotTarget: 暂时固定为1（检测到目标）或0（未检测到）
  frame.data[1] = command.control ? 1 : 0;

  // Yaw和Pitch: 转为 f16，大端字节序
  f16tools::f16 yaw_int = f16tools::f64_to_f16(static_cast<double>(command.yaw));
  f16tools::f16 pitch_int = f16tools::f64_to_f16(static_cast<double>(command.pitch));

  frame.data[2] = (yaw_int >> 8) & 0xFF;    // Yaw高8位
  frame.data[3] = yaw_int & 0xFF;           // Yaw低8位
  frame.data[4] = (pitch_int >> 8) & 0xFF;  // Pitch高8位
  frame.data[5] = pitch_int & 0xFF;         // Pitch低8位

  try {
    can_->write(&frame);
  } catch (const std::exception & e) {
    tools::logger()->warn("[NewCAN] write failed: {}", e.what());
  }
  return;
}

void CBoard::callback(const can_frame & frame)
{
  // 解析下位机发送的新协议（由 UpdateControl/下位机发送）
  // 发送端布局（UpdateControl）：
  // byte0-1: w (f16, big-endian)
  // byte2-3: x (f16, big-endian)
  // byte4-5: y (f16, big-endian)
  // byte6: [bit7:z_sign][bit6:id_bit][bit5-4:mode(2bit)][bit3-0:imu_low4]
  // byte7: bullet_speed (8bit quantized -> 0..32)

  auto timestamp = std::chrono::steady_clock::now();
  if (frame.can_id != control_canid_) return;
  if (frame.can_dlc < 8) return;

  using f16 = f16tools::f16;
  auto be_to_u16 = [](const uint8_t * p) -> f16 {
    return static_cast<f16>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
  };

  f16 w16 = be_to_u16(&frame.data[0]);
  f16 x16 = be_to_u16(&frame.data[2]);
  f16 y16 = be_to_u16(&frame.data[4]);

  f32 qw = f16tools::f16_to_f32(w16);
  f32 qx = f16tools::f16_to_f32(x16);
  f32 qy = f16tools::f16_to_f32(y16);

  const uint8_t byte6 = frame.data[6];
  const uint8_t z_sign = (byte6 & 0x80) ? 1 : 0;
  const uint8_t id_bit = (byte6 & 0x40) ? 1 : 0;
  const uint8_t mode_bits = (byte6 >> 4) & 0x3;  // 2 bits
  const uint8_t imu_low4 = byte6 & 0x0F;

  // 恢复 z 分量（使用 z_sign 指定正/负）
  f32 z_sq = 1.0f - (qw * qw + qx * qx + qy * qy);
  f32 qz = (z_sq > 0.0f) ? std::sqrt(z_sq) : 0.0f;
  if (!z_sign) qz = -qz;

  const uint8_t bs_raw = frame.data[7];
  f32 bullet_sp = static_cast<f32>(bs_raw) * (32.0f / 255.0f);
  const uint8_t slot = imu_low4 % IMU_RING_BUFFER_SIZE;

  imu_ring_buffer_[slot].q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
  imu_ring_buffer_[slot].imu_count_short = imu_low4;
  imu_ring_buffer_[slot].timestamp = timestamp;
  imu_ring_buffer_[slot].valid.store(true, std::memory_order_release);

  robot_id = id_bit ? 101 : 1;
  mode = static_cast<Mode>(mode_bits < MODES.size() ? mode_bits : 0);
  bullet_speed = bullet_sp;
}
uint8_t CBoard::compute_aimbotstate(bool control, bool fire) const
{
  // 位域实现（优先）：
  // bit0 = HAS_TARGET, bit1 = SUGGEST_FIRE, bit5 = SELF_AIM
  if (!serial_aimbotstate_enum_) {
    uint8_t bits = 0;
    if (control) bits |= AIMBOT_BIT_HAS_TARGET;
    // 当上位机认为应该开火或强制开火配置打开时，置建议开火位
    if (fire || serial_force_fire_when_control_) bits |= AIMBOT_BIT_SUGGEST_FIRE;
    // 保持兼容：把自瞄标志一并置位（旧逻辑中存在）
    if (control) bits |= AIMBOT_BIT_SELF_AIM;
    // 其它位保留为 0（固件未使用，预留扩展）
    return bits;
  }
  return 0;
}
}  // namespace io