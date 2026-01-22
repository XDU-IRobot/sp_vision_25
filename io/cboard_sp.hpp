#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <sys/types.h>

#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "io/command.hpp"
#include "io/socketcan.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"
namespace io
{
enum Mode
{
  idle,
  auto_aim,
  small_buff,
  big_buff,
  outpost
};
enum ShootMode
{
  left_shoot,
  right_shoot,
  both_shoot
};
const std::vector<std::string> MODES = {"idle", "auto_aim", "small_buff", "big_buff", "outpost"};

struct IMUData
{
  Eigen::Quaterniond q;  //四元数 0x150
  std::chrono::steady_clock::time_point timestamp;
  uint8_t imu_count_short;  // 保存IMU计数器的低4位(0-15)
  ShootMode shoot_mode;
  std::atomic<bool> valid;  // 数据有效性标志
  IMUData() : q(1, 0, 0, 0), timestamp(), imu_count_short(0), valid(false) {}
};
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot", "both_shoot"};

class CBoard
{
public:
  double bullet_speed;
  Mode mode;
  double ft_angle;  //无人机专有
  uint8_t robot_id;
  CBoard(const std::string & config_path);

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);
  Eigen::Quaterniond imu_by_count(uint16_t target_count);  //查找对应帧的imu
  void send(Command command) const;

private:
  std::unique_ptr<SocketCAN> can_;
  uint16_t imu_count;
  uint32_t mcu_count;

  // tools::ThreadSafeQueue<IMUData> queue_;  // 必须在can_之前初始化，否则存在死锁的可能
  // SocketCAN can_;
  // IMUData data_ahead_;
  // IMUData data_behind_;
  static constexpr size_t IMU_RING_BUFFER_SIZE = 10;  //使用环形数组存取
  IMUData imu_ring_buffer_[IMU_RING_BUFFER_SIZE];     // 环形数组

  int control_canid_ = 0x150;
  int send_canid_ = 0x170;
  static constexpr uint8_t AIMBOT_BIT_HAS_TARGET = 0x01;    // bit0
  static constexpr uint8_t AIMBOT_BIT_SUGGEST_FIRE = 0x02;  // bit1
  static constexpr uint8_t AIMBOT_BIT_SELF_AIM = 0x20;      // bit5
  void callback(const can_frame & frame);
  uint8_t compute_aimbotstate(bool control, bool fire) const;
  std::string read_yaml(const std::string & config_path);
  bool serial_aimbotstate_enum_ = false;
  bool serial_force_fire_when_control_ = false;
};

}  // namespace io

#endif  // IO__CBOARD_HPP