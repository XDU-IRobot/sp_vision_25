#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
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

// 哨兵专有
enum ShootMode { left_shoot, right_shoot, both_shoot };
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot",
                                              "both_shoot"};

class CBoard {
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle; //无人机专有

  CBoard(const std::string &config_path);
  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  void send(Command command);

private:
  struct IMUData {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  tools::ThreadSafeQueue<IMUData>
      queue_; // 必须在can_之前初始化，否则存在死锁的可能
  // 传输后端：CAN（SocketCAN）或 SERIAL（USB 串口）
  std::unique_ptr<SocketCAN> can_;
  serial::Serial serial_;
  std::thread serial_thread_;
  std::atomic<bool> serial_quit_{false};
  bool use_serial_ = false;
  IMUData data_ahead_;
  IMUData data_behind_;

  int quaternion_canid_, bullet_speed_canid_, send_canid_;
  // 串口帧配置（可与 CAN ID 对应，或独立配置）
  std::string serial_port_ = "/dev/ttyACM0";
  uint32_t serial_baudrate_ = 115200;
  uint32_t serial_timeout_ms_ = 20;
  uint8_t serial_sof_ = 0x55;
  uint8_t serial_eof_ = 0xFF;
  uint8_t serial_id_quat_ = 0x10;
  uint8_t serial_id_status_ = 0x11;
  uint8_t serial_id_cmd_ = 0x12;
  bool serial_skip_crc_ = true;
  bool serial_debug_hex_ = false;
  // SCM 固定帧协议（可选）：当启用时，解析 Gimaballmurname_SCM_t 固定长度帧
  bool serial_protocol_scm_ = false;
  uint8_t serial_scm_rx_id_ = 0x01; // 电控->自瞄帧 ID
  uint8_t serial_scm_tx_id_ = 0x02; // 自瞄->电控帧 ID（AimbotFrame_SCM_t）
  bool serial_scm_angles_in_deg_ = true; // 角度单位是否用度
  std::chrono::steady_clock::time_point start_tp_;

  void callback(const can_frame &frame);
  void serial_read_loop();
  void handle_serial_frame(uint8_t id, const uint8_t *payload, size_t len);
  void send_scm(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                float pitch, float pitch_vel, float pitch_acc);

  std::string read_yaml(const std::string &config_path);
};

} // namespace io

#endif // IO__CBOARD_HPP