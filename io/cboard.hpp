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

  // ===== 新CAN协议配置 =====
  bool use_new_can_protocol_ = false;    // 是否使用新CAN协议
  int new_can_quat_id_ = 0x150;          // 新协议：四元数帧ID
  int new_can_status_id_ = 0x160;        // 新协议：状态帧ID
  int new_can_cmd_id_ = 0x170;           // 新协议：命令帧ID（上位机→下位机）
  uint8_t robot_id_ = 0;                 // 机器人ID（从0x160帧接收）
  uint16_t imu_count_ = 0;               // IMU计数器（从0x160帧接收）
  uint16_t last_imu_count_ = 0;          // 上次IMU计数器（用于检测丢帧）
  bool nuc_start_flag_sent_ = false;     // NUC启动标志是否已发送

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
  // 日志方向开关：分别控制 RX(电控->自瞄) 与 TX(自瞄->电控) 的调试输出
  bool serial_log_rx_ = true;
  bool serial_log_tx_ = true;
  // SCM 固定帧协议（可选）：当启用时，解析 Gimaballmurname_SCM_t 固定长度帧
  bool serial_protocol_scm_ = false;
  uint8_t serial_scm_rx_id_ = 0x01; // 电控->自瞄帧 ID
  uint8_t serial_scm_tx_id_ = 0x02; // 自瞄->电控帧 ID（AimbotFrame_SCM_t）
  bool serial_scm_angles_in_deg_ = true; // 角度单位是否用度
  // 目标类型位默认值（按位 bitmask），上层未指定时使用
  uint8_t serial_scm_default_target_ = 0x00;
  // 兼容开关：当 control=true 时强制置 BIT1（可打击）
  bool serial_force_fire_when_control_ = false;
  // 联调辅助：无论检测是否有目标，强制按“有目标”发送（置 AimbotState.bit0）
  bool serial_force_control_ = false;
  // Aimbotstate 编码方式：false=bitfield（默认），true=enum(0/1/2)
  bool serial_aimbotstate_enum_ = false;
  std::chrono::steady_clock::time_point start_tp_;

  // Aimbotstate 位定义（位域模式）：
  // bit0 (0x01) = HAS_TARGET: 上位机检测到目标，电控可据此接管/使用 NUC 提供的目标角度
  // bit1 (0x02) = SUGGEST_FIRE: 建议开火（供电控本地决策参考）
  // bit5 (0x20) = SELF_AIM: 自瞄标志（与历史实现兼容，原代码把此位作为自瞄标记）
  // bits 2..4,6..7 保留（目前固件未使用，可未来扩展）
  static constexpr uint8_t AIMBOT_BIT_HAS_TARGET = 0x01; // bit0
  static constexpr uint8_t AIMBOT_BIT_SUGGEST_FIRE = 0x02; // bit1
  static constexpr uint8_t AIMBOT_BIT_SELF_AIM = 0x20; // bit5

  // 云台绝对角零位偏置（弧度），用于将 IMU 欧拉角对齐到“云台绝对角”定义
  double gimbal_yaw_offset_rad_ = 0.0;
  double gimbal_pitch_offset_rad_ = 0.0;
  // 发送侧细调偏置（弧度），在所有协议（SCM/RAW/CAN）上叠加到上层给定的相对角
  double tx_yaw_bias_rad_ = 0.0;
  double tx_pitch_bias_rad_ = 0.0;
  // 发送侧滤波/限速（减少抖动），在绝对角上生效（单位：弧度/秒）
  bool tx_filter_enable_ = true;         // 可总开关
  double tx_ema_alpha_ = 0.0;            // EMA 系数，[0,1)，0 表示不启用 EMA；越大越平滑
  double tx_yaw_rate_limit_rad_s_ = 0.0; // 0 表示不限制
  double tx_pitch_rate_limit_rad_s_ = 0.0;
  // 欧拉角提取与符号设置
  bool gimbal_pitch_from_x_ = false; // false: 从Y轴取pitch（默认）；true: 从X轴取pitch
  int yaw_sign_ = 1;                 // 允许根据坐标系翻转符号（+1或-1）
  int pitch_sign_ = 1;               // 允许根据坐标系翻转符号（+1或-1）
  bool normalize_abs_angles_ = true; // 是否把绝对角归一化到 (-pi, pi]

  // 发送侧用于估算角速度的上次状态
  bool tx_has_last_ = false;
  float tx_last_yaw_ = 0.f;
  float tx_last_pitch_ = 0.f;
  std::chrono::steady_clock::time_point tx_last_tp_{};

  // 对将要发送的绝对角做滤波/限速（就地修改 yaw/pitch，单位：弧度）
  void filter_tx_angles(double &yaw_abs, double &pitch_abs);

  // 计算 Aimbotstate（根据配置选择 enum 或 bitfield）
  uint8_t compute_aimbotstate(bool control, bool fire);

  void callback(const can_frame &frame);
  void serial_read_loop();
  void handle_serial_frame(uint8_t id, const uint8_t *payload, size_t len);
  void send_scm(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                float pitch, float pitch_vel, float pitch_acc);

  // 发送启动帧（新CAN协议），通知电控上位机已启动
  void send_startup_frame();

  std::string read_yaml(const std::string &config_path);
};

} // namespace io

#endif // IO__CBOARD_HPP