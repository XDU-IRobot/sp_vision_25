#ifndef IO__CBOARD_HPP
#define IO__CBOARD_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cmath>
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

// 哨兵专有
enum ShootMode { left_shoot, right_shoot, both_shoot };
const std::vector<std::string> SHOOT_MODES = {"left_shoot", "right_shoot",
                                              "both_shoot"};

class CBoard {
public:
  // IMU数据结构（移到public以便外部访问）
  struct IMUData {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;  // 上位机接收时间戳
    uint32_t mcu_timestamp;  // 🆕 MCU发送时的时间戳（从0x160帧获取，单位：毫秒）
    uint16_t imu_count;      // 完整的IMU计数器：0-9999循环（从0x160帧获取）
    uint8_t cycle_count;     // IMU周期计数：1-10循环
  };

  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle; //无人机专有

  CBoard(const std::string &config_path);
  ~CBoard();

  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  // 🆕 基于imu_count的硬同步查找（推荐使用）
  // 从队列中查找指定imu_count的IMU数据
  // 参数：target_count - 目标IMU计数器值（0-9999）
  // 返回：找到的IMU四元数，如果找不到则等待或使用最近的数据
  Eigen::Quaterniond imu_by_count(uint16_t target_count);

  // 🆕 获取最近一次 imu_by_count() 匹配的 IMU 数据时间戳
  // 注意：必须在调用 imu_by_count() 后立即调用，返回上次匹配的 IMU 时间戳
  std::chrono::steady_clock::time_point get_last_matched_imu_timestamp() const {
    // imu_by_count() 会更新 data_ahead_ 或 data_behind_
    // 返回 data_behind_ 的时间戳（最近匹配的数据）
    return data_behind_.timestamp;
  }

  // 🆕 获取最近一个完整的IMU周期（10帧）
  // 返回count从1到10的完整序列，如果不完整则返回空
  std::vector<Eigen::Quaterniond> get_last_imu_cycle();

  // 🆕 获取最近一个完整IMU周期的中间帧（第5或第6帧）
  Eigen::Quaterniond get_last_imu_cycle_middle();

  // 🆕 获取最近的IMU数据（包含四元数、时间戳和imu_count）
  IMUData get_last_imu_data() const { return data_behind_; }

  // 🆕 获取当前的imu_count
  uint16_t get_imu_count() const { return imu_count_; }

  // 🆕 环形数组直接查询接口（推荐使用）
  // 根据imu_count直接从环形数组中查询IMU数据
  // 参数：target_imu_count - 目标IMU计数器值（0-9999）
  // 返回：{q, mcu_timestamp, mcu_synced_timestamp, rx_timestamp, valid}
  // 注意：mcu_synced_timestamp 是转换后的 MCU 时间戳，应作为后续解算的唯一时间基准
  struct IMUQueryResult {
    Eigen::Quaterniond q;
    uint32_t mcu_timestamp;          // MCU 原始时间戳（毫秒，用于调试）
    std::chrono::steady_clock::time_point mcu_synced_timestamp;  // 🔑 转换后的 MCU 时间戳（主时间基准）
    std::chrono::steady_clock::time_point rx_timestamp;  // 上位机接收时间戳（仅供参考）
    bool valid;
  };
  IMUQueryResult get_imu_from_ring_buffer(uint16_t target_imu_count) const;

  void send(Command command);

  // 🆕 启动相机触发信号（在程序完全初始化后调用）
  void start_camera_trigger();

  // 🆕 根据robot_id获取敌方颜色
  // robot_id = 0: 己方蓝色，击打红色
  // robot_id = 1: 己方红色，击打蓝色
  // 返回值需要包含auto_aim::Color枚举，因此这里返回int（0=red, 1=blue）
  int get_enemy_color() const;

  // 🆕 获取当前的robot_id
  uint8_t get_robot_id() const { return robot_id_; }

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

  tools::ThreadSafeQueue<IMUData>
      queue_; // 必须在can_之前初始化，否则存在死锁的可能

  // 🆕 环形数组：用于高效存储和查询IMU数据
  static constexpr size_t IMU_RING_BUFFER_SIZE = 2000;  // 环形数组大小：足够容纳约4秒@500Hz的数据

  struct IMUFrame {
    uint16_t imu_count;       // IMU计数器（0-9999循环）
    uint32_t mcu_timestamp;   // MCU时间戳（毫秒，原始值，用于调试）
    std::chrono::steady_clock::time_point mcu_synced_timestamp;  // 🔑 转换后的MCU时间戳（主时间基准）
    std::chrono::steady_clock::time_point rx_timestamp;  // 上位机接收时间戳（仅供参考）
    Eigen::Quaterniond q;     // 四元数
    std::atomic<bool> valid;  // 数据有效性标志（原子操作，保证线程安全）

    IMUFrame() : imu_count(0), mcu_timestamp(0), q(1, 0, 0, 0), valid(false) {}
  };

  IMUFrame imu_ring_buffer_[IMU_RING_BUFFER_SIZE];  // 环形数组

  // 🆕 MCU 时间基准映射（用于将 MCU 时间戳转换为上位机 steady_clock）
  std::atomic<bool> time_base_initialized_{false};  // 时间基准是否已初始化
  uint32_t mcu_time_base_ = 0;                      // MCU 时间基准（毫秒）
  std::chrono::steady_clock::time_point host_time_base_;  // 上位机时间基准

  // 🆕 Pending缓存：用于临时存储未绑定的四元数（0x150帧先到的情况）
  Eigen::Quaterniond pending_q_;                     // 待绑定的四元数
  std::chrono::steady_clock::time_point pending_q_rx_timestamp_;  // 待绑定的四元数接收时间
  std::atomic<bool> quaternion_ready_{false};        // 四元数是否就绪（原子操作）

  // 🆕 帧配对缓存结构（保留向后兼容，但优先使用环形数组）
  struct PendingQuatFrame {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point rx_timestamp;  // 上位机接收时间
  };

  struct PendingStatusFrame {
    uint8_t robot_id;
    uint8_t mode;
    uint16_t imu_count;
    uint32_t mcu_timestamp;  // MCU发送的时间戳
    std::chrono::steady_clock::time_point rx_timestamp;  // 上位机接收时间
  };

  std::deque<PendingQuatFrame> pending_quat_frames_;      // 未匹配的0x150帧队列（向后兼容）
  std::deque<PendingStatusFrame> pending_status_frames_;  // 未匹配的0x160帧队列（向后兼容）
  std::mutex frame_match_mutex_;  // 保护帧配对缓存的互斥锁
  const size_t max_pending_frames_ = 10;  // 最大缓存帧数
  const double frame_match_time_window_ms_ = 10.0;  // 帧配对时间窗口（毫秒）

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
  std::atomic<bool> mcu_online_{false};  // MCU是否在线（imu_count!=0时为true）

  // 🆕 心跳线程（用于硬触发模式下在MCU上线前持续发送start=1）
  std::thread heartbeat_thread_;             // 心跳线程
  std::atomic<bool> heartbeat_quit_{false};  // 心跳线程退出标志
  int heartbeat_interval_ms_ = 2;            // 心跳间隔（毫秒），默认2ms=500Hz

  // 🆕 主循环发送监测（用于智能切换心跳/正常数据发送）
  std::atomic<int64_t> last_send_timestamp_ns_{0};  // 最后一次send()调用的时间戳（纳秒）
  int64_t heartbeat_takeover_timeout_ms_ = 100;     // 心跳接管超时时间（毫秒）：超过此时间未调用send()，心跳线程接管

  // 🆕 调试开关配置
  bool debug_rx_ = false;                // 是否输出RX（接收）调试信息
  bool debug_tx_ = false;                // 是否输出TX（发送）调试信息
  bool debug_frame_match_ = false;       // 是否输出frame匹配调试信息

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

  // 🆕 心跳线程函数（在MCU上线前持续发送start=1心跳）
  void heartbeat_loop();

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