#include "gimbal.hpp"

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"
namespace io {
Gimbal::Gimbal(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  // 协议选择与参数（可选）
  try {
    if (yaml["gimbal_protocol"]) {
      auto p = yaml["gimbal_protocol"].as<std::string>();
      use_scm_ = (p == "scm" || p == "SCM");
    }
    if (yaml["scm_sof"])
      scm_sof_ = static_cast<uint8_t>(yaml["scm_sof"].as<uint32_t>());
    if (yaml["scm_eof"])
      scm_eof_ = static_cast<uint8_t>(yaml["scm_eof"].as<uint32_t>());
    if (yaml["scm_rx_id"])
      scm_rx_id_ = static_cast<uint8_t>(yaml["scm_rx_id"].as<uint32_t>());
    if (yaml["scm_tx_id"])
      scm_tx_id_ = static_cast<uint8_t>(yaml["scm_tx_id"].as<uint32_t>());
    if (yaml["scm_angles_in_deg"])
      scm_angles_in_deg_ = yaml["scm_angles_in_deg"].as<bool>();
  } catch (...) {
  }
  // 读取波特率（优先使用 gimbal_baudrate，其次回退到顶层 baudrate），缺省为
  // 115200
  uint32_t baudrate = 328125;
  try {
    if (yaml["gimbal_baudrate"]) {
      baudrate = yaml["gimbal_baudrate"].as<uint32_t>();
    } else if (yaml["baudrate"]) {
      baudrate = yaml["baudrate"].as<uint32_t>();
    }
  } catch (...) {
    // ignore malformed value and keep default
  }

  try {
    serial_.setPort(com_port);
    // 配置串口参数：波特率、数据位、校验位、停止位、流控与读取超时
    serial_.setBaudrate(baudrate);
    serial_.setBytesize(serial::eightbits);
    serial_.setParity(serial::parity_none);
    serial_.setStopbits(serial::stopbits_one);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    auto timeout =
        serial::Timeout::simpleTimeout(50); // 50 ms 读超时，避免频繁误判
    serial_.setTimeout(timeout);
    serial_.open();
    tools::logger()->info("[Gimbal] Opened serial {} @ {} baud (protocol: {})",
                          com_port, baudrate, use_scm_ ? "SCM" : "SP+CRC16");
  } catch (const std::exception &e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal() {
  quit_ = true;
  if (thread_.joinable())
    thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const {
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

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t) {
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a)
      return q_c;
    if (!(t_a < t && t <= t_b))
      continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal) {
  if (use_scm_) {
    bool control = (VisionToGimbal.mode != 0);
    bool fire = (VisionToGimbal.mode == 2);
    send_scm(control, fire, VisionToGimbal.yaw, VisionToGimbal.yaw_vel,
             VisionToGimbal.yaw_acc, VisionToGimbal.pitch,
             VisionToGimbal.pitch_vel, VisionToGimbal.pitch_acc);
    return;
  }
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(reinterpret_cast<uint8_t *>(&tx_data_),
                                    sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception &e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}
void Gimbal::send(bool control, bool fire, float yaw, float yaw_vel,
                  float yaw_acc, float pitch, float pitch_vel,
                  float pitch_acc) {
  if (use_scm_) {
    send_scm(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc);
    return;
  }
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(reinterpret_cast<uint8_t *>(&tx_data_),
                                    sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception &e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t *buffer, size_t size) {
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception &e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread() {
  tools::logger()->info("[Gimbal] read_thread started.");
  start_tp_ = std::chrono::steady_clock::now();
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn(
          "[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    // SCM 协议分支：按整帧解析（失败时不累加错误，避免误触发重连）
    if (use_scm_) {
      if (!parse_scm_rx()) {
        // 轻微退让，避免忙等
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      } else {
        error_count = 0;
      }
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P')
      continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
              sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_),
                            sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2],
                         rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

    switch (rx_data_.mode) {
    case 0:
      mode_ = GimbalMode::IDLE;
      break;
    case 1:
      mode_ = GimbalMode::AUTO_AIM;
      break;
    case 2:
      mode_ = GimbalMode::SMALL_BUFF;
      break;
    case 3:
      mode_ = GimbalMode::BIG_BUFF;
      break;
    default:
      mode_ = GimbalMode::IDLE;
      tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
      break;
    }
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

// --- SCM 协议实现 ---
static inline float rad2deg(float r) { return r * 57.29577951308232f; }
static inline float deg2rad(float d) { return d * 0.017453292519943295f; }

void Gimbal::send_scm(bool control, bool fire, float yaw, float yaw_vel,
                      float yaw_acc, float pitch, float pitch_vel,
                      float pitch_acc) {
  uint8_t aimbot_state = 0; // 0:不控 1:控不火 2:控且火
  if (control)
    aimbot_state = fire ? 2 : 1;
  uint8_t aimbot_target = 0;

  float out_yaw = scm_angles_in_deg_ ? rad2deg(yaw) : yaw;
  float out_pitch = scm_angles_in_deg_ ? rad2deg(pitch) : pitch;
  float out_yaw_vel = scm_angles_in_deg_ ? rad2deg(yaw_vel) : yaw_vel;
  float out_pitch_vel = scm_angles_in_deg_ ? rad2deg(pitch_vel) : pitch_vel;
  float system_timer =
      std::chrono::duration<float>(std::chrono::steady_clock::now() - start_tp_)
          .count();
  
   
  AimbotFrame_SCM_t frame{};

  frame.SOF = scm_sof_;
  frame.SOF = 0x55;
  // frame.ID = scm_tx_id_;
  frame.ID = 0x02;
  frame.AimbotState = aimbot_state;
  frame.AimbotTarget = aimbot_target;
  frame.PitchRelativeAngle = out_pitch;
  frame.YawRelativeAngle = out_yaw;
  frame.TargetPitchSpeed = out_pitch_vel;
  frame.TargetYawSpeed = out_yaw_vel;
  
  frame.SystemTimer = static_cast<uint32_t>(  
      std::chrono::duration_cast<std::chrono::milliseconds>(  
          std::chrono::steady_clock::now() - start_tp_).count());  
  
  // frame.EOF = scm_eof_; 
  frame.EOF = 0xFF; 
  // frame.PitchRelativeAngle = frame.Pitch;
  // frame.YawRelativeAngle = frame.Yaw;
  
  try {
    serial_.write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
    // tools::logger()->info(
    //     "[Gimbal][SCM] tx: mode={}, yaw={}, yaw_vel={}, yaw_acc={}, pitch_vel={}, pitch_acc={}, system_timer={}",
    //     static_cast<int>(aimbot_state), static_cast<float>(out_yaw),
    //     static_cast<float>(out_yaw_vel), static_cast<float>(yaw_acc),
    //     static_cast<float>(out_pitch_vel), static_cast<float>(pitch_acc),
    //     static_cast<float>(system_timer));
  } catch (const std::exception &e) {
    tools::logger()->warn("[Gimbal][SCM] Failed to write serial: {}", e.what());
  }
}
void Gimbal::send_command_scm(io::Command command) {
  // TODO: Implement SCM command sending
  uint8_t aimbot_state = 2; // 0:不控 1:控不火 2:控且火
  if (command.control)
    aimbot_state = command.shoot ? 2 : 1;
  uint8_t aimbot_target = 0;
  float out_yaw = scm_angles_in_deg_ ? rad2deg(command.yaw) : command.yaw;
  float out_pitch =
      scm_angles_in_deg_ ? rad2deg(command.pitch) : command.pitch;
  float system_timer =
      std::chrono::duration<float>(std::chrono::steady_clock::now() - start_tp_)
          .count();

  AimbotFrame_SCM_t frame{};
  frame.SOF = scm_sof_;
  frame.ID = 0x03;
  frame.AimbotState = aimbot_state;
  frame.AimbotTarget = aimbot_target;
  frame.PitchRelativeAngle = out_pitch;
  frame.YawRelativeAngle = out_yaw;
  frame.TargetPitchSpeed = 0.0f;
  frame.TargetYawSpeed = 0.0f;
  frame.SystemTimer = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_tp_).count());
  frame.EOF = scm_eof_;

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&frame), sizeof(frame));
  } catch (const std::exception &e) {
    tools::logger()->warn("[Gimbal][SCM] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::parse_scm_rx() {
  Gimaballmurname_SCM_t rx{};
  // 寻找 SOF：逐字节直到匹配
  uint8_t b = 0;
  while (true) {
    if (!read(&b, 1))
      return false;
    if (b == scm_sof_)
      break;
  }
  rx.SOF = b;
  // 读剩余
  size_t remain = sizeof(rx) - 1;
  size_t got = 0;
  while (got < remain) {
    try {
      size_t n = serial_.read(reinterpret_cast<uint8_t *>(&rx) + 1 + got,
                              remain - got);
      if (n == 0) {
        // 本次无数据，到下轮再试
        return false;
      }
      got += n;
    } catch (const std::exception &) {
      return false;
    }
  }
  static uint32_t eof_err = 0, id_err = 0;
  if (rx.EOF != scm_eof_) {
    const uint32_t ts_copy = static_cast<uint32_t>(rx.TimeStamp);
    if ((++eof_err % 1000) == 1) {
      tools::logger()->warn(
          "[Gimbal][SCM] EOF mismatch: got {}, expect {} (id={}, ts={})",
          static_cast<int>(rx.EOF), static_cast<int>(scm_eof_),
          static_cast<int>(rx.ID), ts_copy);
    }
    return false;
  }
  if (rx.ID != scm_rx_id_) {
    const uint32_t ts_copy = static_cast<uint32_t>(rx.TimeStamp);
    if ((++id_err % 1000) == 1) {
      tools::logger()->warn(
          "[Gimbal][SCM] ID mismatch: got {}, expect {} (ts={})",
          static_cast<int>(rx.ID), static_cast<int>(scm_rx_id_), ts_copy);
    }
    return false;
  }

  Eigen::Quaterniond q(rx.q0, rx.q1, rx.q2, rx.q3);
  if (q.norm() > 1e-6)
    q.normalize();
  auto t = std::chrono::steady_clock::now();
  queue_.push({q, t});

  std::lock_guard<std::mutex> lock(mutex_);
  state_.yaw = 0;
  state_.yaw_vel = 0;
  state_.pitch = 0;
  state_.pitch_vel = 0;
  state_.bullet_speed = 0;
  state_.bullet_count = 0;
  // tools::logger()->info(
  //     "[Gimbal][SCM] rx: q1={}, q2={}, q3={}", static_cast<float>(rx.q1),
  //     static_cast<float>(rx.q2), static_cast<float>(rx.q3));
  switch (rx.mode) {
  case 0:
    mode_ = GimbalMode::IDLE;
    break;
  case 1:
    mode_ = GimbalMode::AUTO_AIM;
    break;
  case 2:
    mode_ = GimbalMode::SMALL_BUFF;
    break;
  case 3:
    mode_ = GimbalMode::BIG_BUFF;
    break;
  default:
    mode_ = GimbalMode::IDLE;
    break;
  }

  return true;
}

void Gimbal::reconnect() {
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...",
                          i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open(); // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception &e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

} // namespace io