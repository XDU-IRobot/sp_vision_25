#include "cboard.hpp"

#include "tools/math_tools.hpp"
#include "tools/crc.hpp"
#include "tools/yaml.hpp"

namespace io {
CBoard::CBoard(const std::string &config_path)
    : mode(Mode::idle), shoot_mode(ShootMode::left_shoot), bullet_speed(0),
      queue_(5000)
// 注意: callback的运行会早于Cboard构造函数的完成
{
  auto transport = read_yaml(config_path);

  if (transport == "serial") {
    use_serial_ = true;
    try {
      serial_.setPort(serial_port_);
      serial_.setBaudrate(serial_baudrate_);
      serial_.setFlowcontrol(serial::flowcontrol_none);
      serial_.setParity(serial::parity_none);
      serial_.setStopbits(serial::stopbits_one);
      serial_.setBytesize(serial::eightbits);
      auto timeout = serial::Timeout::simpleTimeout(serial_timeout_ms_);
      serial_.setTimeout(timeout);
      serial_.open();
      tools::logger()->info("[Cboard] Serial opened {} @ {} baud", serial_port_,
                            serial_baudrate_);
  start_tp_ = std::chrono::steady_clock::now();
      serial_quit_ = false;
      serial_thread_ = std::thread(&CBoard::serial_read_loop, this);
    } catch (const std::exception &e) {
      tools::logger()->warn("[Cboard] Serial open failed: {}", e.what());
      throw;
    }
  } else {
    // 默认使用 CAN
    can_ = std::make_unique<SocketCAN>(
        transport, std::bind(&CBoard::callback, this, std::placeholders::_1));
  }

  tools::logger()->info("[Cboard] Waiting for q...");
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[Cboard] Opened.");
}

Eigen::Quaterniond
CBoard::imu_at(std::chrono::steady_clock::time_point timestamp) {
  if (data_behind_.timestamp < timestamp)
    data_ahead_ = data_behind_;

  while (true) {
    queue_.pop(data_behind_);
    if (data_behind_.timestamp > timestamp)
      break;
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

void CBoard::send(Command command) {
  if (use_serial_) {
    if (serial_protocol_scm_) {
      // 走 SCM 固定帧 AimbotFrame_SCM_t，无 CRC
      float yaw = static_cast<float>(command.yaw);
      float pitch = static_cast<float>(command.pitch);
      // 在当前实现中暂不输出速度/加速度（可扩展），默认置 0
      send_scm(command.control, command.shoot, yaw, 0.f, 0.f, pitch, 0.f, 0.f);
      return;
    } else {
      // RAW 可变帧（无 CRC）：[SOF][ID][LEN=8][payload][EOF]
      uint8_t buf[1 + 1 + 1 + 8 + 1];
      size_t idx = 0;
      buf[idx++] = serial_sof_;
      buf[idx++] = serial_id_cmd_;
      buf[idx++] = 8;
      buf[idx++] = command.control ? 1 : 0;
      buf[idx++] = command.shoot ? 1 : 0;
      int16_t yaw_i = static_cast<int16_t>(command.yaw * 1e4);
      int16_t pitch_i = static_cast<int16_t>(command.pitch * 1e4);
      int16_t dist_i = static_cast<int16_t>(command.horizon_distance * 1e4);
      buf[idx++] = (yaw_i >> 8) & 0xFF;
      buf[idx++] = yaw_i & 0xFF;
      buf[idx++] = (pitch_i >> 8) & 0xFF;
      buf[idx++] = pitch_i & 0xFF;
      buf[idx++] = (dist_i >> 8) & 0xFF;
      buf[idx++] = dist_i & 0xFF;
      buf[idx++] = serial_eof_;
      try {
        serial_.write(reinterpret_cast<const uint8_t *>(buf), idx);
      } catch (const std::exception &e) {
        tools::logger()->warn("[Cboard][SERIAL] write failed: {}", e.what());
      }
      return;
    }
  } else {
    can_frame frame;
    frame.can_id = send_canid_;
    frame.can_dlc = 8;
    frame.data[0] = (command.control) ? 1 : 0;
    frame.data[1] = (command.shoot) ? 1 : 0;
    frame.data[2] = (int16_t)(command.yaw * 1e4) >> 8;
    frame.data[3] = (int16_t)(command.yaw * 1e4);
    frame.data[4] = (int16_t)(command.pitch * 1e4) >> 8;
    frame.data[5] = (int16_t)(command.pitch * 1e4);
    frame.data[6] = (int16_t)(command.horizon_distance * 1e4) >> 8;
    frame.data[7] = (int16_t)(command.horizon_distance * 1e4);

    try {
      can_->write(&frame);
    } catch (const std::exception &e) {
      tools::logger()->warn("{}", e.what());
    }
  }
}

CBoard::~CBoard() {
  if (use_serial_) {
    serial_quit_ = true;
    if (serial_thread_.joinable())
      serial_thread_.join();
    try {
      serial_.close();
    } catch (...) {
    }
  }
}

void CBoard::callback(const can_frame &frame) {
  auto timestamp = std::chrono::steady_clock::now();

  if (frame.can_id == quaternion_canid_) {
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp});
  }

  else if (frame.can_id == bullet_speed_canid_) {
    bullet_speed = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e2;
    mode = Mode(frame.data[2]);
    shoot_mode = ShootMode(frame.data[3]);
    ft_angle = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;

    // 限制日志输出频率为1Hz
    static auto last_log_time = std::chrono::steady_clock::time_point::min();
    auto now = std::chrono::steady_clock::now();

    if (bullet_speed > 0 && tools::delta_time(now, last_log_time) >= 1.0) {
      tools::logger()->info("[CBoard] Bullet speed: {:.2f} m/s, Mode: {}, "
                            "Shoot mode: {}, FT angle: {:.2f} rad",
                            bullet_speed, MODES[mode], SHOOT_MODES[shoot_mode],
                            ft_angle);
      last_log_time = now;
    }
  }
}

// 实现方式有待改进
std::string CBoard::read_yaml(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  // 传输后端选择
  std::string transport = "can";
  if (yaml["cboard_transport"]) {
    transport = yaml["cboard_transport"].as<std::string>();
  }

  if (transport == "serial") {
    if (yaml["cboard_serial_port"])
      serial_port_ = yaml["cboard_serial_port"].as<std::string>();
    if (yaml["cboard_serial_baudrate"])
      serial_baudrate_ = yaml["cboard_serial_baudrate"].as<uint32_t>();
    if (yaml["cboard_serial_timeout_ms"])
      serial_timeout_ms_ = yaml["cboard_serial_timeout_ms"].as<uint32_t>();
    if (yaml["cboard_serial_sof"])
      serial_sof_ =
          static_cast<uint8_t>(yaml["cboard_serial_sof"].as<uint32_t>());
    if (yaml["cboard_serial_eof"])
      serial_eof_ =
          static_cast<uint8_t>(yaml["cboard_serial_eof"].as<uint32_t>());
    if (yaml["cboard_serial_id_quat"])
      serial_id_quat_ =
          static_cast<uint8_t>(yaml["cboard_serial_id_quat"].as<uint32_t>());
    if (yaml["cboard_serial_id_status"])
      serial_id_status_ =
          static_cast<uint8_t>(yaml["cboard_serial_id_status"].as<uint32_t>());
    if (yaml["cboard_serial_id_cmd"])
      serial_id_cmd_ =
          static_cast<uint8_t>(yaml["cboard_serial_id_cmd"].as<uint32_t>());
    if (yaml["cboard_serial_skip_crc"])
      serial_skip_crc_ = yaml["cboard_serial_skip_crc"].as<bool>();
    if (yaml["cboard_serial_debug_hex"])
      serial_debug_hex_ = yaml["cboard_serial_debug_hex"].as<bool>();
    if (yaml["cboard_serial_log_rx"]) serial_log_rx_ = yaml["cboard_serial_log_rx"].as<bool>();
    if (yaml["cboard_serial_log_tx"]) serial_log_tx_ = yaml["cboard_serial_log_tx"].as<bool>();
    if (yaml["cboard_serial_protocol"]) {
      auto p = yaml["cboard_serial_protocol"].as<std::string>();
      serial_protocol_scm_ = (p == "scm" || p == "SCM");
    }
    if (yaml["cboard_serial_scm_rx_id"]) {
      serial_scm_rx_id_ = static_cast<uint8_t>(yaml["cboard_serial_scm_rx_id"].as<uint32_t>());
    }
    if (yaml["cboard_serial_scm_tx_id"]) {
      serial_scm_tx_id_ = static_cast<uint8_t>(yaml["cboard_serial_scm_tx_id"].as<uint32_t>());
    }
    if (yaml["cboard_serial_angles_in_deg"]) {
      serial_scm_angles_in_deg_ = yaml["cboard_serial_angles_in_deg"].as<bool>();
    }
  tools::logger()->info(
    "[Cboard] Serial config: CRC {} (skip_crc={}), SOF=0x{:02X}, EOF=0x{:02X}, id_quat=0x{:02X}, id_status=0x{:02X}, protocol={} rx_id=0x{:02X} tx_id=0x{:02X} angles_in_deg={} log_rx={} log_tx={}",
    serial_skip_crc_ ? "disabled" : "enabled", serial_skip_crc_, serial_sof_, serial_eof_, serial_id_quat_,
    serial_id_status_, serial_protocol_scm_ ? "SCM" : "RAW", serial_scm_rx_id_, serial_scm_tx_id_, serial_scm_angles_in_deg_, serial_log_rx_, serial_log_tx_);
    return "serial";
  }

  // CAN 模式：读取 CAN 相关配置
  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }
  return yaml["can_interface"].as<std::string>();
}

void CBoard::serial_read_loop() {
  std::vector<uint8_t> buf;
  buf.reserve(256);
  while (!serial_quit_) {
    try {
      // 读入数据
      uint8_t b;
      size_t n = serial_.read(&b, 1);
      if (n == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      buf.push_back(b);

      // 对齐到 SOF
      while (!buf.empty() && buf.front() != serial_sof_) {
        buf.erase(buf.begin());
      }

      if (buf.size() < 2)
        continue; // 需要至少 SOF + ID

      uint8_t id = buf[1];

      if (serial_protocol_scm_) {
        // 固定长度帧：Gimaballmurname_SCM_t（共 25 字节）
        const size_t need = 25;
        if (buf.size() < need)
          continue;
        if (id != serial_scm_rx_id_) {
          // 非目标 ID，丢弃一个字节后重试
          buf.erase(buf.begin());
          continue;
        }
        if (buf[need - 1] != serial_eof_) {
          if (serial_debug_hex_) {
            std::string hex;
            for (size_t i = 0; i < std::min(buf.size(), need); ++i)
              hex += fmt::format("{:02X} ", buf[i]);
            tools::logger()->warn(
                "[Cboard][SERIAL][SCM] EOF mismatch, drop head: {}", hex);
          }
          buf.erase(buf.begin());
          continue;
        }
        // 解析小端帧
        auto timestamp = std::chrono::steady_clock::now();
        if (need <= buf.size()) {
          // 结构：SOF(0) ID(1) TimeStamp(2..5) q0..q3(6..21) robot_id(22) mode(23) EOF(24)
          float q0, q1, q2, q3;
          std::memcpy(&q0, buf.data() + 6, 4);
          std::memcpy(&q1, buf.data() + 10, 4);
          std::memcpy(&q2, buf.data() + 14, 4);
          std::memcpy(&q3, buf.data() + 18, 4);
          Eigen::Quaterniond q(q0, q1, q2, q3);
          if (q.norm() > 1e-6)
            q.normalize();
          queue_.push({q, timestamp});
          // 模式字段（按原样映射）
          uint8_t robot_id = buf[22];
          uint8_t mode_byte = buf[23];
          mode = static_cast<Mode>(mode_byte);
          if (serial_debug_hex_ && serial_log_rx_) {
            std::string hex;
            for (size_t i = 0; i < need; ++i)
              hex += fmt::format("{:02X} ", buf[i]);
            tools::logger()->info(
                "[Cboard][SERIAL][SCM] frame ok: {} | q=({:.3f},{:.3f},{:.3f},{:.3f}) robot_id={} mode={}",
                hex, q.w(), q.x(), q.y(), q.z(), static_cast<int>(robot_id), static_cast<int>(mode));
          }
          buf.erase(buf.begin(), buf.begin() + need);
        }
        continue;
      }

      // 原有可变长度帧：[SOF][ID][LEN][PAYLOAD][CRC?][EOF]
      if (buf.size() < 3)
        continue;
      uint8_t len = buf[2];
      size_t need = 1 + 1 + 1 + len + (serial_skip_crc_ ? 0 : 2) + 1;
      if (buf.size() < need)
        continue;
      if (buf[need - 1] != serial_eof_) {
        if (serial_debug_hex_) {
          std::string hex;
          for (size_t i = 0; i < std::min(buf.size(), need); ++i)
            hex += fmt::format("{:02X} ", buf[i]);
          tools::logger()->warn("[Cboard][SERIAL] EOF mismatch, drop head: {}",
                                hex);
        }
        buf.erase(buf.begin());
        continue;
      }
      const uint8_t *payload = buf.data() + 3;
      handle_serial_frame(id, payload, len);
      if (serial_debug_hex_) {
        std::string hex;
        for (size_t i = 0; i < need; ++i)
          hex += fmt::format("{:02X} ", buf[i]);
        tools::logger()->info("[Cboard][SERIAL] frame ok: {}", hex);
      }
      buf.erase(buf.begin(), buf.begin() + need);
    } catch (const std::exception &e) {
      tools::logger()->warn("[Cboard][SERIAL] read failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

static inline float rad2deg(float r) { return r * 57.29577951308232f; }

void CBoard::send_scm(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                      float pitch, float pitch_vel, float pitch_acc)
{
  // AimbotFrame_SCM_t（packed）
  struct __attribute__((packed)) AimbotFrame_SCM_t {
    uint8_t SOF;
    uint8_t ID;
    // 按电控约定命名：Aimbotstate=0 不控，1 控不火，2 控且火
    uint8_t Aimbotstate;
    uint8_t AimbotTarget;
    float Pitch;
    float Yaw;
    float TargetPitchSpeed;
    float TargetYawSpeed;
    float SystemTimer;
    uint8_t _EOF; // 注意：MCU 协议要求 EOF 在相对角之后的前面
    float PitchRelativeAngle;
    float YawRelativeAngle;
  } frame{};

  frame.SOF = serial_sof_;
  frame.ID = serial_scm_tx_id_;
  // AimbotState 按位：BIT0 已识别到；BIT1 可以打击；BIT5 自瞄模式
  uint8_t aimbot_state = 0;
  if (control) {
    aimbot_state |= (1u << 0);            // 已识别到
    aimbot_state |= (1u << 5);            // 自瞄模式
    if (fire) aimbot_state |= (1u << 1);  // 可以打击
  }
  frame.Aimbotstate = aimbot_state;
  // 目标类型位：默认未知，置 0；如需按识别类别设位可在上层赋值
  frame.AimbotTarget = 0u;

  float out_yaw = serial_scm_angles_in_deg_ ? rad2deg(yaw) : yaw;
  float out_pitch = serial_scm_angles_in_deg_ ? rad2deg(pitch) : pitch;
  float out_yaw_vel = serial_scm_angles_in_deg_ ? rad2deg(yaw_vel) : yaw_vel;
  float out_pitch_vel = serial_scm_angles_in_deg_ ? rad2deg(pitch_vel) : pitch_vel;
  float system_timer = std::chrono::duration<float>(
                        std::chrono::steady_clock::now() - start_tp_).count();

  frame.Pitch = out_pitch;
  frame.Yaw = out_yaw;
  frame.TargetPitchSpeed = out_pitch_vel;
  frame.TargetYawSpeed = out_yaw_vel;
  frame.SystemTimer = system_timer;
  frame._EOF = serial_eof_;
  frame.PitchRelativeAngle = frame.Pitch;
  frame.YawRelativeAngle = frame.Yaw;

  try {
    serial_.write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    if (serial_debug_hex_ && serial_log_tx_) {
      tools::logger()->info(
        "[Cboard][SCM][TX] id=0x{:02X} state=0b{:08b} target=0b{:08b} yaw={:.3f} pitch={:.3f} EOF=0x{:02X}",
        static_cast<unsigned>(frame.ID), static_cast<unsigned>(frame.Aimbotstate), static_cast<unsigned>(frame.AimbotTarget), out_yaw, out_pitch, static_cast<unsigned>(frame._EOF));
    }
  } catch (const std::exception &e) {
    tools::logger()->warn("[Cboard][SCM] write failed: {}", e.what());
  }
}

void CBoard::handle_serial_frame(uint8_t id, const uint8_t *payload,
                                 size_t len) {
  auto timestamp = std::chrono::steady_clock::now();
  if (id == serial_id_quat_ && len >= 8) {
    Eigen::Quaterniond q;
    if (len >= 16) {
      // 4个 float 序列化（小端）
      float fx, fy, fz, fw;
      std::memcpy(&fx, payload + 0, 4);
      std::memcpy(&fy, payload + 4, 4);
      std::memcpy(&fz, payload + 8, 4);
      std::memcpy(&fw, payload + 12, 4);
      q = Eigen::Quaterniond(fw, fx, fy, fz);
    } else {
      // 兼容与 CAN 相同的缩放：每 2 字节一个 int16，/1e4（大端序）
      auto s16 = [&](int i) {
        return static_cast<int16_t>(payload[i] << 8 | payload[i + 1]);
      };
      double x = s16(0) / 1e4, y = s16(2) / 1e4, z = s16(4) / 1e4,
             w = s16(6) / 1e4;
      q = Eigen::Quaterniond(w, x, y, z);
    }
    if (q.norm() > 1e-6)
      q.normalize();
    // 简单有效性检查
    if (std::isnan(q.w()) || std::isnan(q.x()) || std::isnan(q.y()) ||
        std::isnan(q.z())) {
      return;
    }
    queue_.push({q, timestamp});
    // 1Hz 心跳日志，确认有数据到达
    static auto last = std::chrono::steady_clock::time_point::min();
    if (tools::delta_time(timestamp, last) > 1.0) {
      tools::logger()->info(
          "[Cboard] q recv: wxyzn=({:.3f},{:.3f},{:.3f},{:.3f})", q.w(), q.x(),
          q.y(), q.z());
      last = timestamp;
    }
  } else if (id == serial_id_status_ && len >= 4) {
    bullet_speed = static_cast<int16_t>(payload[0] << 8 | payload[1]) / 1e2;
    mode = Mode(payload[2]);
    shoot_mode = ShootMode(payload[3]);
  }
}

} // namespace io