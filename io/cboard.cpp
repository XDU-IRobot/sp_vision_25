#include "cboard.hpp"

#include "tools/float16.hpp"
#include "tools/math_tools.hpp"
#include "tools/crc.hpp"
#include "tools/yaml.hpp"
#include <cstdint>
#include <openvino/core/type/element_type.hpp>
#include "openvino/core/visibility.hpp"


// ROS2 TF2 支持头文件（需要在 namespace 之前包含）
#ifdef AMENT_CMAKE_FOUND
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#endif

namespace io {
CBoard::CBoard(const std::string &config_path)
    : mode(Mode::idle), shoot_mode(ShootMode::left_shoot), bullet_speed(0),
      queue_(5000)// 注意: callback的运行会早于Cboard构造函数的完成
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
      // 串口参数设置完成后，拉起 DTR/RTS 并启动读取线程
      // 某些下位机（CDC/STM32 等）只有在 DTR 为高时才开始收/发数据；主动拉起 DTR/RTS
      try {
        serial_.setDTR(true);
        serial_.setRTS(true);
        tools::logger()->info("[Cboard] Serial lines: DTR=1, RTS=1");
      } catch (const std::exception &e) {
        tools::logger()->warn("[Cboard] Failed to set DTR/RTS: {}", e.what());
      }
  start_tp_ = std::chrono::steady_clock::now();
      serial_quit_ = false;
      serial_thread_ = std::thread(&CBoard::serial_read_loop, this);
    } catch (const std::exception &e) {
      tools::logger()->warn("[Cboard] Serial open failed: {}", e.what());
      throw;
    }
  }

  else {
    // 默认使用 CAN——此处为同济实现的 SocketCAN，选择了保留这种接口
    can_ = std::make_unique<SocketCAN>(
        transport, std::bind(&CBoard::callback, this, std::placeholders::_1));
  }

  // 🆕 新CAN协议：不在构造函数中立即启动heartbeat
  // 等待程序完全初始化后，由外部调用 start_camera_trigger() 启动
  if (use_new_can_protocol_ && !use_serial_) {
    tools::logger()->info("[Cboard] CAN protocol ready. Waiting for start_camera_trigger() call...");
    // heartbeat将在start_camera_trigger()中启动
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

// 🆕 基于imu_count的硬同步查找
Eigen::Quaterniond CBoard::imu_by_count(uint16_t target_count) {
  // 确保target_count在有效范围内
  target_count = target_count % 10000;

  // 辅助函数：计算两个count之间的差值（考虑循环）
  auto count_diff = [](uint16_t a, uint16_t b) -> int64_t {
    int64_t diff = static_cast<int64_t>(a) - static_cast<int64_t>(b);
    if (diff < -5000) {
      diff += 10000;
    } else if (diff > 5000) {
      diff -= 10000;
    }
    return diff;
  };

  // 步骤1：检查data_behind_是否已经到达或超过目标count
  int64_t behind_diff = count_diff(data_behind_.imu_count, target_count);

  if (behind_diff < 0) {
    // data_behind_还没到达target_count，需要从队列读取更多数据
    data_ahead_ = data_behind_;

    while (true) {
      queue_.pop(data_behind_);
      int64_t new_diff = count_diff(data_behind_.imu_count, target_count);

      if (new_diff >= 0) {
        // 找到了 >= target_count的帧
        break;
      }
      data_ahead_ = data_behind_;
    }
  } else if (behind_diff > 0) {
    // data_behind_已经超过target_count
    // 这种情况说明目标帧已经被消费掉了，使用data_behind_作为最接近的数据
    tools::logger()->error(
      "❌ [IMU Sync] Target count {} already passed! current count is {}, diff=+{} frames. Using WRONG IMU data!",
      target_count, data_behind_.imu_count, behind_diff);
    // ⚠️ 这是错误的！使用了不匹配的 IMU 数据
    // 应该从历史缓冲区查找，但当前实现没有历史缓冲
    return data_behind_.q.normalized();
  }

  // 步骤2：检查是否精确匹配
  if (data_behind_.imu_count == target_count) {
    // 精确匹配，直接返回
    if (debug_frame_match_) {
      tools::logger()->info(
        "[IMU Sync] ✅ Exact match! target_count={}, found count={}, cycle={}",
        target_count, data_behind_.imu_count, data_behind_.cycle_count);
    }
    return data_behind_.q.normalized();
  }

  // 步骤3：data_behind_ > target_count，检查data_ahead_
  if (data_ahead_.imu_count == target_count) {
    // data_ahead_精确匹配
    if (debug_frame_match_) {
      tools::logger()->info(
        "[IMU Sync] ✅ Exact match (ahead)! target_count={}, found count={}, cycle={}",
        target_count, data_ahead_.imu_count, data_ahead_.cycle_count);
    }
    return data_ahead_.q.normalized();
  }

  // 步骤4：target_count在data_ahead_和data_behind_之间，使用时间戳插值
  // 这种情况通常不应该发生（因为IMU频率500Hz，相机50Hz，应该能精确匹配）
  // 但为了鲁棒性，还是做插值处理
  tools::logger()->warn(
    "[IMU Sync] Target count {} not found exactly, interpolating between {} and {}",
    target_count, data_ahead_.imu_count, data_behind_.imu_count);

  Eigen::Quaterniond q_a = data_ahead_.q.normalized();
  Eigen::Quaterniond q_b = data_behind_.q.normalized();
  auto t_a = data_ahead_.timestamp;
  auto t_b = data_behind_.timestamp;

  // 假设IMU频率恒定为500Hz（2ms周期），估算目标时间戳
  int64_t count_offset = count_diff(target_count, data_ahead_.imu_count);
  auto estimated_target_time = t_a + std::chrono::microseconds(count_offset * 2000);

  std::chrono::duration<double> t_ab = t_b - t_a;
  std::chrono::duration<double> t_ac = estimated_target_time - t_a;

  // 限制插值参数在[0, 1]范围内
  double k = std::clamp(t_ac / t_ab, 0.0, 1.0);
  Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

  return q_c;
}

void CBoard::send(Command command) {
  // 🆕 第一次调用send时停止心跳线程（主循环已开始工作）
  static bool first_send = true;
  if (first_send && heartbeat_thread_.joinable()) {
    heartbeat_quit_ = true;
    heartbeat_thread_.join();
    tools::logger()->info("[Cboard] Heartbeat thread stopped (main loop started)");
    first_send = false;
  }

  if (use_serial_) {
    if (serial_protocol_scm_) {
      // 走 SCM 固定帧 AimbotFrame_SCM_t，无 CRC
      float yaw = static_cast<float>(command.yaw + tx_yaw_bias_rad_);
      float pitch = static_cast<float>(command.pitch + tx_pitch_bias_rad_);
      // 在当前实现中暂不输出速度/加速度（可扩展），默认置 0
      bool force_ctrl = serial_force_control_ ? true : command.control;
      send_scm(force_ctrl, command.shoot, yaw, 0.f, 0.f, pitch, 0.f, 0.f);
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
      int16_t yaw_i = static_cast<int16_t>((command.yaw + tx_yaw_bias_rad_) * 1e4);
      int16_t pitch_i = static_cast<int16_t>((command.pitch + tx_pitch_bias_rad_) * 1e4);
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
    // ===== CAN通信 =====
    if (use_new_can_protocol_) {
      // 新CAN协议发送
      can_frame frame;
      frame.can_id = new_can_cmd_id_;
      frame.can_dlc = 7;  // 7字节数据

      // 🆕 start字段一直为1，表示NUC在线
      frame.data[6] = 1;

      // 检查MCU是否在线
      if (!mcu_online_) {
        // MCU未在线：发送空数据（其他字段全0），仅保持start=1
        frame.data[0] = 0;  // AimbotState = 0
        frame.data[1] = 0;  // AimbotTarget = 0
        frame.data[2] = 0;  // Yaw高字节 = 0
        frame.data[3] = 0;  // Yaw低字节 = 0
        frame.data[4] = 0;  // Pitch高字节 = 0
        frame.data[5] = 0;  // Pitch低字节 = 0

        try {
          can_->write(&frame);

          // 🆕 TX调试信息：心跳帧（当debug_tx_=true时每帧都打印）
          if (debug_tx_) {
            tools::logger()->info(
              "[TX][NewCAN][Heartbeat] id=0x{:03X} dlc={} data=[{:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}] | MCU offline, sending heartbeat with start=1",
              frame.can_id, frame.can_dlc,
              frame.data[0], frame.data[1], frame.data[2], frame.data[3],
              frame.data[4], frame.data[5], frame.data[6]);
          } else {
            // 低频日志（避免刷屏）
            static auto last_log = std::chrono::steady_clock::time_point::min();
            auto now = std::chrono::steady_clock::now();
            if (tools::delta_time(now, last_log) > 2.0) {
              tools::logger()->info(
                "[NewCAN] Waiting for MCU online, sending heartbeat: start=1, all data=0");
              last_log = now;
            }
          }
        } catch (const std::exception &e) {
          tools::logger()->warn("[NewCAN] write heartbeat failed: {}", e.what());
        }
        return;
      }

      // MCU已在线：正常发送数据
      // 数据打包：
      // byte 0: AimbotState (u8)
      // byte 1: AimbotTarget (u8)
      // byte 2-3: Yaw (int16, 需要×10000)
      // byte 4-5: Pitch (int16, 需要×10000)
      // byte 6: NucStartFlag (u8) - 一直为1

      // AimbotState: 使用compute_aimbotstate计算
      frame.data[0] = compute_aimbotstate(command.control, command.shoot);

      // AimbotTarget: 暂时固定为1（检测到目标）或0（未检测到）
      frame.data[1] = command.control ? 1 : 0;

      // Yaw和Pitch: 角度×10000转换为int16，大端字节序
      double yaw_rel = static_cast<double>(command.yaw) + tx_yaw_bias_rad_;
      double pitch_rel = static_cast<double>(command.pitch) + tx_pitch_bias_rad_;
      f16tools::f16 yaw_int=f16tools::f64_to_f16(yaw_rel);
      f16tools::f16 pitch_int=f16tools::f64_to_f16(pitch_rel);

      frame.data[2] = (yaw_int >> 8) & 0xFF;    // Yaw高8位
      frame.data[3] = yaw_int & 0xFF;            // Yaw低8位
      frame.data[4] = (pitch_int >> 8) & 0xFF;   // Pitch高8位
      frame.data[5] = pitch_int & 0xFF;          // Pitch低8位

      try {
        can_->write(&frame);

        // 🆕 TX调试信息：每帧都打印（当debug_tx_=true时）
        if (debug_tx_) {
          tools::logger()->info(
            "[TX][NewCAN][Cmd] id=0x{:03X} dlc={} data=[{:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}] | state=0x{:02X} target={} yaw={:.4f}rad({}) pitch={:.4f}rad({}) start={} mcu_online={}",
            frame.can_id, frame.can_dlc,
            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
            frame.data[4], frame.data[5], frame.data[6],
            frame.data[0], frame.data[1],
            yaw_rel, yaw_int, pitch_rel, pitch_int, frame.data[6], mcu_online_.load());
        } else {
          // 低频日志（1Hz）
          static auto last_log = std::chrono::steady_clock::time_point::min();
          auto now = std::chrono::steady_clock::now();
          if (tools::delta_time(now, last_log) > 1.0) {
            tools::logger()->debug(
              "[NewCAN] CMD sent: state=0x{:02X}, target={}, yaw={:.4f}rad({}) pitch={:.4f}rad({}) start={}",
              frame.data[0], frame.data[1],
              yaw_rel, yaw_int, pitch_rel, pitch_int, frame.data[6]);
            last_log = now;
          }
        }
      } catch (const std::exception &e) {
        tools::logger()->warn("[NewCAN] write failed: {}", e.what());
      }
      return;
    }

    // ===== 旧CAN协议发送 =====
    can_frame frame;
    frame.can_id = send_canid_;
    frame.can_dlc = 8;
    frame.data[0] = (command.control) ? 1 : 0;
    frame.data[1] = (command.shoot) ? 1 : 0;
    // 直接发送相对角(上层命令 + bias)，按 1e4 量化（与 SCM 路径一致）
    double yaw_rel = static_cast<double>(command.yaw) + tx_yaw_bias_rad_;
    double pitch_rel = static_cast<double>(command.pitch) + tx_pitch_bias_rad_;
    int16_t can_yaw = static_cast<int16_t>(yaw_rel * 1e4);
    int16_t can_pitch = static_cast<int16_t>(pitch_rel * 1e4);
    frame.data[2] = (can_yaw >> 8) & 0xFF;
    frame.data[3] = (can_yaw) & 0xFF;
    frame.data[4] = (can_pitch >> 8) & 0xFF;
    frame.data[5] = (can_pitch) & 0xFF;
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
  // 🆕 停止心跳线程
  if (heartbeat_thread_.joinable()) {
    heartbeat_quit_ = true;
    heartbeat_thread_.join();
    tools::logger()->info("[Cboard] Heartbeat thread stopped");
  }

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

  // ===== 新CAN协议处理 =====
  if (use_new_can_protocol_) {
    // 处理四元数帧 (0x150)
    if (frame.can_id == new_can_quat_id_) {
      if (frame.can_dlc < 8) {
        tools::logger()->warn("[NewCAN] Quaternion frame length invalid: {}", frame.can_dlc);
        return;
      }
      int64_t packed=0;
      for(int i=0;i<8;i++){
        packed=(packed<<8)|frame.data[i];
      }
      uint16_t uw = (packed>>49)&0x7FFF;
      uint16_t ux = (packed>>34)&0x7FFF;
      uint16_t uy = (packed>>19)&0x7FFF;
      uint16_t uz = (packed>>4)&0x7FFF;
      uint16_t imu_count_low = packed&0x0F;
    //将u15转换为i15
    auto restore_i16=[](uint16_t bits15)->int16_t{
      if(bits15&0x4000){
          return bits15|0x8000;
      }else{
          return bits15&0x7FFF;
      }
  };
  int16_t iw16=restore_i16(uw);
  int16_t ix16=restore_i16(ux);
  int16_t iy16=restore_i16(uy);
  int16_t iz16=restore_i16(uz);

      double qw = float(iw16)/32767.0f;
      double qx = float(ix16)/32767.0f;
      double qy = float(iy16)/32767.0f;
      double qz = float(iz16)/32767.0f;
      


      // f16tools::f16 q_raw[4];
      // for (int i = 0; i < 4; i++) {
      //   q_raw[i] = (int16_t)(frame.data[i*2] << 8 | frame.data[i*2+1]);
      // }
      // f16tools::f16 q_f16[4];
      // double qw = f16tools::f16_to_f32(q_raw[0]);
      // double qx = f16tools::f16_to_f32(q_raw[1]);
      // double qy = f16tools::f16_to_f32(q_raw[2]);
      // double qz = f16tools::f16_to_f32(q_raw[3]);



      // 归一化四元数
      Eigen::Quaterniond q(qw, qx, qy, qz);
      if (q.norm() > 1e-6) {
        q.normalize();
      }

      // 🆕 环形数组方案：缓存四元数到pending_q_，等待0x160帧绑定
      pending_q_ = q;
      pending_q_rx_timestamp_ = timestamp;
      quaternion_ready_.store(true, std::memory_order_release);  // 原子操作，标记四元数就绪

      // 🆕 RX调试信息：显示接收到的四元数
      if (debug_rx_) {
        tools::logger()->info(
          "[RX][NewCAN][Quat] id=0x{:03X} q(w,x,y,z)=({:.4f},{:.4f},{:.4f},{:.4f}) cached, waiting for 0x160",
          frame.can_id, q.w(), q.x(), q.y(), q.z());
      }

      // 1Hz心跳日志
      static auto last_log = std::chrono::steady_clock::time_point::min();
      if (tools::delta_time(timestamp, last_log) > 1.0) {
        tools::logger()->info(
          "[NewCAN] Quaternion received: w={:.3f}, x={:.3f}, y={:.3f}, z={:.3f}",
          q.w(), q.x(), q.y(), q.z());
        last_log = timestamp;
      }
      return;
    }

    // 处理状态帧 (0x160)
    if (frame.can_id == new_can_status_id_) {
      if (frame.can_dlc < 8) {  // 🆕 需要8字节：robot_id + mode + imu_count + mcu_timestamp
        tools::logger()->warn("[NewCAN] Status frame length invalid: {}", frame.can_dlc);
        return;
      }

      // 解析状态信息
      uint8_t robot_id = frame.data[0];
      uint8_t mode_byte = frame.data[1];
      uint16_t imu_count = (uint16_t)(frame.data[2] << 8 | frame.data[3]);

      // 🆕 解析MCU时间戳（byte 4-7，大端字节序，单位：毫秒）
      uint32_t mcu_timestamp = (uint32_t)(frame.data[4] << 24 | frame.data[5] << 16 |
                                          frame.data[6] << 8  | frame.data[7]);

      // 更新内部状态（保留向后兼容性）
      robot_id_ = robot_id;
      mode = static_cast<Mode>(mode_byte);
      imu_count_ = imu_count;  // 保留全局变量，用于旧代码兼容

      // 🆕 MCU在线检测：imu_count != 0 说明MCU在发送数据
      if (!mcu_online_ && imu_count != 0) {
        mcu_online_ = true;
        tools::logger()->info(
          "[NewCAN] ✅ MCU online! imu_count={}, switching to normal data transmission",
          imu_count);
        tools::logger()->info("[Cboard] Heartbeat thread continues until main loop sends first command");
      }

      // 检测IMU计数器跳变（丢帧检测）
      if (last_imu_count_ != 0 && imu_count != last_imu_count_ + 1) {
        uint16_t dropped = imu_count > last_imu_count_ ?
                          (imu_count - last_imu_count_ - 1) :
                          (65536 - last_imu_count_ + imu_count - 1);
        if (dropped > 0 && dropped < 1000) {  // 防止初始化时误报
          tools::logger()->warn("[NewCAN] IMU frame dropped: {} frames lost", dropped);
        }
      }
      last_imu_count_ = imu_count;

      // 🆕 环形数组方案：计算索引并写入环形数组
      size_t index = imu_count % IMU_RING_BUFFER_SIZE;

      // 🔑 首次建立 MCU 时间基准映射
      if (!time_base_initialized_.load(std::memory_order_acquire)) {
        mcu_time_base_ = mcu_timestamp;
        host_time_base_ = timestamp;  // 使用当前 rx_timestamp 作为上位机时间基准
        time_base_initialized_.store(true, std::memory_order_release);
        tools::logger()->info(
          "[CBoard] 🔑 Time base initialized: mcu_base={}ms, host_base=system_clock",
          mcu_time_base_);
      }

      // 🔑 将 MCU 时间戳转换为上位机 steady_clock::time_point
      // 计算相对于基准的时间差，处理 uint32_t 溢出（约49.7天）
      int64_t mcu_diff_ms = static_cast<int64_t>(mcu_timestamp) - static_cast<int64_t>(mcu_time_base_);
      auto mcu_synced_timestamp = host_time_base_ + std::chrono::milliseconds(mcu_diff_ms);

      // 先写入0x160帧的数据（imu_count和mcu_timestamp）
      imu_ring_buffer_[index].imu_count = imu_count;
      imu_ring_buffer_[index].mcu_timestamp = mcu_timestamp;
      imu_ring_buffer_[index].mcu_synced_timestamp = mcu_synced_timestamp;  // 🔑 转换后的 MCU 时间戳
      imu_ring_buffer_[index].rx_timestamp = timestamp;
      imu_ring_buffer_[index].valid.store(false, std::memory_order_release);  // 初始标记为无效

      // 检查是否有待绑定的四元数
      if (quaternion_ready_.load(std::memory_order_acquire)) {
        // 绑定四元数
        imu_ring_buffer_[index].q = pending_q_;
        imu_ring_buffer_[index].valid.store(true, std::memory_order_release);  // 标记为有效
        quaternion_ready_.store(false, std::memory_order_release);  // 清除pending标志

        // 🆕 同时推入queue（保持向后兼容）
        uint8_t cycle_count = imu_count == 0 ? 1 : ((imu_count - 1) % 10) + 1;
        IMUData imu_data = {
          pending_q_,
          pending_q_rx_timestamp_,  // 使用0x150的接收时间作为主时间戳
          mcu_timestamp,
          imu_count,
          cycle_count
        };
        queue_.push(imu_data);

        // 🆕 调试信息：绑定成功
        if (debug_frame_match_) {
          tools::logger()->info(
            "[RingBuffer] ✅ Bound! index={} imu_count={} mcu_ts={}ms valid=true",
            index, imu_count, mcu_timestamp);
        }
      } else {
        // 四元数尚未到达，等待后续绑定
        if (debug_frame_match_) {
          tools::logger()->warn(
            "[RingBuffer] ⚠️ Quaternion not ready! index={} imu_count={} valid=false (waiting for 0x150)",
            index, imu_count);
        }
      }

      // 🆕 RX调试信息：显示接收到的状态
      if (debug_rx_) {
        tools::logger()->info(
          "[RX][NewCAN][Status] id=0x{:03X} robot_id={} mode={} imu_count={} mcu_ts={}ms index={} bound={} mcu_online={}",
          frame.can_id, static_cast<int>(robot_id), MODES[mode], imu_count, mcu_timestamp,
          index, quaternion_ready_.load(), mcu_online_.load());
      }

      // 1Hz心跳日志
      static auto last_log = std::chrono::steady_clock::time_point::min();
      if (tools::delta_time(timestamp, last_log) > 1.0) {
        tools::logger()->info(
          "[NewCAN] Status received: robot_id={}, mode={}, imu_count={}, mcu_ts={}ms, mcu_online={}",
          static_cast<int>(robot_id), MODES[mode], imu_count, mcu_timestamp, mcu_online_.load());
        last_log = timestamp;
      }
      return;
    }
  }

  // ===== 旧CAN协议处理 =====
  if (frame.can_id == quaternion_canid_) {
    auto x = (int16_t)(frame.data[0] << 8 | frame.data[1]) / 1e4;
    auto y = (int16_t)(frame.data[2] << 8 | frame.data[3]) / 1e4;
    auto z = (int16_t)(frame.data[4] << 8 | frame.data[5]) / 1e4;
    auto w = (int16_t)(frame.data[6] << 8 | frame.data[7]) / 1e4;

    if (std::abs(x * x + y * y + z * z + w * w - 1) > 1e-2) {
      tools::logger()->warn("Invalid q: {} {} {} {}", w, x, y, z);
      return;
    }

    queue_.push({{w, x, y, z}, timestamp, 0, 0, 0});  // 旧协议无imu_count和mcu_timestamp，都设为0
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
    if (yaml["cboard_serial_scm_default_target"]) {
      serial_scm_default_target_ = static_cast<uint8_t>(yaml["cboard_serial_scm_default_target"].as<uint32_t>());
    }
    if (yaml["cboard_serial_force_fire_when_control"]) {
      serial_force_fire_when_control_ = yaml["cboard_serial_force_fire_when_control"].as<bool>();
    }
    if (yaml["cboard_serial_force_control"]) {
      serial_force_control_ = yaml["cboard_serial_force_control"].as<bool>();
    }
    if (yaml["cboard_serial_aimbotstate_mode"]) {
      auto ms = yaml["cboard_serial_aimbotstate_mode"].as<std::string>();
      serial_aimbotstate_enum_ = (ms == "enum"); // enum 或 bitfield
    }
    // 云台绝对角零位偏置（优先读取度，回退到弧度）
    if (yaml["gimbal_yaw_offset_deg"]) {
      gimbal_yaw_offset_rad_ = yaml["gimbal_yaw_offset_deg"].as<double>() * M_PI / 180.0;
    } else if (yaml["gimbal_yaw_offset_rad"]) {
      gimbal_yaw_offset_rad_ = yaml["gimbal_yaw_offset_rad"].as<double>();
    }
    if (yaml["gimbal_pitch_offset_deg"]) {
      gimbal_pitch_offset_rad_ = yaml["gimbal_pitch_offset_deg"].as<double>() * M_PI / 180.0;
    } else if (yaml["gimbal_pitch_offset_rad"]) {
      gimbal_pitch_offset_rad_ = yaml["gimbal_pitch_offset_rad"].as<double>();
    }
    // 发送角度细调偏置（优先读取度，回退到弧度）
    if (yaml["tx_yaw_bias_deg"]) {
      tx_yaw_bias_rad_ = yaml["tx_yaw_bias_deg"].as<double>() * M_PI / 180.0;
    } else if (yaml["tx_yaw_bias_rad"]) {
      tx_yaw_bias_rad_ = yaml["tx_yaw_bias_rad"].as<double>();
    }
    if (yaml["tx_pitch_bias_deg"]) {
      tx_pitch_bias_rad_ = yaml["tx_pitch_bias_deg"].as<double>() * M_PI / 180.0;
    } else if (yaml["tx_pitch_bias_rad"]) {
      tx_pitch_bias_rad_ = yaml["tx_pitch_bias_rad"].as<double>();
    }
    // 欧拉角提取与符号、归一化设置
    if (yaml["gimbal_pitch_from_x"]) {
      gimbal_pitch_from_x_ = yaml["gimbal_pitch_from_x"].as<bool>();
    }
    if (yaml["yaw_sign"]) {
      yaw_sign_ = yaml["yaw_sign"].as<int>();
      if (yaw_sign_ == 0) yaw_sign_ = 1;
    }
    if (yaml["pitch_sign"]) {
      pitch_sign_ = yaml["pitch_sign"].as<int>();
      if (pitch_sign_ == 0) pitch_sign_ = 1;
    }
    if (yaml["normalize_abs_angles"]) {
      normalize_abs_angles_ = yaml["normalize_abs_angles"].as<bool>();
    }

    // 读取调试开关配置
    if (yaml["debug_rx"]) {
      debug_rx_ = yaml["debug_rx"].as<bool>();
    }
    if (yaml["debug_tx"]) {
      debug_tx_ = yaml["debug_tx"].as<bool>();
    }
    if (yaml["debug_frame_match"]) {
      debug_frame_match_ = yaml["debug_frame_match"].as<bool>();
    }

  tools::logger()->info(
    "[Cboard] Serial config: CRC {} (skip_crc={}), SOF=0x{:02X}, EOF=0x{:02X}, id_quat=0x{:02X}, id_status=0x{:02X}, protocol={} rx_id=0x{:02X} tx_id=0x{:02X} angles_in_deg={} default_target=0x{:02X} force_control={} force_fire_when_control={} aimbotstate_mode={} log_rx={} log_tx={} gimbal_offset(deg)=(yaw={:.2f},pitch={:.2f}) pitch_from_x={} yaw_sign={} pitch_sign={} normalize_abs={} debug(rx/tx/match)={}/{}/{}",
    serial_skip_crc_ ? "disabled" : "enabled", serial_skip_crc_, serial_sof_, serial_eof_, serial_id_quat_,
    serial_id_status_, serial_protocol_scm_ ? "SCM" : "RAW", serial_scm_rx_id_, serial_scm_tx_id_, serial_scm_angles_in_deg_, serial_scm_default_target_, serial_force_control_, serial_force_fire_when_control_, (serial_aimbotstate_enum_ ? "enum" : "bitfield"), serial_log_rx_, serial_log_tx_,
    gimbal_yaw_offset_rad_ * 180.0 / M_PI, gimbal_pitch_offset_rad_ * 180.0 / M_PI,
    gimbal_pitch_from_x_, yaw_sign_, pitch_sign_, normalize_abs_angles_, debug_rx_, debug_tx_, debug_frame_match_);
    tools::logger()->info("[Cboard] TX bias (deg): yaw={:.3f}, pitch={:.3f}", tx_yaw_bias_rad_ * 180.0 / M_PI, tx_pitch_bias_rad_ * 180.0 / M_PI);
    return "serial";
  }

  // CAN 模式：读取 CAN 相关配置
  quaternion_canid_ = tools::read<int>(yaml, "quaternion_canid");
  bullet_speed_canid_ = tools::read<int>(yaml, "bullet_speed_canid");
  send_canid_ = tools::read<int>(yaml, "send_canid");

  // 读取新CAN协议配置
  if (yaml["use_new_can_protocol"]) {
    use_new_can_protocol_ = yaml["use_new_can_protocol"].as<bool>();
    if (use_new_can_protocol_) {
      tools::logger()->info("[CBoard] Using NEW CAN protocol");

      // 读取新协议的CAN ID配置（提供默认值）
      if (yaml["new_can_quat_id"]) {
        new_can_quat_id_ = yaml["new_can_quat_id"].as<int>();
      }
      if (yaml["new_can_status_id"]) {
        new_can_status_id_ = yaml["new_can_status_id"].as<int>();
      }
      if (yaml["new_can_cmd_id"]) {
        new_can_cmd_id_ = yaml["new_can_cmd_id"].as<int>();
      }

      tools::logger()->info(
        "[CBoard] New CAN IDs: quat=0x{:03X}, status=0x{:03X}, cmd=0x{:03X}",
        new_can_quat_id_, new_can_status_id_, new_can_cmd_id_);
    } else {
      tools::logger()->info("[CBoard] Using OLD CAN protocol");
    }
  }

  if (!yaml["can_interface"]) {
    throw std::runtime_error("Missing 'can_interface' in YAML configuration.");
  }
  // 发送角度细调偏置（优先读取度，回退到弧度）——CAN 模式同样支持
  if (yaml["tx_yaw_bias_deg"]) {
    tx_yaw_bias_rad_ = yaml["tx_yaw_bias_deg"].as<double>() * M_PI / 180.0;
  } else if (yaml["tx_yaw_bias_rad"]) {
    tx_yaw_bias_rad_ = yaml["tx_yaw_bias_rad"].as<double>();
  }
  if (yaml["tx_pitch_bias_deg"]) {
    tx_pitch_bias_rad_ = yaml["tx_pitch_bias_deg"].as<double>() * M_PI / 180.0;
  } else if (yaml["tx_pitch_bias_rad"]) {
    tx_pitch_bias_rad_ = yaml["tx_pitch_bias_rad"].as<double>();
  }

  // 读取调试开关配置（CAN模式）
  if (yaml["debug_rx"]) {
    debug_rx_ = yaml["debug_rx"].as<bool>();
  }
  if (yaml["debug_tx"]) {
    debug_tx_ = yaml["debug_tx"].as<bool>();
  }
  if (yaml["debug_frame_match"]) {
    debug_frame_match_ = yaml["debug_frame_match"].as<bool>();
  }

  tools::logger()->info("[Cboard] TX bias (deg): yaw={:.3f}, pitch={:.3f}", tx_yaw_bias_rad_ * 180.0 / M_PI, tx_pitch_bias_rad_ * 180.0 / M_PI);
  tools::logger()->info("[Cboard] Debug switches: RX={}, TX={}, FrameMatch={}", debug_rx_, debug_tx_, debug_frame_match_);
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
          queue_.push({q, timestamp, 0});  // 串口SCM协议无imu_count，设为0
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

uint8_t CBoard::compute_aimbotstate(bool control, bool fire) {
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

  // Enum 实现（兼容老固件）：0=不控，1=控不火，2=控且火
  if (control) return (fire || serial_force_fire_when_control_) ? 2 : 1;
  return 0;
}

void CBoard::send_scm(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
                      float pitch, float pitch_vel, float pitch_acc)
{
  // AimbotFrame_SCM_t（packed）
  // 按 MCU 协议：EOF 紧随 SystemTimer，后续 PitchRelativeAngle/YawRelativeAngle 为 C 板内部使用，不在帧内发送
  struct __attribute__((packed)) AimbotFrame_SCM_TX_t {
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
    uint8_t _EOF; // EOF 放在 SystemTimer 之后（总 25 字节）
  } frame{};

  frame.SOF = serial_sof_;
  frame.ID = serial_scm_tx_id_;
  // AimbotState：支持两种编码
  frame.Aimbotstate = compute_aimbotstate(control, fire);
  // 目标类型位：当识别到目标（以 control 为判据）置 1，否则使用配置默认值
  frame.AimbotTarget = control ? static_cast<uint8_t>(1u)
                               : serial_scm_default_target_;

  // 计算当前IMU欧拉角（ZYX顺序）：yaw(Z), pitch(Y) 或按配置从 X 轴取 pitch
  double curr_yaw = 0.0, curr_pitch = 0.0;
  {
    Eigen::Quaterniond q = data_behind_.q; // 快照
    if (q.norm() > 1e-9) {
      q.normalize();
      Eigen::Matrix3d R = q.toRotationMatrix();
      Eigen::Vector3d eul_zyx = R.eulerAngles(2, 1, 0);
      curr_yaw = eul_zyx[0];                    // Z 轴
      curr_pitch = gimbal_pitch_from_x_ ? R.eulerAngles(2, 0, 1)[1] // 从 X 轴序列取第二分量
                                        : eul_zyx[1];               // 从 Y 轴
      curr_yaw *= static_cast<double>(yaw_sign_);
      curr_pitch *= static_cast<double>(pitch_sign_);
    }
  }

  // 相对角（来自上层 yaw/pitch）与“绝对角”（= (当前IMU角 - 云台零位偏置) + 相对角）
  double abs_yaw = (curr_yaw - gimbal_yaw_offset_rad_) + static_cast<double>(yaw);
  double abs_pitch = (curr_pitch - gimbal_pitch_offset_rad_) + static_cast<double>(pitch);

  // 归一化绝对角到 (-pi, pi]
  auto norm_pi = [](double a) {
    while (a <= -M_PI) a += 2 * M_PI;
    while (a > M_PI) a -= 2 * M_PI;
    return a;
  };

  if (normalize_abs_angles_) {
    abs_yaw = norm_pi(abs_yaw);
    abs_pitch = norm_pi(abs_pitch);
  }

  // 单位转换：按配置选择用度或弧度
  float rel_yaw_out = serial_scm_angles_in_deg_ ? rad2deg(yaw) : yaw;
  float rel_pitch_out = serial_scm_angles_in_deg_ ? rad2deg(pitch) : pitch;
  float abs_yaw_out = serial_scm_angles_in_deg_ ? rad2deg(static_cast<float>(abs_yaw)) : static_cast<float>(abs_yaw);
  float abs_pitch_out = serial_scm_angles_in_deg_ ? rad2deg(static_cast<float>(abs_pitch)) : static_cast<float>(abs_pitch);
  float out_yaw_vel = serial_scm_angles_in_deg_ ? rad2deg(yaw_vel) : yaw_vel;
  float out_pitch_vel = serial_scm_angles_in_deg_ ? rad2deg(pitch_vel) : pitch_vel;
  float system_timer = std::chrono::duration<float>(
                        std::chrono::steady_clock::now() - start_tp_).count();

  frame.Pitch = rel_pitch_out;
  frame.Yaw = rel_yaw_out;
  frame.TargetPitchSpeed = out_pitch_vel;
  frame.TargetYawSpeed = out_yaw_vel;
  frame.SystemTimer = system_timer;
  frame._EOF = serial_eof_;

  try {
    auto nbytes = serial_.write(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame));
    if (serial_debug_hex_ && serial_log_tx_) {
      tools::logger()->info(
        "[Cboard][SCM][TX] wrote={}B id=0x{:02X} state=0b{:08b}(dec={}) target=0b{:08b} curr(yaw,pitch)_rad=({:.4f},{:.4f}) zero_offs_deg=({:.2f},{:.2f}) rel(yaw,pitch)=({:.3f},{:.3f}) abs(yaw,pitch)=({:.3f},{:.3f}) EOF=0x{:02X}",
        static_cast<unsigned>(nbytes), static_cast<unsigned>(frame.ID), static_cast<unsigned>(frame.Aimbotstate), static_cast<unsigned>(frame.Aimbotstate), static_cast<unsigned>(frame.AimbotTarget),
        curr_yaw, curr_pitch, gimbal_yaw_offset_rad_ * 180.0 / M_PI, gimbal_pitch_offset_rad_ * 180.0 / M_PI,
        rel_yaw_out, rel_pitch_out, abs_yaw_out, abs_pitch_out, static_cast<unsigned>(frame._EOF));
    }
    if (nbytes != sizeof(frame)) {
      tools::logger()->warn("[Cboard][SCM] partial write: {} < {}", nbytes, sizeof(frame));
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
    queue_.push({q, timestamp, 0});  // 串口普通协议无imu_count，设为0
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

void CBoard::send_startup_frame() {
  // 新CAN协议：发送心跳帧，通知电控上位机已启动
  // 帧格式：start=1, 其他数据全部为0
  can_frame frame;
  frame.can_id = new_can_cmd_id_;
  frame.can_dlc = 7;

  // 初始化所有字段为0，仅start为1
  frame.data[0] = 0;  // AimbotState = 0 (无控制)
  frame.data[1] = 0;  // AimbotTarget = 0 (无目标)
  frame.data[2] = 0;  // Yaw高字节 = 0
  frame.data[3] = 0;  // Yaw低字节 = 0
  frame.data[4] = 0;  // Pitch高字节 = 0
  frame.data[5] = 0;  // Pitch低字节 = 0
  frame.data[6] = 1;  // NucStartFlag = 1 ⭐ (一直保持1)

  try {
    can_->write(&frame);
    tools::logger()->info(
      "[NewCAN] Startup heartbeat sent! start=1, all data=0, waiting for MCU response...");
  } catch (const std::exception &e) {
    tools::logger()->error("[NewCAN] Failed to send startup frame: {}", e.what());
  }
}

// 🆕 获取最近一个完整的IMU周期（10帧）
std::vector<Eigen::Quaterniond> CBoard::get_last_imu_cycle() {
  std::vector<Eigen::Quaterniond> result;
  result.reserve(10);

  // 从队列中peek最近的数据（不pop），找到完整的1-10周期
  // 注意：这里需要访问队列的内部数据，可能需要修改ThreadSafeQueue
  // 简化实现：假设队列中至少有10个元素，取最近的10个

  std::vector<IMUData> recent_data;

  // 临时方案：从队列中pop出数据到临时容器，然后再push回去
  // 更好的方案是在ThreadSafeQueue中添加peek_back_n方法

  // 这里简化实现：只返回空vector，提示用户使用get_last_imu_cycle_middle
  tools::logger()->warn("[CBoard] get_last_imu_cycle not fully implemented, use get_last_imu_cycle_middle instead");

  return result;
}

// 🆕 获取最近一个完整IMU周期的中间帧
Eigen::Quaterniond CBoard::get_last_imu_cycle_middle() {
  // 简化实现：直接使用当前的data_behind_，它应该是最新的IMU数据
  // 更精确的实现需要找到count=5或6的帧

  return data_behind_.q;
}

// 🆕 启动相机触发信号（在程序完全初始化后调用）
void CBoard::start_camera_trigger() {
  if (!use_new_can_protocol_ || use_serial_) {
    tools::logger()->warn("[Cboard] start_camera_trigger() only works with new CAN protocol");
    return;
  }

  // 发送启动帧
  tools::logger()->info("[Cboard] 🚀 All modules initialized! Sending startup frame to MCU...");
  send_startup_frame();

  // 启动心跳线程
  heartbeat_quit_ = false;
  heartbeat_thread_ = std::thread(&CBoard::heartbeat_loop, this);
  tools::logger()->info("[Cboard] ✅ Camera trigger enabled! Heartbeat started (interval={}ms)", heartbeat_interval_ms_);
}

// 🆕 心跳线程：在MCU上线前持续发送start=1心跳帧
void CBoard::heartbeat_loop() {
  tools::logger()->info("[Cboard][Heartbeat] Thread started, sending start=1 at 500Hz");

  // 注意：只检查heartbeat_quit_，不检查mcu_online_
  // 让心跳线程持续运行，直到主循环第一次调用send()时主动停止它
  while (!heartbeat_quit_) {
    // 构造心跳帧：start=1, 其他数据全0
    can_frame frame;
    frame.can_id = new_can_cmd_id_;
    frame.can_dlc = 7;
    frame.data[0] = 0;  // AimbotState = 0
    frame.data[1] = 0;  // AimbotTarget = 0
    frame.data[2] = 0;  // Yaw高字节 = 0
    frame.data[3] = 0;  // Yaw低字节 = 0
    frame.data[4] = 0;  // Pitch高字节 = 0
    frame.data[5] = 0;  // Pitch低字节 = 0
    frame.data[6] = 1;  // NucStartFlag = 1 ⭐

    try {
      can_->write(&frame);

      // 🆕 调试输出（低频，避免刷屏）
      if (debug_tx_) {
        static auto last_log = std::chrono::steady_clock::time_point::min();
        auto now = std::chrono::steady_clock::now();
        if (tools::delta_time(now, last_log) > 1.0) {
          tools::logger()->info(
            "[TX][Heartbeat] id=0x{:03X} dlc={} data=[{:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}] | 500Hz heartbeat, mcu_online={}",
            frame.can_id, frame.can_dlc,
            frame.data[0], frame.data[1], frame.data[2], frame.data[3],
            frame.data[4], frame.data[5], frame.data[6], mcu_online_.load());
          last_log = now;
        }
      }
    } catch (const std::exception &e) {
      tools::logger()->warn("[Heartbeat] write failed: {}", e.what());
    }

    // 500Hz = 2ms间隔
    std::this_thread::sleep_for(std::chrono::milliseconds(heartbeat_interval_ms_));
  }

  tools::logger()->info("[Cboard][Heartbeat] Thread exiting (quit=true)");
}

// 🆕 环形数组直接查询接口
CBoard::IMUQueryResult CBoard::get_imu_from_ring_buffer(uint16_t target_imu_count) const {
  // 计算环形数组索引
  size_t index = target_imu_count % IMU_RING_BUFFER_SIZE;

  // 读取数据（原子操作保证线程安全）
  bool is_valid = imu_ring_buffer_[index].valid.load(std::memory_order_acquire);

  if (is_valid) {
    // 数据有效，返回IMU数据
    return {
      imu_ring_buffer_[index].q,
      imu_ring_buffer_[index].mcu_timestamp,
      imu_ring_buffer_[index].mcu_synced_timestamp,  // 🔑 转换后的 MCU 时间戳
      imu_ring_buffer_[index].rx_timestamp,
      true
    };
  } else {
    // 数据无效，返回默认值
    return {
      Eigen::Quaterniond(1, 0, 0, 0),  // 默认四元数（单位四元数）
      0,
      std::chrono::steady_clock::time_point(),  // mcu_synced_timestamp
      std::chrono::steady_clock::time_point(),  // rx_timestamp
      false
    };
  }
}

#ifdef AMENT_CMAKE_FOUND
void CBoard::set_ros2_tf_publisher(
  std::shared_ptr<void> node_ptr,
  const Eigen::Matrix3d & R_gimbal2imubody)
{
  auto node = std::static_pointer_cast<rclcpp::Node>(node_ptr);
  ros_node_ = node_ptr;
  R_gimbal2imubody_ = R_gimbal2imubody;

  // 创建TF broadcaster
  auto broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);
  tf_broadcaster_ = std::static_pointer_cast<void>(broadcaster);

  // 建立时间基准
  ros_time_base_ = std::chrono::steady_clock::now();
  ros_time_start_ = std::make_shared<rclcpp::Time>(node->now());

  tools::logger()->info("[CBoard] ROS2 TF publisher initialized - will publish TF on every IMU update");
}

// 🆕 时间转换函数：将 steady_clock 时间戳转换为 ROS 时间
std::shared_ptr<void> CBoard::convert_to_ros_time(std::chrono::steady_clock::time_point timestamp) {
  // 计算时间差（纳秒）
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
    timestamp - ros_time_base_);

  // 获取 ROS 时间基准并计算目标时间
  auto ros_time_start = *std::static_pointer_cast<rclcpp::Time>(ros_time_start_);
  auto ros_time = std::make_shared<rclcpp::Time>(ros_time_start.nanoseconds() + duration.count());

  return std::static_pointer_cast<void>(ros_time);
}
#endif

} // namespace io