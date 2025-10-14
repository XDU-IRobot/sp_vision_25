#include "dm_imu.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io
{
DM_IMU::DM_IMU() : queue_(5000)
{
  init_serial();
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[DM_IMU] initialized");
}

DM_IMU::DM_IMU(const std::string & config_path) : queue_(5000)
{
  // Try read from YAML: dm_imu section
  try {
    auto yaml = tools::load(config_path);
    if (yaml["dm_imu"]) {
      auto y = yaml["dm_imu"];
      if (y["port"]) port_ = y["port"].as<std::string>();
      if (y["baudrate"]) baudrate_ = y["baudrate"].as<uint32_t>();
      if (y["timeout_ms"]) timeout_ms_ = y["timeout_ms"].as<uint32_t>();
      if (y["sof"]) sof_ = static_cast<uint8_t>(y["sof"].as<uint32_t>());
      if (y["flag"]) flag_ = static_cast<uint8_t>(y["flag"].as<uint32_t>());
      if (y["slave_id"]) slave_id_ = static_cast<uint8_t>(y["slave_id"].as<uint32_t>());
      if (y["reg_acc"]) reg_acc_ = static_cast<uint8_t>(y["reg_acc"].as<uint32_t>());
      if (y["skip_crc"]) skip_crc_ = y["skip_crc"].as<bool>();
      if (y["debug_hex"]) debug_hex_ = y["debug_hex"].as<bool>();
    }
  } catch (...) {
    tools::logger()->warn("[DM_IMU] YAML parse failed, using defaults.");
  }

  init_serial();
  rec_thread_ = std::thread(&DM_IMU::get_imu_data_thread, this);
  queue_.pop(data_ahead_);
  queue_.pop(data_behind_);
  tools::logger()->info("[DM_IMU] initialized with YAML");
  if (skip_crc_) {
    tools::logger()->warn("[DM_IMU] CRC check is DISABLED (skip_crc=true)");
  }
}


DM_IMU::~DM_IMU()
{
  stop_thread_ = true;
  if (rec_thread_.joinable()) {
    rec_thread_.join();
  }
  if (serial_.isOpen()) {
    serial_.close();
  }
}

void DM_IMU::init_serial()
{
  try {
    serial_.setPort(port_);
    serial_.setBaudrate(baudrate_);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setParity(serial::parity_none);  //default is parity_none
    serial_.setStopbits(serial::stopbits_one);
    serial_.setBytesize(serial::eightbits);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(timeout_ms_);
    serial_.setTimeout(time_out);
    serial_.open();
    usleep(1000000);  //1s

    tools::logger()->info("[DM_IMU] serial port opened {} @ {} baud (timeout={}ms)", port_, baudrate_, timeout_ms_);
  }

  catch (serial::IOException & e) {
    tools::logger()->warn("[DM_IMU] failed to open serial port ");
    exit(0);
  }
}

void DM_IMU::get_imu_data_thread()
{
  while (!stop_thread_) {
    if (!serial_.isOpen()) {
      tools::logger()->warn("In get_imu_data_thread,imu serial port unopen");
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      continue;
    }

    // 寻找起始字节 SOF，逐字节对齐
    uint8_t b = 0;
    while (!stop_thread_) {
      size_t n = serial_.read(&b, 1);
      if (n == 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
      if (b == sof_) break;
    }
    if (stop_thread_) break;

    receive_data.FrameHeader1 = b;
    // 读取 flag/slave/reg 共 3 字节
    if (serial_.read(&receive_data.flag1, 3) != 3) {
      continue;
    }

    // 记录是否与期望的 flag/slave/reg 匹配
    bool header_match = (receive_data.FrameHeader1 == sof_ && receive_data.flag1 == flag_ &&
                         receive_data.slave_id1 == slave_id_ && receive_data.reg_acc == reg_acc_);

    if (!header_match) {
      static uint32_t miss = 0;
      if (((++miss) % 200) == 1) {
        tools::logger()->info(
          "[DM_IMU] header mismatch: got (hdr=0x{:02X}, flag=0x{:02X}, slave=0x{:02X}, reg=0x{:02X}) expect (hdr=0x{:02X}, flag=0x{:02X}, slave=0x{:02X}, reg=0x{:02X})",
          receive_data.FrameHeader1, receive_data.flag1, receive_data.slave_id1, receive_data.reg_acc,
          sof_, flag_, slave_id_, reg_acc_);
      }
    }

    // 无论是否匹配，都尝试读取完整帧剩余部分，由 CRC 决定是否有效
    size_t remain = 57 - 4;
    size_t got = 0;
    while (got < remain && !stop_thread_) {
      size_t n = serial_.read(reinterpret_cast<uint8_t *>(&receive_data.accx_u32) + got,
                              remain - got);
      if (n == 0) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
      got += n;
    }
    if (got != remain) continue;

    auto crc1_calc = tools::get_crc16((uint8_t *)(&receive_data.FrameHeader1), 16);
    if (skip_crc_ || crc1_calc == receive_data.crc1) {
      float fx, fy, fz;
      std::memcpy(&fx, &receive_data.accx_u32, sizeof(float));
      std::memcpy(&fy, &receive_data.accy_u32, sizeof(float));
      std::memcpy(&fz, &receive_data.accz_u32, sizeof(float));
      data.accx = fx;
      data.accy = fy;
      data.accz = fz;
    } else {
      static uint32_t c1_miss = 0;
      if ((++c1_miss % 200) == 1) {
        auto recv_crc1 = static_cast<unsigned int>(receive_data.crc1);
        tools::logger()->warn("[DM_IMU] CRC1 mismatch: calc=0x{:04X}, recv=0x{:04X}",
                              static_cast<unsigned int>(crc1_calc), recv_crc1);
        if (debug_hex_) {
          auto p = reinterpret_cast<uint8_t*>(&receive_data.FrameHeader1);
          char buf[128]; int len = 0;
          for (int i=0;i<16 && len<120;i++) len += snprintf(buf+len, sizeof(buf)-len, "%02X ", p[i]);
          tools::logger()->warn("[DM_IMU] CRC1 window: {}", std::string(buf, len));
        }
      }
    }
    auto crc2_calc = tools::get_crc16((uint8_t *)(&receive_data.FrameHeader2), 16);
    if (skip_crc_ || crc2_calc == receive_data.crc2) {
      float gx, gy, gz;
      std::memcpy(&gx, &receive_data.gyrox_u32, sizeof(float));
      std::memcpy(&gy, &receive_data.gyroy_u32, sizeof(float));
      std::memcpy(&gz, &receive_data.gyroz_u32, sizeof(float));
      data.gyrox = gx;
      data.gyroy = gy;
      data.gyroz = gz;
    } else {
      static uint32_t c2_miss = 0;
      if ((++c2_miss % 200) == 1) {
        auto recv_crc2 = static_cast<unsigned int>(receive_data.crc2);
        tools::logger()->warn("[DM_IMU] CRC2 mismatch: calc=0x{:04X}, recv=0x{:04X}",
                              static_cast<unsigned int>(crc2_calc), recv_crc2);
        if (debug_hex_) {
          auto p = reinterpret_cast<uint8_t*>(&receive_data.FrameHeader2);
          char buf[128]; int len = 0;
          for (int i=0;i<16 && len<120;i++) len += snprintf(buf+len, sizeof(buf)-len, "%02X ", p[i]);
          tools::logger()->warn("[DM_IMU] CRC2 window: {}", std::string(buf, len));
        }
      }
    }
    auto crc3_calc = tools::get_crc16((uint8_t *)(&receive_data.FrameHeader3), 16);
    if (skip_crc_ || crc3_calc == receive_data.crc3) {
      float fr, fp, fyaw;
      std::memcpy(&fr, &receive_data.roll_u32, sizeof(float));
      std::memcpy(&fp, &receive_data.pitch_u32, sizeof(float));
      std::memcpy(&fyaw, &receive_data.yaw_u32, sizeof(float));
      data.roll = fr;
      data.pitch = fp;
      data.yaw = fyaw;
      // tools::logger()->debug(
      //   "yaw: {:.2f}, pitch: {:.2f}, roll: {:.2f}", static_cast<double>(data.yaw),
      //   static_cast<double>(data.pitch), static_cast<double>(data.roll));
    } else {
      static uint32_t c3_miss = 0;
      if ((++c3_miss % 200) == 1) {
        auto recv_crc3 = static_cast<unsigned int>(receive_data.crc3);
        tools::logger()->warn("[DM_IMU] CRC3 mismatch: calc=0x{:04X}, recv=0x{:04X}",
                              static_cast<unsigned int>(crc3_calc), recv_crc3);
        if (debug_hex_) {
          auto p = reinterpret_cast<uint8_t*>(&receive_data.FrameHeader3);
          char buf[128]; int len = 0;
          for (int i=0;i<16 && len<120;i++) len += snprintf(buf+len, sizeof(buf)-len, "%02X ", p[i]);
          tools::logger()->warn("[DM_IMU] CRC3 window: {}", std::string(buf, len));
        }
      }
    }
    auto timestamp = std::chrono::steady_clock::now();
    Eigen::Quaterniond q = Eigen::AngleAxisd(data.yaw * M_PI / 180, Eigen::Vector3d::UnitZ()) *
                           Eigen::AngleAxisd(data.pitch * M_PI / 180, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(data.roll * M_PI / 180, Eigen::Vector3d::UnitX());
    q.normalize();
    queue_.push({q, timestamp});
  }
}

Eigen::Quaterniond DM_IMU::imu_at(std::chrono::steady_clock::time_point timestamp)
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

}  // namespace io
