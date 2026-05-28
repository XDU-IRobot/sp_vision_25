#ifndef IO__GIMBAL_FDCAN_HPP
#define IO__GIMBAL_FDCAN_HPP

#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include "io/command.hpp"
#include "tools/thread_safe_queue.hpp"
#include "gimbal.hpp"
namespace io {

class GimbalFdcan {
public:
	explicit GimbalFdcan(const std::string &config_path);
	~GimbalFdcan();

	GimbalMode mode() const;
	GimbalState state() const;
	Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

	void send(io::Command command);
    void send(bool control, bool fire, float yaw, float yaw_vel,
                float yaw_acc, float pitch, float pitch_vel, float pitch_acc);
private:
	class CanRxDevice;

	void handle_can_frame(const rm::hal::CanFrame *msg);

	std::unique_ptr<rm::hal::Can> can_;
	std::unique_ptr<CanRxDevice> can_device_;
	std::string can_interface_;
	uint16_t can_rx_id_ = 0x402;
	uint32_t can_tx_id_ = 0x401;
	bool can_extended_ = true;

	uint8_t scm_sof_ = 0x55;
	uint8_t scm_eof_ = 0xFF;
	uint8_t scm_rx_id_ = 0x01;
	uint8_t scm_tx_id_ = 0x02;
	bool scm_angles_in_deg_ = true;
	std::chrono::steady_clock::time_point start_tp_;

	mutable std::mutex mutex_;
	GimbalMode mode_ = GimbalMode::IDLE;
	GimbalState state_{};
	tools::ThreadSafeQueue<
			std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
			queue_{1000};
};
typedef struct __attribute__((packed)) {
  // 包头
  uint8_t SOF;
  uint8_t ID;
  // 自瞄状态
  uint8_t AimbotState;
  uint8_t AimbotTarget;
  // 自瞄数据
  float Pitch;
  float Yaw;
  float PitchSpeed;        // pitch速度
  float YawSpeed;          // yaw速度
  float PitchAcceSpeed;    // pitch加速度
  float YawAcceSpeed;      // yaw加速度
  float PitchAngSpeed;     // pitch角速度
  float YawAngSpeed;       // yaw角速度

  float TargetPitchSpeed;  // 目标pitch速度
  float TargetYawSpeed;    // 目标yaw速度
  // 时间戳
  uint32_t SystemTimer;
  // 包尾
} AimbotFrame_CAN;

typedef struct __attribute__((packed)) {
  float q0;
  float q1;
  float q2;
  float q3;
  float bullet_speed;
  uint8_t robot_id;
  uint8_t mode;
  // 包尾
  uint8_t EOF;
  uint8_t padding;  // 对齐下位机发送的固定长度帧
} Gimaballmurname_CAN;

} // namespace io

#endif // IO__GIMBAL_FDCAN_HPP
