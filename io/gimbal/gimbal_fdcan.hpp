#ifndef IO__GIMBAL_FDCAN_HPP
#define IO__GIMBAL_FDCAN_HPP

#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include "gimbal.hpp"
#include "io/command.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

class GimbalFdcan {
public:
	explicit GimbalFdcan(const std::string &config_path);
	~GimbalFdcan();

	GimbalMode mode() const;
	GimbalState state() const;
	Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

	void send_command_scm(io::Command command);

private:
	class CanRxDevice;

	void handle_can_frame(const rm::hal::CanFrame *msg);

	std::unique_ptr<rm::hal::Can> can_;
	std::unique_ptr<CanRxDevice> can_device_;
	std::string can_interface_;
	uint16_t can_rx_id_ = 0x02;
	uint16_t can_tx_id_ = 0x01;

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

} // namespace io

#endif // IO__GIMBAL_FDCAN_HPP
