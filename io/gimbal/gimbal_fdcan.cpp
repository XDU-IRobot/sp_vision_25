#include "gimbal_fdcan.hpp"

#include <algorithm>
#include <cstring>
#include <linux/can.h>

#include "io/gimbal/gimbal.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io {

class GimbalFdcan::CanRxDevice : public rm::device::CanDevice {
public:
	CanRxDevice(rm::hal::CanInterface &can, GimbalFdcan &owner, uint16_t rx_id)
			: rm::device::CanDevice(can, rx_id), owner_(owner) {}

	void RxCallback(const rm::hal::CanFrame *msg) override {
		owner_.handle_can_frame(msg);
	}

private:
	GimbalFdcan &owner_;
};

static inline float rad2deg(float r) { return r * 57.29577951308232f; }

GimbalFdcan::GimbalFdcan(const std::string &config_path) {
	auto yaml = tools::load(config_path);

	try {
		if (yaml["gimbal_can_interface"]) {
			can_interface_ = yaml["gimbal_can_interface"].as<std::string>();
		} else if (yaml["can_interface"]) {
			can_interface_ = yaml["can_interface"].as<std::string>();
		}

		if (yaml["gimbal_can_rx_id"]) {
			can_rx_id_ = static_cast<uint16_t>(yaml["gimbal_can_rx_id"].as<uint32_t>());
		}
		if (yaml["gimbal_can_tx_id"]) {
			can_tx_id_ = static_cast<uint16_t>(yaml["gimbal_can_tx_id"].as<uint32_t>());
		}

		if (yaml["scm_sof"]) {
			scm_sof_ = static_cast<uint8_t>(yaml["scm_sof"].as<uint32_t>());
		}
		if (yaml["scm_eof"]) {
			scm_eof_ = static_cast<uint8_t>(yaml["scm_eof"].as<uint32_t>());
		}
		if (yaml["scm_rx_id"]) {
			scm_rx_id_ = static_cast<uint8_t>(yaml["scm_rx_id"].as<uint32_t>());
		} else {
			scm_rx_id_ = static_cast<uint8_t>(can_rx_id_ & 0xFFu);
		}
		if (yaml["scm_tx_id"]) {
			scm_tx_id_ = static_cast<uint8_t>(yaml["scm_tx_id"].as<uint32_t>());
		} else {
			scm_tx_id_ = static_cast<uint8_t>(can_tx_id_ & 0xFFu);
		}
		if (yaml["scm_angles_in_deg"]) {
			scm_angles_in_deg_ = yaml["scm_angles_in_deg"].as<bool>();
		}
	} catch (...) {
	}

	if (can_interface_.empty()) {
		tools::logger()->error("[GimbalFdcan] CAN interface not set.");
		exit(1);
	}

	try {
		can_ = std::make_unique<rm::hal::Can>(can_interface_.c_str());
		can_->SetFilter(can_rx_id_, CAN_SFF_MASK);
		can_device_ = std::make_unique<CanRxDevice>(*can_, *this, can_rx_id_);
		can_->Begin();
		start_tp_ = std::chrono::steady_clock::now();
		tools::logger()->info("[GimbalFdcan] Opened CAN {} (rx=0x{:03X}, tx=0x{:03X})",
													can_interface_, can_rx_id_, can_tx_id_);
	} catch (const std::exception &e) {
		tools::logger()->error("[GimbalFdcan] Failed to open CAN: {}", e.what());
		exit(1);
	}

	tools::logger()->info("[GimbalFdcan] Waiting for q...");
	queue_.pop();
	tools::logger()->info("[GimbalFdcan] First q received.");
}

GimbalFdcan::~GimbalFdcan() {
	if (can_) {
		can_->Stop();
	}
}

GimbalMode GimbalFdcan::mode() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return mode_;
}

GimbalState GimbalFdcan::state() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return state_;
}

Eigen::Quaterniond GimbalFdcan::q(std::chrono::steady_clock::time_point t) {
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

void GimbalFdcan::send(io::Command command) {
	if (!can_) {
		return;
	}

	uint8_t aimbot_state = 0; // 0:不控 2:控不火 4:控且火
	if (command.control)
		aimbot_state = command.shoot ? 4 : 2;
	uint8_t aimbot_target = command.shoot; // 0: 不开火 1: 开火

	float out_yaw = scm_angles_in_deg_ ? rad2deg(command.yaw) : command.yaw;
	float out_pitch =
			scm_angles_in_deg_ ? rad2deg(command.pitch) : command.pitch;

	AimbotFrame_SCM_t frame{};
	frame.SOF = scm_sof_;
	frame.ID = scm_tx_id_;
	frame.AimbotState = aimbot_state;
	frame.AimbotTarget = aimbot_target;
	frame.Pitch = out_pitch;
	frame.Yaw = out_yaw;
	frame.TargetPitchSpeed = 0.0f;
	frame.TargetYawSpeed = 0.0f;
	frame.PitchAcceSpeed = 0.0f;
	frame.YawAcceSpeed = 0.0f;
	frame.PitchAngSpeed = 0.0f;
	frame.YawAngSpeed = 0.0f;
	frame.SystemTimer = static_cast<uint32_t>(
			std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now() - start_tp_)
					.count());
	frame.EOF = scm_eof_;

	try {
		can_->Write(can_tx_id_, reinterpret_cast<const uint8_t *>(&frame),
								sizeof(frame));
	} catch (const std::exception &e) {
		tools::logger()->warn("[GimbalFdcan] CAN write failed: {}", e.what());
	}
}

void GimbalFdcan::handle_can_frame(const rm::hal::CanFrame *msg) {
	if (msg == nullptr) {
		return;
	}

	if (msg->dlc < sizeof(Gimaballmurname_CAN)) {
		return;
	}

	Gimaballmurname_CAN rx{};
	std::memcpy(&rx, msg->data.data(), sizeof(rx));

	Eigen::Quaterniond q(rx.q0, rx.q1, rx.q2, rx.q3);
	if (q.norm() > 1e-6)
		q.normalize();
	auto t = std::chrono::steady_clock::now();
	queue_.push({q, t});

	std::lock_guard<std::mutex> lock(mutex_);
	// state_.yaw = 0;
	// state_.yaw_vel = 0;
	// state_.pitch = 0;
	// state_.pitch_vel = 0;
	state_.bullet_speed = rx.bullet_speed;
	state_.bullet_count = 0;

	mode_ = GimbalMode::AUTO_AIM;
};

} // namespace io
