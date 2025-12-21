#include "camera.hpp"

#include <stdexcept>

#include "daheng.hpp"
#include "hikrobot/hikrobot.hpp"
#include "mindvision/mindvision.hpp"
#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io
{
Camera::Camera(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto camera_name = tools::read<std::string>(yaml, "camera_name");
  auto exposure_ms = tools::read<double>(yaml, "exposure_ms");
  
  tools::logger()->info("Camera constructor: config_path={}, camera_name={}", config_path, camera_name);

  if (camera_name == "mindvision") {
    auto gamma = tools::read<double>(yaml, "gamma");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    tools::logger()->info("Creating MindVision camera");
    camera_ = std::make_unique<MindVision>(exposure_ms, gamma, vid_pid);
  }

  else if (camera_name == "hikrobot") {
    auto gain = tools::read<double>(yaml, "gain");
    auto vid_pid = tools::read<std::string>(yaml, "vid_pid");
    tools::logger()->info("Creating HikRobot camera");
    camera_ = std::make_unique<HikRobot>(exposure_ms, gain, vid_pid);
  }

  else if (camera_name == "daheng") {
    tools::logger()->info("Creating Daheng camera with config: {}", config_path);
    camera_ = std::make_unique<Daheng>(config_path);
  }

  else {
    throw std::runtime_error("Unknow camera_name: " + camera_name + "!");
  }
  
  tools::logger()->info("Camera constructor completed");
}

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  camera_->read(img, timestamp);
}

uint64_t Camera::get_last_frame_id() const
{
  // 尝试转换为Daheng相机，如果成功则返回frame_id
  auto* daheng = dynamic_cast<Daheng*>(camera_.get());
  if (daheng) {
    return daheng->get_last_frame_id();
  }
  // 其他相机类型不支持frame_id，返回0
  return 0;
}

}  // namespace io