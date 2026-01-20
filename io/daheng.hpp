#ifndef IO__DAHENG_HPP
#define IO__DAHENG_HPP

#include <atomic>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "driver/GxIAPI.h"
#include "io/camera.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io
{
class Daheng : public CameraBase
{
public:
  Daheng(const std::string & config_path);
  ~Daheng() override;
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  // 获取最后读取的相机帧ID
  uint64_t get_last_frame_id() const { return last_frame_id_; }

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;
    uint64_t frame_id;   // 来自 Daheng SDK 的 nFrameID
  };

  double exposure_, gain_;
  int width_, height_, fps_;
  bool auto_exposure_, auto_white_balance_, auto_exp_change_;
  double rgain_, bgain_, ggain_;
  int64_t time_offset_;
  int max_exp_, min_exp_;
  bool debug_;
  std::string vid_pid_;
  int bayer_code_;  // OpenCV Bayer转换代码（如 cv::COLOR_BayerBG2BGR）

  // 硬触发相关参数
  bool trigger_enable_;          // 是否启用硬触发
  int trigger_source_;           // 触发源：0=Line0, 1=Line1, 2=Line2
  int trigger_activation_;       // 触发沿：0=上升沿, 1=下降沿

  GX_DEV_HANDLE device_handle_;
  int64_t payload_size_;
  std::atomic<bool> quit_;
  std::atomic<bool> ok_;
  std::thread capture_thread_;
  std::thread daemon_thread_;
  tools::ThreadSafeQueue<CameraData, true> queue_;  // PopWhenFull=true，新帧覆盖旧帧
  uint64_t last_frame_id_;  // 最近一次 read() 对应的帧 ID

  void open();
  void try_open();
  void close();
  void set_camera_params();
};

}  // namespace io

#endif  // IO__DAHENG_HPP
