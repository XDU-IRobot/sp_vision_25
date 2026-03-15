#ifndef IO__DAHENG_HPP
#define IO__DAHENG_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <thread>

#include "driver/GxIAPI.h"
#include "driver/DxImageProc.h"
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

  // 🆕 获取最后读取的相机帧ID
  uint64_t get_last_frame_id() const { return last_frame_id_; }

private:
  struct CameraData
  {
    cv::Mat img;
    std::chrono::steady_clock::time_point timestamp;  // 转换后的系统时间戳
    uint64_t frame_id;        // 🆕 相机SDK的帧ID
    uint64_t hw_timestamp;    // 🆕 相机硬件时间戳 (nTimestamp)
  };

  double exposure_, gain_;
  int width_, height_, fps_;
  bool auto_exposure_, auto_white_balance_, auto_exp_change_;
  double rgain_, bgain_, ggain_;
  int64_t time_offset_;
  int max_exp_, min_exp_;
  bool debug_;
  std::string vid_pid_;
  std::string pixel_format_config_;  // 配置文件指定的 Bayer 格式

  // 硬触发相关参数
  bool trigger_enable_;          // 是否启用硬触发
  int trigger_source_;           // 触发源：0=Line0, 1=Line1, 2=Line2
  int trigger_activation_;       // 触发沿：0=上升沿, 1=下降沿

  GX_DEV_HANDLE device_handle_;
  int64_t payload_size_;
  bool quit_, ok_;
  std::thread capture_thread_;
  std::thread daemon_thread_;
  // 🔧 启用PopWhenFull=true：队列满时自动丢弃旧帧，保留最新帧
  tools::ThreadSafeQueue<CameraData, true> queue_;
  uint64_t last_frame_id_;  // 🆕 最后读取的帧ID
  int bayer_format_index_;  // 🆕 保存相机实际使用的 Bayer 格式索引 (0=RG, 1=GR, 2=GB, 3=BG)

  void open();
  void try_open();
  void close();
  void set_camera_params();
  static void GX_STDC frame_callback(GX_FRAME_CALLBACK_PARAM * frame_data);
};

}  // namespace io

#endif  // IO__DAHENG_HPP
