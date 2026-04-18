#include <fmt/core.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "io/camera.hpp"
#include "tasks/auto_aim/yolos/yolo26_trt.hpp"

namespace
{
const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{config-path c  | configs/standard4.yaml | YAML配置文件路径（需启用YOLO26 TRT） }"
  "{source s       | camera                 | 数据源: camera / image / synthetic }"
  "{image-path i   |                        | 测试图片路径（source=image时必填） }"
  "{width w        | 1280                   | 合成图宽度（source=synthetic时生效） }"
  "{height         | 1024                   | 合成图高度（source=synthetic时生效） }"
  "{camera-warmup  | 30                     | 相机预热抓帧数（source=camera时生效） }"
  "{warmup         | 50                     | 预热轮数 }"
  "{iters          | 300                    | 统计轮数 }"
  "{debug d        | false                  | 是否开启YOLO26_TRT内部调试日志 }";

double percentile_ms(std::vector<double> values, double q)
{
  if (values.empty()) {
    return 0.0;
  }
  q = std::clamp(q, 0.0, 1.0);
  std::sort(values.begin(), values.end());
  const double pos = q * static_cast<double>(values.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(pos));
  const size_t high = static_cast<size_t>(std::ceil(pos));
  if (low == high) {
    return values[low];
  }
  const double alpha = pos - static_cast<double>(low);
  return values[low] * (1.0 - alpha) + values[high] * alpha;
}
}  // namespace

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  const std::string config_path = cli.get<std::string>("config-path");
  const std::string source = cli.get<std::string>("source");
  const std::string image_path = cli.get<std::string>("image-path");
  const int width = cli.get<int>("width");
  const int height = cli.get<int>("height");
  const int camera_warmup = cli.get<int>("camera-warmup");
  const int warmup = cli.get<int>("warmup");
  const int iters = cli.get<int>("iters");
  const bool debug = cli.get<bool>("debug");

  if (!cli.check()) {
    cli.printErrors();
    return 1;
  }

  if (warmup < 0 || iters <= 0 || camera_warmup < 0) {
    fmt::print("[ERROR] invalid args: warmup>=0, camera-warmup>=0, iters>0 are required\n");
    return 1;
  }

  if (source != "camera" && source != "image" && source != "synthetic") {
    fmt::print("[ERROR] invalid source: {} (expected camera/image/synthetic)\n", source);
    return 1;
  }

  cv::Mat fixed_img;
  if (source == "image") {
    if (image_path.empty()) {
      fmt::print("[ERROR] source=image requires --image-path\n");
      return 1;
    }
    fixed_img = cv::imread(image_path, cv::IMREAD_COLOR);
    if (fixed_img.empty()) {
      fmt::print("[ERROR] failed to load image: {}\n", image_path);
      return 1;
    }
  } else if (source == "synthetic") {
    if (width <= 0 || height <= 0) {
      fmt::print("[ERROR] invalid synthetic image size: {}x{}\n", width, height);
      return 1;
    }
    fixed_img = cv::Mat(height, width, CV_8UC3);
    cv::randu(fixed_img, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
  }

  try {
    auto_aim::YOLO26_TRT yolo(config_path, debug);
    std::unique_ptr<io::Camera> camera;
    std::chrono::steady_clock::time_point frame_ts;

    if (source == "camera") {
      camera = std::make_unique<io::Camera>(config_path);
      cv::Mat cam_img;
      for (int i = 0; i < camera_warmup; ++i) {
        camera->read(cam_img, frame_ts);
      }
      if (cam_img.empty()) {
        fmt::print("[ERROR] camera returned empty frame after warmup\n");
        return 1;
      }
      fixed_img = cam_img;
    }

    fmt::print("[Benchmark] config={}\n", config_path);
    fmt::print("[Benchmark] source={}, warmup={}, iters={}\n", source, warmup, iters);
    if (source == "camera") {
      fmt::print("[Benchmark] camera_warmup_frames={}\n", camera_warmup);
    } else if (source == "image") {
      fmt::print("[Benchmark] input={}x{}\n", fixed_img.cols, fixed_img.rows);
      fmt::print("[Benchmark] source=image ({})\n", image_path);
    } else {
      fmt::print("[Benchmark] input={}x{}\n", fixed_img.cols, fixed_img.rows);
      fmt::print("[Benchmark] source=synthetic random image\n");
    }

    for (int i = 0; i < warmup; ++i) {
      cv::Mat frame;
      if (source == "camera") {
        camera->read(frame, frame_ts);
        if (frame.empty()) {
          fmt::print("[ERROR] empty frame during warmup at iter {}\n", i);
          return 1;
        }
      } else {
        frame = fixed_img;
      }
      (void)yolo.detect(frame, -1 - i);
    }

    std::vector<double> latency_ms;
    latency_ms.reserve(static_cast<size_t>(iters));
    size_t total_armors = 0;

    for (int i = 0; i < iters; ++i) {
      cv::Mat frame;
      if (source == "camera") {
        camera->read(frame, frame_ts);
        if (frame.empty()) {
          fmt::print("[WARN] empty frame at iter {}, skip\n", i);
          continue;
        }
      } else {
        frame = fixed_img;
      }

      const auto t0 = std::chrono::high_resolution_clock::now();
      auto result = yolo.detect(frame, i);
      const auto t1 = std::chrono::high_resolution_clock::now();
      const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
      latency_ms.push_back(ms);
      total_armors += result.size();
    }

    if (latency_ms.empty()) {
      fmt::print("[ERROR] no valid frames collected for benchmark\n");
      return 1;
    }

    const double sum_ms = std::accumulate(latency_ms.begin(), latency_ms.end(), 0.0);
    const double mean_ms = sum_ms / static_cast<double>(latency_ms.size());
    const double min_ms = *std::min_element(latency_ms.begin(), latency_ms.end());
    const double max_ms = *std::max_element(latency_ms.begin(), latency_ms.end());
    double sq_sum = 0.0;
    for (double v : latency_ms) {
      const double d = v - mean_ms;
      sq_sum += d * d;
    }
    const double stddev_ms = std::sqrt(sq_sum / static_cast<double>(latency_ms.size()));

    const double p50 = percentile_ms(latency_ms, 0.50);
    const double p90 = percentile_ms(latency_ms, 0.90);
    const double p99 = percentile_ms(latency_ms, 0.99);
    const double fps = (mean_ms > 0.0) ? (1000.0 / mean_ms) : 0.0;
    const double avg_armors = static_cast<double>(total_armors) / static_cast<double>(iters);

    fmt::print("\n=== YOLO26_TRT Benchmark Result ===\n");
    fmt::print("latency(ms): mean={:.3f}, std={:.3f}, min={:.3f}, max={:.3f}\n",
      mean_ms, stddev_ms, min_ms, max_ms);
    fmt::print("percentile(ms): p50={:.3f}, p90={:.3f}, p99={:.3f}\n", p50, p90, p99);
    fmt::print("throughput: {:.2f} FPS\n", fps);
    fmt::print("avg detections per frame: {:.2f}\n", avg_armors);
    fmt::print("===================================\n");
  } catch (const std::exception & e) {
    fmt::print("[ERROR] benchmark failed: {}\n", e.what());
    return 1;
  }

  return 0;
}
