#include "opencv2/opencv.hpp"
#include "io/camera.hpp"
using namespace cv;
const std::string keys =
  "{help h usage ? |                  | 输出命令行参数说明}"
  "{@config-path   | configs/uav.yaml | yaml配置文件路径 }";
int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  io::Camera camera(config_path);
  cv::VideoWriter video_writer;
  std::string output_path = "./output.avi";
  cv::Size frame_size(1920, 1200);
  int fps = 168;
  cv::Mat img;
  std::chrono::steady_clock::time_point t;
  video_writer.open(
      output_path,
      cv::VideoWriter::fourcc('M','J','P','G'),
      fps,
      frame_size
  );
  // 录制视频
  while (true) {
    camera.read(img,t);
    if (img.empty()) {
      break; // 结束录制
      // or continue; // 跳过空帧继续录制
      // 根据需求选择适当的处理方式
      // Here you can add logging or error handling
      std::cerr << "Warning: Empty frame captured at time " << t.time_since_epoch().count() << "ns" << std::endl;
      continue;
    }
    video_writer.write(img);
    // 查看是否录制成功
    cv::imshow("Video", img);
  }
}