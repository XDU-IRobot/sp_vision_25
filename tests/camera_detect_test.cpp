#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml    | yaml配置文件的路径}"
  "{@image-path    |                        | 输入图片的路径}"
  "{tradition t    |  false                 | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto image_path = cli.get<std::string>(1);
  auto use_tradition = cli.get<bool>("tradition");

  if (image_path.empty()) {
    tools::logger()->error("必须指定图片路径，使用 --help 查看帮助");
    return 1;
  }

  // 加载单张图片
  cv::Mat img = cv::imread(image_path);
  if (img.empty()) {
    tools::logger()->error("无法加载图片: {}", image_path);
    return 1;
  }

  auto_aim::Detector detector(config_path, true);
  auto_aim::YOLO yolo(config_path, true);

  std::list<auto_aim::Armor> armors;

  auto last = std::chrono::steady_clock::now();

  if (use_tradition)
    armors = detector.detect(img);
  else
    armors = yolo.detect(img);

  // auto now = std::chrono::steady_clock::now();
  // auto dt = tools::delta_time(now, last);
  // tools::logger()->info("检测耗时: {:.2f} ms", dt * 1000);

  // 绘制检测框
  for (const auto & armor : armors) {
    cv::rectangle(img, armor.box, cv::Scalar(0, 0, 255), 2);
    cv::putText(img, std::to_string(static_cast<int>(armor.name)+1), armor.box.tl(), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 1);
  }

  // 显示结果图像
  cv::imshow("Detection Result", img);
  cv::waitKey(0);

  return 0;
}